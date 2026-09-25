/*
 * PM100 binary telemetry stream over a dedicated (second) USB CDC ACM port.
 *
 * Data path:
 *
 *   control thread (1 kHz)          stream thread (period_ms)         host
 *   ----------------------          -----------------------           ----
 *   stream_push_sample()  --+
 *   stream_push_meta()      +--> SPSC ring --> frame assembly --> uart_fifo_fill
 *
 * The ring is a plain byte-mode Zephyr ring buffer used strictly
 * single-producer/single-consumer, so no locking is involved and the 1 kHz
 * control loop only pays for two small memcpys.
 *
 * Each record is stored as [u8 type][u8 len][payload...]. Because the producer
 * only ever commits whole records (ring_buf_put updates the tail index only
 * after copying), the consumer never observes a torn record: as soon as two
 * header bytes are readable, the whole record is.
 *
 * Frames are written with uart_fifo_fill() rather than the console/printf path:
 * one call per frame instead of one work-schedule per character, no per-sample
 * float formatting, and no shell/console bytes in the stream.
 */

#include "stream.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(PM100_devel, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Tuning                                                                    */
/* ------------------------------------------------------------------------- */

/** SPSC ring size: ~170 samples of flight time, absorbs host hiccups. */
#define STREAM_RING_BYTES 8192u

/** Hard cap on samples per frame (also the frame buffer size below). */
#define STREAM_MAX_BATCH 48u

/** Default / allowed flush periods. At 1 kHz one period unit == one sample. */
#define STREAM_PERIOD_MS_DEFAULT 4u
#define STREAM_PERIOD_MS_MIN     1u
#define STREAM_PERIOD_MS_MAX     48u

/** Controller snapshot period (in addition to "on every change"). */
#define STREAM_META_PERIOD_MS 1000u

/** How long to wait for TX room mid-frame before retrying. */
#define STREAM_TX_RETRY_MS 1

#define STREAM_THREAD_STACK_SIZE 2048
#define STREAM_THREAD_PRIORITY   K_PRIO_PREEMPT(8)

/* Internal record types (not part of the wire format). */
#define STREAM_REC_SAMPLE 1u
#define STREAM_REC_META   2u
#define STREAM_REC_IDENT  3u
#define STREAM_REC_HDR    2u

#define STREAM_HDR_BYTES 8u

#define STREAM_FRAME_BYTES (STREAM_HDR_BYTES + STREAM_MAX_BATCH * PM100_STREAM_SAMPLE_BYTES)

BUILD_ASSERT(sizeof(struct pm100_stream_sample) == PM100_STREAM_SAMPLE_BYTES,
	     "pm100_stream_sample size changed, update the host parser");
BUILD_ASSERT(sizeof(struct pm100_stream_meta) == PM100_STREAM_META_BYTES,
	     "pm100_stream_meta size changed, update the host parser");
BUILD_ASSERT(sizeof(struct pm100_stream_ident) == PM100_STREAM_IDENT_BYTES,
	     "pm100_stream_ident size changed, update the host parser");
BUILD_ASSERT(STREAM_MAX_BATCH * PM100_STREAM_IDENT_BYTES <= STREAM_FRAME_BYTES - STREAM_HDR_BYTES,
	     "IDENT frame would not fit the frame buffer");
BUILD_ASSERT(STREAM_PERIOD_MS_MIN >= 1u && STREAM_PERIOD_MS_MAX <= STREAM_MAX_BATCH,
	     "flush period must fit in one frame");

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

static const struct device *stream_uart;

/* NB: RING_BUF_DECLARE() already declares its storage static, and it emits a
 * BUILD_ASSERT at the top, so this cannot be prefixed with `static`. */
RING_BUF_DECLARE(stream_rb, STREAM_RING_BYTES);

static volatile bool g_stream_ready;      /* port claimed by stream_init()   */
static volatile bool g_stream_on;         /* user enabled the stream         */
static volatile bool g_stream_reset_req;  /* producer must drop stale data   */
static volatile uint16_t g_period_ms = STREAM_PERIOD_MS_DEFAULT;
static uint16_t g_seq;                    /* frame counter, stream thread    */
static struct pm100_stream_stats g_stats;

/* Frame scratch: header followed by up to STREAM_MAX_BATCH samples. */
static uint8_t frame_buf[STREAM_FRAME_BYTES];

/* ------------------------------------------------------------------------- */
/* Producer side (control thread)                                            */
/* ------------------------------------------------------------------------- */

/*
 * Ring ownership: the control thread owns the write side, the stream thread
 * the read side. ring_buf_reset() touches both indices, so it may only run
 * from the producer, and only while the consumer is held off
 * (g_stream_reset_req == true keeps the consumer out of stream_service()).
 */
static bool stream_producer_begin(void)
{
	if (!g_stream_on) {
		return false;
	}

	if (g_stream_reset_req) {
		ring_buf_reset(&stream_rb);
		g_stream_reset_req = false;
	}

	return true;
}

static bool stream_push_record(uint8_t type, const void *payload, uint8_t len)
{
	uint8_t rec[1 + 1 + PM100_STREAM_META_BYTES];
	size_t need = 1u + 1u + len;

	__ASSERT_NO_MSG(len <= PM100_STREAM_META_BYTES);

	/*
	 * The consumer only ever frees space, so if the record fits now it still
	 * fits when ring_buf_put() runs; this avoids committing a partial record.
	 */
	if (ring_buf_space_get(&stream_rb) < need) {
		return false;
	}

	rec[0] = type;
	rec[1] = len;
	memcpy(&rec[2], payload, len);

	return ring_buf_put(&stream_rb, rec, need) == need;
}

void stream_push_sample(const struct pm100_stream_sample *smp)
{
	if (!smp || !stream_producer_begin()) {
		return;
	}

	if (!stream_push_record(STREAM_REC_SAMPLE, smp, sizeof(*smp))) {
		g_stats.samples_dropped++;
	}
}

void stream_push_meta(const struct pm100_stream_meta *meta)
{
	if (!meta || !stream_producer_begin()) {
		return;
	}

	/* A lost snapshot is not fatal: the next one is at most 1 s away. */
	(void)stream_push_record(STREAM_REC_META, meta, sizeof(*meta));
}

void stream_push_ident(const struct pm100_stream_ident *idn)
{
	if (!idn || !stream_producer_begin()) {
		return;
	}

	/* Identification captures are diagnostic; dropping one is acceptable. */
	(void)stream_push_record(STREAM_REC_IDENT, idn, sizeof(*idn));
}

/* ------------------------------------------------------------------------- */
/* Consumer side (stream thread)                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief Pop one record from the ring.
 *
 * @param type Receives the record type.
 * @param payload Buffer of at least PM100_STREAM_META_BYTES.
 * @return payload length, or 0 if the ring holds no complete record.
 */
static uint32_t stream_pop_record(uint8_t *type, uint8_t *payload)
{
	uint8_t hdr[STREAM_REC_HDR];

	if (ring_buf_size_get(&stream_rb) < STREAM_REC_HDR) {
		return 0u;
	}

	if (ring_buf_get(&stream_rb, hdr, sizeof(hdr)) != sizeof(hdr)) {
		return 0u;
	}

	if (hdr[1] > PM100_STREAM_META_BYTES) {
		/* Should be unreachable: framing is broken, ask the producer (which
		 * owns ring_buf_reset()) to drop the ring, and stop reading it. */
		g_stats.corrupt_records++;
		g_stream_reset_req = true;
		return 0u;
	}

	*type = hdr[0];

	if (hdr[1] != 0u && ring_buf_get(&stream_rb, payload, hdr[1]) != hdr[1]) {
		g_stats.corrupt_records++;
		g_stream_reset_req = true;
		return 0u;
	}

	return hdr[1];
}

static void stream_build_header(uint8_t *hdr, uint8_t type, uint16_t seq, uint8_t count,
				uint16_t len)
{
	hdr[0] = (uint8_t)PM100_STREAM_MAGIC;
	hdr[1] = (uint8_t)PM100_STREAM_VERSION;
	hdr[2] = type;
	hdr[3] = (uint8_t)(seq & 0xFFu);
	hdr[4] = (uint8_t)(seq >> 8);
	hdr[5] = count;
	hdr[6] = (uint8_t)(len & 0xFFu);
	hdr[7] = (uint8_t)(len >> 8);
}

/**
 * @brief Hand a whole frame to the CDC ACM port.
 *
 * Returns false if nothing could be queued at all: the frame is dropped
 * (counted) rather than partially written, which keeps the host in sync.
 * A stall in the middle of a frame is waited out, because a truncated frame
 * would desynchronise the parser.
 */
static bool stream_tx(const uint8_t *buf, size_t len)
{
	size_t off = 0u;

	while (off < len) {
		int written = uart_fifo_fill(stream_uart, &buf[off], (int)(len - off));

		if (written > 0) {
			off += (size_t)written;
			continue;
		}

		if (off == 0u) {
			/* Host not reading (port closed) or ring saturated. */
			g_stats.frames_dropped++;
			return false;
		}

		/* Mid-frame: must complete it to stay frame-aligned. */
		g_stats.tx_stalls++;
		k_msleep(STREAM_TX_RETRY_MS);
	}

	g_stats.frames_sent++;
	return true;
}

static void stream_send_samples(uint16_t count)
{
	uint16_t len = (uint16_t)(count * PM100_STREAM_SAMPLE_BYTES);

	stream_build_header(frame_buf, PM100_STREAM_TYPE_SAMPLES, g_seq++, (uint8_t)count, len);

	if (stream_tx(frame_buf, STREAM_HDR_BYTES + len)) {
		g_stats.samples_sent += count;
	}
}

static void stream_send_meta(const uint8_t *payload)
{
	stream_build_header(frame_buf, PM100_STREAM_TYPE_META, g_seq++, 1u,
			    (uint16_t)PM100_STREAM_META_BYTES);
	memcpy(&frame_buf[STREAM_HDR_BYTES], payload, PM100_STREAM_META_BYTES);
	(void)stream_tx(frame_buf, STREAM_HDR_BYTES + PM100_STREAM_META_BYTES);
}

static void stream_send_idents(uint16_t count)
{
	uint16_t len = (uint16_t)(count * PM100_STREAM_IDENT_BYTES);

	stream_build_header(frame_buf, PM100_STREAM_TYPE_IDENT, g_seq++, (uint8_t)count, len);
	(void)stream_tx(frame_buf, STREAM_HDR_BYTES + len);
}

/** @brief Drain the ring and emit frames until it is empty. */
static void stream_service(void)
{
	uint8_t rec[PM100_STREAM_META_BYTES];
	uint16_t batch = 0u;
	uint16_t ident_batch = 0u;
	uint16_t batch_max = g_period_ms;

	if (batch_max == 0u) {
		batch_max = STREAM_PERIOD_MS_DEFAULT;
	} else if (batch_max > STREAM_MAX_BATCH) {
		batch_max = STREAM_MAX_BATCH;
	}

	for (;;) {
		uint8_t type = 0u;
		uint32_t plen = stream_pop_record(&type, rec);

		if (plen == 0u) {
			break;
		}

		if (type == STREAM_REC_SAMPLE) {
			if (plen != PM100_STREAM_SAMPLE_BYTES) {
				g_stats.corrupt_records++;
				continue;
			}

			/* A frame carries one payload type only. */
			if (ident_batch != 0u) {
				stream_send_idents(ident_batch);
				ident_batch = 0u;
			}

			memcpy(&frame_buf[STREAM_HDR_BYTES +
					 (size_t)batch * PM100_STREAM_SAMPLE_BYTES],
			       rec, plen);

			if (++batch >= batch_max) {
				stream_send_samples(batch);
				batch = 0u;
			}
		} else if (type == STREAM_REC_IDENT) {
			if (plen != PM100_STREAM_IDENT_BYTES) {
				g_stats.corrupt_records++;
				continue;
			}

			if (batch != 0u) {
				stream_send_samples(batch);
				batch = 0u;
			}

			memcpy(&frame_buf[STREAM_HDR_BYTES +
					 (size_t)ident_batch * PM100_STREAM_IDENT_BYTES],
			       rec, plen);

			/* Bounded by the frame buffer, which is sized for samples. */
			if (++ident_batch >= STREAM_MAX_BATCH) {
				stream_send_idents(ident_batch);
				ident_batch = 0u;
			}
		} else if (type == STREAM_REC_META) {
			if (plen != PM100_STREAM_META_BYTES) {
				g_stats.corrupt_records++;
				continue;
			}

			/* Keep the sample batches contiguous around the snapshot. */
			if (batch != 0u) {
				stream_send_samples(batch);
				batch = 0u;
			}
			if (ident_batch != 0u) {
				stream_send_idents(ident_batch);
				ident_batch = 0u;
			}

			stream_send_meta(rec);
		} else {
			g_stats.corrupt_records++;
		}
	}

	if (batch != 0u) {
		stream_send_samples(batch);
	}
	if (ident_batch != 0u) {
		stream_send_idents(ident_batch);
	}
}

static void stream_thread_handler(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (!g_stream_ready) {
		k_msleep(10);
	}

	LOG_INF("Telemetry stream thread started (USB CDC ACM port 2)");

	while (1) {
		if (!g_stream_on || g_stream_reset_req) {
			/* Idle, or waiting for the producer to drop stale data. */
			k_msleep(100);
			continue;
		}

		k_msleep(g_period_ms);
		stream_service();
	}
}

K_THREAD_DEFINE(stream_thread, STREAM_THREAD_STACK_SIZE, stream_thread_handler, NULL, NULL,
		NULL, STREAM_THREAD_PRIORITY, 0, 0);

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

int stream_init(void)
{
	stream_uart = DEVICE_DT_GET(DT_NODELABEL(stream_cdc_acm_uart));

	if (!device_is_ready(stream_uart)) {
		LOG_ERR("Telemetry stream port %s not ready, binary stream disabled",
			DEVICE_DT_NAME(DT_NODELABEL(stream_cdc_acm_uart)));
		return -ENODEV;
	}

	ring_buf_reset(&stream_rb);
	g_stream_ready = true;

	LOG_INF("Binary telemetry stream port ready: %s (period %u ms)",
		DEVICE_DT_NAME(DT_NODELABEL(stream_cdc_acm_uart)), (unsigned int)g_period_ms);

	return 0;
}

void stream_set_enabled(bool enable)
{
	if (enable == g_stream_on) {
		return;
	}

	if (enable) {
		/*
		 * Order matters: request the producer-side reset *before* the
		 * consumer is allowed in, so it never reads a half-cleared ring.
		 * The producer performs the reset on its next push.
		 */
		g_stream_reset_req = true;
		memset(&g_stats, 0, sizeof(g_stats));
		g_stream_on = true;
	} else {
		g_stream_on = false;
		g_stream_reset_req = false;
	}
}

bool stream_is_enabled(void)
{
	return g_stream_on;
}

void stream_set_period_ms(uint16_t period_ms)
{
	if (period_ms < STREAM_PERIOD_MS_MIN) {
		period_ms = STREAM_PERIOD_MS_MIN;
	} else if (period_ms > STREAM_PERIOD_MS_MAX) {
		period_ms = STREAM_PERIOD_MS_MAX;
	}

	g_period_ms = period_ms;
}

uint16_t stream_get_period_ms(void)
{
	return g_period_ms;
}

bool stream_is_ready(void)
{
	return g_stream_ready;
}

void stream_get_stats(struct pm100_stream_stats *out)
{
	if (!out) {
		return;
	}

	*out = g_stats;
}
