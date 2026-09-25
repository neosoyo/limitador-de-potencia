#ifndef __PM100_STREAM_H
#define __PM100_STREAM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * ============================================================================
 *  PM100 binary telemetry stream  (second USB CDC ACM port)
 * ============================================================================
 *
 * All multi-byte integers are little-endian, all structures are packed with
 * no padding. The stream is one-way (device -> host); the port accepts no
 * commands and is independent from the shell/console port.
 *
 *   frame   := header payload
 *   payload := count * <item>
 *
 *   header (8 bytes)
 *     u8   magic      always PM100_STREAM_MAGIC (0xA5)
 *     u8   version    PM100_STREAM_VERSION
 *     u8   type       PM100_STREAM_TYPE_{SAMPLES,META}
 *     u16  seq        frame counter, +1 per frame, wraps at 65536
 *     u8   count      number of items in the payload
 *     u16  len        payload length in bytes
 *
 *   type SAMPLES : payload := count * struct pm100_stream_sample
 *   type META    : payload := 1     * struct pm100_stream_meta
 *   type IDENT   : payload := count * struct pm100_stream_ident
 *
 * Type IDENT only appears while `pm100 ident run` is executing: it carries the
 * raw 1 kHz capture windows (one record per control tick) so each staircase
 * step fit can be checked offline instead of being taken on trust.
 *
 * A host parser should:
 *   1. scan for `magic`, then validate `version`, `type`, `count` and `len`
 *      (len == count * item_size) before accepting a frame;
 *   2. use `seq` to detect *dropped frames* (a gap means whole frames were
 *      lost, e.g. the host was not reading);
 *   3. use the per-sample `t_ms` to detect *dropped samples* (a gap means the
 *      on-device ring overflowed) and to build the time base.
 *
 * `seq` and `len` are deliberately redundant: `len` lets the parser skip an
 * unknown/oversized payload safely, `seq` makes loss visible.
 * ============================================================================
 */

#define PM100_STREAM_MAGIC   0xA5u
#define PM100_STREAM_VERSION 0x01u

#define PM100_STREAM_TYPE_SAMPLES 1u
#define PM100_STREAM_TYPE_META    2u
#define PM100_STREAM_TYPE_IDENT   3u

/* Size in bytes of one item of each payload type. */
#define PM100_STREAM_SAMPLE_BYTES 44u
#define PM100_STREAM_META_BYTES   84u
#define PM100_STREAM_IDENT_BYTES  12u

/*
 * Flags of struct pm100_stream_sample::flags.
 *
 * INPUT_VALID    : pilot throttle pulse is inside the valid window
 * BATTERY_VALID  : bus voltage above BATTERY_MIN_V
 * SAFE           : INPUT_VALID && BATTERY_VALID (control law is running)
 * LEARNING       : b0/ADRC identification mode is active
 * LEARN_POWER_CUT: learning safety cap tripped, throttle forced to safe-stop
 * ADRC_SATURATED : raw ADRC output was clamped to the ESC pulse limits
 *                  (pwm_ctrl_raw is outside [PWM_MIN_US, PWM_MAX_US])
 * SENSOR_ERROR   : the INA226 read failed this cycle (voltage/current are 0)
 */
#define PM100_FLAG_INPUT_VALID     (1u << 0)
#define PM100_FLAG_BATTERY_VALID   (1u << 1)
#define PM100_FLAG_SAFE            (1u << 2)
#define PM100_FLAG_LEARNING        (1u << 3)
#define PM100_FLAG_LEARN_POWER_CUT (1u << 4)
#define PM100_FLAG_ADRC_SATURATED  (1u << 5)
#define PM100_FLAG_SENSOR_ERROR    (1u << 6)

/**
 * @brief One control-loop sample (1 kHz tick), fixed-point, 44 bytes.
 *
 * Everything the ADRC controller needs to be tuned offline is here: the
 * measurement `y_dw`, the reference `tgt_dw`, the control effort `pwm_ctrl`
 * (applied, clamped) / `pwm_ctrl_raw` (unclamped, for wind-up inspection) and
 * the observer states.
 *
 * The law is FIRST-ORDER (plant model dP/dt = b0*u + f), so the second-order
 * LESO only has two states:
 *   z1_mw  = z1, estimated power [mW]
 *   z3_mws = z2, estimated total disturbance f [mW/s]   <- kept in the z3 slot
 *   z2_mws = always 0 (there is no dP/dt state to report)
 * Field names and offsets are unchanged from the second-order variant so host
 * parsers do not need to change.
 *
 * z1_mw, z3_mws and pwm_ctrl_raw are only meaningful on ticks where the
 * control law actually ran, i.e. when PM100_FLAG_SAFE is set and
 * PM100_FLAG_LEARNING is clear; they hold the previous value otherwise.
 */
struct __attribute__((packed)) pm100_stream_sample {
	uint32_t t_ms;         /**< [  4] uptime, ms (wraps after ~49.7 days)      */
	int32_t e_j;           /**< [  4] accumulated energy, J                    */
	int32_t z1_mw;         /**< [  4] LESO z1, estimated power, mW             */
	int32_t z2_mws;        /**< [  4] always 0 (no dP/dt state, see above)     */
	int32_t z3_mws;        /**< [  4] LESO z2, total disturbance f, mW/s       */
	uint16_t v_cv;         /**< [  2] bus voltage, 10 mV  (2550 = 25.50 V)     */
	int16_t i_ca;          /**< [  2] current, 10 mA      (1250 = 12.50 A)     */
	int16_t y_dw;          /**< [  2] measured power (INA226, controller y), 0.1 W */
	uint16_t pmax_w;       /**< [  2] peak power since boot, W                 */
	int16_t tgt_dw;        /**< [  2] power target (reference), 0.1 W          */
	uint16_t pwm_in;       /**< [  2] pilot throttle input, us                 */
	uint16_t pwm_out;      /**< [  2] applied ESC output, us                   */
	uint16_t pwm_ctrl;     /**< [  2] ADRC effort after clamping, us           */
	int16_t pwm_ctrl_raw;  /**< [  2] ADRC effort before clamping, us          */
	uint8_t state;         /**< [  1] enum ctrl_state                          */
	uint8_t flags;         /**< [  1] PM100_FLAG_*                             */
	uint8_t _rsvd[4];      /**< [  4] reserved, keep the item 4-byte aligned   */
};

/**
 * @brief One raw sample of an automatic-identification capture window, 12 bytes.
 *
 * Emitted at the full 1 kHz control rate only while `pm100 ident run` executes
 * (frame type IDENT). One capture window starts at the tick the firmware steps
 * the throttle to a new staircase level, so the record with PM100_IDENT_FLAG_STEP
 * set is t = 0 for that fit.
 */
struct __attribute__((packed)) pm100_stream_ident {
	uint32_t t_ms;         /**< [  4] uptime, ms                               */
	uint16_t cmd_us;       /**< [  2] throttle the firmware commanded, us      */
	int16_t p_dw;          /**< [  2] measured power, 0.1 W                    */
	uint8_t level;         /**< [  1] staircase level index (0..N-1)           */
	uint8_t flags;         /**< [  1] PM100_IDENT_FLAG_*                       */
	uint8_t _rsvd[2];      /**< [  2] reserved                                 */
};

/** Flag of pm100_stream_ident::flags: first sample of a capture window. */
#define PM100_IDENT_FLAG_STEP (1u << 0)

/**
 * @brief Controller/configuration snapshot, sent once per second and right
 *        after any change, so a capture is self-describing. 84 bytes.
 *
 * With the first-order law: l1 = 2*wo, l2 = wo^2, l3 = 0, and kd is stored for
 * configuration compatibility but is NOT used by the control law.
 */
struct __attribute__((packed)) pm100_stream_meta {
	float wo;              /**< [  4] LESO bandwidth, rad/s                    */
	float b0;              /**< [  4] input gain, (W/s) per us                 */
	float kp;              /**< [  4] loop gain, 1/s                           */
	float kd;              /**< [  4] UNUSED by the first-order law            */
	float l1;              /**< [  4] LESO gain 2*wo                           */
	float l2;              /**< [  4] LESO gain wo^2                           */
	float l3;              /**< [  4] always 0 (no third LESO state)           */
	float target_power;    /**< [  4] power target, W                          */
	float shunt_mohm;      /**< [  4] configured shunt resistor, mOhm          */
	float dt;              /**< [  4] control period, s                        */
	uint32_t uptime_ms;    /**< [  4] uptime when this snapshot was taken, ms  */
	uint32_t team_number;  /**< [  4] team / controller number                 */
	char team_name[32];    /**< [ 32] NUL-padded team name                     */
	uint8_t learning_stage;/**< [  1] enum learning_stage                      */
	uint8_t _rsvd[3];      /**< [  3] reserved                                 */
};

/** @brief Diagnostic counters (not part of the wire format). */
struct pm100_stream_stats {
	uint32_t samples_sent;
	uint32_t samples_dropped; /**< on-device ring was full                */
	uint32_t frames_sent;
	uint32_t frames_dropped;  /**< port stalled, frame never started      */
	uint32_t tx_stalls;       /**< fifo_fill could not take the whole frame */
	uint32_t corrupt_records; /**< internal record framing errors          */
};

/**
 * @brief Claim the second CDC ACM port and start the stream thread.
 *
 * Safe to call when the port is missing: the stream stays disabled and an
 * error is logged, the rest of the application is unaffected.
 *
 * @return int 0 on success, negative errno if the port is unavailable.
 */
int stream_init(void);

/**
 * @brief Enable/disable the binary stream.
 *
 * Takes effect on the next control-loop tick. Disabling only stops new
 * samples; frames already queued are still flushed.
 */
void stream_set_enabled(bool enable);

/** @return true while the binary stream is enabled. */
bool stream_is_enabled(void);

/**
 * @brief Set the flush period in milliseconds.
 *
 * The control loop runs at 1 kHz, so this is also the number of samples per
 * frame (1 = one sample per frame, ..., 48 = 48 samples per frame).
 */
void stream_set_period_ms(uint16_t period_ms);

/** @return the current flush period in milliseconds. */
uint16_t stream_get_period_ms(void);

/** @return true if the second CDC ACM port was found and is ready. */
bool stream_is_ready(void);

/**
 * @brief Queue one control-loop sample. Called from the 1 kHz control thread.
 *
 * Single-producer/single-consumer: must only be called from one thread.
 * Cheap and non-blocking; drops the sample and bumps a counter if full.
 */
void stream_push_sample(const struct pm100_stream_sample *smp);

/**
 * @brief Queue a controller/configuration snapshot.
 *
 * Single-producer/single-consumer: must only be called from the control
 * thread. It is emitted as its own frame, so it also delimits sample batches.
 */
void stream_push_meta(const struct pm100_stream_meta *meta);

/**
 * @brief Queue one raw identification-capture sample (frame type IDENT).
 *
 * Called from the 1 kHz control thread while an automatic identification is
 * running, for every tick of a capture window. Same single-producer contract
 * as stream_push_sample().
 */
void stream_push_ident(const struct pm100_stream_ident *idn);

/** @brief Copy the current diagnostic counters. */
void stream_get_stats(struct pm100_stream_stats *out);

#endif /* __PM100_STREAM_H */
