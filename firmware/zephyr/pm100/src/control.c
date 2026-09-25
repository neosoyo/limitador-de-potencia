#include "control.h"
#include "persistence.h"
#include "stream.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_DECLARE(PM100_devel, LOG_LEVEL_DBG);

#define CONTROL_STACK_SIZE 2048
#define CONTROL_PRIORITY   K_PRIO_COOP(1) // High-priority cooperative thread for precise 1 kHz execution

/* ------------------------------------------------------------------------- */
/* Timebase                                                                  */
/* ------------------------------------------------------------------------- */

/**
 * @brief Monotonic microsecond timebase.
 *
 * The system timer on nRF runs at 32768 Hz, so a microsecond conversion from
 * uptime ticks resolves the control period (~2.7 ms as built) to better than
 * 0.1 %. k_uptime_get() only has millisecond resolution, which is too coarse
 * both for dt in the LESO and for fitting a 30 ms time constant.
 */
static uint64_t ctrl_now_us(void)
{
	return k_ticks_to_us_near64((uint64_t)k_uptime_ticks());
}

/* Measured control period. Published in the stream snapshot so a capture is
 * self-describing; it is NOT the nominal CONTROL_PERIOD_S. */
static float g_loop_dt_s = CONTROL_PERIOD_S;

/* ------------------------------------------------------------------------- */
/* Global shared state                                                       */
/* ------------------------------------------------------------------------- */

struct system_telemetry g_telemetry = {
    .power_w = 0.0f,
    .current_a = 0.0f,
    .voltage_v = 0.0f,
    .total_consumption_j = 0.0f,
    .time_ms = 0,
    .pwm_input_us = PWM_SAFE_STOP_US,
    .pwm_output_us = PWM_SAFE_STOP_US,
    .pwm_control_us = PWM_SAFE_STOP_US,
    .state = READY
};
K_MUTEX_DEFINE(g_telemetry_mutex);

struct device_config g_config = {
    .dt = 0.001f,               // 1 kHz sampling period
    .wo = 100.0f,               // LESO bandwidth (rad/s); wo*dt = 0.1; overwritten by b0 learning
    .b0 = 0.0f,                 // 0 = enter b0/ADRC learning mode on first boot
    .kp = 60.0f,                // Loop gain [1/s]; overwritten by b0 learning
    .kd = 0.0f,                 // UNUSED by the first-order law (kept in the NVS layout)
    .target_power = 600.0f,     // Power target limit in Watts
    .shunt_resistor_mohm = 1.0f, // Default 1 mOhm shunt
    .team_name = "time",
    .team_number = 0,
    .PIN_code = "123456"
};
K_MUTEX_DEFINE(g_config_mutex);

volatile bool g_stream_active = false;
volatile bool g_learning_mode = false;
volatile enum learning_stage g_learning_stage = LEARNING_STAGE_IDLE;

static volatile bool g_initialized = false;

/* ------------------------------------------------------------------------- */
/* Binary telemetry stream flags                                             */
/* ------------------------------------------------------------------------- */

/* 1 kHz control loop -> one controller snapshot per second, plus one right
 * after every configuration change. */
#define STREAM_META_PERIOD_TICKS 1000u

/* Set by the configuration setters and when learning finishes, consumed by the
 * control loop (the only stream producer) so the host sees the new gains
 * immediately instead of up to a second later. Declared here because
 * learning_finalize() (below) publishes the learned gains too. */
static volatile bool stream_meta_dirty = true;

/* ------------------------------------------------------------------------- */
/* PWM input capture (pilot throttle)                                        */
/* ------------------------------------------------------------------------- */

static const struct gpio_dt_spec throttle_in = GPIO_DT_SPEC_GET(DT_NODELABEL(throtle), gpios);

static nrfx_timer_t timer2_inst = NRFX_TIMER_INSTANCE(2);
static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
static nrf_ppi_channel_t ppi_channel_cap;
static nrf_ppi_channel_t ppi_channel_clr;

static volatile uint32_t last_high_time;
static volatile uint32_t last_period;
static uint32_t last_low_time;
static volatile int input_period;

static void pwm_input_gpiote_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger,
                                     void *p_context)
{
    uint32_t phase_time_ticks = nrfx_timer_capture_get(&timer2_inst, NRF_TIMER_CC_CHANNEL0);
    // Timer runs at 16 MHz (prescaler 0): 16 ticks = 1 us.
    uint32_t phase_time_us = phase_time_ticks / 16;

    int pin_val = nrf_gpio_pin_read(pin);

    if (pin_val == 1) {
        last_low_time = phase_time_us;
        last_period = last_high_time + last_low_time;
    } else {
        last_high_time = phase_time_us;
        if (last_high_time > 2500) {
            input_period = 0;
            return;
        }
        input_period = last_high_time;
    }
}

/**
 * @brief Initialize the GPIOTE/PPI/timer input-capture path for the pilot PWM signal.
 *
 * @return int 0 on success, or a negative error code on failure.
 */
static int pwm_input_init(void)
{
    nrfx_err_t status;
    uint32_t abs_pin = NRF_GPIO_PIN_MAP(0, throttle_in.pin); // Assuming port 0 for xiao_ble

    LOG_INF("Init Timer...");
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

    status = nrfx_timer_init(&timer2_inst, &timer_config, NULL);
    if (status != NRFX_SUCCESS && status != NRFX_ERROR_ALREADY) {
        LOG_ERR("Failed to initialize timer: %d", status);
        return -1;
    }

    LOG_INF("Init GPIOTE...");
    if (!nrfx_gpiote_init_check(&gpiote_inst)) {
        status = nrfx_gpiote_init(&gpiote_inst, 0);
        if (status != 0 && status != NRFX_ERROR_ALREADY) {
            LOG_ERR("Failed to initialize GPIOTE: %d", status);
            return -1;
        }
    }

    LOG_INF("Alloc GPIOTE channel...");
    uint8_t channel;
    status = nrfx_gpiote_channel_alloc(&gpiote_inst, &channel);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate GPIOTE channel: %d", status);
        return -1;
    }

    LOG_INF("Config GPIOTE pin...");
    nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLDOWN;
    nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
        .p_in_channel = &channel,
    };
    nrfx_gpiote_handler_config_t handler_config = {
        .handler = pwm_input_gpiote_handler,
    };
    nrfx_gpiote_input_pin_config_t input_config = {
        .p_pull_config = &pull_config,
        .p_trigger_config = &trigger_config,
        .p_handler_config = &handler_config,
    };

    status = nrfx_gpiote_input_configure(&gpiote_inst, abs_pin, &input_config);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Failed to initialize GPIOTE input pin %d: %d", abs_pin, status);
        return -1;
    }

    LOG_INF("Enable GPIOTE trigger...");
    nrfx_gpiote_trigger_enable(&gpiote_inst, abs_pin, true);

    uint32_t gpiote_evt_addr = nrfx_gpiote_in_event_address_get(&gpiote_inst, abs_pin);

    if (gpiote_evt_addr == 0) {
        LOG_ERR("Could not find GPIOTE channel for input pin");
        return -1;
    }

    LOG_INF("Alloc PPI channels...");
    uint32_t timer_cap_task = nrfx_timer_task_address_get(&timer2_inst, NRF_TIMER_TASK_CAPTURE0);
    uint32_t timer_clr_task = nrfx_timer_task_address_get(&timer2_inst, NRF_TIMER_TASK_CLEAR);

    if (nrfx_ppi_channel_alloc(&ppi_channel_cap) != NRFX_SUCCESS) {
        LOG_ERR("Failed to alloc PPI cap");
        return -1;
    }
    if (nrfx_ppi_channel_alloc(&ppi_channel_clr) != NRFX_SUCCESS) {
        LOG_ERR("Failed to alloc PPI clr");
        return -1;
    }

    LOG_INF("Assign PPI channels...");
    nrfx_ppi_channel_assign(ppi_channel_cap, gpiote_evt_addr, timer_cap_task);
    nrfx_ppi_channel_assign(ppi_channel_clr, gpiote_evt_addr, timer_clr_task);

    LOG_INF("Enable PPI channels...");
    nrfx_ppi_channel_enable(ppi_channel_cap);
    nrfx_ppi_channel_enable(ppi_channel_clr);

    LOG_INF("Enable Timer...");
    nrfx_timer_enable(&timer2_inst);

    LOG_INF("PWM input init complete.");
    return 0;
}

static inline int pwm_input_get_period(void)
{
    return input_period;
}

/* ------------------------------------------------------------------------- */
/* PWM output (ESC throttle)                                                 */
/* ------------------------------------------------------------------------- */

static const struct pwm_dt_spec esc_out = PWM_DT_SPEC_GET(DT_NODELABEL(esc));

/**
 * @brief Write a throttle pulse to the ESC output, clamped to valid PWM bounds.
 *
 * @param throttle_us Desired pulse width in microseconds.
 * @return int 0 on success, or a negative error code on failure.
 */
static int esc_set_throttle(float throttle_us)
{
    int pulse_us = (int)throttle_us;
    if (pulse_us < PWM_MIN_US) {
        pulse_us = PWM_MIN_US;
    } else if (pulse_us > PWM_MAX_US) {
        pulse_us = PWM_MAX_US;
    }

    int ret = pwm_set_pulse_dt(&esc_out, PWM_USEC(pulse_us));
    if (ret < 0) {
        LOG_ERR("Failed to set PWM pulse %d us: error %d", pulse_us, ret);
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* INA226 power sensor                                                       */
/* ------------------------------------------------------------------------- */

static struct {
    const struct device *dev;
    float current_mV;
    float current_mA;
    float current_joules;
} g_ina226;

/**
 * @brief Sample voltage, current, and power from the INA226 sensor.
 *
 * Applies the shunt-resistor scaling factor from the active configuration.
 *
 * @param power_w Pointer to store the measured power in Watts.
 * @return int 0 on success, or a negative error code on failure.
 */
static int ina226_sample(float *power_w)
{
    if (!power_w || !g_ina226.dev) {
        return -EINVAL;
    }

    struct sensor_value v_val, i_val, p_val;
    int ret;

    ret = sensor_sample_fetch(g_ina226.dev);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(g_ina226.dev, SENSOR_CHAN_VOLTAGE, &v_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(g_ina226.dev, SENSOR_CHAN_CURRENT, &i_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(g_ina226.dev, SENSOR_CHAN_POWER, &p_val);
    if (ret < 0) {
        return ret;
    }

    float voltage = (float)sensor_value_to_double(&v_val);
    float current = (float)sensor_value_to_double(&i_val);
    float power = (float)sensor_value_to_double(&p_val);

    // Retrieve active shunt resistor value thread-safely.
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    float shunt_mohm = g_config.shunt_resistor_mohm;
    k_mutex_unlock(&g_config_mutex);

    if (shunt_mohm <= 0.0f) {
        shunt_mohm = 1.0f; // Safety fallback
    }

    // The devicetree default shunt is 1.0 mOhm; scale by (DTS shunt / physical shunt).
    float shunt_scale = 1.0f / shunt_mohm;
    current *= shunt_scale;
    power *= shunt_scale;

    g_ina226.current_mV = voltage * 1000.0f;
    g_ina226.current_mA = current * 1000.0f;

    *power_w = power;

    return 0;
}

/* ------------------------------------------------------------------------- */
/* ADRC control law (first-order plant)                                      */
/* ------------------------------------------------------------------------- */

/*
 * Plant model:   dP/dt = b0 * u + f
 *   u  = ESC pulse actually applied        [us]
 *   b0 = input gain                        [(W/s) per us]  = static gain / tau
 *   f  = total disturbance (plant pole, load, voltage sag, unmodelled)  [W/s]
 *
 * The plant is treated as FIRST order (a throttle step drives power to a new
 * steady value with a single time constant), which is what the identification
 * measures: learning_fit_tau() fits log|target - y(t)| linearly, i.e. tau is a
 * first-order time constant, and the band averages give the static gain
 * dp/du. Hence b0 = (dp/du)/tau.
 *
 * Why not a second-order (double integrator) model: it forces the observer to
 * reconstruct dP/dt by differentiating a quantised, noisy INA226 reading, and
 * it needs the third state, whose bandwidth-parameterised gain is wo^3. With
 * wo = 308 rad/s that is l3*dt = 29,247 per 1 ms step, so 10 W of estimation
 * error injects ~290 kW/s^2 into the command - the rail-to-rail chopping seen
 * in flight. Here the disturbance-integrator gain is wo^2 instead (308x less)
 * and there is no derivative state at all.
 *
 * Second-order LESO (bandwidth-parameterised, forward Euler):
 *   z1 ~ P [W],  z2 ~ f [W/s]
 *   z1_dot = z2 + l1*(y - z1) + b0*u_applied,   l1 = 2*wo
 *   z2_dot =      l2*(y - z1),                  l2 = wo^2
 *
 * Control law:
 *   u = (kp*(r - z1) - z2) / b0
 *
 * u is fed back through adrc_set_applied() with the pulse the plant REALLY
 * received. Without that the observer is told the command was applied even
 * when the pilot's PWM (pwm_out = min(pwm_in, u)) or the [1000, 2000] us
 * actuator limits cut it - a phantom input that biases z2 and saturates the
 * command on the next tick.
 */
static struct {
    float b0;           // Input gain [(W/s) per us]
    float kp;           // Loop gain [1/s]: closed loop pole is -(1/tau + kp)
    float kd;           // UNUSED by the first-order law, kept for NVS/CLI compat
    float l1, l2;       // LESO gains: 2*wo, wo^2
    float z1;           // Estimated power [W]
    float z2;           // Estimated total disturbance f [W/s]
    float u_applied;    // Pulse the plant received during the last interval [us]
    float u_raw;        // Last computed command *before* clamping (diagnostics)
} g_adrc;

/* Upper bound on the observer bandwidth: keeps wo*dt <= 0.2 at the 1 kHz loop,
 * which is where a forward-Euler second-order LESO still behaves. */
#define ADRC_WO_MAX 200.0f

/* Plausibility ceiling for the stored input gain b0 = K/tau [(W/s) per us].
 * Reaching it would need, e.g., a static gain of 0.15 W/us with a plant time
 * constant below 3 ms. Values above it can only come from the old second-order
 * identification and trigger re-identification at boot. */
#define ADRC_B0_MAX_PLAUSIBLE 50.0f

/**
 * @brief Reset the ADRC observer and control states.
 *
 * @return int 0 on success.
 */
static int adrc_reset(void)
{
    g_adrc.z1 = 0.0f;
    g_adrc.z2 = 0.0f;
    g_adrc.u_raw = 0.0f;
    /* The ESC sits at safe stop until the first command is applied. */
    g_adrc.u_applied = (float)PWM_SAFE_STOP_US;

    return 0;
}

/**
 * @brief Tune and reset the ADRC controller.
 *
 * Computes bandwidth-parameterized second-order LESO gains:
 * l1 = 2*wo, l2 = wo^2.
 *
 * @param wo Observer bandwidth (rad/s).
 * @param b0 Input gain, (W/s) per us.
 * @param kp Loop gain, 1/s.
 * @param kd Unused by the first-order law (kept for CLI/NVS compatibility).
 * @return int 0 on success.
 */
static int adrc_tune(float wo, float b0, float kp, float kd)
{
    g_adrc.b0 = b0;
    g_adrc.kp = kp;
    g_adrc.kd = kd;
    g_adrc.l1 = 2.0f * wo;
    g_adrc.l2 = wo * wo;

    return adrc_reset();
}

/**
 * @brief Record the pulse the plant actually received.
 *
 * Must be called every control tick with the final, post-limit output pulse,
 * otherwise the observer integrates an input the actuator never applied.
 *
 * @param applied_us Applied ESC pulse width in us.
 */
static void adrc_set_applied(float applied_us)
{
    g_adrc.u_applied = applied_us;
}

/**
 * @brief Advance the ADRC observer and compute the control effort.
 *
 * Integrates the second-order LESO (forward Euler) driven by the last applied
 * pulse and computes the command, clamped to the ESC pulse limits.
 *
 * @param target_power Desired power limit in Watts.
 * @param measured_power Measured power in Watts.
 * @param dt Sampling time step in seconds.
 * @param u_ff Feedforward pulse width in us (0 for feedback only).
 * @param command Pointer to store the computed control effort (PWM us).
 * @return int 0 on success, -EINVAL if command is NULL or b0 is not positive.
 */
static int adrc_update(float target_power, float measured_power, float dt,
                       float u_ff, float *command)
{
    float b0 = g_adrc.b0;

    if (!command || b0 <= 0.0f) {
        return -EINVAL;
    }

    float z1 = g_adrc.z1;
    float z2 = g_adrc.z2;
    float err = measured_power - z1;

    // 1. LESO dynamics, driven by the input that was actually applied.
    float z1_dot = z2 + g_adrc.l1 * err + b0 * g_adrc.u_applied;
    float z2_dot = g_adrc.l2 * err;

    // 2. Update the estimated states.
    z1 += z1_dot * dt;
    z2 += z2_dot * dt;

    // 3. Control law: proportional action on the estimated power error plus
    //    full cancellation of the estimated disturbance, on top of the
    //    inverse-model feedforward. The feedforward is open loop, so it adds no
    //    phase lag: it supplies the throttle the target needs and lets the
    //    feedback only correct the residual. z2 absorbs any feedforward bias,
    //    so a stale map costs transient quality, never steady-state accuracy.
    float u = u_ff + (g_adrc.kp * (target_power - z1) - z2) / b0;

    // Keep the pre-clamp value: the binary stream reports it so the desktop
    // can see saturation / observer wind-up.
    g_adrc.u_raw = u;

    // 4. Clamp to physical ESC boundaries.
    if (u > PWM_MAX_US) {
        u = PWM_MAX_US;
    } else if (u < PWM_MIN_US) {
        u = PWM_MIN_US;
    }

    // Save state back.
    g_adrc.z1 = z1;
    g_adrc.z2 = z2;

    *command = u;

    return 0;
}

/* ------------------------------------------------------------------------- */
/* b0 learning mode                                                          */
/* ------------------------------------------------------------------------- */

/* PWM throttle bands used for b0 identification (percent of 1000-2000 us):
 * - Low band:  10% (1100 us) to 50% (1500 us)
 * - High band: 75% (1750 us) to the top of the valid input window (2100 us),
 *   so a pilot holding a little past full stick keeps feeding the high band.
 */
#define LEARNING_SAMPLES         256
#define LEARNING_STEPS_REQUIRED  3
#define LEARNING_PWM_LOW_MIN     (PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) / 10)
#define LEARNING_PWM_LOW_MAX     (PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) / 2)
#define LEARNING_PWM_HIGH_MIN    (PWM_MIN_US + 3 * (PWM_MAX_US - PWM_MIN_US) / 4)
#define LEARNING_PWM_HIGH_MAX    PWM_INPUT_MAX_US
#define LEARNING_TAU_MIN         0.01f
#define LEARNING_TAU_MAX         2.0f
#define LEARNING_TAU_SPREAD_MAX  0.5f
#define LEARNING_WC_MIN          5.0f
#define LEARNING_WC_MAX          100.0f
#define LEARNING_B0_MIN          0.001f
#define LEARNING_B0_MAX          100000.0f
#define LEARNING_POWER_CAP_FACTOR 1.2f

struct learn_sample {
    float pwm;
    float pwr;
    uint32_t tick;
};

static struct learn_sample learn_low[LEARNING_SAMPLES];
static struct learn_sample learn_high[LEARNING_SAMPLES];
static uint16_t learn_low_count;
static uint16_t learn_low_index;
static uint16_t learn_high_count;
static uint16_t learn_high_index;

static uint32_t learn_tick;        // 1 kHz learning sample counter
static int learn_band_state;       // 0 = none, 1 = low band, 2 = high band
static bool learn_step_detected;
static bool learn_step_up;
static uint32_t learn_step_tick;

static float learn_tau_samples[LEARNING_STEPS_REQUIRED];
static uint8_t learn_tau_count;
static float learn_power_cap;      // Hard power limit applied during learning
static bool learn_power_cut;       // True while the learning power cap is active

/**
 * @brief Clear all b0 learning buffers, counters, and step state.
 */
static void learning_reset(void)
{
    learn_low_count = 0;
    learn_low_index = 0;
    learn_high_count = 0;
    learn_high_index = 0;
    learn_tick = 0;
    learn_band_state = 0;
    learn_step_detected = false;
    learn_step_up = false;
    learn_step_tick = 0;
    learn_tau_count = 0;
    learn_power_cut = false;
    memset(learn_low, 0, sizeof(learn_low));
    memset(learn_high, 0, sizeof(learn_high));
    memset(learn_tau_samples, 0, sizeof(learn_tau_samples));
}

/**
 * @brief Start (or restart) a b0 learning run and print the first instruction
 *        over the USB console.
 */
static void learning_begin(void)
{
    learning_reset();
    g_learning_stage = LEARNING_STAGE_LOW_BAND;

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    learn_power_cap = LEARNING_POWER_CAP_FACTOR * g_config.target_power;
    k_mutex_unlock(&g_config_mutex);

    printf("[LEARN] Hold throttle between 10%% and 50%% (1100-1500 us).\r\n");
    printf("[LEARN] Safety cap during learning: %.0f W.\r\n", (double)learn_power_cap);
}

/**
 * @brief Restart a failed b0 learning run with a reason message over USB.
 *
 * @param reason Human-readable failure reason.
 */
static void learning_restart(const char *reason)
{
    learning_reset();
    g_learning_stage = LEARNING_STAGE_LOW_BAND;
    printf("[LEARN] %s Restarting: hold throttle between 10%% and 50%% (1100-1500 us).\r\n",
           reason);
}

static bool learning_capture_tau(void);
static void learning_finalize(void);

/**
 * @brief Collect one moving-average sample pair, detect throttle steps, and
 *        advance the multi-step learning sequence.
 *
 * @param pwm_in Pilot throttle pulse width in us.
 * @param power_w Measured power in Watts.
 */
static void learning_collect(int pwm_in, float power_w)
{
    int band = 0;

    if (pwm_in >= LEARNING_PWM_LOW_MIN && pwm_in <= LEARNING_PWM_LOW_MAX) {
        band = 1; // Low band
    } else if (pwm_in >= LEARNING_PWM_HIGH_MIN && pwm_in <= LEARNING_PWM_HIGH_MAX) {
        band = 2; // High band
    }

    // Detect a band-to-band step and reset the destination ring so it only
    // contains post-step samples (needed for the time-constant fit).
    if (band != 0 && band != learn_band_state) {
        if (learn_band_state == 1 && band == 2) {
            learn_step_detected = true;
            learn_step_up = true;
            learn_step_tick = learn_tick;
            learn_high_count = 0;
            learn_high_index = 0;
            g_learning_stage = LEARNING_STAGE_STEP_HIGH;
            printf("[LEARN] Step up detected. Hold throttle above 75%% (1750-2100 us).\r\n");
        } else if (learn_band_state == 2 && band == 1) {
            learn_step_detected = true;
            learn_step_up = false;
            learn_step_tick = learn_tick;
            learn_low_count = 0;
            learn_low_index = 0;
            g_learning_stage = LEARNING_STAGE_STEP_LOW;
            printf("[LEARN] Step down detected. Hold throttle between 10%% and 50%%.\r\n");
        }
        learn_band_state = band;
    }

    if (band == 1) {
        learn_low[learn_low_index].pwm = (float)pwm_in;
        learn_low[learn_low_index].pwr = power_w;
        learn_low[learn_low_index].tick = learn_tick;
        learn_low_index = (learn_low_index + 1) % LEARNING_SAMPLES;
        if (learn_low_count < LEARNING_SAMPLES) {
            learn_low_count++;
        }

        if (learn_low_count == LEARNING_SAMPLES) {
            if (g_learning_stage == LEARNING_STAGE_LOW_BAND) {
                g_learning_stage = LEARNING_STAGE_STEP_HIGH;
                printf("[LEARN] Low band captured. Slam throttle above 75%% and hold.\r\n");
            } else if (g_learning_stage == LEARNING_STAGE_STEP_LOW) {
                if (learning_capture_tau()) {
                    if (learn_tau_count >= LEARNING_STEPS_REQUIRED) {
                        g_learning_stage = LEARNING_STAGE_ESTIMATING;
                        learning_finalize();
                    } else {
                        g_learning_stage = LEARNING_STAGE_STEP_HIGH;
                        printf("[LEARN] Step %d/%d captured (tau=%.3f s). Slam throttle above 75%% and hold.\r\n",
                               learn_tau_count, LEARNING_STEPS_REQUIRED,
                               (double)learn_tau_samples[learn_tau_count - 1]);
                    }
                } else {
                    learning_restart("Step fit failed.");
                }
            }
        }
    } else if (band == 2) {
        learn_high[learn_high_index].pwm = (float)pwm_in;
        learn_high[learn_high_index].pwr = power_w;
        learn_high[learn_high_index].tick = learn_tick;
        learn_high_index = (learn_high_index + 1) % LEARNING_SAMPLES;
        if (learn_high_count < LEARNING_SAMPLES) {
            learn_high_count++;
        }

        if (learn_high_count == LEARNING_SAMPLES &&
            g_learning_stage == LEARNING_STAGE_STEP_HIGH) {
            if (learning_capture_tau()) {
                if (learn_tau_count >= LEARNING_STEPS_REQUIRED) {
                    g_learning_stage = LEARNING_STAGE_ESTIMATING;
                    learning_finalize();
                } else {
                    g_learning_stage = LEARNING_STAGE_STEP_LOW;
                    printf("[LEARN] Step %d/%d captured (tau=%.3f s). Return throttle to 10-50%% and hold.\r\n",
                           learn_tau_count, LEARNING_STEPS_REQUIRED,
                           (double)learn_tau_samples[learn_tau_count - 1]);
                }
            } else {
                learning_restart("Step fit failed.");
            }
        }
    }
}

/**
 * @brief Compute the average of a full learning sample ring.
 *
 * @param ring Sample ring to average.
 * @param use_pwr True to average power, false to average PWM.
 * @return float Ring average.
 */
static float learning_ring_avg(const struct learn_sample *ring, bool use_pwr)
{
    float sum = 0.0f;

    for (uint16_t i = 0; i < LEARNING_SAMPLES; i++) {
        sum += use_pwr ? ring[i].pwr : ring[i].pwm;
    }

    return sum / LEARNING_SAMPLES;
}

/**
 * @brief Fit the motor time constant from a recorded step response.
 *
 * Uses a log-linear least-squares fit of a first-order response
 * y(t) = y0 + dy*(1 - exp(-t/tau)). Only samples between 10% and 90% of the
 * steady-state transition are used, where ln(1 - (y-y0)/dy) is well behaved.
 *
 * @param ring Post-step sample ring (full).
 * @param idx Oldest sample index in the ring.
 * @param start_pwr Power before the step (Watts).
 * @param target_pwr Settled power after the step (Watts).
 * @return float Time constant in seconds, or -1 if the fit failed.
 */
static float learning_fit_tau(const struct learn_sample *ring, uint16_t idx,
                              float start_pwr, float target_pwr)
{
    float dy = target_pwr - start_pwr;

    if (fabsf(dy) < 1e-3f) {
        return -1.0f;
    }

    float sum_t = 0.0f;
    float sum_z = 0.0f;
    float sum_tz = 0.0f;
    float sum_tt = 0.0f;
    int n = 0;

    for (uint16_t i = 0; i < LEARNING_SAMPLES; i++) {
        const struct learn_sample *s = &ring[(idx + i) % LEARNING_SAMPLES];
        float t = (float)(s->tick - learn_step_tick) * CONTROL_PERIOD_S;
        float frac = (s->pwr - start_pwr) / dy;

        if (frac < 0.10f || frac > 0.90f) {
            continue;
        }

        float z = logf(1.0f - frac);
        sum_t += t;
        sum_z += z;
        sum_tz += t * z;
        sum_tt += t * t;
        n++;
    }

    if (n < 16) {
        return -1.0f;
    }

    float denom = n * sum_tt - sum_t * sum_t;
    if (fabsf(denom) < 1e-9f) {
        return -1.0f;
    }

    float slope = (n * sum_tz - sum_t * sum_z) / denom;
    if (slope >= 0.0f) {
        return -1.0f; // The response must decay toward the target.
    }

    return -1.0f / slope;
}

/**
 * @brief Capture the time constant of the most recent throttle step.
 *
 * The pre-step average comes from the band the step left; the post-step
 * average and step record come from the band the step entered.
 *
 * @return true if a tau estimate was captured, false otherwise.
 */
static bool learning_capture_tau(void)
{
    if (!learn_step_detected ||
        learn_low_count < LEARNING_SAMPLES ||
        learn_high_count < LEARNING_SAMPLES) {
        return false;
    }

    const struct learn_sample *post_ring;
    uint16_t post_idx;
    float start_pwr;
    float target_pwr;

    if (learn_step_up) {
        post_ring = learn_high;
        post_idx = learn_high_index;
        start_pwr = learning_ring_avg(learn_low, true);
        target_pwr = learning_ring_avg(learn_high, true);
    } else {
        post_ring = learn_low;
        post_idx = learn_low_index;
        start_pwr = learning_ring_avg(learn_high, true);
        target_pwr = learning_ring_avg(learn_low, true);
    }

    float tau = learning_fit_tau(post_ring, post_idx, start_pwr, target_pwr);
    if (tau <= 0.0f) {
        return false;
    }

    learn_tau_samples[learn_tau_count++] = tau;
    return true;
}

/**
 * @brief Return the median of the captured tau samples.
 *
 * @return float Median time constant in seconds.
 */
static float learning_median_tau(void)
{
    float v[LEARNING_STEPS_REQUIRED];

    for (uint8_t i = 0; i < LEARNING_STEPS_REQUIRED; i++) {
        v[i] = learn_tau_samples[i];
    }

    for (uint8_t i = 0; i < LEARNING_STEPS_REQUIRED - 1; i++) {
        for (uint8_t j = i + 1; j < LEARNING_STEPS_REQUIRED; j++) {
            if (v[j] < v[i]) {
                float tmp = v[i];
                v[i] = v[j];
                v[j] = tmp;
            }
        }
    }

    return v[LEARNING_STEPS_REQUIRED / 2];
}

/**
 * @brief Finalize the learning run: validate the tau estimates, derive all
 *        ADRC parameters, save them to NVS, and exit learning mode.
 */
static void learning_finalize(void)
{
    if (learn_low_count < LEARNING_SAMPLES || learn_high_count < LEARNING_SAMPLES) {
        learning_restart("Incomplete bands.");
        return;
    }

    float pwm_low_avg = learning_ring_avg(learn_low, false);
    float pwm_high_avg = learning_ring_avg(learn_high, false);
    float pwr_low_avg = learning_ring_avg(learn_low, true);
    float pwr_high_avg = learning_ring_avg(learn_high, true);

    float du = pwm_high_avg - pwm_low_avg;
    float dp = pwr_high_avg - pwr_low_avg;

    if (du <= 0.0f || dp <= 0.0f) {
        LOG_WRN("b0 learning: invalid slope (du=%.1f, dp=%.1f).", (double)du, (double)dp);
        learning_restart("Invalid power slope.");
        return;
    }

    // Consistency check: the captured step responses must agree with each other.
    float tau = learning_median_tau();
    float tau_min = learn_tau_samples[0];
    float tau_max = learn_tau_samples[0];

    for (uint8_t i = 1; i < LEARNING_STEPS_REQUIRED; i++) {
        if (learn_tau_samples[i] < tau_min) {
            tau_min = learn_tau_samples[i];
        }
        if (learn_tau_samples[i] > tau_max) {
            tau_max = learn_tau_samples[i];
        }
    }

    if ((tau_max - tau_min) > LEARNING_TAU_SPREAD_MAX * tau) {
        LOG_WRN("b0 learning: tau estimates inconsistent (%.3f .. %.3f s).",
                (double)tau_min, (double)tau_max);
        learning_restart("Step responses inconsistent.");
        return;
    }

    if (tau < LEARNING_TAU_MIN) {
        tau = LEARNING_TAU_MIN;
    } else if (tau > LEARNING_TAU_MAX) {
        LOG_WRN("b0 learning: tau %.3f s out of range.", (double)tau);
        learning_restart("Time constant out of range.");
        return;
    }

    float wc = 2.0f / tau;
    if (wc < LEARNING_WC_MIN) {
        wc = LEARNING_WC_MIN;
    } else if (wc > LEARNING_WC_MAX) {
        wc = LEARNING_WC_MAX;
    }

    /*
     * First-order plant identification.
     *
     * learning_fit_tau() fits log|target - y(t)| linearly, so the median tau is
     * a genuine first-order time constant, and the band averages give the
     * static gain dp/du. The input gain of the model dP/dt = b0*u + f is
     * therefore
     *     b0 = (dp/du) / tau          [(W/s) per us]
     * (the previous second-order reading multiplied by another 1/tau, which is
     * why it came out ~40x too large for this plant).
     */
    float learned_b0 = dp / (tau * du);
    if (learned_b0 < LEARNING_B0_MIN || learned_b0 > LEARNING_B0_MAX) {
        LOG_WRN("b0 learning: learned b0 %.4f out of range.", (double)learned_b0);
        learning_restart("Learned b0 out of range.");
        return;
    }

    /*
     * Loop gain: kp has units 1/s and sets the closed-loop pole at
     * -(1/tau + kp), i.e. wc = 2/tau makes the closed loop three times faster
     * than the open-loop plant. It also fixes how much power error uses the
     * full actuator stroke:
     *     full stroke (1000 us) at error = 1000*b0/kp = 500*(dp/du)
     * e.g. a static gain of 0.15 W/us gives 75 W of error for full authority.
     */
    float learned_kp = wc;
    float learned_kd = 0.0f; // Not used by the first-order law.

    float learned_wo = 5.0f * wc;
    if (learned_wo > ADRC_WO_MAX) {
        LOG_WRN("b0 learning: wo %.1f clamped to %.1f (wo*dt <= 0.2 at 1 kHz).",
                (double)learned_wo, (double)ADRC_WO_MAX);
        learned_wo = ADRC_WO_MAX;
    }

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.wo = learned_wo;
    g_config.b0 = learned_b0;
    g_config.kp = learned_kp;
    g_config.kd = learned_kd;
    int ret = adrc_tune(learned_wo, learned_b0, learned_kp, learned_kd);
    if (ret == 0) {
        persistence_save_config(&g_config);
    }
    k_mutex_unlock(&g_config_mutex);

    if (ret < 0) {
        LOG_ERR("b0 learning: failed to apply learned parameters (error %d)", ret);
        learning_restart("Failed to apply learned parameters.");
        return;
    }

    g_learning_mode = false;
    g_learning_stage = LEARNING_STAGE_DONE;
    stream_meta_dirty = true; // Publish the learned ADRC gains to the stream.
    printf("[LEARN] Done: tau=%.3f s, wc=%.1f, wo=%.1f, b0=%.4f, kp=%.2f\r\n",
           (double)tau, (double)wc, (double)learned_wo, (double)learned_b0,
           (double)learned_kp);
    printf("[LEARN] Parameters saved to NVS. Normal power limiting resumed.\r\n");
    LOG_INF("b0 learning complete: tau=%.3f s, wc=%.1f, wo=%.1f, b0=%.4f, kp=%.2f",
            (double)tau, (double)wc, (double)learned_wo, (double)learned_b0,
            (double)learned_kp);
}

/* ------------------------------------------------------------------------- */
/* Control loop (Thread 1, 1 kHz)                                           */
/* ------------------------------------------------------------------------- */

/**
 * @brief Evaluate the active control state from the current operating conditions.
 *
 * @param input_valid True when the pilot PWM input is inside the valid window.
 * @param battery_valid True when the bus voltage is above the minimum threshold.
 * @param pwm_in Pilot throttle pulse width in us.
 * @param pwm_out Applied ESC pulse width in us.
 * @return enum ctrl_state The active control state.
 */
static enum ctrl_state control_evaluate_state(bool input_valid, bool battery_valid,
                                              int pwm_in, int pwm_out)
{
    if ((int64_t)(k_uptime_get() - g_blink_start_time) < BLINK_HOLD_MS) {
        return BLINK;
    } else if (!input_valid) {
        return ERROR_NO_INPUT;
    } else if (!battery_valid) {
        return ERROR_NO_BATTERY;
    } else if (g_learning_mode) {
        return LEARNING;
    } else if (pwm_out < pwm_in) {
        return LIMITING_POWER;
    } else {
        return READY;
    }
}

/* ------------------------------------------------------------------------- */
/* Binary telemetry stream feed                                              */
/* ------------------------------------------------------------------------- */

static inline int16_t stream_i16(float v)
{
    if (v > 32767.0f) {
        return INT16_MAX;
    }
    if (v < -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

static inline uint16_t stream_u16(float v)
{
    if (v < 0.0f) {
        return 0u;
    }
    if (v > 65535.0f) {
        return UINT16_MAX;
    }
    return (uint16_t)v;
}

static inline int32_t stream_i32(float v)
{
    if (v > 2147483000.0f) {
        return INT32_MAX;
    }
    if (v < -2147483000.0f) {
        return INT32_MIN;
    }
    return (int32_t)v;
}

/**
 * @brief Queue a controller/configuration snapshot for the binary stream.
 *
 * Called from the control thread only (single producer).
 */
static void stream_feed_meta(void)
{
    struct pm100_stream_meta meta;

    memset(&meta, 0, sizeof(meta));

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    meta.wo = g_config.wo;
    meta.b0 = g_config.b0;
    meta.kp = g_config.kp;
    meta.kd = g_config.kd;
    meta.target_power = g_config.target_power;
    meta.shunt_mohm = g_config.shunt_resistor_mohm;
    meta.dt = g_config.dt;
    meta.team_number = g_config.team_number;
    strncpy(meta.team_name, g_config.team_name, sizeof(meta.team_name) - 1);
    k_mutex_unlock(&g_config_mutex);

    meta.l1 = g_adrc.l1;
    meta.l2 = g_adrc.l2;
    meta.l3 = 0.0f; // No third LESO state with the first-order law.
    meta.uptime_ms = (uint32_t)k_uptime_get();
    meta.learning_stage = (uint8_t)g_learning_stage;

    /* Report the period the loop actually runs at, not the nominal 1 ms. */
    meta.dt = g_loop_dt_s;

    stream_push_meta(&meta);
}

/* ------------------------------------------------------------------------- */
/* Automatic throttle identification (staircase)                             */
/* ------------------------------------------------------------------------- */

/*
 * The firmware commands a sequence of settled throttle levels itself and fits
 * K, tau and the stimulus dead time on each step. Compared with the manual
 * procedure (pilot slams the stick through a band) this removes every
 * contamination term:
 *   - the step is one control tick wide, not a human ramp,
 *   - t = 0 is known exactly,
 *   - each level's settled power is measured AFTER it settles, instead of
 *     using the average of the whole transient as the asymptote,
 *   - the stimulus dead time (the ESC PWM frame is 20 ms wide, so a step lands
 *     0..20 ms after the command) is fitted instead of being assumed zero.
 *
 * Gains are then derived from the IMC / lambda rule. With the plant written as
 * dP/dt = b0*u + f and b0 = K/tau, an IMC-tuned PI maps onto this law as:
 *     kp = 1/lambda          [1/s]
 *     wo = 1/sqrt(lambda*tau) [rad/s]
 * so the only remaining choice is lambda.
 */

/* 10%..90% of the 1000..2000 us ESC band: never rails the actuator. */
static const uint16_t ident_level_us[CONTROL_IDENT_LEVELS] = {1100, 1300, 1500, 1700, 1900};

#define IDENT_ARM_IDLE_US      1200u  /* stick below this counts as idle       */
#define IDENT_ARM_HOLD_MS      1000u  /* ... for this long before takeover     */
#define IDENT_TIMEOUT_MS       30000u /* hard abort, wall clock                */
#define IDENT_POWER_CAP_W      400.0f /* hard abort on measured power          */
#define IDENT_SETTLE_US        700000u/* hold per level after its capture      */
#define IDENT_CAPTURE_US       300000u/* shortest usable capture window        */
#define IDENT_CAPTURE_TICKS    256u   /* ... and the fewest samples it may hold */
#define IDENT_CAPTURE_MAX      512u   /* capture buffer bound                  */
#define IDENT_TAU_MIN_S        0.003f /* plausible plant time constants        */
#define IDENT_TAU_MAX_S        2.000f
#define IDENT_CROSS_LOW        0.10f  /* crossings that define the transit time */
#define IDENT_CROSS_MID        0.63f
#define IDENT_CROSS_HIGH       0.90f
#define IDENT_MIN_BAND_SAMPLES 3u     /* samples the 10-90% transit must span  */
#define IDENT_SHAPE_MIN        0.7f   /* (t90-t63)/(t63-t10); 1.47 = first order */
#define IDENT_SHAPE_MAX        3.5f
#define IDENT_MIN_STEP_W       5.0f   /* steps smaller than this identify noise */
#define IDENT_MAX_TAU_REL_ERR  0.45f  /* reject a step whose tau is this vague */
#define IDENT_TAU_SPREAD_WARN  2.5f   /* warn if tau varies this much across steps */

enum ident_phase {
	IDENT_IDLE = 0,
	IDENT_ARM,     /* watching the stick, output NOT taken over */
	IDENT_SETTLE,  /* holding a level, measuring its settled power */
	IDENT_CAPTURE, /* transient of the step into the current level  */
};

static struct {
	enum ident_phase phase;
	float lambda_req;
	uint8_t idx;             /* level currently applied */
	uint32_t start_ms;       /* when the run was armed */
	uint32_t arm_since_ms;   /* when the stick first became idle (0 = not yet) */
	uint64_t step_us;        /* timebase at the step into the current level */
	uint64_t settle_us;      /* timebase the current settle window began at */
	uint16_t pulse;          /* pulse the ident commands */
	struct {
		float pwr;
		uint32_t t_us;   /* elapsed since the step, us */
	} cap[IDENT_CAPTURE_MAX];
	uint16_t cap_count;
	float settle_sum;
	float settle_sumsq;
	uint16_t settle_avg_count;
	float v_sum;
	uint32_t v_count;
} g_ident;

static struct control_ident_result g_ident_res;

static void ident_fit(uint8_t level, float start_w, float target_w, float noise_w);
static void ident_finish(bool success, const char *reason);

/**
 * @brief Throttle that produces target_w at the current bus voltage, by
 *        inverting the identified staircase map (inverse-model feedforward).
 *
 * The dead-band-corrected interpolation is linear in the incremental throttle
 * above the ESC floor, and scaled by V_ref/V_bus because the plant gain is
 * proportional to the bus voltage. Returns 0 when there is no usable map or
 * the target lies outside the identified power range, leaving the feedback
 * path to do all the work.
 *
 * @param target_w Power target in Watts.
 * @param v_bus Measured bus voltage in Volts.
 * @return Feedforward throttle in us, or 0 when unavailable.
 */
static float control_ff_throttle(float target_w, float v_bus)
{
	const struct control_ident_result *r = &g_ident_res;
	float frac, u;

	if (!r->valid || r->levels < 2 || r->v_ref < 1.0f) {
		return 0.0f;
	}
	if (target_w <= r->p_settled[0] || target_w >= r->p_settled[r->levels - 1]) {
		return 0.0f; /* outside what was identified: stay feedback-only */
	}

	for (uint8_t i = 1; i < r->levels; i++) {
		float p0 = r->p_settled[i - 1];
		float p1 = r->p_settled[i];

		if (target_w > p1) {
			continue;
		}
		if (p1 - p0 < 1.0f) {
			return 0.0f;
		}

		frac = (target_w - p0) / (p1 - p0);
		u = (float)r->u_us[i - 1] + frac * (float)(r->u_us[i] - r->u_us[i - 1]);

		/* Plant gain ~ V_bus, so the throttle increment scales as 1/V_bus. */
		if (v_bus > 1.0f) {
			u = (float)PWM_MIN_US +
			    (u - (float)PWM_MIN_US) * (r->v_ref / v_bus);
		}

		return (u > (float)PWM_MAX_US) ? (float)PWM_MAX_US : u;
	}

	return 0.0f;
}

/**
 * @brief Push one raw capture sample to the binary stream.
 *
 * @param power_w Measured power in Watts.
 * @param step True for the first sample of a capture window.
 */
static void ident_record(float power_w, bool step)
{
	struct pm100_stream_ident rec;

	rec.t_ms = (uint32_t)k_uptime_get();
	rec.cmd_us = g_ident.pulse;
	rec.p_dw = stream_i16(power_w * 10.0f);
	rec.level = g_ident.idx;
	rec.flags = step ? PM100_IDENT_FLAG_STEP : 0u;
	rec._rsvd[0] = 0u;
	rec._rsvd[1] = 0u;

	stream_push_ident(&rec);
}

/**
 * @brief Time at which the step response first crosses a fraction of the step.
 *
 * Works on the normalised progress g = (P - P0)/dy, which rises from 0 to 1 for
 * a step in either direction, and interpolates linearly between the two samples
 * that straddle the crossing.
 *
 * @param frac Fraction of the step to look for (0..1).
 * @param start_w Settled power of the previous level.
 * @param dy Step size (target - start).
 * @param idx Index of the first sample at or past the crossing.
 * @param t_us Interpolated crossing time, us after the step.
 * @return true if the crossing happens inside the capture window.
 */
static bool ident_crossing(float frac, float start_w, float dy, uint16_t *idx, float *t_us)
{
	float g_prev = (g_ident.cap[0].pwr - start_w) / dy;

	for (uint16_t i = 1; i < g_ident.cap_count; i++) {
		float g = (g_ident.cap[i].pwr - start_w) / dy;

		if (g_prev < frac && g >= frac) {
			float span = (float)(g_ident.cap[i].t_us - g_ident.cap[i - 1].t_us);

			*idx = i;
			*t_us = (float)g_ident.cap[i - 1].t_us +
				((frac - g_prev) / (g - g_prev)) * span;
			return true;
		}
		g_prev = g;
	}

	return false;
}

/**
 * @brief Estimate tau from the step's 10%, 63% and 90% crossings.
 *
 * A first-order response has t(f) = t0 - tau*ln(1 - f), so
 *     t10 = t0 + 0.105*tau,  t63 = t0 + 0.994*tau,  t90 = t0 + 2.303*tau
 * and therefore
 *     tau = (t63 - t10) / 0.889,   t0 = t10 - 0.105*tau.
 *
 * This replaces a weighted log-linear regression, which was the wrong tool for
 * this plant: it fits a straight line to ln(1 - frac), a transform that
 * diverges as frac -> 1, so the slow tail of a non-exponential response drags
 * the slope down. On the bench that produced 192 ms from one step and 49 ms
 * from the next, for the same plant. The two crossings used here both sit in
 * the steep part of the response, so they are insensitive to what the tail
 * does, and the estimator is immune to the actuator dead time (a shift moves
 * both crossings equally).
 *
 * Two checks come free with it:
 *  - **Resolution**: the 10-90% transit must span IDENT_MIN_BAND_SAMPLES samples,
 *    otherwise the step happened between samples and no tau can be extracted.
 *  - **Shape**: (t90 - t63)/(t63 - t10) is 1.31/0.89 = 1.47 for *any*
 *    first-order response, regardless of tau or dead time. A value far from
 *    that means the first-order model does not describe this step.
 *
 * The uncertainty is closed form: a crossing time is wrong by
 * sigma_P / (dP/dt), and dP/dt = dy*(1-f)/tau at fraction f, so
 *     sigma_tau / tau = (sigma_P / dy) * sqrt((1/0.9)^2 + (1/0.37)^2) / 0.889
 *                     = 3.29 * sigma_P / dy
 * i.e. a step must be many times the measurement noise to say anything about
 * tau at all.
 *
 * @param level Level index the step entered (> 0).
 * @param start_w Settled power of the previous level (Watts).
 * @param target_w Settled power of this level (Watts).
 * @param noise_w Power noise measured in the settled phase, W rms.
 */
static void ident_fit(uint8_t level, float start_w, float target_w, float noise_w)
{
	float dy = target_w - start_w;
	uint16_t i10 = 0u, i63 = 0u, i90 = 0u;
	float t10, t63, t90, tau, t0, shape, rel_err;

	if (g_ident.cap_count < 8u || fabsf(dy) < IDENT_MIN_STEP_W) {
		LOG_WRN("ident: level %u skipped (dy=%.1f W, %u samples)",
			(unsigned int)level, (double)dy, (unsigned int)g_ident.cap_count);
		return;
	}

	if (!ident_crossing(IDENT_CROSS_LOW, start_w, dy, &i10, &t10) ||
	    !ident_crossing(IDENT_CROSS_MID, start_w, dy, &i63, &t63) ||
	    !ident_crossing(IDENT_CROSS_HIGH, start_w, dy, &i90, &t90)) {
		LOG_WRN("ident: level %u (step %.1f W) never reached 90%% - motor stalled, "
			"ESC ramping, or the level is saturated",
			(unsigned int)level, (double)dy);
		return;
	}

	if ((i90 - i10) < IDENT_MIN_BAND_SAMPLES) {
		LOG_WRN("ident: level %u step is faster than the %.2f ms sample period can "
			"resolve (%u samples between 10%% and 90%%)",
			(unsigned int)level, (double)(g_loop_dt_s * 1000.0f),
			(unsigned int)(i90 - i10));
		return;
	}

	tau = (t63 - t10) * 1e-6f / 0.889f;
	shape = (t90 - t63) / (t63 - t10);
	t0 = t10 * 1e-6f - 0.105f * tau;
	rel_err = (fabsf(dy) > 1e-3f) ? (3.29f * noise_w / fabsf(dy)) : 1.0f;

	LOG_INF("ident: L%u u=%u dy=%.1f W tau=%.1f ms +-%.0f%% t10=%.0f t63=%.0f t90=%.0f ms "
		"shape=%.2f (1.47=first order) noise=%.2f W band=%u",
		(unsigned int)level, (unsigned int)ident_level_us[level], (double)dy,
		(double)(tau * 1000.0f), (double)(100.0f * rel_err),
		(double)(t10 * 1e-3f), (double)(t63 * 1e-3f), (double)(t90 * 1e-3f),
		(double)shape, (double)noise_w, (unsigned int)(i90 - i10));

	g_ident_res.shape[level] = shape;
	g_ident_res.tau_rel_err[level] = rel_err;
	g_ident_res.deadtime[level] = (t0 > 0.0f) ? t0 : 0.0f;
	g_ident_res.n_band[level] = i90 - i10;
	g_ident_res.noise_w[level] = noise_w;

	if (tau < IDENT_TAU_MIN_S || tau > IDENT_TAU_MAX_S) {
		LOG_WRN("ident: level %u tau %.1f ms is outside the plausible %.0f-%.0f ms range",
			(unsigned int)level, (double)(tau * 1000.0f),
			(double)(IDENT_TAU_MIN_S * 1000.0f), (double)(IDENT_TAU_MAX_S * 1000.0f));
		return;
	}

	if (shape < IDENT_SHAPE_MIN || shape > IDENT_SHAPE_MAX) {
		LOG_WRN("ident: level %u shape %.2f is far from the 1.47 of a first-order "
			"response - the plant is not a single lag here, tau is a compromise",
			(unsigned int)level, (double)shape);
	}

	/* Keep the numbers for the report; zero tau means "did not vote". */
	g_ident_res.tau[level] =
		(rel_err <= IDENT_MAX_TAU_REL_ERR) ? tau : 0.0f;

	if (rel_err > IDENT_MAX_TAU_REL_ERR) {
		LOG_WRN("ident: level %u rejected, tau %.1f ms is only known to +-%.0f%% "
			"(step %.1f W against %.2f W of noise)",
			(unsigned int)level, (double)(tau * 1000.0f),
			(double)(100.0f * rel_err), (double)dy, (double)noise_w);
	}
}

/**
 * @brief Abort or complete an identification run and hand the throttle back.
 *
 * @param success True if the staircase completed.
 * @param reason Abort reason, or NULL on success.
 */
static void ident_finish(bool success, const char *reason)
{
	g_ident.phase = IDENT_IDLE;
	g_ident_res.running = false;

	if (!success && reason) {
		strncpy(g_ident_res.last_abort, reason, sizeof(g_ident_res.last_abort) - 1);
		g_ident_res.last_abort[sizeof(g_ident_res.last_abort) - 1] = '\0';
		LOG_WRN("ident: aborted (%s)", reason);
		printf("[IDENT] Aborted: %s\r\n", reason);

		/* Leaving an operator to discover this in the air is not acceptable. */
		if (g_learning_mode) {
			LOG_WRN("ident: still in b0 learning pass-through - the power limiter "
				"is NOT active");
			printf("[IDENT] No gains applied: the LIMITER IS STILL OFF "
			       "(b0 learning pass-through).\r\n");
		}
	}
}

/**
 * @brief Derive lambda-tuned gains from the staircase and apply them.
 */
static void ident_finalize(void)
{
	float taus[CONTROL_IDENT_LEVELS];
	uint8_t n_tau = 0;
	uint8_t usable = 1u;
	float tau_med, lambda, gain, target, b0, kp, wo;

	/*
	 * Only the monotonic prefix of the staircase is invertible. Above the point
	 * where K stops being positive the ESC (or the battery) is saturated: the
	 * bench run showed P(1900 us) = 187.7 W against P(1700 us) = 189.2 W, so an
	 * inverse map over the whole staircase would command a throttle that
	 * produces no more power and the feedforward would overshoot for nothing.
	 */
	for (uint8_t i = 1; i < CONTROL_IDENT_LEVELS; i++) {
		if (g_ident_res.k[i] > 0.0f) {
			usable = i + 1u;
		} else {
			break;
		}
	}
	g_ident_res.levels = usable;

	for (uint8_t i = 1; i < CONTROL_IDENT_LEVELS; i++) {
		/* ident_fit() zeroes tau for steps it does not trust. */
		if (g_ident_res.tau[i] > 0.0f &&
		    g_ident_res.tau_rel_err[i] <= IDENT_MAX_TAU_REL_ERR) {
			taus[n_tau++] = g_ident_res.tau[i];
		}
	}

	g_ident_res.n_tau = n_tau;

	if (usable < 2u) {
		ident_finish(false, "Staircase is not monotonic: no usable throttle range.");
		return;
	}

	if (n_tau == 0u) {
		for (uint8_t i = 1; i < CONTROL_IDENT_LEVELS; i++) {
			LOG_WRN("ident: L%u P %.1f -> %.1f W, dy=%.1f W, tau=%.1f ms +-%.0f%%, "
				"shape=%.2f, noise=%.2f W, band=%u",
				(unsigned int)i,
				(double)g_ident_res.p_settled[i - 1],
				(double)g_ident_res.p_settled[i],
				(double)(g_ident_res.p_settled[i] - g_ident_res.p_settled[i - 1]),
				(double)(g_ident_res.tau[i] * 1000.0f),
				(double)(100.0f * g_ident_res.tau_rel_err[i]),
				(double)g_ident_res.shape[i], (double)g_ident_res.noise_w[i],
				(unsigned int)g_ident_res.n_band[i]);
		}
		ident_finish(false, "No step produced a usable tau.");
		return;
	}

	for (uint8_t i = 1; i < n_tau; i++) {
		float v = taus[i];
		int8_t j = (int8_t)i - 1;

		while (j >= 0 && taus[j] > v) {
			taus[j + 1] = taus[j];
			j--;
		}
		taus[j + 1] = v;
	}
	/* Geometric mean for an even count: tau is a ratio-scale quantity, and the
	 * bench run showed an arithmetic middle would land on the outlier (49 and
	 * 192 ms would average to 120 ms rather than the 97 ms the pair implies). */
	tau_med = (n_tau & 1u)
			  ? taus[n_tau / 2u]
			  : sqrtf(taus[n_tau / 2u - 1u] * taus[n_tau / 2u]);

	if (n_tau < 2u) {
		LOG_WRN("ident: only %u of %u steps produced a usable tau - the value below "
			"has no cross-check",
			(unsigned int)n_tau, (unsigned int)(CONTROL_IDENT_LEVELS - 1u));
	} else if (taus[n_tau - 1u] > taus[0] * IDENT_TAU_SPREAD_WARN) {
		/* A spread this wide means the plant is not uniformly first-order over
		 * the throttle range, so no single tau describes it. The median is the
		 * loop's compromise and the per-step table shows where it comes from. */
		LOG_WRN("ident: tau spans %.1f..%.1f ms across the range (%.1fx) - the plant "
			"is not one first-order lag everywhere",
			(double)(taus[0] * 1000.0f), (double)(taus[n_tau - 1u] * 1000.0f),
			(double)(taus[n_tau - 1u] / taus[0]));
	}

	k_mutex_lock(&g_config_mutex, K_FOREVER);
	target = g_config.target_power;
	k_mutex_unlock(&g_config_mutex);

	/* Local gain: the slope of the interval that brackets the target, searched
	 * only over the monotonic prefix. The plant gain varies across the throttle
	 * range, so the interval slope beats a global one. A target above the
	 * identified range falls back to the last invertible interval. */
	gain = g_ident_res.k[1];
	for (uint8_t i = 1; i < usable; i++) {
		gain = g_ident_res.k[i];
		if (target <= g_ident_res.p_settled[i]) {
			break;
		}
	}

	if (gain <= 0.0f) {
		/* A non-monotonic or noisy interval would otherwise throw away an
		 * otherwise good run. Fall back to the mean of the usable slopes. */
		float sum = 0.0f;
		uint8_t n = 0u;

		for (uint8_t i = 1; i < usable; i++) {
			if (g_ident_res.k[i] > 0.0f) {
				sum += g_ident_res.k[i];
				n++;
			}
		}
		if (n > 0u) {
			gain = sum / (float)n;
			LOG_WRN("ident: bracketing interval had no usable slope, "
				"using the mean of %u intervals (K=%.4f)",
				(unsigned int)n, (double)gain);
		}
	}

	g_ident_res.k_at_target = gain;

	if (gain <= 0.0f || tau_med <= 0.0f) {
		ident_finish(false, "Invalid gain or time constant.");
		return;
	}

	lambda = (g_ident.lambda_req > 0.0f) ? g_ident.lambda_req : tau_med;
	if (lambda < CONTROL_LAMBDA_MIN_S) {
		lambda = CONTROL_LAMBDA_MIN_S;
	} else if (lambda > CONTROL_LAMBDA_MAX_S) {
		lambda = CONTROL_LAMBDA_MAX_S;
	}

	b0 = gain / tau_med;               /* (W/s) per us */
	kp = 1.0f / lambda;                /* 1/s          */
	wo = 1.0f / sqrtf(lambda * tau_med); /* rad/s      */
	if (wo > ADRC_WO_MAX) {
		wo = ADRC_WO_MAX;
	}

	g_ident_res.tau_median = tau_med;
	g_ident_res.lambda = lambda;
	g_ident_res.b0 = b0;
	g_ident_res.kp = kp;
	g_ident_res.wo = wo;
	g_ident_res.v_ref = (g_ident.v_count > 0u)
				    ? (g_ident.v_sum / (float)g_ident.v_count) : 0.0f;
	g_ident_res.loop_dt = g_loop_dt_s;
	g_ident_res.valid = true;

	k_mutex_lock(&g_config_mutex, K_FOREVER);
	g_config.wo = wo;
	g_config.b0 = b0;
	g_config.kp = kp;
	g_config.kd = 0.0f;
	(void)adrc_tune(wo, b0, kp, 0.0f);
	(void)persistence_save_config(&g_config);
	k_mutex_unlock(&g_config_mutex);

	/* The observer was just reset to a safe-stop input, but the ESC is still
	 * sitting at the last test pulse. Tell it the truth before the loop runs
	 * the control law again, otherwise z1 gets a one-tick kick of
	 * b0 * (idle - last_level). */
	adrc_set_applied((float)ident_level_us[CONTROL_IDENT_LEVELS - 1]);

	g_learning_mode = false; /* identified gains replace the pass-through */
	g_learning_stage = LEARNING_STAGE_DONE;
	stream_meta_dirty = true;

	printf("[IDENT] tau=%.1f ms from %u/%u steps, K=%.4f W/us at target %.0f W, "
	       "lambda=%.0f ms (loop %.2f ms)\r\n",
	       (double)(tau_med * 1000.0f), (unsigned int)n_tau,
	       (unsigned int)(CONTROL_IDENT_LEVELS - 1u), (double)gain, (double)target,
	       (double)(lambda * 1000.0f), (double)(g_loop_dt_s * 1000.0f));
	printf("[IDENT] Applied: wo=%.1f rad/s, b0=%.4f (W/s)/us, kp=%.2f 1/s\r\n",
	       (double)wo, (double)b0, (double)kp);
	printf("[IDENT] Feedforward map armed for %.1f..%.1f W at Vref=%.2f V.\r\n",
	       (double)g_ident_res.p_settled[0],
	       (double)g_ident_res.p_settled[usable - 1u],
	       (double)g_ident_res.v_ref);
	LOG_INF("ident complete: tau=%.1f ms (n=%u/%u) K=%.4f lambda=%.3f wo=%.1f b0=%.4f "
		"kp=%.2f loop_dt=%.2f ms",
		(double)(tau_med * 1000.0f), (unsigned int)n_tau,
		(unsigned int)(CONTROL_IDENT_LEVELS - 1u), (double)gain, (double)lambda,
		(double)wo, (double)b0, (double)kp, (double)(g_loop_dt_s * 1000.0f));

	ident_finish(true, NULL);
}

/**
 * @brief Advance the identification state machine by one 1 kHz tick.
 *
 * @param pwm_in Pilot throttle pulse width in us (the arm/abort signal).
 * @param voltage Bus voltage in Volts.
 * @param power_w Measured power in Watts.
 * @param is_safe True while the control loop considers the input and battery valid.
 * @return true if the identification currently owns the ESC output.
 */
static bool control_ident_tick(int pwm_in, float voltage, float power_w, bool is_safe)
{
	uint32_t now_ms = (uint32_t)k_uptime_get();

	if (!g_ident_res.running) {
		return false;
	}

	g_ident.v_sum += voltage;
	g_ident.v_count++;

	if ((now_ms - g_ident.start_ms) > IDENT_TIMEOUT_MS) {
		ident_finish(false, "Timeout.");
		return false;
	}
	if (power_w > IDENT_POWER_CAP_W) {
		ident_finish(false, "Power cap exceeded.");
		return false;
	}
	if (g_ident.phase == IDENT_ARM) {
		/* Arming only observes: the output is still min(pwm_in, u). */
		if (!is_safe || pwm_in > IDENT_ARM_IDLE_US) {
			g_ident.arm_since_ms = 0u;
			return false;
		}
		if (g_ident.arm_since_ms == 0u) {
			g_ident.arm_since_ms = (now_ms != 0u) ? now_ms : 1u;
			return false;
		}
		if ((now_ms - g_ident.arm_since_ms) < IDENT_ARM_HOLD_MS) {
			return false;
		}

		g_ident.phase = IDENT_SETTLE;
		g_ident.idx = 0u;
		g_ident.pulse = ident_level_us[0];
		g_ident.step_us = ctrl_now_us();
		g_ident.settle_us = g_ident.step_us;
		g_ident.settle_sum = 0.0f;
		g_ident.settle_sumsq = 0.0f;
		g_ident.settle_avg_count = 0u;
		LOG_INF("ident: running, %u levels, loop period %.2f ms",
			(unsigned int)CONTROL_IDENT_LEVELS, (double)(g_loop_dt_s * 1000.0f));
		printf("[IDENT] Running: %u levels, motor under firmware control.\r\n",
		       (unsigned int)CONTROL_IDENT_LEVELS);
		return false;
	}

	/* From here the firmware drives the ESC. Any stick movement or unsafe
	 * condition is an abort: that is the operator's escape hatch. */
	if (!is_safe) {
		ident_finish(false, "Control loop became unsafe.");
		return false;
	}
	if (pwm_in > IDENT_ARM_IDLE_US) {
		ident_finish(false, "Stick moved (pilot took over).");
		return false;
	}

	switch (g_ident.phase) {
	case IDENT_SETTLE: {
		uint32_t elapsed = (uint32_t)(ctrl_now_us() - g_ident.settle_us);

		/* Only the tail of the settle window feeds the asymptote, so a plant
		 * slower than expected is not measured while still in transit. */
		if (elapsed >= (IDENT_SETTLE_US * 6u) / 10u) {
			g_ident.settle_sum += power_w;
			g_ident.settle_sumsq += power_w * power_w;
			g_ident.settle_avg_count++;
		}

		if (elapsed < IDENT_SETTLE_US) {
			break;
		}

		{
			uint16_t navg = (g_ident.settle_avg_count > 0u)
						? g_ident.settle_avg_count : 1u;
			float mean = g_ident.settle_sum / (float)navg;
			float var = g_ident.settle_sumsq / (float)navg - mean * mean;
			float noise = (var > 0.0f) ? sqrtf(var) : 0.0f;

			g_ident_res.p_settled[g_ident.idx] = mean;
			g_ident_res.noise_w[g_ident.idx] = noise;
			g_ident_res.u_us[g_ident.idx] = ident_level_us[g_ident.idx];

			if (g_ident.idx > 0u) {
				/* This level's settled power is now known, so the step into it can
				 * be fitted with a real asymptote. */
				g_ident_res.k[g_ident.idx] =
					(g_ident_res.p_settled[g_ident.idx] -
					 g_ident_res.p_settled[g_ident.idx - 1]) /
					(float)(ident_level_us[g_ident.idx] -
						ident_level_us[g_ident.idx - 1]);
				ident_fit(g_ident.idx, g_ident_res.p_settled[g_ident.idx - 1],
					  mean, noise);
			}
		}

		if (g_ident.idx + 1u >= CONTROL_IDENT_LEVELS) {
			ident_finalize();
			return false;
		}

		/* Step to the next level and capture its transient. */
		g_ident.idx++;
		g_ident.pulse = ident_level_us[g_ident.idx];
		g_ident.cap_count = 0u;
		g_ident.step_us = ctrl_now_us();
		g_ident.settle_sum = 0.0f;
		g_ident.settle_sumsq = 0.0f;
		g_ident.settle_avg_count = 0u;
		g_ident.phase = IDENT_CAPTURE;
		break;
	}

	case IDENT_CAPTURE: {
		uint32_t elapsed = (uint32_t)(ctrl_now_us() - g_ident.step_us);

		if (g_ident.cap_count < IDENT_CAPTURE_MAX) {
			g_ident.cap[g_ident.cap_count].pwr = power_w;
			g_ident.cap[g_ident.cap_count].t_us = elapsed;
			g_ident.cap_count++;
			ident_record(power_w, g_ident.cap_count == 1u);
		}

		/* Long enough to resolve a slow plant, and dense enough to resolve a
		 * fast one: the window is at least IDENT_CAPTURE_US of wall time AND at
		 * least IDENT_CAPTURE_TICKS samples, whichever takes longer. */
		if ((g_ident.cap_count >= IDENT_CAPTURE_MAX) ||
		    ((elapsed >= IDENT_CAPTURE_US) &&
		     (g_ident.cap_count >= IDENT_CAPTURE_TICKS))) {
			g_ident.phase = IDENT_SETTLE;
			g_ident.settle_us = ctrl_now_us();
			g_ident.settle_sum = 0.0f;
			g_ident.settle_sumsq = 0.0f;
			g_ident.settle_avg_count = 0u;
		}
		break;
	}

	default:
		break;
	}

	return true;
}

int control_ident_start(float lambda_s)
{
	uint32_t now_ms = (uint32_t)k_uptime_get();
	float per_level_s, est_s;

	if (g_ident_res.running) {
		return -EALREADY;
	}

	memset(&g_ident, 0, sizeof(g_ident));
	memset(&g_ident_res, 0, sizeof(g_ident_res));
	g_ident.lambda_req = lambda_s;
	g_ident.phase = IDENT_ARM;
	g_ident.start_ms = now_ms;
	g_ident_res.running = true;

	/* Quote the duration from the ACTUAL loop period: with the INA226 blocking
	 * the loop for ~2 ms per tick, a run takes several seconds longer than the
	 * 1 kHz nominal would suggest. */
	per_level_s = (float)IDENT_CAPTURE_TICKS * g_loop_dt_s;
	if (per_level_s < (float)IDENT_CAPTURE_US * 1e-6f) {
		per_level_s = (float)IDENT_CAPTURE_US * 1e-6f;
	}
	per_level_s += (float)IDENT_SETTLE_US * 1e-6f;
	est_s = per_level_s * (float)CONTROL_IDENT_LEVELS + 1.0f;

	LOG_INF("ident armed: stick idle for 1 s to take over (loop period %.2f ms)",
		(double)(g_loop_dt_s * 1000.0f));
	printf("[IDENT] Armed. LEAVE THE STICK AT IDLE: the firmware will take over the "
	       "throttle for ~%.0f s and drive the motor through %u levels.\r\n",
	       (double)est_s, (unsigned int)CONTROL_IDENT_LEVELS);
	printf("[IDENT] Move the stick at any time to abort.\r\n");

	return 0;
}

void control_ident_abort(void)
{
	if (g_ident_res.running) {
		ident_finish(false, "Aborted by operator.");
	}
}

bool control_ident_running(void)
{
	return g_ident_res.running;
}

const struct control_ident_result *control_ident_result(void)
{
	return &g_ident_res;
}

/* Semaphore limit of 1 drops ticks when an iteration overruns, preventing a catch-up cascade. */
static K_SEM_DEFINE(control_sem, 0, 1);

/* 1 kHz control tick source: periodic timer posts the control semaphore from ISR context. */
static void control_timer_expiry(struct k_timer *timer_id)
{
    k_sem_give(&control_sem);
}

K_TIMER_DEFINE(control_timer, control_timer_expiry, NULL);

/**
 * @brief Thread 1 entry point: hard real-time ADRC control loop running at 1 kHz.
 */
void control_thread_handler(void *p1, void *p2, void *p3)
{
    // Wait for control_init to complete.
    while (!g_initialized) {
        k_msleep(10);
    }

    LOG_INF("Control Thread (Thread 1) started at 1kHz");

    static float max_power = 0.0f;
    /* Last reference the control law ran with; cached so the binary stream can
     * report it even on ticks where the law is skipped (unsafe / learning). */
    static float target_power_snapshot = 0.0f;
    static uint32_t stream_ticks;

    target_power_snapshot = g_config.target_power;

    while (1) {
        // Block until the 1 kHz control timer expires.
        k_sem_take(&control_sem, K_FOREVER);

        // 0. Measure the real control period. The nominal CONTROL_PERIOD_S is
        //    only 1 ms, but the INA226 read below blocks for ~2 ms at 100 kHz
        //    I2C, and the 1 kHz timer only free-runs when an iteration is
        //    shorter than its period. The LESO integrates dt, so passing the
        //    nominal value while the loop runs at a third of that rate makes
        //    the observer (and the energy integral) wrong by the same factor.
        static uint64_t prev_ctrl_us;
        uint64_t now_ctrl_us = ctrl_now_us();
        float dt = (prev_ctrl_us != 0u)
                       ? (float)(now_ctrl_us - prev_ctrl_us) * 1e-6f
                       : CONTROL_PERIOD_S;

        prev_ctrl_us = now_ctrl_us;
        if (dt < 0.0002f || dt > 0.050f) {
            dt = CONTROL_PERIOD_S; // A stalled or flooded tick must not poison the observer.
        }
        g_loop_dt_s = dt;

        // One-shot report of the achieved rate: it is a property of the build
        // (sensor payload, I2C clock), not of the tuning, and every dt-dependent
        // number below inherits it.
        static uint32_t rate_n;
        static float rate_sum;
        if (rate_n < 2000u) {
            rate_sum += dt;
            if (++rate_n == 2000u) {
                float mean = rate_sum / 2000.0f;

                LOG_INF("control loop: %.0f Hz (%.2f ms mean period, %u ticks)",
                        (double)(1.0f / mean), (double)mean, (unsigned int)rate_n);
            }
        }

        // 1. Read the pilot throttle input pulse width in microseconds.
        int pwm_in = pwm_input_get_period();

        // 2. Sample the INA226 power sensor.
        float current_power = 0.0f;
        int sensor_err = ina226_sample(&current_power);

        // On sensor read error, force voltage/current to 0 to indicate no battery/sensor.
        if (sensor_err < 0) {
            g_ina226.current_mV = 0.0f;
            g_ina226.current_mA = 0.0f;
            current_power = 0.0f;
        } else if (current_power > max_power) {
            max_power = current_power;
        }

        float voltage = g_ina226.current_mV / 1000.0f;

        // 3. Determine if we are in a safe execution state.
        bool input_valid = (pwm_in >= PWM_INPUT_MIN_US && pwm_in <= PWM_INPUT_MAX_US);
        bool battery_valid = (voltage >= BATTERY_MIN_V);
        bool is_safe = input_valid && battery_valid;

        // Detect learning-mode transitions so buffers are reset in thread context.
        static bool learn_was_active;
        if (g_learning_mode != learn_was_active) {
            learn_was_active = g_learning_mode;
            if (g_learning_mode) {
                learning_begin();
                LOG_INF("b0 learning mode: hold throttle at 10-50%%, then slam to >75%% and hold.");
            }
        }

        // 4. Automatic identification: while it runs it owns the ESC output.
        //    It only takes over after the arming delay, so the pilot keeps the
        //    throttle until the stick has been idle for a second.
        bool ident_active = control_ident_tick(pwm_in, voltage, current_power, is_safe);

        // 5. Compute the control effort if safe.
        float command_val = PWM_SAFE_STOP_US; // Default safe-stop pulse.
        if (ident_active) {
            command_val = (float)g_ident.pulse;
        } else if (is_safe) {
            if (g_learning_mode) {
                // Learning: pass the pilot throttle through while collecting
                // samples, with a hard power cap as a safety net.
                if (!learn_power_cut && current_power > learn_power_cap) {
                    learn_power_cut = true;
                    printf("[LEARN] Power %.0f W exceeded the %.0f W cap. Cutting throttle; reduce throttle.\r\n",
                           (double)current_power, (double)learn_power_cap);
                } else if (learn_power_cut && current_power < 0.9f * learn_power_cap) {
                    learn_power_cut = false;
                    learning_restart("Power cap triggered.");
                }

                if (learn_power_cut) {
                    command_val = PWM_SAFE_STOP_US;
                } else {
                    command_val = (float)pwm_in;
                    learn_tick++;
                    learning_collect(pwm_in, current_power);
                }
            } else {
                k_mutex_lock(&g_config_mutex, K_FOREVER);

                target_power_snapshot = g_config.target_power;

                /* Inverse-model feedforward from the identified staircase:
                 * the throttle that produces the target at this bus voltage.
                 * Zero (feedback only) when no map is armed. */
                float u_ff = control_ff_throttle(target_power_snapshot, voltage);

                int adrc_err = adrc_update(target_power_snapshot, current_power,
                                           dt, u_ff, &command_val);
                if (adrc_err < 0) {
                    LOG_ERR("ADRC control law update failed: %d", adrc_err);
                    is_safe = false; // Treat ADRC failure as unsafe.
                }

                k_mutex_unlock(&g_config_mutex);
            }
        }

        // 6. Apply the power-limiter override: PWM_out = min(PWM_in, u_ctrl).
        //    The identification bypasses the min() because the whole point is
        //    to command above the pilot's idle stick.
        int pwm_out = PWM_SAFE_STOP_US;
        if (ident_active) {
            pwm_out = (int)command_val;
            if (esc_set_throttle((float)pwm_out) < 0) {
                is_safe = false;
            }
        } else if (is_safe) {
            int applied_val = (int)command_val;
            pwm_out = (pwm_in < applied_val) ? pwm_in : applied_val;

            /* esc_set_throttle() clamps the pulse to [PWM_MIN_US, PWM_MAX_US]
             * before driving the ESC, so mirror that clamp here. Otherwise the
             * telemetry and the LESO see a pulse the ESC never received (e.g.
             * pwm_out 850 us while the ESC got 1000 us), which biases z2. */
            if (pwm_out < PWM_MIN_US) {
                pwm_out = PWM_MIN_US;
            } else if (pwm_out > PWM_MAX_US) {
                pwm_out = PWM_MAX_US;
            }

            if (esc_set_throttle((float)pwm_out) < 0) {
                is_safe = false;
            }
        }

        // Write the safe shutdown pulse on any error condition.
        if (!is_safe) {
            esc_set_throttle((float)PWM_SAFE_STOP_US);
            pwm_out = PWM_SAFE_STOP_US;
        }

        // 5b. Feed the observer the pulse the plant actually received. The
        //     command above may have been cut by the pilot's PWM or by the
        //     actuator limits, and the LESO must integrate the real input,
        //     not the requested one.
        adrc_set_applied((float)pwm_out);

        // 6b. Integrate energy consumption: Joules = Power (W) * dt (s).
        float power_measurement = is_safe ? current_power : 0.0f;
        g_ina226.current_joules += power_measurement * dt;

        // 7. Evaluate the active control state (identification shows as LEARNING
        //    so the LED and the desk console report an autonomous run).
        enum ctrl_state state = control_ident_running()
                                    ? LEARNING
                                    : control_evaluate_state(input_valid, battery_valid,
                                                             pwm_in, pwm_out);

        // 8. Store results to the thread-safe telemetry structure.
        k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
        g_telemetry.power_w = max_power;
        g_telemetry.current_a = g_ina226.current_mA / 1000.0f;
        g_telemetry.voltage_v = voltage;
        g_telemetry.total_consumption_j = g_ina226.current_joules;
        g_telemetry.time_ms = k_uptime_get();
        g_telemetry.pwm_input_us = pwm_in;
        g_telemetry.pwm_output_us = pwm_out;
        g_telemetry.pwm_control_us = (int)command_val;
        g_telemetry.state = state;
        k_mutex_unlock(&g_telemetry_mutex);

        // 9. Feed the binary telemetry stream (second USB CDC ACM port).
        //    This is the only producer for src/stream.c; it is a couple of
        //    fixed-point conversions plus one ring_buf_put(), no formatting.
        if (stream_is_enabled()) {
            struct pm100_stream_sample smp;
            // The control law only runs on a safe tick outside learning and
            // outside identification; the observer states are stale otherwise.
            bool adrc_ran = is_safe && !g_learning_mode && !ident_active;
            bool saturated = adrc_ran && ((int)g_adrc.u_raw != (int)command_val);

            smp.t_ms = (uint32_t)g_telemetry.time_ms;
            smp.e_j = stream_i32(g_ina226.current_joules);
            // First-order law: z1 estimates power, z2 estimates the total
            // disturbance f. There is no dP/dt state, so z2_mws is 0 and the
            // disturbance travels in the z3_mws slot (see src/stream.h).
            smp.z1_mw = stream_i32(g_adrc.z1 * 1000.0f);
            smp.z2_mws = 0;
            smp.z3_mws = stream_i32(g_adrc.z2 * 1000.0f);
            smp.v_cv = stream_u16(g_ina226.current_mV * 0.1f);
            smp.i_ca = stream_i16(g_ina226.current_mA * 0.1f);
            smp.y_dw = stream_i16(current_power * 10.0f);
            smp.pmax_w = stream_u16(max_power);
            smp.tgt_dw = stream_i16(target_power_snapshot * 10.0f);
            smp.pwm_in = stream_u16((float)pwm_in);
            smp.pwm_out = stream_u16((float)pwm_out);
            smp.pwm_ctrl = stream_u16(command_val);
            smp.pwm_ctrl_raw = stream_i16(g_adrc.u_raw);
            smp.state = (uint8_t)state;
            smp.flags = (input_valid ? PM100_FLAG_INPUT_VALID : 0u) |
                        (battery_valid ? PM100_FLAG_BATTERY_VALID : 0u) |
                        (is_safe ? PM100_FLAG_SAFE : 0u) |
                        (g_learning_mode ? PM100_FLAG_LEARNING : 0u) |
                        (learn_power_cut ? PM100_FLAG_LEARN_POWER_CUT : 0u) |
                        (saturated ? PM100_FLAG_ADRC_SATURATED : 0u) |
                        ((sensor_err < 0) ? PM100_FLAG_SENSOR_ERROR : 0u);

            stream_push_sample(&smp);

            // Controller snapshot: once per second and right after any change.
            if (stream_meta_dirty || (++stream_ticks % STREAM_META_PERIOD_TICKS) == 0u) {
                stream_feed_meta();
                stream_meta_dirty = false;
            }
        }
    }
}

K_THREAD_DEFINE(control_thread, CONTROL_STACK_SIZE, control_thread_handler,
                NULL, NULL, NULL, CONTROL_PRIORITY, 0, 0);

/* ------------------------------------------------------------------------- */
/* Initialization and thread-safe configuration setters                      */
/* ------------------------------------------------------------------------- */

/**
 * @brief Initialize the hardware driver, sensor, and ADRC controller.
 *
 * @return int 0 on success, or negative error code on failure.
 */
int control_init(void)
{
    LOG_INF("Initializing hardware and control systems...");

    const struct device *const ina226_dev = DEVICE_DT_GET_ANY(ti_ina226);
    if (!ina226_dev) {
        LOG_ERR("Could not find ti,ina226 device in the device tree!");
        return -ENODEV;
    }

    if (!device_is_ready(ina226_dev)) {
        LOG_ERR("INA226 sensor device not ready");
        return -ENODEV;
    }

    int ret = pwm_input_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize PWM input (error %d)", ret);
        return -ENODEV;
    }

    if (!pwm_is_ready_dt(&esc_out)) {
        LOG_ERR("PWM output device not ready");
        return -ENODEV;
    }

    g_ina226.dev = ina226_dev;
    g_ina226.current_mV = 0.0f;
    g_ina226.current_mA = 0.0f;
    g_ina226.current_joules = 0.0f;

    // Initialize persistence and load device configuration from NVS/simulated storage.
    persistence_init();
    struct device_config loaded_cfg;
    if (persistence_load_config(&loaded_cfg) == 0) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        g_config = loaded_cfg;
        g_config.dt = CONTROL_PERIOD_S; // Keep ADRC dt in sync with the fixed 1 kHz loop.
        k_mutex_unlock(&g_config_mutex);
        LOG_INF("Stored device configuration successfully loaded.");
    } else {
        LOG_INF("No stored configuration found. Storing current defaults to storage...");
        persistence_save_config(&g_config);
    }

    /*
     * Migration guard for the first-order law.
     *
     * b0 is now the input gain K/tau, whereas the previous second-order model
     * identified b0 = K/tau^2, i.e. 1/tau (~30-50x for the plants seen so far)
     * larger. A value above ADRC_B0_MAX_PLAUSIBLE therefore cannot be valid for
     * this law: treat it as "not identified yet" and re-enter learning instead
     * of flying with a controller that is scaled wrong.
     */
    if (g_config.b0 > ADRC_B0_MAX_PLAUSIBLE) {
        LOG_WRN("Stored b0 %.1f is out of range for the first-order law "
                "(b0 = K/tau); forcing re-identification.",
                (double)g_config.b0);
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        g_config.b0 = 0.0f;
        k_mutex_unlock(&g_config_mutex);
    }

    /* The default target is 600 W, i.e. above anything this plant can produce,
     * so a freshly flashed device would silently run unlimited. Say so. */
    if (g_config.target_power > 500.0f) {
        LOG_WRN("power target is %.1f W, above what a plant of this size can reach: "
                "the limiter will never engage. Set it with 'pm100 target <W>'.",
                (double)g_config.target_power);
    }

    if (g_config.b0 == 0.0f) {
        g_learning_mode = true;
        LOG_INF("b0 = 0: ADRC learning mode enabled.");
    } else {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        ret = adrc_tune(g_config.wo, g_config.b0, g_config.kp, g_config.kd);
        k_mutex_unlock(&g_config_mutex);

        if (ret < 0) {
            LOG_ERR("Failed to tune ADRC controller (error %d)", ret);
            return ret;
        }
    }

    g_initialized = true;
    k_timer_start(&control_timer, K_MSEC(CONTROL_PERIOD_MS), K_MSEC(CONTROL_PERIOD_MS));
    LOG_INF("Hardware and control initialization successful.");

    return 0;
}

int control_update_gains(float wo, float b0, float kp, float kd)
{
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.wo = wo;
    g_config.b0 = b0;
    g_config.kp = kp;
    g_config.kd = kd;

    int ret;

    if (b0 == 0.0f) {
        // b0 = 0 triggers the b0 identification (learning) mode.
        g_learning_mode = true;
        ret = persistence_save_config(&g_config);
    } else {
        ret = adrc_tune(wo, b0, kp, kd);
        if (ret == 0) {
            /* Leave learning mode. The pass-through path has no power limiter
             * (only the 1.2x safety cap), so setting usable gains without
             * clearing this flag would tune the ADRC and then never run it. */
            g_learning_mode = false;
            g_learning_stage = LEARNING_STAGE_IDLE;
            persistence_save_config(&g_config);
            LOG_INF("ADRC tuned (wo=%.2f b0=%.4f kp=%.2f), learning pass-through cleared",
                    (double)wo, (double)b0, (double)kp);
        }
    }

    k_mutex_unlock(&g_config_mutex);

    stream_meta_dirty = true; // Let the binary stream re-publish the gains.

    return ret;
}

int control_update_target(float target_power)
{
    if (!isfinite(target_power) || target_power <= 0.0f ||
        target_power > CONTROL_TARGET_MAX_W) {
        LOG_ERR("power target %.1f W rejected (must be 0 < target <= %.0f W)",
                (double)target_power, (double)CONTROL_TARGET_MAX_W);
        return -EINVAL;
    }

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.target_power = target_power;
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);

    stream_meta_dirty = true;

    return ret;
}

int control_update_shunt(float shunt_resistor_mohm)
{
    if (shunt_resistor_mohm <= 0.0f) {
        return -EINVAL;
    }
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.shunt_resistor_mohm = shunt_resistor_mohm;
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);

    stream_meta_dirty = true;

    return ret;
}

/* Include persistence implementation for compilation in a single translation unit */
#include "persistence.c"
