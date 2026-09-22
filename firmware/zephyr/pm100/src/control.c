#include "control.h"
#include "persistence.h"

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
    .wo = 100.0f,               // Observer bandwidth (rad/s); overwritten by b0 learning
    .b0 = 0.0f,                 // 0 = enter b0/ADRC learning mode on first boot
    .kp = 400.0f,               // Proportional gain; overwritten by b0 learning
    .kd = 40.0f,                // Derivative gain; overwritten by b0 learning
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
/* ADRC control law                                                          */
/* ------------------------------------------------------------------------- */

static struct {
    float b0;           // Input gain scaling
    float kp;           // Proportional gain
    float kd;           // Derivative gain
    float l1, l2, l3;   // Linear Extended State Observer (LESO) gains
    float z1, z2, z3;   // Estimated states: power, power derivative, total disturbance
    float u_prev;       // Previous control effort (PWM us)
} g_adrc;

/**
 * @brief Reset the ADRC observer and control states.
 *
 * @return int 0 on success.
 */
static int adrc_reset(void)
{
    g_adrc.z1 = 0.0f;
    g_adrc.z2 = 0.0f;
    g_adrc.z3 = 0.0f;
    g_adrc.u_prev = 0.0f;

    return 0;
}

/**
 * @brief Tune and reset the ADRC controller.
 *
 * Computes bandwidth-parameterized continuous-time LESO gains:
 * l1 = 3*wo, l2 = 3*wo^2, l3 = wo^3.
 *
 * @param wo Observer bandwidth (rad/s).
 * @param b0 Controller input gain scaling factor.
 * @param kp Proportional gain.
 * @param kd Derivative gain.
 * @return int 0 on success.
 */
static int adrc_tune(float wo, float b0, float kp, float kd)
{
    g_adrc.b0 = b0;
    g_adrc.kp = kp;
    g_adrc.kd = kd;
    g_adrc.l1 = 3.0f * wo;
    g_adrc.l2 = 3.0f * wo * wo;
    g_adrc.l3 = wo * wo * wo;

    return adrc_reset();
}

/**
 * @brief Advance the ADRC observer and compute the control effort.
 *
 * Performs Euler integration of the 3rd-order LESO and computes the control
 * command, with anti-windup clamping to the physical ESC bounds [1000, 2000] us.
 *
 * @param target_power Desired power limit in Watts.
 * @param measured_power Measured power in Watts.
 * @param dt Sampling time step in seconds.
 * @param command Pointer to store the computed control effort (PWM us).
 * @return int 0 on success, or -EINVAL if command is NULL.
 */
static int adrc_update(float target_power, float measured_power, float dt, float *command)
{
    if (!command) {
        return -EINVAL;
    }

    float z1 = g_adrc.z1;
    float z2 = g_adrc.z2;
    float z3 = g_adrc.z3;
    float u_prev = g_adrc.u_prev;
    float b0 = g_adrc.b0;

    // 1. LESO observer dynamics (continuous Euler integration).
    float z1_dot = -g_adrc.l1 * z1 + z2 + g_adrc.l1 * measured_power;
    float z2_dot = -g_adrc.l2 * z1 + z3 + g_adrc.l2 * measured_power + b0 * u_prev;
    float z3_dot = -g_adrc.l3 * z1 + g_adrc.l3 * measured_power;

    // 2. Update the estimated states.
    z1 += z1_dot * dt;
    z2 += z2_dot * dt;
    z3 += z3_dot * dt;

    // 3. Compute the control effort command (target PWM pulse in microseconds).
    float u = (g_adrc.kp * (target_power - z1) - g_adrc.kd * z2 - z3) / b0;

    // 4. Anti-windup clamping to physical ESC boundaries.
    if (u > PWM_MAX_US) {
        u = PWM_MAX_US;
    } else if (u < PWM_MIN_US) {
        u = PWM_MIN_US;
    }

    // Save state back.
    g_adrc.z1 = z1;
    g_adrc.z2 = z2;
    g_adrc.z3 = z3;
    g_adrc.u_prev = u;

    *command = u;

    return 0;
}

/* ------------------------------------------------------------------------- */
/* b0 learning mode                                                          */
/* ------------------------------------------------------------------------- */

/* PWM throttle bands used for b0 identification (percent of 1000-2000 us):
 * - Low band:  10% (1100 us) to 50% (1500 us)
 * - High band: 75% (1750 us) to 100% (2000 us)
 */
#define LEARNING_SAMPLES         256
#define LEARNING_STEPS_REQUIRED  3
#define LEARNING_PWM_LOW_MIN     (PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) / 10)
#define LEARNING_PWM_LOW_MAX     (PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) / 2)
#define LEARNING_PWM_HIGH_MIN    (PWM_MIN_US + 3 * (PWM_MAX_US - PWM_MIN_US) / 4)
#define LEARNING_PWM_HIGH_MAX    PWM_MAX_US
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
            printf("[LEARN] Step up detected. Hold throttle above 75%% (1750-2000 us).\r\n");
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

    float learned_b0 = dp / (tau * tau * du);
    if (learned_b0 < LEARNING_B0_MIN || learned_b0 > LEARNING_B0_MAX) {
        LOG_WRN("b0 learning: learned b0 %.4f out of range.", (double)learned_b0);
        learning_restart("Learned b0 out of range.");
        return;
    }

    float learned_wo = 5.0f * wc;
    float learned_kp = wc * wc;
    float learned_kd = 2.0f * wc;

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
    printf("[LEARN] Done: tau=%.3f s, wc=%.1f, wo=%.1f, b0=%.4f, kp=%.2f, kd=%.2f\r\n",
           (double)tau, (double)wc, (double)learned_wo, (double)learned_b0,
           (double)learned_kp, (double)learned_kd);
    printf("[LEARN] Parameters saved to NVS. Normal power limiting resumed.\r\n");
    LOG_INF("b0 learning complete: tau=%.3f s, wc=%.1f, wo=%.1f, b0=%.4f, kp=%.2f, kd=%.2f",
            (double)tau, (double)wc, (double)learned_wo, (double)learned_b0,
            (double)learned_kp, (double)learned_kd);
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
    if ((int64_t)(k_uptime_get() - g_blink_start_time) < 5000) {
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

    while (1) {
        // Block until the 1 kHz control timer expires.
        k_sem_take(&control_sem, K_FOREVER);

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

        // 4. Compute the control effort if safe.
        float command_val = PWM_SAFE_STOP_US; // Default safe-stop pulse.
        if (is_safe) {
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

                int adrc_err = adrc_update(g_config.target_power, current_power,
                                           CONTROL_PERIOD_S, &command_val);
                if (adrc_err < 0) {
                    LOG_ERR("ADRC control law update failed: %d", adrc_err);
                    is_safe = false; // Treat ADRC failure as unsafe.
                }

                k_mutex_unlock(&g_config_mutex);
            }
        }

        // 5. Apply the power-limiter override logic: PWM_out = min(PWM_in, u_ctrl).
        int pwm_out = PWM_SAFE_STOP_US;
        if (is_safe) {
            int applied_val = (int)command_val;
            pwm_out = (pwm_in < applied_val) ? pwm_in : applied_val;

            if (esc_set_throttle((float)pwm_out) < 0) {
                is_safe = false;
            }
        }

        // Write the safe shutdown pulse on any error condition.
        if (!is_safe) {
            esc_set_throttle((float)PWM_SAFE_STOP_US);
            pwm_out = PWM_SAFE_STOP_US;
        }

        // 6. Integrate energy consumption: Joules = Power (W) * dt (s).
        float power_measurement = is_safe ? current_power : 0.0f;
        g_ina226.current_joules += power_measurement * CONTROL_PERIOD_S;

        // 7. Evaluate the active control state.
        enum ctrl_state state = control_evaluate_state(input_valid, battery_valid,
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
            persistence_save_config(&g_config);
        }
    }

    k_mutex_unlock(&g_config_mutex);
    return ret;
}

int control_update_target(float target_power)
{
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.target_power = target_power;
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);
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
    return ret;
}

/* Include persistence implementation for compilation in a single translation unit */
#include "persistence.c"
