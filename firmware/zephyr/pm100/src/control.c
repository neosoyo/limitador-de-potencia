#include "control.h"
#include "ina226.h"
#include "adrc_control_law.h"
#include "persistence.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(PM100_devel, LOG_LEVEL_DBG);

#define CONTROL_STACK_SIZE 2048
#define CONTROL_PRIORITY K_PRIO_COOP(1) // High-priority cooperative thread for precise 500Hz execution

/* Global shared structures */
struct system_telemetry g_telemetry = {
    .power_w = 0.0f,
    .current_a = 0.0f,
    .voltage_v = 0.0f,
    .total_consumption_j = 0.0f,
    .time_ms = 0,
    .pwm_input_us = 1000,
    .pwm_output_us = 1000,
    .pwm_control_us = 1000,
    .state = READY
};
K_MUTEX_DEFINE(g_telemetry_mutex);

struct device_config g_config = {
    .dt = 0.002f,             // 500Hz sampling period
    .wo = 100.0f,             // Observer bandwidth (rad/s)
    .b0 = 50.0f,              // Input gain scaling
    .kp = 400.0f,             // Proportional gain
    .kd = 40.0f,              // Derivative gain
    .target_power = 600.0f,   // Power target limit in Watts
    .shunt_resistor_mohm = 1.0f, // Default 1mOhm shunt
    .team_name = "time",
    .team_number = 0,
    .PIN_code = "123456"
};
K_MUTEX_DEFINE(g_config_mutex);

volatile bool g_stream_active = false;

/* Thread static controller and hardware driver instances */
static struct adrc_control_law g_adrc_cl;
static struct ina226 g_ina226_hw;
static volatile bool g_initialized = false;

/**
 * @brief Thread 1 entry point: hard real-time ADRC control loop running at 500Hz.
 */
void control_thread_handler(void *p1, void *p2, void *p3)
{
    // Wait for control_init to complete
    while (!g_initialized) {
        k_msleep(10);
    }

    LOG_INF("Control Thread (Thread 1) started at 500Hz");

    uint64_t next_wakeup = k_uptime_get();
    static float max_power = 0.0f;

    while (1) {
        // Enforce exact 2ms execution intervals (500Hz) to eliminate loop jitter/drift
        next_wakeup += 2;
        int64_t remaining_ms = (int64_t)(next_wakeup - k_uptime_get());
        if (remaining_ms > 0) {
            k_msleep((uint32_t)remaining_ms);
        } else {
            // Reset next_wakeup if we fell behind to prevent a catching-up cascade
            next_wakeup = k_uptime_get();
        }

        // 1. Read input PWM pulse width in microseconds
        int pwm_in = pwm_input_get_period();

        // 2. Sample INA226 power sensor
        float current_power = 0.0f;
        int sensor_err = motor_hardware_get_power(&g_ina226_hw.interface, &current_power);
        
        // If there's a sensor read error, force the current voltage to 0.0f to indicate no battery/sensor
        if (sensor_err < 0) {
            g_ina226_hw.current_mV = 0.0f;
            g_ina226_hw.current_mA = 0.0f;
            current_power = 0.0f;
        } else {
            if (current_power > max_power) {
                max_power = current_power;
            }
        }

        float voltage = g_ina226_hw.current_mV / 1000.0f;

        // 3. Determine if we are in a safe execution state (input is valid and voltage is sufficient)
        bool input_valid = (pwm_in >= 900 && pwm_in <= 2000);
        bool battery_valid = (voltage >= 5.0f);
        bool is_safe = input_valid && battery_valid;

        // 4. Calculate control effort using ADRC if safe
        float command_val = 1000.0f; // Default safe stop pulse
        if (is_safe) {
            k_mutex_lock(&g_config_mutex, K_FOREVER);
            
            // Update reference target and measurement in ADRC interface
            control_law_set(&g_adrc_cl.interface, g_config.target_power, current_power);
            
            // Execute state observer and compute control output
            int adrc_err = control_law_update(&g_adrc_cl.interface, g_config.dt, &command_val);
            if (adrc_err < 0) {
                LOG_ERR("ADRC control law update failed: %d", adrc_err);
                is_safe = false; // treat ADRC failure as unsafe
            }
            
            k_mutex_unlock(&g_config_mutex);
        }

        // 5. Apply the power-limiter override logic and update output PWM
        int pwm_out = 1000;
        if (is_safe) {
            // Send command_val (ADRC control output) to the ESC driver.
            int throttle_err = motor_hardware_set_throtle(&g_ina226_hw.interface, command_val);
            if (throttle_err < 0) {
                is_safe = false;
                pwm_out = 1000;
            } else {
                // Determine applied output PWM based on whether controller or pilot was lower
                int applied_val = (int)command_val;
                if (pwm_in < applied_val) {
                    pwm_out = pwm_in;
                } else {
                    pwm_out = applied_val;
                }
            }
        }

        if (!is_safe) {
            // Write safe shutdown pulse on Error
            motor_hardware_set_throtle(&g_ina226_hw.interface, 1000.0f);
            pwm_out = 1000;
        }

        // 6. Integrator to track energy consumption: Joules = Power (W) * dt (s)
        float power_measurement = is_safe ? current_power : 0.0f;
        g_ina226_hw.current_joules += power_measurement * 0.002f;

        // 7. Store results to the thread-safe telemetry structure
        k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
        g_telemetry.power_w = max_power;
        g_telemetry.current_a = g_ina226_hw.current_mA / 1000.0f;
        g_telemetry.voltage_v = voltage;
        g_telemetry.total_consumption_j = g_ina226_hw.current_joules;
        g_telemetry.time_ms = k_uptime_get();
        g_telemetry.pwm_input_us = pwm_in;
        g_telemetry.pwm_output_us = pwm_out;
        g_telemetry.pwm_control_us = (int)command_val;
        k_mutex_unlock(&g_telemetry_mutex);
    }
}

K_THREAD_DEFINE(control_thread, CONTROL_STACK_SIZE, control_thread_handler, NULL, NULL, NULL, CONTROL_PRIORITY, 0, 0);

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

    int ret = ina226_init(&g_ina226_hw, ina226_dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize INA226 driver (error %d)", ret);
        return ret;
    }

    // Initialize persistence and load device configuration from NVS/simulated storage
    persistence_init();
    struct device_config loaded_cfg;
    if (persistence_load_config(&loaded_cfg) == 0) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        g_config = loaded_cfg;
        k_mutex_unlock(&g_config_mutex);
        LOG_INF("Stored device configuration successfully loaded.");
    } else {
        LOG_INF("No stored configuration found. Storing current defaults to storage...");
        persistence_save_config(&g_config);
    }

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    ret = adrc_control_law_tune(&g_adrc_cl, g_config.dt, g_config.wo, g_config.b0, g_config.kp, g_config.kd);
    k_mutex_unlock(&g_config_mutex);

    if (ret < 0) {
        LOG_ERR("Failed to tune ADRC controller (error %d)", ret);
        return ret;
    }

    g_initialized = true;
    LOG_INF("Hardware and control initialization successful.");

    return 0;
}

/* Thread-safe set functions for configuration values */

int control_update_gains(float dt, float wo, float b0, float kp, float kd)
{
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.dt = dt;
    g_config.wo = wo;
    g_config.b0 = b0;
    g_config.kp = kp;
    g_config.kd = kd;
    
    int ret = adrc_control_law_tune(&g_adrc_cl, dt, wo, b0, kp, kd);
    if (ret == 0) {
        persistence_save_config(&g_config);
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

/* Include auxiliary .c files for compilation in single translation unit */
#include "persistence.c"
#include "adrc_control_law.c"
#include "ina226.c"
