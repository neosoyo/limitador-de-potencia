/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/led_strip.h>
#include "control.h"
#include "shell_cmds.h"
#include "ble_telemetry.h"

#define LED0_NODE DT_ALIAS(led0)
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define STRIP_NODE DT_ALIAS(rgba)

LOG_MODULE_REGISTER(PM100_devel, LOG_LEVEL_DBG);

/* Getting peripherals from device tree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

/* Thread definitions */
#define TELEM_THREAD_STACK_SIZE 2048
#define TELEM_THREAD_PRIORITY   K_PRIO_PREEMPT(8)

#define LED_THREAD_STACK_SIZE   1024
#define LED_THREAD_PRIORITY     K_PRIO_PREEMPT(9)

volatile int64_t g_blink_start_time = 0;

/**
 * @brief Thread 2 entry point: Updates and transmits telemetry diagnostics at 10Hz.
 * 
 * Drives the CSV formatting of active states and outputs to the console port
 * if stream is enabled.
 */
void telem_thread_handler(void *p1, void *p2, void *p3)
{
    // Wait for serial interface & initialization to settle
    k_msleep(2500);

    LOG_INF("Telemetry Thread (Thread 2) started at 10Hz");

    while (1) {
        k_msleep(100); // 10Hz period

        struct system_telemetry local_telem;

        // Fetch current telemetry thread-safely
        k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
        local_telem = g_telemetry;
        k_mutex_unlock(&g_telemetry_mutex);

        // Print CSV data stream if toggle flag is active
        if (g_stream_active) {
            printf("%llu,%0.2f,%0.2f,%0.2f,%0.2f,%d,%d,%d,%d\r\n",
                   local_telem.time_ms,
                   local_telem.power_w,
                   local_telem.current_a,
                   local_telem.voltage_v,
                   local_telem.total_consumption_j,
                   local_telem.pwm_input_us,
                   local_telem.pwm_output_us,
                   local_telem.pwm_control_us,
                   (int)local_telem.state);
        }

        // Push binary telemetry updates over Bluetooth BLE at 10Hz
        ble_telemetry_update(&local_telem);
    }
}

K_THREAD_DEFINE(telem_thread, TELEM_THREAD_STACK_SIZE, telem_thread_handler, NULL, NULL, NULL, TELEM_THREAD_PRIORITY, 0, 0);

/**
 * @brief Thread 3 entry point: Controls status WS2812 RGB LED signaling at 4Hz.
 * 
 * Reads the controller's current state and updates the blinking pattern dynamically.
 */
void led_thread_handler(void *p1, void *p2, void *p3)
{
    // Wait for hardware systems to initialize
    k_msleep(3000);

    LOG_INF("LED Status Thread (Thread 3) started");

    bool toggle_state = false;
    while (1) {
        // Retrieve control state computed by the 1 kHz control loop
        k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
        enum ctrl_state state = g_telemetry.state;
        k_mutex_unlock(&g_telemetry_mutex);

        int sleep_ms = 250;
        struct led_rgb color = {0};

        toggle_state = !toggle_state;

        if (state == BLINK) {
            // Blink LED white at 5Hz (100ms ON, 100ms OFF)
            if (toggle_state) {
                color.r = 0x20;
                color.g = 0x20;
                color.b = 0x20;
            } else {
                color.r = 0;
                color.g = 0;
                color.b = 0;
            }
            sleep_ms = 100; // 5Hz blink period per toggle (100ms)
        } else {
            if (toggle_state) {
                switch (state) {
                case READY:
                    color.g = 0x20; // Green
                    sleep_ms = 500; // 1Hz blink (500ms ON)
                    break;
                case LIMITING_POWER:
                    color.b = 0x20; // Blue
                    sleep_ms = 125; // 4Hz blink (125ms ON)
                    break;
                case ERROR_NO_INPUT:
                case ERROR_NO_BATTERY:
                    color.r = 0x20; // Red
                    sleep_ms = 250; // 2Hz blink (250ms ON)
                    break;
                case LEARNING:
                    color.r = 0x20; // Orange
                    color.g = 0x0A;
                    sleep_ms = 125; // 4Hz blink (125ms ON)
                    break;
                }
            } else {
                switch (state) {
                case READY:
                    sleep_ms = 500; // 1Hz blink (500ms OFF)
                    break;
                case LIMITING_POWER:
                    sleep_ms = 125; // 4Hz blink (125ms OFF)
                    break;
                case ERROR_NO_INPUT:
                case ERROR_NO_BATTERY:
                    sleep_ms = 250; // 2Hz blink (250ms OFF)
                    break;
                case LEARNING:
                    sleep_ms = 125; // 4Hz blink (125ms OFF)
                    break;
                }
            }
        }

        // Output to WS2812 RGB strip (if ready)
        if (device_is_ready(strip)) {
            led_strip_update_rgb(strip, &color, STRIP_NUM_PIXELS);
        }

        // Toggle standard on-board GPIO LED as a heartbeat backup
        if (gpio_is_ready_dt(&led)) {
            gpio_pin_set_dt(&led, toggle_state ? 1 : 0);
        }

        k_msleep(sleep_ms);
    }
}

K_THREAD_DEFINE(led_thread, LED_THREAD_STACK_SIZE, led_thread_handler, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);


int main(void)
{
    // Allow USB CDC ACM interface time to initialize
    k_msleep(2000);

    LOG_INF("Starting PM100 Initialization...");

    int ret;

    LOG_INF("Calling control_init()...");
    ret = control_init();
    if (ret < 0) {
        LOG_ERR("control_init failed with %d", ret);
    } else {
        LOG_INF("control_init success.");
    }

    LOG_INF("Calling ble_telemetry_init()...");
    ret = ble_telemetry_init();
    if (ret < 0) {
        LOG_ERR("ble_telemetry_init failed with %d", ret);
    } else {
        LOG_INF("ble_telemetry_init success.");
    }

    LOG_INF("Checking LED Pin...");
    if (gpio_is_ready_dt(&led)) {
        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure LED pin: %d", ret);
        }
    }

    LOG_INF("Checking ADC channel...");
    if (adc_is_ready_dt(&adc_channel)) {
        ret = adc_channel_setup_dt(&adc_channel);
        if (ret < 0) {
            LOG_ERR("Could not setup ADC channel: %d", ret);
        }
    }

    LOG_INF("Initialization Complete. Main thread idle.");

    while (1) {
        k_msleep(5000);
    }

    return 0;
}

// SHELL COMMANDS IMPLEMENTATION

static int cmd_update_adrc_gains(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active ADRC Gains: dt=%0.4f (fixed), w0=%0.1f, b0=%0.2f, kp=%0.2f, kd=%0.2f",
                    g_config.dt, g_config.wo, g_config.b0, g_config.kp, g_config.kd);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 5) {
        float wo = strtof(argv[1], NULL);
        float b0 = strtof(argv[2], NULL);
        float kp = strtof(argv[3], NULL);
        float kd = strtof(argv[4], NULL);

        int ret = control_update_gains(wo, b0, kp, kd);
        if (ret < 0) {
            shell_error(sh, "Failed to update ADRC gains: %d", ret);
            return ret;
        }

        if (b0 == 0.0f) {
            shell_print(sh, "b0 = 0: learning mode enabled. Hold throttle at 10-50%%, then slam to >75%% and hold.");
        } else {
            shell_print(sh, "ADRC gains successfully updated and stored in NVS.");
        }
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'pm100 gains' or 'pm100 gains <wo> <b0> <kp> <kd>'");
        return -EINVAL;
    }
}

static int cmd_target(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active Power Target: %0.2f W", g_config.target_power);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 2) {
        float target_power = strtof(argv[1], NULL);

        int ret = control_update_target(target_power);
        if (ret < 0) {
            shell_error(sh, "Failed to update target power: %d", ret);
            return ret;
        }

        shell_print(sh, "Target power set to %0.2f W and stored in NVS.", target_power);
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'pm100 target' or 'pm100 target <power_watts>'");
        return -EINVAL;
    }
}

static int cmd_shunt(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active Shunt Resistor: %0.2f mOhm", g_config.shunt_resistor_mohm);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 2) {
        float shunt_mohm = strtof(argv[1], NULL);

        int ret = control_update_shunt(shunt_mohm);
        if (ret < 0) {
            shell_error(sh, "Failed to update shunt resistor: %d. Must be > 0.0", ret);
            return ret;
        }

        shell_print(sh, "Shunt resistor successfully updated to %0.2f mOhm and stored in NVS.", shunt_mohm);
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'pm100 shunt' or 'pm100 shunt <value_mohm>'");
        return -EINVAL;
    }
}

static int cmd_team_name(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active Team Name: %s", g_config.team_name);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 2) {
        const char *name = argv[1];

        int ret = shell_update_team_name(name);
        if (ret < 0) {
            shell_error(sh, "Failed to set team name: %d", ret);
            return ret;
        }

        shell_print(sh, "Team name successfully updated and stored in NVS.");
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'team_name' or 'team_name <name>'");
        return -EINVAL;
    }
}

static int cmd_team_number(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active Team Number: %u", g_config.team_number);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 2) {
        uint32_t number = (uint32_t)strtoul(argv[1], NULL, 10);

        int ret = shell_update_team_number(number);
        if (ret < 0) {
            shell_error(sh, "Failed to set team number: %d", ret);
            return ret;
        }

        shell_print(sh, "Team number successfully updated and stored in NVS.");
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'team_number' or 'team_number <number>'");
        return -EINVAL;
    }
}

static int cmd_set_pin(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        shell_print(sh, "Active BLE Pairing PIN: %s", g_config.PIN_code);
        k_mutex_unlock(&g_config_mutex);
        return 0;
    } else if (argc == 2) {
        const char *pin = argv[1];

        int ret = shell_update_pin(pin);
        if (ret < 0) {
            shell_error(sh, "Failed to set PIN code: %d. Must be exactly 6 characters.", ret);
            return ret;
        }

        shell_print(sh, "PIN code successfully updated and stored in NVS.");
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'ble_pin' or 'ble_pin <pin_code>'");
        return -EINVAL;
    }
}

static int cmd_stream(const struct shell *sh, size_t argc, char **argv)
{
    if (strcmp(argv[1], "on") == 0) {
        g_stream_active = true;
        shell_print(sh, "Telemetry stream enabled.");
    } else if (strcmp(argv[1], "off") == 0) {
        g_stream_active = false;
        shell_print(sh, "Telemetry stream disabled.");
    } else {
        shell_error(sh, "Invalid argument. Use 'on' or 'off'.");
        return -EINVAL;
    }
    return 0;
}

static int cmd_readings(const struct shell *sh, size_t argc, char **argv)
{
    k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
    shell_print(sh, "%llu,%0.2f,%0.2f,%0.2f,%0.2f,%d,%d,%d,%d",
                g_telemetry.time_ms,
                g_telemetry.power_w,
                g_telemetry.current_a,
                g_telemetry.voltage_v,
                g_telemetry.total_consumption_j,
                g_telemetry.pwm_input_us,
                g_telemetry.pwm_output_us,
                g_telemetry.pwm_control_us,
                (int)g_telemetry.state);
    k_mutex_unlock(&g_telemetry_mutex);
    return 0;
}

static int cmd_learn(const struct shell *sh, size_t argc, char **argv)
{
    switch (g_learning_stage) {
    case LEARNING_STAGE_IDLE:
        shell_print(sh, "Learning is not active.");
        shell_print(sh, "Start it with: pm100 gains <wo> 0 <kp> <kd>");
        break;
    case LEARNING_STAGE_LOW_BAND:
        shell_print(sh, "Learning: hold throttle between 10%% and 50%% (1100-1500 us).");
        break;
    case LEARNING_STAGE_STEP_HIGH:
        shell_print(sh, "Learning: slam throttle above 75%% (1750-2000 us) and hold.");
        break;
    case LEARNING_STAGE_STEP_LOW:
        shell_print(sh, "Learning: return throttle to 10%%-50%% and hold.");
        break;
    case LEARNING_STAGE_ESTIMATING:
        shell_print(sh, "Learning: estimating parameters from the captured steps...");
        break;
    case LEARNING_STAGE_DONE: {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        float wo = g_config.wo;
        float b0 = g_config.b0;
        float kp = g_config.kp;
        float kd = g_config.kd;
        k_mutex_unlock(&g_config_mutex);
        shell_print(sh, "Learning complete: wo=%.2f, b0=%.4f, kp=%.2f, kd=%.2f", wo, b0, kp, kd);
        shell_print(sh, "To relearn: pm100 gains %.2f 0 %.2f %.2f", wo, kp, kd);
        break;
    }
    default:
        shell_print(sh, "Unknown learning stage.");
        break;
    }
    return 0;
}

static int cmd_blink(const struct shell *sh, size_t argc, char **argv)
{
    g_blink_start_time = k_uptime_get();
    shell_print(sh, "Forcing solid white LED for 5 seconds...");
    return 0;
}

/* Define master pm100 subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(pm100_subcmds,
    SHELL_CMD_ARG(gains, NULL, "Get active gains, or set: <wo> <b0> <kp> <kd>", cmd_update_adrc_gains, 1, 4),
    SHELL_CMD_ARG(learn, NULL, "Show b0 learning instructions/status", cmd_learn, 1, 0),
    SHELL_CMD_ARG(target, NULL, "Get active power target, or set: <power_watts>", cmd_target, 1, 1),
    SHELL_CMD_ARG(shunt, NULL, "Get active shunt resistor (mOhm), or set: <value_mohm>", cmd_shunt, 1, 1),
    SHELL_CMD_ARG(sample, NULL, "Print current telemetry CSV readings once", cmd_readings, 1, 0),
    SHELL_SUBCMD_SET_END
);

/* Register master pm100 command */
SHELL_CMD_REGISTER(pm100, &pm100_subcmds, "PM100 Power Limiter System Control Commands", NULL);

/* Standalone global commands */
SHELL_CMD_ARG_REGISTER(team_name, NULL, "Get active team name, or set: <name>", cmd_team_name, 1, 1);
SHELL_CMD_ARG_REGISTER(team_number, NULL, "Get active team number, or set: <number>", cmd_team_number, 1, 1);
SHELL_CMD_ARG_REGISTER(ble_pin, NULL, "Get active pairing PIN, or set: <pin_code> (6 characters)", cmd_set_pin, 1, 1);
SHELL_CMD_ARG_REGISTER(stream, NULL, "Toggle default port CSV stream <on|off>", cmd_stream, 2, 0);
SHELL_CMD_ARG_REGISTER(blink, NULL, "Trigger 10x white LED blinks", cmd_blink, 1, 0);

/* Include auxiliary .c files for compilation in single translation unit */
#include "shell_cmds.c"
#include "ble_telemetry.c"
