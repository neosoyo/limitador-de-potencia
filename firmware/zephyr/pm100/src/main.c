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
#include "stream.h"

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

/*
 * Timestamp of the last CLI/BLE white-blink trigger (BLINK_HOLD_MS in control.h
 * sets how long it overrides the state colour). Initialised to a time in the
 * past (not 0) so that the "k_uptime_get() - g_blink_start_time < BLINK_HOLD_MS"
 * test is FALSE at boot: with 0, the first five seconds of every power-up
 * counted as BLINK and the LED was white before it settled into its real state
 * colour.
 */
volatile int64_t g_blink_start_time = -100000;

/*
 * Status LED colour palette (projeto.md, section 1 "Thread 3" state table).
 * Full-scale RGB values; the ws2812-spi driver maps the logical channels to the
 * strip's wire order using "color-mapping" in the devicetree (GREEN, RED, BLUE).
 */
#define LED_COLOR_OFF    ((struct led_rgb){ .r = 0x00, .g = 0x00, .b = 0x00 }) /* black                     */
#define LED_COLOR_GREEN  ((struct led_rgb){ .r = 0x00, .g = 0xFF, .b = 0x00 }) /* #00FF00 -> READY          */
#define LED_COLOR_BLUE   ((struct led_rgb){ .r = 0x00, .g = 0x00, .b = 0xFF }) /* #0000FF -> LIMITING_POWER */
#define LED_COLOR_RED    ((struct led_rgb){ .r = 0xFF, .g = 0x00, .b = 0x00 }) /* #FF0000 -> ERROR_*        */
#define LED_COLOR_WHITE  ((struct led_rgb){ .r = 0xFF, .g = 0xFF, .b = 0xFF }) /* #FFFFFF -> BLINK          */
#define LED_COLOR_ORANGE ((struct led_rgb){ .r = 0xFF, .g = 0x7F, .b = 0x00 }) /* #FF7F00 -> LEARNING       */

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

    /* One-shot readiness report: without it a missing/unready strip is silent,
     * and a WS2812B sits at its power-on state (usually solid white), which is
     * indistinguishable from a broken colour implementation. */
    if (device_is_ready(strip)) {
        LOG_INF("WS2812 strip %s ready, %u pixel(s)", DEVICE_DT_NAME(STRIP_NODE),
                (unsigned int)STRIP_NUM_PIXELS);
    } else {
        LOG_ERR("WS2812 strip %s NOT ready: colour output disabled (check the "
                "SPI bus and the worldsemi,ws2812-spi node)",
                DEVICE_DT_NAME(STRIP_NODE));
    }

    bool toggle_state = false;
    while (1) {
        // Retrieve control state computed by the 1 kHz control loop
        k_mutex_lock(&g_telemetry_mutex, K_FOREVER);
        enum ctrl_state state = g_telemetry.state;
        k_mutex_unlock(&g_telemetry_mutex);

        int sleep_ms = 250;
        struct led_rgb color = LED_COLOR_OFF;

        toggle_state = !toggle_state;

        if (state == BLINK) {
            // White, 5Hz (100ms ON, 100ms OFF), triggered via CLI/BLE
            if (toggle_state) {
                color = LED_COLOR_WHITE;
            }
            sleep_ms = 100; // 5Hz blink period per toggle (100ms)
        } else {
            if (toggle_state) {
                switch (state) {
                case READY:
                    color = LED_COLOR_GREEN;
                    sleep_ms = 500; // 1Hz blink (500ms ON)
                    break;
                case LIMITING_POWER:
                    color = LED_COLOR_BLUE;
                    sleep_ms = 125; // 4Hz blink (125ms ON)
                    break;
                case ERROR_NO_INPUT:
                case ERROR_NO_BATTERY:
                    color = LED_COLOR_RED;
                    sleep_ms = 250; // 2Hz blink (250ms ON)
                    break;
                case LEARNING:
                    color = LED_COLOR_ORANGE;
                    sleep_ms = 125; // 4Hz blink (125ms ON)
                    break;
                default:
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
                default:
                    break;
                }
            }
        }

        // Output to WS2812 RGB strip (if ready). The return value used to be
        // dropped, so an SPI failure was invisible and the strip simply kept
        // showing its previous (power-on) colour.
        //
        // led_strip_update_rgb() expects an ARRAY of STRIP_NUM_PIXELS pixels.
        // Passing the address of a single struct only worked because
        // chain-length == 1; with a longer chain it read past the end of a
        // stack object. Broadcasting through a static array also keeps it off
        // the 1 KiB LED thread stack.
        if (device_is_ready(strip)) {
            static struct led_rgb strip_pixels[STRIP_NUM_PIXELS];
            int strip_rc;

            for (size_t i = 0; i < STRIP_NUM_PIXELS; i++) {
                strip_pixels[i] = color;
            }

            strip_rc = led_strip_update_rgb(strip, strip_pixels, STRIP_NUM_PIXELS);

            if (strip_rc < 0) {
                LOG_WRN_ONCE("led_strip_update_rgb failed (%d); colour output "
                             "is not reaching the strip", strip_rc);
            }
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

    // Claim the second CDC ACM port used by the binary telemetry stream. A
    // failure only disables the stream; the shell and the rest keep working.
    LOG_INF("Calling stream_init()...");
    ret = stream_init();
    if (ret < 0) {
        LOG_ERR("stream_init failed with %d (binary stream unavailable)", ret);
    } else {
        LOG_INF("stream_init success.");
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
        float b0 = g_config.b0;
        float kp = g_config.kp;

        shell_print(sh, "Active ADRC (1st-order plant): dt=%0.4f s, wo=%0.1f rad/s, b0=%0.4f (W/s)/us, kp=%0.2f 1/s",
                    (double)g_config.dt, (double)g_config.wo, (double)b0, (double)kp);
        if (g_learning_mode) {
            shell_print(sh, "!! b0 = 0: learning pass-through is ACTIVE, the power limiter is OFF.");
        }
        if (b0 > 0.0f && kp > 0.0f) {
            /* kp = 1/lambda and b0 = K/tau. With the default lambda = tau the
             * plant follows from the gains alone; with an explicit lambda only
             * the authority below is exact. */
            shell_print(sh, "  lambda = 1/kp = %.1f ms; if lambda == tau, then tau = %.1f ms, K = %.4f W/us",
                        (double)(1000.0f / kp), (double)(1000.0f / kp), (double)(b0 / kp));
            shell_print(sh, "  full 1000 us stroke at %.0f W of power error",
                        (double)(1000.0f * b0 / kp));
        }
        shell_print(sh, "  kd=%0.2f is stored but UNUSED by the first-order law.",
                    (double)g_config.kd);
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
        } else if (kd != 0.0f) {
            shell_print(sh, "ADRC gains stored in NVS (note: kd=%0.2f is ignored, the law is first-order).",
                        (double)kd);
        } else {
            shell_print(sh, "ADRC gains successfully updated and stored in NVS.");
        }
        return 0;
    } else {
        shell_error(sh, "Invalid arguments. Usage: 'pm100 gains' or 'pm100 gains <wo> <b0> <kp> <kd>'");
        shell_error(sh, "  wo ~ 3-5x kp, b0 = static_gain/tau, kp = 2/tau, kd is ignored.");
        return -EINVAL;
    }
}

static int cmd_target(const struct shell *sh, size_t argc, char **argv)
{
    const struct control_ident_result *res = control_ident_result();
    float lo = 0.0f, hi = 0.0f;

    if (res->valid && res->levels >= 2u) {
        lo = res->p_settled[0];
        hi = res->p_settled[res->levels - 1];
    }

    if (argc == 1) {
        k_mutex_lock(&g_config_mutex, K_FOREVER);
        float tgt = g_config.target_power;
        k_mutex_unlock(&g_config_mutex);

        shell_print(sh, "Active Power Target: %0.2f W", (double)tgt);

        /* The target is useless if the plant cannot reach it, and that failure
         * is invisible in flight: pwm_out == pwm_in and everything looks fine. */
        if (!res->valid) {
            shell_print(sh, "  no identification yet: 'pm100 ident run' learns what this "
                            "plant can actually reach");
        } else {
            shell_print(sh, "  identified range: %0.1f..%0.1f W", (double)lo, (double)hi);
            if (tgt > hi) {
                shell_print(sh, "  ! above the identified range: the limiter can NEVER engage.");
            } else if (tgt < lo) {
                shell_print(sh, "  ! below the identified range: the feedforward is off, "
                                "feedback only.");
            }
        }
        return 0;
    } else if (argc == 2) {
        char *end;
        float target_power = strtof(argv[1], &end);

        if (end == argv[1]) {
            shell_error(sh, "'%s' is not a number.", argv[1]);
            return -EINVAL;
        }

        int ret = control_update_target(target_power);
        if (ret < 0) {
            shell_error(sh, "Failed to update target power: %d. Must be 0 < target <= %d W.",
                        ret, (int)CONTROL_TARGET_MAX_W);
            return ret;
        }

        shell_print(sh, "Target power set to %0.2f W and stored in NVS.", (double)target_power);

        if (res->valid && res->levels >= 2u && target_power > hi) {
            shell_print(sh, "  ! warning: above the identified maximum of %0.1f W, so the "
                            "limiter will never engage.", (double)hi);
        }
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

static void cmd_stream_status(const struct shell *sh)
{
    struct pm100_stream_stats stats;

    stream_get_stats(&stats);

    if (!stream_is_ready()) {
        shell_print(sh, "Binary stream : UNAVAILABLE (no second CDC ACM port)");
    } else {
        shell_print(sh, "Binary stream : %s, %u ms/frame (%u samples/frame)",
                    stream_is_enabled() ? "ON" : "off",
                    (unsigned int)stream_get_period_ms(),
                    (unsigned int)stream_get_period_ms());
        shell_print(sh, "  frames=%u sent/%u dropped, samples=%u sent/%u dropped, "
                            "%u tx_stalls, %u corrupt",
                    stats.frames_sent, stats.frames_dropped, stats.samples_sent,
                    stats.samples_dropped, stats.tx_stalls, stats.corrupt_records);
    }

    shell_print(sh, "CSV stream    : %s (first CDC ACM port, 10 Hz)",
                g_stream_active ? "ON" : "off");
}

static int cmd_stream(const struct shell *sh, size_t argc, char **argv)
{
    /* Legacy form: `stream <on|off>` toggles the CSV stream on the console
     * port. `stream csv <on|off>` is the explicit spelling of the same thing.
     * `stream bin <on|off> [period_ms]` drives the binary stream on the second
     * CDC ACM port, where period_ms is the frame period (1 kHz control loop, so
     * one period unit == one sample per frame; 1..48).
     */
    const char *what = argv[1];
    const char *state = NULL;

    if (strcmp(what, "status") == 0) {
        cmd_stream_status(sh);
        return 0;
    }

    if (strcmp(what, "bin") == 0) {
        if (argc < 3) {
            shell_error(sh, "Usage: stream bin <on|off> [period_ms]");
            return -EINVAL;
        }
        state = argv[2];
    } else if (strcmp(what, "csv") == 0) {
        if (argc < 3) {
            shell_error(sh, "Usage: stream csv <on|off>");
            return -EINVAL;
        }
        state = argv[2];
    } else if (strcmp(what, "on") == 0 || strcmp(what, "off") == 0) {
        state = what;
    } else {
        shell_error(sh, "Usage: stream <on|off> | stream csv <on|off> | "
                            "stream bin <on|off> [period_ms] | stream status");
        return -EINVAL;
    }

    bool enable;

    if (strcmp(state, "on") == 0) {
        enable = true;
    } else if (strcmp(state, "off") == 0) {
        enable = false;
    } else {
        shell_error(sh, "Invalid state '%s', use 'on' or 'off'.", state);
        return -EINVAL;
    }

    if (strcmp(what, "bin") == 0) {
        if (!stream_is_ready()) {
            shell_error(sh, "Binary stream port not available.");
            return -ENODEV;
        }

        if (argc >= 4) {
            unsigned long period = strtoul(argv[3], NULL, 10);

            if (period == 0 || period > 48) {
                shell_error(sh, "period_ms must be 1..48.");
                return -EINVAL;
            }
            stream_set_period_ms((uint16_t)period);
        }

        stream_set_enabled(enable);
        shell_print(sh, "Binary stream %s (%u ms/frame).",
                    enable ? "enabled" : "disabled", (unsigned int)stream_get_period_ms());
    } else {
        g_stream_active = enable;
        shell_print(sh, "CSV stream %s.", enable ? "enabled" : "disabled");
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
    if (g_learning_mode) {
        shell_print(sh, "!! Pass-through is ACTIVE: the throttle goes straight to the ESC and");
        shell_print(sh, "!! only the 1.2x safety cap protects the power train - there is NO limiter.");
        shell_print(sh, "!! Fix it with 'pm100 ident run', or by hand: 'pm100 gains <wo> <b0> <kp> 0'.");
    }

    switch (g_learning_stage) {
    case LEARNING_STAGE_IDLE:
        shell_print(sh, "Learning is not active.");
        shell_print(sh, "Start it with: pm100 gains <wo> 0 0 0  (b0 = 0 enters learning mode)");
        break;
    case LEARNING_STAGE_LOW_BAND:
        shell_print(sh, "Learning: hold throttle between 10%% and 50%% (1100-1500 us).");
        break;
    case LEARNING_STAGE_STEP_HIGH:
        shell_print(sh, "Learning: slam throttle above 75%% (1750-2100 us) and hold.");
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
        k_mutex_unlock(&g_config_mutex);
        shell_print(sh, "Learning complete: wo=%.2f, b0=%.4f, kp=%.2f", (double)wo, (double)b0,
                    (double)kp);
        shell_print(sh, "To relearn: pm100 gains %.2f 0 0 0", (double)wo);
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

static int cmd_ident(const struct shell *sh, size_t argc, char **argv)
{
    const struct control_ident_result *res = control_ident_result();

    if (argc >= 2 && strcmp(argv[1], "abort") == 0) {
        if (control_ident_running()) {
            control_ident_abort();
            shell_print(sh, "Identification aborted. Throttle returned to the pilot.");
        } else {
            shell_print(sh, "No identification is running.");
        }
        return 0;
    }

    if (argc >= 2 && (strcmp(argv[1], "run") == 0 || strcmp(argv[1], "tune") == 0)) {
        float lambda_s = 0.0f;

        if (argc >= 3) {
            lambda_s = (float)atoi(argv[2]) / 1000.0f;
            if (lambda_s <= 0.0f || lambda_s > CONTROL_LAMBDA_MAX_S) {
                shell_error(sh, "lambda must be 1..%d ms (omit for automatic)",
                            (int)(CONTROL_LAMBDA_MAX_S * 1000.0f));
                return -EINVAL;
            }
        }

        if (control_ident_start(lambda_s) != 0) {
            shell_error(sh, "Identification is already running.");
            return -EBUSY;
        }

        shell_print(sh, "");
        shell_print(sh, "  !!  The FIRMWARE WILL DRIVE THE MOTOR  !!");
        shell_print(sh, "  Secure the vehicle and clear the propeller arc.");
        shell_print(sh, "  Arming: leave the stick at idle for 1 second.");
        shell_print(sh, "  Move the stick at any time to abort.");
        shell_print(sh, "");
        shell_print(sh, "  5 levels, 600 ms settle each, ~4 s total. Gains are applied and");
        shell_print(sh, "  saved to NVS at the end. Use 'pm100 ident' to see the result.");
        return 0;
    }

    /* Default: report status / the last result. */
    if (res->running) {
        shell_print(sh, "Identification running... ('pm100 ident abort' to stop)");
        return 0;
    }

    if (!res->valid) {
        shell_print(sh, "No identification result yet.");
        shell_print(sh, "Run 'pm100 ident run [lambda_ms]' (alias: 'tune') to identify the plant.");
        shell_print(sh, "Mapping: b0 = K/tau, kp = 1/lambda, wo = 1/sqrt(lambda*tau)");
        if (res->last_abort[0] != '\0') {
            shell_print(sh, "Last abort: %s", res->last_abort);
        }
        return 0;
    }

    shell_print(sh, "  u[us]    P[W]   K[W/us]  tau[ms]  +-%%   t0[ms]  band  shape  noise[W]");
    for (uint8_t i = 0; i < CONTROL_IDENT_LEVELS; i++) {
        const char *note = "";

        if (i > 0 && i >= res->levels) {
            note = "  <- saturated";
        } else if (i > 0 && res->tau[i] <= 0.0f) {
            note = "  <- rejected";
        }

        if (i == 0) {
            shell_print(sh, "  %4u   %6.1f        -        -     -       -     -      -      %6.2f",
                        res->u_us[i], (double)res->p_settled[i], (double)res->noise_w[i]);
        } else {
            shell_print(sh, "  %4u   %6.1f  %8.4f  %7.1f  %4.0f  %6.1f  %4u  %5.2f      %6.2f%s",
                        res->u_us[i], (double)res->p_settled[i], (double)res->k[i],
                        (double)(res->tau[i] * 1000.0f),
                        (double)(100.0f * res->tau_rel_err[i]),
                        (double)(res->deadtime[i] * 1000.0f),
                        (unsigned int)res->n_band[i], (double)res->shape[i],
                        (double)res->noise_w[i], note);
        }
    }
    shell_print(sh, "  shape 1.47 = first-order response. tau median %.1f ms from %u steps, "
                "K at target %.4f W/us, Vref %.2f V",
                (double)(res->tau_median * 1000.0f), (unsigned int)res->n_tau,
                (double)res->k_at_target, (double)res->v_ref);
    shell_print(sh, "  lambda %.1f ms -> wo %.1f rad/s, b0 %.4f (W/s)/us, kp %.2f 1/s (loop %.2f ms)",
                (double)(res->lambda * 1000.0f), (double)res->wo, (double)res->b0,
                (double)res->kp, (double)(res->loop_dt * 1000.0f));
    shell_print(sh, "  Feedforward active for %.1f..%.1f W (rescaled by Vref/Vbus).",
                (double)res->p_settled[0], (double)res->p_settled[res->levels - 1]);
    return 0;
}

/* Define master pm100 subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(pm100_subcmds,
    SHELL_CMD_ARG(gains, NULL, "Get active gains, or set: <wo> <b0> <kp> <kd>", cmd_update_adrc_gains, 1, 4),
    SHELL_CMD_ARG(learn, NULL, "Show b0 learning instructions/status", cmd_learn, 1, 0),
    SHELL_CMD_ARG(target, NULL, "Get active power target, or set: <power_watts>", cmd_target, 1, 1),
    SHELL_CMD_ARG(shunt, NULL, "Get active shunt resistor (mOhm), or set: <value_mohm>", cmd_shunt, 1, 1),
    SHELL_CMD_ARG(sample, NULL, "Print current telemetry CSV readings once", cmd_readings, 1, 0),
    SHELL_CMD_ARG(ident, NULL, "Automatic plant identification: <[run|tune] [lambda_ms]|abort>", cmd_ident, 1, 2),
    SHELL_SUBCMD_SET_END
);

/* Register master pm100 command */
SHELL_CMD_REGISTER(pm100, &pm100_subcmds, "PM100 Power Limiter System Control Commands", NULL);

/* Standalone global commands */
SHELL_CMD_ARG_REGISTER(team_name, NULL, "Get active team name, or set: <name>", cmd_team_name, 1, 1);
SHELL_CMD_ARG_REGISTER(team_number, NULL, "Get active team number, or set: <number>", cmd_team_number, 1, 1);
SHELL_CMD_ARG_REGISTER(ble_pin, NULL, "Get active pairing PIN, or set: <pin_code> (6 characters)", cmd_set_pin, 1, 1);
SHELL_CMD_ARG_REGISTER(stream, NULL, "CSV stream <on|off>, binary stream <bin on|off [period_ms]>, <status>", cmd_stream, 2, 2);
SHELL_CMD_ARG_REGISTER(blink, NULL, "Trigger 10x white LED blinks", cmd_blink, 1, 0);

/* Include auxiliary .c files for compilation in single translation unit */
#include "shell_cmds.c"
#include "ble_telemetry.c"
