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

//static const struct device *const ina226_dev = DEVICE_DT_GET_ANY(ti_ina226);

#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define STRIP_NODE DT_ALIAS(rgba)

LOG_MODULE_REGISTER(PM100_devel, LOG_LEVEL_DBG);


// Getting peripherals from device tree
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

struct led_rgb pixels[STRIP_NUM_PIXELS];

int main(void)
{
	// Give the USB CDC ACM console time to connect
	k_msleep(2000);

	LOG_INF("Starting PM100 Initialization...");

	int ret;
	bool led_state = true;
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		//Optional
		//.calibrate = true,
	};

	ret = device_is_ready(strip);
	if (ret < 0) {
		LOG_ERR("Led strip failed with %d", ret);
	} else {
		LOG_INF("Led strip success.");
	}

	LOG_INF("Calling control_init()...");
	ret = control_init();
	if (ret < 0) {
		LOG_ERR("control_init failed with %d", ret);
	} else {
		LOG_INF("control_init success.");
	}

	LOG_INF("Checking LED DT spec...");
	if (!gpio_is_ready_dt(&led)) {
	        LOG_ERR("Error: LED device not ready");
	        return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
	        LOG_ERR("Error: Failed to configure LED pin");
	        return 0;
	}

	LOG_INF("Checking ADC channel...");
	if (!adc_is_ready_dt(&adc_channel)) {
	        LOG_ERR("Error: ADC controller device not ready");
	        return 0;
	}
	ret = adc_channel_setup_dt(&adc_channel);
	if (ret < 0) {
	        LOG_ERR("Error: Could not setup ADC channel (%d)", ret);
	        return 0;
	}
	ret = adc_sequence_init_dt(&adc_channel, &sequence);
	if (ret < 0) {
	        LOG_ERR("Error: Could not initalize ADC sequence (%d)", ret);
	        return 0;
	}

	LOG_INF("Checking LED Strip...");
	if (!device_is_ready(strip)) {
		LOG_ERR("Error: LED strip device %s is not ready", strip->name);
		return 0;
	}

	uint8_t color_idx = 0;
	LOG_INF("Initialization Complete. Entering main loop.");

	while (1) {
		int val_mv;
		float voltage, current;

		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		/* Cycle LED colors */
		memset(&pixels, 0, sizeof(pixels));
		switch (color_idx) {
		case 0: pixels[0].r = 0x20; break;
		case 1: pixels[0].g = 0x20; break;
		case 2: pixels[0].b = 0x20; break;
		}
		color_idx = (color_idx + 1) % 3;

		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

		ret = adc_read(adc_channel.dev, &sequence);
		if (ret < 0) {
			LOG_WRN("Could not read (%d)\n", ret);
			continue;
		}
		val_mv = (buf*3600)/4095;
		/* conversion to mV may not be supported, skip if not */
		if (ret < 0) {
					printf(" (value in mV not available)\n");
			} else {
				printf(" = %d mV\n", val_mv);
				printf("Raw data %d\n", buf);
		}
		led_state = !led_state;
		printf("LED state: %s\r\n", led_state ? "ON" : "OFF");
		
		printf("dt moto_control thread: %d\n",times());
		printf("dt moto_control thread: %f\n",deltaTime() * 1e6);
		// Validating pwm output

	
		// ret = get_power(&voltage, &current);
		// if (ret == 0) {
		// 	printk("Bus Voltage: %.3f V, Current: %.3f A\n", voltage, current);
		// } else {
		// 	printk("Error: Failed to read INA226 (%d)\n", ret);
		// }
		
		k_msleep(SLEEP_TIME_MS);
		
	}
	return 0;
}

// SHELL COMMANDS 

void update_target(const struct shell *shell, size_t argc, char **argv){
	float target = strtof(argv[0],NULL);
}


void update_adrc_gains(const struct shell *shell, size_t argc, char **argv){
	if (argc != 5){
		return -EINVAL;
	}
	float dt = strtof(argv[0], NULL);
    float wo = strtof(argv[1], NULL);
    float bo = strtof(argv[2], NULL);
    float kp = strtof(argv[3], NULL);
    float kd = strtof(argv[4], NULL);
}

SHELL_CMD_ARG_REGISTER(update_adrc_gains, NULL, "Update dt, w0, b0, kp and kd from ADRC controller", update_adrc_gains,1,0);
SHELL_CMD_ARG_REGISTER(target,NULL,"Power Target in Watts [float]", update_target,1,0);