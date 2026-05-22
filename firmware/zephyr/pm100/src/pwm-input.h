#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <hal/nrf_gpio.h>

static const struct gpio_dt_spec input = GPIO_DT_SPEC_GET(DT_NODELABEL(throtle), gpios);

static nrfx_timer_t timer2_inst = NRFX_TIMER_INSTANCE(2);
static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
static nrf_ppi_channel_t ppi_channel_cap;
static nrf_ppi_channel_t ppi_channel_clr;

volatile uint32_t last_high_time;
volatile uint32_t last_period;
static uint32_t last_low_time;
volatile static int input_period;

static void nrfx_gpiote_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
	uint32_t phase_time_ticks = nrfx_timer_capture_get(&timer2_inst, NRF_TIMER_CC_CHANNEL0);
    // Timer runs at 16MHz (prescaler 0). 16 ticks = 1 us.
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

static int pwm_input_init(void)
{
    int ret;
    nrfx_err_t status;
    uint32_t abs_pin = NRF_GPIO_PIN_MAP(0, input.pin); // Assuming port 0 for xiao_ble

    printk("Init Timer...\n");
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(16000000); 
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	status = nrfx_timer_init(&timer2_inst, &timer_config, NULL);
    if (status != NRFX_SUCCESS && status != NRFX_ERROR_ALREADY) {
        printk("Failed to initialize timer: %d\n", status);
        return -1;
    }

    printk("Init GPIOTE...\n");
	if (!nrfx_gpiote_init_check(&gpiote_inst)) {
		status = nrfx_gpiote_init(&gpiote_inst, 0);
		if (status != 0 && status != NRFX_ERROR_ALREADY) {
			printk("Failed to initialize GPIOTE: %d\n", status);
			return -1;
		}
	}

    printk("Alloc GPIOTE channel...\n");
    uint8_t channel;
    status = nrfx_gpiote_channel_alloc(&gpiote_inst, &channel);
    if (status != NRFX_SUCCESS) {
        printk("Failed to allocate GPIOTE channel: %d\n", status);
        return -1;
    }

    printk("Config GPIOTE pin...\n");
    nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLDOWN;
    nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
        .p_in_channel = &channel,
    };
    nrfx_gpiote_handler_config_t handler_config = {
        .handler = nrfx_gpiote_handler,
    };
    nrfx_gpiote_input_pin_config_t input_config = {
        .p_pull_config = &pull_config,
        .p_trigger_config = &trigger_config,
        .p_handler_config = &handler_config,
    };

	status = nrfx_gpiote_input_configure(&gpiote_inst, abs_pin, &input_config);
	if (status != NRFX_SUCCESS) {
		printk("Failed to initialize GPIOTE input pin %d: %d\n", abs_pin, status);
        return -1;
	}

    printk("Enable GPIOTE trigger...\n");
	nrfx_gpiote_trigger_enable(&gpiote_inst, abs_pin, true);

	uint32_t gpiote_evt_addr = nrfx_gpiote_in_event_address_get(&gpiote_inst, abs_pin);

	if (gpiote_evt_addr != 0) {
        printk("Alloc PPI channels...\n");
		uint32_t timer_cap_task = nrfx_timer_task_address_get(&timer2_inst, NRF_TIMER_TASK_CAPTURE0);
		uint32_t timer_clr_task = nrfx_timer_task_address_get(&timer2_inst, NRF_TIMER_TASK_CLEAR);

		if (nrfx_ppi_channel_alloc(&ppi_channel_cap) != NRFX_SUCCESS) {
			printk("Failed to alloc PPI cap\n");
            return -1;
		}
		if (nrfx_ppi_channel_alloc(&ppi_channel_clr) != NRFX_SUCCESS) {
			printk("Failed to alloc PPI clr\n");
            return -1;
		}

        printk("Assign PPI channels...\n");
		nrfx_ppi_channel_assign(ppi_channel_cap, gpiote_evt_addr, timer_cap_task);
		nrfx_ppi_channel_assign(ppi_channel_clr, gpiote_evt_addr, timer_clr_task);

        printk("Enable PPI channels...\n");
		nrfx_ppi_channel_enable(ppi_channel_cap);
		nrfx_ppi_channel_enable(ppi_channel_clr);
	} else {
		printk("Could not find GPIOTE channel for input pin\n");
        return -1;
	}

    printk("Enable Timer...\n");
    nrfx_timer_enable(&timer2_inst);

    printk("PWM input init complete.\n");
    return 0;
}

static inline int pwm_input_get_period(void)
{
    return input_period;
}

#endif // PWM_INPUT_H
