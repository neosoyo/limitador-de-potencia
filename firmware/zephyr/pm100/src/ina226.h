#ifndef __INA226_H
#define __INA226_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include "motor_hardware.h"
#include "pwm-input.h"

static const struct pwm_dt_spec output = PWM_DT_SPEC_GET(DT_NODELABEL(esc));

struct ina226 {
    struct motor_hardware_if interface;

    float current_power;
    float current_mV;
    float current_mA;
    float current_joules;

};


static inline
struct motor_hardware_if * ina226_get_if(struct ina226 *mh)
{
    if(mh) {
        return &mh->interface;
    }

    return NULL;
}

int ina226_init(struct ina226 *mh, const struct device *ina226_dev);

#endif // INA266_H
