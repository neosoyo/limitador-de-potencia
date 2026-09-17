#include "ina226.h"
#include <zephyr/logging/log.h>

/**
 * @brief Set the throttle output for the Electronic Speed Controller (ESC).
 * 
 * Writes the minimum value between the ADRC controller command and the pilot's input
 * throttle to the physical PWM device, enforcing safety bounds.
 * 
 * @param self Pointer to the base motor hardware interface.
 * @param current_throtle Calculated throttle command from the ADRC controller (in us).
 * @return int 0 on success, or a negative error code on failure.
 */
static int set_throtle(struct motor_hardware_if *self, float current_throtle)
{
    if (!self) {
        return -EINVAL;
    }

    int input = pwm_input_get_period();
    int output_control = (int)current_throtle;

    // Enforce pilot override: throttle output can never exceed physical input from pilot
    if (input < output_control) {
        output_control = input;
    }

    int ret;
    // Standard RC pulse limits (900 us to 2100 us) with safe guard bounds [750, 2500]
    if (output_control >= 750 && output_control <= 2500) {
        ret = pwm_set_pulse_dt(&output, PWM_USEC(output_control));
        if (ret < 0) {
            LOG_ERR("Failed to set PWM pulse %d: error %d", output_control, ret);
            return ret;
        }
        return 0;
    } else {
        // Safe minimum pulse for out of bounds
        ret = pwm_set_pulse_dt(&output, PWM_USEC(1000));
        if (ret < 0) {
            LOG_ERR("Failed to set safe PWM pulse: error %d", ret);
            return ret;
        }
    }

    return -EINVAL;
}

/**
 * @brief Fetch current power, voltage, and current readings from the INA226 sensor.
 * 
 * @param self Pointer to the base motor hardware interface.
 * @param current_power Pointer to store the fetched active power measurement (in Watts).
 * @return int 0 on success, or a negative error code on failure.
 */
static int get_power(struct motor_hardware_if *self, float *current_power)
{
    if (!self || !current_power) {
        return -EINVAL;
    }

    struct ina226 *mh = MH_CONTAINER_OF(self, struct ina226, interface);

    if (!mh->dev) {
        return -ENODEV;
    }

    struct sensor_value v_val, i_val, p_val;
    int ret;

    ret = sensor_sample_fetch(mh->dev);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(mh->dev, SENSOR_CHAN_VOLTAGE, &v_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(mh->dev, SENSOR_CHAN_CURRENT, &i_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(mh->dev, SENSOR_CHAN_POWER, &p_val);
    if (ret < 0) {
        return ret;
    }

    // Convert sensor values to standard SI units
    float voltage = (float)sensor_value_to_double(&v_val);
    float current = (float)sensor_value_to_double(&i_val);
    float power = (float)sensor_value_to_double(&p_val);

    // Retrieve active shunt resistor value thread-safely
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    float shunt_mohm = g_config.shunt_resistor_mohm;
    k_mutex_unlock(&g_config_mutex);

    if (shunt_mohm <= 0.0f) {
        shunt_mohm = 1.0f; // safety fallback
    }

    // Zephyr's DTS default configured shunt is 1.0mOhm (1000 micro-ohms)
    // Scale factor = (DTS Shunt) / (Physical Shunt)
    float shunt_scale = 1.0f / shunt_mohm;
    current *= shunt_scale;
    power *= shunt_scale;

    mh->current_power = power;
    mh->current_mV = voltage * 1000.0f;
    mh->current_mA = current * 1000.0f;

    *current_power = power;

    return 0;
}

/**
 * @brief Initialize the INA226 and PWM input/output interfaces.
 * 
 * Configures the hardware interface handlers.
 * 
 * @param mh Pointer to the ina226 driver struct.
 * @param ina226_dev Pointer to the Zephyr sensor device instance.
 * @return int 0 on success, or a negative error code on failure.
 */
int ina226_init(struct ina226 *mh, const struct device *ina226_dev)
{
    if (!mh || !ina226_dev) {
        return -EINVAL;
    }

    int ret;

    ret = pwm_input_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize PWM input (error %d)", ret);
        return -ENODEV;
    }

    if (!pwm_is_ready_dt(&output)) {
        LOG_ERR("PWM output device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(ina226_dev)) {
        LOG_ERR("INA226 sensor device not ready");
        return -ENODEV;
    }

    mh->dev = ina226_dev;
    mh->interface.get_power = get_power;
    mh->interface.set_throtle = set_throtle;

    mh->current_power = 0.0f;
    mh->current_mV = 0.0f;
    mh->current_mA = 0.0f;
    mh->current_joules = 0.0f;

    return 0;
}
