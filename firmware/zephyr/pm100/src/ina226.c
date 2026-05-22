#include "ina226.h"


int ina226_init(struct ina226 *mh, const struct device *ina226_dev)
{
    
   	ret = pwm_input_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize PWM input\n");
		return -ENODEV;
	}
    if (!pwm_is_ready_dt(&output)) {
	        printk("Error: PWM device not ready\n");
	    return -ENODEV;
	}
    
    if (!device_is_ready(ina226_dev)) {
        printk("INA226 (ti,ina226) device not ready\n");
        return -ENODEV;
    }

    mh->interface.get_power = get_power;

    return motor_hardware_reset(&mh->interface);
}
static int set_throtle(struct ina226 *mh, float current_throtle)
{
    // Set the throtle output for the esc, it will be the lowest value
    // between ADRC controller and pilot input.

    struct ina226 *mh =
         MH_CONTAINER_OF(self, struct ina226, interface);

	int input = pwm_input_get_period();
    int output_control = (int) current_throtle;

    if(input < output_control){
        output_control = input;
    }

    if (output_control > 750 & output_control < 2500 ){
        ret = pwm_set_pulse_dt(&output,PWM_USEC(output_control));

        if (ret < 0) {
            printk("Error %d: failed to set pulse width\n", ret);
            return ret;
        }
        return 0;
    }else{
        ret = pwm_set_pulse_dt(&output,PWM_USEC(1000));

        if (ret < 0) {
            printk("Error %d: failed to set pulse width\n", ret);
            return ret;
        }
    }

	return -EINVAL;

}

static int get_power(struct ina226 *mh, float *current_power)
{
    struct ina226 *mh =
         MH_CONTAINER_OF(self, struct ina226, interface);

    struct sensor_value v_val, i_val, p_val;
    float voltage, current, power;
    int ret;

    ret = sensor_sample_fetch(ina226_dev);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_VOLTAGE, &v_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_CURRENT, &i_val);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_POWER, &p_val);
    if (ret < 0) {
        return ret;
    }
    voltage = sensor_value_to_double(&v_val) *  1000;
    current = sensor_value_to_double(&i_val) * 1000;

    power = sensor_value_to_double(&p_val);

    mh->current_power = power;
    mh->current_mV = voltage;
    mh->current_mA = current;

    current_power = mh->current_power;

    return 0;
}
