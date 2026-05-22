#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/counter.h>

#define CONTROL_PIPELINE_US_PERIOD (500)

static float now_seconds = 0.0f;
static float last_seconds = 0.0f;
static float dt = 0.0f;
static const struct device *timer_dev = DEVICE_DT_GET(timer1);
static struct counter_alarm_cfg alarm_cfg;

static void motor_control_work_handler(struct k_work *work)
{
    uint32_t now_ticks;
    counter_get_value(timer_dev, &now_ticks);
    now_seconds = (float)(counter_ticks_to_us(timer_dev, now_ticks)) * 1e-6;
    dt = now_seconds - last_seconds;
    last_seconds = now_seconds;
}

K_WORK_DEFINE(motor_work, motor_control_work_handler);

static void timer_interrupt_fn(const struct device *counter_dev,
                            uint8_t chan_id, uint32_t ticks,
                            void *user_data)
{
    k_work_submit(&motor_work);
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);
}

static int motor_control_pipeline_init_backend(void)
{
    counter_start(timer_dev);
    alarm_cfg.flags = 0;
    alarm_cfg.ticks = counter_us_to_ticks(timer_dev, CONTROL_PIPELINE_US_PERIOD);
    alarm_cfg.callback = timer_interrupt_fn;
    alarm_cfg.user_data = &alarm_cfg;
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);

    return 0;
}

SYS_INIT(motor_control_pipeline_init_backend, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
