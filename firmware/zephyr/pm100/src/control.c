#include "control.h"

static float now_seconds = 0.0f;
static float last_seconds = 0.0f;
static float delta = 0.0f;
static int ttt = 1;

static void motor_control_work_handler(struct k_work *work)
{
    uint32_t now_ticks = k_cycle_get_32();
    
    // Assuming 32768Hz RTC clock or System clock. Using k_cyc_to_us_floor32 is safer.
    uint32_t now_us = k_cyc_to_us_floor32(now_ticks);
    
    now_seconds = (float)now_us * 1e-6f; 
    delta = now_seconds - last_seconds;
    last_seconds = now_seconds;
    ttt = ttt+1;
}

K_WORK_DEFINE(motor_work, motor_control_work_handler);

static void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&motor_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

float deltaTime(void)
{
    return delta;
}

int times(void)
{
    return ttt;
}

int control_init(void)
{ 
    // Start the timer with a period of 500 microseconds.
    // The first argument is duration before first expiry, second is the period.
    k_timer_start(&my_timer, K_USEC(CONTROL_US_PERIOD), K_USEC(CONTROL_US_PERIOD));
    return 0;
}

