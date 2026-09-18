#ifndef __CONTROL_H
#define __CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

#define CONTROL_PERIOD_US 300 // 500 Hz control loop period

enum ctrl_state {
    READY,
    LIMITING_POWER,
    ERROR_NO_INPUT,
    ERROR_NO_BATTERY,
    BLINK
};

/* Telemetry data structure */
struct system_telemetry {
    float power_w;
    float current_a;
    float voltage_v;
    float total_consumption_j;
    uint64_t time_ms;
    int pwm_input_us;
    int pwm_output_us;
    int pwm_control_us;
    enum ctrl_state state;
};

/* Persistent configuration structure saved in NVS */
struct __attribute__((packed)) device_config {
    /* ADRC Controller Parameters */
    float dt;              // Controller sampling time step (seconds)
    float wo;              // Observer bandwidth (rad/s)
    float b0;              // Controller input gain scaling factor
    float kp;              // Proportional gain
    float kd;              // Derivative gain
    float target_power;    // Default Power Target in Watts
    float shunt_resistor_mohm; // Shunt Resistor value in mOhm

    /* Team & Identification Information */
    char team_name[32];    // Team name string (up to 32 bytes, null-terminated)
    uint32_t team_number;  // Unique identifier for the team / controller
    char PIN_code[7];      // 6-character security PIN (+1 byte for null terminator)
};

/* Global shared variables */
extern struct system_telemetry g_telemetry;
extern struct k_mutex g_telemetry_mutex;

extern struct device_config g_config;
extern struct k_mutex g_config_mutex;

extern volatile bool g_stream_active;
extern volatile int64_t g_blink_start_time;

/* Function prototypes */
int control_init(void);

/* Mutex-protected set functions for shell command thread safety */
int control_update_gains(float dt, float wo, float b0, float kp, float kd);
int control_update_target(float target_power);
int control_update_shunt(float shunt_resistor_mohm);

#endif /* __CONTROL_H */
