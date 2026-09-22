#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

<<<<<<< HEAD
/* Control loop timing: 1 kHz */
#define CONTROL_PERIOD_US 1000u
#define CONTROL_PERIOD_MS 1u
#define CONTROL_PERIOD_S  0.001f

/* ESC PWM pulse boundaries (microseconds) */
#define PWM_SAFE_STOP_US  1000
#define PWM_MIN_US        1000
#define PWM_MAX_US        2000

/* Throttle input validity window (microseconds) */
#define PWM_INPUT_MIN_US  900
#define PWM_INPUT_MAX_US  2000

/* Minimum battery voltage to consider the system safe (Volts) */
#define BATTERY_MIN_V     5.0f
=======
#define CONTROL_PERIOD_US 300 // 500 Hz control loop period
>>>>>>> refs/remotes/origin/main

enum ctrl_state {
    READY,
    LIMITING_POWER,
    ERROR_NO_INPUT,
    ERROR_NO_BATTERY,
    BLINK,
    LEARNING
};

/* b0 learning progress stages */
enum learning_stage {
    LEARNING_STAGE_IDLE,
    LEARNING_STAGE_LOW_BAND,
    LEARNING_STAGE_STEP_HIGH,
    LEARNING_STAGE_STEP_LOW,
    LEARNING_STAGE_ESTIMATING,
    LEARNING_STAGE_DONE
};

/* Telemetry data structure */
struct system_telemetry {
    float power_w;              // Peak power measured since boot (Watts)
    float current_a;            // Instantaneous current (Amperes)
    float voltage_v;            // Instantaneous bus voltage (Volts)
    float total_consumption_j;  // Accumulated energy (Joules)
    uint64_t time_ms;           // Uptime timestamp (ms)
    int pwm_input_us;           // Pilot throttle input pulse (us)
    int pwm_output_us;          // Applied ESC output pulse (us)
    int pwm_control_us;         // Raw ADRC control effort (us)
    enum ctrl_state state;      // Active control state
};

/* Persistent configuration structure saved in NVS */
struct __attribute__((packed)) device_config {
    /* ADRC Controller Parameters */
    float dt;                   // Fixed sampling time step (1 kHz loop, seconds)
    float wo;                   // Observer bandwidth (rad/s)
    float b0;                   // Controller input gain scaling factor
    float kp;                   // Proportional gain
    float kd;                   // Derivative gain
    float target_power;         // Power target limit (Watts)
    float shunt_resistor_mohm;  // Shunt resistor value (mOhm)

    /* Team & Identification Information */
    char team_name[32];         // Team name string (null-terminated)
    uint32_t team_number;       // Unique identifier for the team / controller
    char PIN_code[7];           // 6-character security PIN (+null terminator)
};

/* Global shared variables */
extern struct system_telemetry g_telemetry;
extern struct k_mutex g_telemetry_mutex;

extern struct device_config g_config;
extern struct k_mutex g_config_mutex;

extern volatile bool g_stream_active;
extern volatile bool g_learning_mode;
extern volatile enum learning_stage g_learning_stage;
extern volatile int64_t g_blink_start_time;

/* Function prototypes */
int control_init(void);

/* Mutex-protected set functions for shell command thread safety */
int control_update_gains(float wo, float b0, float kp, float kd);
int control_update_target(float target_power);
int control_update_shunt(float shunt_resistor_mohm);

#endif /* __CONTROL_H */
