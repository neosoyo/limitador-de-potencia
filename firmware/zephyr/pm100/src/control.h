#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/* Control loop timing: 1 kHz */
#define CONTROL_PERIOD_US 1000u
#define CONTROL_PERIOD_MS 1u
#define CONTROL_PERIOD_S  0.001f

/* How long a CLI/BLE white-blink request overrides the state colour, in ms. */
#define BLINK_HOLD_MS 5000

/* ------------------------------------------------------------------------- */
/* Automatic identification (staircase)                                      */
/* ------------------------------------------------------------------------- */

/* Throttle levels the firmware commands itself, and the pre-step power (W)
 * each level must reach before the next step. */
#define CONTROL_IDENT_LEVELS 5

/* IMC closed-loop time constant used to derive the gains from K and tau.
 * Floor: the ESC output PWM has a 20 ms period, so the actuator alone adds a
 * 0-20 ms (mean 10 ms) delay; a loop faster than this rings regardless of the
 * observer tuning. */
#define CONTROL_LAMBDA_MIN_S 0.025f
#define CONTROL_LAMBDA_MAX_S 0.500f

/* Sanity bound on the power target. A target of 0 or less would command the ESC
 * floor regardless of the stick (the limiter becomes a motor kill switch), and
 * an absurd value silently disables the limiter altogether - the bench plant
 * tops out around 190 W, so a typo like `pm100 target 800` for 80 W looks
 * exactly like "the limiter is not running". Both are rejected rather than
 * stored. See also the reachability check against the identified range. */
#define CONTROL_TARGET_MAX_W 1000.0f

/** Result of the last automatic identification. */
struct control_ident_result {
	bool valid;                        /* a completed run is available     */
	bool running;                      /* a run is in progress             */
	uint8_t levels;                    /* number of populated staircase points */
	uint16_t u_us[CONTROL_IDENT_LEVELS];  /* commanded throttles            */
	float p_settled[CONTROL_IDENT_LEVELS];/* settled power at each level, W  */
	float k[CONTROL_IDENT_LEVELS];     /* incremental gain dP/du, W per us */
	float tau[CONTROL_IDENT_LEVELS];   /* fitted time constant, s          */
	float shape[CONTROL_IDENT_LEVELS]; /* (t90-t63)/(t63-t10), 1.47 = first order */
	float tau_rel_err[CONTROL_IDENT_LEVELS]; /* 1-sigma tau uncertainty, rel. */
	float deadtime[CONTROL_IDENT_LEVELS]; /* t0 from the 10% crossing, s   */
	uint16_t n_band[CONTROL_IDENT_LEVELS]; /* samples in the 10-90% transit */
	float noise_w[CONTROL_IDENT_LEVELS]; /* settled-phase power noise, W rms */
	uint8_t n_tau;                     /* steps that passed the fit gate   */
	float tau_median;                  /* median of the valid tau values   */
	float k_at_target;                 /* local gain at the power target   */
	float v_ref;                       /* mean bus voltage during the run  */
	float lambda;                      /* chosen closed-loop time constant */
	float b0, kp, wo;                  /* gains that were applied          */
	float loop_dt;                     /* measured control period during the run */
	char last_abort[40];               /* reason the last run stopped      */
};

/**
 * @brief Arm and run the automatic identification staircase.
 *
 * The firmware takes ownership of the ESC output (bypassing
 * pwm_out = min(pwm_in, u)) and commands CONTROL_IDENT_LEVELS settled throttle
 * levels, fitting K, tau and the stimulus dead time at each step. On success it
 * derives lambda-tuned gains, applies them and saves them to NVS.
 *
 * @warning The motor is driven by the firmware for the duration of the run.
 *          Requires the pilot's stick to have been at idle for 1 s (that is the
 *          arm condition) and aborts immediately if the stick moves, if the
 *          power cap or timeout is exceeded, or if the input becomes unsafe.
 *
 * @param lambda_s Requested closed-loop time constant in seconds; <= 0 selects
 *                 max(tau_median, CONTROL_LAMBDA_MIN_S).
 * @return int 0 if the run was armed, negative errno otherwise.
 */
int control_ident_start(float lambda_s);

/** @brief Abort a running identification and hand the throttle back. */
void control_ident_abort(void);

/** @return true while the identification owns the ESC output. */
bool control_ident_running(void);

/** @return pointer to the last identification result (never NULL). */
const struct control_ident_result *control_ident_result(void);

/* ESC PWM pulse boundaries (microseconds) */
#define PWM_SAFE_STOP_US  1000
#define PWM_MIN_US        1000
#define PWM_MAX_US        2000

/*
 * Throttle input validity window (microseconds).
 *
 * Receivers do not stop at exactly 2000 us: the flight log showed full stick
 * producing 2002-2016 us, which failed the old upper bound of 2000 and turned
 * a full-throttle request into ERROR_NO_INPUT + safe stop (throttle cut to
 * 1000 us). The window is therefore 850..2100 us. It still rejects "no signal"
 * (input_period is forced to 0 when no pulse arrives, and above 2500 us by the
 * capture path) and anything below the transmitter's low endpoint, while
 * tolerating the normal endpoint overshoot of real receivers.
 */
#define PWM_INPUT_MIN_US  850
#define PWM_INPUT_MAX_US  2100

/* Minimum battery voltage to consider the system safe (Volts) */
#define BATTERY_MIN_V     5.0f

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
