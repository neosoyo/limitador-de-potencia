#include "adrc_control_law.h"
#include <math.h>

/**
 * @brief Reset the ADRC controller states.
 * 
 * Sets reference, measurement, and observer states (zhat) to 0.0f.
 * Resets u_prev to 0.0f.
 * 
 * @param self Pointer to the base control law interface.
 * @return int 0 on success.
 */
static int reset (struct siso_control_law * self)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    al->interface.reference = 0.0f;
    al->interface.measurement = 0.0f;
    al->zhat[0] = 0.0f;
    al->zhat[1] = 0.0f;
    al->zhat[2] = 0.0f;
    al->u_prev = 0.0f;

    return 0;
}

/**
 * @brief Set the reference and measurement for the ADRC controller.
 * 
 * @param self Pointer to the base control law interface.
 * @param reference Desired power target in Watts.
 * @param measurement Measured power in Watts.
 * @return int 0 on success.
 */
static int set (struct siso_control_law * self, float reference, float measurement)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    al->interface.reference = reference;
    al->interface.measurement = measurement;

    return 0;
}

/**
 * @brief Update the ADRC controller observer and calculate control command.
 * 
 * Performs Euler integration of the 3rd-order Linear Extended State Observer (LESO)
 * and calculates the control effort. Employs anti-windup clamping [1000, 2000]
 * to prevent observer and actuator windup.
 * 
 * @param self Pointer to the base control law interface.
 * @param dt Sampling time step in seconds.
 * @param command Pointer to store the calculated control effort (PWM pulse in us).
 * @return int 0 on success, or -EINVAL if command pointer is NULL.
 */
static int update (struct siso_control_law * self, float dt, float *command)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    if(!command)
        return -EINVAL;

    float ref = al->interface.reference;
    float mes = al->interface.measurement;
    float zhat_1 = al->zhat[0];
    float zhat_2 = al->zhat[1];
    float zhat_3 = al->zhat[2];
    
    // Discrete-to-continuous scaled LESO gains
    float l1 = al->l[0];
    float l2 = al->l[1];
    float l3 = al->l[2];
    
    float kp = al->kp;
    float kd = al->kd;
    float b0 = al->b0;
    float u_prev = al->u_prev;

    // 1. LESO Observer Dynamics (Continuous Euler Integration):
    float zhat_dot_1 = -l1 * zhat_1 + zhat_2 + l1 * mes;
    float zhat_dot_2 = -l2 * zhat_1 + zhat_3 + l2 * mes + b0 * u_prev;
    float zhat_dot_3 = -l3 * zhat_1 + l3 * mes;

    // 2. Update the estimated states:
    zhat_1 += zhat_dot_1 * dt;
    zhat_2 += zhat_dot_2 * dt;
    zhat_3 += zhat_dot_3 * dt;

    // 3. Compute control effort command (represents target PWM pulse in microseconds):
    float u = (kp * (ref - zhat_1) - kd * zhat_2 - zhat_3) / b0;

    // 4. Anti-Windup Clamping: Keep control effort within physical ESC boundaries (1000 - 2000 uS)
    if (u > 2000.0f) {
        u = 2000.0f;
    } else if (u < 1000.0f) {
        u = 1000.0f;
    }

    // Save state back to struct
    al->u_prev = u;
    al->zhat[0] = zhat_1;
    al->zhat[1] = zhat_2;
    al->zhat[2] = zhat_3;

    *command = u;

    return 0;
}

/**
 * @brief Tune and initialize the ADRC controller.
 * 
 * Computes bandwidth-parameterized discrete-to-continuous gains for the observer
 * and registers the interface pointers.
 * 
 * @param al Pointer to the adrc_control_law struct.
 * @param dt Sampling time step in seconds.
 * @param wo Observer bandwidth (rad/s).
 * @param b0 Controller input gain scaling factor.
 * @param kp Proportional gain.
 * @param kd Derivative gain.
 * @return int 0 on success, or -EINVAL if al is NULL.
 */
int adrc_control_law_tune(struct adrc_control_law *al, float dt, float wo, float b0, float kp, float kd)
{
    if(!al)
        return -EINVAL;

    al->interface.reset = reset;
    al->interface.set = set;
    al->interface.update = update;
    al->b0 = b0;
    al->kp = kp;
    al->kd = kd;

    // Compute continuous-time LESO observer gains:
    al->l[0] = 3.0f * wo;
    al->l[1] = 3.0f * wo * wo;
    al->l[2] = wo * wo * wo;

    return (control_law_reset(&al->interface));
}
