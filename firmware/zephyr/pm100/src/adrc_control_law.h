#ifndef __ADRC_CONTROL_LAW_H
#define __ADRC_CONTROL_LAW_H

#include "control_law.h"

struct adrc_control_law {
    struct siso_control_law interface;
    float b0;
    float kp;
    float kd;
    float l[3];
    float zhat[3];
    float error;
    float last_time;
    float dt;
    float target_power;
    float current_power;
    float control_effort;
    int u_prev;
    int ticks;
    int sample_ratio;
    int reading_error;
    int writing_error;
};

static inline struct siso_control_law * get_adrc_control_law_interface(struct adrc_control_law *al)
{
    if(al) {
        return &al->interface;
    }

    return NULL;
}

int adrc_control_law_tune(struct adrc_control_law *al, float dt, float wo, float b0, float kp, float kd);

#endif