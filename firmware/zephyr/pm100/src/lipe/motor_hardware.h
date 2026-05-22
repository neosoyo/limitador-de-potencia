#ifndef __MOTOR_HARDWARE_H
#define __MOTOR_HARDWARE_H

#include <stddef.h>
#include <errno.h>
#include <math.h>

struct motor_hardware_if {
    int (*get_power) (struct motor_hardware_if *self, float *power);
    int (*set_throtle) (struct motor_hardware_if *self, float throtle);
};

static inline float motor_hardware_get_power(struct motor_hardware_if *mh, float *power)
{
    if(!mh)
        return -EINVAL;

    if(mh->get_power) {
        return mh->get_power(mh, power);
    }

    return -ENOTSUP;
}

static inline float motor_hardware_set_throtle(struct motor_hardware_if *mh, float throtle)
{
    if(!mh)
        return -EINVAL;

    if(mh->get_power) {
        return mh->set_throtle(mh, throtle);
    }

    return -ENOTSUP;
}

//extract derived motor hw class from this base class:
#define MH_CONTAINER_OF(ptr, type, field)  ((type *)(((char *)(ptr)) - offsetof(type, field)))

#endif