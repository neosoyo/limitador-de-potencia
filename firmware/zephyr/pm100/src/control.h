#ifndef __CONTROL_H
#define __CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

#define CONTROL_US_PERIOD 500

int control_init(void);

float deltaTime(void);
int times(void);
#endif