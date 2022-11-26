#ifndef __UTILS_H__
#define __UTILS_H__

#include "stm32f10x.h"

#define SERVO_MIN_PWM	500
#define SERVO_MAX_PWM 2500
#define SERVO_MIN_DEG -90
#define SERVO_MAX_DEG 90

long map(long x, long in_min, long in_max, long out_min, long out_max);
int deg_to_ccr(long deg);
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
int32_t constrain(int32_t in_val, int32_t min_val, int32_t max_val);

#endif
