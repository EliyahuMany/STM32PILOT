#include "utils.h"



long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int deg_to_ccr(long deg)
{
	return map(deg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_PWM, SERVO_MAX_PWM);
}
