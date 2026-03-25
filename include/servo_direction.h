#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_PIN       6
#define PWM_CLKDIV      64.0f
#define PWM_WRAP        39062
#define SERVO_MIN       1953
#define SERVO_MID       2929
#define SERVO_MAX       3906

void  servo_init();
void  servo_set_angle(float angle);

#endif