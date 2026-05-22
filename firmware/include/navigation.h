#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors/cmps12.h"
#include "actuators/servo_motor.h"
#include "pid.h"

#define FRONT_WHEEL_MIN_DEG 40.0f
#define FRONT_WHEEL_MAX_DEG 160.0f
#define FRONT_WHEEL_CENTER_DEG 133.0f

#define DT 0.15f
#define DEADBAND 4.0f

void navigation_init();
void navigation_set_center_deg(float center_deg);

float calculate_heading_error(float current_heading, float target_heading);

void steer_to_heading(float target_heading, CMPS12& compass, ServoMotor& front_wheel);

#endif