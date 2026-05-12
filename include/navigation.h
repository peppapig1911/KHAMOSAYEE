#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors/cmps12.h"
#include "actuators/servo_motor.h"
#include "pid.h"


#define DT 0.15f
#define DEADBAND 2.0f

void navigation_init();

float calculate_heading_error(float current_heading, float target_heading);

void steer_to_heading(float target_heading, CMPS12& compass, ServoMotor& front_wheel);

#endif