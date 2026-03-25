#include "pid.h"

void pid_init(PID* pid, float kp, float ki, float kd, float integral_max) {
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->integral_max = integral_max;
}

float pid_compute(PID* pid, float error, float dt) {
    // Terme proportionnel
    float P = pid->kp * error;

    // Terme intégral avec anti-windup
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I = pid->ki * pid->integral;

    // Terme dérivé
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error  = error;
    float D = pid->kd * derivative;

    return P + I + D;
}

void pid_reset(PID* pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}