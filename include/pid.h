#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_max;
} PID;

void  pid_init(PID* pid, float kp, float ki, float kd, float integral_max);
float pid_compute(PID* pid, float error, float dt);
void  pid_reset(PID* pid);

#endif