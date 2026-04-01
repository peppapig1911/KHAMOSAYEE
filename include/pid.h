// #ifndef PID_H
// #define PID_H

// typedef struct
// {
//     float kp, ki, kd;
//     float integral;
//     float prev_error;
//     float integral_max;
// } PID;

// void pid_init(PID *pid, float kp, float ki, float kd, float integral_max);
// float pid_compute(PID *pid, float error, float dt);
// void pid_reset(PID *pid);

// #endif

class PID
{
public:
    PID(float kp, float ki, float kd, float integral_max);

    float compute(float error, float dt);

    void reset();

private:
    float _kP, _kI, _kD, _integral_max;
    float _prev_error = 0, _integral = 0;
};