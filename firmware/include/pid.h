#pragma once
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