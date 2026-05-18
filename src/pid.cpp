#include "pid.h"

PID::PID(float kp, float ki, float kd, float integral_max) : _kP(kp), _kI(ki), _kD(kd), _integral_max(integral_max)
{
}

float PID::compute(float error, float dt)
{
    // Terme proportionnel
    float P = -_kP * error;

    // Terme intégral avec anti-windup
    _integral += error * dt;
    if (_integral > _integral_max)
        _integral = _integral_max;
    if (_integral < -_integral_max)
        _integral = -_integral_max;
    float I = _kI * _integral;

    // Terme dérivé: evite le pic au premier echantillon apres reset
    float derivative = 0.0f;
    if (_has_prev_error && dt > 0.0f)
        derivative = (error - _prev_error) / dt;
    _prev_error = error;
    _has_prev_error = true;
    float D = _kD * derivative;

    return P + I + D;
}

void PID::reset()
{
    _integral = 0.0f;
    _prev_error = 0.0f;
    _has_prev_error = false;
}
