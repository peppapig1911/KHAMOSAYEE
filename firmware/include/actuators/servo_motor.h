#pragma once

#include <stdlib.h>

class ServoMotor
{
public:
    ServoMotor(uint pin, float min = 0, float max = 180, float neutral = 0.5);

    void init();

    // Position 0..1
    void rotate(float position) const;
    void rotate_deg(float angle) const;

    void reset() const;

private:
    float position_to_angle(float position) const;
    int angle_to_pwm(float angle) const;

private:
    uint _pin;
    float _neutral_angle, _min_angle, _max_angle;
};