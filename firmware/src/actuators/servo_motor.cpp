#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include "actuators/servo_motor.h"

ServoMotor::ServoMotor(uint pin, float min, float max, float neutral) : _pin(pin), _neutral_angle(neutral), _min_angle(min), _max_angle(max)
{
}

void ServoMotor::init()
{
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(_pin);

    // Set PWM frequency to 50Hz
    pwm_set_clkdiv(slice_num, 125.0f); // 125 MHz / 125 = 1 MHz
    pwm_set_wrap(slice_num, 20000);    // 1 MHz / 20000 = 50 Hz

    pwm_set_enabled(slice_num, true);

    reset();
}

void ServoMotor::rotate(float position) const
{
    float angle = position_to_angle(position);
    rotate_deg(angle);
}

void ServoMotor::rotate_deg(float angle) const
{
    float pwm = angle_to_pwm(angle);
    pwm_set_gpio_level(_pin, pwm);
}

float ServoMotor::position_to_angle(float position) const
{
    if (position < 0)
        position = 0;
    else if (position > 1)
        position = 1;

    return _min_angle + position * (_max_angle - _min_angle);
}

int ServoMotor::angle_to_pwm(float angle) const
{
    // Map 0..180 => 1000..2000
    return 1000 + (angle / 180.0f) * 1000;
}

void ServoMotor::reset() const
{
    rotate(_neutral_angle);
}