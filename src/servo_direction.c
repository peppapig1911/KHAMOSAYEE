#include "servo_direction.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

void servo_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLKDIV);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(SERVO_PIN, SERVO_MID);
}

void servo_set_angle(float angle) {
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    uint16_t val = SERVO_MIN + (uint16_t)(angle * (SERVO_MAX - SERVO_MIN) / 180.0f);
    pwm_set_gpio_level(SERVO_PIN, val);
}