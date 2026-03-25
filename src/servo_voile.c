#include "servo_voile.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

void servo_voile_init() {
    gpio_set_function(SERVO_VOILE_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_VOILE_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLKDIV_VOILE);
    pwm_config_set_wrap(&config, PWM_WRAP_VOILE);
    pwm_init(slice, &config, true);
    // Position fermée au démarrage
    servo_voile_set_angle(VOILE_FERMEE);
}

void servo_voile_set_angle(float angle) {
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    uint16_t val = SERVO_VOILE_MIN + (uint16_t)(angle * (SERVO_VOILE_MAX - SERVO_VOILE_MIN) / 180.0f);
    pwm_set_gpio_level(SERVO_VOILE_PIN, val);
}