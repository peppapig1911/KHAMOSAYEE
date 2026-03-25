#ifndef SERVO_VOILE_H
#define SERVO_VOILE_H

#define SERVO_VOILE_PIN     7
#define PWM_CLKDIV_VOILE    64.0f
#define PWM_WRAP_VOILE      39062
#define SERVO_VOILE_MIN     1953    // 0°
#define SERVO_VOILE_MID     2929    // 90°
#define SERVO_VOILE_MAX     3906    // 180°

// Positions prédéfinies
#define VOILE_FERMEE        0.0f    // voile repliée
#define VOILE_MI_OUVERTE    90.0f   // voile à moitié
#define VOILE_OUVERTE       180.0f  // voile déployée

void  servo_voile_init();
void  servo_voile_set_angle(float angle);

#endif