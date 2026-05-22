#include <cstring>
#include <cstdio>
#include <cmath>
#include "log.h"
#include "command/sail.h"
#include "actuators/servo_motor.h"

static constexpr const char *MODULE = "Sail";
static ServoMotor sail(7, 0, 250);
static float s_last_auto_angle = -1000.0f;

void sail_init(void) {
    sail.init();
}

bool sail_parse_console_command(const char *input, float *angle_deg)
{
    float requested = 0.0f;
    if (sscanf(input, "sail %f", &requested) != 1)
        return false;
        
    if (requested < 0.0f)   requested = 0.0f;
    if (requested > 180.0f) requested = 180.0f;
    
    *angle_deg = requested;
    return true;
}

float sail_angle_from_wind(float wind_direction_deg)
{
    float wind = wind_direction_deg;
    while (wind < 0.0f)   wind += 360.0f;
    while (wind >= 360.0f) wind -= 360.0f;

    float beta = wind;
    if (beta > 180.0f)
        beta = 360.0f - beta;

    // Zone morte : moins de 30° du vent → voile complètement dans l'axe (bordée à fond)
    if (beta < 30.0f)
    {
        LOGD(MODULE, "Voile : zone creuse (beta=%.1f), voile fermee", beta);
        return 180.0f; 
    }

    float ratio = (beta - 30.0f) / 150.0f; 
    if (ratio > 1.0f) ratio = 1.0f;

    // 180 (fermé) → 0 (ouvert à 90°)
    float angle = 180.0f - (ratio * 180.0f);
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;
    return angle;
}

void sail_set_angle_from_wind(float wind_direction_deg) {
    const float target_angle = sail_angle_from_wind(wind_direction_deg);

    // Evite de spammer le servo pour des variations tres faibles
    if (fabsf(target_angle - s_last_auto_angle) < 1.0f)
        return;

    sail.rotate_deg(target_angle);
    s_last_auto_angle = target_angle;
    LOGI(MODULE, "AUTO wind=%.1f deg -> sail=%.1f deg", wind_direction_deg, target_angle);
}

void sail_set_manual_angle(float angle_deg) {
    if (angle_deg < 0.0f)
        angle_deg = 0.0f;
    if (angle_deg > 180.0f)
        angle_deg = 180.0f;
    sail.rotate_deg(angle_deg);
    s_last_auto_angle = angle_deg;
}