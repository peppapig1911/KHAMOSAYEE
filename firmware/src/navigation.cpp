#include "navigation.h"
#include <stdio.h>
#include <cmath>

static PID pid_cap(0.0018f, 0.0f, 0.0002f, 0.5f);
static const int steering_sign = 1;
static float steering_center_deg = FRONT_WHEEL_CENTER_DEG;
static bool has_prev_err = false;
static float prev_err_unwrapped = 0.0f;
static bool has_prev_servo_deg = false;
static float prev_servo_deg = FRONT_WHEEL_CENTER_DEG;
static bool has_filtered_err = false;
static float filtered_err = 0.0f;
static constexpr float ERR_FILTER_ALPHA = 0.18f;
static constexpr float MAX_SERVO_STEP_DEG = 1.2f;
static constexpr float MAX_CORRECTION = 0.30f;
static constexpr float MAX_STEER_DELTA_DEG = 12.0f;

static float smooth_servo_angle(float target_deg)
{
    if (!has_prev_servo_deg)
    {
        prev_servo_deg = target_deg;
        has_prev_servo_deg = true;
        return target_deg;
    }

    float delta = target_deg - prev_servo_deg;
    if (delta > MAX_SERVO_STEP_DEG)
        delta = MAX_SERVO_STEP_DEG;
    else if (delta < -MAX_SERVO_STEP_DEG)
        delta = -MAX_SERVO_STEP_DEG;

    prev_servo_deg += delta;
    return prev_servo_deg;
}

static float unwrap_error(float wrapped_err)
{
    float err = wrapped_err;

    if (has_prev_err)
    {
        while (err - prev_err_unwrapped > 180.0f)
            err -= 360.0f;
        while (err - prev_err_unwrapped < -180.0f)
            err += 360.0f;
    }

    prev_err_unwrapped = err;
    has_prev_err = true;
    return err;
}

void navigation_init() 
{
    pid_cap.reset();
    has_prev_err = false;
    prev_err_unwrapped = 0.0f;
    has_prev_servo_deg = false;
    prev_servo_deg = steering_center_deg;
    has_filtered_err = false;
    filtered_err = 0.0f;
}

void navigation_set_center_deg(float center_deg)
{
    if (center_deg < FRONT_WHEEL_MIN_DEG)
        center_deg = FRONT_WHEEL_MIN_DEG;
    else if (center_deg > FRONT_WHEEL_MAX_DEG)
        center_deg = FRONT_WHEEL_MAX_DEG;

    steering_center_deg = center_deg;
    has_prev_servo_deg = false;
    prev_servo_deg = steering_center_deg;
}

float calculate_heading_error(float current_heading, float target_heading)
{
    float err = target_heading - current_heading;
    
    // Normalisation pour toujours tourner du bon côté
    while (err > 180.0f)  err -= 360.0f;
    while (err <= -180.0f) err += 360.0f;
    
    return err;
}

void steer_to_heading(float target_heading, CMPS12& compass, ServoMotor& front_wheel)
{
    auto nav = compass.navigation();

    if (std::isnan(nav.cap))
    {
        has_prev_err = false;
        printf("CMPS12 non connecte\n");
        return;
    }

    // Calcul de l'erreur entre -180 et +180
    float err_wrapped = calculate_heading_error(nav.cap, target_heading);
    float err = unwrap_error(err_wrapped);

    if (!has_filtered_err)
    {
        filtered_err = err;
        has_filtered_err = true;
    }
    else
    {
        filtered_err += ERR_FILTER_ALPHA * (err - filtered_err);
    }

    // ZONE MORTE CORRIGÉE : On ne détruit plus les variables d'état (pas de reset agressif)
    if (filtered_err > -DEADBAND && filtered_err < DEADBAND)
    {
        // On force la roue au centre de manière douce via la rampe
        float servo_deg = smooth_servo_angle(steering_center_deg);
        front_wheel.rotate_deg(servo_deg);
        
        // On reset le terme intégral du PID si ton PID en a un, pour éviter l'effet "windup"
        pid_cap.reset(); 

        printf("Cible:%.1f | Cap:%.1f | ErrF:%.1f | DEADBAND (Tout droit doux)\n", target_heading, nav.cap, filtered_err);
    }
    else
    {
        float correction = steering_sign * pid_cap.compute(filtered_err, DT);

        // Borner la correction
        float clamped = correction;
        if (clamped > MAX_CORRECTION)  clamped = MAX_CORRECTION;
        if (clamped < -MAX_CORRECTION) clamped = -MAX_CORRECTION;

        float target_deg = steering_center_deg + clamped * MAX_STEER_DELTA_DEG;
        if (target_deg < FRONT_WHEEL_MIN_DEG)
            target_deg = FRONT_WHEEL_MIN_DEG;
        else if (target_deg > FRONT_WHEEL_MAX_DEG)
            target_deg = FRONT_WHEEL_MAX_DEG;

        float servo_deg = smooth_servo_angle(target_deg);
        front_wheel.rotate_deg(servo_deg);
        
        printf("Cible:%.1f | Cap:%.1f | Err:%+.1f | ErrF:%+.1f | Corr:%+.2f | ServoDeg:%.1f\n", 
             target_heading, nav.cap, err, filtered_err, correction, servo_deg);
    }
}