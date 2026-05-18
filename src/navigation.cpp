#include "navigation.h"
#include <stdio.h>
#include <cmath>

static PID pid_cap(0.005f, 0.0f, 0.001f, 0.5f);
static const int steering_sign = 1;
static bool has_prev_err = false;
static float prev_err_unwrapped = 0.0f;

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

    // Calcul de l'erreur
    float err_wrapped = calculate_heading_error(nav.cap, target_heading);
    float err = unwrap_error(err_wrapped);

    // Zone morte pour éviter le tremblement du servo
    if (err > -DEADBAND && err < DEADBAND)
    {
        front_wheel.reset();
        pid_cap.reset();
        has_prev_err = false;
        printf("Cible:%.1f | Cap:%.1f | Err:%.1f | DEADBAND (Tout droit)\n", target_heading, nav.cap, err);
    }
    else
    {
        float correction = steering_sign * pid_cap.compute(err, DT);

        // Borner la correction, calcul du PID (ex: max ±1 pour une correction max)
        float clamped = correction;
        if (clamped > 1.0f)  clamped = 1.0f;
        if (clamped < -1.0f) clamped = -1.0f;

        // 0..1 centré sur 0.5
        float position = 0.5f + clamped * 0.5f;
        //Commande du servo
        front_wheel.rotate(position);
        
         printf("Cible:%.1f | Cap:%.1f | Err:%+.1f | Corr:%+.1f | ServoPos:%.2f\n", 
             target_heading, nav.cap, err, correction, position);
    }
}