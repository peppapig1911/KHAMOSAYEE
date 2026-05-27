#pragma once

#include <stdint.h>

#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"

enum class NavigationState : uint8_t
{
    IDLE = 0,
    NAVIGATE = 1,
    TACKING = 2,
    ARRIVED = 3,
};

enum class ControlMode : uint8_t
{
    AUTOMATIC = 0,
    MANUAL = 1,
};

struct NavigationTelemetry
{
    NavigationState state = NavigationState::IDLE;

    float x = 0.0f;
    float y = 0.0f;
    float psi = 0.0f;
    float psi_dot = 0.0f;
    float delta_deg = 0.0f;
    float theta_s_deg = 0.0f;
    float speed_mps = 0.0f;
    float distance_m = 0.0f;
    float bearing_deg = 0.0f;
    float beta_deg = 0.0f;
    float wind_dir_deg = 0.0f;
    float wind_speed_mps = 0.0f;
    float target_x = 0.0f;
    float target_y = 0.0f;
    bool has_target = false;
};

void controller_init();
bool controller_start(ZedF9P &gps, CMPS12 &imu);
void controller_stop();
void controller_set_mode(ControlMode mode);
ControlMode controller_get_mode();

void controller_set_target_waypoint(float x_B, float y_B);
bool controller_set_target_waypoint_gps(double latitude_deg, double longitude_deg);
void controller_update_wind(float alpha_w_deg, float wind_speed_mps);

NavigationState controller_get_state();
NavigationTelemetry controller_get_telemetry();
bool controller_is_running();