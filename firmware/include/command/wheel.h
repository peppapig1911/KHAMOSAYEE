#pragma once

#include "sensors/cmps12.h"

bool wheel_parse_manual_angle_command(const char *input, float &angle_deg);
bool wheel_parse_center_command(const char *input, float &center_deg);
bool wheel_parse_direct_degree_input(const char *input, float &angle_deg);

void wheel_init();
void wheel_set_center_deg(float center_deg);
float wheel_get_center_deg();
void wheel_rotate_deg(float angle_deg);
void wheel_rotate_to_center();

float wheel_compute_bearing_deg(float dx, float dy);
void wheel_steer_to_heading(float heading_deg, CMPS12 &cmps12);
