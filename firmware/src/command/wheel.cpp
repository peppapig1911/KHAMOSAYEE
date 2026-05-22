#include "command/wheel.h"

#include "actuators/servo_motor.h"
#include "navigation.h"

#include <cmath>
#include <cstdio>

static float s_front_wheel_center_deg = FRONT_WHEEL_CENTER_DEG;
static constexpr float FRONT_WHEEL_NEUTRAL_POS =
	(FRONT_WHEEL_CENTER_DEG - FRONT_WHEEL_MIN_DEG) / (FRONT_WHEEL_MAX_DEG - FRONT_WHEEL_MIN_DEG);

static ServoMotor s_front_wheel(6, FRONT_WHEEL_MIN_DEG, FRONT_WHEEL_MAX_DEG, FRONT_WHEEL_NEUTRAL_POS);

bool wheel_parse_manual_angle_command(const char *input, float &angle_deg)
{
	float requested_angle = 0.0f;

	if (sscanf(input, "a %f", &requested_angle) != 1)
		return false;

	if (requested_angle < 0.0f)
		requested_angle = 0.0f;
	if (requested_angle > 180.0f)
		requested_angle = 180.0f;

	angle_deg = requested_angle;
	return true;
}

bool wheel_parse_center_command(const char *input, float &center_deg)
{
	float requested_center = 0.0f;

	if (sscanf(input, "center %f", &requested_center) != 1)
		return false;

	if (requested_center < FRONT_WHEEL_MIN_DEG)
		requested_center = FRONT_WHEEL_MIN_DEG;
	if (requested_center > FRONT_WHEEL_MAX_DEG)
		requested_center = FRONT_WHEEL_MAX_DEG;

	center_deg = requested_center;
	return true;
}

bool wheel_parse_direct_degree_input(const char *input, float &angle_deg)
{
	float requested_angle = 0.0f;

	if (sscanf(input, "%f", &requested_angle) != 1)
		return false;

	if (requested_angle < FRONT_WHEEL_MIN_DEG || requested_angle > FRONT_WHEEL_MAX_DEG)
		return false;

	angle_deg = requested_angle;
	return true;
}

void wheel_init()
{
	s_front_wheel.init();
}

void wheel_set_center_deg(float center_deg)
{
	if (center_deg < FRONT_WHEEL_MIN_DEG)
		center_deg = FRONT_WHEEL_MIN_DEG;
	if (center_deg > FRONT_WHEEL_MAX_DEG)
		center_deg = FRONT_WHEEL_MAX_DEG;

	s_front_wheel_center_deg = center_deg;
	navigation_set_center_deg(s_front_wheel_center_deg);
	s_front_wheel.rotate_deg(s_front_wheel_center_deg);
}

float wheel_get_center_deg()
{
	return s_front_wheel_center_deg;
}

void wheel_rotate_deg(float angle_deg)
{
	s_front_wheel.rotate_deg(angle_deg);
}

void wheel_rotate_to_center()
{
	s_front_wheel.rotate_deg(s_front_wheel_center_deg);
}

float wheel_compute_bearing_deg(float dx, float dy)
{
	float bearing = atan2f(dx, dy) * 180.0f / M_PI;

	if (bearing < 0.0f)
		bearing += 360.0f;
	if (bearing >= 360.0f)
		bearing -= 360.0f;

	return bearing;
}

void wheel_steer_to_heading(float heading_deg, CMPS12 &cmps12)
{
	steer_to_heading(heading_deg, cmps12, s_front_wheel);
}
