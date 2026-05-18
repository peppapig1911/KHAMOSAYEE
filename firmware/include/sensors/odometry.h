#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// --- Wheel / sensor constants ---
#define HALL_SENSOR_1_PIN 2  // left wheel
#define HALL_SENSOR_2_PIN 3  // right wheel

#define WHEEL_DIAMETER 15.5f // cm
#define WHEEL_MAGNET_COUNT 20
// Distance (cm) covered each time one magnet passes the sensor
#define CM_PER_EDGE (3.14159265358979323846 * WHEEL_DIAMETER / WHEEL_MAGNET_COUNT)

// Wheelbase: distance between the two wheels (cm)
#define WHEEL_BASE 45.0f

// Robot position in the 2D plane
typedef struct
{
    float x;     // cm
    float y;     // cm
    float theta; // radians
} robot_position_t;

// Initialise GPIO pins and register interrupts for all sensors.
// Call once before starting the scheduler (or at the top of odometry_task).
void odometry_init(void);

// Returns speed in km/h computed over the last 100 ms poll window.
// Returns 0 until the first window elapses.
float odometry_get_speed_kmh(int sensor_idx);

// Returns total distance travelled in cm for sensor index i.
float odometry_get_distance_cm(int sensor_idx);

// Returns the current estimated robot position.
robot_position_t odometry_get_position(void);

// FreeRTOS task: calls odometry_init() then prints speed/distance every 100 ms.
void odometry_task(void *param);

#ifdef __cplusplus
}
#endif
