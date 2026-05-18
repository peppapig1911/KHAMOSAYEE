#pragma once

#include <stdint.h>

#define HALL_SENSOR_1_PIN  2
#define HALL_SENSOR_2_PIN  3
#define WHEEL_DIAMETER     15.5f
#define WHEEL_MAGNET_COUNT 20
#define CM_PER_EDGE        (3.14159265358979323846f * WHEEL_DIAMETER / WHEEL_MAGNET_COUNT)
#define WHEEL_BASE         45.0f

struct RobotPosition {
    float x;
    float y;
    float theta;
};

class Odometry {
public:
    Odometry();

    float         getSpeedKmh(int sensor_idx) const;
    float         getDistanceCm(int sensor_idx) const;
    RobotPosition getPosition() const;

    // FreeRTOS task entry point — pass an Odometry* as param
    static void task(void *param);

private:
    static constexpr uint32_t DEBOUNCE_US = 500;
    static constexpr int      POLL_MS     = 20;
    static constexpr int      NUM_SENSORS = 2;

    struct HallSensor {
        uint32_t            gpio;
        volatile uint32_t   edge_count;
        volatile uint64_t   last_edge_us;
        uint32_t            last_total;
        float               distance_cm;
        float               speed_kmh;
    };

    HallSensor    sensors_[NUM_SENSORS];
    RobotPosition position_;

    void init();
    void update();

    static void     gpioCallback(unsigned int gpio, uint32_t events);
    static Odometry *instance_;
};
