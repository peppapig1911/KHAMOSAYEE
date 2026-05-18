#include "sensors/odometry.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#define POLL_INTERVAL_MS 20
// Minimum time between two valid edges (µs). Filters sensor noise/oscillation
// around the switching threshold. At 20 magnets and 50 km/h the real
// inter-edge period is ~1750 µs, so 500 µs leaves ample margin.
#define DEBOUNCE_US 500

// --- Internal sensor state (one entry per sensor) ---
typedef struct
{
    uint gpio;
    volatile uint32_t edge_count;   // valid magnet passes since boot, written by ISR
    volatile uint64_t last_edge_us; // timestamp of the last accepted edge
    uint32_t last_total;            // edge_count snapshot from previous poll window
    float distance_cm;              // accumulated distance (cm)
    float speed_kmh;                // last computed speed
} hall_sensor_t;

static hall_sensor_t sensors[] = {
    {.gpio = HALL_SENSOR_1_PIN},  // left wheel
    {.gpio = HALL_SENSOR_2_PIN},  // right wheel
};
#define NUM_SENSORS (sizeof(sensors) / sizeof(sensors[0]))

// Robot position estimated via differential-drive odometry
static robot_position_t position = {0.0f, 0.0f, 0.0f};

// Shared GPIO IRQ callback – counts edges, ignoring bursts within DEBOUNCE_US
static void hall_gpio_callback(uint gpio, uint32_t events)
{
    uint64_t now = time_us_64();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensors[i].gpio == gpio)
        {
            if (now - sensors[i].last_edge_us >= DEBOUNCE_US)
            {
                sensors[i].last_edge_us = now;
                sensors[i].edge_count++;
            }
            break;
        }
    }
}

void odometry_init(void)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        gpio_set_function(sensors[i].gpio, GPIO_FUNC_SIO);
        gpio_set_dir(sensors[i].gpio, GPIO_IN);
        gpio_pull_up(sensors[i].gpio);
        // Enable the built-in Schmitt trigger: adds voltage hysteresis so the
        // pin doesn't re-fire on slow or noisy signal transitions.
        gpio_set_input_hysteresis_enabled(sensors[i].gpio, true);

        // First sensor registers the shared callback; the rest reuse it.
        if (i == 0)
        {
            gpio_set_irq_enabled_with_callback(sensors[i].gpio,
                                               GPIO_IRQ_EDGE_FALL,
                                               true,
                                               &hall_gpio_callback);
        }
        else
        {
            gpio_set_irq_enabled(sensors[i].gpio,
                                 GPIO_IRQ_EDGE_FALL,
                                 true);
        }
    }
}

float odometry_get_speed_kmh(int sensor_idx)
{
    if (sensor_idx < 0 || sensor_idx >= (int)NUM_SENSORS)
        return 0.0f;

    return sensors[sensor_idx].speed_kmh;
}

float odometry_get_distance_cm(int sensor_idx)
{
    if (sensor_idx < 0 || sensor_idx >= (int)NUM_SENSORS)
        return 0.0f;

    return sensors[sensor_idx].distance_cm;
}

robot_position_t odometry_get_position(void)
{
    return position;
}

// Called every POLL_INTERVAL_MS ms: compute per-wheel deltas,
// then update robot (x, y, θ) using differential-drive odometry.
//
// Formulas (cf. https://fr.wikipedia.org/wiki/Odométrie):
//   Δs = (d_right + d_left) / 2
//   Δθ = (d_right - d_left) / e
//   x  += Δs · cos(θ + Δθ/2)
//   y  += Δs · sin(θ + Δθ/2)
//   θ  += Δθ
static void odometry_update(void)
{
    float delta_cm[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        uint32_t total = sensors[i].edge_count;
        uint32_t new_edges = total - sensors[i].last_total;
        sensors[i].last_total = total;

        float d = new_edges * (float)CM_PER_EDGE;
        sensors[i].distance_cm += d;
        delta_cm[i] = d;

        sensors[i].speed_kmh = new_edges * 50.0f * (float)CM_PER_EDGE * 0.036f;
    }

    // Differential-drive position update
    float d_left  = delta_cm[0];
    float d_right = delta_cm[1];

    float delta_s     = (d_right + d_left) / 2.0f;
    float delta_theta = (d_right - d_left) / WHEEL_BASE;

    position.x     += delta_s * cosf(position.theta + delta_theta / 2.0f);
    position.y     += delta_s * sinf(position.theta + delta_theta / 2.0f);
    position.theta += delta_theta;
}

void odometry_task(void *param)
{
    (void)param;
    odometry_init();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        odometry_update();

        robot_position_t pos = odometry_get_position();
        printf("x=%.1f y=%.1f theta=%.2f  L:%.2fkm/h R:%.2fkm/h\n",
               pos.x, pos.y, pos.theta,
               odometry_get_speed_kmh(0),
               odometry_get_speed_kmh(1));
    }
}
