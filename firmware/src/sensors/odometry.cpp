#include "sensors/odometry.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

Odometry *Odometry::instance_ = nullptr;

Odometry::Odometry()
    : sensors_{
          {.gpio = HALL_SENSOR_1_PIN},
          {.gpio = HALL_SENSOR_2_PIN},
      },
      position_{0.0f, 0.0f, 0.0f}
{
    instance_ = this;
}

void Odometry::init()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        gpio_set_function(sensors_[i].gpio, GPIO_FUNC_SIO);
        gpio_set_dir(sensors_[i].gpio, GPIO_IN);
        gpio_pull_up(sensors_[i].gpio);
        gpio_set_input_hysteresis_enabled(sensors_[i].gpio, true);

        if (i == 0)
            gpio_set_irq_enabled_with_callback(sensors_[i].gpio, GPIO_IRQ_EDGE_FALL, true, &gpioCallback);
        else
            gpio_set_irq_enabled(sensors_[i].gpio, GPIO_IRQ_EDGE_FALL, true);
    }
}

void Odometry::gpioCallback(unsigned int gpio, uint32_t events)
{
    if (!instance_) return;
    uint64_t now = time_us_64();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (instance_->sensors_[i].gpio == gpio)
        {
            if (now - instance_->sensors_[i].last_edge_us >= DEBOUNCE_US)
            {
                instance_->sensors_[i].last_edge_us = now;
                instance_->sensors_[i].edge_count++;
            }
            break;
        }
    }
}

float Odometry::getSpeedKmh(int sensor_idx) const
{
    if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return 0.0f;
    return sensors_[sensor_idx].speed_kmh;
}

float Odometry::getDistanceCm(int sensor_idx) const
{
    if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return 0.0f;
    return sensors_[sensor_idx].distance_cm;
}

RobotPosition Odometry::getPosition() const
{
    return position_;
}

// Formulas (cf. https://fr.wikipedia.org/wiki/Odométrie):
//   Δs = (d_right + d_left) / 2
//   Δθ = (d_right - d_left) / e
//   x  += Δs · cos(θ + Δθ/2)
//   y  += Δs · sin(θ + Δθ/2)
//   θ  += Δθ
void Odometry::update()
{
    float delta_cm[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        uint32_t total     = sensors_[i].edge_count;
        uint32_t new_edges = total - sensors_[i].last_total;
        sensors_[i].last_total = total;

        float d = new_edges * (float)CM_PER_EDGE;
        sensors_[i].distance_cm += d;
        delta_cm[i] = d;

        sensors_[i].speed_kmh = new_edges * 50.0f * (float)CM_PER_EDGE * 0.036f;
    }

    float d_left  = delta_cm[0];
    float d_right = delta_cm[1];

    float delta_s     = (d_right + d_left) / 2.0f;
    float delta_theta = (d_right - d_left) / WHEEL_BASE;

    position_.x     += delta_s * cosf(position_.theta + delta_theta / 2.0f);
    position_.y     += delta_s * sinf(position_.theta + delta_theta / 2.0f);
    position_.theta += delta_theta;
}

void Odometry::task(void *param)
{
    Odometry *self = static_cast<Odometry *>(param);
    self->init();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        self->update();

        RobotPosition pos = self->getPosition();
        printf("x=%.1f y=%.1f theta=%.2f  L:%.2fkm/h R:%.2fkm/h\n",
               pos.x, pos.y, pos.theta,
               self->getSpeedKmh(0),
               self->getSpeedKmh(1));
    }
}
