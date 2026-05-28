#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"

#include "actuators/servo_motor.h"
#include "comms/ble_server.h"

#include <algorithm>
#include <cmath>

static constexpr const char *MODULE = "NavCtrl";

static constexpr float CONTROL_LOOP_PERIOD_MS = 100.0f;
static constexpr float Kp = 0.85f;
static constexpr float Kd = 0.02f;
static constexpr float NO_GO_ANGLE_DEG = 40.0f;
static constexpr float NO_GO_HYSTERESIS_DEG = 45.0f;
static constexpr float DELTA_MAX_DEG = 30.0f;
static constexpr float THETA_S_MAX_DEG = 180.0f;
static constexpr float ARRIVAL_THRESHOLD_M = 0.75f;

static constexpr float FRONT_WHEEL_MIN_DEG = 40.0f;
static constexpr float FRONT_WHEEL_MAX_DEG = 160.0f;

static ZedF9P *s_gps = nullptr;
static CMPS12 *s_imu = nullptr;
static NavigationTelemetry s_telemetry{};
static volatile bool s_running = false;
static volatile ControlMode s_mode = ControlMode::AUTOMATIC;
static bool s_task_started = false;
static bool s_has_projection_origin = false;
static double s_origin_lat = 0.0;
static double s_origin_lon = 0.0;
static bool s_has_pending_waypoint = false;
static double s_pending_waypoint_lat = 0.0;
static double s_pending_waypoint_lon = 0.0;

static ServoMotor sail(7, 0, 175, 0);
static ServoMotor wheel(6, 77, 117 + 40);

static float wrap_180(float angle_deg)
{
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    while (angle_deg <= -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

static float wrap_360(float angle_deg)
{
    while (angle_deg < 0.0f)
        angle_deg += 360.0f;
    while (angle_deg >= 360.0f)
        angle_deg -= 360.0f;
    return angle_deg;
}

static float clamp_float(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

static float signed_gyro_z_dps(const RawData &raw)
{
    return static_cast<float>(static_cast<int16_t>(raw.gyro.z));
}

static bool project_gps_to_local_xy(const GNSSData &gnss, float &x, float &y)
{
    if (!gnss.valid)
        return false;

    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_RADIUS_M = 6378137.0;

    if (!s_has_projection_origin)
    {
        if (gnss.fix_type < 3)
            return false;

        s_origin_lat = gnss.lat;
        s_origin_lon = gnss.lon;
        s_has_projection_origin = true;
        LOGI(MODULE, "Origine GPS fixee : lat=%.7f lon=%.7f", gnss.lat, gnss.lon);
    }

    const double dlat = (gnss.lat - s_origin_lat) * DEG_TO_RAD;
    const double dlon = (gnss.lon - s_origin_lon) * DEG_TO_RAD;
    const double origin_lat_rad = s_origin_lat * DEG_TO_RAD;

    x = static_cast<float>(EARTH_RADIUS_M * dlon * std::cos(origin_lat_rad));
    y = static_cast<float>(EARTH_RADIUS_M * dlat);
    return true;
}

static bool project_lat_lon_to_local_xy(double latitude_deg, double longitude_deg, float &x, float &y)
{
    if (!s_has_projection_origin)
        return false;

    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_RADIUS_M = 6378137.0;

    const double dlat = (latitude_deg - s_origin_lat) * DEG_TO_RAD;
    const double dlon = (longitude_deg - s_origin_lon) * DEG_TO_RAD;
    const double origin_lat_rad = s_origin_lat * DEG_TO_RAD;

    x = static_cast<float>(EARTH_RADIUS_M * dlon * std::cos(origin_lat_rad));
    y = static_cast<float>(EARTH_RADIUS_M * dlat);
    return true;
}

static void apply_pending_waypoint_if_ready()
{
    if (!s_has_pending_waypoint)
        return;

    float target_x = 0.0f;
    float target_y = 0.0f;
    if (!project_lat_lon_to_local_xy(s_pending_waypoint_lat, s_pending_waypoint_lon, target_x, target_y))
        return;

    s_telemetry.target_x = target_x;
    s_telemetry.target_y = target_y;
    s_has_pending_waypoint = false;

    LOGI(MODULE, "Waypoint appliquee: lat=%.7f lon=%.7f -> x=%.2f y=%.2f",
         s_pending_waypoint_lat,
         s_pending_waypoint_lon,
         target_x,
         target_y);
}

static void park_servos()
{
    wheel.reset();
    sail.reset();
}

static void apply_controls(float delta_deg, float theta_s_deg)
{
    // Use the servo's configured min/max angles and drive servos via .rotate(position)
    static constexpr float WHEEL_SERVO_MIN_DEG = 77.0f;
    static constexpr float WHEEL_SERVO_MAX_DEG = 157.0f; // 117 + 40 from constructor
    static constexpr float SAIL_SERVO_MIN_DEG = 0.0f;
    static constexpr float SAIL_SERVO_MAX_DEG = 175.0f;

    // Compute targets and clamp to servo physical range
    const float wheel_center_deg = (WHEEL_SERVO_MIN_DEG + WHEEL_SERVO_MAX_DEG) * 0.5f;
    const float wheel_target_deg = clamp_float(wheel_center_deg + delta_deg, WHEEL_SERVO_MIN_DEG, WHEEL_SERVO_MAX_DEG);
    const float sail_target_deg = clamp_float(theta_s_deg, SAIL_SERVO_MIN_DEG, SAIL_SERVO_MAX_DEG);

    // Update telemetry
    s_telemetry.delta_deg = delta_deg;
    s_telemetry.theta_s_deg = sail_target_deg;

    // Convert to normalized 0..1 positions and apply using .rotate()
    const float wheel_pos = clamp_float((wheel_target_deg - WHEEL_SERVO_MIN_DEG) / (WHEEL_SERVO_MAX_DEG - WHEEL_SERVO_MIN_DEG), 0.0f, 1.0f);
    const float sail_pos = clamp_float((sail_target_deg - SAIL_SERVO_MIN_DEG) / (SAIL_SERVO_MAX_DEG - SAIL_SERVO_MIN_DEG), 0.0f, 1.0f);

    wheel.rotate(wheel_pos);
    sail.rotate(sail_pos);
}

static void controller_task(void *param)
{
    (void)param;

    NavigationState last_state = NavigationState::IDLE;

    while (true)
    {
        ControlMode mode;
        taskENTER_CRITICAL();
        mode = s_mode;
        taskEXIT_CRITICAL();

        if (mode == ControlMode::MANUAL)
        {
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            LOGD(MODULE, "Manual mode: %d sail angle | %d wheel angle", (int)get_current_sail_opening_pct(), (int)get_current_front_wheel_offset());
            sail.rotate(static_cast<float>(get_current_sail_opening_pct()) / 100.0f);
            wheel.rotate(static_cast<float>(get_current_front_wheel_offset()) / 100.0f);
            continue;
        }

        if (!s_running)
        {
            if (s_telemetry.state != NavigationState::IDLE)
                s_telemetry.state = NavigationState::IDLE;

            park_servos();
            last_state = NavigationState::IDLE;
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        if (s_gps == nullptr || s_imu == nullptr)
        {
            if (last_state != NavigationState::IDLE)
            {
                LOGI(MODULE, "Capteurs navigation non initialises.");
                last_state = NavigationState::IDLE;
            }

            park_servos();
            s_telemetry.state = NavigationState::IDLE;
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        s_gps->update();
        const GNSSData &gnss = s_gps->data();

        float x = 0.0f;
        float y = 0.0f;
        if (!project_gps_to_local_xy(gnss, x, y))
        {
            if (last_state != NavigationState::IDLE)
            {
                LOGI(MODULE, "Position GPS indisponible: attente d'un fix valide.");
                last_state = NavigationState::IDLE;
            }

            s_telemetry.state = NavigationState::IDLE;
            park_servos();
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        const NavData nav = s_imu->navigation();
        if (std::isnan(nav.cap))
        {
            if (last_state != NavigationState::IDLE)
            {
                LOGI(MODULE, "Capteur IMU indisponible.");
                last_state = NavigationState::IDLE;
            }

            s_telemetry.state = NavigationState::IDLE;
            park_servos();
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        apply_pending_waypoint_if_ready();

        const RawData raw = s_imu->raw();

        float alpha_w_deg = 0.0f;
        float wind_speed_mps = 0.0f;
        taskENTER_CRITICAL();
        alpha_w_deg = s_telemetry.wind_dir_deg;
        wind_speed_mps = s_telemetry.wind_speed_mps;
        taskEXIT_CRITICAL();

        const float psi = wrap_360(nav.cap);
        const float psi_dot = signed_gyro_z_dps(raw);
        const float beta = wrap_180(alpha_w_deg - psi);

        const float dx = s_telemetry.target_x - x;
        const float dy = s_telemetry.target_y - y;
        const float distance_to_target = std::sqrt(dx * dx + dy * dy);
        const float psi_desired = wrap_360(std::atan2(dx, dy) * 180.0f / static_cast<float>(M_PI));
        const float heading_error = wrap_180(psi_desired - psi);

        s_telemetry.x = x;
        s_telemetry.y = y;
        s_telemetry.psi = psi;
        s_telemetry.psi_dot = psi_dot;
        s_telemetry.speed_mps = gnss.speed;
        s_telemetry.distance_m = distance_to_target;
        s_telemetry.bearing_deg = psi_desired;
        s_telemetry.beta_deg = beta;
        s_telemetry.wind_dir_deg = alpha_w_deg;
        s_telemetry.wind_speed_mps = wind_speed_mps;

        if (!s_telemetry.has_target)
        {
            s_telemetry.state = NavigationState::IDLE;
            if (last_state != NavigationState::IDLE)
            {
                LOGI(MODULE, "Aucune cible definie.");
                last_state = NavigationState::IDLE;
            }

            park_servos();
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        if (distance_to_target < ARRIVAL_THRESHOLD_M)
        {
            s_telemetry.state = NavigationState::ARRIVED;
            if (last_state != NavigationState::ARRIVED)
            {
                LOGI(MODULE, "Cible atteinte (dist=%.2f m).", distance_to_target);
                last_state = NavigationState::ARRIVED;
            }

            park_servos();
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
            continue;
        }

        if (s_telemetry.state == NavigationState::ARRIVED)
            s_telemetry.state = NavigationState::NAVIGATE;

        if (s_telemetry.state == NavigationState::TACKING)
        {
            if (std::fabs(beta) > NO_GO_HYSTERESIS_DEG)
                s_telemetry.state = NavigationState::NAVIGATE;
        }
        else if (std::fabs(beta) < NO_GO_ANGLE_DEG)
        {
            s_telemetry.state = NavigationState::TACKING;
        }
        else
        {
            s_telemetry.state = NavigationState::NAVIGATE;
        }

        if (s_telemetry.state == NavigationState::TACKING)
        {
            const float delta = (beta >= 0.0f) ? -DELTA_MAX_DEG : DELTA_MAX_DEG;
            if (last_state != NavigationState::TACKING)
            {
                LOGI(MODULE, "TACKING | beta=%.1f deg | delta=%.1f deg", beta, delta);
                last_state = NavigationState::TACKING;
            }

            apply_controls(delta, THETA_S_MAX_DEG);
        }
        else
        {
            const float delta = clamp_float(Kp * heading_error + Kd * psi_dot, -DELTA_MAX_DEG, DELTA_MAX_DEG);
            const float theta_s = clamp_float(std::fabs(beta) * 0.5f, 0.0f, THETA_S_MAX_DEG);

            if (last_state != NavigationState::NAVIGATE)
            {
                LOGI(MODULE, "NAVIGATE | psi=%.1f target=%.1f err=%.1f beta=%.1f", psi, psi_desired, heading_error, beta);
                last_state = NavigationState::NAVIGATE;
            }

            apply_controls(delta, theta_s);
        }

        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(CONTROL_LOOP_PERIOD_MS)));
    }
}

void controller_init()
{
    taskENTER_CRITICAL();
    s_telemetry = NavigationTelemetry{};
    s_running = false;
    s_mode = ControlMode::AUTOMATIC;
    s_has_projection_origin = false;
    s_origin_lat = 0.0;
    s_origin_lon = 0.0;
    sail.init();
    wheel.init();
    taskEXIT_CRITICAL();
}

bool controller_start(ZedF9P &gps, CMPS12 &imu)
{
    s_gps = &gps;
    s_imu = &imu;
    s_running = true;

    if (!s_task_started)
    {
        if (xTaskCreate(controller_task, "navCtrl", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
        {
            s_running = false;
            return false;
        }

        s_task_started = true;
    }

    return true;
}

void controller_stop()
{
    s_running = false;
    s_telemetry.state = NavigationState::IDLE;
    park_servos();
}

void controller_set_mode(ControlMode mode)
{
    taskENTER_CRITICAL();
    s_mode = mode;
    if (mode == ControlMode::MANUAL)
        s_telemetry.state = NavigationState::IDLE;
    taskEXIT_CRITICAL();

    LOGI(MODULE, "Control mode -> %s", mode == ControlMode::MANUAL ? "MANUAL" : "AUTOMATIC");
}

ControlMode controller_get_mode()
{
    taskENTER_CRITICAL();
    const ControlMode mode = s_mode;
    taskEXIT_CRITICAL();
    return mode;
}

void controller_set_target_waypoint(float x_B, float y_B)
{
    taskENTER_CRITICAL();
    s_telemetry.target_x = x_B;
    s_telemetry.target_y = y_B;
    s_telemetry.has_target = true;
    s_telemetry.state = NavigationState::NAVIGATE;
    taskEXIT_CRITICAL();

    LOGI(MODULE, "Nouvelle cible definie : (%.2f, %.2f)", x_B, y_B);
}

bool controller_set_target_waypoint_gps(double latitude_deg, double longitude_deg)
{
    float target_x = 0.0f;
    float target_y = 0.0f;
    bool target_is_projected = false;

    taskENTER_CRITICAL();
    s_telemetry.has_target = true;
    s_telemetry.state = NavigationState::NAVIGATE;
    s_pending_waypoint_lat = latitude_deg;
    s_pending_waypoint_lon = longitude_deg;
    s_has_pending_waypoint = true;

    if (project_lat_lon_to_local_xy(latitude_deg, longitude_deg, target_x, target_y))
    {
        s_telemetry.target_x = target_x;
        s_telemetry.target_y = target_y;
        s_has_pending_waypoint = false;
        target_is_projected = true;
    }
    taskEXIT_CRITICAL();

    if (target_is_projected)
    {
        LOGI(MODULE, "Nouvelle cible GPS definie : lat=%.7f lon=%.7f -> x=%.2f y=%.2f",
             latitude_deg,
             longitude_deg,
             target_x,
             target_y);
    }
    else
    {
        LOGI(MODULE, "Nouvelle cible GPS en attente du point d'origine : lat=%.7f lon=%.7f",
             latitude_deg,
             longitude_deg);
    }

    return true;
}

void controller_update_wind(float alpha_w_deg, float wind_speed_mps)
{
    taskENTER_CRITICAL();
    s_telemetry.wind_dir_deg = wrap_360(alpha_w_deg);
    s_telemetry.wind_speed_mps = wind_speed_mps;
    taskEXIT_CRITICAL();
}

NavigationState controller_get_state()
{
    return s_telemetry.state;
}

NavigationTelemetry controller_get_telemetry()
{
    return s_telemetry;
}

bool controller_is_running()
{
    return s_running;
}
