#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "btstack.h"

#include "actuators/servo_motor.h"
#include "sensors/odometry.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"

#include "comms/ble_server.h"
#include "comms/ntrip_client.h"
#include "log.h"
#include "navigation.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cmath>
#include <cstring>

static constexpr const char *MODULE = "Main";

static constexpr float ARRIVAL_RADIUS = 0.75f;
static float front_wheel_center_deg = FRONT_WHEEL_CENTER_DEG;
static constexpr float FRONT_WHEEL_NEUTRAL_POS =
    (FRONT_WHEEL_CENTER_DEG - FRONT_WHEEL_MIN_DEG) / (FRONT_WHEEL_MAX_DEG - FRONT_WHEEL_MIN_DEG);

static ServoMotor front_wheel(6, FRONT_WHEEL_MIN_DEG, FRONT_WHEEL_MAX_DEG, FRONT_WHEEL_NEUTRAL_POS);
static ServoMotor sail(7, 0, 250);

static CalypsoAnemometer calypso;
static BleServer ble_server;

static CMPS12 cmps12(i2c1);
static ZedF9P gps(i2c1);
static NtripClient ntrip("crtk.net", 2101, "IGNU", &gps, &gps);

static volatile float last_known_wind = 0.0f;

static float sail_angle_from_wind(float wind_direction_deg)
{
    float wind = wind_direction_deg;

    while (wind < 0.0f)
        wind += 360.0f;
    while (wind >= 360.0f)
        wind -= 360.0f;

    float beta = wind;
    if (beta > 180.0f)
        beta = 360.0f - beta;

    float delta = beta - 15.0f;
    if (delta < 0.0f)
        delta = 0.0f;
    if (delta > 90.0f)
        delta = 90.0f;

    return 180.0f - (delta * 2.0f);
}

static void control_sail_from_wind(float wind_direction_deg)
{
    sail.rotate_deg(sail_angle_from_wind(wind_direction_deg));
}

static void on_wind_data(const CalypsoData *data)
{
    last_known_wind = data->wind_direction;
    control_sail_from_wind(data->wind_direction);
    LOGD(MODULE, "Wind speed: %.2f m/s | direction: %.1f deg | battery: %.2f V", data->wind_speed, data->wind_direction, data->battery);
    ble_server.update(data);
}

static float cap_vers_point(float dx, float dy)
{
    float angle_rad = atan2f(dx, dy); // dx=East, dy=North
    float cap = angle_rad * 180.0f / M_PI;

    if (cap < 0.0f)
        cap += 360.0f;

    return cap;
}

struct LocalProjection
{
    bool has_origin = false;
    double origin_lat = 0.0;
    double origin_lon = 0.0;
};

static bool project_gps_to_local_xy(const GNSSData &gnss, LocalProjection &projection, float &x, float &y)
{
    if (!gnss.valid)
        return false;

    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_RADIUS_M = 6378137.0;

    if (!projection.has_origin)
    {
        projection.origin_lat = gnss.lat;
        projection.origin_lon = gnss.lon;
        projection.has_origin = true;
    }

    const double dlat = (gnss.lat - projection.origin_lat) * DEG_TO_RAD;
    const double dlon = (gnss.lon - projection.origin_lon) * DEG_TO_RAD;
    const double origin_lat_rad = projection.origin_lat * DEG_TO_RAD;

    x = static_cast<float>(EARTH_RADIUS_M * dlon * std::cos(origin_lat_rad));
    y = static_cast<float>(EARTH_RADIUS_M * dlat);
    return true;
}

static bool project_lat_lon_to_local_xy(double lat, double lon, const LocalProjection &projection, float &x, float &y)
{
    if (!projection.has_origin)
        return false;

    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_RADIUS_M = 6378137.0;

    const double dlat = (lat - projection.origin_lat) * DEG_TO_RAD;
    const double dlon = (lon - projection.origin_lon) * DEG_TO_RAD;
    const double origin_lat_rad = projection.origin_lat * DEG_TO_RAD;

    x = static_cast<float>(EARTH_RADIUS_M * dlon * std::cos(origin_lat_rad));
    y = static_cast<float>(EARTH_RADIUS_M * dlat);
    return true;
}

static bool parse_manual_angle_command(const char *input, float &angle_deg)
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

static bool parse_center_command(const char *input, float &center_deg)
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

static bool parse_lat_lon_command(const char *input, double &lat, double &lon)
{
    if (sscanf(input, "ll %lf %lf", &lat, &lon) == 2)
        return true;
    if (sscanf(input, "latlon %lf %lf", &lat, &lon) == 2)
        return true;
    return false;
}

static bool parse_direct_degree_input(const char *input, float &angle_deg)
{
    float requested_angle = 0.0f;

    if (sscanf(input, "%f", &requested_angle) != 1)
        return false;

    if (requested_angle < FRONT_WHEEL_MIN_DEG || requested_angle > FRONT_WHEEL_MAX_DEG)
        return false;

    angle_deg = requested_angle;
    return true;
}

static void print_position_task(void *param)
{
    auto *gps_device = static_cast<ZedF9P *>(param);

    while (true)
    {
        if (gps_device->update())
        {
            const auto &position = gps_device->data();
            LOGI(MODULE,
                 "POS lat=%.7f lon=%.7f alt=%.1f hacc=%.1fcm vacc=%.1fcm fix=%u sats=%u rtk=%u",
                 position.lat,
                 position.lon,
                 position.altitude,
                 position.h_acc * 100.0f,
                 position.v_acc * 100.0f,
                 position.fix_type,
                 position.num_sv,
                 static_cast<unsigned>(gps_device->rtk_state()));
        }
        else
        {
            LOGD(MODULE, "No fix");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_init_task(void *param)
{
    UNUSED(param);

    vTaskDelay(pdMS_TO_TICKS(100));

    if (cyw43_arch_init())
    {
        LOGE(MODULE, "cyw43_arch_init failed");
        vTaskDelete(NULL);
        return;
    }
    LOGI(MODULE, "cyw43_arch_init succeeded");

    cyw43_arch_enable_sta_mode();
    while (true)
    {
        int wifi_result = cyw43_arch_wifi_connect_timeout_ms("Rover", "00990088", CYW43_AUTH_WPA2_AES_PSK, 20000);
        if (wifi_result == 0)
            break;

        LOGE(MODULE, "WiFi connection failed (error: %d)", wifi_result);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    LOGI(MODULE, "WiFi connected successfully");

    ble_server.init();
    calypso.init("F9:26:B6:C0:42:F3", 1, on_wind_data);
    ble_server.startAdvertising();
    calypso.connect();

    if (!gps.probe())
        LOGE(MODULE, "GPS not found on I2C (addr 0x%02X)!", ZedF9P::DEFAULT_ADDR);
    else
    {
        gps.init();
        LOGI(MODULE, "GPS found and initialized.");
    }

    if (!ntrip.init())
    {
        LOGE(MODULE, "Failed to connect to NTRIP server!");
        vTaskDelete(NULL);
        return;
    }

    LOGI(MODULE, "Connected to NTRIP server successfully!");
    LOGI(MODULE, "All initiated!");

    xTaskCreate(NtripClient::task, "ntripTask", 2048, (void *)&ntrip, tskIDLE_PRIORITY, NULL);
    xTaskCreate(print_position_task, "posTask", 2048, (void *)&gps, tskIDLE_PRIORITY, NULL);

    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

static void navigation_console_task(void *param)
{
    UNUSED(param);

    enum class SystemPhase
    {
        CALIBRATION,
        SERVO_ZERO,
        WAIT_TARGET,
        NAVIGATION
    };

    SystemPhase phase = SystemPhase::CALIBRATION;

    float target_x = 0.0f;
    float target_y = 0.0f;
    float x_actuel = 0.0f;
    float y_actuel = 0.0f;
    bool has_current_position = false;
    LocalProjection gps_projection{};
    bool target_is_latlon = false;
    double target_lat = 0.0;
    double target_lon = 0.0;

    char input_buf[64] = {0};
    int input_idx = 0;
    bool target_prompt_printed = false;

    absolute_time_t last_calib_print = get_absolute_time();

    LOGI(MODULE, "[PHASE 1] Calibration CMPS12 en cours.");
    LOGI(MODULE, "Tourner le capteur sur plusieurs axes jusqu'a obtenir SYS=3.");
    LOGI(MODULE, "Vous pouvez aussi taper 'start' pour forcer la suite.");

    while (true)
    {
        int c;
        while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
        {
            if (c == '\r')
                continue;

            if (c == '\n')
            {
                input_buf[input_idx] = '\0';

                float manual_angle = 0.0f;
                float new_center = 0.0f;
                if (parse_center_command(input_buf, new_center))
                {
                    front_wheel_center_deg = new_center;
                    navigation_set_center_deg(front_wheel_center_deg);
                    front_wheel.rotate_deg(front_wheel_center_deg);
                    LOGI(MODULE, "Nouveau neutre applique : %.1f deg", front_wheel_center_deg);
                    target_prompt_printed = false;
                }
                else if (parse_manual_angle_command(input_buf, manual_angle))
                {
                    front_wheel.rotate_deg(manual_angle);
                    LOGI(MODULE, "Angle servo applique : %.1f deg", manual_angle);
                    target_prompt_printed = false;
                }
                else if (parse_direct_degree_input(input_buf, manual_angle))
                {
                    front_wheel.rotate_deg(manual_angle);
                    LOGI(MODULE, "Angle servo applique (direct): %.1f deg", manual_angle);
                    target_prompt_printed = false;
                }
                else if (phase == SystemPhase::CALIBRATION)
                {
                    if (strcmp(input_buf, "start") == 0)
                    {
                        LOGI(MODULE, "Calibration forcee par l'utilisateur.");
                        phase = SystemPhase::SERVO_ZERO;
                    }
                }
                else if (phase == SystemPhase::WAIT_TARGET || phase == SystemPhase::NAVIGATION)
                {
                    double new_lat, new_lon;

                    if (sscanf(input_buf, "%lf %lf", &new_lat, &new_lon) == 2)
                    {
                        target_lat = new_lat;
                        target_lon = new_lon;
                        target_is_latlon = true;

                        navigation_init();
                        LOGI(MODULE, "Nouvelle cible lat/lon validee (x=lat, y=lon) : (%.7f, %.7f)", target_lat, target_lon);
                        phase = SystemPhase::NAVIGATION;
                        target_prompt_printed = false;
                    }
                    else if (parse_lat_lon_command(input_buf, new_lat, new_lon))
                    {
                        target_lat = new_lat;
                        target_lon = new_lon;
                        target_is_latlon = true;

                        navigation_init();
                        LOGI(MODULE, "Nouvelle cible lat/lon validee : (%.7f, %.7f)", target_lat, target_lon);
                        phase = SystemPhase::NAVIGATION;
                        target_prompt_printed = false;
                    }
                    else if (input_buf[0] != '\0')
                    {
                        LOGI(MODULE, "Format invalide. Entrez: x y (lat lon) ou ll <lat> <lon> | center <deg>");
                        target_prompt_printed = false;
                    }
                }

                input_idx = 0;
            }
            else if (input_idx < (int)sizeof(input_buf) - 1)
            {
                input_buf[input_idx++] = (char)c;
            }
        }

        if (phase == SystemPhase::CALIBRATION)
        {
            if (absolute_time_diff_us(last_calib_print, get_absolute_time()) >= 1000000)
            {
                auto calib = cmps12.calibration();
                LOGI(MODULE, "Calib -> M:%d A:%d G:%d S:%d", calib.mag, calib.acc, calib.gyro, calib.sys);
                last_calib_print = get_absolute_time();

                if (calib.sys == 3)
                {
                    LOGI(MODULE, "[PHASE 1] Calibration terminee.");
                    phase = SystemPhase::SERVO_ZERO;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (phase == SystemPhase::SERVO_ZERO)
        {
            front_wheel.rotate_deg(front_wheel_center_deg);
            navigation_init();
            navigation_set_center_deg(front_wheel_center_deg);
            LOGI(MODULE, "[PHASE 2] Servo direction positionne au neutre (%.1f deg).", front_wheel_center_deg);
            LOGI(MODULE, "[PHASE 3] Entrez la cible: x y (lat lon) ou ll <lat> <lon> (ou angle manuel: a <deg>)");
            LOGI(MODULE, "Navigation GPS reelle activee (sans simulation).");
            phase = SystemPhase::WAIT_TARGET;
            target_prompt_printed = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (phase == SystemPhase::WAIT_TARGET)
        {
            if (!target_prompt_printed)
            {
                LOGI(MODULE, "Entrer cible x y (lat lon) ou ll <lat> <lon> ou a <deg> :");
                target_prompt_printed = true;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const GNSSData &fix = gps.data();
        has_current_position = project_gps_to_local_xy(fix, gps_projection, x_actuel, y_actuel);
        if (!has_current_position)
        {
            LOGI(MODULE, "Position GPS indisponible: attente d'un fix valide.");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (target_is_latlon)
        {
            if (!project_lat_lon_to_local_xy(target_lat, target_lon, gps_projection, target_x, target_y))
            {
                LOGI(MODULE, "Impossible de projeter la cible lat/lon: origine GPS locale non definie.");
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }
        }

        float dx = target_x - x_actuel;
        float dy = target_y - y_actuel;
        float distance_to_target = sqrtf(dx * dx + dy * dy);

        if (distance_to_target <= ARRIVAL_RADIUS)
        {
            front_wheel.rotate_deg(front_wheel_center_deg);
            navigation_init();
            navigation_set_center_deg(front_wheel_center_deg);
            LOGI(MODULE, "Cible atteinte (dist=%.2f). Direction au neutre (%.1f deg).", distance_to_target, front_wheel_center_deg);
            phase = SystemPhase::WAIT_TARGET;
            target_prompt_printed = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float cap_cible = cap_vers_point(dx, dy);

        float wind_gap = calculate_heading_error(last_known_wind, cap_cible);
        float cap_to_follow = cap_cible;

        if (fabsf(wind_gap) < 45.0f)
        {
            if (wind_gap >= 0.0f)
                cap_to_follow = last_known_wind + 45.0f;
            else
                cap_to_follow = last_known_wind - 45.0f;
        }

        if (cap_to_follow < 0.0f)
            cap_to_follow += 360.0f;
        if (cap_to_follow >= 360.0f)
            cap_to_follow -= 360.0f;

        LOGI(MODULE,
             "Pos:(%.2f, %.2f) -> Cible:(%.2f, %.2f) | Dist:%.2f | CapSuivi:%.1f",
             x_actuel,
             y_actuel,
             target_x,
             target_y,
             distance_to_target,
             cap_to_follow);

        steer_to_heading(cap_to_follow, cmps12, front_wheel);

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
        tight_loop_contents();

    LOGI(MODULE, "USB connected, starting system...");

    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
    i2c_init(i2c1, 100 * 1000);

    sail.init();
    front_wheel.init();
    navigation_init();
    navigation_set_center_deg(front_wheel_center_deg);

    static Odometry odometry;
    BaseType_t xReturned = xTaskCreate(Odometry::task, "Odom_Task", 512, &odometry, 2, NULL);

    if (xReturned == pdPASS)
        LOGI(MODULE, "Odometry started");
    else
        LOGI(MODULE, "Odometry launch failed");

    xTaskCreate(wifi_init_task, "wifiInitTask", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(navigation_console_task, "navTask", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    return 0;
}
