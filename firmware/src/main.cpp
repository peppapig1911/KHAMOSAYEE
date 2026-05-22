#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "btstack.h"

#include "sensors/odometry.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"

#include "comms/ble_server.h"
#include "comms/ntrip_client.h"

#include "command/wheel.h"
#include "command/sail.h"
#include "log.h"
#include "navigation.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cmath>
#include <cstring>

static constexpr const char *MODULE = "Main";

static constexpr float ARRIVAL_RADIUS = 0.75f;


static CalypsoAnemometer calypso;
static BleServer ble_server;

static Odometry odometry;
static CMPS12 cmps12(i2c1);
static ZedF9P gps(i2c1);
static NtripClient ntrip("crtk.net", 2101, "IGNU", &gps, &gps);


static volatile float last_known_wind = 0.0f;

// Callback pour l'anémomètre
static void on_wind_data(const CalypsoData *data)
{
    last_known_wind = data->wind_direction;
    sail_set_angle_from_wind(data->wind_direction);
    LOGD(MODULE, "Wind speed: %.2f m/s | direction: %.1f deg | battery: %.2f V", data->wind_speed, data->wind_direction, data->battery);
    ble_server.update(data);
}

// Serial output format (one pair per second):
//   RAW,<lat>,<lon>,<hacc_cm>,<fix_type>,<num_sv>,<rtk_state>
//   CORR,<lat>,<lon>,<drift_m>          (odometry dead-reckoning from last <5cm anchor)
//   CORR,NO_ANCHOR                       (no sub-5cm fix seen yet)
//   RAW,NO_FIX / CORR,NO_FIX            (no GPS data)
static constexpr float  RTK_ANCHOR_THRESHOLD_M = 0.05f;
static constexpr double EARTH_RADIUS_M         = 6371000.0;

struct PosTaskParam { ZedF9P *gps; Odometry *odometry; CMPS12 *compass; };

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
        if (gnss.fix_type < 3)  // attendre fix 3D minimum avant de fixer l'origine
            return false;
        projection.origin_lat = gnss.lat;
        projection.origin_lon = gnss.lon;
        projection.has_origin = true;
        LOGI(MODULE, "Origine GPS fixee : lat=%.7f lon=%.7f", gnss.lat, gnss.lon);
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

static bool parse_lat_lon_command(const char *input, double &lat, double &lon)
{
    if (sscanf(input, "ll %lf %lf", &lat, &lon) == 2)
        return true;
    if (sscanf(input, "latlon %lf %lf", &lat, &lon) == 2)
        return true;
    return false;
}

static void print_position_task(void *param)
{
    auto *p = static_cast<PosTaskParam *>(param);

    bool   anchor_valid  = false;
    double anchor_lat    = 0.0, anchor_lon = 0.0;
    float  anchor_odo_x  = 0.0f, anchor_odo_y = 0.0f;
    float  theta_offset  = 0.0f;

    while (true)
    {
        if (p->gps->update())
        {
            const GNSSData &g   = p->gps->data();
            RobotPosition   odo = p->odometry->getPosition();

            if (g.valid)
            {
                if (g.h_acc < RTK_ANCHOR_THRESHOLD_M)
                {
                    // Recompute the odometry->ENU rotation on every good fix so
                    // it stays time-consistent with the anchor and re-zeroes
                    // accumulated odometry heading drift against the compass.
                    float cap_rad = p->compass->navigation().cap * (float)M_PI / 180.0f;
                    theta_offset  = ((float)M_PI / 2.0f - cap_rad) - odo.theta;
                    anchor_valid  = true;
                    anchor_lat    = g.lat;
                    anchor_lon    = g.lon;
                    anchor_odo_x  = odo.x;
                    anchor_odo_y  = odo.y;
                }

                printf("RAW,%.7f,%.7f,%.2f,%u,%u,%u\n",
                       g.lat, g.lon, g.h_acc * 100.0f,
                       g.fix_type, g.num_sv,
                       static_cast<unsigned>(p->gps->rtk_state()));

                if (anchor_valid)
                {
                    if (g.h_acc >= RTK_ANCHOR_THRESHOLD_M)
                    {
                        float dx = (odo.x - anchor_odo_x) / 100.0f;
                        float dy = (odo.y - anchor_odo_y) / 100.0f;
                        float dx_enu = dx * cosf(theta_offset) - dy * sinf(theta_offset);
                        float dy_enu = dx * sinf(theta_offset) + dy * cosf(theta_offset);
                        double corr_lat = anchor_lat + (double)dy_enu / EARTH_RADIUS_M * (180.0 / M_PI);
                        double corr_lon = anchor_lon + (double)dx_enu / (EARTH_RADIUS_M * cos(anchor_lat * M_PI / 180.0)) * (180.0 / M_PI);
                        printf("CORR,%.7f,%.7f,%.3f\n", corr_lat, corr_lon, sqrtf(dx_enu*dx_enu + dy_enu*dy_enu));
                    }
                    else
                    {
                        printf("CORR,%.7f,%.7f,0.000\n", g.lat, g.lon);
                    }
                }
                else
                {
                    printf("CORR,NO_ANCHOR\n");
                }
            }
            else
            {
                printf("RAW,NO_FIX\n");
                printf("CORR,NO_FIX\n");
            }
        }
        else
        {
            printf("RAW,NO_FIX\n");
            printf("CORR,NO_FIX\n");
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
        int wifi_result = cyw43_arch_wifi_connect_timeout_ms("iPhone de Christine", "etbahnonlol", CYW43_AUTH_WPA2_AES_PSK, 20000);
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

    static PosTaskParam pos_param = {&gps, &odometry, &cmps12};
    xTaskCreate(print_position_task, "posTask", 2048, (void *)&pos_param, tskIDLE_PRIORITY, NULL);

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
                float sail_angle = 0.0f;
                float new_center = 0.0f;
                if (wheel_parse_center_command(input_buf, new_center))
                {
                    wheel_set_center_deg(new_center);
                    LOGI(MODULE, "Nouveau neutre applique : %.1f deg", wheel_get_center_deg());
                    target_prompt_printed = false;
                }
                else if (wheel_parse_manual_angle_command(input_buf, manual_angle))
                {
                    wheel_rotate_deg(manual_angle);
                    LOGI(MODULE, "Angle servo applique : %.1f deg", manual_angle);
                    target_prompt_printed = false;
                }
                else if (wheel_parse_direct_degree_input(input_buf, manual_angle))
                {
                    wheel_rotate_deg(manual_angle);
                    LOGI(MODULE, "Angle servo applique (direct): %.1f deg", manual_angle);
                    target_prompt_printed = false;
                }
                else if (sail_parse_console_command(input_buf, &sail_angle))
                {
                    sail_set_manual_angle(sail_angle);
                    LOGI(MODULE, "Voile angle manuel applique : %.1f deg", sail_angle);
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
                    phase = SystemPhase::SERVO_ZERO;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (phase == SystemPhase::SERVO_ZERO)
        {
            wheel_rotate_to_center();
            navigation_init();
            navigation_set_center_deg(wheel_get_center_deg());
            LOGI(MODULE, "[PHASE 2] Servo direction positionne au neutre (%.1f deg).", wheel_get_center_deg());
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
        
        gps.update();
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
            wheel_rotate_to_center();
            navigation_init();
            navigation_set_center_deg(wheel_get_center_deg());
            LOGI(MODULE, "Cible atteinte (dist=%.2f). Direction au neutre (%.1f deg).", distance_to_target, wheel_get_center_deg());
            phase = SystemPhase::WAIT_TARGET;
            target_prompt_printed = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float cap_cible = wheel_compute_bearing_deg(dx, dy);

        taskENTER_CRITICAL();
        float wind = last_known_wind;
        taskEXIT_CRITICAL();
        float wind_gap = calculate_heading_error(wind, cap_cible);
        float cap_to_follow = cap_cible;

        if (fabsf(wind_gap) < 45.0f)
        {
            if (wind_gap >= 0.0f)
                cap_to_follow = wind+ 45.0f;
            else
                cap_to_follow = wind - 45.0f;
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

        wheel_steer_to_heading(cap_to_follow, cmps12);

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

    sail_init();
    wheel_init();
    navigation_init();
    navigation_set_center_deg(wheel_get_center_deg());

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
