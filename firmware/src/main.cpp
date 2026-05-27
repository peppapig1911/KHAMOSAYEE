#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "btstack.h"

#include "sensors/odometry.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"
#include "comms/i2c.h"

#include "comms/ble_server.h"
#include "comms/ntrip_client.h"

#include "controller.h"
#include "log.h"
#include "navigation.h"

#include <cmath>
#include <cstring>

static constexpr const char *MODULE = "Main";

static constexpr float ARRIVAL_RADIUS = 0.75f;

static CalypsoAnemometer calypso;
static BleServer ble_server;

I2C i2c(i2c1, 400000, 26, 27);

static CMPS12 cmps12(i2c);
static ZedF9P gps(i2c);
static NtripClient ntrip("crtk.net", 2101, "IGNU", &gps, &gps);

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
        if (gnss.fix_type < 3) // attendre fix 3D minimum avant de fixer l'origine
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

static void cpms12_calib_task(void *param)
{
    UNUSED(param);

    while (true)
    {
        auto cal = cmps12.calibration();
        LOGI(MODULE, "Calib -> M:%d A:%d G:%d S:%d", cal.mag, cal.acc, cal.gyro, cal.sys);
        auto nav = cmps12.navigation();
        LOGI(MODULE, "Nav -> Cap: %.1f deg | Pitch: %.1f | Roll: %.1f", nav.cap, nav.pitch, nav.roll);
        auto raw = cmps12.raw();
        LOGI(MODULE, "Raw -> Mag:(%d, %d, %d) | Acc:(%d, %d, %d) | Gyro:(%d, %d, %d)", raw.mag.x, raw.mag.y, raw.mag.z, raw.acc.x, raw.acc.y, raw.acc.z, raw.gyro.x, raw.gyro.y, raw.gyro.z);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void print_position_task(void *param)
{
    auto *gps_device = static_cast<ZedF9P *>(param);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        const auto &position = gps_device->data();
        if (position.lat == 0.0 && position.lon == 0.0)
        {
            LOGI(MODULE, "Attente d'un fix GPS valide...");
            continue;
        }

        ble_server.updateLocation(&position);
        ble_server.updateHeading(controller_get_telemetry().psi);
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
}

static void wifi_init_task(void *param)
{
    UNUSED(param);

    vTaskDelay(pdMS_TO_TICKS(100));

    LOGI(MODULE, "I2C initialized at %u baud", i2c.init());

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
        int wifi_result = cyw43_arch_wifi_connect_timeout_ms("Rover", "00990088", CYW43_AUTH_WPA2_AES_PSK, 5000);
        if (wifi_result == 0)
            break;

        LOGE(MODULE, "WiFi connection failed (error: %d)", wifi_result);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    LOGI(MODULE, "WiFi connected successfully");

    ble_server.init();
    calypso.init("F9:26:B6:C0:42:F3", 1);
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
    }
    else
    {
        LOGI(MODULE, "Connected to NTRIP server successfully!");
    }

    LOGI(MODULE, "All initiated!");

    controller_init();
    if (!controller_start(gps, cmps12))
    {
        LOGE(MODULE, "Failed to start controller task!");
    }

    if (xTaskCreate(print_position_task, "gpsBleTask", 2048, (void *)&gps, tskIDLE_PRIORITY, NULL) != pdPASS)
    {
        LOGE(MODULE, "Failed to start GPS BLE task!");
    }

    xTaskCreate(NtripClient::task, "ntripTask", 2048, (void *)&ntrip, tskIDLE_PRIORITY, NULL);
    // xTaskCreate(cpms12_calib_task, "cmps12CalibTask", 2048, (void *)&cmps12, tskIDLE_PRIORITY, NULL);

    while (true)
        vTaskDelay(pdMS_TO_TICKS(100000));
}

int main()
{
    stdio_init_all();

    // while (!stdio_usb_connected())
    //     tight_loop_contents();

    LOGI(MODULE, "USB connected, starting system...");

    // i2c.init();

    // sail_init();
    // wheel_init();
    // navigation_init();
    // navigation_set_center_deg(wheel_get_center_deg());

    // static Odometry odometry;
    // BaseType_t xReturned = xTaskCreate(Odometry::task, "Odom_Task", 512, &odometry, 2, NULL);

    // if (xReturned == pdPASS)
    //     LOGI(MODULE, "Odometry started");
    // else
    //     LOGI(MODULE, "Odometry launch failed");

    xTaskCreate(wifi_init_task, "wifiInitTask", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(navigation_console_task, "navTask", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    return 0;
}
