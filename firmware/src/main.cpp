#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "btstack.h"

#include "pid.h"


#include "actuators/servo_motor.h"

#include "sensors/odometry.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"

#include "comms/ble_server.h"
#include "comms/ntrip_client.h"
#include "log.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

#define DT 0.15f
#define DEADBAND 2.0f
#define SERVO_CENTER_OFFSET 133.0f

const int steering_sign = 1;

static constexpr const char *MODULE = "Main";

static PID pid_cap(0.3f, 0.0f, 0.05f, 30.0f);
static ServoMotor front_wheel(6, 50, 160);
static ServoMotor sail(7, 0, 250);

static CalypsoAnemometer calypso;

static BleServer ble_server;

static Odometry odometry;
static CMPS12 cmps12(i2c1);
static ZedF9P gps(i2c1);

static NtripClient ntrip("crtk.net", 2101, "IGNU", &gps, &gps);

static float heading_error_to_north(float heading_deg)
{
    if (heading_deg > 180.0f)
        heading_deg -= 360.0f;
    return heading_deg;
}

static void system_init()
{
    LOGI(MODULE, "=== Char a Voile - North hold + Voile ===");
    LOGI(MODULE, "Servo direction offset: %.1f deg", SERVO_CENTER_OFFSET);
    LOGI(MODULE, "Commandes voile : 'o'=ouverte 'm'=mi-ouverte 'f'=fermee '+/-'=+/-5deg");
}

static void print_calib()
{
    auto calib = cmps12.calibration();
    LOGD(MODULE, "| Calib M:%d A:%d G:%d S:%d", calib.mag, calib.acc, calib.gyro, calib.sys);
}

static void steer_to_north()
{
    auto nav = cmps12.navigation();
    float err = heading_error_to_north(nav.cap);

    if (err > -DEADBAND && err < DEADBAND)
    {
        front_wheel.reset();
        pid_cap.reset();
        LOGD(MODULE, "Cap:%.1f Pitch:%d Roll:%d | Err:%.1f | DEADBAND | Servo:%.1f", nav.cap, nav.pitch, nav.roll, err, SERVO_CENTER_OFFSET);
    }

    float correction = steering_sign * pid_cap.compute(err, DT);
    float angle = SERVO_CENTER_OFFSET + correction;
    front_wheel.rotate(angle);
    LOGD(MODULE, "Cap:%.1f Pitch:%d Roll:%d | Err:%+.1f | Corr:%+.1f | Servo:%.1f", nav.cap, nav.pitch, nav.roll, err, correction, angle);

    print_calib();
}

static void on_wind_data(const CalypsoData *data)
{
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
        int wifi_result = cyw43_arch_wifi_connect_timeout_ms("Rover", "00990088", CYW43_AUTH_WPA2_AES_PSK, 20000);
        if (wifi_result == 0)
            break;

        LOGE(MODULE, "WiFi connection failed (error: %d)", wifi_result);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    LOGI(MODULE, "WiFi connected successfully");

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    ble_server.init();

    calypso.init("F9:26:B6:C0:42:F3", 1, on_wind_data);
    ble_server.startAdvertising();

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
    else
    {
        LOGI(MODULE, "Connected to NTRIP server successfully!");
    }

    LOGI(MODULE, "All initiated!");

    xTaskCreate(
        NtripClient::task,
        "ntripTask",
        2048,
        (void *)&ntrip,
        tskIDLE_PRIORITY,
        NULL);

    static PosTaskParam pos_param = {&gps, &odometry, &cmps12};
    xTaskCreate(
        print_position_task,
        "posTask",
        2048,
        (void *)&pos_param,
        tskIDLE_PRIORITY,
        NULL);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        tight_loop_contents();
    }

    LOGI(MODULE, "USB connected, starting system...");

    BaseType_t xReturned;

    xReturned = xTaskCreate(Odometry::task, "Odom_Task", 512, &odometry, 2, NULL);

    if (xReturned == pdPASS)
    {
        LOGI(MODULE, "Odometry started");
    }
    else
    {
        LOGI(MODULE, "Odometry launch failed");
    }

    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
    i2c_init(i2c1, 100 * 1000);

    xTaskCreate(
        wifi_init_task,
        "wifiInitTask",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

    vTaskStartScheduler();

    return 0;
}
