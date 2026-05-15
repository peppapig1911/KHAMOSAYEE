#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "btstack.h"

#include "pid.h"

#include "actuators/servo_motor.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "sensors/zed_f9p.h"

#include "comms/ble_server.h"
#include "comms/ntrip_client.h"
#include "log.h"

#include "FreeRTOS.h"
#include "task.h"

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

static void print_position_task(void *param)
{
    auto *gps_device = static_cast<ZedF9P *>(param);

    while (true)
    {
        if (gps_device->update())
        {
            const auto &position = gps_device->data();
            LOGI(MODULE,
                 "POS lat=%.7f lon=%.7f alt=%.1f hacc=%.1fcm vacc=%.1fcm  fix=%u sats=%u rtk=%u",
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

    xTaskCreate(
        print_position_task,
        "posTask",
        2048,
        (void *)&gps,
        tskIDLE_PRIORITY,
        NULL);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

int read_int()
{
    const uint BUFFER_SIZE = 32;
    char buffer[BUFFER_SIZE];
    int index = 0;

    while (true)
    {
        int c = getchar_timeout_us(0); // non-blocking

        if (c != PICO_ERROR_TIMEOUT)
        {
            if (c == '\n' || c == '\r')
            {
                buffer[index] = '\0'; // end string

                int value = atoi(buffer); // convert to int
                return value;
            }
            else if (index < BUFFER_SIZE - 1)
            {
                buffer[index++] = (char)c;
            }
        }
    }
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        tight_loop_contents();
    }

    LOGI(MODULE, "USB connected, starting system...");

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
