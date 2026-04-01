extern "C"
{
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "pid.h"
}

#include "actuators/servo_motor.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"

#include "ble_server.h"

#define DT 0.15f
#define DEADBAND 2.0f
#define SERVO_CENTER_OFFSET 133.0f

PID pid_cap(0.3f, 0.0f, 0.05f, 30.0f);
const int steering_sign = 1;

CalypsoAnemometer calypso;
BleServer ble_server;
ServoMotor front_wheel(6, 40, 160);
ServoMotor sail(7, 0, 250);
CMPS12 cmps12(i2c1);

static float heading_error_to_north(float heading_deg)
{
    if (heading_deg > 180.0f)
        heading_deg -= 360.0f;
    return heading_deg;
}

static void system_init()
{
    printf("=== Char a Voile - North hold + Voile ===\n");
    printf("Servo direction offset: %.1f deg\n", SERVO_CENTER_OFFSET);
    printf("Commandes voile : 'o'=ouverte 'm'=mi-ouverte 'f'=fermee '+/-'=+/-5deg\n");
}

static void print_calib()
{
    auto calib = cmps12.calibration();
    printf(" | Calib M:%d A:%d G:%d S:%d\n", calib.mag, calib.acc, calib.gyro, calib.sys);
}

static void steer_to_north()
{
    auto nav = cmps12.navigation();
    // TODO: Handle read fail
    // {
    //     front_wheel.reset();
    //     pid_cap.reset();
    //     printf("Compass read failed, steering center\n");
    //     return;
    // }

    float err = heading_error_to_north(nav.cap);

    if (err > -DEADBAND && err < DEADBAND)
    {
        front_wheel.reset();
        pid_cap.reset();
        printf("Cap:%.1f Pitch:%d Roll:%d | Err:%.1f | DEADBAND | Servo:%.1f", nav.cap, nav.pitch, nav.roll, err, SERVO_CENTER_OFFSET);
    }

    float correction = steering_sign * pid_cap.compute(err, DT);
    float angle = SERVO_CENTER_OFFSET + correction;
    front_wheel.rotate(angle);
    printf("Cap:%.1f Pitch:%d Roll:%d | Err:%+.1f | Corr:%+.1f | Servo:%.1f", nav.cap, nav.pitch, nav.roll, err, correction, angle);

    print_calib();
}

static void on_wind_data(const CalypsoData *data)
{
    ble_server.update(data);
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
        sleep_ms(100);

    // Init I2C
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    sail.init();
    front_wheel.init();

    if (cyw43_arch_init())
    {
        printf("CYW43 init failed!\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    ble_server.init();
    calypso.init("F9:26:B6:C0:42:F3", 1, on_wind_data);

    ble_server.startAdvertising();

    hci_power_control(HCI_POWER_ON);

    sleep_ms(1000);
    calypso.connect();

    printf("All Initiated!");

    // while (1)
    // {
    //     int v = read_int();
    //     printf("Sending: %f\n", v / 100.0f);
    //     front_wheel.rotate(v / 100.0f);
    // }

    return 0;
}
