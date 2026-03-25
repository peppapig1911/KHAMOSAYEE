extern "C" {
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "cmps12.h"
#include "servo_direction.h"
#include "servo_voile.h"
#include "pid.h"
}

#include "calypso_anemometer.h"

#define DT 0.15f
#define DEADBAND 2.0f
#define SERVO_CENTER_OFFSET 133.0f

static PID pid_cap;
static const int steering_sign = 1;
static float voile_angle = VOILE_FERMEE;
static CalypsoAnemometer calypso;

// =====================================================
// FONCTIONS
// =====================================================

static float heading_error_to_north(float heading_deg)
{
    if (heading_deg > 180.0f)
        heading_deg -= 360.0f;
    return heading_deg;
}

static void system_init()
{
    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(100);
    cmps12_init();
    servo_init();
    servo_voile_init();
    pid_init(&pid_cap, 0.3f, 0.0f, 0.05f, 30.0f);
    printf("=== Char a Voile - North hold + Voile ===\n");
    printf("Servo direction offset: %.1f deg\n", SERVO_CENTER_OFFSET);
    printf("Commandes voile : 'o'=ouverte 'm'=mi-ouverte 'f'=fermee '+/-'=+/-5deg\n");
}

static void print_calib()
{
    CalibData calib;
    if (check_calibration(&calib))
    {
        printf(" | Calib M:%d A:%d G:%d S:%d\n",
               calib.mag_cal, calib.acc_cal, calib.gyro_cal, calib.sys_cal);
    }
    else
    {
        printf(" | Calib read failed\n");
    }
}

static void steer_to_north()
{
    NavData nav;

    if (!read_navigation_data(&nav))
    {
        servo_set_angle(SERVO_CENTER_OFFSET);
        pid_reset(&pid_cap);
        printf("Compass read failed, steering center\n");
        return;
    }

    float err = heading_error_to_north(nav.cap);

    if (err > -DEADBAND && err < DEADBAND)
    {
        servo_set_angle(SERVO_CENTER_OFFSET);
        pid_reset(&pid_cap);
        printf("Cap:%.1f Pitch:%d Roll:%d | Err:%.1f | DEADBAND | Servo:%.1f",
               nav.cap, nav.pitch, nav.roll, err, SERVO_CENTER_OFFSET);
    }
    else
    {
        float correction = steering_sign * pid_compute(&pid_cap, err, DT);
        float angle = SERVO_CENTER_OFFSET + correction;
        servo_set_angle(angle);
        printf("Cap:%.1f Pitch:%d Roll:%d | Err:%+.1f | Corr:%+.1f | Servo:%.1f",
               nav.cap, nav.pitch, nav.roll, err, correction, angle);
    }

    print_calib();
}

static void handle_voile_command()
{
    int c = 0;

    if (c == 'o')
        voile_angle = VOILE_OUVERTE;
    else if (c == 'm')
        voile_angle = VOILE_MI_OUVERTE;
    else if (c == 'f')
        voile_angle = VOILE_FERMEE;
    else if (c == '+')
        voile_angle += 5.0f;
    else if (c == '-')
        voile_angle -= 5.0f;
    else
        return;

    if (voile_angle > 180.0f) voile_angle = 180.0f;
    if (voile_angle < 0.0f)   voile_angle = 0.0f;

    servo_voile_set_angle(voile_angle);
    printf(">>> Voile : %.1f deg\n", voile_angle);
}

static void on_wind_data(const CalypsoData *data)
{
    printf("Wind: %.2f m/s @ %.0f°  Battery: %.1fV\n",
           data->wind_speed,
           data->wind_direction,
           data->battery);
}

// =====================================================
// MAIN
// =====================================================

int main()
{
    system_init();

    stdio_init_all();

    if (cyw43_arch_init())
    {
        printf("CYW43 init failed!\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    calypso.init("F9:26:B6:C0:42:F3", 1, on_wind_data);

    hci_power_control(HCI_POWER_ON);

    sleep_ms(1000);
    calypso.connect();

    while (true)
    {
        handle_voile_command();
        steer_to_north();
        tight_loop_contents();
        sleep_ms((uint32_t)(DT * 1000));
    }

    return 0;
}
