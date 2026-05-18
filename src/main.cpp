extern "C"
{
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
}

#include "actuators/servo_motor.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/cmps12.h"
#include "ble_server.h"
#include "navigation.h" 
#include <cmath>
#include <cstring>

CalypsoAnemometer calypso;
BleServer ble_server;
static constexpr float ARRIVAL_RADIUS = 0.75f;
static constexpr float SIM_STEP = 0.20f;
static float front_wheel_center_deg = FRONT_WHEEL_CENTER_DEG;
static constexpr float FRONT_WHEEL_NEUTRAL_POS =
    (FRONT_WHEEL_CENTER_DEG - FRONT_WHEEL_MIN_DEG) / (FRONT_WHEEL_MAX_DEG - FRONT_WHEEL_MIN_DEG);
ServoMotor front_wheel(6, FRONT_WHEEL_MIN_DEG, FRONT_WHEEL_MAX_DEG, FRONT_WHEEL_NEUTRAL_POS);
ServoMotor sail(7, 0, 250);
CMPS12 cmps12(i2c1);

volatile float last_known_wind = 0.0f; 

static float sail_angle_from_wind(float wind_direction_deg)
{
    float wind = wind_direction_deg;

    while (wind < 0.0f)
        wind += 360.0f;
    while (wind >= 360.0f)
        wind -= 360.0f;

    // Keep only the incidence magnitude: 0..180 deg.
    float beta = wind;
    if (beta > 180.0f)
        beta = 360.0f - beta;

    // Aerodynamic law then clamped physical range.
    float delta = beta - 15.0f;
    if (delta < 0.0f)
        delta = 0.0f;
    if (delta > 90.0f)
        delta = 90.0f;

// Servo command: 180 (open) -> 0 (closed).
    return 180.0f - (delta * 2.0f);
}

static void control_sail_from_wind(float wind_direction_deg)
{
    float target_angle = sail_angle_from_wind(wind_direction_deg);
    sail.rotate_deg(target_angle);
}

static void print_calib()
{
    auto calib = cmps12.calibration();
    printf(" | Calib M:%d A:%d G:%d S:%d\n", calib.mag, calib.acc, calib.gyro, calib.sys);
}

static void on_wind_data(const CalypsoData *data)
{
    last_known_wind = data->wind_direction;
    control_sail_from_wind(data->wind_direction);
    ble_server.update(data);
}



static float cap_vers_point(float dx, float dy)
{
    // Calculate angle
    float angle_rad = atan2f(dx, dy);  // dx=East, dy=North
    float cap = angle_rad * 180.0f / M_PI;

    // Normalisation 0..360
    if (cap < 0.0f) cap += 360.0f;

    return cap;
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

static bool parse_position_update_command(const char *input, float &x, float &y)
{
    return sscanf(input, "p %f %f", &x, &y) == 2;
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
    navigation_init(); 
    navigation_set_center_deg(front_wheel_center_deg);

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

    printf("All Initiated!\n");

    enum class SystemPhase
    {
        CALIBRATION,
        SERVO_ZERO,
        WAIT_TARGET,
        NAVIGATION
    };

    SystemPhase phase = SystemPhase::CALIBRATION;

    // Definition des coordonnees de la cible
    float target_x = 0.0f;
    float target_y = 0.0f;
    float x_actuel = 0.0f;
    float y_actuel = 0.0f;
    bool has_current_position = false;
    bool simulate_position = true;

    static char input_buf[64];
    static int input_idx = 0;
    bool target_prompt_printed = false;

    absolute_time_t last_calib_print = get_absolute_time();

    printf("[PHASE 1] Calibration CMPS12 en cours.\n");
    printf("Tourner le capteur sur plusieurs axes jusqu'a obtenir SYS=3.\n");
    printf("Vous pouvez aussi taper 'start' pour forcer la suite.\n");

    while (1)
    {
        // Lecture non-bloquante : drain tous les octets disponibles
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
                    printf("Nouveau neutre applique : %.1f deg\n", front_wheel_center_deg);
                    target_prompt_printed = false;
                }
                else if (parse_manual_angle_command(input_buf, manual_angle))
                {
                    front_wheel.rotate_deg(manual_angle);
                    printf("Angle servo applique : %.1f deg\n", manual_angle);
                    target_prompt_printed = false;
                }
                else if (parse_direct_degree_input(input_buf, manual_angle))
                {
                    front_wheel.rotate_deg(manual_angle);
                    printf("Angle servo applique (direct): %.1f deg\n", manual_angle);
                    target_prompt_printed = false;
                }
                else if (phase == SystemPhase::CALIBRATION)
                {
                    if (strcmp(input_buf, "start") == 0)
                    {
                        printf("Calibration forcee par l'utilisateur.\n");
                        phase = SystemPhase::SERVO_ZERO;
                    }
                }
                else if (phase == SystemPhase::WAIT_TARGET || phase == SystemPhase::NAVIGATION)
                {
                    float pos_x, pos_y;
                    float new_x, new_y;

                    if (strcmp(input_buf, "sim on") == 0)
                    {
                        simulate_position = true;
                        has_current_position = true;
                        printf("Simulation position activee.\n");
                        target_prompt_printed = false;
                    }
                    else if (strcmp(input_buf, "sim off") == 0)
                    {
                        simulate_position = false;
                        has_current_position = false;
                        printf("Simulation position desactivee. Attente de p x y.\n");
                        target_prompt_printed = false;
                    }
                    else if (parse_position_update_command(input_buf, pos_x, pos_y))
                    {
                        x_actuel = pos_x;
                        y_actuel = pos_y;
                        simulate_position = false;
                        has_current_position = true;
                        printf("Position courante mise a jour : (%.2f, %.2f) [mode reel]\n", x_actuel, y_actuel);
                        target_prompt_printed = false;
                    }
                    else if (sscanf(input_buf, "%f %f", &new_x, &new_y) == 2)
                    {
                        target_x = new_x;
                        target_y = new_y;
                        
                        navigation_init(); 
                        
                        if (simulate_position)
                        {
                            x_actuel = 0.0f;
                            y_actuel = 0.0f;
                            has_current_position = true;
                        }
                        printf("Nouvelle cible validee : (%.2f, %.2f)\n", target_x, target_y);
                        phase = SystemPhase::NAVIGATION;
                        target_prompt_printed = false;
                    }
                    else if (input_buf[0] != '\0')
                    {
                        printf("Format invalide. Entrez: x y | p x y | sim on | sim off | center <deg>\n");
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
                printf("Calib -> M:%d A:%d G:%d S:%d\n", calib.mag, calib.acc, calib.gyro, calib.sys);
                last_calib_print = get_absolute_time();

                if (calib.sys == 3)
                {
                    printf("[PHASE 1] Calibration terminee.\n");
                    phase = SystemPhase::SERVO_ZERO;
                }
            }

            sleep_ms(20);
            continue;
        }

        if (phase == SystemPhase::SERVO_ZERO)
        {
            // Position neutre de direction (a calibrer selon la mecanique).
            front_wheel.rotate_deg(front_wheel_center_deg);
            navigation_init();
            navigation_set_center_deg(front_wheel_center_deg);
            printf("[PHASE 2] Servo direction positionne au neutre (%.1f deg).\n", front_wheel_center_deg);
            printf("[PHASE 3] Entrez la cible: x y (ou angle manuel: a <deg>)\n");
            printf("Sans GPS, la simulation est activee (sim on).\n");
            phase = SystemPhase::WAIT_TARGET;
            target_prompt_printed = false;
            sleep_ms(20);
            continue;
        }

        if (phase == SystemPhase::WAIT_TARGET)
        {
            if (!target_prompt_printed)
            {
                printf("Entrer cible x y, position p x y, sim on/sim off ou a <deg> : ");
                target_prompt_printed = true;
            }

            sleep_ms(20);
            continue;
        }

        // phase == NAVIGATION
        if (!has_current_position && !simulate_position)
        {
            printf("Position inconnue: envoyez p x y ou sim on.\n");
            sleep_ms(250);
            continue;
        }

        if (!has_current_position && simulate_position)
        {
            x_actuel = 0.0f;
            y_actuel = 0.0f;
            has_current_position = true;
            printf("Simulation position initialisee a (0.00, 0.00).\n");
        }

        float dx = target_x - x_actuel;
        float dy = target_y - y_actuel;
        float distance_to_target = sqrtf(dx * dx + dy * dy);

        if (distance_to_target <= ARRIVAL_RADIUS)
        {
            front_wheel.rotate_deg(front_wheel_center_deg);
            navigation_init();
            navigation_set_center_deg(front_wheel_center_deg);
            printf("Cible atteinte (dist=%.2f). Direction au neutre (%.1f deg).\n", distance_to_target, front_wheel_center_deg);
            phase = SystemPhase::WAIT_TARGET;
            target_prompt_printed = false;
            sleep_ms(100);
            continue;
        }

        float cap_cible_gps = cap_vers_point(dx, dy);

        float wind_gap = calculate_heading_error(last_known_wind, cap_cible_gps);
        float cap_to_follow = cap_cible_gps;

        // Zone morte : si la cible est a moins de 45 deg du vent
        if (fabsf(wind_gap) < 45.0f)
        {
            if (wind_gap >= 0.0f)
                cap_to_follow = last_known_wind + 45.0f; // tribord
            else
                cap_to_follow = last_known_wind - 45.0f; // babord
        }

        // Normalise cap_to_follow entre 0 et 360
        if (cap_to_follow < 0.0f)
            cap_to_follow += 360.0f;
        if (cap_to_follow >= 360.0f)
            cap_to_follow -= 360.0f;

        printf("Pos:(%.2f, %.2f) -> Cible:(%.2f, %.2f) | Dist:%.2f | CapSuivi:%.1f\n",
               x_actuel, y_actuel, target_x, target_y, distance_to_target, cap_to_follow);
        steer_to_heading(cap_to_follow, cmps12, front_wheel);

        if (simulate_position && distance_to_target > 0.0f)
        {
            float ratio = SIM_STEP / distance_to_target;
            if (ratio > 1.0f)
                ratio = 1.0f;

            x_actuel += dx * ratio;
            y_actuel += dy * ratio;
        }

        sleep_ms(150);
    }

    return 0;
}