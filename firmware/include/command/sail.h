#ifndef SAIL_H
#define SAIL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void sail_init(void);
void sail_set_angle_from_wind(float wind_direction_deg);
void sail_set_manual_angle(float angle_deg);
float sail_angle_from_wind(float wind_direction_deg);

// Les fonctions de parsing appelées par le main doivent être déclarées ici sans conflits
bool sail_parse_manual_angle_command(const char *input, float *angle_deg);
bool sail_parse_direct_degree_input(const char *input, float *angle_deg);
bool sail_parse_console_command(const char *input, float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif // SAIL_H