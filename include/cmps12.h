#ifndef CMPS12_H
#define CMPS12_H

#include <stdint.h>
#include <stdbool.h>

// --- Configuration CMPS12 ---
#define CMPS12_ADDR     0x60
#define REG_FIRST_RAW   0x06
#define REG_NAV_DATA    0x02
#define REG_CALIB_STAT  0x1E

// --- Pins I2C ---
#define I2C_PORT  i2c1
#define PIN_SDA   26
#define PIN_SCL   27

// --- Structures ---
typedef struct {
    int16_t mag_x, mag_y, mag_z;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} RawData;

typedef struct {
    float   cap;
    int8_t  pitch;
    int8_t  roll;
} NavData;

typedef struct {
    int mag_cal, acc_cal, gyro_cal, sys_cal;
} CalibData;

// --- Fonctions ---
void cmps12_init();
bool read_raw_data(RawData* data);
bool read_navigation_data(NavData* data);
bool check_calibration(CalibData* data);

#endif