#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "hardware/i2c.h"

// --- Pins I2C ---

class XYZ
{
public:
    static XYZ fromBuffer(unsigned char *buffer);

public:
    uint16_t x, y, z;
};

// --- Structures ---
typedef struct
{
    XYZ mag, acc, gyro;
} RawData;

typedef struct
{
    float cap;
    int8_t pitch;
    int8_t roll;
} NavData;

typedef struct
{
    int mag, acc, gyro, sys;
} CalibData;

class CMPS12
{
public:
    CMPS12(i2c_inst_t *i2c, uint8_t addr = 0x60) : _i2c(i2c), _addr(addr) {}

public:
    RawData raw();
    NavData navigation();
    CalibData calibration();

private:
    bool read_register(unsigned char reg, unsigned char *buffer, uint length);

private:
    i2c_inst_t *_i2c;
    uint8_t _addr;
};

// --- Fonctions ---
void cmps12_init();
