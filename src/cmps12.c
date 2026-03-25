#include "cmps12.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "stdlib.h"
#include <stdio.h>

void cmps12_init()
{
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
}

bool read_raw_data(RawData *data)
{
    uint8_t reg = REG_FIRST_RAW;
    uint8_t buffer[18];

    if (i2c_write_blocking(I2C_PORT, CMPS12_ADDR, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(I2C_PORT, CMPS12_ADDR, buffer, 18, false) != 18)
        return false;

    data->mag_x = (buffer[0] << 8) | buffer[1];
    data->mag_y = (buffer[2] << 8) | buffer[3];
    data->mag_z = (buffer[4] << 8) | buffer[5];
    data->acc_x = (buffer[6] << 8) | buffer[7];
    data->acc_y = (buffer[8] << 8) | buffer[9];
    data->acc_z = (buffer[10] << 8) | buffer[11];
    data->gyro_x = (buffer[12] << 8) | buffer[13];
    data->gyro_y = (buffer[14] << 8) | buffer[15];
    data->gyro_z = (buffer[16] << 8) | buffer[17];
    return true;
}

bool read_navigation_data(NavData *data)
{
    uint8_t reg = REG_NAV_DATA;
    uint8_t buffer[4];

    if (i2c_write_blocking(I2C_PORT, CMPS12_ADDR, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(I2C_PORT, CMPS12_ADDR, buffer, 4, false) != 4)
        return false;

    data->cap = ((buffer[0] << 8) | buffer[1]) / 10.0f;
    data->pitch = (int8_t)buffer[2];
    data->roll = (int8_t)buffer[3];
    return true;
}

bool check_calibration(CalibData *data)
{
    uint8_t reg = REG_CALIB_STAT;
    uint8_t status;

    if (i2c_write_blocking(I2C_PORT, CMPS12_ADDR, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(I2C_PORT, CMPS12_ADDR, &status, 1, false) != 1)
        return false;

    data->mag_cal = status & 0x03;
    data->acc_cal = (status >> 2) & 0x03;
    data->gyro_cal = (status >> 4) & 0x03;
    data->sys_cal = (status >> 6) & 0x03;
    return true;
}