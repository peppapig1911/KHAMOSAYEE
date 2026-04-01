#include "sensors/cmps12.h"

#include "hardware/gpio.h"

RawData CMPS12::raw()
{
    unsigned char buffer[18];
    if (!read_register(0x06, buffer, 18))
    {
        // TODO: Handle error
    }

    return RawData{
        .mag = XYZ::fromBuffer(buffer),
        .acc = XYZ::fromBuffer(buffer + 6),
        .gyro = XYZ::fromBuffer(buffer + 12),
    };
}

NavData CMPS12::navigation()
{
    unsigned char buffer[4];
    if (!read_register(0x02, buffer, 4))
    {
        // TODO: Handle error
    }

    return NavData{
        .cap = ((buffer[0] << 8) | buffer[1]) / 10.0f,
        .pitch = (int8_t)buffer[2],
        .roll = (int8_t)buffer[3],
    };
}

CalibData CMPS12::calibration()
{
    unsigned char buffer[1];
    if (!read_register(0x1E, buffer, 1))
    {
        // TODO: Handle error
    }

    uint8_t status = buffer[0];
    return CalibData{
        .mag = status & 0b11,
        .acc = (status >> 2) & 0b11,
        .gyro = (status >> 4) & 0b11,
        .sys = (status >> 6) & 0b11,
    };
}

bool CMPS12::read_register(unsigned char reg, unsigned char *dst, uint length)
{
    if (i2c_write_blocking(_i2c, _addr, &reg, 1, true) != 1)
        return false;
    if (i2c_read_blocking(_i2c, _addr, dst, length, false) != length)
        return false;

    return true;
}

XYZ XYZ::fromBuffer(unsigned char *buffer)
{
    return XYZ{
        .x = static_cast<uint16_t>((buffer[0] << 8) | buffer[1]),
        .y = static_cast<uint16_t>((buffer[2] << 8) | buffer[3]),
        .z = static_cast<uint16_t>((buffer[4] << 8) | buffer[5]),
    };
}