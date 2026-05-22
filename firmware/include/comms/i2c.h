#ifndef __I2C_H
#define __I2C_H

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "semphr.h"

/**
 * \file comms/i2c.h
 *
 * \brief C++ wrapper for I2C communication, providing thread safety via a mutex.
 *
 * This class encapsulates the I2C hardware instance and provides methods for
 * initializing the I2C peripheral, as well as reading and writing data to I2C
 * devices. The read and write methods are thread-safe, ensuring that concurrent
 * access to the I2C bus is properly synchronized.
 *
 */
class I2C
{
public:
    /// Constructor takes the I2C instance (e.g. i2c0 or i2c1), desired baudrate, and GPIO pins for SDA and SCL.
    /// The init() method must be called before using read/write.
    I2C(i2c_inst_t *instance, uint baudrate, uint sda, uint scl);

    /// Initialize the I2C peripheral with the given baudrate. Returns the actual baudrate set.
    uint init();

    /// Read bytes from the I2C device at the given address and register.
    /// Returns the number of bytes read on success, -1 on failure (e.g. NACK or timeout).
    int read(uint8_t addr, uint8_t *dst, size_t len, bool nostop = false);

    /// Write bytes to the I2C device at the given address and register.
    /// Returns the number of bytes written on success, -1 on failure (e.g. NACK or timeout).
    int write(uint8_t addr, const uint8_t *src, size_t len, bool nostop = false);

private:
    uint _baudrate;
    uint _sda, _scl;
    i2c_inst_t *_instance;
    SemaphoreHandle_t _mutex = nullptr;
};

#endif // __I2C_H