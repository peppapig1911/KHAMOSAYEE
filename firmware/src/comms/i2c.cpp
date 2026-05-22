#include "comms/i2c.h"
#include "log.h"

static constexpr const char *MODULE = "I2C";

I2C::I2C(i2c_inst_t *instance, uint baudrate, uint sda, uint scl)
    : _instance(instance), _baudrate(baudrate), _sda(sda), _scl(scl)
{
    LOGD(MODULE, "I2C instance created with baudrate %u, SDA pin %u, SCL pin %u",
         baudrate, sda, scl);
}

uint I2C::init()
{
    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_pull_up(_scl);
    uint ret = i2c_init(_instance, _baudrate);
    _mutex = xSemaphoreCreateMutex();
    configASSERT(_mutex != nullptr);
    LOGD(MODULE, "I2C initialized at %u baud", ret);
    return ret;
}

int I2C::read(uint8_t addr, uint8_t *dst, size_t len, bool nostop)
{
    LOGD(MODULE, "Reading from I2C device at address 0x%02X", addr);
    xSemaphoreTake(_mutex, portMAX_DELAY);
    int result = i2c_read_blocking(_instance, addr, dst, len, nostop);
    xSemaphoreGive(_mutex);
    return result;
}

int I2C::write(uint8_t addr, const uint8_t *src, size_t len, bool nostop)
{
    LOGD(MODULE, "Writing %u bytes to I2C device at address 0x%02X", len, addr);
    xSemaphoreTake(_mutex, portMAX_DELAY);
    int result = i2c_write_blocking(_instance, addr, src, len, nostop);
    xSemaphoreGive(_mutex);
    return result;
}
