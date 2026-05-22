#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "comms/ntrip_client.h"
#include "comms/i2c.h"

enum class RTKState : uint8_t
{
    NONE = 0,
    FLOAT = 1,
    FIXED = 2,
};

struct GNSSData
{
    uint32_t tow_ms;   // GPS time of week, milliseconds
    uint16_t year;     // UTC year
    uint8_t month;     // UTC month
    uint8_t day;       // UTC day
    uint8_t hour;      // UTC hour
    uint8_t min;       // UTC minute
    uint8_t sec;       // UTC second
    double lat;        // degrees, WGS84
    double lon;        // degrees, WGS84
    float altitude;    // meters above MSL
    float geoid_sep;   // meters, default 0 when unavailable
    float speed;       // m/s ground speed
    float heading;     // degrees, motion heading [0, 360)
    float h_acc;       // horizontal accuracy, meters
    float v_acc;       // vertical accuracy, meters
    float h_dop;       // horizontal dilution of precision
    uint8_t fix_type;  // 0=none, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t num_sv;    // satellites used in solution
    uint8_t carr_soln; // RTK carrier solution: 0=none, 1=float, 2=fixed
    bool valid;        // gnssFixOk flag from module
};

class ZedF9P : public IRtcmSink, public IGgaSource
{
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x42;

    ZedF9P(I2C &i2c, uint8_t addr = DEFAULT_ADDR)
        : _i2c(i2c), _addr(addr) {}

public:
    // Returns true if the module ACKs on I2C (basic connectivity check)
    bool probe();

    // Configure the module to auto-output NAV-PVT every epoch over I2C
    void init();

    // Send a NAV-PVT poll request; the module responds with one frame.
    // Call this if auto-output is not working, then call update() to read the response.
    bool poll_pvt();

    // Read pending bytes from the module and parse UBX frames.
    // Returns true if a new NAV-PVT fix was decoded since the last call.
    bool update();

    // Write raw RTCM3 correction bytes to the module.
    // The module applies RTK corrections internally upon receiving valid RTCM3 frames.
    void onRtcmWrite(
        const uint8_t *data,
        size_t len) override;

    const GNSSData &data() const { return _data; }
    RTKState rtk_state() const;
    bool build_gga(char *dst, size_t dst_size) const;

private:
    // Returns how many bytes the module has buffered for reading
    uint16_t available();

    // Read `length` bytes from the DDC stream register (0xFF)
    bool read_stream(uint8_t *dst, uint16_t length);

    // Transmit a UBX frame (sync + class + id + len + payload + checksum)
    bool send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

    // Feed one byte into the UBX parser state machine.
    // Returns true when a valid NAV-PVT frame has just been completed.
    bool parse(uint8_t byte);

    // Extract fields from a validated NAV-PVT payload
    void dispatch_nav_pvt(const uint8_t *payload);

    // Extract fields from a validated NAV-DOP payload
    void dispatch_nav_dop(const uint8_t *payload);

    static void ubx_checksum(uint8_t cls, uint8_t id,
                             const uint8_t *payload, uint16_t len,
                             uint8_t &ck_a, uint8_t &ck_b);

private:
    I2C &_i2c;
    uint8_t _addr;
    GNSSData _data{};

    // UBX parser state machine
    enum class ParseState : uint8_t
    {
        SYNC1,
        SYNC2,
        CLASS,
        ID,
        LEN_L,
        LEN_H,
        PAYLOAD,
        CK_A,
        CK_B,
    };
    ParseState _state = ParseState::SYNC1;

    uint8_t _cls = 0, _id = 0;
    uint16_t _len = 0, _payload_idx = 0;
    uint8_t _ck_a = 0, _ck_b = 0;

    // NAV-PVT payload is 92 bytes; 128 covers any future message we may add
    static constexpr uint16_t MAX_PAYLOAD = 128;
    uint8_t _payload[MAX_PAYLOAD]{};
};
