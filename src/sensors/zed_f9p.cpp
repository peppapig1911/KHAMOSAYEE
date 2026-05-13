#include "sensors/zed_f9p.h"

#include <cmath>
#include <cstdio>
#include "sensors/zed_f9p.h"

// DDC (I2C) register map
static constexpr uint8_t REG_BYTES_HI = 0xFD; // MSB of bytes available
static constexpr uint8_t REG_BYTES_LO = 0xFE; // LSB of bytes available
static constexpr uint8_t REG_DATA = 0xFF;     // stream data port

// UBX framing
static constexpr uint8_t UBX_SYNC1 = 0xB5;
static constexpr uint8_t UBX_SYNC2 = 0x62;

// UBX class/ID codes used here
static constexpr uint8_t UBX_CLASS_NAV = 0x01;
static constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
static constexpr uint8_t UBX_ID_NAV_DOP = 0x04;
static constexpr uint8_t UBX_CLASS_CFG = 0x06;
static constexpr uint8_t UBX_ID_CFG_MSG = 0x01;

// NAV-PVT payload length
static constexpr uint16_t NAV_PVT_LEN = 92;
static constexpr uint16_t NAV_DOP_LEN = 18;

bool ZedF9P::probe()
{
    // Write the register address 0xFD and check the module ACKs
    uint8_t reg = REG_BYTES_HI;
    return i2c_write_blocking(_i2c, _addr, &reg, 1, false) == 1;
}

void ZedF9P::init()
{
    // CFG-MSG: enable NAV-PVT auto-output on DDC port (port 0) at every epoch
    // Payload: [msgClass, msgID, ratePort0, ratePort1, ..., ratePort5]
    uint8_t cfg[] = {UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1, 0, 0, 0, 0, 0};
    send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_MSG, cfg, sizeof(cfg));

    // NAV-DOP provides HDOP for NMEA GGA generation.
    uint8_t dop_cfg[] = {UBX_CLASS_NAV, UBX_ID_NAV_DOP, 1, 0, 0, 0, 0, 0};
    send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_MSG, dop_cfg, sizeof(dop_cfg));
}

bool ZedF9P::poll_pvt()
{
    // Zero-length UBX frame = poll request; module replies with current NAV-PVT
    return send_ubx(UBX_CLASS_NAV, UBX_ID_NAV_PVT, nullptr, 0);
}

bool ZedF9P::update()
{
    uint16_t n = available();
    if (n == 0)
        return false;

    uint8_t buf[64];
    bool got_fix = false;

    while (n > 0)
    {
        uint16_t chunk = (n > sizeof(buf)) ? static_cast<uint16_t>(sizeof(buf)) : n;
        if (!read_stream(buf, chunk))
            break;
        for (uint16_t i = 0; i < chunk; i++)
        {
            if (parse(buf[i]))
                got_fix = true;
        }
        n -= chunk;
    }
    return got_fix;
}

void ZedF9P::onRtcmWrite(const uint8_t *data, size_t len)
{
    // RTCM3 bytes are written directly to the module's DDC input without a register prefix
    i2c_write_blocking(_i2c, _addr, data, len, false);
}

uint16_t ZedF9P::available()
{
    uint8_t reg = REG_BYTES_HI;
    uint8_t bytes[2];
    if (i2c_write_blocking(_i2c, _addr, &reg, 1, true) != 1)
        return 0;
    if (i2c_read_blocking(_i2c, _addr, bytes, 2, false) != 2)
        return 0;
    uint16_t n = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
    // Module returns 0xFFFF when the output buffer is empty
    return (n == 0xFFFF) ? 0 : n;
}

bool ZedF9P::read_stream(uint8_t *dst, uint16_t length)
{
    uint8_t reg = REG_DATA;
    if (i2c_write_blocking(_i2c, _addr, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(_i2c, _addr, dst, length, false) == length;
}

bool ZedF9P::send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    // Maximum frame we ever send is a short config message; 128 bytes is ample
    uint8_t frame[128];
    uint16_t total = 6 + len + 2;
    if (total > sizeof(frame))
        return false;

    uint8_t ck_a, ck_b;
    ubx_checksum(cls, id, payload, len, ck_a, ck_b);

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = static_cast<uint8_t>(len & 0xFF);
    frame[5] = static_cast<uint8_t>(len >> 8);
    for (uint16_t i = 0; i < len; i++)
        frame[6 + i] = payload[i];
    frame[6 + len] = ck_a;
    frame[6 + len + 1] = ck_b;

    // Write directly — u-blox DDC does not use a register prefix for writes
    return i2c_write_blocking(_i2c, _addr, frame, total, false) == total;
}

bool ZedF9P::parse(uint8_t byte)
{
    switch (_state)
    {
    case ParseState::SYNC1:
        if (byte == UBX_SYNC1)
            _state = ParseState::SYNC2;
        break;

    case ParseState::SYNC2:
        _state = (byte == UBX_SYNC2) ? ParseState::CLASS : ParseState::SYNC1;
        break;

    case ParseState::CLASS:
        _cls = byte;
        _ck_a = byte;
        _ck_b = byte;
        _state = ParseState::ID;
        break;

    case ParseState::ID:
        _id = byte;
        _ck_a += byte;
        _ck_b += _ck_a;
        _state = ParseState::LEN_L;
        break;

    case ParseState::LEN_L:
        _len = byte;
        _ck_a += byte;
        _ck_b += _ck_a;
        _state = ParseState::LEN_H;
        break;

    case ParseState::LEN_H:
        _len |= static_cast<uint16_t>(byte) << 8;
        _ck_a += byte;
        _ck_b += _ck_a;
        _payload_idx = 0;
        _state = (_len == 0) ? ParseState::CK_A : ParseState::PAYLOAD;
        break;

    case ParseState::PAYLOAD:
        if (_payload_idx < MAX_PAYLOAD)
            _payload[_payload_idx] = byte;
        _ck_a += byte;
        _ck_b += _ck_a;
        if (++_payload_idx >= _len)
            _state = ParseState::CK_A;
        break;

    case ParseState::CK_A:
        if (byte != _ck_a)
            _state = ParseState::SYNC1;
        else
            _state = ParseState::CK_B;
        break;

    case ParseState::CK_B:
        _state = ParseState::SYNC1;
        if (byte != _ck_b)
            break;
        if (_cls == UBX_CLASS_NAV && _id == UBX_ID_NAV_PVT && _len >= NAV_PVT_LEN)
        {
            dispatch_nav_pvt(_payload);
            return true;
        }
        if (_cls == UBX_CLASS_NAV && _id == UBX_ID_NAV_DOP && _len >= NAV_DOP_LEN)
        {
            dispatch_nav_dop(_payload);
        }
        break;
    }
    return false;
}

RTKState ZedF9P::rtk_state() const
{
    switch (_data.carr_soln)
    {
    case 1:
        return RTKState::FLOAT;
    case 2:
        return RTKState::FIXED;
    default:
        return RTKState::NONE;
    }
}

bool ZedF9P::build_gga(char *dst, size_t dst_size) const
{
    if (!dst || dst_size == 0)
        return false;

    if (_data.year == 0 || _data.month == 0 || _data.day == 0)
        return false;

    char lat_field[16];
    char lon_field[16];
    double abs_lat = std::fabs(_data.lat);
    double abs_lon = std::fabs(_data.lon);
    int lat_deg = static_cast<int>(abs_lat);
    int lon_deg = static_cast<int>(abs_lon);
    double lat_min = (abs_lat - static_cast<double>(lat_deg)) * 60.0;
    double lon_min = (abs_lon - static_cast<double>(lon_deg)) * 60.0;

    if (std::snprintf(lat_field, sizeof(lat_field), "%02d%07.4f", lat_deg, lat_min) < 0)
        return false;
    if (std::snprintf(lon_field, sizeof(lon_field), "%03d%07.4f", lon_deg, lon_min) < 0)
        return false;

    char time_field[16];
    uint32_t centis = (_data.tow_ms % 1000U) / 10U;
    if (std::snprintf(time_field, sizeof(time_field), "%02u%02u%02u.%02u",
                      static_cast<unsigned>(_data.hour),
                      static_cast<unsigned>(_data.min),
                      static_cast<unsigned>(_data.sec),
                      static_cast<unsigned>(centis)) < 0)
        return false;

    uint8_t quality = 0;
    switch (rtk_state())
    {
    case RTKState::FIXED:
        quality = 4;
        break;
    case RTKState::FLOAT:
        quality = 5;
        break;
    default:
        quality = _data.valid ? 1 : 0;
        break;
    }

    float hdop = (_data.h_dop > 0.0f) ? _data.h_dop : 99.9f;
    const char lat_hemi = (_data.lat >= 0.0) ? 'N' : 'S';
    const char lon_hemi = (_data.lon >= 0.0) ? 'E' : 'W';

    char body[160];
    int body_len = std::snprintf(
        body, sizeof(body),
        "GNGGA,%s,%s,%c,%s,%c,%u,%02u,%.1f,%.1f,M,%.1f,M,,",
        time_field,
        lat_field,
        lat_hemi,
        lon_field,
        lon_hemi,
        static_cast<unsigned>(quality),
        static_cast<unsigned>(_data.num_sv),
        hdop,
        _data.altitude,
        _data.geoid_sep);

    if (body_len < 0 || static_cast<size_t>(body_len) >= sizeof(body))
        return false;

    uint8_t checksum = 0;
    for (int i = 0; i < body_len; i++)
        checksum ^= static_cast<uint8_t>(body[i]);

    int written = std::snprintf(dst, dst_size, "$%s*%02X\r\n", body, checksum);
    return written >= 0 && static_cast<size_t>(written) < dst_size;
}

void ZedF9P::dispatch_nav_pvt(const uint8_t *p)
{
    _data.tow_ms = static_cast<uint32_t>(p[0]) |
                   (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) |
                   (static_cast<uint32_t>(p[3]) << 24);
    _data.year = static_cast<uint16_t>(p[4]) |
                 (static_cast<uint16_t>(p[5]) << 8);
    _data.month = p[6];
    _data.day = p[7];
    _data.hour = p[8];
    _data.min = p[9];
    _data.sec = p[10];
    _data.fix_type = p[20];
    // byte 21 flags: bit 0 = gnssFixOk, bits 7:6 = carrSoln (RTK)
    bool fix_ok = (p[21] & 0x01) != 0;
    _data.carr_soln = (p[21] >> 6) & 0x03;
    _data.num_sv = p[23];

    auto le32s = [](const uint8_t *b) -> int32_t
    {
        return static_cast<int32_t>(b[0]) |
               (static_cast<int32_t>(b[1]) << 8) |
               (static_cast<int32_t>(b[2]) << 16) |
               (static_cast<int32_t>(b[3]) << 24);
    };
    auto le32u = [](const uint8_t *b) -> uint32_t
    {
        return static_cast<uint32_t>(b[0]) |
               (static_cast<uint32_t>(b[1]) << 8) |
               (static_cast<uint32_t>(b[2]) << 16) |
               (static_cast<uint32_t>(b[3]) << 24);
    };

    _data.geoid_sep = (le32s(p + 32) * 1e-3f) - (le32s(p + 36) * 1e-3f);
    _data.lon = le32s(p + 24) * 1e-7; // 1e-7 deg → degrees
    _data.lat = le32s(p + 28) * 1e-7;
    _data.altitude = le32s(p + 36) * 1e-3f; // mm → meters (hMSL)
    _data.h_acc = le32u(p + 40) * 1e-3f;    // mm → meters
    _data.v_acc = le32u(p + 44) * 1e-3f;
    _data.speed = le32s(p + 60) * 1e-3f;   // mm/s → m/s
    _data.heading = le32s(p + 64) * 1e-5f; // 1e-5 deg → degrees

    _data.valid = fix_ok && (_data.fix_type >= 2);
}

void ZedF9P::dispatch_nav_dop(const uint8_t *p)
{
    auto le16u = [](const uint8_t *b) -> uint16_t
    {
        return static_cast<uint16_t>(b[0]) |
               (static_cast<uint16_t>(b[1]) << 8);
    };

    _data.h_dop = le16u(p + 12) * 0.01f;
}

void ZedF9P::ubx_checksum(uint8_t cls, uint8_t id,
                          const uint8_t *payload, uint16_t len,
                          uint8_t &ck_a, uint8_t &ck_b)
{
    ck_a = ck_b = 0;
    auto feed = [&](uint8_t b)
    {
        ck_a += b;
        ck_b += ck_a;
    };
    feed(cls);
    feed(id);
    feed(static_cast<uint8_t>(len & 0xFF));
    feed(static_cast<uint8_t>(len >> 8));
    for (uint16_t i = 0; i < len; i++)
        feed(payload[i]);
}
