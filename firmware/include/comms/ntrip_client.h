#pragma once

#include <cstdint>
#include <cstddef>

class IRtcmSink
{
public:
    virtual void onRtcmWrite(
        const uint8_t *data,
        size_t len) = 0;
};

class IGgaSource
{
public:
    virtual bool build_gga(char *dst, size_t dst_size) const = 0;
};

class NtripClient
{
public:
    NtripClient(
        const char *host,
        uint16_t port,
        const char *mountpoint,
        IRtcmSink *sink,
        IGgaSource *gga_source);

    ~NtripClient();

    bool init();
    void run();

public:
    static void task(void *param);

private:
    bool sendRequest();
    bool sendGga();

    const char *_host;
    uint16_t _port;
    const char *_mountpoint;
    IRtcmSink *_sink;
    IGgaSource *_gga_source;

    int _socket = -1;
};