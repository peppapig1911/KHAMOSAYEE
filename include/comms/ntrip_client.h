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

class NtripClient
{
public:
    NtripClient(
        const char *host,
        uint16_t port,
        const char *mountpoint,
        IRtcmSink *sink);

    ~NtripClient();

    bool init();
    void run();

public:
    static void task(void *param);

private:
    bool sendRequest();

    const char *_host;
    uint16_t _port;
    const char *_mountpoint;
    IRtcmSink *_sink;

    int _socket = -1;
};