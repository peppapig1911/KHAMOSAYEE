#pragma once

#include <cstdint>
#include <cstddef>

/**
 * \brief Interface for receiving RTCM data.
 */
class IRtcmSink
{
public:
    virtual void onRtcmWrite(
        const uint8_t *data,
        size_t len) = 0;
};

/**
 * \brief Interface for providing GGA data.
 */
class IGgaSource
{
public:
    virtual bool build_gga(char *dst, size_t dst_size) const = 0;
};

/**
 * \brief NTRIP client for connecting to an NTRIP server.
 */
class NtripClient
{
public:
    /// @brief Constructor for the NTRIP client.
    /// @param host The hostname of the NTRIP server.
    /// @param port The port of the NTRIP server.
    /// @param mountpoint The mountpoint to connect to.
    /// @param sink The sink for receiving RTCM data.
    /// @param gga_source The source for providing GGA data.
    NtripClient(
        const char *host,
        uint16_t port,
        const char *mountpoint,
        IRtcmSink *sink,
        IGgaSource *gga_source);

    ~NtripClient();

    /// @brief Initialize the NTRIP client and connect to the server.
    /// @return True on successful connection, false on failure.
    bool init();

    /// @brief Main loop for the NTRIP client. Call after init() to start receiving data.
    void run();

public:
    static void task(void *param);

private:
    /// @brief Send the initial NTRIP request.
    /// @return True on success, false on failure.
    bool sendRequest();
    /// @brief Send GGA data.
    /// @return True on success, false on failure.
    bool sendGga();

    const char *_host;
    uint16_t _port;
    const char *_mountpoint;
    IRtcmSink *_sink;
    IGgaSource *_gga_source;

    int _socket = -1;
};