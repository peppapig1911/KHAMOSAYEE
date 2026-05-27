#include "comms/ntrip_client.h"

#include <cstdio>
#include <cstring>
#include <memory>

#include "log.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/errno.h"

static constexpr const char *MODULE = "NtripClient";

NtripClient::NtripClient(
    const char *host,
    uint16_t port,
    const char *mountpoint,
    IRtcmSink *sink,
    IGgaSource *gga_source)
    : _host(host),
      _port(port),
      _mountpoint(mountpoint),
      _sink(sink),
      _gga_source(gga_source)
{
}

NtripClient::~NtripClient()
{
    if (_socket >= 0)
    {
        lwip_close(_socket);
    }
}

bool NtripClient::init()
{
    struct sockaddr_in addr{};

    addr.sin_family = AF_INET;
    addr.sin_port = htons(_port);

    struct hostent *server = lwip_gethostbyname(_host);
    if (!server)
    {
        LOGE(MODULE, "DNS resolution failed for %s", _host);
        return false;
    }

    memcpy(&addr.sin_addr, server->h_addr, server->h_length);

    _socket = lwip_socket(AF_INET, SOCK_STREAM, 0);

    if (_socket < 0)
    {
        LOGE(MODULE, "Failed to create socket!, errno: %d", errno);
        return false;
    }

    if (lwip_connect(
            _socket,
            (struct sockaddr *)&addr,
            sizeof(addr)) < 0)
    {
        lwip_close(_socket);
        _socket = -1;
        LOGE(MODULE, "Failed to connect to server!, errno: %d", errno);
        return false;
    }

    struct timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    lwip_setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    return sendRequest();
}

bool NtripClient::sendRequest()
{
    char request[512];

    snprintf(
        request,
        sizeof(request),
        "GET /%s HTTP/1.0\r\n"
        "User-Agent: CentraleNantes\r\n"
        "\r\n",
        _mountpoint);

    int sent = lwip_send(
        _socket,
        request,
        strlen(request),
        0);

    if (sent <= 0)
    {
        LOGE(MODULE, "Failed to send request!, errno: %d", errno);
    }
    return sent > 0;
}

bool NtripClient::sendGga()
{
    if (_gga_source == nullptr)
        return false;

    char gga[256];
    if (!_gga_source->build_gga(gga, sizeof(gga)))
        return false;

    int sent = lwip_send(_socket, gga, strlen(gga), 0);
    if (sent <= 0)
    {
        LOGE(MODULE, "Failed to send GGA!, errno: %d", errno);
        return false;
    }

    LOGD(MODULE, "Sent GGA: %s", gga);
    return true;
}

void NtripClient::run()
{
    uint8_t buffer[1024];
    uint32_t last_gga_ms = 0;

    while (true)
    {
        int len = lwip_recv(
            _socket,
            buffer,
            sizeof(buffer),
            0);

        if (len > 0)
        {
            _sink->onRtcmWrite(buffer, static_cast<size_t>(len));
        }
        else if (len == 0)
        {
            LOGE(MODULE, "Server closed NTRIP socket");
            break;
        }
        else
        {
            int err = errno;
            if (err != EAGAIN && err != EWOULDBLOCK)
            {
                LOGE(MODULE, "NTRIP receive error: %d", err);
                break;
            }
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (_gga_source != nullptr && (last_gga_ms == 0 || now_ms - last_gga_ms >= 1000U))
        {
            if (sendGga())
                last_gga_ms = now_ms;
        }
    }
}

void NtripClient::task(void *param)
{
    auto *client =
        static_cast<NtripClient *>(param);

    client->run();

    vTaskDelete(NULL);
}