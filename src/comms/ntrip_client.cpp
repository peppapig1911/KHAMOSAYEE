#include "comms/ntrip_client.h"

#include <cstdio>
#include <cstring>
#include <memory>

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "FreeRTOS.h"
#include "task.h"

NtripClient::NtripClient(
    const char *host,
    uint16_t port,
    const char *mountpoint, IRtcmSink *sink)
    : _host(host),
      _port(port),
      _mountpoint(mountpoint),
      _sink(sink)
{
}

NtripClient::~NtripClient()
{
    if (_socket >= 0)
    {
        close(_socket);
    }
}

bool NtripClient::init()
{
    struct sockaddr_in addr{};

    addr.sin_family = AF_INET;
    addr.sin_port = htons(_port);

    inet_aton(_host, &addr.sin_addr);

    _socket = socket(AF_INET, SOCK_STREAM, 0);

    if (_socket < 0)
    {
        return false;
    }

    if (::connect(
            _socket,
            (struct sockaddr *)&addr,
            sizeof(addr)) < 0)
    {
        close(_socket);
        _socket = -1;
        return false;
    }

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

    int sent = send(
        _socket,
        request,
        strlen(request),
        0);

    return sent > 0;
}

void NtripClient::run()
{
    uint8_t buffer[1024];

    while (true)
    {
        int len = recv(
            _socket,
            buffer,
            sizeof(buffer),
            0);

        _sink->onRtcmWrite(buffer, len);
    }
}

void NtripClient::task(void *param)
{
    auto *client =
        static_cast<NtripClient *>(param);

    client->run();

    vTaskDelete(NULL);
}