#pragma once

#include <stdint.h>
#include <stdbool.h>

extern "C"
{
#include "btstack.h"
}

struct CalypsoData
{
    float wind_speed;     // m/s
    float wind_direction; // degrees
    float battery;        // volts
};

class CalypsoAnemometer
{
public:
    using DataCallback = void (*)(const CalypsoData *);

    CalypsoAnemometer();

    void init(const char *mac_address, uint8_t addr_type, DataCallback callback);
    void connect();
    void disconnect();
    CalypsoData getData() const;
    bool isConnected() const;

private:
    enum class State
    {
        IDLE,
        CONNECTING,
        CONNECTED,
        DISCOVERING_SERVICES,
        DISCOVERING_CHARACTERISTICS,
        ENABLING_NOTIFICATIONS,
        SUBSCRIBED,
        DISCONNECTED,
    };

    static constexpr uint16_t SERVICE_UUID = 0x180D;
    static constexpr uint16_t MEASUREMENT_UUID = 0x2A39;
    static constexpr uint32_t RECONNECT_DELAY_MS = 3000;

    State state_;
    bd_addr_t target_addr_;
    bd_addr_type_t target_addr_type_;
    hci_con_handle_t connection_handle_;
    CalypsoData latest_data_;
    DataCallback user_callback_;
    gatt_client_service_t service_;
    gatt_client_characteristic_t measurement_char_;
    gatt_client_notification_t notification_listener_;
    bool notification_registered_;
    btstack_packet_callback_registration_t hci_registration_;
    btstack_timer_source_t reconnect_timer_;

    static CalypsoAnemometer *instance_;

    // Static trampolines for BTstack C callbacks
    static void hciEventHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void gattEventHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void notificationHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void reconnectTimerHandlerS(btstack_timer_source_t *ts);

    void hciEventHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    void gattEventHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    void notificationHandler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
    void scheduleReconnect();
    void parseMeasurement(const uint8_t *data, uint16_t length);
    static void parseMacAddress(const char *str, bd_addr_t addr);
};
