/**
 * calypso_anemometer.cpp
 *
 * BLE GATT Client for the Calypso Ultrasonic Anemometer
 * Pico SDK + BTstack
 *
 * State machine:
 *   IDLE -> CONNECTING -> CONNECTED -> DISCOVERING_SERVICES
 *        -> DISCOVERING_CHARACTERISTICS -> ENABLING_NOTIFICATIONS -> SUBSCRIBED
 */

#include "calypso_anemometer.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

CalypsoAnemometer *CalypsoAnemometer::instance_ = nullptr;

CalypsoAnemometer::CalypsoAnemometer()
    : state_(State::IDLE),
      target_addr_type_(BD_ADDR_TYPE_LE_PUBLIC),
      connection_handle_(HCI_CON_HANDLE_INVALID),
      latest_data_{},
      user_callback_(nullptr),
      service_{},
      measurement_char_{},
      notification_listener_{},
      notification_registered_(false),
      hci_registration_{},
      reconnect_timer_{}
{
}

void CalypsoAnemometer::init(const char *mac_address, uint8_t addr_type, DataCallback callback)
{
    instance_ = this;

    parseMacAddress(mac_address, target_addr_);
    target_addr_type_ = static_cast<bd_addr_type_t>(addr_type);
    user_callback_ = callback;
    state_ = State::IDLE;
    connection_handle_ = HCI_CON_HANDLE_INVALID;
    notification_registered_ = false;
    memset(&latest_data_, 0, sizeof(latest_data_));

    hci_registration_.callback = &hciEventHandlerS;
    hci_add_event_handler(&hci_registration_);

    gatt_client_init();

    printf("[Calypso] Initialized. Target: %s (type: %s)\n",
           mac_address,
           addr_type == 1 ? "random" : "public");
}

void CalypsoAnemometer::connect()
{
    if (state_ == State::CONNECTING || state_ == State::SUBSCRIBED)
    {
        printf("[Calypso] Already connecting or connected.\n");
        return;
    }

    state_ = State::CONNECTING;
    printf("[Calypso] Connecting to %02X:%02X:%02X:%02X:%02X:%02X...\n",
           target_addr_[0], target_addr_[1], target_addr_[2],
           target_addr_[3], target_addr_[4], target_addr_[5]);

    gap_connect(target_addr_, target_addr_type_);
}

void CalypsoAnemometer::disconnect()
{
    if (connection_handle_ != HCI_CON_HANDLE_INVALID)
        gap_disconnect(connection_handle_);
    state_ = State::IDLE;
}

CalypsoData CalypsoAnemometer::getData() const
{
    return latest_data_;
}

bool CalypsoAnemometer::isConnected() const
{
    return state_ == State::SUBSCRIBED;
}

// --- Static trampolines ---

void CalypsoAnemometer::hciEventHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (instance_)
        instance_->hciEventHandler(type, channel, packet, size);
}

void CalypsoAnemometer::gattEventHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (instance_)
        instance_->gattEventHandler(type, channel, packet, size);
}

void CalypsoAnemometer::notificationHandlerS(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (instance_)
        instance_->notificationHandler(type, channel, packet, size);
}

void CalypsoAnemometer::reconnectTimerHandlerS(btstack_timer_source_t *ts)
{
    UNUSED(ts);
    if (instance_)
        instance_->connect();
}

// --- Instance handlers ---

void CalypsoAnemometer::notificationHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;
    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION)
        return;

    uint16_t value_length = gatt_event_notification_get_value_length(packet);
    const uint8_t *value = gatt_event_notification_get_value(packet);
    parseMeasurement(value, value_length);
}

void CalypsoAnemometer::gattEventHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    uint8_t att_status;

    switch (state_)
    {
    case State::DISCOVERING_SERVICES:
        switch (event_type)
        {
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            gatt_event_service_query_result_get_service(packet, &service_);
            printf("[Calypso] Found service 0x%04X\n", SERVICE_UUID);
            break;

        case GATT_EVENT_QUERY_COMPLETE:
            att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS)
            {
                printf("[Calypso] Service discovery failed: 0x%02X\n", att_status);
                disconnect();
                return;
            }
            printf("[Calypso] Discovering characteristics...\n");
            state_ = State::DISCOVERING_CHARACTERISTICS;
            gatt_client_discover_characteristics_for_service_by_uuid16(
                gattEventHandlerS, connection_handle_, &service_, MEASUREMENT_UUID);
            break;

        default:
            break;
        }
        break;

    case State::DISCOVERING_CHARACTERISTICS:
        switch (event_type)
        {
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            gatt_event_characteristic_query_result_get_characteristic(packet, &measurement_char_);
            printf("[Calypso] Found measurement characteristic 0x%04X (handle: 0x%04X)\n",
                   MEASUREMENT_UUID, measurement_char_.value_handle);
            break;

        case GATT_EVENT_QUERY_COMPLETE:
            att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS)
            {
                printf("[Calypso] Characteristic discovery failed: 0x%02X\n", att_status);
                disconnect();
                return;
            }

            if (!notification_registered_)
            {
                gatt_client_listen_for_characteristic_value_updates(
                    &notification_listener_, notificationHandlerS,
                    connection_handle_, &measurement_char_);
                notification_registered_ = true;
            }

            printf("[Calypso] Enabling notifications...\n");
            state_ = State::ENABLING_NOTIFICATIONS;
            gatt_client_write_client_characteristic_configuration(
                gattEventHandlerS, connection_handle_, &measurement_char_,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
            break;

        default:
            break;
        }
        break;

    case State::ENABLING_NOTIFICATIONS:
        if (event_type == GATT_EVENT_QUERY_COMPLETE)
        {
            att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS)
            {
                printf("[Calypso] Enable notifications failed: 0x%02X\n", att_status);
                disconnect();
                return;
            }
            state_ = State::SUBSCRIBED;
            printf("[Calypso] Subscribed to notifications. Receiving data.\n");
        }
        break;

    default:
        break;
    }
}

void CalypsoAnemometer::hciEventHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    switch (hci_event_packet_get_type(packet))
    {
    case HCI_EVENT_LE_META:
    {
        if (hci_event_le_meta_get_subevent_code(packet) != HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
            break;

        uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
        if (status != ERROR_CODE_SUCCESS)
        {
            printf("[Calypso] Connection failed: 0x%02X\n", status);
            state_ = State::DISCONNECTED;
            scheduleReconnect();
            return;
        }
        connection_handle_ = hci_subevent_le_connection_complete_get_connection_handle(packet);
        printf("[Calypso] Connected! (handle: 0x%04X)\n", connection_handle_);

        printf("[Calypso] Discovering services...\n");
        state_ = State::DISCOVERING_SERVICES;
        gatt_client_discover_primary_services_by_uuid16(
            gattEventHandlerS, connection_handle_, SERVICE_UUID);
        break;
    }

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (state_ == State::IDLE)
            break;
        printf("[Calypso] Disconnected.\n");
        notification_registered_ = false;
        state_ = State::DISCONNECTED;
        connection_handle_ = HCI_CON_HANDLE_INVALID;
        scheduleReconnect();
        break;

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
            printf("[Calypso] BTstack ready.\n");
        break;

    default:
        break;
    }
}

void CalypsoAnemometer::scheduleReconnect()
{
    printf("[Calypso] Will reconnect in %lu ms\n", RECONNECT_DELAY_MS);
    btstack_run_loop_set_timer(&reconnect_timer_, RECONNECT_DELAY_MS);
    btstack_run_loop_set_timer_handler(&reconnect_timer_, reconnectTimerHandlerS);
    btstack_run_loop_add_timer(&reconnect_timer_);
}

void CalypsoAnemometer::parseMeasurement(const uint8_t *data, uint16_t length)
{
    if (length < 10)
        return;

    uint16_t speed_raw = data[0] | (data[1] << 8);
    uint16_t dir_raw = data[2] | (data[3] << 8);
    uint8_t batt_raw = data[4];

    latest_data_.wind_speed = speed_raw / 100.0f;
    latest_data_.wind_direction = static_cast<float>(dir_raw);
    latest_data_.battery = (batt_raw * 10) / 100.0f;

    if (user_callback_)
        user_callback_(&latest_data_);
}

void CalypsoAnemometer::parseMacAddress(const char *str, bd_addr_t addr)
{
    unsigned int bytes[6];
    sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
           &bytes[0], &bytes[1], &bytes[2],
           &bytes[3], &bytes[4], &bytes[5]);
    for (int i = 0; i < 6; i++)
        addr[i] = static_cast<uint8_t>(bytes[i]);
}
