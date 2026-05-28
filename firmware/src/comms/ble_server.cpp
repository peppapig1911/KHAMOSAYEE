/**
 * Service layout (mirrors ble.gatt):
 *   Environmental Sensing Service 0x181A
 *     ├── Apparent Wind Speed     0x2A72  READ | NOTIFY
 *     └── Apparent Wind Direction 0x2A73  READ | NOTIFY
 *   Battery Service 0x180F
 *     └── Battery Level           0x2A19  READ | NOTIFY
 *   Location and Navigation Service 0x1819
 *     ├── Latitude                0x2A69  READ | NOTIFY
 *     ├── Longitude               0x2A6A  READ | NOTIFY
 *     └── Altitude                0x2A6B  READ | NOTIFY
 *     └── Heading                 0xFFF5  READ | NOTIFY
 *     └── GPS Accuracy            0xFFF6  READ | NOTIFY
 *   Manual Control Service 0xFFF0
 *     ├── Control Mode             0xFFF1  READ | WRITE | WRITE_CMD
 *     ├── Front Wheel Position     0xFFF2  READ | WRITE | WRITE_CMD
 *     └── Sail Opening             0xFFF3  READ | WRITE | WRITE_CMD
 *     └── Target Waypoint          0xFFF4  WRITE
 */

#include "comms/ble_server.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "ble.h"

#include "log.h"
#include "controller.h"

// ---------------------------------------------------------------------------
// Advertising data
// ---------------------------------------------------------------------------

// Flags: LE General Discoverable | BR/EDR Not Supported
// Complete 16-bit UUIDs: 0x181A (ESS), 0x180F (Battery), 0x1819 (LNS), 0xFFF0 (Manual Control)
// Complete Local Name: "KHAMOSAYEE"
static const uint8_t adv_data[] = {
    // Flags
    0x02,
    BLUETOOTH_DATA_TYPE_FLAGS,
    0x06,
    // Complete list of 16-bit service UUIDs
    0x09,
    BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
    0x1A,
    0x18, // ESS
    0x0F,
    0x18, // Battery
    0x19,
    0x18, // Location and Navigation
    0xF0,
    0xFF, // Manual Control
    // Complete local name
    0x0C,
    BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'K',
    'H',
    'A',
    'M',
    'O',
    'S',
    'A',
    'Y',
    'E',
    'E',
};

// ---------------------------------------------------------------------------
// State (file-static; accessed by BTstack C callbacks)
// ---------------------------------------------------------------------------

static uint16_t current_wind_speed_raw = 0;     // 0.01 m/s units
static uint16_t current_wind_direction_raw = 0; // 0.01 degree units
static uint8_t current_battery_pct = 0;         // 0-100
static int32_t current_latitude_raw = 0;        // 1e-7 degrees
static int32_t current_longitude_raw = 0;       // 1e-7 degrees
static int32_t current_altitude_raw = 0;        // millimeters
static uint16_t current_heading_raw = 0;        // 0.01 degrees
static uint16_t current_accuracy_raw = 0;       // 0.01 meters
static uint8_t current_control_mode = 0;        // 0 = auto, 1 = manual
static uint8_t current_front_wheel_offset = 0;  // 0-100 percent position
static uint8_t current_sail_opening_pct = 0;    // 0-100

uint8_t get_current_front_wheel_offset()
{
    return current_front_wheel_offset;
}

uint8_t get_current_sail_opening_pct()
{
    return current_sail_opening_pct;
}

static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;

static btstack_packet_callback_registration_t hci_event_cb_reg;

// Pointer back to the owning instance so C callbacks can call startAdvertising()
static BleServer *s_instance = nullptr;

static constexpr const char *MODULE = "BLE Server";

// ---------------------------------------------------------------------------
// ATT read callback
// ---------------------------------------------------------------------------

static uint16_t att_read_callback(hci_con_handle_t connection_handle,
                                  uint16_t att_handle,
                                  uint16_t offset,
                                  uint8_t *buffer,
                                  uint16_t buffer_size)
{
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_0x2A72_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_16(
            current_wind_speed_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0x2A73_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_16(
            current_wind_direction_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0x2A19_01_VALUE_HANDLE)
        return att_read_callback_handle_byte(
            current_battery_pct, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0x2A69_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_32(
            current_latitude_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0x2A6A_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_32(
            current_longitude_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0x2A6B_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_32(
            current_altitude_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0xFFF5_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_16(
            current_heading_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0xFFF6_01_VALUE_HANDLE)
        return att_read_callback_handle_little_endian_16(
            current_accuracy_raw, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0xFFF1_01_VALUE_HANDLE)
        return att_read_callback_handle_byte(current_control_mode, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0xFFF2_01_VALUE_HANDLE)
        return att_read_callback_handle_byte(current_front_wheel_offset, offset, buffer, buffer_size);

    if (att_handle == ATT_CHARACTERISTIC_0xFFF3_01_VALUE_HANDLE)
        return att_read_callback_handle_byte(current_sail_opening_pct, offset, buffer, buffer_size);

    return 0;
}

// ---------------------------------------------------------------------------
// ATT write callback (unused but required by att_server_init)
// ---------------------------------------------------------------------------

static int att_write_callback(hci_con_handle_t connection_handle,
                              uint16_t att_handle,
                              uint16_t transaction_mode,
                              uint16_t offset,
                              uint8_t *buffer,
                              uint16_t buffer_size)
{
    UNUSED(connection_handle);
    UNUSED(transaction_mode);

    if (offset != 0)
        return 0x07;

    if (att_handle == ATT_CHARACTERISTIC_0xFFF1_01_VALUE_HANDLE)
    {
        if (buffer_size < 1)
            return 0x0D;

        current_control_mode = buffer[0] ? 1 : 0;
        controller_set_mode(current_control_mode ? ControlMode::MANUAL : ControlMode::AUTOMATIC);

        return 0;
    }

    if (att_handle == ATT_CHARACTERISTIC_0xFFF2_01_VALUE_HANDLE)
    {
        if (buffer_size < 1)
            return 0x0D;

        current_front_wheel_offset = buffer[0] > 100 ? 100 : buffer[0];
        return 0;
    }

    if (att_handle == ATT_CHARACTERISTIC_0xFFF3_01_VALUE_HANDLE)
    {
        if (buffer_size < 1)
            return 0x0D;

        current_sail_opening_pct = buffer[0] > 100 ? 100 : buffer[0];
        return 0;
    }

    if (att_handle == ATT_CHARACTERISTIC_0xFFF4_01_VALUE_HANDLE)
    {
        if (buffer_size < 8)
            return 0x0D;

        const int32_t latitude_raw = static_cast<int32_t>(
            static_cast<uint32_t>(buffer[0]) |
            (static_cast<uint32_t>(buffer[1]) << 8) |
            (static_cast<uint32_t>(buffer[2]) << 16) |
            (static_cast<uint32_t>(buffer[3]) << 24));
        const int32_t longitude_raw = static_cast<int32_t>(
            static_cast<uint32_t>(buffer[4]) |
            (static_cast<uint32_t>(buffer[5]) << 8) |
            (static_cast<uint32_t>(buffer[6]) << 16) |
            (static_cast<uint32_t>(buffer[7]) << 24));

        controller_set_target_waypoint_gps(static_cast<double>(latitude_raw) * 1e-7,
                                           static_cast<double>(longitude_raw) * 1e-7);
        return 0;
    }

    return 0;
}

// ---------------------------------------------------------------------------
// HCI event handler — manages connection lifecycle & advertising
// ---------------------------------------------------------------------------

static void hci_event_handler(uint8_t packet_type, uint16_t channel,
                              uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    switch (hci_event_packet_get_type(packet))
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
        {
            bd_addr_t local_addr;
            gap_local_bd_addr(local_addr);
            LOGI(MODULE, "Stack ready. MAC: %s", bd_addr_to_str(local_addr));
            if (s_instance)
                s_instance->startAdvertising();
        }
        break;

    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
        {
            con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            LOGI(MODULE, "Client connected (handle 0x%04X).", con_handle);
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        LOGI(MODULE, "Client disconnected. Re-advertising.");
        con_handle = HCI_CON_HANDLE_INVALID;
        if (s_instance)
            s_instance->startAdvertising();
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// BleServer
// ---------------------------------------------------------------------------

void BleServer::init()
{
    s_instance = this;

    l2cap_init();
    sm_init();

    att_server_init(profile_data, att_read_callback, att_write_callback);

    hci_event_cb_reg.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_cb_reg);

    hci_power_control(HCI_POWER_ON);

    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);

    LOGI(MODULE, "Server initialized.");
}

void BleServer::startAdvertising()
{
    gap_advertisements_set_params(
        0x0030, 0x00A0, // 30 ms / 100 ms interval
        0x00,           // connectable undirected
        0x00,
        nullptr, // no direct address
        0x07,    // all channels
        0x00);   // no filter

    gap_advertisements_set_data(sizeof(adv_data), const_cast<uint8_t *>(adv_data));
    gap_advertisements_enable(1);

    bd_addr_t local_addr;
    gap_local_bd_addr(local_addr);
    LOGI(MODULE, "Advertising as \"KHAMOSAYEE\" (%s)...", bd_addr_to_str(local_addr));
}

void BleServer::updateWind(const CalypsoData *data)
{
    current_wind_speed_raw = static_cast<uint16_t>(data->wind_speed * 100.0f);
    current_wind_direction_raw = static_cast<uint16_t>(data->wind_direction * 100.0f);
    const float battery_pct = data->battery < 0.0f ? 0.0f : (data->battery > 100.0f ? 100.0f : data->battery);
    current_battery_pct = static_cast<uint8_t>(battery_pct);

    if (con_handle == HCI_CON_HANDLE_INVALID)
        return;

    att_server_request_can_send_now_event(con_handle);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A72_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_wind_speed_raw), 2);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A73_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_wind_direction_raw), 2);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A19_01_VALUE_HANDLE,
                      &current_battery_pct, 1);
}

void BleServer::updateLocation(const GNSSData *data)
{
    current_latitude_raw = static_cast<int32_t>(data->lat * 10000000.0);
    current_longitude_raw = static_cast<int32_t>(data->lon * 10000000.0);
    current_altitude_raw = static_cast<int32_t>(data->altitude * 1000.0f);
    const float accuracy_m = data->h_acc;
    const float clamped_accuracy_m = accuracy_m < 0.0f ? 0.0f : (accuracy_m > 655.35f ? 655.35f : accuracy_m);
    current_accuracy_raw = static_cast<uint16_t>(clamped_accuracy_m * 100.0f);

    if (con_handle == HCI_CON_HANDLE_INVALID)
        return;

    att_server_request_can_send_now_event(con_handle);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A69_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_latitude_raw), 4);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A6A_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_longitude_raw), 4);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0x2A6B_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_altitude_raw), 4);

    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0xFFF6_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_accuracy_raw), 2);
}

void BleServer::updateHeading(float heading_deg)
{
    float normalized_heading = heading_deg;
    while (normalized_heading < 0.0f)
        normalized_heading += 360.0f;
    while (normalized_heading >= 360.0f)
        normalized_heading -= 360.0f;

    current_heading_raw = static_cast<uint16_t>(normalized_heading * 100.0f);

    if (con_handle == HCI_CON_HANDLE_INVALID)
        return;

    att_server_request_can_send_now_event(con_handle);
    att_server_notify(con_handle,
                      ATT_CHARACTERISTIC_0xFFF5_01_VALUE_HANDLE,
                      reinterpret_cast<uint8_t *>(&current_heading_raw), 2);
}
