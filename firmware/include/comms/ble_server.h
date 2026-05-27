#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdint.h>
#include "btstack.h"
#include "sensors/calypso_anemometer.h"
#include "sensors/zed_f9p.h"

/**
 * ble_server.h
 *
 * BLE GATT Peripheral that restreams Calypso anemometer data and Zed F9P
 * location data using standard Bluetooth SIG services.
 *
 * Characteristics:
 *   0x2A72 — Apparent Wind Speed   (uint16, 0.01 m/s)
 *   0x2A73 — Apparent Wind Direction (uint16, 0.01 degrees)
 *   0x2A19 — Battery Level          (uint8,  0–100%)
 *   0x2A69 — Latitude               (int32,  1e-7 degrees)
 *   0x2A6A — Longitude              (int32,  1e-7 degrees)
 *   0x2A6B — Altitude               (int32,  millimeters)
 */

class BleServer
{
public:
    /// Initialize the BLE peripheral. Call after cyw43_arch_init() and btstack setup.
    void init();

    /// Start advertising. Call after HCI_STATE_WORKING.
    void startAdvertising();

    /// Feed new sensor data to the server. Triggers notifications if clients are subscribed.
    void update(const CalypsoData *data);

    /// Feed GNSS location data to the server. Triggers notifications if clients are subscribed.
    void updateLocation(const GNSSData *data);

    /// Feed compass heading in degrees to the server. Triggers notifications if clients are subscribed.
    void updateHeading(float heading_deg);
};

uint8_t get_current_front_wheel_offset();
uint8_t get_current_sail_opening_pct();

#endif // BLE_SERVER_H