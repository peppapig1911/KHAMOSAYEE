#ifndef WIND_BLE_SERVER_H
#define WIND_BLE_SERVER_H

#include <stdint.h>
#include "btstack.h"
#include "sensors/calypso_anemometer.h"

/**
 * wind_ble_server.h
 *
 * BLE GATT Peripheral that restreams Calypso anemometer data
 * using standard Bluetooth SIG Environmental Sensing Service (0x181A)
 * and Battery Service (0x180F).
 *
 * Characteristics:
 *   0x2A72 — Apparent Wind Speed   (uint16, 0.01 m/s)
 *   0x2A73 — Apparent Wind Direction (uint16, 0.01 degrees)
 *   0x2A19 — Battery Level          (uint8,  0–100%)
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
};

#endif // WIND_BLE_SERVER_H