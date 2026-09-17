#ifndef __BLE_TELEMETRY_H
#define __BLE_TELEMETRY_H

#include "control.h"

/**
 * @brief Initialize the Bluetooth BLE telemetry advertising and GATT services.
 * 
 * Configures the BLE stack, registers characteristics (Voltage, Current, Energy, 
 * Uptime, PWM State, Team Name, and Team Number), and begins advertising.
 * 
 * If CONFIG_BT is disabled in the build, this function behaves as a safe, 
 * compile-ready stub.
 * 
 * @return int 0 on success, or a negative error code on failure.
 */
int ble_telemetry_init(void);

/**
 * @brief Update the BLE characteristics with the latest thread-safe telemetry data.
 * 
 * Converts the telemetry floats to scaled, compact binary integers and triggers
 * GATT notifications if peers are subscribed.
 * 
 * @param telem Pointer to the latest telemetry structure.
 */
void ble_telemetry_update(const struct system_telemetry *telem);

#endif /* __BLE_TELEMETRY_H */
