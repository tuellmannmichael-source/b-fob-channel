#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * BLE connection states.
 */
enum ble_state {
    BLE_STATE_IDLE,
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_RECONNECTING,
};

/** Source of BLE-derived distance measurements. */
enum ble_distance_source {
    BLE_DISTANCE_SOURCE_RSSI_FALLBACK = 0,
    BLE_DISTANCE_SOURCE_CHANNEL_SOUNDING,
};

/**
 * Callback: invoked when a new RSSI measurement is available from the
 * connected keyfob (or from a scanned advertisement during reconnect).
 *
 * @param antenna_id    Antenna index associated with this sample.
 * @param rssi          RSSI in dBm.
 * @param sample_ts_ms  Monotonic sample timestamp (session-corrected).
 */
typedef void (*ble_rssi_cb_t)(uint8_t antenna_id, int8_t rssi,
                              int64_t sample_ts_ms);

/**
 * Callback: invoked when a distance measurement is available.
 */
typedef void (*ble_distance_cb_t)(uint8_t antenna_id,
                                  float distance_m,
                                  float sigma_m,
                                  enum ble_distance_source source);

/**
 * Callback: invoked when the connection state changes.
 */
typedef void (*ble_state_cb_t)(enum ble_state new_state);

/**
 * Initialize the BLE manager.
 *
 * Enables the BLE stack and registers callbacks.
 *
 * @param rssi_cb      Called on each RSSI measurement (may be NULL).
 * @param distance_cb  Called on each distance measurement (may be NULL).
 * @param state_cb     Called on state transitions (may be NULL).
 * @return 0 on success, negative errno on failure.
 */
int ble_manager_init(ble_rssi_cb_t rssi_cb,
                     ble_distance_cb_t distance_cb,
                     ble_state_cb_t state_cb);

int ble_manager_start(void);
void ble_manager_stop(void);
enum ble_state ble_manager_get_state(void);
int ble_manager_read_rssi(int8_t *rssi);

#endif /* BLE_MANAGER_H */
