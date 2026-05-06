#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ble/ble_manager.h"
#include "triangulation/triangulation.h"
#include "config/app_config.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static triangulation_ctx_t tri_ctx;

/* --------------------------------------------------------------------------
 * Callbacks
 * ----------------------------------------------------------------------- */

/** Called by ble_manager when a local RSSI measurement is available. */
static void on_local_rssi(uint8_t antenna_id, int8_t rssi, int64_t sample_ts_ms)
{
    /*
     * Distance updates are ingested through on_distance().
     * Keep RSSI callback for debug/telemetry visibility.
     */
    ARG_UNUSED(antenna_id);
    ARG_UNUSED(rssi);
    ARG_UNUSED(sample_ts_ms);
}

static void on_distance(uint8_t antenna_id,
                        float distance_m,
                        float sigma_m,
                        enum ble_distance_source source)
{
    enum triangulation_measurement_source tri_source =
        (source == BLE_DISTANCE_SOURCE_CHANNEL_SOUNDING)
            ? TRI_MEAS_SOURCE_CHANNEL_SOUNDING
            : TRI_MEAS_SOURCE_RSSI_FALLBACK;

    triangulation_feed_distance(&tri_ctx,
                                antenna_id,
                                distance_m,
                                sigma_m,
                                tri_source);
}

static void on_ble_state(enum ble_state new_state)
{
    switch (new_state) {
    case BLE_STATE_CONNECTED:
        LOG_INF("Keyfob connected -- ranging active");
        break;
    case BLE_STATE_RECONNECTING:
        LOG_WRN("Keyfob disconnected -- reconnecting");
        break;
    default:
        break;
    }
}

int main(void)
{
    LOG_INF("BLE Triangulation Receiver starting");

    triangulation_init(&tri_ctx);

    int ret = ble_manager_init(on_local_rssi, on_distance, on_ble_state);
    if (ret) {
        LOG_ERR("BLE Manager init failed (err %d)", ret);
        return ret;
    }

    ret = ble_manager_start();
    if (ret) {
        LOG_ERR("BLE Manager start failed (err %d)", ret);
        return ret;
    }

    while (1) {
        k_msleep(POSITION_UPDATE_INTERVAL_MS);

        position_t pos;
        if (triangulation_solve(&tri_ctx, &pos)) {
            LOG_INF("Position: (%.2f, %.2f) m", (double)pos.x, (double)pos.y);
        }
    }

    return 0;
}
