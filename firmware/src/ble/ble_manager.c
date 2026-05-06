#include "ble/ble_manager.h"
#include "ble/cs_adapter.h"
#include "config/app_config.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(ble_manager, LOG_LEVEL_INF);

/* --------------------------------------------------------------------------
 * State
 * ----------------------------------------------------------------------- */

static enum ble_state current_state = BLE_STATE_IDLE;
static struct bt_conn *keyfob_conn;

static ble_rssi_cb_t     rssi_callback;
static ble_distance_cb_t distance_callback;
static ble_state_cb_t    state_callback;

static uint32_t reconnect_delay_ms;
static bool cs_session_active;

/* Reconnect work (runs on system workqueue) */
static void reconnect_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(reconnect_work, reconnect_work_handler);

/* RSSI polling work (reads RSSI from active connection) */
static void rssi_poll_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(rssi_poll_work, rssi_poll_handler);

/* Deterministic CS schedule state */
static int64_t next_poll_deadline_ms;
static uint32_t cs_cycle_counter;
static uint8_t current_antenna;
static uint8_t antenna_failures[NUM_ANTENNAS];
static uint8_t antenna_cooldown[NUM_ANTENNAS];

/* --------------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */


static float rssi_to_distance_fallback(int8_t rssi)
{
    float exponent = ((float)RSSI_REF_DBM - (float)rssi) / (10.0f * PATH_LOSS_EXP);
    float d = powf(10.0f, exponent);

    if (d < 0.1f) {
        d = 0.1f;
    }
    if (d > 30.0f) {
        d = 30.0f;
    }

    return d;
}
static void cs_distance_handler(uint8_t antenna_id,
                                float distance_m,
                                float sigma_m)
{
    if (distance_callback) {
        distance_callback(antenna_id,
                          distance_m,
                          sigma_m,
                          BLE_DISTANCE_SOURCE_CHANNEL_SOUNDING);
    }
}

static void set_state(enum ble_state new_state)
{
    if (current_state != new_state) {
        LOG_INF("State: %d -> %d", current_state, new_state);
        current_state = new_state;
        if (state_callback) {
            state_callback(new_state);
        }
    }
}

static void rf_switch_select(uint8_t antenna_id)
{
    /* Hardware GPIO control for the SP3T RF switch lives here. */
    LOG_DBG("RF switch -> antenna %u", antenna_id);
}

static void cs_reset_watchdog(void)
{
    memset(antenna_failures, 0, sizeof(antenna_failures));
    memset(antenna_cooldown, 0, sizeof(antenna_cooldown));
    cs_cycle_counter = 0;
}

static void cs_prepare_session(void)
{
    cs_reset_watchdog();
    current_antenna = 0;
    rf_switch_select(current_antenna);
    next_poll_deadline_ms = k_uptime_get() + CONN_INTERVAL_MS;
}

static uint8_t cs_next_rotating_antenna(void)
{
    for (uint8_t off = 1; off <= NUM_ANTENNAS; off++) {
        uint8_t cand = (current_antenna + off) % NUM_ANTENNAS;
        if (antenna_cooldown[cand] == 0U) {
            return cand;
        }
    }

    /* Fallback: all antennas in cooldown, keep deterministic round-robin. */
    return (current_antenna + 1) % NUM_ANTENNAS;
}

static void cs_watchdog_on_success(uint8_t antenna)
{
    antenna_failures[antenna] = 0;
}

static void cs_watchdog_on_failure(uint8_t antenna)
{
    if (antenna_failures[antenna] < UINT8_MAX) {
        antenna_failures[antenna]++;
    }

    if (antenna_failures[antenna] >= CS_WATCHDOG_MAX_FAILURES &&
        antenna_cooldown[antenna] == 0U) {
        antenna_cooldown[antenna] = CS_WATCHDOG_COOLDOWN_CYCLES;
        antenna_failures[antenna] = 0;
        LOG_WRN("Antenna %u watchdog tripped, cooldown %u cycles", antenna,
                antenna_cooldown[antenna]);
    }
}

static void cs_tick_cooldowns(void)
{
    cs_cycle_counter++;
    for (uint8_t i = 0; i < NUM_ANTENNAS; i++) {
        if (antenna_cooldown[i] > 0U) {
            antenna_cooldown[i]--;
            if (antenna_cooldown[i] == 0U) {
                LOG_INF("Antenna %u watchdog cooldown ended", i);
            }
        }
    }
}

/**
 * Check if a scanned device is our target keyfob by matching the
 * complete local name in the advertisement data.
 */
static bool is_target_keyfob(struct net_buf_simple *ad)
{
    while (ad->len > 0) {
        uint8_t len = net_buf_simple_pull_u8(ad);

        if (len == 0) {
            break;
        }

        /* Record must include type field and payload bytes. */
        if (ad->len < len) {
            return false;
        }

        uint8_t type = net_buf_simple_pull_u8(ad);
        uint8_t payload_len = len - 1; /* Type byte already consumed */

        if ((type == BT_DATA_NAME_COMPLETE || type == BT_DATA_NAME_SHORTENED) &&
            payload_len == strlen(KEYFOB_NAME) &&
            memcmp(ad->data, KEYFOB_NAME, payload_len) == 0) {
            return true;
        }

        net_buf_simple_pull(ad, payload_len);
    }

    return false;
}

/* --------------------------------------------------------------------------
 * Scan callback
 * ----------------------------------------------------------------------- */

static struct bt_le_scan_param scan_params = {
    .type       = BT_LE_SCAN_TYPE_ACTIVE,
    .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval   = BT_GAP_SCAN_FAST_INTERVAL,
    .window     = BT_GAP_SCAN_FAST_WINDOW,
};

static struct bt_le_conn_param conn_params =
    BT_LE_CONN_PARAM_INIT(
        (CONN_INTERVAL_MS * 1000 / 1250),   /* min interval (1.25 ms units) */
        (CONN_INTERVAL_MS * 1000 / 1250),   /* max interval */
        0,                                   /* latency */
        (SUPERVISION_TIMEOUT_MS / 10));      /* timeout (10 ms units) */

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *ad)
{
    ARG_UNUSED(adv_type);

    /* Make a copy so is_target_keyfob can consume the buffer */
    struct net_buf_simple ad_copy;
    uint8_t ad_data[64];

    if (ad->len > sizeof(ad_data)) {
        return;
    }

    net_buf_simple_init_with_data(&ad_copy, ad_data, sizeof(ad_data));
    memcpy(ad_data, ad->data, ad->len);
    ad_copy.len = ad->len;

    if (!is_target_keyfob(&ad_copy)) {
        return;
    }

    LOG_INF("Keyfob found (RSSI %d dBm)", rssi);

    /* Use scanned RSSI as fallback measurement on current antenna */
    if (rssi_callback) {
        rssi_callback(current_antenna, rssi, k_uptime_get());
    }
    if (distance_callback) {
        distance_callback(current_antenna,
                          rssi_to_distance_fallback(rssi),
                          RSSI_FALLBACK_SIGMA_M,
                          BLE_DISTANCE_SOURCE_RSSI_FALLBACK);
    }
#if CS_MODE_ROTATING
    current_antenna = (current_antenna + 1) % NUM_ANTENNAS;
#endif

    /* Stop scanning and connect */
    bt_le_scan_stop();

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                &conn_params, &keyfob_conn);
    if (err) {
        LOG_ERR("Connection create failed (err %d)", err);
        reconnect_delay_ms = MIN(reconnect_delay_ms * 2, RECONNECT_MAX_MS);
        k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));
        return;
    }
    set_state(BLE_STATE_CONNECTING);
}

/* --------------------------------------------------------------------------
 * Connection callbacks
 * ----------------------------------------------------------------------- */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    ARG_UNUSED(conn);

    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        bt_conn_unref(keyfob_conn);
        keyfob_conn = NULL;
        reconnect_delay_ms = MIN(reconnect_delay_ms * 2, RECONNECT_MAX_MS);
        k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));
        return;
    }

    LOG_INF("Connected to keyfob");
    set_state(BLE_STATE_CONNECTED);
    reconnect_delay_ms = RECONNECT_INITIAL_MS;

    cs_prepare_session();

    cs_session_active = false;
    if (BLE_CS_ENABLE) {
        int cs_err = cs_adapter_start(keyfob_conn, cs_distance_handler);
        if (cs_err == 0) {
            cs_session_active = true;
            LOG_INF("Channel Sounding session started");
        } else {
            LOG_WRN("Channel Sounding unavailable (err %d), falling back to RSSI", cs_err);
        }
    }

    /* Start deterministic RSSI schedule from the next interval. */
    k_work_reschedule(&rssi_poll_work, K_MSEC(CONN_INTERVAL_MS));
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_WRN("Disconnected (reason 0x%02x)", reason);

    bt_conn_unref(keyfob_conn);
    keyfob_conn = NULL;

    /* Cancel RSSI polling */
    k_work_cancel_delayable(&rssi_poll_work);

    if (cs_session_active) {
        cs_adapter_stop();
        cs_session_active = false;
    }

    /* Start reconnect */
    set_state(BLE_STATE_RECONNECTING);
    reconnect_delay_ms = RECONNECT_INITIAL_MS;
    k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);

    LOG_INF("Conn params updated: interval=%u latency=%u timeout=%u",
            interval, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected        = on_connected,
    .disconnected     = on_disconnected,
    .le_param_updated = on_le_param_updated,
};

/* --------------------------------------------------------------------------
 * RSSI polling
 * ----------------------------------------------------------------------- */

static void rssi_poll_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (current_state != BLE_STATE_CONNECTED || !keyfob_conn) {
        return;
    }

    int64_t now_ms = k_uptime_get();
    int8_t rssi;
    int err = bt_conn_le_get_rssi(keyfob_conn, &rssi);

    if (err) {
        LOG_WRN("Failed to read RSSI ant=%u (err %d)", current_antenna, err);
        cs_watchdog_on_failure(current_antenna);
    } else {
        cs_watchdog_on_success(current_antenna);

        int64_t corrected_ts_ms = now_ms - (CS_SESSION_TIMING_US / 1000);
        if (rssi_callback) {
            rssi_callback(current_antenna, rssi, corrected_ts_ms);
        }
        if (distance_callback) {
            distance_callback(current_antenna,
                              rssi_to_distance_fallback(rssi),
                              RSSI_FALLBACK_SIGMA_M,
                              BLE_DISTANCE_SOURCE_RSSI_FALLBACK);
        }
    }

    cs_tick_cooldowns();

#if CS_MODE_ROTATING
    current_antenna = cs_next_rotating_antenna();
    rf_switch_select(current_antenna);
    k_busy_wait(CS_SWITCH_SETTLE_US);
#endif

    /* Deterministic schedule with drift correction to anchor cadence. */
    next_poll_deadline_ms += CONN_INTERVAL_MS;
    now_ms = k_uptime_get();
    int64_t delay_ms = next_poll_deadline_ms - now_ms;

    if (delay_ms < 1) {
        int64_t missed = ((-delay_ms) / CONN_INTERVAL_MS) + 1;
        next_poll_deadline_ms += missed * CONN_INTERVAL_MS;
        delay_ms = 1;
    }

    k_work_reschedule(&rssi_poll_work, K_MSEC(delay_ms));
}

/* --------------------------------------------------------------------------
 * Reconnect logic
 * ----------------------------------------------------------------------- */

static void reconnect_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    LOG_INF("Reconnect attempt (delay=%u ms)", reconnect_delay_ms);

    int err = bt_le_scan_start(&scan_params, scan_cb);
    if (err) {
        LOG_ERR("Scan start failed (err %d)", err);
        reconnect_delay_ms = MIN(reconnect_delay_ms * 2, RECONNECT_MAX_MS);
        k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));
    } else {
        set_state(BLE_STATE_SCANNING);
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

int ble_manager_init(ble_rssi_cb_t rssi_cb,
                     ble_distance_cb_t distance_cb,
                     ble_state_cb_t state_cb)
{
    rssi_callback = rssi_cb;
    distance_callback = distance_cb;
    state_callback = state_cb;

    cs_prepare_session();

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
        return err;
    }

    LOG_INF("BLE Manager initialized (%s mode)",
            CS_MODE_ROTATING ? "rotating" : "single-antenna");
    set_state(BLE_STATE_IDLE);
    reconnect_delay_ms = RECONNECT_INITIAL_MS;

    return 0;
}

int ble_manager_start(void)
{
    LOG_INF("Starting BLE scan for keyfob \"%s\"", KEYFOB_NAME);

    int err = bt_le_scan_start(&scan_params, scan_cb);
    if (err) {
        LOG_ERR("Scan start failed (err %d)", err);
        return err;
    }
    set_state(BLE_STATE_SCANNING);
    return 0;
}

void ble_manager_stop(void)
{
    k_work_cancel_delayable(&reconnect_work);
    k_work_cancel_delayable(&rssi_poll_work);
    if (cs_session_active) {
        cs_adapter_stop();
        cs_session_active = false;
    }
    bt_le_scan_stop();

    if (keyfob_conn) {
        bt_conn_disconnect(keyfob_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(keyfob_conn);
        keyfob_conn = NULL;
    }

    set_state(BLE_STATE_IDLE);
}

enum ble_state ble_manager_get_state(void)
{
    return current_state;
}

int ble_manager_read_rssi(int8_t *rssi)
{
    if (current_state != BLE_STATE_CONNECTED || !keyfob_conn) {
        return -ENOTCONN;
    }
    return bt_conn_le_get_rssi(keyfob_conn, rssi);
}
