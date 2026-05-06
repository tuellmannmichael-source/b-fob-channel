#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(keyfob, LOG_LEVEL_INF);

#define HEARTBEAT_INTERVAL_MS       100
#define CS_TELEMETRY_INTERVAL_MS    1000

/* --------------------------------------------------------------------------
 * Custom GATT service: Heartbeat
 *
 * The keyfob sends a 1-byte sequence counter as a periodic notification for
 * liveness/debug only. Distance measurement is handled by Channel Sounding.
 *
 * Service UUID:  12345678-1234-5678-1234-56789abcdef0
 * Char UUID:     12345678-1234-5678-1234-56789abcdef1
 * ----------------------------------------------------------------------- */

#define HEARTBEAT_SVC_UUID \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define HEARTBEAT_CHAR_UUID \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 heartbeat_svc_uuid = BT_UUID_INIT_128(HEARTBEAT_SVC_UUID);
static struct bt_uuid_128 heartbeat_char_uuid = BT_UUID_INIT_128(HEARTBEAT_CHAR_UUID);

enum cs_role {
    CS_ROLE_RESPONDER,
    CS_ROLE_INITIATOR,
};

static uint8_t heartbeat_seq;
static bool notify_enabled;
static struct bt_conn *active_conn;

static struct {
    bool active;
    uint32_t measurements;
    uint32_t errors;
    int64_t started_at_ms;
    enum cs_role role;
} cs_session = {
    .role = CS_ROLE_RESPONDER,
};

static const char *cs_role_str(enum cs_role role)
{
    return (role == CS_ROLE_INITIATOR) ? "initiator" : "responder";
}

static void cs_session_stop(void)
{
    if (!cs_session.active) {
        return;
    }

    cs_session.active = false;
    LOG_INF("CS session stopped");
}

static int cs_session_start(struct bt_conn *conn)
{
    ARG_UNUSED(conn);

    /* Keyfob acts as CS responder in this architecture. */
    cs_session.active = true;
    cs_session.measurements = 0U;
    cs_session.errors = 0U;
    cs_session.started_at_ms = k_uptime_get();

    LOG_INF("CS session started (%s)", cs_role_str(cs_session.role));
    return 0;
}

static void cs_measurement_tick(bool ok)
{
    if (!cs_session.active) {
        return;
    }

    if (ok) {
        cs_session.measurements++;
    } else {
        cs_session.errors++;
    }
}

static void cs_log_telemetry(void)
{
    if (!cs_session.active) {
        return;
    }

    int64_t elapsed_ms = k_uptime_get() - cs_session.started_at_ms;
    uint32_t rate_hz = 0U;

    if (elapsed_ms > 0) {
        rate_hz = (uint32_t)((cs_session.measurements * 1000LL) / elapsed_ms);
    }

    LOG_INF("CS telemetry: active=%d rate=%uHz errors=%u",
            cs_session.active,
            rate_hz,
            cs_session.errors);
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Heartbeat notifications %s", notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(heartbeat_svc,
    BT_GATT_PRIMARY_SERVICE(&heartbeat_svc_uuid),
    BT_GATT_CHARACTERISTIC(&heartbeat_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* --------------------------------------------------------------------------
 * Advertising
 * ----------------------------------------------------------------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "KEYFOB", 6),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, HEARTBEAT_SVC_UUID),
};

static void start_advertising(void)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising start failed (err %d)", err);
    } else {
        LOG_INF("Advertising started");
    }
}

/* --------------------------------------------------------------------------
 * Connection callbacks
 * ----------------------------------------------------------------------- */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        start_advertising();
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected: %s", addr);

    if (active_conn) {
        bt_conn_unref(active_conn);
        active_conn = NULL;
    }

    active_conn = bt_conn_ref(conn);

    /* Reconnect path: always reset and restart CS session on a fresh link. */
    cs_session_stop();
    if (cs_session_start(conn) != 0) {
        cs_session.errors++;
    }

    /* Advertising restarts on disconnect (see on_disconnected). */
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason 0x%02x)", reason);

    cs_session_stop();

    if (active_conn) {
        bt_conn_unref(active_conn);
        active_conn = NULL;
    }

    start_advertising();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/* --------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------- */

int main(void)
{
    LOG_INF("BLE Keyfob starting");

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
        return err;
    }

    start_advertising();

    int64_t last_telemetry_ms = k_uptime_get();

    /* Main loop:
     * - heartbeat = liveness/debug only
     * - CS stats = session health telemetry */
    while (1) {
        k_msleep(HEARTBEAT_INTERVAL_MS);

        /*
         * TODO: Wire actual CS measurement callbacks here.
         * When controller-specific CS APIs are available, call
         * cs_measurement_tick(true/false) from the CS result callback
         * instead of from this loop.
         */

        if (notify_enabled && active_conn) {
            heartbeat_seq++;
            int err_notify = bt_gatt_notify(active_conn, &heartbeat_svc.attrs[1],
                                            &heartbeat_seq, sizeof(heartbeat_seq));
            if (err_notify) {
                LOG_WRN("Heartbeat notify failed (err %d)", err_notify);
            }
        }

        int64_t now = k_uptime_get();
        if ((now - last_telemetry_ms) >= CS_TELEMETRY_INTERVAL_MS) {
            cs_log_telemetry();
            last_telemetry_ms = now;
        }
    }

    return 0;
}
