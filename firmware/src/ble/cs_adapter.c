#include "ble/cs_adapter.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

LOG_MODULE_REGISTER(cs_adapter, LOG_LEVEL_INF);

static cs_distance_cb_t distance_cb;
static bool active;
static cs_adapter_stats_t stats;

int cs_adapter_start(struct bt_conn *conn, cs_distance_cb_t cb)
{
    ARG_UNUSED(conn);

    active = false;
    distance_cb = NULL;
    memset(&stats, 0, sizeof(stats));

    /*
     * TODO: Wire controller-specific BLE 6.0 Channel Sounding APIs here.
     * When CS is available, set distance_cb = cb and active = true on success.
     * Keep explicit -ENOTSUP until supported stack hooks are available.
     *
     * On a completed procedure:
     *   stats.procedures_requested++;
     *   if (success) { stats.procedures_completed++; }
     *   else         { stats.procedures_dropped++; }
     * On timeout:
     *   stats.procedures_timeout++;
     */
    LOG_WRN("Channel Sounding adapter not implemented for current controller/stack");
    return -ENOTSUP;
}

void cs_adapter_stop(void)
{
    if (!active) {
        return;
    }

    /* TODO: stop controller-specific CS session */
    active = false;
    distance_cb = NULL;
}

cs_adapter_stats_t cs_adapter_get_stats(void)
{
    return stats;
}

void cs_adapter_reset_stats(void)
{
    memset(&stats, 0, sizeof(stats));
}
