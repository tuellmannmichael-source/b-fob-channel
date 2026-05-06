#ifndef CS_ADAPTER_H
#define CS_ADAPTER_H

#include <zephyr/bluetooth/conn.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Callback invoked by adapter when a Channel Sounding distance is available.
 */
typedef void (*cs_distance_cb_t)(uint8_t antenna_id,
                                 float distance_m,
                                 float sigma_m);

/**
 * Start Channel Sounding session for an active connection.
 *
 * Returns 0 on success, negative errno otherwise.
 */
int cs_adapter_start(struct bt_conn *conn, cs_distance_cb_t cb);

/** Stop Channel Sounding session if active. */
void cs_adapter_stop(void);

/** CS adapter runtime statistics. */
typedef struct {
    uint32_t procedures_requested;
    uint32_t procedures_completed;
    uint32_t procedures_dropped;
    uint32_t procedures_timeout;
} cs_adapter_stats_t;

/** Get current adapter statistics (read-only snapshot). */
cs_adapter_stats_t cs_adapter_get_stats(void);

/** Reset adapter statistics counters. */
void cs_adapter_reset_stats(void);

#endif /* CS_ADAPTER_H */
