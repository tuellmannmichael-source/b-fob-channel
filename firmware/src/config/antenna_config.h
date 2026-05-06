#ifndef ANTENNA_CONFIG_H
#define ANTENNA_CONFIG_H

#include "app_config.h"

/*
 * Antenna positions in vehicle coordinate system.
 *
 * Origin: center of rear axle
 * X-axis: right (passenger side)
 * Y-axis: forward (towards front of vehicle)
 *
 * Units: meters
 *
 * Adjust these to match your actual antenna mounting positions.
 *
 *         Y (forward)
 *         ^
 *         |      A (dashboard)
 *         |
 *         |
 *    B ---+--- C
 *  (L-rear)   (R-rear)
 *         |
 *         +-------> X (right)
 */

typedef struct {
    float x;
    float y;
    float rssi_ref;      /* Per-antenna calibrated RSSI at 1m (dBm) */
    float path_loss_exp;  /* Per-antenna calibrated path-loss exponent */
} antenna_pos_t;

static const antenna_pos_t antenna_positions[NUM_ANTENNAS] = {
    /* Antenna A: dashboard center */
    {
        .x = 0.0f,
        .y = 4.5f,
        .rssi_ref = RSSI_REF_DBM,
        .path_loss_exp = PATH_LOSS_EXP,
    },
    /* Antenna B: left rear pillar */
    {
        .x = -0.9f,
        .y = 0.0f,
        .rssi_ref = RSSI_REF_DBM,
        .path_loss_exp = PATH_LOSS_EXP,
    },
    /* Antenna C: right rear pillar */
    {
        .x = 0.9f,
        .y = 0.0f,
        .rssi_ref = RSSI_REF_DBM,
        .path_loss_exp = PATH_LOSS_EXP,
    },
};

#endif /* ANTENNA_CONFIG_H */
