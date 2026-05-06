#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "triangulation/position.h"
#include "triangulation/rssi_filter.h"
#include "config/antenna_config.h"
#include "config/app_config.h"

#include <stdbool.h>

/**
 * Source of a distance measurement.
 */
enum triangulation_measurement_source {
    TRI_MEAS_SOURCE_RSSI_FALLBACK = 0,
    TRI_MEAS_SOURCE_CHANNEL_SOUNDING,
};

/**
 * Triangulation engine state.
 */
typedef struct {
    rssi_filter_t filters[NUM_ANTENNAS];
    float         distances[NUM_ANTENNAS];
    float         distance_sigma_m[NUM_ANTENNAS];
    bool          distance_valid[NUM_ANTENNAS];
    position_t    position;
    bool          position_valid;
} triangulation_ctx_t;

/**
 * Initialize the triangulation engine.
 * Sets up Kalman filters for each antenna.
 */
void triangulation_init(triangulation_ctx_t *ctx);

/**
 * Feed a new distance measurement for one antenna.
 *
 * @param ctx         Engine context.
 * @param antenna_id  Antenna index (0..NUM_ANTENNAS-1).
 * @param distance_m  Distance in meters.
 * @param sigma_m     1-sigma uncertainty (meters). Lower => higher weight.
 * @param source      Origin of measurement (CS or RSSI fallback).
 */
void triangulation_feed_distance(triangulation_ctx_t *ctx,
                                 uint8_t antenna_id,
                                 float distance_m,
                                 float sigma_m,
                                 enum triangulation_measurement_source source);

/**
 * Feed a new RSSI measurement from a specific antenna.
 *
 * @param ctx        Engine context.
 * @param antenna_id Index into antenna_positions[] (0..NUM_ANTENNAS-1).
 * @param rssi_dbm   Raw RSSI in dBm.
 */
void triangulation_feed_rssi(triangulation_ctx_t *ctx,
                             uint8_t antenna_id, int8_t rssi_dbm);

/**
 * Compute 2D position using weighted nonlinear least-squares trilateration.
 *
 * @param ctx  Engine context.
 * @param pos  Output: estimated keyfob position.
 * @return     true if the position was computed successfully.
 */
bool triangulation_solve(triangulation_ctx_t *ctx, position_t *pos);

/**
 * Convert RSSI (dBm) to distance (meters) using the log-distance
 * path-loss model.
 */
float rssi_to_distance(float rssi_dbm, float rssi_ref_dbm,
                       float path_loss_exp);

#endif /* TRIANGULATION_H */
