#ifndef RSSI_FILTER_H
#define RSSI_FILTER_H

/**
 * 1D Kalman filter state for RSSI smoothing.
 *
 * Model:
 *   State: x = true RSSI (dBm)
 *   Process:     x_k = x_{k-1} + w,  w ~ N(0, Q)
 *   Measurement: z_k = x_k + v,      v ~ N(0, R)
 */
typedef struct {
    float x;    /* State estimate (filtered RSSI in dBm) */
    float p;    /* Estimate covariance */
    float q;    /* Process noise variance */
    float r;    /* Measurement noise variance */
} rssi_filter_t;

/**
 * Initialize the Kalman filter.
 *
 * @param f          Filter state.
 * @param q          Process noise variance (e.g. 0.5).
 * @param r          Measurement noise variance (e.g. 8.0).
 * @param initial    Initial RSSI estimate (dBm), e.g. -60.
 */
void rssi_filter_init(rssi_filter_t *f, float q, float r, float initial);

/**
 * Feed a new raw RSSI measurement into the filter.
 *
 * @param f     Filter state.
 * @param rssi  Raw RSSI measurement (dBm).
 * @return      Filtered RSSI estimate (dBm).
 */
float rssi_filter_update(rssi_filter_t *f, float rssi);

/**
 * Get the current filtered RSSI without feeding a new measurement.
 */
float rssi_filter_get(const rssi_filter_t *f);

#endif /* RSSI_FILTER_H */
