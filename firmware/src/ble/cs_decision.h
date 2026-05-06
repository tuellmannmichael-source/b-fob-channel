#ifndef CS_DECISION_H
#define CS_DECISION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file cs_decision.h
 * @brief Confidence-based near/far zone decision with hysteresis.
 *
 * Uses the CDF of the Kalman-filtered distance state (Gaussian assumed)
 * to compute P(distance < threshold). A hysteresis band and
 * min-consecutive counter suppress flapping between zones.
 */

/** Decision zone output. */
enum cs_zone {
    CS_ZONE_UNKNOWN = 0,
    CS_ZONE_NEAR,
    CS_ZONE_FAR,
};

/** Decision configuration. */
typedef struct {
    float threshold_m;       /**< Zone boundary distance (m) */
    float hysteresis_m;      /**< Half-width of dead band around threshold */
    float confidence_min;    /**< Min CDF probability to assert zone (e.g. 0.8) */
    int   min_consecutive;   /**< Consecutive confirmations before switching */
} cs_decision_cfg_t;

/** Persistent decision state. */
typedef struct {
    enum cs_zone current_zone;
    enum cs_zone candidate_zone;
    int          consecutive_count;
} cs_decision_state_t;

/**
 * Initialize decision state.
 */
void cs_decision_init(cs_decision_state_t *state);

/**
 * Update the near/far decision given new Kalman state.
 *
 * @param state     Persistent state (updated)
 * @param cfg       Configuration
 * @param x_mean    Kalman distance mean (m)
 * @param p_var     Kalman distance variance (m^2)
 * @return Current zone after update
 */
enum cs_zone cs_decision_update(cs_decision_state_t *state,
                                const cs_decision_cfg_t *cfg,
                                float x_mean, float p_var);

#endif /* CS_DECISION_H */
