#ifndef CS_ESTIMATOR_H
#define CS_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file cs_estimator.h
 * @brief BLE Channel Sounding distance estimation from raw tone data.
 *
 * Implements two estimators that are fused for robust ranging:
 *   1. Weighted phase-slope (linear regression of unwrapped phase vs frequency)
 *   2. Windowed IFFT with first-path detection
 *
 * Both estimators include tone quality filtering, calibration offset removal,
 * and an outlier gate on the output.
 */

/** Maximum number of tones per CS procedure (BLE CS uses up to 72 of 79). */
#define CS_MAX_TONES 79

/** Minimum number of good-quality tones required for an estimate. */
#define CS_MIN_GOOD_TONES 8

/** Tone quality threshold (0-3 scale from bt_le_cs step data). */
#define CS_TONE_QUALITY_MIN 2

/**
 * Single CS tone measurement.
 *
 * Constructed by the cs_adapter from initiator and reflector PCTs:
 *   phase_rad = angle(PCT_init * conj(PCT_refl))  (cancels LO offsets)
 */
typedef struct {
    float freq_hz;     /**< Tone center frequency (Hz) */
    float i_val;       /**< In-phase component of combined PCT */
    float q_val;       /**< Quadrature component of combined PCT */
    uint8_t quality;   /**< Tone quality indicator (0=bad, 3=best) */
} cs_tone_t;

/**
 * Per-procedure RTT measurement passed alongside the tone array.
 * Set rtt_valid = false when no RTT is available for this procedure.
 */
typedef struct {
    float rtt_distance_m;  /**< Round-trip-time distance from mode-0 steps */
    bool  rtt_valid;       /**< True if RTT measurement is present */
} cs_rtt_info_t;

/**
 * Optional RSSI-derived distance estimate for multi-source fusion.
 * Set rssi_valid = false when no RSSI measurement is available.
 */
typedef struct {
    float rssi_distance_m; /**< RSSI-derived distance (log-distance model) */
    float rssi_sigma_m;    /**< Uncertainty of RSSI distance estimate */
    bool  rssi_valid;      /**< True if RSSI distance is available */
} cs_rssi_info_t;

/**
 * Estimator configuration (mirrors Kconfig at runtime).
 */
typedef struct {
    int32_t phase_slope_offset_mm; /**< Calibration offset for phase slope */
    int32_t ifft_offset_mm;        /**< Calibration offset for IFFT */
    float   outlier_gate_sigma;    /**< Innovation gate width in sigmas */
    float   fusion_agree_m;        /**< Max disagreement for fusion (m) */
    float   rtt_gate_tolerance_m;  /**< Max |PBR - RTT| before RTT overrides (m) */
} cs_estimator_cfg_t;

/**
 * Persistent estimator state for Kalman-gated output.
 */
typedef struct {
    float x_pred;  /**< Predicted distance (m) */
    float p_pred;  /**< Predicted variance (m^2) */
    bool  valid;   /**< True after first estimate */
} cs_estimator_state_t;

/**
 * Initialize estimator state.
 */
void cs_estimator_init(cs_estimator_state_t *state);

/**
 * Estimate distance from a set of CS tone measurements.
 *
 * Steps:
 *   1. Filter tones by quality >= CS_TONE_QUALITY_MIN
 *   2. Run weighted phase-slope estimator (with IRLS outlier rejection)
 *   3. Run Hann-windowed IFFT with zero-padding and first-path detection
 *   4. Apply calibration offsets
 *   5. Fuse the two estimates (agreement check)
 *   5b. RTT plausibility gate (reject PBR when |PBR - RTT| > tolerance)
 *   6. Apply outlier gate against prior state
 *   7. Update internal state
 *
 * @param state   Persistent state (updated on success)
 * @param cfg     Configuration
 * @param tones   Array of tone measurements
 * @param n_tones Number of tones in the array
 * @param rtt     Optional RTT info (may be NULL)
 * @param dist_m  [out] Estimated distance (m)
 * @param sigma_m [out] Estimated uncertainty (m)
 *
 * @return 0 on success, -EINVAL if insufficient good tones
 */
int cs_estimate_distance(cs_estimator_state_t *state,
                         const cs_estimator_cfg_t *cfg,
                         const cs_tone_t *tones, int n_tones,
                         const cs_rtt_info_t *rtt,
                         const cs_rssi_info_t *rssi,
                         float *dist_m, float *sigma_m);

/**
 * Compute distance using only the phase-slope method.
 * Exported for unit testing.
 */
int cs_phase_slope_estimate(const cs_tone_t *good_tones, int n,
                            float offset_m, float *dist_m);

/**
 * Compute distance using only the windowed-IFFT method.
 * Exported for unit testing.
 */
int cs_ifft_estimate(const cs_tone_t *good_tones, int n,
                     float offset_m, float *dist_m);

#endif /* CS_ESTIMATOR_H */
