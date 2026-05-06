#ifndef CS_PHASE_UNWRAP_H
#define CS_PHASE_UNWRAP_H

#include <stdint.h>

/**
 * @file cs_phase_unwrap.h
 * @brief Gap- and quality-aware predictive phase unwrap for BLE CS tones.
 *
 * Input tones MUST be sorted by freq_hz ascending.
 * phase_rad is the principal value from atan2 (typically in (-pi, pi]).
 */

/** One tone for unwrap (mirrors cs_tone_t fields used here). */
typedef struct {
	float    freq_hz;
	float    phase_rad;
	uint8_t  quality;
} cs_unwrap_tone_t;

/**
 * Unwrap phase vs. frequency using a predictive + refit scheme.
 *
 * @param tones            Sorted by freq_hz (in-place phase_rad not modified).
 * @param n                Number of tones (>= 2).
 * @param out_phi_unwrapped Output contiguous phase (length n).
 * @param out_slope        Optional: estimated dphi/df (rad/Hz) after refit.
 *
 * @return 0 on success, -EINVAL on bad args / degenerate geometry.
 */
int cs_phase_unwrap(const cs_unwrap_tone_t *tones, int n,
		    float *out_phi_unwrapped, float *out_slope);

#endif /* CS_PHASE_UNWRAP_H */
