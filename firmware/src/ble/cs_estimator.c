#include "ble/cs_estimator.h"
#include "ble/cs_phase_unwrap.h"

#include <math.h>
#include <string.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** Speed of light (m/s). */
#define SPEED_OF_LIGHT 299792458.0f

/** BLE CS tone spacing (Hz). */
#define TONE_SPACING_HZ 1000000.0f

/** Zero-padding factor for IFFT (8× gives ~0.23 m sub-bin resolution). */
#define IFFT_PAD_FACTOR 8

/** Maximum IFFT size: CS_MAX_TONES * IFFT_PAD_FACTOR. */
#define IFFT_MAX_SIZE (CS_MAX_TONES * IFFT_PAD_FACTOR)

/** First-path detection threshold above noise floor (linear scale, ~6 dB). */
#define FIRST_PATH_THRESHOLD 4.0f

/** Baseline sigma when estimators agree (m) -- scaled dynamically. */
#define SIGMA_FUSED_BASE 0.07f

/** Baseline sigma for single-estimator result (m) -- scaled dynamically. */
#define SIGMA_SINGLE_BASE 0.10f

/** Frequency span (Hz) at which baseline sigma applies. */
#define SIGMA_REF_SPAN_HZ 72e6f

/** Number of good tones at which baseline sigma applies. */
#define SIGMA_REF_N_GOOD 40

/** Maximum quality value for tones (quality 3 is best). */
#define SIGMA_REF_QUALITY 3.0f

/** Elevated sigma when outlier gate fires (m). */
#define SIGMA_OUTLIER 1.0f

/** Kalman predict process noise Q for distance state (m^2). */
#define DISTANCE_KALMAN_Q 0.01f

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

/**
 * Compute an adaptive measurement sigma based on actual measurement quality.
 *
 * Factors considered:
 *   - n_good / SIGMA_REF_N_GOOD: fewer tones → larger sigma (Cramér-Rao bound)
 *   - freq_span / SIGMA_REF_SPAN_HZ: narrower band → larger sigma
 *   - avg_quality / SIGMA_REF_QUALITY: lower quality → larger sigma
 *
 * Returns base_sigma * combined_penalty, clamped to [base_sigma, 5*base_sigma].
 */
static float compute_adaptive_sigma(float base_sigma, int n_good,
                                    float freq_span_hz, float avg_quality)
{
    float tone_factor = (float)SIGMA_REF_N_GOOD / (float)(n_good > 1 ? n_good : 1);
    float span_factor = SIGMA_REF_SPAN_HZ / (freq_span_hz > 1e6f ? freq_span_hz : 1e6f);
    float qual_factor = SIGMA_REF_QUALITY / (avg_quality > 0.5f ? avg_quality : 0.5f);

    float penalty = sqrtf(tone_factor) * sqrtf(span_factor) * qual_factor;
    if (penalty < 1.0f) {
        penalty = 1.0f;
    }
    float sigma = base_sigma * penalty;
    float max_sigma = 5.0f * base_sigma;
    return sigma < max_sigma ? sigma : max_sigma;
}

static int compare_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float median_float(float *arr, int n)
{
    /* Simple insertion sort for small arrays */
    for (int i = 1; i < n; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
    if (n % 2 == 1) {
        return arr[n / 2];
    }
    return 0.5f * (arr[n / 2 - 1] + arr[n / 2]);
}

/* -------------------------------------------------------------------------
 * Missing-channel interpolation
 *
 * BLE CS uses a 1 MHz frequency grid. When some channels are excluded
 * (interference, regulatory), the IFFT suffers from spectral leakage due
 * to gaps. This function fills missing grid positions with unit-circle
 * interpolated values (linear phase interpolation between neighbors).
 * ---------------------------------------------------------------------- */

/**
 * Interpolate missing tones on the 1 MHz grid between known tones.
 *
 * Input tones MUST be sorted by freq_hz (ascending).
 * Fills output buffer with known tones + interpolated ones.
 * Returns total number of tones in output (may be > n_in).
 * Output buffer must be at least CS_MAX_TONES elements.
 */
static int interpolate_missing_tones(const cs_tone_t *sorted_in, int n_in,
                                     cs_tone_t *out)
{
    if (n_in < 2) {
        for (int i = 0; i < n_in; i++) {
            out[i] = sorted_in[i];
        }
        return n_in;
    }

    int n_out = 0;
    out[n_out++] = sorted_in[0];

    for (int i = 1; i < n_in && n_out < CS_MAX_TONES; i++) {
        float f_prev = sorted_in[i - 1].freq_hz;
        float f_curr = sorted_in[i].freq_hz;
        float gap = f_curr - f_prev;
        int n_missing = (int)(gap / TONE_SPACING_HZ + 0.5f) - 1;

        if (n_missing > 0 && n_missing < 20) {
            float phase_prev = atan2f(sorted_in[i - 1].q_val,
                                      sorted_in[i - 1].i_val);
            float phase_curr = atan2f(sorted_in[i].q_val,
                                      sorted_in[i].i_val);

            float d_phase = phase_curr - phase_prev;
            while (d_phase > M_PI)  d_phase -= 2.0f * M_PI;
            while (d_phase < -M_PI) d_phase += 2.0f * M_PI;

            for (int m = 1; m <= n_missing && n_out < CS_MAX_TONES; m++) {
                float t = (float)m / (float)(n_missing + 1);
                float interp_phase = phase_prev + t * d_phase;
                out[n_out].freq_hz = f_prev + (float)m * TONE_SPACING_HZ;
                out[n_out].i_val = cosf(interp_phase);
                out[n_out].q_val = sinf(interp_phase);
                out[n_out].quality = 1;
                n_out++;
            }
        }

        if (n_out < CS_MAX_TONES) {
            out[n_out++] = sorted_in[i];
        }
    }
    return n_out;
}

/* -------------------------------------------------------------------------
 * Phase slope estimator (weighted linear regression with IRLS)
 *
 * Model: phase(f) = slope * f + intercept
 * Distance = |slope| * c / (4 * pi)    (round-trip → factor of 4pi)
 *
 * Two passes:
 *   Pass 1: weighted least-squares using quality as weight
 *   Pass 2: reweight using residuals (IRLS) to reject multipath tones
 * ---------------------------------------------------------------------- */

/**
 * Legacy sequential unwrap (fallback if robust unwrap fails).
 */
static void unwrap_phases_naive(float *phases, int n)
{
    for (int i = 1; i < n; i++) {
        float diff = phases[i] - phases[i - 1];
        while (diff > M_PI) {
            diff -= 2.0f * M_PI;
        }
        while (diff < -M_PI) {
            diff += 2.0f * M_PI;
        }
        phases[i] = phases[i - 1] + diff;
    }
}

/** Gap-/quality-aware unwrap; falls back to naive on error. */
static void unwrap_phases_robust(const float *freqs, float *phases,
                                  const uint8_t *qualities, int n)
{
    cs_unwrap_tone_t ut[CS_MAX_TONES];

    for (int i = 0; i < n; i++) {
        ut[i].freq_hz = freqs[i];
        ut[i].phase_rad = phases[i];
        ut[i].quality = qualities[i];
    }
    if (cs_phase_unwrap(ut, n, phases, NULL) != 0) {
        unwrap_phases_naive(phases, n);
    }
}

/**
 * Weighted linear regression: y = slope * x + intercept.
 * Returns slope.
 */
static float weighted_linear_regression(const float *x, const float *y,
                                        const float *w, int n,
                                        float *residuals)
{
    float sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0;

    for (int i = 0; i < n; i++) {
        sw   += w[i];
        swx  += w[i] * x[i];
        swy  += w[i] * y[i];
        swxx += w[i] * x[i] * x[i];
        swxy += w[i] * x[i] * y[i];
    }

    float det = sw * swxx - swx * swx;
    if (fabsf(det) < 1e-20f) {
        return 0.0f;
    }

    float slope     = (sw * swxy - swx * swy) / det;
    float intercept = (swy - slope * swx) / sw;

    if (residuals) {
        for (int i = 0; i < n; i++) {
            residuals[i] = y[i] - (slope * x[i] + intercept);
        }
    }

    return slope;
}

int cs_phase_slope_estimate(const cs_tone_t *good_tones, int n,
                            float offset_m, float *dist_m)
{
    if (n < CS_MIN_GOOD_TONES) {
        return -EINVAL;
    }

    float freqs[CS_MAX_TONES];
    float phases[CS_MAX_TONES];
    float weights[CS_MAX_TONES];
    uint8_t qualities[CS_MAX_TONES];
    float residuals[CS_MAX_TONES];

    /* Extract phases from I/Q and sort by frequency */
    for (int i = 0; i < n; i++) {
        freqs[i]     = good_tones[i].freq_hz;
        phases[i]    = atan2f(good_tones[i].q_val, good_tones[i].i_val);
        weights[i]   = (float)good_tones[i].quality / 3.0f;
        qualities[i] = good_tones[i].quality;
    }

    /* Sort by frequency (insertion sort, small N) */
    for (int i = 1; i < n; i++) {
        float kf = freqs[i], kp = phases[i], kw = weights[i];
        uint8_t kq = qualities[i];
        int j = i - 1;
        while (j >= 0 && freqs[j] > kf) {
            freqs[j + 1]     = freqs[j];
            phases[j + 1]    = phases[j];
            weights[j + 1]   = weights[j];
            qualities[j + 1] = qualities[j];
            j--;
        }
        freqs[j + 1]     = kf;
        phases[j + 1]    = kp;
        weights[j + 1]   = kw;
        qualities[j + 1] = kq;
    }

    /* Unwrap phases (gap-/quality-aware; falls back to sequential) */
    unwrap_phases_robust(freqs, phases, qualities, n);

    /* Pass 1: weighted linear regression */
    float slope = weighted_linear_regression(freqs, phases, weights, n,
                                             residuals);

    /* Compute MAD (median absolute deviation) of residuals */
    float abs_res[CS_MAX_TONES];
    for (int i = 0; i < n; i++) {
        abs_res[i] = fabsf(residuals[i]);
    }
    float mad = median_float(abs_res, n);
    if (mad < 1e-6f) {
        mad = 1e-6f;
    }

    /* Pass 2: IRLS -- down-weight tones with residual > 1.5 * MAD */
    for (int i = 0; i < n; i++) {
        if (fabsf(residuals[i]) > 1.5f * mad) {
            weights[i] *= 0.1f;  /* Heavily suppress outlier tones */
        }
    }
    slope = weighted_linear_regression(freqs, phases, weights, n, NULL);

    /* Distance = |slope| * c / (4 * pi)
     * The factor 4*pi accounts for round-trip (2×) and phase-to-rad (2pi). */
    float d = fabsf(slope) * SPEED_OF_LIGHT / (4.0f * M_PI);

    /* Apply calibration offset */
    d -= offset_m;
    if (d < 0.05f) {
        d = 0.05f;
    }

    *dist_m = d;
    return 0;
}

/* -------------------------------------------------------------------------
 * IFFT-based estimator
 *
 * 1. Construct CTF H[k] from tone I/Q data
 * 2. Apply Hann window
 * 3. Zero-pad to IFFT_PAD_FACTOR × N
 * 4. Compute IFFT (DFT, since we run on MCU with small N)
 * 5. Magnitude profile → first-path detection
 * 6. Parabolic interpolation for sub-bin accuracy
 * 7. Convert bin index to distance
 * ---------------------------------------------------------------------- */

/**
 * Simple DFT-based IFFT for small arrays (N <= IFFT_MAX_SIZE).
 * We use a direct O(N*M) computation since M (number of tones) is small
 * and the DFT is over the zero-padded array.
 *
 * H_padded[k] is non-zero only for k=0..n_tones-1 (with Hann window).
 * So h[n] = sum_{k=0}^{n_tones-1} H_win[k] * exp(j*2*pi*k*n/M) / M
 */
static void compute_ifft_magnitude(const float *h_re, const float *h_im,
                                   int n_tones, int m_size,
                                   float *mag_out)
{
    float inv_m = 1.0f / (float)m_size;

    for (int n = 0; n < m_size; n++) {
        float re = 0.0f, im = 0.0f;
        for (int k = 0; k < n_tones; k++) {
            float angle = 2.0f * M_PI * (float)k * (float)n * inv_m;
            float cos_a = cosf(angle);
            float sin_a = sinf(angle);
            re += h_re[k] * cos_a - h_im[k] * sin_a;
            im += h_re[k] * sin_a + h_im[k] * cos_a;
        }
        re *= inv_m;
        im *= inv_m;
        mag_out[n] = sqrtf(re * re + im * im);
    }
}

/**
 * Find the first peak above a noise-floor threshold.
 * Returns bin index, or -1 if no peak found.
 */
static int find_first_path(const float *mag, int m_size)
{
    /* Estimate noise floor from the upper quarter of bins
     * (far field, unlikely to contain real signal). */
    int noise_start = (3 * m_size) / 4;
    float noise_sum = 0.0f;
    int noise_count = m_size - noise_start;

    for (int i = noise_start; i < m_size; i++) {
        noise_sum += mag[i];
    }
    float noise_floor = noise_sum / (float)noise_count;
    float threshold = noise_floor * FIRST_PATH_THRESHOLD;

    /* Scan from bin 1 upward (skip DC bin 0) for first peak above threshold */
    for (int i = 1; i < m_size / 2; i++) {
        if (mag[i] > threshold &&
            mag[i] >= mag[i - 1] &&
            (i + 1 >= m_size || mag[i] >= mag[i + 1])) {
            return i;
        }
    }

    return -1;
}

/**
 * Parabolic interpolation around a peak for sub-bin accuracy.
 */
static float parabolic_interpolation(const float *mag, int peak, int m_size)
{
    if (peak <= 0 || peak >= m_size - 1) {
        return (float)peak;
    }

    float alpha = mag[peak - 1];
    float beta  = mag[peak];
    float gamma = mag[peak + 1];
    float denom = alpha - 2.0f * beta + gamma;

    if (fabsf(denom) < 1e-10f) {
        return (float)peak;
    }

    return (float)peak + 0.5f * (alpha - gamma) / denom;
}

int cs_ifft_estimate(const cs_tone_t *good_tones, int n,
                     float offset_m, float *dist_m)
{
    if (n < CS_MIN_GOOD_TONES) {
        return -EINVAL;
    }

    /* Sort input tones by frequency */
    cs_tone_t sorted[CS_MAX_TONES];
    for (int i = 0; i < n; i++) {
        sorted[i] = good_tones[i];
    }
    for (int i = 1; i < n; i++) {
        cs_tone_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j].freq_hz > key.freq_hz) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    /* Interpolate missing channels on 1 MHz grid */
    cs_tone_t interp[CS_MAX_TONES];
    int n_interp = interpolate_missing_tones(sorted, n, interp);

    int m_size = n_interp * IFFT_PAD_FACTOR;
    if (m_size > IFFT_MAX_SIZE) {
        m_size = IFFT_MAX_SIZE;
    }

    /* Build CTF: normalize to unit circle (phase-only), then Hann window.
     * Amplitude normalization removes frequency-dependent gain variations
     * from antenna/board/channel and focuses on phase-vs-distance. */
    float h_re[CS_MAX_TONES];
    float h_im[CS_MAX_TONES];

    for (int k = 0; k < n_interp; k++) {
        float i_raw = interp[k].i_val;
        float q_raw = interp[k].q_val;
        float mag_sq = i_raw * i_raw + q_raw * q_raw;

        if (mag_sq > 1e-20f) {
            float inv_mag = 1.0f / sqrtf(mag_sq);
            i_raw *= inv_mag;
            q_raw *= inv_mag;
        }

        float hann = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)k / (float)(n_interp - 1)));
        h_re[k] = i_raw * hann;
        h_im[k] = q_raw * hann;
    }

    /* Compute IFFT magnitude profile */
    static float mag[IFFT_MAX_SIZE];
    compute_ifft_magnitude(h_re, h_im, n_interp, m_size, mag);

    /* First-path detection */
    int peak_bin = find_first_path(mag, m_size);
    if (peak_bin < 0) {
        return -EINVAL;
    }

    /* Parabolic interpolation for sub-bin accuracy */
    float frac_bin = parabolic_interpolation(mag, peak_bin, m_size);

    /* Convert bin to distance:
     * d = frac_bin * c / (2 * M * delta_f)
     * Factor of 2 for round-trip. */
    float d = frac_bin * SPEED_OF_LIGHT / (2.0f * (float)m_size * TONE_SPACING_HZ);

    /* Apply calibration offset */
    d -= offset_m;
    if (d < 0.05f) {
        d = 0.05f;
    }

    *dist_m = d;
    return 0;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void cs_estimator_init(cs_estimator_state_t *state)
{
    state->x_pred = 0.0f;
    state->p_pred = 100.0f;  /* Large initial uncertainty */
    state->valid = false;
}

int cs_estimate_distance(cs_estimator_state_t *state,
                         const cs_estimator_cfg_t *cfg,
                         const cs_tone_t *tones, int n_tones,
                         const cs_rtt_info_t *rtt,
                         const cs_rssi_info_t *rssi,
                         float *dist_m, float *sigma_m)
{
    if (!state || !cfg || !tones || !dist_m || !sigma_m) {
        return -EINVAL;
    }

    /* Step 1: Filter tones by quality */
    cs_tone_t good[CS_MAX_TONES];
    int n_good = 0;
    for (int i = 0; i < n_tones && i < CS_MAX_TONES; i++) {
        if (tones[i].quality >= CS_TONE_QUALITY_MIN) {
            good[n_good++] = tones[i];
        }
    }

    if (n_good < CS_MIN_GOOD_TONES) {
        return -EINVAL;
    }

    /* Compute measurement quality metrics for adaptive sigma */
    float freq_min = good[0].freq_hz;
    float freq_max = good[0].freq_hz;
    float quality_sum = 0.0f;
    for (int i = 0; i < n_good; i++) {
        if (good[i].freq_hz < freq_min) freq_min = good[i].freq_hz;
        if (good[i].freq_hz > freq_max) freq_max = good[i].freq_hz;
        quality_sum += (float)good[i].quality;
    }
    float freq_span_hz = freq_max - freq_min;
    float avg_quality = quality_sum / (float)n_good;

    /* Step 2: Run both estimators */
    float phase_slope_offset_m = cfg->phase_slope_offset_mm / 1000.0f;
    float ifft_offset_m = cfg->ifft_offset_mm / 1000.0f;

    float d_phase = 0.0f, d_ifft = 0.0f;
    int phase_ok = cs_phase_slope_estimate(good, n_good, phase_slope_offset_m,
                                           &d_phase);
    int ifft_ok = cs_ifft_estimate(good, n_good, ifft_offset_m, &d_ifft);

    /* Step 3: Fuse results with adaptive sigma */
    float d_fused;
    float sigma;

    if (phase_ok == 0 && ifft_ok == 0) {
        float disagree = fabsf(d_phase - d_ifft);
        if (disagree <= cfg->fusion_agree_m) {
            d_fused = 0.5f * (d_phase + d_ifft);
            sigma = compute_adaptive_sigma(SIGMA_FUSED_BASE, n_good,
                                           freq_span_hz, avg_quality);
        } else {
            d_fused = d_ifft;
            sigma = compute_adaptive_sigma(SIGMA_SINGLE_BASE, n_good,
                                           freq_span_hz, avg_quality);
        }
    } else if (ifft_ok == 0) {
        d_fused = d_ifft;
        sigma = compute_adaptive_sigma(SIGMA_SINGLE_BASE, n_good,
                                       freq_span_hz, avg_quality);
    } else if (phase_ok == 0) {
        d_fused = d_phase;
        sigma = compute_adaptive_sigma(SIGMA_SINGLE_BASE, n_good,
                                       freq_span_hz, avg_quality);
    } else {
        return -EINVAL;
    }

    /* Step 3b: RTT plausibility gate.
     * If RTT is available and disagrees with PBR by more than the configured
     * tolerance, fall back to RTT (lower precision but immune to multipath). */
    if (rtt && rtt->rtt_valid && cfg->rtt_gate_tolerance_m > 0.0f) {
        float pbr_rtt_diff = fabsf(d_fused - rtt->rtt_distance_m);
        if (pbr_rtt_diff > cfg->rtt_gate_tolerance_m) {
            d_fused = rtt->rtt_distance_m;
            sigma = compute_adaptive_sigma(SIGMA_SINGLE_BASE * 2.0f, n_good,
                                           freq_span_hz, avg_quality);
        }
    }

    /* Step 3c: Multi-source fusion via weighted median.
     * If RSSI-derived distance is available, combine PBR + RSSI using
     * inverse-variance weighting through a weighted median selector. */
    if (rssi && rssi->rssi_valid && rssi->rssi_sigma_m > 0.0f) {
        float w_pbr = 1.0f / (sigma * sigma);
        float w_rssi = 1.0f / (rssi->rssi_sigma_m * rssi->rssi_sigma_m);
        float w_total = w_pbr + w_rssi;

        float pbr_weight_frac = w_pbr / w_total;

        if (pbr_weight_frac > 0.3f && pbr_weight_frac < 0.7f) {
            d_fused = (w_pbr * d_fused + w_rssi * rssi->rssi_distance_m) / w_total;
            sigma = 1.0f / sqrtf(w_total);
        } else if (pbr_weight_frac <= 0.3f) {
            d_fused = rssi->rssi_distance_m;
            sigma = rssi->rssi_sigma_m;
        }
    }

    /* Step 4: Outlier gate (Mahalanobis) against prior state */
    if (state->valid) {
        float innovation = d_fused - state->x_pred;
        float gate = cfg->outlier_gate_sigma *
                     sqrtf(state->p_pred + sigma * sigma);
        if (fabsf(innovation) > gate) {
            /* Outlier detected: still report but with elevated sigma */
            sigma = SIGMA_OUTLIER;
        }
    }

    /* Step 5: Simple Kalman update on the distance state */
    if (!state->valid) {
        state->x_pred = d_fused;
        state->p_pred = sigma * sigma;
        state->valid = true;
    } else {
        /* Predict */
        state->p_pred += DISTANCE_KALMAN_Q;

        /* Update */
        float r = sigma * sigma;
        float k = state->p_pred / (state->p_pred + r);
        state->x_pred += k * (d_fused - state->x_pred);
        state->p_pred = (1.0f - k) * state->p_pred;
    }

    *dist_m = state->x_pred;
    *sigma_m = sqrtf(state->p_pred);
    return 0;
}
