#include "ble/cs_decision.h"

#include <math.h>

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.7071067811865475f
#endif

/* -------------------------------------------------------------------------
 * Internal: Gaussian CDF approximation (Abramowitz & Stegun 7.1.26)
 * Max absolute error ~1.5e-7. Sufficient for embedded confidence checks.
 * ---------------------------------------------------------------------- */

static float erff_approx(float x)
{
    float sign = 1.0f;
    if (x < 0.0f) {
        sign = -1.0f;
        x = -x;
    }

    float t = 1.0f / (1.0f + 0.3275911f * x);
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;

    float poly = 0.254829592f * t
               - 0.284496736f * t2
               + 1.421413741f * t3
               - 1.453152027f * t4
               + 1.061405429f * t5;

    float result = 1.0f - poly * expf(-x * x);
    return sign * result;
}

static float gaussian_cdf(float x, float mean, float var)
{
    if (var < 1e-12f) {
        return (x >= mean) ? 1.0f : 0.0f;
    }
    float z = (x - mean) / sqrtf(var);
    return 0.5f * (1.0f + erff_approx(z * (float)M_SQRT1_2));
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void cs_decision_init(cs_decision_state_t *state)
{
    state->current_zone = CS_ZONE_UNKNOWN;
    state->candidate_zone = CS_ZONE_UNKNOWN;
    state->consecutive_count = 0;
}

enum cs_zone cs_decision_update(cs_decision_state_t *state,
                                const cs_decision_cfg_t *cfg,
                                float x_mean, float p_var)
{
    if (!state || !cfg) {
        return CS_ZONE_UNKNOWN;
    }

    /* Determine effective threshold with hysteresis direction */
    float thresh;
    if (state->current_zone == CS_ZONE_NEAR) {
        thresh = cfg->threshold_m + cfg->hysteresis_m;
    } else if (state->current_zone == CS_ZONE_FAR) {
        thresh = cfg->threshold_m - cfg->hysteresis_m;
    } else {
        thresh = cfg->threshold_m;
    }

    /* P(distance < threshold) */
    float p_near = gaussian_cdf(thresh, x_mean, p_var);
    float p_far = 1.0f - p_near;

    /* Determine proposed zone by confidence */
    enum cs_zone proposed;
    if (p_near >= cfg->confidence_min) {
        proposed = CS_ZONE_NEAR;
    } else if (p_far >= cfg->confidence_min) {
        proposed = CS_ZONE_FAR;
    } else {
        proposed = state->current_zone;
    }

    /* Min-consecutive counter for zone transition */
    if (proposed != state->current_zone) {
        if (proposed == state->candidate_zone) {
            state->consecutive_count++;
        } else {
            state->candidate_zone = proposed;
            state->consecutive_count = 1;
        }

        if (state->consecutive_count >= cfg->min_consecutive) {
            state->current_zone = proposed;
            state->candidate_zone = CS_ZONE_UNKNOWN;
            state->consecutive_count = 0;
        }
    } else {
        state->candidate_zone = CS_ZONE_UNKNOWN;
        state->consecutive_count = 0;
    }

    return state->current_zone;
}
