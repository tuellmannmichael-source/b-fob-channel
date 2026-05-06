#include "ble/cs_phase_unwrap.h"
#include "ble/cs_estimator.h"

#include <math.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void wrap_to_pi_inplace(float *d)
{
	while (*d > (float)M_PI) {
		*d -= 2.0f * (float)M_PI;
	}
	while (*d < -(float)M_PI) {
		*d += 2.0f * (float)M_PI;
	}
}

static float median_float_buf(float *arr, int n)
{
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

/**
 * Initial slope: median of wrapped (phi_i - phi_0) / (f_i - f_0).
 * Ignores pairs with |df| too small (same bin).
 */
static float coarse_slope_from_origin(const cs_unwrap_tone_t *t, int n,
				      float *work_slopes, int work_cap)
{
	int ns = 0;
	for (int i = 1; i < n && ns < work_cap; i++) {
		float df = t[i].freq_hz - t[0].freq_hz;
		if (fabsf(df) < 1e3f) {
			continue;
		}
		float d = t[i].phase_rad - t[0].phase_rad;
		wrap_to_pi_inplace(&d);
		work_slopes[ns++] = d / df;
	}
	if (ns < 1) {
		return 0.0f;
	}
	return median_float_buf(work_slopes, ns);
}

/** WLS slope dy/dx only (x = f, y = unwrapped phase). */
static float wls_slope(const float *x, const float *y, const float *w, int n)
{
	float sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0;

	for (int i = 0; i < n; i++) {
		sw += w[i];
		swx += w[i] * x[i];
		swy += w[i] * y[i];
		swxx += w[i] * x[i] * x[i];
		swxy += w[i] * x[i] * y[i];
	}
	float det = sw * swxx - swx * swx;
	if (fabsf(det) < 1e-20f) {
		return 0.0f;
	}
	return (sw * swxy - swx * swy) / det;
}

static void predictive_pass(const cs_unwrap_tone_t *t, int n,
			     float *uw, float slope_est)
{
	uw[0] = t[0].phase_rad;
	for (int i = 1; i < n; i++) {
		float df = t[i].freq_hz - t[i - 1].freq_hz;
		float predicted = uw[i - 1] + slope_est * df;
		float k = roundf((t[i].phase_rad - predicted) / (2.0f * (float)M_PI));
		uw[i] = t[i].phase_rad - k * (2.0f * (float)M_PI);
	}
}

int cs_phase_unwrap(const cs_unwrap_tone_t *tones, int n,
		    float *out_phi_unwrapped, float *out_slope)
{
	if (!tones || !out_phi_unwrapped || n < 2) {
		return -EINVAL;
	}

	float work_slopes[CS_MAX_TONES];
	float slope_est = coarse_slope_from_origin(tones, n, work_slopes, CS_MAX_TONES);
	if (fabsf(slope_est) < 1e-18f) {
		/* Fallback: consecutive pairwise median */
		int ns = 0;
		for (int i = 1; i < n && ns < CS_MAX_TONES; i++) {
			float df = tones[i].freq_hz - tones[i - 1].freq_hz;
			if (fabsf(df) < 1.0f) {
				continue;
			}
			float d = tones[i].phase_rad - tones[i - 1].phase_rad;
			wrap_to_pi_inplace(&d);
			work_slopes[ns++] = d / df;
		}
		if (ns < 1) {
			return -EINVAL;
		}
		slope_est = median_float_buf(work_slopes, ns);
	}

	float uw[CS_MAX_TONES];
	float w[CS_MAX_TONES];
	float x[CS_MAX_TONES];

	predictive_pass(tones, n, uw, slope_est);

	for (int i = 0; i < n; i++) {
		w[i] = (float)tones[i].quality / 3.0f;
		if (w[i] < 1e-6f) {
			w[i] = 1e-6f;
		}
		x[i] = tones[i].freq_hz;
	}
	slope_est = wls_slope(x, uw, w, n);

	predictive_pass(tones, n, uw, slope_est);

	for (int i = 0; i < n; i++) {
		out_phi_unwrapped[i] = uw[i];
	}
	if (out_slope) {
		*out_slope = slope_est;
	}
	return 0;
}
