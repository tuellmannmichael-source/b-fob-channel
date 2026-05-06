# BLE Channel Sounding Reliability Plan (Reply Draft)

## Context

The Zephyr Channel Sounding API appears available (`zephyr/bluetooth/cs.h`,
`group__bt__le__cs`). The blocking issue is therefore no longer API discovery,
but **distance-estimation reliability** under realistic multipath conditions.

Observed behavior from field feedback aligns with this:

- **Phase-slope estimator** can glitch in strong reflection environments.
- **IFFT-based estimator** is generally more robust, but still exhibits
  intermittent outliers.

The proposed path is a robust estimation pipeline that treats Channel Sounding
samples as noisy measurements and adds confidence-driven fusion + filtering.

---

## Executive Summary (for colleague response)

1. **API availability is not the bottleneck anymore.**
   Integration work should proceed, but production quality depends on robust
   post-processing.
2. **Single-estimator strategies are brittle.**
   Prefer a hybrid estimator (`IFFT + phase`) with dynamic confidence weights.
3. **Outlier handling must be multi-layered.**
   Apply hard plausibility gates, robust short-window filtering, and Kalman
   innovation gating.
4. **Graceful degradation is mandatory.**
   Add confidence-state logic (`good/degraded/invalid`) and fallback behavior
   when CS quality drops.

---

## Recommended Processing Pipeline

```
CS raw samples
  -> feature extraction
  -> d_ifft + d_phase
  -> per-estimator confidence
  -> weighted fusion
  -> robust outlier prefilter
  -> 1D CV Kalman + innovation gating
  -> output distance + quality state
```

### 1) Feature extraction per CS sample

Compute and expose at least these metrics:

- `d_ifft_m`: delay/ToF-derived distance from IFFT domain.
- `d_phase_m`: phase-slope-derived distance.
- `snr_db`: SNR or equivalent quality indicator.
- `ifft_peak_prominence`: stronger, isolated peaks are preferred.
- `ifft_peak_width`: narrower peaks are usually better.
- optional controller quality flags from CS result metadata.

### 2) Confidence model

Do not treat all samples equally. Build confidence values in `[0,1]`:

- `conf_ifft = f(prominence, peak_width, snr)`
- `conf_phase = f(snr, temporal_consistency)`

Temporal consistency term should punish abrupt jumps relative to recent state.

### 3) Dynamic estimator fusion

Fuse both estimates each sample:

\[
 d = w_{ifft} d_{ifft} + w_{phase} d_{phase},\quad
 w_i = \frac{conf_i}{conf_{ifft}+conf_{phase}}
\]

If phase quality drops, `w_phase` naturally tends toward zero.

### 4) Hard plausibility gates

Before sending to state filtering:

- absolute bounds: e.g. `0.2 m <= d <= 30 m`
- max step gate based on physical speed and `dt`
- reject non-finite or malformed measurements

### 5) Robust prefilter

Use a small rolling robust filter before Kalman:

- median-of-5 or Hampel window
- attenuates impulsive spikes without large latency

### 6) State estimator (1D Constant Velocity Kalman)

State:

- `x = [distance, distance_rate]^T`

Recommendations:

- Adaptive measurement variance `R` based on confidence.
- Innovation gate (NIS/Mahalanobis) to reject updates inconsistent with state.
- On rejected update: keep predict-only step and increment degraded counter.

### 7) Quality states + fallback behavior

Track quality state:

- `GOOD`: confident, normal output
- `DEGRADED`: high uncertainty, widened sigma
- `INVALID`: hold-last-good or RSSI fallback

Example policy:

- enter fallback after `N_bad` consecutive low-confidence samples
- return to CS only after `N_good` consecutive stable samples

---

## Suggested Start Parameters

These are safe first-pass values to tune in logs:

- sample rate: `10-20 Hz`
- robust prefilter: median window `5`
- distance bounds: `[0.2, 30] m`
- Kalman base measurement sigma: `0.35 m` (variance `~0.1225`)
- process acceleration sigma: `1.0 m/s^2`
- NIS reject threshold: `9.0` (~3 sigma, 1D)
- quality thresholds:
  - `GOOD >= 0.7`
  - `DEGRADED = 0.4..0.7`
  - `INVALID < 0.4`
- fallback policy:
  - activate fallback after `8` bad samples
  - leave fallback after `10` good samples

---

## C-like Reference Pseudocode

```c
typedef struct {
    float d_ifft_m;
    float d_phase_m;
    float snr_db;
    float ifft_peak_prom;
    float ifft_peak_width_bins;
    bool valid;
} cs_meas_t;

typedef struct {
    float d_m;
    float conf_ifft;
    float conf_phase;
    float conf_total;
    bool outlier;
    bool degraded;
} obs_t;

obs_t process_cs(cs_meas_t m, float last_d, float dt_s) {
    obs_t o = {0};
    if (!m.valid) { o.degraded = true; return o; }

    o.conf_ifft  = conf_ifft(m.ifft_peak_prom, m.ifft_peak_width_bins, m.snr_db);
    o.conf_phase = conf_phase(m.snr_db, m.d_phase_m, last_d);

    float sum = o.conf_ifft + o.conf_phase;
    if (sum < 1e-3f) { o.degraded = true; return o; }

    float w_ifft = o.conf_ifft / sum;
    float w_phase = o.conf_phase / sum;
    float d_raw = w_ifft * m.d_ifft_m + w_phase * m.d_phase_m;

    if (d_raw < 0.2f || d_raw > 30.0f || !isfinite(d_raw)) {
        o.outlier = true;
        o.degraded = true;
        return o;
    }

    // Optional speed gate
    if (fabsf(d_raw - last_d) > (1.2f * dt_s + 0.5f)) {
        o.outlier = true;
    }

    float d_robust = robust_prefilter_push_and_get(d_raw); // median/hampel

    o.conf_total = 0.65f * o.conf_ifft + 0.35f * o.conf_phase;
    o.d_m = d_robust;
    return o;
}
```

---

## Rollout Plan

1. Add logging for all quality features and confidence values.
2. Implement hybrid fusion and robust prefilter in adapter layer.
3. Add Kalman innovation-gating + adaptive `R`.
4. Enable fallback state machine and telemetry counters.
5. Tune with replayed logs from multipath-heavy routes.

---

## Suggested Reply Text (ready to send)

"Thanks, this matches what we see as well: the Zephyr CS API is available, so
our main issue is estimator robustness, not API access. We should move to a
confidence-driven hybrid pipeline (IFFT + phase), plus robust outlier handling
(median/Hampel + Kalman innovation gating) and explicit degraded/fallback
states. That should reduce multipath-induced phase glitches and keep output
stable under difficult channels. If useful, I can share a concrete implementation
skeleton with initial thresholds for fast A/B tuning." 

