# CS Distance Estimator -- Robustness Architecture

How the system turns noisy, glitch-prone BLE Channel Sounding tone data
into reliable distance measurements for trilateration.

## Pipeline overview

```
Raw CS tones --> cs_estimator --> cs_adapter --> triangulation --> position
                  Layer 1-3       callback        Layer 4
```

---

## Layer 1: Tone quality filter (`cs_estimator.c`)

Tones with `quality < 2` (on the 0-3 scale from `bt_le_cs_step_data`) are
discarded before any math runs. A single corrupted tone (interference,
co-channel collision) would bias both estimators -- this prevents it from
ever entering the pipeline.

At least `CS_MIN_GOOD_TONES` (8) tones must survive filtering, otherwise
the procedure is rejected entirely.

---

## Layer 2: Per-estimator outlier suppression (`cs_estimator.c`)

### Phase slope: IRLS (iteratively reweighted least squares)

After the first weighted regression pass, residuals are computed per tone.
Tones with residual > 1.5x the MAD (median absolute deviation) get their
weight cut to 10%. This means 1-2 multipath-corrupted tones that distort
the phase-vs-frequency linearity don't drag the slope -- they're
effectively ignored on the refit.

### IFFT: Hann window + zero-padding + first-path detection

- **Hann window** reduces spectral sidelobes from ~-13 dB to ~-31 dB,
  preventing strong reflections from masking the LOS peak through leakage.
- **8x zero-padding** before IFFT improves time-domain resolution from
  ~1.87 m (raw bin width at 72 tones / 1 MHz spacing) to ~0.23 m.
- **Parabolic interpolation** around the peak gives sub-bin accuracy to
  ~0.05 m.
- **First-path detection** replaces `argmax` (strongest path). The noise
  floor is estimated from the upper-quarter bins (far field, no real signal
  expected). The first peak exceeding `4x noise_floor` (~6 dB) is selected
  as LOS. A strong reflection at a longer delay is ignored even if it has
  higher magnitude.

---

## Layer 3: Fusion + Kalman outlier gate (`cs_estimator.c`)

### Estimator fusion

Both estimators run on the same filtered tone set. Their results are
combined with a simple agreement check:

- **Agree within 0.3 m**: average them, report `sigma = 0.07 m`
  (high confidence -- both see the same path).
- **Disagree by > 0.3 m**: trust only the IFFT result with
  `sigma = 0.10 m` (IFFT is more multipath-robust).

### Mahalanobis outlier gate

The estimator maintains a 1D Kalman state (predicted distance + variance).
Before accepting a new measurement:

```
innovation = d_new - x_predicted
gate = 3.0 * sqrt(P_predicted + sigma^2)

if |innovation| > gate:
    --> outlier: report with sigma = 1.0 m (heavily discounted)
```

This prevents a single bad procedure from corrupting the smoothed state.
The measurement is still passed through (not hard-rejected), but the
elevated sigma means it gets very low weight in the trilateration solver.

---

## Layer 4: Weighted Gauss-Newton solver (`triangulation.c`)

The trilateration solver computes position by minimizing:

```
sum_i [ w_i * (||pos - antenna_i|| - d_i)^2 ]
```

where `w_i = 1 / sigma_i^2`. This means:

| Source                     | sigma   | Weight  |
|----------------------------|---------|---------|
| CS (both estimators agree) | 0.07 m  | ~204    |
| CS (single estimator)      | 0.10 m  | 100     |
| CS (outlier-gated)         | 1.00 m  | 1       |
| RSSI fallback              | 4.00 m  | ~0.063  |

### Key design principle

**Nothing is hard-rejected -- everything is soft-weighted by confidence.**

- A clean CS measurement dominates an outlier-gated one by 100-200x.
- An outlier-gated CS measurement is still 16x more trusted than RSSI
  fallback.
- RSSI fallback is always available (no CS hardware required) but
  contributes minimally when CS is running well.

This gives **smooth degradation** instead of sudden jumps: when CS
glitches, the solver gracefully falls back toward prior state and RSSI
rather than snapping to an erroneous position.

---

## Configuration (Kconfig)

| Parameter                     | Default | Description |
|-------------------------------|---------|-------------|
| `CS_PHASE_SLOPE_OFFSET_MM`   | 1910    | Calibration offset for phase slope estimator |
| `CS_IFFT_OFFSET_MM`          | 460     | Calibration offset for IFFT estimator |
| `CS_OUTLIER_GATE_SIGMA_X10`  | 30      | Innovation gate width (3.0 sigma) |
| `CS_FUSION_AGREE_MM`         | 300     | Agreement threshold for fusion (0.3 m) |

Calibration offsets should be measured on your hardware at a known
reference distance. The defaults are from BLE CS literature.

---

## Source files

- `cs_estimator.h` -- public interface and types
- `cs_estimator.c` -- estimator implementation (this module)
- `cs_adapter.c` -- BLE CS session lifecycle, calls estimator
- `triangulation.c` -- weighted Gauss-Newton solver (Layer 4)
- `simulation/cs_estimator_sim.py` -- Python validation under synthetic multipath
