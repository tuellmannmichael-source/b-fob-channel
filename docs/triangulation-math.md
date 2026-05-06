# Position Estimation Math

This document covers the math for determining keyfob **proximity** (how far)
and **direction** (which direction) using sequential RSSI measurements from
three switched antennas on a single nRF54L15 receiver. Full trilateration math
is retained as an optional upgrade path.

---

## Part 1: RSSI to Distance

### Log-Distance Path-Loss Model

```
RSSI(d) = RSSI(d0) - 10 * n * log10(d / d0)
```

Where:
- `RSSI(d)` = measured RSSI at distance `d` (dBm)
- `RSSI(d0)` = RSSI at reference distance `d0` (typically 1 meter)
- `n` = path-loss exponent (environment-dependent)
- `d0` = reference distance (1 m)

### Solving for distance

```
d = d0 * 10 ^ ((RSSI(d0) - RSSI(d)) / (10 * n))
```

### Path-loss exponent `n` for roof-mounted antennas

| Scenario | Typical n |
|----------|-----------|
| Free space (outdoor, line-of-sight) | 2.0 |
| Roof antenna to keyfob outside vehicle | 2.0 - 2.5 |
| Roof antenna to keyfob inside vehicle | 3.0 - 4.0 |
| Body blocking (keyfob in pocket) | 3.5 - 5.0 |

**Note on roof mounting:** Roof antennas radiate outward/downward. A keyfob
at ground level 3 m from the vehicle experiences roughly free-space path loss
plus vehicle body diffraction. Expect `n = 2.0-2.8` for outdoor approach.

### Cable loss correction

All three antenna paths have ~2.5 dB cable/switch loss. Since the losses are
symmetric, they're absorbed into the `RSSI(d0)` calibration. If cable lengths
differ, add a per-antenna offset:

```
RSSI_corrected = RSSI_raw + cable_loss_offset[antenna_id]
```

### Calibration procedure

1. Place keyfob at exactly 1 m from each antenna. Record 50+ RSSI samples per
   antenna. Average = `RSSI(d0)` per antenna.
2. Repeat at 0.5 m, 2 m, 3 m, 5 m, 10 m.
3. For each antenna, fit `n` using least-squares:
   ```
   minimize sum_i (RSSI_measured_i - (RSSI(d0) - 10*n*log10(d_i)))^2
   ```
4. Store per-antenna `RSSI(d0)` and `n` in config.

---

## Part 2: RSSI Filtering (Kalman Filter)

One 1D Kalman filter instance per antenna.

### Model

```
State:       x_k = x_{k-1} + w,   w ~ N(0, Q)
Measurement: z_k = x_k + v,       v ~ N(0, R)
```

### Update equations

```
Predict:
    x_pred = x_prev
    P_pred = P_prev + Q

Update:
    K = P_pred / (P_pred + R)           # Kalman gain
    x_new = x_pred + K * (z - x_pred)
    P_new = (1 - K) * P_pred
```

### Recommended starting values

| Parameter | Value | Notes |
|-----------|-------|-------|
| Q (process noise) | 0.5 dBm^2 | Keyfob moves at walking speed |
| R (measurement noise) | 8.0 - 16.0 dBm^2 | 3-4 dBm std dev typical |

### Sequential antenna switching note

With 3 antennas and a 100 ms connection interval, each antenna gets an update
every 300 ms. The Kalman filter for antenna B doesn't get a measurement during
antenna A's and C's slots. This is fine -- during the predict step, `P`
increases by `Q`, correctly reflecting growing uncertainty when no measurement
is available.

---

## Part 3: Direction Estimation

### Method: Strongest-antenna with RSSI ratio interpolation

The simplest and most robust approach for 3 roof-mounted antennas.

**Step 1: Identify the strongest antenna**

```
strongest = argmax(rssi_filtered[0], rssi_filtered[1], rssi_filtered[2])
```

This gives a coarse 120-degree sector (front, left-rear, right-rear).

**Step 2: Refine using RSSI ratios**

Compute the bearing angle by interpolating between antenna directions:

```
# Antenna directions in vehicle coordinates (radians from forward)
#   A =   0 deg (front)
#   B = 210 deg or -150 deg (left rear)
#   C = 150 deg (right rear)

# Convert filtered RSSI to linear power (for weighting)
p_i = 10 ^ (rssi_filtered[i] / 10)   for i in {A, B, C}

# Weighted circular mean
direction = atan2(sum(p_i * sin(theta_i)), sum(p_i * cos(theta_i)))
```

Where `theta_i` is the angular position of antenna `i` relative to vehicle
center.

**Why this works:** The antenna closest to the keyfob receives the strongest
signal. By weighting each antenna's known direction by its received power, we
get a smooth interpolated bearing.

**Accuracy:** With a well-spread triangle of antennas, expect +/-30-45 degree
angular accuracy with RSSI alone. This is enough to distinguish front,
left-rear, right-rear, and the transitions between them.

### Direction sectors

For a simpler output, map the bearing to discrete sectors:

```
Sector assignment (6 sectors, 60 deg each):

            FRONT
         330   30
    F-LEFT \  / F-RIGHT
            \/
  270 LEFT --+-- RIGHT 90
            /\
   R-LEFT /  \ R-RIGHT
         210  150
            REAR
```

---

## Part 4: Proximity Estimation

### Method: Weighted average distance

```
# Convert each antenna's filtered RSSI to distance
d_i = 10 ^ ((RSSI_ref_i - rssi_filtered[i]) / (10 * n_i))

# The minimum distance is the best single-antenna estimate
d_min = min(d_A, d_B, d_C)

# Or use a weighted average (weights = linear RSSI power)
p_i = 10 ^ (rssi_filtered[i] / 10)
d_weighted = sum(p_i * d_i) / sum(p_i)
```

For proximity, `d_min` is usually the most useful: if any antenna sees the
keyfob close, the keyfob is close.

### Proximity bands

| Band | Distance | Typical RSSI (n=2.5) |
|------|----------|---------------------|
| IMMEDIATE | < 0.5 m | > -52 dBm |
| NEAR | 0.5 - 2 m | -52 to -67 dBm |
| MEDIUM | 2 - 5 m | -67 to -79 dBm |
| FAR | 5 - 15 m | -79 to -92 dBm |
| OUT_OF_RANGE | > 15 m | < -92 dBm |

These thresholds depend heavily on TX power, path-loss exponent, and
environment. Calibrate empirically.

---

## Part 5: Full 2D Trilateration (Optional Upgrade)

Use when higher precision is needed.

### Important: sequential RSSI and time skew

With an RF-switch architecture, the three RSSI readings are not simultaneous.
Antenna A's RSSI is 200 ms older than antenna C's. For a walking keyfob
(~1.5 m/s), this is ~30 cm of movement. Two mitigations:

1. **Use Kalman filter state** -- the filtered RSSI estimate is a prediction
   of the current value, not a stale measurement. The filter's predict step
   accounts for the time gap.
2. **Timestamp compensation** -- weight more recent measurements higher, or
   extrapolate older measurements forward using the Kalman velocity estimate
   (requires a 2-state Kalman: position + velocity).

### Algebraic solution (3 antennas)

Three distance circles:
```
(x - x_a)^2 + (y - y_a)^2 = d_a^2
(x - x_b)^2 + (y - y_b)^2 = d_b^2
(x - x_c)^2 + (y - y_c)^2 = d_c^2
```

Subtract pairs to get a 2x2 linear system, solve with Cramer's rule.

### Weighted Gauss-Newton least-squares (Channel Sounding-ready)

Minimize: `sum_i [ w_i * (||pos - antenna_i|| - d_i)^2 ]` where
`w_i = 1 / sigma_i^2`.

- Channel Sounding distances should use smaller `sigma_i` (higher weight).
- RSSI fallback distances use larger `sigma_i` (lower weight).

```
For each iteration:
    r_i = ||pos - antenna_i|| - d_i                    (residual)
    J_i = (pos - antenna_i) / ||pos - antenna_i||     (Jacobian row)
    w_i = 1 / sigma_i^2

    delta = -(J^T * W * J)^{-1} * J^T * W * r
    pos += delta

    if |delta| < epsilon: converged
```

Converges in 3-8 iterations from centroid initial guess.

---

## Part 6: Angle of Arrival (Future -- CTE)

BLE 5.1+ Constant Tone Extension enables phase-based AoA. The SKYA21039 RF
switch cycles antennas during the CTE tone and the radio samples IQ data.

### Phase difference to angle

```
theta = arcsin(phi * lambda / (2 * pi * d_ant))
```

Where:
- `phi` = phase difference between antennas (radians)
- `d_ant` = antenna spacing (typically lambda/2 = 6.24 cm at 2.4 GHz)
- `lambda` = 12.5 cm at 2.4 GHz

With the SKYA21039's < 0.5 us switching speed, it's compatible with CTE
sample intervals (1-2 us). However, roof-mounted antennas spaced ~1.5 m apart
(not lambda/2) work as a sparse array -- the math changes to account for
spatial aliasing.

---

## Part 7: Practical Accuracy Expectations

| Method | Angular accuracy | Distance accuracy |
|--------|-----------------|-------------------|
| RSSI ratio direction | +/-30-45 deg | N/A |
| RSSI proximity | N/A | +/-50% of true distance |
| RSSI proximity + direction | +/-30-45 deg | +/-50% |
| Full trilateration (RSSI) | N/A | 1-3 m absolute |
| CTE AoA (future) | +/-5-10 deg | N/A |

For the use case of "keyfob approaching from the left, ~3 m away", RSSI-based
proximity + direction is sufficient.

---

## Part 8: Scenario Comparison (Simulation Update)

The file `simulation/triangulation_sim.py` now compares three operating modes:

1. **RSSI-only**: Distance estimation exclusively from filtered RSSI.
2. **CS-only**: Distance estimation from Channel Sounding measurements with a
   smaller, optionally **non-Gaussian** noise model.
3. **Hybrid (CS + RSSI fallback)**: Primarily CS, falling back to RSSI on CS
   dropout.

### CS Distance Noise Model

For CS, a small base noise and an optional heavy-tail component are modeled:

- Base: `N(0, sigma_cs)` with `sigma_cs = 0.10 m`
- Optional tail: with probability `p_tail = 0.15`, an additional Laplace error
  `Laplace(0, b=0.15 m)`

This keeps CS more precise than RSSI on average while still exhibiting
occasional non-Gaussian outliers (e.g. multipath residuals).

### Comparison Metrics

The simulator aggregates per scenario:

- **Median error (m)**
- **P90 error (m)**
- **Time-to-first-fix (TTFF)** in seconds
- **Fix rate** (fraction of time steps with a valid 2D fix)
- **Dropout robustness**: fraction of valid fixes during steps with at least one
  measurement dropout

### Example Results (Seed=42, Grid-Step=0.5 m, 60 Steps/Point)

| Scenario | Median Error | P90 Error | TTFF | Fix Rate | Dropout Robustness |
|----------|--------------|-----------|------|----------|--------------------|
| RSSI-only | 0.31 m | 0.99 m | 0.17 s | 55.3 % | 44.9 % |
| CS-only | 0.13 m | 0.31 m | 0.43 s | 33.0 % | 18.1 % |
| Hybrid (CS+RSSI) | 0.16 m | 0.49 m | 0.03 s | 84.0 % | 80.2 % |

### Interpretation

- **CS-only** delivers the best pure distance accuracy (median/P90) but is less
  robust on its own due to the higher CS dropout rate.
- **RSSI-only** is more consistently available than CS-only but significantly
  less accurate.
- **Hybrid** achieves the best system trade-off: substantially better
  availability and dropout robustness while maintaining good accuracy.

For production systems, Hybrid is therefore the most practical default strategy,
especially in environments with intermittent CS dropouts.
