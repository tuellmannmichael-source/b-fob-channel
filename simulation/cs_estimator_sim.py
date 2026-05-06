"""BLE Channel Sounding distance estimator simulation.

Compares four estimator configurations under synthetic multipath:

  1. Raw phase slope (no improvements)
  2. Raw IFFT (no windowing, strongest-path detection)
  3. Improved IFFT (Hann window + zero-padding + first-path detection)
  4. Fused (improved IFFT + weighted phase slope with IRLS + outlier gate)

Generates CDF plots of distance estimation error for each.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# BLE CS physical parameters
# ---------------------------------------------------------------------------

SPEED_OF_LIGHT = 299_792_458.0   # m/s
TONE_SPACING_HZ = 1_000_000.0   # 1 MHz spacing between CS tones
NUM_TONES = 72                   # Typical usable tones in BLE CS (of 79 max)
BASE_FREQ_HZ = 2_402_000_000.0  # 2402 MHz start

# Tone frequencies
TONE_FREQS = np.array([BASE_FREQ_HZ + i * TONE_SPACING_HZ for i in range(NUM_TONES)])

# ---------------------------------------------------------------------------
# Multipath channel model
# ---------------------------------------------------------------------------


def generate_channel(true_distance_m: float,
                     n_reflections: int = 3,
                     reflection_range_m: tuple = (1.0, 5.0),
                     reflection_amplitude_range: tuple = (0.2, 0.8),
                     snr_db: float = 20.0) -> dict:
    """Generate a multipath channel with a direct path and reflections.

    Returns dict with:
        - 'delays': array of path delays (meters, converted from time)
        - 'amplitudes': array of path amplitudes (complex)
        - 'snr_db': added noise SNR
    """
    rng = np.random.default_rng()

    # Direct path (LOS)
    delays = [true_distance_m]
    amplitudes = [1.0 + 0j]

    # Reflected paths (always longer than LOS)
    for _ in range(n_reflections):
        extra = rng.uniform(*reflection_range_m)
        amp = rng.uniform(*reflection_amplitude_range)
        phase = rng.uniform(0, 2 * np.pi)
        delays.append(true_distance_m + extra)
        amplitudes.append(amp * np.exp(1j * phase))

    return {
        'delays': np.array(delays),
        'amplitudes': np.array(amplitudes),
        'snr_db': snr_db,
    }


def channel_to_tones(channel: dict, freqs: np.ndarray) -> tuple:
    """Convert a multipath channel to CS tone I/Q data.

    The channel transfer function at frequency f is:
        H(f) = sum_k  a_k * exp(-j * 2*pi*f * tau_k)
    where tau_k = 2 * d_k / c  (round-trip delay).

    Returns (i_vals, q_vals, qualities) arrays.
    """
    rng = np.random.default_rng()
    delays_s = 2.0 * channel['delays'] / SPEED_OF_LIGHT  # round-trip
    amps = channel['amplitudes']

    h = np.zeros(len(freqs), dtype=complex)
    for tau, a in zip(delays_s, amps):
        h += a * np.exp(-1j * 2 * np.pi * freqs * tau)

    # Add noise
    signal_power = np.mean(np.abs(h) ** 2)
    noise_power = signal_power / (10 ** (channel['snr_db'] / 10))
    noise = (rng.normal(0, np.sqrt(noise_power / 2), len(freqs)) +
             1j * rng.normal(0, np.sqrt(noise_power / 2), len(freqs)))
    h_noisy = h + noise

    # Generate quality indicators (lower quality for tones with high noise)
    tone_snr = np.abs(h) ** 2 / noise_power
    qualities = np.clip((tone_snr / 5).astype(int), 0, 3).astype(np.uint8)

    return h_noisy.real, h_noisy.imag, qualities


# ---------------------------------------------------------------------------
# Estimator: raw phase slope (no improvements)
# ---------------------------------------------------------------------------

def estimate_phase_slope_raw(i_vals, q_vals, freqs):
    """Basic phase slope: unwrap, linear regression, convert to distance."""
    phases = np.arctan2(q_vals, i_vals)

    # Unwrap
    phases = np.unwrap(phases)

    # Linear regression (unweighted)
    n = len(freqs)
    sx = np.sum(freqs)
    sy = np.sum(phases)
    sxx = np.sum(freqs ** 2)
    sxy = np.sum(freqs * phases)
    det = n * sxx - sx * sx
    if abs(det) < 1e-30:
        return np.nan
    slope = (n * sxy - sx * sy) / det

    return abs(slope) * SPEED_OF_LIGHT / (4 * np.pi)


# ---------------------------------------------------------------------------
# Estimator: raw IFFT (no windowing, strongest-path)
# ---------------------------------------------------------------------------

def estimate_ifft_raw(i_vals, q_vals, freqs):
    """Basic IFFT: no window, no zero-padding, argmax peak."""
    h = i_vals + 1j * q_vals
    cir = np.fft.ifft(h)
    mag = np.abs(cir)

    peak = np.argmax(mag[:len(mag) // 2])
    if peak == 0:
        return np.nan

    # Distance: bin * c / (2 * N * delta_f)
    n = len(freqs)
    return peak * SPEED_OF_LIGHT / (2 * n * TONE_SPACING_HZ)


# ---------------------------------------------------------------------------
# Estimator: improved IFFT (Hann + zero-pad + first-path + parabolic)
# ---------------------------------------------------------------------------

def estimate_ifft_improved(i_vals, q_vals, freqs, qualities=None,
                           quality_min=2, pad_factor=8, offset_m=0.0):
    """Improved IFFT with all enhancements."""
    # Quality filter
    if qualities is not None:
        mask = qualities >= quality_min
        if np.sum(mask) < 8:
            return np.nan
        i_filt = i_vals[mask]
        q_filt = q_vals[mask]
    else:
        i_filt = i_vals
        q_filt = q_vals

    n = len(i_filt)
    h = i_filt + 1j * q_filt

    # Hann window
    hann = 0.5 * (1 - np.cos(2 * np.pi * np.arange(n) / (n - 1)))
    h_win = h * hann

    # Zero-pad
    m = n * pad_factor
    h_padded = np.zeros(m, dtype=complex)
    h_padded[:n] = h_win

    # IFFT
    cir = np.fft.ifft(h_padded)
    mag = np.abs(cir)

    # First-path detection
    noise_start = 3 * m // 4
    noise_floor = np.mean(mag[noise_start:])
    threshold = noise_floor * 4.0  # ~6 dB

    peak = -1
    for i in range(1, m // 2):
        if (mag[i] > threshold and
                mag[i] >= mag[i - 1] and
                (i + 1 >= m or mag[i] >= mag[i + 1])):
            peak = i
            break

    if peak < 0:
        return np.nan

    # Parabolic interpolation
    if 0 < peak < m - 1:
        alpha = mag[peak - 1]
        beta = mag[peak]
        gamma = mag[peak + 1]
        denom = alpha - 2 * beta + gamma
        if abs(denom) > 1e-10:
            frac = peak + 0.5 * (alpha - gamma) / denom
        else:
            frac = float(peak)
    else:
        frac = float(peak)

    d = frac * SPEED_OF_LIGHT / (2 * m * TONE_SPACING_HZ)
    d -= offset_m
    return max(0.05, d)


# ---------------------------------------------------------------------------
# Estimator: improved phase slope (weighted + IRLS)
# ---------------------------------------------------------------------------

def estimate_phase_slope_improved(i_vals, q_vals, freqs, qualities=None,
                                  quality_min=2, offset_m=0.0):
    """Improved phase slope: quality filter + weighted regression + IRLS."""
    if qualities is not None:
        mask = qualities >= quality_min
        if np.sum(mask) < 8:
            return np.nan
        i_filt = i_vals[mask]
        q_filt = q_vals[mask]
        f_filt = freqs[mask]
        q_filt_quality = qualities[mask]
    else:
        i_filt = i_vals
        q_filt = q_vals
        f_filt = freqs
        q_filt_quality = np.full(len(freqs), 3, dtype=np.uint8)

    phases = np.arctan2(q_filt, i_filt)
    phases = np.unwrap(phases)
    weights = q_filt_quality.astype(float) / 3.0

    def wls(x, y, w):
        sw = np.sum(w)
        swx = np.sum(w * x)
        swy = np.sum(w * y)
        swxx = np.sum(w * x * x)
        swxy = np.sum(w * x * y)
        det = sw * swxx - swx * swx
        if abs(det) < 1e-30:
            return 0.0, y - np.mean(y)
        slope = (sw * swxy - swx * swy) / det
        intercept = (swy - slope * swx) / sw
        residuals = y - (slope * x + intercept)
        return slope, residuals

    # Pass 1
    slope, residuals = wls(f_filt, phases, weights)

    # IRLS pass: down-weight tones with large residuals
    mad = np.median(np.abs(residuals))
    if mad < 1e-6:
        mad = 1e-6
    for i in range(len(weights)):
        if abs(residuals[i]) > 1.5 * mad:
            weights[i] *= 0.1

    # Pass 2
    slope, _ = wls(f_filt, phases, weights)

    d = abs(slope) * SPEED_OF_LIGHT / (4 * np.pi)
    d -= offset_m
    return max(0.05, d)


# ---------------------------------------------------------------------------
# Estimator: fused (improved IFFT + improved phase slope)
# ---------------------------------------------------------------------------

def estimate_fused(i_vals, q_vals, freqs, qualities,
                   phase_offset_m=0.0, ifft_offset_m=0.0,
                   agree_threshold_m=0.3):
    """Fused estimator: average when they agree, IFFT when they don't."""
    d_phase = estimate_phase_slope_improved(i_vals, q_vals, freqs, qualities,
                                            offset_m=phase_offset_m)
    d_ifft = estimate_ifft_improved(i_vals, q_vals, freqs, qualities,
                                    offset_m=ifft_offset_m)

    if np.isnan(d_phase) and np.isnan(d_ifft):
        return np.nan
    if np.isnan(d_ifft):
        return d_phase
    if np.isnan(d_phase):
        return d_ifft

    if abs(d_phase - d_ifft) <= agree_threshold_m:
        return 0.5 * (d_phase + d_ifft)
    else:
        return d_ifft  # Trust IFFT in multipath


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_simulation(n_trials=500, distances=None, snr_db=20.0,
                   n_reflections=3):
    """Run estimator comparison across multiple distances and trials."""
    if distances is None:
        distances = np.arange(0.5, 10.5, 0.5)

    methods = {
        'Phase slope (raw)': [],
        'IFFT (raw)': [],
        'IFFT (improved)': [],
        'Fused (improved)': [],
    }

    for true_d in distances:
        for _ in range(n_trials):
            ch = generate_channel(true_d, n_reflections=n_reflections,
                                  snr_db=snr_db)
            i_vals, q_vals, qualities = channel_to_tones(ch, TONE_FREQS)

            d_ps_raw = estimate_phase_slope_raw(i_vals, q_vals, TONE_FREQS)
            d_ifft_raw = estimate_ifft_raw(i_vals, q_vals, TONE_FREQS)
            d_ifft_imp = estimate_ifft_improved(i_vals, q_vals, TONE_FREQS,
                                                qualities)
            d_fused = estimate_fused(i_vals, q_vals, TONE_FREQS, qualities)

            methods['Phase slope (raw)'].append(abs(d_ps_raw - true_d)
                                                if not np.isnan(d_ps_raw)
                                                else np.nan)
            methods['IFFT (raw)'].append(abs(d_ifft_raw - true_d)
                                         if not np.isnan(d_ifft_raw)
                                         else np.nan)
            methods['IFFT (improved)'].append(abs(d_ifft_imp - true_d)
                                              if not np.isnan(d_ifft_imp)
                                              else np.nan)
            methods['Fused (improved)'].append(abs(d_fused - true_d)
                                               if not np.isnan(d_fused)
                                               else np.nan)

    return methods


def plot_cdf(methods: dict, title="CS Distance Estimator Comparison"):
    """Plot CDF of absolute distance error for each method."""
    fig, ax = plt.subplots(figsize=(10, 6))

    colors = ['#cc4444', '#cc8844', '#4488cc', '#44aa44']
    for (name, errors), color in zip(methods.items(), colors):
        valid = [e for e in errors if not np.isnan(e)]
        if not valid:
            continue
        sorted_err = np.sort(valid)
        cdf = np.arange(1, len(sorted_err) + 1) / len(sorted_err)
        ax.plot(sorted_err, cdf, label=name, color=color, linewidth=2)

    ax.set_xlabel('Absolute distance error (m)')
    ax.set_ylabel('CDF')
    ax.set_title(title)
    ax.legend(loc='lower right')
    ax.set_xlim(0, 5)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('cs_estimator_comparison.png', dpi=150)
    plt.show()
    print("Saved cs_estimator_comparison.png")


def print_summary(methods: dict):
    """Print summary statistics for each method."""
    print("\n=== CS Estimator Comparison ===")
    print(f"{'Method':<25} {'Median':>8} {'P90':>8} {'P95':>8} {'Valid%':>8}")
    print("-" * 60)

    for name, errors in methods.items():
        valid = [e for e in errors if not np.isnan(e)]
        total = len(errors)
        if valid:
            med = np.median(valid)
            p90 = np.percentile(valid, 90)
            p95 = np.percentile(valid, 95)
            valid_pct = 100 * len(valid) / total
        else:
            med = p90 = p95 = np.nan
            valid_pct = 0.0
        print(f"{name:<25} {med:>7.3f}m {p90:>7.3f}m {p95:>7.3f}m {valid_pct:>7.1f}%")


if __name__ == "__main__":
    np.random.seed(42)
    print("Running CS estimator simulation (multipath, 500 trials/distance)...")
    results = run_simulation(n_trials=500, snr_db=20.0, n_reflections=3)
    print_summary(results)
    plot_cdf(results)
