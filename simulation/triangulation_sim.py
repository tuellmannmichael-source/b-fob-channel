"""2D trilateration simulation with RSSI / CS / Hybrid comparison.

The simulator models three receiver antennas in a vehicle and compares three
distance sourcing strategies:

* RSSI-only
* CS-only (Channel Sounding style distance estimate)
* Hybrid (prefer CS, fallback to RSSI)

It reports accuracy and robustness metrics:

* Median and P90 error
* Time-to-first-fix (TTFF)
* Robustness under measurement dropouts
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

from rssi_model import rssi_at_distance, distance_from_rssi, add_rssi_noise, KalmanFilter1D


# ---------------------------------------------------------------------------
# Vehicle / antenna geometry
# ---------------------------------------------------------------------------

# Antenna positions (meters) in vehicle coordinate system
# Origin = center of rear axle, X = right, Y = forward
ANTENNAS = np.array([
    [0.0, 4.5],    # A: dashboard center
    [-0.9, 0.0],   # B: left rear
    [0.9, 0.0],    # C: right rear
])

RSSI_REF = -59.0   # RSSI at 1 meter (dBm)
PATH_LOSS_N = 3.0   # Path-loss exponent
RSSI_NOISE_STD = 4.0  # Standard deviation of RSSI noise (dBm)

CS_NOISE_STD = 0.10  # meters, small CS jitter
CS_NON_GAUSSIAN_TAIL_PROB = 0.15
CS_NON_GAUSSIAN_TAIL_SCALE = 0.15  # meters

RSSI_DROPOUT_PROB = 0.18
CS_DROPOUT_PROB = 0.30
SIM_DT_SECONDS = 0.2


# ---------------------------------------------------------------------------
# Trilateration: algebraic (closed-form for 3 antennas)
# ---------------------------------------------------------------------------

def trilaterate_algebraic(antennas: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """
    Solve 2D position from 3 distance measurements using the algebraic
    (circle-subtraction) method.

    Each distance defines a circle:
        (x - x_i)^2 + (y - y_i)^2 = d_i^2

    Subtracting pairs eliminates the quadratic terms, yielding a 2x2
    linear system.

    Args:
        antennas:  (3, 2) array of antenna positions.
        distances: (3,) array of estimated distances.

    Returns:
        (2,) array [x, y] of estimated position, or [nan, nan] if degenerate.
    """
    xa, ya = antennas[0]
    xb, yb = antennas[1]
    xc, yc = antennas[2]
    da, db, dc = distances

    # 2x2 linear system coefficients
    A = np.array([
        [2 * (xb - xa), 2 * (yb - ya)],
        [2 * (xc - xa), 2 * (yc - ya)],
    ])
    b = np.array([
        da**2 - db**2 - xa**2 + xb**2 - ya**2 + yb**2,
        da**2 - dc**2 - xa**2 + xc**2 - ya**2 + yc**2,
    ])

    det = np.linalg.det(A)
    if abs(det) < 1e-10:
        return np.array([np.nan, np.nan])

    return np.linalg.solve(A, b)


# ---------------------------------------------------------------------------
# Trilateration: nonlinear least-squares (Gauss-Newton via scipy)
# ---------------------------------------------------------------------------

def trilaterate_least_squares(antennas: np.ndarray, distances: np.ndarray,
                              x0: np.ndarray = None) -> np.ndarray:
    """
    Solve 2D position by minimizing:
        sum_i [ (||pos - antenna_i|| - d_i)^2 ]

    Uses scipy.optimize.least_squares (Levenberg-Marquardt).

    Args:
        antennas:  (N, 2) array of antenna positions.
        distances: (N,) array of estimated distances.
        x0:        Initial guess (2,). Defaults to centroid of antennas.

    Returns:
        (2,) array [x, y] of estimated position.
    """
    if x0 is None:
        x0 = antennas.mean(axis=0)

    def residuals(pos):
        diffs = antennas - pos
        dists_est = np.sqrt(np.sum(diffs**2, axis=1))
        return dists_est - distances

    result = least_squares(residuals, x0, method="lm")
    return result.x


def sample_cs_distance(true_distance: float, non_gaussian: bool = True) -> float:
    """Sample CS distance measurement with optional non-Gaussian noise."""
    error = np.random.normal(0.0, CS_NOISE_STD)
    if non_gaussian and np.random.random() < CS_NON_GAUSSIAN_TAIL_PROB:
        # Small, heavy-tailed disturbance (e.g., multipath residuals)
        error += np.random.laplace(0.0, CS_NON_GAUSSIAN_TAIL_SCALE)
    return max(0.05, true_distance + error)


def estimate_position_from_distances(distances: np.ndarray) -> tuple[np.ndarray, bool]:
    """Estimate position if enough valid distances are available."""
    valid = np.isfinite(distances)
    if np.sum(valid) < 3:
        return np.array([np.nan, np.nan]), False

    pos = trilaterate_least_squares(ANTENNAS[valid], distances[valid])
    return pos, True


def evaluate_scenario(true_pos: np.ndarray,
                      scenario: str,
                      n_steps: int,
                      non_gaussian_cs: bool) -> dict:
    """Evaluate one scenario over time for a fixed true position."""
    true_dists = np.sqrt(np.sum((ANTENNAS - true_pos) ** 2, axis=1))
    true_rssi = np.array([rssi_at_distance(d, RSSI_REF, PATH_LOSS_N) for d in true_dists])

    # Note: r = variance of RSSI noise (std^2 = 16.0).
    # The firmware Kconfig uses a tuned R = 8.0 (KALMAN_R_X10=80 / 10).
    # The difference is intentional: the simulation uses the theoretical
    # variance while the firmware value is empirically tuned for hardware.
    filters = [
        KalmanFilter1D(q=0.5, r=RSSI_NOISE_STD**2, initial=true_rssi[i])
        for i in range(len(ANTENNAS))
    ]

    step_errors = []
    valid_fixes = 0
    dropout_steps = 0
    valid_fixes_during_dropout = 0
    ttff_s = np.nan

    for step in range(n_steps):
        rssi_distances = np.full(len(ANTENNAS), np.nan)
        cs_distances = np.full(len(ANTENNAS), np.nan)
        had_dropout = False

        for i in range(len(ANTENNAS)):
            rssi_available = np.random.random() >= RSSI_DROPOUT_PROB
            cs_available = np.random.random() >= CS_DROPOUT_PROB
            had_dropout |= (not rssi_available) or (not cs_available)

            if rssi_available:
                noisy = add_rssi_noise(true_rssi[i], RSSI_NOISE_STD)
                filters[i].update(noisy)
                rssi_distances[i] = distance_from_rssi(filters[i].estimate, RSSI_REF, PATH_LOSS_N)

            if cs_available:
                cs_distances[i] = sample_cs_distance(true_dists[i], non_gaussian=non_gaussian_cs)

        if scenario == "rssi":
            fused_distances = rssi_distances
        elif scenario == "cs":
            fused_distances = cs_distances
        elif scenario == "hybrid":
            fused_distances = np.where(np.isfinite(cs_distances), cs_distances, rssi_distances)
        else:
            raise ValueError(f"Unknown scenario: {scenario}")

        pos_est, is_fix = estimate_position_from_distances(fused_distances)
        if is_fix:
            err = np.linalg.norm(pos_est - true_pos)
            step_errors.append(err)
            valid_fixes += 1
            if np.isnan(ttff_s):
                ttff_s = step * SIM_DT_SECONDS

            if had_dropout:
                valid_fixes_during_dropout += 1

        if had_dropout:
            dropout_steps += 1

    return {
        "scenario": scenario,
        "errors": step_errors,
        "ttff_s": ttff_s,
        "fix_rate": valid_fixes / n_steps,
        "dropout_recovery_rate": (
            valid_fixes_during_dropout / dropout_steps if dropout_steps else np.nan
        ),
    }


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_grid_simulation(grid_step: float = 0.5,
                        n_steps: int = 50,
                        non_gaussian_cs: bool = True):
    """Run scenario comparison across a grid of keyfob positions."""
    x_range = np.arange(-1.5, 2.0, grid_step)
    y_range = np.arange(-1.0, 6.0, grid_step)

    scenario_results = {"rssi": [], "cs": [], "hybrid": []}
    for x in x_range:
        for y in y_range:
            true_pos = np.array([x, y])
            for scenario in scenario_results:
                scenario_results[scenario].append(
                    evaluate_scenario(
                        true_pos,
                        scenario=scenario,
                        n_steps=n_steps,
                        non_gaussian_cs=non_gaussian_cs,
                    )
                )

    return scenario_results


def summarize_results(scenario_results: dict):
    """Build scenario metrics required for comparison output."""
    summary = {}

    for scenario, runs in scenario_results.items():
        errors = [e for run in runs for e in run["errors"]]
        ttffs = [run["ttff_s"] for run in runs if np.isfinite(run["ttff_s"])]
        fix_rates = [run["fix_rate"] for run in runs]
        dropout_recovery = [run["dropout_recovery_rate"] for run in runs
                            if np.isfinite(run["dropout_recovery_rate"])]

        summary[scenario] = {
            "median_error_m": np.median(errors) if errors else np.nan,
            "p90_error_m": np.percentile(errors, 90) if errors else np.nan,
            "ttff_s": np.mean(ttffs) if ttffs else np.nan,
            "fix_rate": np.mean(fix_rates) if fix_rates else np.nan,
            "dropout_recovery_rate": (
                np.mean(dropout_recovery) if dropout_recovery else np.nan
            ),
        }

    return summary


def print_summary(summary: dict):
    """Print compact scenario comparison table."""
    names = {"rssi": "RSSI-only", "cs": "CS-only", "hybrid": "Hybrid (CS+RSSI)"}
    print("\n=== Scenario comparison ===")
    print("Scenario            MedianErr  P90Err   TTFF    FixRate  DropoutRobust")
    print("-----------------------------------------------------------------------")
    for key in ["rssi", "cs", "hybrid"]:
        s = summary[key]
        print(f"{names[key]:<18} {s['median_error_m']:>7.2f}m  {s['p90_error_m']:>6.2f}m  "
              f"{s['ttff_s']:>5.2f}s  {100*s['fix_rate']:>6.1f}%   "
              f"{100*s['dropout_recovery_rate']:>6.1f}%")


def plot_results(summary: dict):
    """Visualize metric comparison of scenarios."""
    labels = ["RSSI-only", "CS-only", "Hybrid"]
    median_vals = [summary[k]["median_error_m"] for k in ["rssi", "cs", "hybrid"]]
    p90_vals = [summary[k]["p90_error_m"] for k in ["rssi", "cs", "hybrid"]]
    ttff_vals = [summary[k]["ttff_s"] for k in ["rssi", "cs", "hybrid"]]
    robust_vals = [100 * summary[k]["dropout_recovery_rate"] for k in ["rssi", "cs", "hybrid"]]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()
    bars = [median_vals, p90_vals, ttff_vals, robust_vals]
    titles = ["Median error (m)", "P90 error (m)", "Time-to-first-fix (s)", "Dropout robustness (%)"]

    for ax, vals, title in zip(axes, bars, titles):
        ax.bar(labels, vals, color=["#999999", "#4c72b0", "#55a868"])
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)

    plt.suptitle("Triangulation scenarios: RSSI vs CS vs Hybrid")
    plt.tight_layout()
    plt.savefig("triangulation_scenario_comparison.png", dpi=150)
    plt.show()
    print("Saved triangulation_scenario_comparison.png")


if __name__ == "__main__":
    np.random.seed(42)
    runs = run_grid_simulation(grid_step=0.5, n_steps=60, non_gaussian_cs=True)
    summary_metrics = summarize_results(runs)
    print_summary(summary_metrics)
    plot_results(summary_metrics)
