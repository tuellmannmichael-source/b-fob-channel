"""
RSSI path-loss model and Kalman filter -- Python reference implementation.

This mirrors the C firmware logic and lets you experiment with parameters
before flashing to hardware.
"""

import numpy as np


def rssi_at_distance(d: float, rssi_ref: float = -59.0, n: float = 3.0) -> float:
    """
    Log-distance path-loss model.

    RSSI(d) = RSSI(d0) - 10 * n * log10(d / d0)

    Args:
        d:        Distance in meters (must be > 0).
        rssi_ref: Calibrated RSSI at 1 meter (dBm).
        n:        Path-loss exponent.

    Returns:
        RSSI in dBm.
    """
    if d <= 0:
        d = 0.01
    return rssi_ref - 10.0 * n * np.log10(d)


def distance_from_rssi(rssi: float, rssi_ref: float = -59.0, n: float = 3.0) -> float:
    """
    Inverse of log-distance path-loss model.

    d = 10 ^ ((RSSI_ref - RSSI) / (10 * n))

    Args:
        rssi:     Measured RSSI in dBm.
        rssi_ref: Calibrated RSSI at 1 meter (dBm).
        n:        Path-loss exponent.

    Returns:
        Estimated distance in meters.
    """
    exponent = (rssi_ref - rssi) / (10.0 * n)
    return 10.0 ** exponent


def add_rssi_noise(rssi: float, std_dev: float = 3.0) -> float:
    """Add Gaussian noise to an RSSI value to simulate real measurements."""
    return rssi + np.random.normal(0, std_dev)


class KalmanFilter1D:
    """
    1D Kalman filter for RSSI smoothing.

    State model:    x_k = x_{k-1} + w,  w ~ N(0, Q)
    Measurement:    z_k = x_k + v,      v ~ N(0, R)
    """

    def __init__(self, q: float = 0.5, r: float = 8.0, initial: float = -60.0):
        """
        Args:
            q:       Process noise variance.
            r:       Measurement noise variance.
            initial: Initial state estimate (RSSI in dBm).
        """
        self.x = initial
        self.p = r  # Start uncertainty at measurement noise level
        self.q = q
        self.r = r

    def update(self, z: float) -> float:
        """
        Feed a new RSSI measurement and return the filtered estimate.

        Args:
            z: Raw RSSI measurement (dBm).

        Returns:
            Filtered RSSI estimate (dBm).
        """
        # Predict
        x_pred = self.x
        p_pred = self.p + self.q

        # Update
        k = p_pred / (p_pred + self.r)  # Kalman gain
        self.x = x_pred + k * (z - x_pred)
        self.p = (1.0 - k) * p_pred

        return self.x

    @property
    def estimate(self) -> float:
        return self.x


if __name__ == "__main__":
    # Quick demo: show RSSI vs distance and Kalman filtering
    import matplotlib.pyplot as plt

    # --- RSSI vs Distance curve ---
    distances = np.linspace(0.3, 10.0, 100)
    rssi_clean = [rssi_at_distance(d) for d in distances]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(distances, rssi_clean, "b-", label="Model (n=3.0)")
    axes[0].set_xlabel("Distance (m)")
    axes[0].set_ylabel("RSSI (dBm)")
    axes[0].set_title("Log-Distance Path-Loss Model")
    axes[0].legend()
    axes[0].grid(True)

    # --- Kalman filter demo ---
    true_rssi = -65.0  # Stationary keyfob
    n_samples = 100
    noisy = [add_rssi_noise(true_rssi, std_dev=4.0) for _ in range(n_samples)]

    kf = KalmanFilter1D(q=0.5, r=16.0, initial=noisy[0])
    filtered = [kf.update(z) for z in noisy]

    axes[1].plot(noisy, "r.", alpha=0.4, label="Raw RSSI")
    axes[1].plot(filtered, "b-", linewidth=2, label="Kalman filtered")
    axes[1].axhline(true_rssi, color="g", linestyle="--", label="True RSSI")
    axes[1].set_xlabel("Sample")
    axes[1].set_ylabel("RSSI (dBm)")
    axes[1].set_title("Kalman Filter on RSSI")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig("rssi_model_demo.png", dpi=150)
    plt.show()
    print("Saved rssi_model_demo.png")
