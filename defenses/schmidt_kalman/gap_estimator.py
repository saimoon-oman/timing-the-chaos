"""
Schmidt-Kalman gap estimator for temporal desynchronization recovery.
Uses a reduced-order Kalman filter that marginalizes over timing uncertainty
while estimating the true gap between vehicles.

Reference: Schmidt, S. F. (1966). Application of State-Space Methods to
Navigation Problems. Advances in Control Systems, 3, 293-340.
"""

import numpy as np
from typing import Tuple


class SchmidtKalmanGapEstimator:
    """
    Schmidt-Kalman filter for estimating true inter-vehicle gap under
    temporal desynchronization.
    
    State vector: [x_follower, x_leader, v_follower, v_leader]
    Measurement: observed (potentially delayed) gap
    The Schmidt formulation treats the timing offset as a "consider" parameter
    whose uncertainty is propagated but not actively estimated.
    """

    def __init__(
        self,
        dt: float = 1.0,
        process_noise: float = 0.5,
        measurement_noise: float = 2.0,
        timing_uncertainty: float = 1.0,
    ):
        self.dt = dt
        self.q = process_noise  # Process noise std
        self.r = measurement_noise  # Measurement noise std
        self.tau = timing_uncertainty  # Timing uncertainty std (seconds)

        # State: [x1, x2, v1, v2]
        self.x = np.zeros(4)
        self.P = np.eye(4) * 10.0  # Initial covariance

        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        # Process noise covariance
        q2 = self.q ** 2
        self.Q = np.array([
            [q2 * dt**3 / 3, 0, q2 * dt**2 / 2, 0],
            [0, q2 * dt**3 / 3, 0, q2 * dt**2 / 2],
            [q2 * dt**2 / 2, 0, q2 * dt, 0],
            [0, q2 * dt**2 / 2, 0, q2 * dt],
        ])

        # Measurement matrix (observes gap = x2 - x1 - length)
        # With timing offset: observed gap = x2(t) - x1(t - tau)
        # Linearized: x2(t) - x1(t) + v1(t) * tau
        self.H = np.array([[-1, 1, 0, 0]])

        # Schmidt "consider" parameter mapping for timing offset tau
        # The timing error maps to position error via v1 * tau
        self.G = np.array([[0], [0], [0], [0]])  # Will be updated with v1

        # Consider covariance
        self.P_tt = self.tau ** 2  # Timing uncertainty variance

    def predict(self):
        """Time update (prediction step)."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measured_gap: float, leader_length: float = 4.5):
        """Measurement update with gap observation."""
        # Update Schmidt mapping with current speed estimate
        v1 = self.x[2]
        self.G = np.array([[v1], [0], [0], [0]])

        # Expected measurement
        z_pred = self.x[1] - self.x[0] - leader_length

        # Innovation covariance with Schmidt consideration
        # P_z = H @ P @ H.T + R + H @ G @ P_tt @ G.T @ H.T
        HP = self.H @ self.P
        S = HP @ self.H.T + self.r ** 2
        S += (self.H @ self.G) @ self.P_tt @ (self.G.T @ self.H.T)
        S = S.reshape(1, 1)

        # Kalman gain (reduced - does not update timing state)
        K = (self.P @ self.H.T) / S[0, 0]

        # State update
        innovation = measured_gap - z_pred
        self.x = self.x + K.flatten() * innovation

        # Covariance update (Joseph form)
        I_KH = np.eye(4) - np.outer(K, self.H[0])
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * self.r ** 2

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2

    def get_estimated_gap(self, leader_length: float = 4.5) -> float:
        """Get the current best estimate of the true gap."""
        return self.x[1] - self.x[0] - leader_length

    def get_timing_uncertainty(self) -> float:
        """Get the current timing uncertainty estimate."""
        return np.sqrt(self.P_tt)

    def get_position(self) -> Tuple[float, float]:
        """Get estimated positions of follower and leader."""
        return self.x[0], self.x[1]
