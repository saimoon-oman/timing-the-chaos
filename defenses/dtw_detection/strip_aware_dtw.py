"""
Strip-aware DTW (Dynamic Time Warping) detection for temporal desync attacks.

Key insight: In DhakaSim's strip-based environment, even small timing errors
manifest as discrete strip misalignments. DTW on strip-occupancy sequences
detects abnormal temporal patterns that indicate desynchronization.
"""

import numpy as np
from scipy.spatial.distance import euclidean
from scipy.spatial.distance import cdist
from typing import List, Optional, Tuple


def strip_quantize_positions(positions: np.ndarray, strip_width: float = 0.5) -> np.ndarray:
    """Convert continuous positions to discrete strip indices."""
    return np.floor(positions / strip_width).astype(int)


def compute_dtw_distance(seq1: np.ndarray, seq2: np.ndarray) -> float:
    """Compute DTW distance between two sequences."""
    n, m = len(seq1), len(seq2)
    dtw_matrix = np.full((n + 1, m + 1), np.inf)
    dtw_matrix[0, 0] = 0

    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = abs(seq1[i - 1] - seq2[j - 1])
            dtw_matrix[i, j] = cost + min(
                dtw_matrix[i - 1, j],
                dtw_matrix[i, j - 1],
                dtw_matrix[i - 1, j - 1],
            )

    return dtw_matrix[n, m] / max(n, m)


def compute_sdtw_distance(seq1: np.ndarray, seq2: np.ndarray, strip_width: float = 0.5) -> float:
    """
    Strip-aware DTW distance.
    First quantizes positions to strips, then computes DTW on strip sequences.
    """
    s1 = strip_quantize_positions(seq1, strip_width)
    s2 = strip_quantize_positions(seq2, strip_width)
    return compute_dtw_distance(s1, s2)


class DTWDetector:
    """
    Detects temporal desync attacks by comparing observed position sequences
    against expected (predicted) sequences using strip-aware DTW.
    
    The detector maintains a baseline of normal DTW distances and flags
    windows where the DTW distance exceeds a threshold.
    """

    def __init__(
        self,
        window_size: int = 10,
        threshold_multiplier: float = 3.0,
        strip_width: float = 0.5,
        min_samples_for_baseline: int = 50,
    ):
        self.window_size = window_size
        self.threshold_multiplier = threshold_multiplier
        self.strip_width = strip_width
        self.min_samples_for_baseline = min_samples_for_baseline

        self.baseline_distances: List[float] = []
        self.observed_positions: List[float] = []
        self.expected_positions: List[float] = []
        self.alarms: List[int] = []

    def update(self, observed_pos: float, expected_pos: float, timestep: int) -> bool:
        """Update detector with new observation. Returns True if attack detected."""
        self.observed_positions.append(observed_pos)
        self.expected_positions.append(expected_pos)

        if len(self.observed_positions) < self.window_size:
            return False

        # Compute DTW on the last window_size samples
        obs_window = np.array(self.observed_positions[-self.window_size:])
        exp_window = np.array(self.expected_positions[-self.window_size:])

        dtw_dist = compute_sdtw_distance(obs_window, exp_window, self.strip_width)

        if len(self.baseline_distances) < self.min_samples_for_baseline:
            self.baseline_distances.append(dtw_dist)
            return False

        # Compute threshold from baseline
        mean_baseline = np.mean(self.baseline_distances)
        std_baseline = np.std(self.baseline_distances)
        threshold = mean_baseline + self.threshold_multiplier * std_baseline

        if dtw_dist > threshold:
            self.alarms.append(timestep)
            return True

        # Update baseline (sliding window)
        self.baseline_distances.append(dtw_dist)
        if len(self.baseline_distances) > self.min_samples_for_baseline * 2:
            self.baseline_distances = self.baseline_distances[-self.min_samples_for_baseline:]

        return False

    def get_alarm_rate(self) -> float:
        """Fraction of time steps where attack was flagged."""
        if not self.alarms:
            return 0.0
        total_steps = len(self.observed_positions)
        return len(self.alarms) / total_steps

    def get_false_positive_rate(self, attack_active_fn) -> float:
        """Compute false positive rate (FP / (FP + TN))."""
        fp = 0
        tn = 0
        for t in range(1, len(self.observed_positions) + 1):
            is_attack = attack_active_fn(t)
            is_alarm = t in self.alarms
            if is_alarm and not is_attack:
                fp += 1
            elif not is_alarm and not is_attack:
                tn += 1
        return fp / max(fp + tn, 1)

    def get_detection_rate(self, attack_active_fn) -> float:
        """Compute true positive rate (TP / (TP + FN))."""
        tp = 0
        fn = 0
        for t in range(1, len(self.observed_positions) + 1):
            is_attack = attack_active_fn(t)
            is_alarm = t in self.alarms
            if is_attack and is_alarm:
                tp += 1
            elif is_attack and not is_alarm:
                fn += 1
        return tp / max(tp + fn, 1)

    def get_precision(self, attack_active_fn) -> float:
        """Compute precision (TP / (TP + FP))."""
        tp = 0
        fp = 0
        for t in range(1, len(self.observed_positions) + 1):
            is_attack = attack_active_fn(t)
            is_alarm = t in self.alarms
            if is_alarm and is_attack:
                tp += 1
            elif is_alarm and not is_attack:
                fp += 1
        return tp / max(tp + fp, 1)
