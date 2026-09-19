"""
Temporal Median Consensus (FT_METHOD 5) defense against temporal desync attacks.

Core idea: Instead of using the most recent (potentially desynchronized) position
reading, each vehicle maintains a buffer of recent position observations and uses
the MEDIAN value across a consensus window. This rejects outliers caused by
intermittent delays, jitter, or packet drops.

This serves as both:
1. A Python reference implementation for offline analysis
2. The specification for the Java-based FT_METHOD 5 in Vehicle.java
"""

import numpy as np
from collections import deque
from typing import List, Optional


class TemporalMedianConsensus:
    """
    Temporal median consensus filter.
    
    Maintains a sliding window of position observations and returns the
    median position as the "consensus" value. This naturally rejects
    outliers from delayed, jittered, or dropped packets.
    
    The window size is the primary design parameter:
    - Larger window = more robust but higher latency
    - Smaller window = faster response but less robust
    
    Default window = 5 (covers 5 seconds at TIME_STEP=1.0)
    """

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.positions: deque = deque(maxlen=window_size)
        self.timestamps: deque = deque(maxlen=window_size)

    def update(self, position: float, timestamp: int) -> float:
        """Add a new position observation and return the consensus position."""
        self.positions.append(position)
        self.timestamps.append(timestamp)

        if len(self.positions) < 3:
            return position

        # Return median of the position buffer
        return float(np.median(list(self.positions)))

    def get_consensus_position(self) -> Optional[float]:
        """Get the current consensus position without adding a new observation."""
        if not self.positions:
            return None
        return float(np.median(list(self.positions)))

    def get_consensus_gap(
        self,
        leader_positions: List[float],
        follower_position: float,
        follower_length: float = 4.5,
        threshold_distance: float = 0.5,
    ) -> float:
        """
        Compute the consensus gap using median of all observed positions.
        
        gap = median(leader_positions) - threshold_distance - follower_position
        """
        if not leader_positions:
            return 0.0
        leader_consensus = float(np.median(leader_positions))
        return leader_consensus - threshold_distance - follower_position

    def reset(self):
        """Clear the position buffer."""
        self.positions.clear()
        self.timestamps.clear()


def online_consensus_estimate(
    raw_gaps: np.ndarray,
    window_size: int = 5,
    outlier_threshold: float = 3.0,
) -> np.ndarray:
    """
    Apply temporal median consensus to a stream of raw gap measurements.
    
    Args:
        raw_gaps: Array of raw (attacked) gap measurements
        window_size: Size of sliding window for median
        outlier_threshold: Z-score threshold for identifying outliers
        
    Returns:
        Array of consensus-filtered gap estimates
    """
    filtered = np.zeros_like(raw_gaps)
    consensus = TemporalMedianConsensus(window_size)

    for i in range(len(raw_gaps)):
        filtered[i] = consensus.update(raw_gaps[i], i)

    return filtered


def evaluate_defense(
    true_gaps: np.ndarray,
    attacked_gaps: np.ndarray,
    window_sizes: List[int] = [3, 5, 7, 10],
) -> dict:
    """
    Evaluate temporal median consensus defense performance.
    
    Returns:
        Dict with RMSE, MAE before and after filtering for each window size
    """
    results = {}
    for ws in window_sizes:
        filtered = online_consensus_estimate(attacked_gaps, ws)
        rmse_before = np.sqrt(np.mean((true_gaps - attacked_gaps) ** 2))
        rmse_after = np.sqrt(np.mean((true_gaps - filtered) ** 2))
        mae_before = np.mean(np.abs(true_gaps - attacked_gaps))
        mae_after = np.mean(np.abs(true_gaps - filtered))
        improvement = ((mae_before - mae_after) / mae_before) * 100

        results[ws] = {
            "rmse_before": rmse_before,
            "rmse_after": rmse_after,
            "mae_before": mae_before,
            "mae_after": mae_after,
            "improvement_pct": improvement,
        }

    return results
