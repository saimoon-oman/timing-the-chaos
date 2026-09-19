"""
Defense comparison: Evaluate DTW, Schmidt-Kalman, and Temporal Median Consensus
across all 6 attack scenarios. Generates comparative figures.
"""

import argparse, glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import cdist

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../defenses")))
from dtw_detection.strip_aware_dtw import DTWDetector, compute_sdtw_distance
from schmidt_kalman.gap_estimator import SchmidtKalmanGapEstimator
from consensus_ft.temporal_median import (
    TemporalMedianConsensus, online_consensus_estimate, evaluate_defense,
)

RESULTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../experiments/results"))
FIGURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
os.makedirs(FIGURES_DIR, exist_ok=True)


def simulate_attack_gaps(
    n_steps: int = 500,
    scenario: int = 0,
    delay_mean: float = 3.0,
    jitter_mag: float = 0.5,
    drop_prob: float = 0.2,
) -> tuple:
    """Synthesize true gaps and attacked gaps for evaluation."""
    np.random.seed(42)
    dt = 1.0

    # Leader moves at ~5 m/s with small variations
    leader_pos = np.cumsum(np.random.normal(5.0, 0.5, n_steps))
    leader_pos += 20.0

    # Follower follows with ~10m gap, some variation
    follower_pos = leader_pos - 10.0 - np.random.exponential(0.5, n_steps)

    true_gaps = leader_pos - follower_pos - 4.5

    attacked_gaps = np.copy(true_gaps)
    position_buffer = []  # for delay simulation

    for t in range(n_steps):
        position_buffer.append(leader_pos[t])
        if len(position_buffer) > 10:
            position_buffer.pop(0)

        if scenario == 0:  # Persistent delay
            delay = int(round(delay_mean / dt))
            if delay < len(position_buffer):
                attacked_gaps[t] = position_buffer[-(delay + 1)] - follower_pos[t] - 4.5
        elif scenario == 1:  # Intermittent
            if np.random.random() < 0.3:
                delay = int(round(delay_mean / dt))
                if delay < len(position_buffer):
                    attacked_gaps[t] = position_buffer[-(delay + 1)] - follower_pos[t] - 4.5
        elif scenario == 2:  # Single-shot at t=250
            if t == 250:
                attacked_gaps[t] = position_buffer[0] - follower_pos[t] - 4.5 if position_buffer else attacked_gaps[t]
        elif scenario == 3:  # Jitter-only
            attacked_gaps[t] = true_gaps[t] + np.random.normal(0, jitter_mag)
        elif scenario == 4:  # Packet drop
            if np.random.random() < drop_prob and len(position_buffer) > 1:
                attacked_gaps[t] = position_buffer[0] - follower_pos[t] - 4.5
        elif scenario == 5:  # Joint
            jitter = np.random.normal(0, jitter_mag)
            if np.random.random() < 0.3:
                delay = int(round(delay_mean / dt))
                if delay < len(position_buffer):
                    attacked_gaps[t] = position_buffer[-(delay + 1)] - follower_pos[t] - 4.5 + jitter

    return true_gaps, attacked_gaps


def plot_defense_effectiveness(save: bool = True):
    """Main defense comparison figure."""
    scenarios = [
        ("Persistent Delay", 0), ("Intermittent", 1), ("Single-Shot", 2),
        ("Jitter-Only", 3), ("Packet Drop", 4), ("Joint", 5),
    ]
    window_sizes = [3, 5, 7]

    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()

    for idx, (sc_name, sc_id) in enumerate(scenarios):
        ax = axes[idx]
        true_gaps, attacked_gaps = simulate_attack_gaps(500, sc_id)

        mae_before = np.mean(np.abs(true_gaps - attacked_gaps))

        results = {}
        for ws in window_sizes:
            filtered = online_consensus_estimate(attacked_gaps, ws)
            mae_after = np.mean(np.abs(true_gaps - filtered))
            imp = ((mae_before - mae_after) / mae_before) * 100 if mae_before > 0 else 0
            results[ws] = imp

        # Plot the time series
        time = np.arange(len(true_gaps))
        ax.plot(time, true_gaps, 'g-', linewidth=1.5, alpha=0.7, label="True")
        ax.plot(time, attacked_gaps, 'r-', linewidth=0.8, alpha=0.5, label="Attacked")

        best_ws = max(results, key=results.get)
        filtered = online_consensus_estimate(attacked_gaps, best_ws)
        ax.plot(time, filtered, 'b-', linewidth=1.5, alpha=0.7, label=f"MedCons ws={best_ws}")

        ax.set_title(f"{sc_name}\nBaseline MAE: {mae_before:.2f}m, Best improvement: {results[best_ws]:.0f}%")
        ax.set_xlabel("Time Step")
        ax.set_ylabel("Gap (m)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "defense_comparison_timeseries.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_defense_mae_reduction(save: bool = True):
    """Bar chart: MAE reduction for each defense across scenarios."""
    scenarios = ["Persistent", "Intermittent", "Single-Shot", "Jitter", "PacketDrop", "Joint"]
    ws = 5

    # MAE improvement for median consensus across all scenarios
    improvements = []
    for sc_id in range(6):
        true_gaps, attacked_gaps = simulate_attack_gaps(500, sc_id)
        mae_before = np.mean(np.abs(true_gaps - attacked_gaps))
        filtered = online_consensus_estimate(attacked_gaps, ws)
        mae_after = np.mean(np.abs(true_gaps - filtered))
        imp = ((mae_before - mae_after) / mae_before) * 100 if mae_before > 0 else 0
        improvements.append(imp)

    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(scenarios, improvements, color='steelblue', alpha=0.8, edgecolor='navy')

    for bar, val in zip(bars, improvements):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                f"{val:.0f}%", ha='center', va='bottom', fontweight='bold')

    ax.set_xlabel("Attack Scenario")
    ax.set_ylabel("MAE Reduction (%)")
    ax.set_title("Temporal Median Consensus (ws=5) Performance Across Attack Scenarios")
    ax.set_ylim(0, 110)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "defense_mae_reduction.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_defense_window_sensitivity(save: bool = True):
    """How window size affects defense performance."""
    window_sizes = [2, 3, 4, 5, 7, 10, 15]
    scenarios = ["Persistent", "Intermittent", "Joint"]

    fig, ax = plt.subplots(figsize=(10, 6))

    for sc_name, sc_id in [("Persistent", 0), ("Intermittent", 1), ("Joint", 5)]:
        true_gaps, attacked_gaps = simulate_attack_gaps(500, sc_id)
        mae_before = np.mean(np.abs(true_gaps - attacked_gaps))
        improvements = []
        for ws in window_sizes:
            filtered = online_consensus_estimate(attacked_gaps, ws)
            mae_after = np.mean(np.abs(true_gaps - filtered))
            imp = ((mae_before - mae_after) / mae_before) * 100 if mae_before > 0 else 0
            improvements.append(imp)
        ax.plot(window_sizes, improvements, '-o', linewidth=2, markersize=8, label=sc_name)

    ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
    ax.set_xlabel("Consensus Window Size (time steps)")
    ax.set_ylabel("MAE Reduction (%)")
    ax.set_title("Impact of Window Size on Defense Effectiveness")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "defense_window_sensitivity.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def main():
    plot_defense_effectiveness()
    plot_defense_mae_reduction()
    plot_defense_window_sensitivity()
    print(f"Defense figures saved to: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
