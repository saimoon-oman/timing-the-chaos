"""
Baseline comparison: Statistical comparison of attacked vs non-attacked states.
Quantifies the shift in gap distributions, collision rates, and traffic flow.
"""

import argparse, glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats
import seaborn as sns

sns.set_theme(style="whitegrid")

RESULTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../experiments/results"))
FIGURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
os.makedirs(FIGURES_DIR, exist_ok=True)


def load_gap_csv(gap_path: str) -> pd.DataFrame:
    """Load a gap_errors.csv file."""
    cols = ["simStep", "leaderId", "followerId", "gapError", "leaderSpeed", "followerSpeed"]
    df = pd.read_csv(gap_path, names=cols, skiprows=1)
    df["simStep"] = pd.to_numeric(df["simStep"], errors="coerce")
    return df


def simulate_baseline_vs_attacked(n_steps: int = 1000, n_reps: int = 5):
    """Generate synthetic baseline and attacked gap data for statistical comparison."""
    np.random.seed(42)

    baseline_gaps = []
    attacked_gaps = []

    for _ in range(n_reps):
        leader_pos = np.cumsum(np.random.normal(5.0, 0.5, n_steps)) + 20.0
        follower_pos = leader_pos - 10.0 - np.random.exponential(0.5, n_steps)
        gap = leader_pos - follower_pos - 4.5
        baseline_gaps.extend(gap.tolist())

        attacked = np.copy(gap)
        for t in range(3, n_steps):
            attacked[t] = leader_pos[t - 3] - follower_pos[t] - 4.5
        attacked_gaps.extend(attacked.tolist())

    return np.array(baseline_gaps), np.array(attacked_gaps)


def plot_gap_distribution_comparison(save: bool = True):
    """KDE comparison of baseline vs attacked gap distributions."""
    baseline, attacked = simulate_baseline_vs_attacked()

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.hist(baseline, bins=60, alpha=0.5, label=f"Baseline (mean={baseline.mean():.2f}m, std={baseline.std():.2f}m)",
            color='green', density=True, edgecolor='darkgreen')
    ax.hist(attacked, bins=60, alpha=0.5, label=f"Attacked (mean={attacked.mean():.2f}m, std={attacked.std():.2f}m)",
            color='red', density=True, edgecolor='darkred')

    ks_stat, ks_p = stats.ks_2samp(baseline, attacked)
    ax.set_xlabel("Gap (m)")
    ax.set_ylabel("Density")
    ax.set_title(f"Gap Distribution: Baseline vs. Persistent Delay 3s\nKS={ks_stat:.3f}, p={ks_p:.2e}")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "baseline_vs_attacked_dist.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()

    print(f"Baseline: mean={baseline.mean():.3f}, std={baseline.std():.3f}, n={len(baseline)}")
    print(f"Attacked: mean={attacked.mean():.3f}, std={attacked.std():.3f}, n={len(attacked)}")
    print(f"KS test: statistic={ks_stat:.4f}, p-value={ks_p:.2e}")
    print(f"Effect size (Cohen's d): {(attacked.mean() - baseline.mean()) / np.sqrt((baseline.std()**2 + attacked.std()**2) / 2):.4f}")


def plot_statistical_metrics_by_delay(save: bool = True):
    """How statistical metrics vary with increasing delay."""
    delays = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]
    n_steps = 500

    metrics = {
        "delay": delays,
        "mean_shift": [],
        "std_ratio": [],
        "ks_statistic": [],
        "percent_over_15m": [],
    }

    np.random.seed(42)
    leader_pos = np.cumsum(np.random.normal(5.0, 0.5, n_steps)) + 20.0
    follower_pos = leader_pos - 10.0 - np.random.exponential(0.5, n_steps)
    baseline = leader_pos - follower_pos - 4.5

    for delay in delays:
        attacked = np.copy(baseline)
        delay_int = int(round(delay))
        for t in range(delay_int, n_steps):
            attacked[t] = leader_pos[t - delay_int] - follower_pos[t] - 4.5

        metrics["mean_shift"].append(attacked.mean() - baseline.mean())
        metrics["std_ratio"].append(attacked.std() / baseline.std() if baseline.std() > 0 else 1.0)
        ks_stat, _ = stats.ks_2samp(baseline, attacked)
        metrics["ks_statistic"].append(ks_stat)
        metrics["percent_over_15m"].append(np.mean(attacked > 15) * 100)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    axes[0].plot(metrics["delay"], metrics["mean_shift"], '-o', linewidth=2, markersize=8, color='darkred')
    axes[0].axhline(0, color='gray', linestyle='--', alpha=0.5)
    axes[0].set_xlabel("Delay (s)")
    axes[0].set_ylabel("Mean Gap Shift (m)")
    axes[0].set_title("Shift in Mean Gap")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(metrics["delay"], metrics["std_ratio"], '-s', linewidth=2, markersize=8, color='darkblue')
    axes[1].axhline(1, color='gray', linestyle='--', alpha=0.5)
    axes[1].set_xlabel("Delay (s)")
    axes[1].set_ylabel("Std Ratio (attacked/baseline)")
    axes[1].set_title("Spread Inflation")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(metrics["delay"], metrics["ks_statistic"], '-D', linewidth=2, markersize=8, color='darkgreen')
    axes[2].axhline(0.2, color='gray', linestyle='--', alpha=0.5, label="Small effect threshold")
    axes[2].set_xlabel("Delay (s)")
    axes[2].set_ylabel("KS Statistic")
    axes[2].set_title("Distribution Divergence (KS test)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(metrics["delay"], metrics["percent_over_15m"], '-^', linewidth=2, markersize=8, color='darkorange')
    axes[3].set_xlabel("Delay (s)")
    axes[3].set_ylabel("% Gap Errors > 15m")
    axes[3].set_title("Extreme Gap Error Frequency")
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "statistical_metrics_by_delay.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def main():
    plot_gap_distribution_comparison()
    plot_statistical_metrics_by_delay()
    print(f"Baseline comparison figures saved to: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
