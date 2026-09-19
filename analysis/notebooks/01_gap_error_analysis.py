"""
Gap error analysis script.
Reads experiment results and generates key figures for the project.
"""

import argparse
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

sns.set_theme(style="whitegrid")

RESULTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../experiments/results"))
FIGURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
os.makedirs(FIGURES_DIR, exist_ok=True)


def load_data(results_path: str) -> pd.DataFrame:
    """Load results summary CSV into a DataFrame."""
    return pd.read_csv(results_path)


def generate_synthetic_results(n_configs: int = 30) -> pd.DataFrame:
    """Generate synthetic experiment results for testing without real experiments."""
    np.random.seed(42)
    rows = []
    for i in range(n_configs):
        sc = np.random.randint(0, 6)
        delay = np.random.choice([0.5, 1.0, 2.0, 3.0, 5.0])
        rows.append({
            "config_id": f"syn_{i}",
            "attack_scenario": sc,
            "delay_mean": delay,
            "jitter_magnitude": np.random.uniform(0, 2),
            "gap_error_mean_abs": np.abs(np.random.normal(delay * 2.5, delay * 0.5)),
            "gap_error_mean": np.random.normal(delay * 1.5, delay * 0.3),
            "gap_error_std": np.random.uniform(0.5, 3),
            "avg_speed": np.random.normal(5.0 - 0.3 * delay * (sc > 0), 0.5),
            "avg_waiting_time": np.random.exponential(10 + delay * 3),
            "collision_count": int(np.random.poisson(delay * 0.5)),
        })
    return pd.DataFrame(rows)


def plot_gap_error_vs_delay(df: pd.DataFrame, save: bool = True):
    """Figure 1: Gap error magnitude vs delay amount for persistent delay scenario."""
    if df.empty or "attack_scenario" not in df.columns:
        print("  Skipping gap vs delay (no data)")
        return
    if "gap_error_mean_abs" not in df.columns:
        print("  Skipping gap vs delay (no gap_error_mean_abs column)")
        return
    df_sc0 = df[df["attack_scenario"] == 0].copy()
    if "delay_mean" not in df_sc0.columns:
        print("  Skipping gap vs delay (no delay_mean column)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    for delay in sorted(df_sc0["delay_mean"].dropna().unique()):
        subset = df_sc0[df_sc0["delay_mean"] == delay]
        ax.scatter([delay] * len(subset), subset["gap_error_mean_abs"], alpha=0.6, s=30)
    ax.set_xlabel("Delay Mean (seconds)")
    ax.set_ylabel("Mean Absolute Gap Error (m)")
    ax.set_title("Gap Error vs. Sensor Delay (Persistent Attack)")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    grouped = df_sc0.groupby("delay_mean")["gap_error_mean_abs"].agg(["mean", "std", "count"])
    if not grouped.empty:
        delays = grouped.index.values
        means = grouped["mean"].values
        errors = grouped["std"].values
        ax.errorbar(delays, means, yerr=errors, fmt="-o", capsize=4, markersize=8)
        ax.set_xlabel("Delay Mean (seconds)")
        ax.set_ylabel("Mean Absolute Gap Error (m)")
        ax.set_title("Aggregated: Gap Error vs. Delay")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "gap_error_vs_delay.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_gap_error_by_scenario(df: pd.DataFrame, save: bool = True):
    """Figure 2: Gap error CDF comparison across attack scenarios."""
    if df.empty or "attack_scenario" not in df.columns:
        print("  Skipping gap by scenario (no data)")
        return
    scenarios = {
        0: "Persistent Delay",
        1: "Intermittent Faults",
        2: "Single-Shot",
        3: "Jitter-Only",
        4: "Packet Drop",
        5: "Joint Multi-Sensor",
    }

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    if "gap_error_mean_abs" not in df.columns:
        print("  Skipping by scenario (no gap_error_mean_abs)")
        return
    for sc_id, sc_name in scenarios.items():
        subset = df[df["attack_scenario"] == sc_id]
        if subset.empty:
            continue
        mean_val = subset["gap_error_mean_abs"].dropna().mean()
        ax.bar(sc_id, mean_val, label=sc_name, alpha=0.7)
    ax.set_xlabel("Attack Scenario")
    ax.set_ylabel("Mean Absolute Gap Error (m)")
    ax.set_title("Attack Scenario Comparison")
    ax.set_xticks(list(scenarios.keys()))
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    for sc_id, sc_name in scenarios.items():
        subset = df[df["attack_scenario"] == sc_id]
        if subset.empty or "gap_error_mean_abs" not in subset.columns:
            continue
        values = subset["gap_error_mean_abs"].dropna().values
        if len(values) == 0:
            continue
        sorted_vals = np.sort(values)
        cdf = np.arange(1, len(sorted_vals) + 1) / len(sorted_vals)
        ax.plot(sorted_vals, cdf, label=sc_name, linewidth=2)
    ax.set_xlabel("Mean Absolute Gap Error (m)")
    ax.set_ylabel("CDF")
    ax.set_title("CDF of Gap Errors by Attack Scenario")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "gap_error_by_scenario.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_strip_quantization_effect(save: bool = True):
    """Figure 3: Theoretical plot of strip quantization amplifying timing errors."""
    strip_widths = [0.3, 0.5, 0.8, 1.0]
    delays = np.linspace(0, 5, 50)
    speeds = [2, 4, 6, 8]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    for sw in strip_widths:
        errors = []
        for d in delays:
            pos_error = 4 * d
            strips_misperceived = pos_error / sw
            errors.append(strips_misperceived)
        ax.plot(delays, errors, label=f"strip={sw}m", linewidth=2)
    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Mis-perceived Strips")
    ax.set_title("Strip Quantification of Timing Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    for v in speeds:
        errors = v * delays / 0.5
        ax.plot(delays, errors, label=f"speed={v} m/s", linewidth=2)
    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Mis-perceived Strips (strip=0.5m)")
    ax.set_title("Speed Amplification Effect")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "strip_quantization.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_speed_impact(df: pd.DataFrame, save: bool = True):
    """Figure 4: Impact of attacks on average speed."""
    if df.empty or "attack_scenario" not in df.columns:
        print("  Skipping speed impact (no data)")
        return
    scenarios = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
                 3: "Jitter", 4: "PacketDrop", 5: "Joint", -1: "Baseline (No Attack)"}

    fig, ax = plt.subplots(figsize=(10, 6))

    baseline_speed = None
    if not df.empty and "attack_scenario" in df.columns:
        baseline_subset = df[df["attack_scenario"].isna()]
        if not baseline_subset.empty:
            baseline_speed = baseline_subset["avg_speed"].mean()
    if baseline_speed is None and not df.empty and "config_id" in df.columns and "baseline" in df["config_id"].values:
        baseline_speed = df[df["config_id"] == "baseline"]["avg_speed"].mean()

    for sc_id, sc_name in scenarios.items():
        if sc_id == -1:
            continue
        subset = df[df["attack_scenario"] == sc_id]
        if subset.empty:
            continue
        speeds = subset["avg_speed"].dropna()
        if speeds.empty:
            continue
        ax.bar(sc_id, speeds.mean(), yerr=speeds.std(), capsize=4, alpha=0.7, label=sc_name)

    if baseline_speed:
        ax.axhline(baseline_speed, color="red", linestyle="--", linewidth=2, label=f"Baseline={baseline_speed:.2f}")

    ax.set_xlabel("Attack Scenario")
    ax.set_ylabel("Average Speed (m/s)")
    ax.set_title("Impact of Temporal Desync Attacks on Traffic Speed")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "speed_impact.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=str, default=None, help="Path to results_summary.csv")
    parser.add_argument("--synthetic", action="store_true", help="Use synthetic data instead of real experiments")
    args = parser.parse_args()

    if args.synthetic:
        print("Using synthetic data for demonstration")
        df = generate_synthetic_results(30)
        plot_gap_error_vs_delay(df)
        plot_gap_error_by_scenario(df)
        plot_strip_quantization_effect()
        plot_speed_impact(df)
        print(f"\nAll figures saved to: {FIGURES_DIR}")
        return

    # Find most recent results file if not specified
    if args.results is None:
        run_dirs = sorted(glob.glob(os.path.join(RESULTS_DIR, "run_*")))
        if not run_dirs:
            print("No experiment results. Use --synthetic for demo mode, or run experiments first.")
            plot_strip_quantization_effect()
            return
        latest = run_dirs[-1]
        args.results = os.path.join(latest, "results_summary.csv")
        print(f"Using latest results: {args.results}")

    if not os.path.exists(args.results):
        print(f"Results file not found: {args.results}")
        plot_strip_quantization_effect()
        return

    df = load_data(args.results)
    print(f"Loaded {len(df)} data points")

    plot_gap_error_vs_delay(df)
    plot_gap_error_by_scenario(df)
    plot_strip_quantization_effect()
    plot_speed_impact(df)

    print(f"\nAll figures saved to: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
