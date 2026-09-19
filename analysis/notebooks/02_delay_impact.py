"""
Delay impact analysis: How increasing delay magnitude affects traffic metrics.
Shows the monotonic relationship between sensor delay and performance degradation.
"""

import argparse, csv, glob, os, sys
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

def load_results(results_path: str) -> pd.DataFrame:
    return pd.read_csv(results_path)

def plot_delay_vs_speed(df: pd.DataFrame, save: bool = True):
    """Figure: How increasing delay reduces average speed."""
    if df.empty or "attack_scenario" not in df.columns:
        return
    df_sc0 = df[df["attack_scenario"] == 0].copy()

    fig, ax = plt.subplots(figsize=(10, 6))

    if "delay_mean" in df_sc0.columns:
        for delay in sorted(df_sc0["delay_mean"].dropna().unique()):
            subset = df_sc0[df_sc0["delay_mean"] == delay]
            if subset.empty:
                continue
            speeds = subset["avg_speed"].dropna()
            if not speeds.empty:
                ax.errorbar(delay, speeds.mean(), yerr=speeds.std(),
                           fmt='o', capsize=4, markersize=10, capthick=2)

    if not df_sc0.empty:
        delays = sorted(df_sc0["delay_mean"].dropna().unique())
        means = [df_sc0[df_sc0["delay_mean"] == d]["avg_speed"].mean() for d in delays]
        ax.plot(delays, means, 'r--', alpha=0.5, label="Trend")

    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Average Speed (m/s)")
    ax.set_title("Traffic Speed Degradation vs. Sensor Delay")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "delay_vs_speed.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()

def plot_delay_vs_waiting_time(df: pd.DataFrame, save: bool = True):
    """Figure: Waiting time increases with delay."""
    if df.empty or "attack_scenario" not in df.columns:
        return
    df_sc0 = df[df["attack_scenario"] == 0].copy()

    fig, ax = plt.subplots(figsize=(10, 6))

    if "delay_mean" in df_sc0.columns:
        for delay in sorted(df_sc0["delay_mean"].dropna().unique()):
            subset = df_sc0[df_sc0["delay_mean"] == delay]
            if subset.empty:
                continue
            wt = subset["avg_waiting_time"].dropna()
            if not wt.empty:
                ax.errorbar(delay, wt.mean(), yerr=wt.std(),
                           fmt='s', capsize=4, markersize=10, capthick=2, color='red')

    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Average Waiting Time (seconds)")
    ax.set_title("Waiting Time Increase vs. Sensor Delay")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "delay_vs_waiting.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()

def plot_delay_vs_gap_error(df: pd.DataFrame, save: bool = True):
    """Figure: Gap error increases linearly with delay."""
    if df.empty or "attack_scenario" not in df.columns:
        return
    df_sc0 = df[df["attack_scenario"] == 0].copy()

    fig, ax = plt.subplots(figsize=(10, 6))

    if "delay_mean" in df_sc0.columns and "gap_error_mean_abs" in df_sc0.columns:
        delays = []
        errors = []
        err_stds = []
        for delay in sorted(df_sc0["delay_mean"].dropna().unique()):
            subset = df_sc0[df_sc0["delay_mean"] == delay]
            ge = subset["gap_error_mean_abs"].dropna()
            if not ge.empty:
                delays.append(delay)
                errors.append(ge.mean())
                err_stds.append(ge.std())
        ax.errorbar(delays, errors, yerr=err_stds, fmt='D-',
                   capsize=4, markersize=8, capthick=2, linewidth=2,
                   color='darkblue', label="Measured")

        # Theoretical line: error = speed * delay  (at typical speed ~5 m/s)
        if delays:
            theo = [5.0 * d for d in delays]
            ax.plot(delays, theo, 'r--', linewidth=2, alpha=0.7, label="Theoretical (v=5 m/s)")

    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Mean Absolute Gap Error (m)")
    ax.set_title("Gap Error Growth with Sensor Delay")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "delay_vs_gap_error.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()

def plot_delay_heatmap(df: pd.DataFrame, save: bool = True):
    """Heatmap: 2D interaction of delay and other parameters."""
    if "delay_mean" not in df.columns or "gap_error_mean_abs" not in df.columns:
        print("Insufficient data for heatmap")
        return

    pivot = df.pivot_table(
        values="gap_error_mean_abs",
        index="delay_mean",
        columns="attack_scenario",
        aggfunc="mean"
    )

    fig, ax = plt.subplots(figsize=(10, 6))
    im = ax.imshow(pivot.values, cmap="YlOrRd", aspect="auto")

    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"Sc{int(c)}" for c in pivot.columns])
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{d:.1f}s" for d in pivot.index])

    for i in range(len(pivot.index)):
        for j in range(len(pivot.columns)):
            val = pivot.values[i, j]
            if not np.isnan(val):
                ax.text(j, i, f"{val:.1f}", ha="center", va="center",
                       color="white" if val > pivot.values.max() / 2 else "black",
                       fontsize=9)

    ax.set_xlabel("Attack Scenario")
    ax.set_ylabel("Delay (seconds)")
    ax.set_title("Gap Error Magnitude: Delay vs. Scenario")

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "delay_scenario_heatmap.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=str, default=None)
    args = parser.parse_args()

    if args.results is None:
        run_dirs = sorted(glob.glob(os.path.join(RESULTS_DIR, "run_*")))
        if run_dirs:
            args.results = os.path.join(run_dirs[-1], "results_summary.csv")

    if args.results and os.path.exists(args.results):
        df = load_results(args.results)
        print(f"Loaded {len(df)} data points from {args.results}")
    else:
        print("No results found. Use --synthetic for demo or run experiments first.")
        df = pd.DataFrame()

    plot_delay_vs_speed(df)
    plot_delay_vs_waiting_time(df)
    plot_delay_vs_gap_error(df)
    if not df.empty:
        plot_delay_heatmap(df)

    print(f"Figures saved to: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
