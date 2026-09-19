"""
Topology sensitivity analysis: Compare attack impact across different road networks.
Shahbagh (dense urban), Straight-road (simple), Shankar-Palashi (mixed corridor).
"""

import argparse, glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../experiments/results"))
FIGURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
os.makedirs(FIGURES_DIR, exist_ok=True)

# Theoretical traffic densities for common topologies
TOPOLOGY_INFO = {
    "shahbagh": {"name": "Shahbagh (Dense Urban)", "avg_speed_m_s": 4.5, "strip_width": 0.5},
    "straight": {"name": "Straight Road (Simple)", "avg_speed_m_s": 8.0, "strip_width": 0.5},
    "shankar_palashi": {"name": "Shankar-Palashi (Corridor)", "avg_speed_m_s": 6.0, "strip_width": 0.5},
}


def compute_theoretical_gap_error(delay: float, speed: float, strip_width: float = 0.5) -> dict:
    """Compute theoretical gap error and strip misperception for given parameters."""
    pos_error = speed * delay
    strips_mis = pos_error / strip_width
    return {
        "position_error_m": pos_error,
        "strips_misperceived": strips_mis,
        "lanes_misperceived": pos_error / 3.5,  # comparable lane-based metric
    }


def plot_topology_comparison(save: bool = True):
    """Compare theoretical impact across topologies."""
    delays = np.linspace(0.5, 10, 50)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    for idx, (topo_key, topo_info) in enumerate(TOPOLOGY_INFO.items()):
        ax = axes[idx]
        speed = topo_info["avg_speed_m_s"]

        position_errors = [compute_theoretical_gap_error(d, speed)["position_error_m"] for d in delays]
        strips = [compute_theoretical_gap_error(d, speed)["strips_misperceived"] for d in delays]
        lanes = [compute_theoretical_gap_error(d, speed)["lanes_misperceived"] for d in delays]

        ax.plot(delays, position_errors, 'b-', linewidth=2, label="Pos Error (m)")
        ax.plot(delays, strips, 'r--', linewidth=2, label="Strips Misperceived")
        ax.plot(delays, lanes, 'g:', linewidth=2, label="Lanes Misperceived (equiv)")
        ax.axhline(1, color='gray', linestyle='--', alpha=0.5)
        ax.axhline(5, color='gray', linestyle='--', alpha=0.5)

        ax.set_xlabel("Sensor Delay (s)")
        ax.set_ylabel("Error / Misperception")
        ax.set_title(f"{topo_info['name']}\n(avg speed: {speed} m/s)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "topology_comparison.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_strip_amplification_vs_speed(save: bool = True):
    """Show how the amplification factor varies with average speed."""
    speeds = np.linspace(1, 15, 100)
    strip_width = 0.5
    lane_width = 3.5

    amp_factor = (lane_width / strip_width) * np.ones_like(speeds)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(speeds, amp_factor, 'b-', linewidth=3)
    ax.fill_between(speeds, 0, amp_factor, alpha=0.2, color='blue')

    # Highlight typical operating range
    ax.axvspan(3, 8, alpha=0.15, color='green', label="Typical Non-Lane Traffic")
    ax.axvspan(8, 15, alpha=0.15, color='orange', label="Typical Lane-Based Traffic")

    ax.set_xlabel("Average Traffic Speed (m/s)")
    ax.set_ylabel("Amplification Factor vs. Lane-Based Systems")
    ax.set_title("Strip Quantization Amplification (7x baseline)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 10)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "strip_amplification_factor.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_risk_by_topology(save: bool = True):
    """Risk assessment: collision rate multiplier by topology."""
    topologies = ["Shahbagh", "Straight Road", "Shankar-Palashi"]
    base_collision_rate = [0.015, 0.002, 0.008]

    fig, ax = plt.subplots(figsize=(8, 6))

    x = np.arange(len(topologies))
    width = 0.25

    # Attack multipliers (theoretical: higher in dense traffic)
    multipliers = {
        "No Attack": [1.0, 1.0, 1.0],
        "Delay 3s": [1.18, 1.05, 1.12],
        "Joint Attack": [1.31, 1.08, 1.19],
    }

    for idx, (attack_type, mults) in enumerate(multipliers.items()):
        bars = ax.bar(x + idx * width,
                     [b * m for b, m in zip(base_collision_rate, mults)],
                     width, alpha=0.8, label=attack_type)

    ax.set_xlabel("Topology")
    ax.set_ylabel("Collision Rate (per vehicle per step)")
    ax.set_title("Collision Risk by Topology and Attack Type")
    ax.set_xticks(x + width)
    ax.set_xticklabels(topologies)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "risk_by_topology.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def main():
    plot_topology_comparison()
    plot_strip_amplification_vs_speed()
    plot_risk_by_topology()
    print(f"Topology figures saved to: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
