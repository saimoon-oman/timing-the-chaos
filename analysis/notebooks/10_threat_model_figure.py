"""
10_threat_model_figure.py - Generate the threat model figure (attack-only)
for the project (Figure 1) and presentation slides.

The figure shows the end-to-end attack chain for temporal sensor
desynchronization in non-lane-based traffic:

    Physical world (trusted)  --V2V-->  Adversary (network MITM)
        |                              (6 attack scenarios)
        |--radar (fresh, not attacked)------>|     |
        v                                     v    v
    Follower perception (victim)      stale leader position x(t-tau)
        | perceived gap error eps ~= -v_leader * tau, strip ~7x
        v
    Consequences: car-following -> braking/speed -> safety (TTC)

The same layout is available as a TikZ drawing for vector output;
this script renders it for slides and as a raster fallback.

Outputs:
    analysis/figures/threat_model.png        (300 dpi)

Usage:
    python analysis/notebooks/10_threat_model_figure.py
"""

import argparse
import os
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, FancyArrowPatch
from matplotlib.path import Path

HERE = os.path.abspath(os.path.dirname(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
FIG_DIR = os.path.join(PROJECT, "analysis", "figures")

# Colors (match TikZ zones)
BLUE = "#1f77b4"
RED = "#d62728"
VIOLET = "#9467bd"
ORANGE = "#ff7f0e"
GREEN = "#2ca02c"

CM_TO_IN = 1 / 2.54
W, H = 17.35 + 0.3, 9.15 + 0.25  # canvas in "cm" units


def zone(ax, x0, y0, x1, y1, color, dashed=False):
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.08", linewidth=1.2,
        edgecolor=color, facecolor=color + "14",
        linestyle="--" if dashed else "-"))


def box(ax, x0, y0, x1, y1, color, lw=1.1, fill="#ffffff"):
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.05", linewidth=lw,
        edgecolor=color, facecolor=fill))


def center_text(ax, x, y, s, size=7.5, color="black", weight="normal",
                ha="center"):
    ax.text(x, y, s, ha=ha, va="center", fontsize=size, color=color,
            weight=weight)


def draw(ax):
    # ---------------- Zones ----------------
    zone(ax, 0.15, 4.9, 4.75, 9.15, BLUE)
    zone(ax, 5.3, 5.55, 12.3, 9.15, RED, dashed=True)
    zone(ax, 12.8, 4.9, 17.35, 9.15, VIOLET)
    zone(ax, 0.15, 1.15, 17.35, 4.4, ORANGE)

    center_text(ax, 2.45, 9.29, "Physical World (Trusted)", 8.5, BLUE, "bold")
    center_text(ax, 8.8, 9.29, "Adversary — MITM on V2V Channel", 8.5, RED, "bold")
    center_text(ax, 15.075, 9.29, "Follower (Victim)", 8.5, VIOLET, "bold")
    center_text(ax, 8.75, 4.53, "Consequences: Perception Error → Control and Safety",
                8.5, ORANGE, "bold")

    # ---------------- V2V channel (through adversary) ----------------
    ax.plot([4.65, 5.3], [8.35, 8.35], color="black", lw=1.2)
    ax.plot([5.3, 12.3], [8.35, 8.35], color="black", lw=1.2)
    ax.add_patch(FancyArrowPatch((12.3, 8.35), (13.0, 8.35),
                                 arrowstyle="-|>", mutation_scale=10,
                                 color="black", lw=1.2))
    center_text(ax, 4.98, 8.52, "x(t)", 6.5)
    center_text(ax, 12.62, 8.52, "x(t−τ) stale", 6.5)

    # ---------------- Radar channel (fresh, below adversary) ----------------
    path = Path([(4.65, 6.6), (5.2, 6.6), (5.2, 5.0), (12.4, 5.0),
                 (12.4, 6.6), (13.0, 6.6)])
    ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=10,
                                 color=GREEN, lw=1.4, linestyle=(0, (4, 3))))
    center_text(ax, 8.8, 4.88, "radar (line-of-sight): fresh — not attacked",
                6.5, GREEN)

    # ---------------- Leader vehicle ----------------
    box(ax, 0.3, 6.0, 4.65, 8.7, BLUE)
    center_text(ax, 2.475, 8.05, "Leader Vehicle", 8, BLUE, "bold")
    center_text(ax, 2.475, 7.55, "position x(t), strip s(t)", 7)
    center_text(ax, 2.475, 7.1, "broadcasts via V2V", 7)

    # ---------------- Non-lane-based strip schematic ----------------
    for i in range(4):
        ax.add_patch(Rectangle((0.4 + i * 1.0, 5.15), 0.94, 0.4,
                               facecolor="white", edgecolor=BLUE, lw=1.0))
    center_text(ax, 2.4, 4.98, "non-lane-based road: 0.5 m strips", 6)

    # ---------------- Adversary internals ----------------
    center_text(ax, 8.8, 8.72, "IEEE 802.11p/DSRC · C-V2X", 6.5, RED)
    center_text(ax, 8.8, 7.95, "capabilities: intercept · delay · drop · jitter",
                6.5, RED)
    chips = [("PD", "delay τ"), ("IF", "prob. p_f"), ("SS", "τ_ss @ t_ss"),
             ("JO", "jitter σ_j"), ("PDr", "drop p_d"), ("JM", "jitter+delay")]
    for i, (abbr, param) in enumerate(chips):
        cx = 5.95 + i * 1.14
        box(ax, cx - 0.5, 6.5, cx + 0.5, 7.55, RED, lw=0.8)
        center_text(ax, cx, 7.22, abbr, 6.5, RED, "bold")
        center_text(ax, cx, 6.8, param, 5.5)
    center_text(ax, 8.8, 6.12,
                "selective targeting: strip mod k_strip · window [t_start, t_end]",
                6.5, RED)
    center_text(ax, 8.8, 5.76, "⇒ follower's V2V position is stale: x(t−τ)",
                6.5, RED)

    # ---------------- Follower perception ----------------
    box(ax, 13.0, 6.3, 17.15, 8.7, VIOLET)
    center_text(ax, 15.075, 8.3, "Follower Vehicle", 8, VIOLET, "bold")
    center_text(ax, 15.075, 7.8, "radar: fresh g_r(t)", 7)
    center_text(ax, 15.075, 7.35, "V2V: stale x(t−τ)", 7)
    center_text(ax, 15.075, 6.9, "perceived gap ĝ(t)", 7)
    center_text(ax, 15.075, 6.5, "from x(t−τ) − x_f(t)", 7)

    box(ax, 12.0, 5.83, 17.2, 6.33, VIOLET, lw=0.9)
    center_text(ax, 14.6, 6.08, "ε(t) ≈ −v_leader(t)·τ   [Eq. 5]", 6.5, VIOLET)
    box(ax, 12.4, 5.33, 16.8, 5.83, VIOLET, lw=0.9)
    center_text(ax, 14.6, 5.58, "strip quantization ≈ 7× amplification", 6.5, VIOLET)

    # ---------------- Consequences ----------------
    ax.add_patch(FancyArrowPatch((16.75, 6.3), (16.75, 4.4),
                                 arrowstyle="-|>", mutation_scale=10,
                                 color="black", lw=1.2))
    center_text(ax, 17.4, 5.15, "CF model acts on\nstale gap ĝ(t)", 6,
                ha="right")

    box(ax, 0.4, 1.5, 7.5, 3.8, ORANGE)
    center_text(ax, 3.95, 3.3, "Car-Following Model", 8, ORANGE, "bold")
    center_text(ax, 3.95, 2.85, "Gipps (Hybrid)", 7)
    center_text(ax, 3.95, 2.4, "safe speed computed from ĝ(t)", 7)

    box(ax, 7.7, 1.5, 12.2, 3.8, ORANGE)
    center_text(ax, 9.95, 3.3, "Reaction", 8, ORANGE, "bold")
    center_text(ax, 9.95, 2.85, "premature braking", 7)
    center_text(ax, 9.95, 2.4, "12% speed loss", 7)

    box(ax, 12.4, 1.5, 17.15, 3.8, ORANGE)
    center_text(ax, 14.775, 3.3, "Safety", 8, ORANGE, "bold")
    center_text(ax, 14.775, 2.85, "TTC bias −3.4 s (27%)", 7)
    center_text(ax, 14.775, 2.4, "TTC<2 s: perc. 18.5% vs true 11.6%", 6.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--figdir", default=FIG_DIR)
    args = parser.parse_args()

    os.makedirs(args.figdir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(W * CM_TO_IN, H * CM_TO_IN), dpi=300)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    draw(ax)

    out = os.path.join(args.figdir, "threat_model.png")
    fig.savefig(out, dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
