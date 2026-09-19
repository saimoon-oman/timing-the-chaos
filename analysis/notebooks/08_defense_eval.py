"""
08_defense_eval.py - Evaluate defenses on REAL experiment gap data.

For each per-config gap_errors.csv, applies:
  1. Temporal Median Consensus (sliding window median of perceived gap)
  2. Moving-average baseline
to the per-vehicle-pair perceived gap series and measures MAE vs the
recorded true gap. Produces a per-scenario reduction table + figure.

Usage:
    python analysis/notebooks/08_defense_eval.py --gapdir <run_dir> [--outdir analysis/figures]
"""

import argparse, os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
SCENARIO_NAMES = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
                  3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}


def median_consensus(series, k):
    """Sliding window median (causal, centered on current sample)."""
    out = np.full(len(series), np.nan)
    for i in range(len(series)):
        lo = max(0, i - k + 1)
        out[i] = np.median(series[lo:i + 1])
    return out


def velocity_compensated(perceived, true, leader_speeds, tau_hat, fs=1.0, stale_mask=None):
    """
    Correct the stale perceived position using the leader's HISTORICAL speed
    recorded at the stale step:
        x_hat(t) = x(t - tau) + v_leader(t - tau) * tau_hat
    Applied only to stale samples (stale_mask), leaving fresh samples intact.
    Returns corrected gap series.
    """
    tau_steps = max(1, int(round(tau_hat / fs)))
    n = len(perceived)
    corrected = perceived.copy()
    for i in range(n):
        j = i - tau_steps
        if j < 0:
            continue
        if stale_mask is not None and not stale_mask[i]:
            continue
        corrected[i] = perceived[i] + leader_speeds[j] * tau_hat
    return corrected


def evaluate_config(gap_path: str, k=5, tau_hat=1.0) -> dict:
    df = pd.read_csv(gap_path)
    if df.empty or "perceivedGap" not in df.columns:
        return None
    cid = os.path.basename(os.path.dirname(gap_path))
    is_baseline = cid.startswith("baseline")

    rows = []
    for (_lid, _fid), grp in df.groupby(["leaderId", "followerId"]):
        if len(grp) < 3:
            continue
        g = grp.sort_values("simStep")
        perc = g["perceivedGap"].values
        true = g["trueGap"].values
        lspeed = g["leaderSpeed"].values
        # stale = perceived deviates from true by more than 0.25 m (oracle staleness)
        stale = np.abs(perc - true) > 0.25
        med = median_consensus(perc, k)
        vcomp_all = velocity_compensated(perc, true, lspeed, tau_hat)
        vcomp_stale = velocity_compensated(perc, true, lspeed, tau_hat, stale_mask=stale)
        mae_raw = np.nanmean(np.abs(perc - true))
        mae_med = np.nanmean(np.abs(med - true))
        mae_vc = np.nanmean(np.abs(vcomp_all - true))
        mae_vcs = np.nanmean(np.abs(vcomp_stale - true))
        rows.append({"mae_raw": mae_raw, "mae_median_k%d" % k: mae_med,
                     "mae_vcomp_tau%g" % tau_hat: mae_vc,
                     "mae_vcomp_stale_tau%g" % tau_hat: mae_vcs,
                     "stale_frac": float(np.mean(stale))})
    if not rows:
        return None
    agg = {}
    for key in rows[0]:
        agg[key] = float(np.nanmean([r[key] for r in rows]))
    agg["config_id"] = cid
    agg["is_baseline"] = is_baseline
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gapdir", required=True)
    ap.add_argument("--outdir", default=FIG_DIR)
    ap.add_argument("--window", type=int, default=5)
    ap.add_argument("--tau_hat", type=float, default=1.0,
                    help="delay estimate (s) used by velocity-compensated estimator")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    results = []
    for root, _dirs, files in os.walk(args.gapdir):
        if "gap_errors.csv" not in files:
            continue
        r = evaluate_config(os.path.join(root, "gap_errors.csv"), k=args.window, tau_hat=args.tau_hat)
        if r:
            results.append(r)
    df = pd.DataFrame(results)
    if df.empty:
        print("No data found"); return

    def scenario_of(cid):
        import re
        if cid.startswith("baseline"):
            return -1
        m = re.match(r"sc(\d)", cid)
        return int(m.group(1)) if m else -2

    df["scenario"] = df["config_id"].apply(scenario_of)
    kcol = "mae_median_k%d" % args.window
    vcol = "mae_vcomp_tau%g" % args.tau_hat
    vscol = "mae_vcomp_stale_tau%g" % args.tau_hat
    df["reduction_median"] = 1 - df[kcol] / df["mae_raw"].replace(0, np.nan)
    df["reduction_vcomp"] = 1 - df[vcol] / df["mae_raw"].replace(0, np.nan)
    df["reduction_vcomp_stale"] = 1 - df[vscol] / df["mae_raw"].replace(0, np.nan)

    out_path = os.path.join(args.gapdir, "defense_eval_summary.csv")
    df.to_csv(out_path, index=False)
    print(f"Saved: {out_path}")

    print(f"\nDefense evaluation on REAL per-vehicle gap series (tau_hat={args.tau_hat}s):")
    print(f"{'Scenario':<15} {'MAE raw':>9} {'MAE median':>11} {'Med red.':>9} {'MAE vcomp':>10} {'VC red.':>8} {'VC stale':>10}")
    rows = []
    for sc in sorted(df["scenario"].unique()):
        if sc < 0:
            continue
        sub = df[df["scenario"] == sc]
        mae_raw = sub["mae_raw"].mean()
        mae_med = sub[kcol].mean()
        mae_vc = sub[vcol].mean()
        mae_vcs = sub[vscol].mean()
        red_med = 100 * (1 - mae_med / mae_raw)
        red_vc = 100 * (1 - mae_vc / mae_raw)
        red_vcs = 100 * (1 - mae_vcs / mae_raw)
        rows.append((sc, mae_raw, mae_med, red_med, mae_vc, red_vc, mae_vcs, red_vcs))
        print(f"{SCENARIO_NAMES.get(sc, sc):<15} {mae_raw:9.3f} {mae_med:11.3f} {red_med:8.1f}% {mae_vc:10.3f} {red_vc:7.1f}% {mae_vcs:10.3f}")
    base = df[df["scenario"] == -1]
    if not base.empty:
        print(f"{'Baseline (no att)':<15} {base['mae_raw'].mean():9.3f}")

    # Figure: reduction by scenario (median vs velocity-compensated stale-gated)
    fig, ax = plt.subplots(figsize=(9, 5))
    labels = [SCENARIO_NAMES.get(r[0], str(r[0])) for r in rows]
    x = np.arange(len(rows)); w = 0.38
    ax.bar(x - w / 2, [r[3] for r in rows], w, label="Median consensus (k=%d)" % args.window,
           color="#dc2626", edgecolor="black", linewidth=0.5)
    ax.bar(x + w / 2, [r[7] for r in rows], w, label="Velocity-compensated, stale-gated (τ̂=%g s)" % args.tau_hat,
           color="#0f766e", edgecolor="black", linewidth=0.5)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=15)
    ax.set_ylabel("Gap error change (%)")
    ax.set_title("Defenses on Real Experiment Data (negative = worse)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, "defense_real_comparison.png"), dpi=200)
    plt.close(fig)
    print("saved defense_real_comparison.png")


if __name__ == "__main__":
    main()
