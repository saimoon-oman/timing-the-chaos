"""
09_tau_eval.py - Evaluate the causal radar-discrepancy delay estimator
against oracle and fixed estimates on REAL experiment gap data.

Attack model (DhakaSim getDxTemporalDesync): the attacker delays V2V
position updates of the IMMEDIATE leader only (leaderNo == 1). The
forward radar (min-of-N) is an independent, uncompromised channel that
reports the fresh physical range. Comparing the delayed V2V gap with
the fresh radar range reveals the timing offset tau:

    tau_hat(t) = (g_radar(t) - g_hat(t)) / v_leader(t - tau_hat)

used by the velocity-compensated correction (Eq. 11):
    g_corr(t) = g_hat(t) + v_leader(t - tau_hat) * tau_hat
applied only to stale samples (|g_hat - g_radar| > 0.25 m).

Candidates compared:
  - raw perceived gap (no defense)
  - fixed tau_hat = 1 s (baseline)
  - oracle tau (true per-config delay; upper bound)
  - causal estimator (radar discrepancy, EMA + trailing median)

Radar model: min of 3 independent Gaussian readings (sigma per-reading)
around the true gap -- matching DhakaSim's min-of-N radar model.
The true gap is the physical range a radar measures; using it as the
radar reference is NOT circular because the attack only delays the V2V
packets (network-level), which cannot touch the radar channel.

Usage:
    python analysis/notebooks/09_tau_eval.py --gapdir <run_dir>
"""

import argparse
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                "../../defenses/schmidt_kalman")))
from tau_estimator import estimate_tau_series

FIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
SCENARIO_NAMES = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
                  3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}
RADAR_SIGMA = 0.3      # per-reading radar noise (m)
RADAR_N = 3            # min-of-N radar readings
STALE_THRESH = 0.25    # stale-sample gate (m)


def velocity_compensated_stale(perceived, leader_speeds, tau_series, stale):
    """Stale-gated vcomp with a per-sample tau_hat series (Eq. 11)."""
    n = len(perceived)
    corrected = perceived.copy()
    for i in range(n):
        ts = max(1, int(round(float(tau_series[i]))))
        j = i - ts
        if j < 0 or not stale[i]:
            continue
        corrected[i] = perceived[i] + leader_speeds[j] * tau_series[i]
    return corrected


def evaluate_config(gap_path, oracle_tau=None, seed=7) -> dict:
    df = pd.read_csv(gap_path)
    if df.empty or "perceivedGap" not in df.columns:
        return None
    cid = os.path.basename(os.path.dirname(gap_path))
    is_baseline = cid.startswith("baseline")
    rng = np.random.default_rng(seed)

    rows = []
    for (_lid, _fid), grp in df.groupby(["leaderId", "followerId"]):
        if len(grp) < 3:
            continue
        g = grp.sort_values("simStep")
        perc = g["perceivedGap"].values.astype(float)
        true = g["trueGap"].values.astype(float)
        lspeed = g["leaderSpeed"].values.astype(float)
        n = len(perc)

        # radar reference: min-of-N readings around the physical range
        radar = np.array([min(rng.normal(true[i], RADAR_SIGMA, RADAR_N))
                          for i in range(n)])
        # staleness gate = detection stage output (perfect detector, as in
        # 08_defense_eval: perceived deviates from truth by > 0.25 m).
        # Isolates the effect of the DELAY ESTIMATE on the correction.
        stale = np.abs(perc - true) > STALE_THRESH

        tau_est = estimate_tau_series(perc, lspeed, radar)
        mae_raw = np.nanmean(np.abs(perc - true))
        mae_fixed = np.nanmean(np.abs(
            velocity_compensated_stale(perc, lspeed, np.full(n, 1.0), stale) - true))
        mae_oracle = (np.nanmean(np.abs(
            velocity_compensated_stale(perc, lspeed, np.full(n, oracle_tau), stale) - true))
            if oracle_tau is not None else np.nan)
        mae_est = np.nanmean(np.abs(
            velocity_compensated_stale(perc, lspeed, tau_est, stale) - true))

        # estimator accuracy on stale (attacked) samples only
        est_stale = tau_est[stale]
        rows.append({
            "mae_raw": mae_raw,
            "mae_vcomp_fixed1s": mae_fixed,
            "mae_vcomp_oracle": mae_oracle,
            "mae_vcomp_estimated": mae_est,
            "tau_true": oracle_tau if oracle_tau is not None else np.nan,
            "tau_hat_median": float(np.nanmedian(est_stale))
                              if len(est_stale) else np.nan,
            "tau_hat_rmse": float(np.sqrt(np.nanmean((est_stale - oracle_tau) ** 2)))
                            if oracle_tau is not None and len(est_stale) else np.nan,
        })
    if not rows:
        return None
    agg = {k: float(np.nanmean([r[k] for r in rows])) for k in rows[0]}
    agg["config_id"] = cid
    agg["is_baseline"] = is_baseline
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gapdir", required=True)
    ap.add_argument("--outdir", default=FIG_DIR)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    # per-config oracle delay (persistent attacks only)
    delays = {"sc0_delay0.5": 0.5, "sc0_delay1.0": 1.0, "sc0_delay3.0": 3.0,
              "sc0_delay5.0": 5.0, "sc0_delay7.0": 7.0, "sc0_delay10.0": 10.0}

    results = []
    for root, _dirs, files in os.walk(args.gapdir):
        if "gap_errors.csv" not in files:
            continue
        cid = os.path.basename(root)
        oracle = None
        for prefix, d in delays.items():
            if cid.startswith(prefix):
                oracle = d
                break
        r = evaluate_config(os.path.join(root, "gap_errors.csv"), oracle_tau=oracle)
        if r:
            results.append(r)
    df = pd.DataFrame(results)
    if df.empty:
        print("No data found")
        return

    def scenario_of(cid):
        if cid.startswith("baseline"):
            return -1
        m = re.match(r"sc(\d)", cid)
        return int(m.group(1)) if m else -2

    df["scenario"] = df["config_id"].apply(scenario_of)
    df["delay"] = df["config_id"].str.extract(r"delay([\d.]+)")[0].astype(float)

    for col in ["mae_vcomp_fixed1s", "mae_vcomp_oracle", "mae_vcomp_estimated"]:
        df["red_" + col] = 1 - df[col] / df["mae_raw"].replace(0, np.nan)

    out_path = os.path.join(args.gapdir, "tau_estimation_summary.csv")
    df.to_csv(out_path, index=False)
    print(f"Saved: {out_path}")

    print("\nVelocity-compensated correction with delay estimator "
          f"(radar sigma={RADAR_SIGMA}m, min-of-{RADAR_N}):")
    print(f"{'Scenario':<14}{'raw':>7}{'fix1s':>8}{'oracle':>8}{'est':>8}"
          f"{'%fix':>7}{'%orc':>7}{'%est':>7}")

    rows = []
    for sc in sorted(x for x in df["scenario"].unique() if x >= 0):
        sub = df[df["scenario"] == sc]
        r = sub["mae_raw"].mean()
        f = sub["mae_vcomp_fixed1s"].mean()
        o = sub["mae_vcomp_oracle"].mean()
        e = sub["mae_vcomp_estimated"].mean()
        rf, ro, r_est = (100 * (1 - f / r), 100 * (1 - o / r), 100 * (1 - e / r))
        rows.append((sc, r, f, o, e, rf, ro, r_est))
        print(f"{SCENARIO_NAMES.get(sc, sc):<14}{r:7.2f}{f:8.2f}{o:8.2f}{e:8.2f}"
              f"{rf:7.0f}%{ro:7.0f}%{r_est:7.0f}%")

    # persistent-delay detail
    sub = df[(df["scenario"] == 0) & ~df["is_baseline"]]
    if not sub.empty:
        print("\nPersistent delay detail (causal estimator vs upper bounds):")
        print(f"{'Delay':>6}{'raw':>7}{'oracle':>8}{'est':>8}{'%orc':>7}{'%est':>7}"
              f"{'tau_rmse':>9}{'tau_med':>8}")
        for d in sorted(sub["delay"].unique()):
            s = sub[sub["delay"] == d]
            r = s["mae_raw"].mean()
            o = s["mae_vcomp_oracle"].mean()
            e = s["mae_vcomp_estimated"].mean()
            print(f"{d:6.1f}{r:7.2f}{o:8.2f}{e:8.2f}{100*(1-o/r):7.0f}%"
                  f"{100*(1-e/r):7.0f}%{s['tau_hat_rmse'].mean():9.2f}"
                  f"{s['tau_hat_median'].mean():8.2f}")

    # Figure: MAE reduction by scenario (3 bars) + estimator accuracy inset
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    labels = [SCENARIO_NAMES.get(r[0], str(r[0])) for r in rows]
    x = np.arange(len(rows)); w = 0.26
    ax1.bar(x - w, [r[5] for r in rows], w, label="Fixed τ̂=1 s",
            color="#d97706", edgecolor="black", linewidth=0.5)
    ax1.bar(x, [r[6] for r in rows], w, label="Oracle τ (upper bound)",
            color="#2563eb", edgecolor="black", linewidth=0.5)
    ax1.bar(x + w, [r[7] for r in rows], w, label="Causal radar τ̂",
            color="#0f766e", edgecolor="black", linewidth=0.5)
    ax1.axhline(0, color="black", lw=0.8)
    ax1.set_xticks(x); ax1.set_xticklabels(labels, rotation=15)
    ax1.set_ylabel("Gap error reduction (%)")
    ax1.set_title("vcomp correction: fixed vs oracle vs causal τ̂")
    ax1.legend(fontsize=9)
    ax1.grid(alpha=0.3, axis="y")

    pers = sub[~sub["tau_true"].isna()] if "sub" in dir() and not sub.empty else \
        df[~df["tau_true"].isna()]
    if not pers.empty:
        ax2.scatter(pers["tau_true"], pers["tau_hat_median"], s=25,
                    color="#0f766e", alpha=0.7)
        lim = [0, max(11, pers["tau_true"].max() + 1, pers["tau_hat_median"].max() + 1)]
        ax2.plot(lim, lim, "k--", lw=1, label="Ideal")
        ax2.set_xlabel("True delay τ (s)"); ax2.set_ylabel("Estimated τ̂ (median, s)")
        ax2.set_title("Causal estimator accuracy (persistent attacks)")
        ax2.legend(fontsize=9); ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, "tau_estimator_accuracy.png"), dpi=200)
    plt.close(fig)
    print(f"saved {os.path.join(args.outdir, 'tau_estimator_accuracy.png')}")


if __name__ == "__main__":
    main()
