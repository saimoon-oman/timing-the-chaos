"""
07_statistics.py - Statistical significance (ANOVA + Tukey HSD), TTC safety
analysis, and publication figures for the temporal desync experiments.

Usage:
    python analysis/notebooks/07_statistics.py --results <results_summary.csv>
    python analysis/notebooks/07_statistics.py --results <...> --gapdir <run_dir> --outdir analysis/figures
"""

import argparse, csv, os, sys
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

FIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))

SCENARIO_NAMES = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
                  3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}

# ---------------------------------------------------------------------------
# ANOVA + Tukey HSD (studentized range, no statsmodels dependency)
# ---------------------------------------------------------------------------

def oneway_anova(groups: dict, alpha=0.05):
    """groups: {name: np.array}. Returns ANOVA table + Tukey HSD comparisons."""
    names = list(groups.keys())
    k = len(names)
    data = [groups[n] for n in names]
    n_all = np.concatenate(data)
    grand_mean = n_all.mean()
    N = len(n_all)

    ss_between = sum(len(d) * (d.mean() - grand_mean) ** 2 for d in data)
    ss_within = sum(((d - d.mean()) ** 2).sum() for d in data)
    df_between, df_within = k - 1, N - k
    ms_between, ms_within = ss_between / df_between, ss_within / df_within
    F = ms_between / ms_within
    p = 1 - stats.f.cdf(F, df_between, df_within)

    # Tukey HSD
    n_avg = N / k
    se = np.sqrt(ms_within / n_avg)
    q_crit = stats.studentized_range.ppf(1 - alpha, k, df_within)
    hsd = q_crit * se / np.sqrt(2)

    comparisons = []
    for i in range(k):
        for j in range(i + 1, k):
            d = groups[names[i]].mean() - groups[names[j]].mean()
            comparisons.append({
                "group_a": names[i], "group_b": names[j],
                "mean_diff": round(d, 3),
                "sig": "significant" if abs(d) > hsd else "not significant",
            })
    return {"F": F, "p": p, "df_between": df_between, "df_within": df_within,
            "hsd": hsd, "comparisons": comparisons}


def run_anova(results: pd.DataFrame) -> dict:
    """ANOVA on scenario means (120s fractional subset, delay>=1)."""
    out = {}

    # ---- Scenario effect on MAE (all 6 scenarios, excluding baseline sc=-1) ----
    scen_groups = {}
    for sc in sorted(results["attack_scenario"].unique()):
        if int(sc) < 0:
            continue
        vals = results[results["attack_scenario"] == sc]["gap_error_mean_abs"].dropna()
        if len(vals) > 0:
            scen_groups[SCENARIO_NAMES.get(int(sc), f"sc{sc}")] = vals.values
    out["scenario_anova"] = oneway_anova(scen_groups)

    # ---- Delay effect on MAE (persistent, scenario 0) ----
    pers = results[results["attack_scenario"] == 0].copy()
    delay_groups = {}
    for d in sorted(pers["delay_mean"].unique()):
        vals = pers[pers["delay_mean"] == d]["gap_error_mean_abs"].dropna()
        if len(vals) > 0:
            delay_groups[f"delay{d:g}"] = vals.values
    out["delay_anova"] = oneway_anova(delay_groups)

    # ---- Attacked vs baseline: speed & waiting (two-sample t-test) ----
    base = results[results["error_mode"] == False]  # noqa: E712
    att = results[results["error_mode"] == True]    # noqa: E712
    for metric, col in [("speed", "avg_speed"), ("waiting", "avg_waiting_time")]:
        a = att[col].dropna().values
        b = base[col].dropna().values
        t, p = stats.ttest_ind(a, b, equal_var=False)
        d = (a.mean() - b.mean()) / np.sqrt((a.var() + b.var()) / 2) if len(a) > 1 and len(b) > 1 else np.nan
        out[f"{metric}_vs_baseline"] = {
            "attacked_mean": round(float(np.mean(a)), 3), "baseline_mean": round(float(np.mean(b)), 3),
            "t": round(float(t), 3), "p": float(p), "cohens_d": round(float(d), 3)}

    # ---- Collision / accident summary ----
    out["collisions"] = {
        "attacked_total": int(att["collision_count"].fillna(0).sum()),
        "baseline_total": int(base["collision_count"].fillna(0).sum()),
        "attacked_runs": len(att), "baseline_runs": len(base)}
    return out


# ---------------------------------------------------------------------------
# TTC analysis from per-run gap_errors.csv
# ---------------------------------------------------------------------------

TTC_THRESHOLDS = [1.0, 2.0, 3.0]


def ttc_series(df: pd.DataFrame, gap_col: str):
    """TTC = gap / closing speed (only where follower is closing in)."""
    closing = (df["followerSpeed"] - df["leaderSpeed"]).values
    gap = df[gap_col].values
    mask = closing > 0.01
    ttc = np.full(len(df), np.nan)
    ttc[mask] = np.maximum(gap[mask], 0) / closing[mask]
    return ttc, mask


def analyze_ttc(gapdir: str) -> pd.DataFrame:
    """Aggregate TTC metrics per config across all gap_errors.csv files."""
    rows = []
    for root, _dirs, files in os.walk(gapdir):
        if "gap_errors.csv" not in files:
            continue
        df = pd.read_csv(os.path.join(root, "gap_errors.csv"))
        if df.empty:
            continue
        cid = os.path.basename(root)
        is_baseline = cid.startswith("baseline")
        sc = -1 if is_baseline else int(re_search(r"sc(\d)", cid))
        delay = None if is_baseline else re_search(r"delay([\d.]+)", cid)

        ttc_true, mask = ttc_series(df, "trueGap")
        ttc_perc, _ = ttc_series(df, "perceivedGap")
        n_close = int(mask.sum())
        for thr in TTC_THRESHOLDS:
            rows.append({
                "config_id": cid, "attack_scenario": sc, "delay_mean": delay,
                "is_baseline": is_baseline, "threshold_s": thr,
                "ttc_true_violation_rate": round(float(np.nanmean(ttc_true[mask] < thr)) if n_close else np.nan, 4),
                "ttc_perceived_violation_rate": round(float(np.nanmean(ttc_perc[mask] < thr)) if n_close else np.nan, 4),
                "n_close_steps": n_close,
                "ttc_true_median": round(float(np.nanmedian(ttc_true)), 3),
                "ttc_perceived_median": round(float(np.nanmedian(ttc_perc)), 3),
                "perc_gap_overestimation": round(float(np.mean(df["gapError"] > 0.1)), 4),
            })
    return pd.DataFrame(rows)


def re_search(pat, text):
    import re
    m = re.search(pat, text)
    return m.group(1) if m else None


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_ttc_cdf(results: pd.DataFrame, ttc: pd.DataFrame, run_dir: str, outdir: str):
    """CDF of true vs perceived TTC for persistent delay (worst case)."""
    fig, ax = plt.subplots(figsize=(8, 5))
    for sc, label, color in [(0, "Persistent: true TTC", "#0f766e"),
                             (0, "Persistent: perceived TTC", "#f59e0b")]:
        pass  # placeholder (perceived/true handled per config below)

    def load_ttc(cond, col):
        raw = []
        for cid in cond["config_id"].unique():
            p = os.path.join(run_dir, cid, "gap_errors.csv")
            if not os.path.exists(p):
                continue
            df = pd.read_csv(p)
            t, _ = ttc_series(df, col)
            raw.append(t)
        return np.concatenate([r[~np.isnan(r)] for r in raw]) if raw else np.array([])

    # Persistent delay: true vs perceived
    pers = ttc[ttc["attack_scenario"] == 0]
    for col, label, color, ls in [("trueGap", "Persistent: true TTC", "#0f766e", "-"),
                                  ("perceivedGap", "Persistent: perceived TTC", "#f59e0b", "-")]:
        all_ttc = load_ttc(pers, col)
        if len(all_ttc):
            xs = np.sort(all_ttc); ys = np.linspace(0, 1, len(xs))
            ax.plot(xs, ys, lw=2, color=color, ls=ls, label=label)

    # Baseline true TTC
    base = ttc[ttc["is_baseline"] == True]  # noqa: E712
    all_ttc = load_ttc(base, "trueGap")
    if len(all_ttc):
        xs = np.sort(all_ttc); ys = np.linspace(0, 1, len(xs))
        ax.plot(xs, ys, lw=2.4, ls="--", color="black", label="Baseline: true TTC")

    ax.axvline(2.0, color="red", lw=1, ls=":")
    ax.text(2.02, 0.5, "TTC = 2 s", color="red", fontsize=9)
    ax.set_xlim(0, 30)
    ax.set_xlabel("TTC (s)")
    ax.set_ylabel("CDF")
    ax.set_title("CDF of True vs Perceived TTC (Persistent Delay, closing-in steps)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "ttc_cdf.png"), dpi=200)
    plt.close(fig)
    print("saved ttc_cdf.png")


def fig_ttc_violation(ttc: pd.DataFrame, outdir: str):
    """Perceived vs true TTC<2s violation rate per scenario (risk misjudgment)."""
    rows = []
    for sc, label in SCENARIO_NAMES.items():
        sub = ttc[(ttc["attack_scenario"] == sc) & (ttc["threshold_s"] == 2.0)]
        if sub.empty:
            continue
        rows.append((label,
                     np.nanmean(sub["ttc_true_violation_rate"]) * 100,
                     np.nanmean(sub["ttc_perceived_violation_rate"]) * 100))

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(rows)); w = 0.38
    ax.bar(x - w / 2, [r[1] for r in rows], w, label="True TTC < 2 s", color="#0f766e", edgecolor="black", linewidth=0.5)
    ax.bar(x + w / 2, [r[2] for r in rows], w, label="Perceived TTC < 2 s", color="#f59e0b", edgecolor="black", linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels([r[0] for r in rows], rotation=15)
    ax.set_ylabel("Steps below 2 s (%)")
    ax.set_title("Risk Misjudgment: Perceived vs True TTC Violation Rate")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "ttc_violation_rate.png"), dpi=200)
    plt.close(fig)
    print("saved ttc_violation_rate.png")


def fig_perceived_vs_true(ttc: pd.DataFrame, outdir: str):
    """Perceived vs true TTC median per scenario (perception divergence)."""
    rows = []
    for sc, label in SCENARIO_NAMES.items():
        sub = ttc[(ttc["attack_scenario"] == sc) & (ttc["threshold_s"] == 2.0)]
        if sub.empty:
            continue
        rows.append({
            "scenario": label,
            "ttc_true_median": np.nanmean(sub["ttc_true_median"]),
            "ttc_perceived_median": np.nanmean(sub["ttc_perceived_median"]),
        })
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(rows)); w = 0.38
    ax.bar(x - w / 2, [r["ttc_true_median"] for r in rows], w, label="True TTC (median)", color="#0f766e")
    ax.bar(x + w / 2, [r["ttc_perceived_median"] for r in rows], w, label="Perceived TTC (median)", color="#f59e0b")
    ax.set_xticks(x); ax.set_xticklabels([r["scenario"] for r in rows], rotation=15)
    ax.set_ylabel("Median TTC (s)")
    ax.set_title("Perception Divergence: True vs Perceived TTC Under Attack")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "perceived_vs_true_ttc.png"), dpi=200)
    plt.close(fig)
    print("saved perceived_vs_true_ttc.png")


def fig_anova_means(anova: dict, results: pd.DataFrame, outdir: str):
    """Scenario means with 95% CI error bars (persistent shows saturation)."""
    scen_groups = anova["scenario_anova"]
    # rebuild means/CI from results
    names, means, cis = [], [], []
    for sc, label in SCENARIO_NAMES.items():
        vals = results[results["attack_scenario"] == sc]["gap_error_mean_abs"].dropna().values
        if len(vals) == 0:
            continue
        names.append(label); means.append(vals.mean())
        se = vals.std(ddof=1) / np.sqrt(len(vals))
        cis.append(1.96 * se)
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(names))
    ax.bar(x, means, yerr=cis, capsize=4, color="#0f766e", edgecolor="black", linewidth=0.5)
    for i, m in enumerate(means):
        ax.text(i, m + cis[i] + 0.2, f"{m:.2f}", ha="center", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=15)
    ax.set_ylabel("Mean |gap error| (m)")
    ax.set_title("Gap Error by Scenario (95% CI)\nANOVA F={:.1f}, p={:.2g}".format(scen_groups["F"], scen_groups["p"]))
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "anova_scenario_means.png"), dpi=200)
    plt.close(fig)
    print("saved anova_scenario_means.png")


# ---------------------------------------------------------------------------

results_ttc_path = None  # set in main; used by fig_ttc_cdf

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", required=True, help="path to results_summary.csv")
    ap.add_argument("--gapdir", default=None, help="run dir containing per-config gap_errors.csv dirs")
    ap.add_argument("--outdir", default=FIG_DIR)
    args = ap.parse_args()

    global results_ttc_path
    results_ttc_path = args.gapdir or os.path.dirname(args.results)
    os.makedirs(args.outdir, exist_ok=True)

    results = pd.read_csv(args.results)
    results["gap_error_mean_abs"] = pd.to_numeric(results["gap_error_mean_abs"], errors="coerce")
    results["avg_speed"] = pd.to_numeric(results["avg_speed"], errors="coerce")
    results["avg_waiting_time"] = pd.to_numeric(results["avg_waiting_time"], errors="coerce")

    anova = run_anova(results)
    print("=" * 60)
    print("ANOVA + Tukey HSD")
    print("=" * 60)
    for key, res in anova.items():
        if key.endswith("_anova"):
            print(f"\n[{key}] F={res['F']:.2f}, p={res['p']:.2e}, HSD={res['hsd']:.3f} m")
            for c in res["comparisons"]:
                print(f"  {c['group_a']} vs {c['group_b']}: diff={c['mean_diff']} ({c['sig']})")
        elif key.endswith("_vs_baseline"):
            print(f"\n[{key}] attacked={res['attacked_mean']}, baseline={res['baseline_mean']}, "
                  f"t={res['t']}, p={res['p']:.2e}, Cohen's d={res['cohens_d']}")
    print(f"\ncollisions: attacked={anova['collisions']['attacked_total']} "
          f"baseline={anova['collisions']['baseline_total']}")

    # TTC
    if args.gapdir:
        ttc = analyze_ttc(args.gapdir)
        ttc_path = os.path.join(args.gapdir, "ttc_summary.csv")
        ttc.to_csv(ttc_path, index=False)
        print(f"\nTTC summary: {ttc_path} ({len(ttc)} rows)")
        print("TTC < 2s violation rates (%):")
        piv = ttc[ttc["threshold_s"] == 2.0].groupby("attack_scenario")["ttc_true_violation_rate"].mean() * 100
        for sc, v in piv.items():
            print(f"  sc{int(sc)} ({SCENARIO_NAMES.get(int(sc))}): {v:.2f}%")
        if not ttc[ttc["is_baseline"] == True].empty:  # noqa: E712
            print(f"  baseline: {ttc[ttc['is_baseline']==True]['ttc_true_violation_rate'].mean()*100:.2f}%")

        fig_ttc_cdf(results, ttc, args.gapdir, args.outdir)
        fig_ttc_violation(ttc, args.outdir)
        fig_perceived_vs_true(ttc, args.outdir)

    fig_anova_means(anova, results, args.outdir)

    # Save ANOVA results
    with open(os.path.join(args.outdir, "statistics_summary.txt"), "w") as f:
        for key, res in anova.items():
            f.write(f"[{key}]\n")
            if key.endswith("_anova"):
                f.write(f"F={res['F']:.3f}, p={res['p']:.2e}, HSD={res['hsd']:.3f}\n")
                for c in res["comparisons"]:
                    f.write(f"  {c['group_a']} vs {c['group_b']}: diff={c['mean_diff']} ({c['sig']})\n")
            else:
                f.write(f"{res}\n")
    print(f"\nSaved: {os.path.join(args.outdir, 'statistics_summary.txt')}")


if __name__ == "__main__":
    main()
