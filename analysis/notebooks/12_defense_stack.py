"""
12_defense_stack.py - End-to-end detect-and-correct evaluation on real traces.

Stages
  1. Detection : alarm when the V2V-derived gap and the fresh radar range
                 disagree by more than one strip, |g_radar - g_hat| > w_s.
  2. Estimation: causal delay estimate tau_hat(t) = median-of-window of
                 (g_radar - g_hat) / v_leader(t - tau_hat), EMA-smoothed.
  3. Correction: g_corr(t) = g_hat(t) + v_leader(t - tau_hat) * tau_hat,
                 applied only on alarmed samples.

Candidates (MAE against the physical gap, per vehicle-pair series, averaged):
  raw            no defense
  median5        sliding-window temporal median, k = 5
  fix1_oracle    vcomp with tau_hat = 1 s, oracle staleness gate
  oracle_oracle  vcomp with the true configured delay, oracle gate (upper bound)
  causal_oracle  vcomp with causal tau_hat, oracle gate (repo configuration)
  causal_det     vcomp with causal tau_hat, DETECTOR gate  <-- deployable stack

Usage: python 12_defense_stack.py <run_dir> <label>
"""
import os, re, sys, json
import numpy as np
import pandas as pd

RADAR_SIGMA, RADAR_N = 0.3, 3
STRIP_W = 0.5
STALE_THRESH = 0.25
DET_THETA = STRIP_W      # one strip
MIN_SPEED = 0.5
SEED = 7

SCEN = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
        3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}
ORACLE = {"sc0_delay0.5": 0.5, "sc0_delay1.0": 1.0, "sc0_delay2.0": 2.0,
          "sc0_delay3.0": 3.0, "sc0_delay4.0": 4.0, "sc0_delay5.0": 5.0,
          "sc0_delay6.0": 6.0, "sc0_delay7.0": 7.0, "sc0_delay10.0": 10.0}


def tau_hat_series(resid, v_lead, window=30, alpha=0.2, lo=0.0, hi=15.0):
    n = len(resid); out = np.zeros(n); tau = 1.0; hist = []
    for t in range(n):
        inst = np.nan
        v = v_lead[t]
        if abs(v) > MIN_SPEED:
            c = resid[t] / v
            if lo <= c <= hi:
                inst = c
        hist.append(inst)
        w = [x for x in hist[-window:] if not np.isnan(x)]
        if w:
            tau = (1 - alpha) * tau + alpha * float(np.median(w))
            tau = min(max(tau, lo), hi)
        out[t] = tau
    return out


def vcomp(gh, v_lead, tau, gate):
    out = gh.copy()
    for i in range(len(gh)):
        if not gate[i]:
            continue
        j = i - max(1, int(round(float(tau[i]))))
        if j >= 0:
            out[i] = gh[i] + v_lead[j] * tau[i]
    return out


def median_k(x, k=5):
    n = len(x); out = np.empty(n)
    for i in range(n):
        out[i] = np.median(x[max(0, i - k + 1):i + 1])
    return out


def scenario_of(cid):
    if cid.startswith("baseline"):
        return -1
    m = re.match(r"sc(\d)", cid)
    return int(m.group(1)) if m else -2


def main():
    run_dir, label = sys.argv[1], sys.argv[2]
    rng = np.random.default_rng(SEED)
    rows = []
    for root, _d, files in sorted(os.walk(run_dir)):
        if "gap_errors.csv" not in files:
            continue
        cid = os.path.basename(root)
        sc = scenario_of(cid)
        oracle_tau = next((v for k, v in ORACLE.items() if cid.startswith(k)), None)
        df = pd.read_csv(os.path.join(root, "gap_errors.csv"))
        if df.empty or "perceivedGap" not in df.columns:
            continue
        per = []
        for _k, grp in df.groupby(["leaderId", "followerId"], sort=False):
            if len(grp) < 5:
                continue
            g = grp.sort_values("simStep")
            gh = g["perceivedGap"].to_numpy(float)
            gt = g["trueGap"].to_numpy(float)
            vl = g["leaderSpeed"].to_numpy(float)
            gr = np.random.default_rng(rng.integers(1 << 31)).normal(
                gt[:, None], RADAR_SIGMA, size=(len(gt), RADAR_N)).min(axis=1)
            resid = gr - gh
            n = len(gh)
            oracle_gate = np.abs(gh - gt) > STALE_THRESH
            det_gate = resid > DET_THETA   # signed: V2V gap reads short by >1 strip
            tau = tau_hat_series(resid, vl)
            mae = lambda a: float(np.nanmean(np.abs(a - gt)))
            rec = dict(
                raw=mae(gh),
                median5=mae(median_k(gh, 5)),
                fix1_oracle=mae(vcomp(gh, vl, np.full(n, 1.0), oracle_gate)),
                causal_oracle=mae(vcomp(gh, vl, tau, oracle_gate)),
                causal_det=mae(vcomp(gh, vl, tau, det_gate)),
                oracle_gate_frac=float(oracle_gate.mean()),
                det_gate_frac=float(det_gate.mean()),
                gate_precision=float((oracle_gate & det_gate).sum() / max(det_gate.sum(), 1)),
                gate_recall=float((oracle_gate & det_gate).sum() / max(oracle_gate.sum(), 1)),
                tau_hat_median=float(np.median(tau[oracle_gate])) if oracle_gate.any() else np.nan,
            )
            rec["oracle_oracle"] = (mae(vcomp(gh, vl, np.full(n, oracle_tau), oracle_gate))
                                    if oracle_tau is not None else np.nan)
            per.append(rec)
        if not per:
            continue
        agg = {k: float(np.nanmean([p[k] for p in per])) for k in per[0]}
        agg.update(config_id=cid, scenario=sc, oracle_tau=oracle_tau,
                   n_series=len(per))
        rows.append(agg)

    df = pd.DataFrame(rows)
    df.to_csv(f"defense_stack_{label}.csv", index=False)
    METHODS = ["median5", "fix1_oracle", "oracle_oracle", "causal_oracle", "causal_det"]
    out = {"topology": label, "n_configs": int(len(df)),
           "det_theta_m": DET_THETA}

    att = df[df.scenario >= 0]
    per_sc = {}
    for sc in sorted(att.scenario.unique()):
        s = att[att.scenario == sc]
        e = {"n_configs": int(len(s)), "mae_raw": float(s.raw.mean()),
             "gate_precision": float(s.gate_precision.mean()),
             "gate_recall": float(s.gate_recall.mean()),
             "oracle_gate_frac": float(s.oracle_gate_frac.mean()),
             "det_gate_frac": float(s.det_gate_frac.mean())}
        for m in METHODS:
            e["red_" + m] = float(100 * (1 - s[m].mean() / s.raw.mean()))
            e["mae_" + m] = float(s[m].mean())
        per_sc[int(sc)] = e
    out["per_scenario"] = per_sc

    p = att[att.scenario == 0]
    out["persistent_by_delay"] = {}
    for d in sorted(p.oracle_tau.dropna().unique()):
        s = p[p.oracle_tau == d]
        e = {"mae_raw": float(s.raw.mean()), "tau_hat_median": float(s.tau_hat_median.mean())}
        for m in METHODS:
            e["red_" + m] = float(100 * (1 - s[m].mean() / s.raw.mean()))
        out["persistent_by_delay"][float(d)] = e

    # median-consensus failure range, per configuration
    ratio = 100 * (att.median5 / att.raw - 1.0)
    out["median_failure"] = {"min_pct_worse": float(ratio.min()),
                             "max_pct_worse": float(ratio.max()),
                             "median_pct_worse": float(ratio.median()),
                             "n_configs_worse": int((ratio > 0).sum()),
                             "n_configs": int(len(ratio))}
    b = df[df.scenario == -1]
    if len(b):
        out["baseline"] = {"mae_raw": float(b.raw.mean()),
                           "det_gate_frac": float(b.det_gate_frac.mean()),
                           "mae_causal_det": float(b.causal_det.mean())}
    with open(f"defense_stack_{label}.json", "w") as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
