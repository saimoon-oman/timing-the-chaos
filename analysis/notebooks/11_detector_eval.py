"""
11_detector_eval.py - Radar-referenced detection of temporal desynchronization.

Channels available to the follower (nothing else is used by any detector):
  g_hat(t)   perceived gap from the (attacked) V2V position stream
  g_radar(t) forward-radar range: min of N=3 Gaussian readings, sigma=0.3 m,
             around the physical range -- DhakaSim's min-of-N radar model.
             The adversary is on the V2V network path, so radar stays fresh.
  v_lead(t)  leader speed carried in the V2V position-history buffer.

Detectors (all causal):
  D1 residual   r(t)      = g_radar(t) - g_hat(t)                    > theta
  D2 windowed   mean_{W}  r                                          > theta
  D3 strip-DTW  DTW( floor(g_hat/w_s), floor(g_radar/w_s) ) over W   > theta
  D4 tau-hat    causal delay estimate r(t)/v_lead, trailing median + EMA > theta

Thresholds are calibrated on UNATTACKED baseline runs only, at a target
false-positive rate; the attacked runs are never used for calibration.

Task A: per-sample staleness (this is the gate for the correction stage).
        label = |g_hat - g_true| > 0.25 m, from the simulator. The detector
        sees only the noisy radar, never g_true.
Task B: per-series attack presence (fleet-level alarm). The series statistic is
        the fraction of its samples that raise a D1 alarm; label comes from the
        run configuration, so it is independent of any simulator quantity.

Motion gate: a timing attack is unobservable while the leader is stationary
(r -> 0 as v_lead -> 0). Results are reported pooled and restricted to
informative samples (v_lead > 0.5 m/s), the same gate the tau estimator uses.

Usage: python 11_detector_eval.py <run_dir> <topology_label>
"""
import os, re, sys, json
import numpy as np
import pandas as pd

RADAR_SIGMA, RADAR_N = 0.3, 3
STRIP_W = 0.5
STALE_THRESH = 0.25
WIN = 10
MIN_SPEED = 0.5
SEED = 7
FPR_TARGETS = [0.001, 0.01, 0.05]

SCEN = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
        3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}


def trailing_mean(x, w):
    c = np.cumsum(np.insert(x, 0, 0.0))
    idx = np.arange(len(x))
    lo = np.maximum(0, idx - w + 1)
    return (c[idx + 1] - c[lo]) / (idx - lo + 1)


def dtw_dist(a, b):
    m = len(b)
    prev = np.full(m + 1, np.inf); prev[0] = 0.0
    for i in range(len(a)):
        cur = np.full(m + 1, np.inf)
        ai = a[i]
        for j in range(1, m + 1):
            cur[j] = abs(ai - b[j - 1]) + min(prev[j], cur[j - 1], prev[j - 1])
        prev = cur
    return prev[m] / max(len(a), m)


def strip_dtw_series(gh, gr, w):
    s1 = np.floor(gh / STRIP_W); s2 = np.floor(gr / STRIP_W)
    out = np.zeros(len(gh))
    for t in range(len(gh)):
        lo = max(0, t - w + 1)
        if t - lo >= 2:
            out[t] = dtw_dist(s1[lo:t + 1], s2[lo:t + 1])
    return out


def tau_hat_series(resid, v_lead, window=30, alpha=0.2, lo=0.0, hi=15.0):
    """Causal delay estimate: trailing median of r/v, EMA-smoothed."""
    n = len(resid)
    out = np.zeros(n); tau = 1.0; hist = []
    for t in range(n):
        inst = np.nan
        v = v_lead[t]
        if abs(v) > MIN_SPEED:
            cand = resid[t] / v
            if lo <= cand <= hi:
                inst = cand
        hist.append(inst)
        w = [x for x in hist[-window:] if not np.isnan(x)]
        if w:
            tau = (1 - alpha) * tau + alpha * float(np.median(w))
            tau = min(max(tau, lo), hi)
        out[t] = tau
    return out


def scenario_of(cid):
    if cid.startswith("baseline"):
        return -1
    m = re.match(r"sc(\d)", cid)
    return int(m.group(1)) if m else -2


def collect(run_dir, with_dtw=True):
    rng = np.random.default_rng(SEED)
    recs = []
    for root, _d, files in sorted(os.walk(run_dir)):
        if "gap_errors.csv" not in files:
            continue
        cid = os.path.basename(root)
        sc = scenario_of(cid)
        df = pd.read_csv(os.path.join(root, "gap_errors.csv"))
        if df.empty or "perceivedGap" not in df.columns:
            continue
        for _k, grp in df.groupby(["leaderId", "followerId"], sort=False):
            if len(grp) < 5:
                continue
            g = grp.sort_values("simStep")
            gh = g["perceivedGap"].to_numpy(float)
            gt = g["trueGap"].to_numpy(float)
            vl = g["leaderSpeed"].to_numpy(float)
            gr = np.random.default_rng(rng.integers(1 << 31)).normal(
                gt[:, None], RADAR_SIGMA, size=(len(gt), RADAR_N)).min(axis=1)
            r = gr - gh
            recs.append(dict(
                cid=cid, sc=sc,
                stale=np.abs(gh - gt) > STALE_THRESH,
                moving=np.abs(vl) > MIN_SPEED,
                d1=r, d2=trailing_mean(r, WIN),
                d3=strip_dtw_series(gh, gr, WIN) if with_dtw else np.zeros(len(gh)),
                d4=tau_hat_series(r, vl)))
    return recs


def auc_from(scores, labels, n=6000, seed=1):
    pos, neg = scores[labels], scores[~labels]
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    rng = np.random.default_rng(seed)
    if len(pos) > n: pos = rng.choice(pos, n, replace=False)
    if len(neg) > n: neg = rng.choice(neg, n, replace=False)
    allv = np.concatenate([pos, neg])
    order = allv.argsort(); ranks = np.empty(len(allv))
    ranks[order] = np.arange(1, len(allv) + 1)
    return float((ranks[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2)
                 / (len(pos) * len(neg)))


def main():
    run_dir, label = sys.argv[1], sys.argv[2]
    recs = collect(run_dir)
    att = [r for r in recs if r["sc"] >= 0]
    base = [r for r in recs if r["sc"] == -1]
    res = {"topology": label, "n_series": len(recs), "n_series_attacked": len(att),
           "n_series_baseline": len(base),
           "n_configs": len({r["cid"] for r in recs}),
           "n_samples": int(sum(len(r["d1"]) for r in recs))}

    def pool(rs, key, gate=None):
        if not rs: return np.array([])
        if gate is None:
            return np.concatenate([r[key] for r in rs])
        return np.concatenate([r[key][r[gate]] for r in rs])

    res["task_A"] = {}
    for gate, gname in ((None, "all"), ("moving", "moving")):
        blk = {}
        for det in ("d1", "d2", "d3", "d4"):
            bs = pool(base, det, gate)
            entry = {}
            for tgt in FPR_TARGETS:
                th = float(np.quantile(bs, 1 - tgt))
                per = {}
                for sc in sorted({r["sc"] for r in att}):
                    rs = [r for r in att if r["sc"] == sc]
                    s = pool(rs, det, gate); lab = pool(rs, "stale", gate)
                    per[int(sc)] = dict(
                        tpr=float((s > th)[lab].mean()) if lab.any() else None,
                        n=int(len(s)), stale_frac=float(lab.mean()))
                s = pool(att, det, gate); lab = pool(att, "stale", gate)
                per["all"] = dict(tpr=float((s > th)[lab].mean()),
                                  n=int(len(s)), stale_frac=float(lab.mean()))
                entry[f"fpr{tgt}"] = dict(theta=th, theta_strips=th / STRIP_W,
                                          per_scenario=per)
            s = pool(att, det, gate); lab = pool(att, "stale", gate)
            entry["auc_pooled"] = auc_from(s, lab)
            entry["auc_by_scenario"] = {
                int(sc): auc_from(pool([r for r in att if r["sc"] == sc], det, gate),
                                  pool([r for r in att if r["sc"] == sc], "stale", gate))
                for sc in sorted({r["sc"] for r in att})}
            blk[det] = entry
        res["task_A"][gname] = blk

    # ---- Task B: per-series alarm = fraction of samples above the D1 gate ----
    bs = pool(base, "d1", "moving")
    th_s = float(np.quantile(bs, 0.99))
    def frac(r):
        m = r["moving"]
        return float((r["d1"][m] > th_s).mean()) if m.any() else 0.0
    sc_id = np.array([r["sc"] for r in recs])
    stat = np.array([frac(r) for r in recs])
    lab = sc_id >= 0
    tb = {"sample_theta": th_s, "auc": auc_from(stat, lab)}
    for tgt in FPR_TARGETS:
        th = float(np.quantile(stat[~lab], 1 - tgt))
        per = {int(sc): dict(tpr=float((stat[sc_id == sc] > th).mean()),
                             n=int((sc_id == sc).sum()))
               for sc in sorted(set(sc_id[sc_id >= 0]))}
        per["all"] = dict(tpr=float((stat[lab] > th).mean()), n=int(lab.sum()))
        tb[f"fpr{tgt}"] = dict(theta=th, per_scenario=per)
    res["task_B"] = tb

    with open(f"detector2_{label}.json", "w") as fh:
        json.dump(res, fh, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "task_A"}, indent=1)[:1500])
    print("saved", f"detector2_{label}.json")


if __name__ == "__main__":
    main()
