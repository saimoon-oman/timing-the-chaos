"""14_sumo_compare.py - Cross-simulator validation: SUMO vs DhakaSim.

Tests the pre-registered hypotheses H1-H6 on the SUMO campaign traces and
places the DhakaSim numbers beside them using IDENTICAL measurement code
(same TTC function, same detector calibration, same tau estimator):

  H1 sign      : mean error negative ~ -vbar_l*tau; moving-leader negfrac
  H2 collapse  : cell MAE vs cell mean leader speed, slope ~= configured tau
  H3 dose      : MAE monotonic in tau (Spearman)
  H4 misjudged : perceived TTC<2s rate > true rate; zero non-junction collisions
  H5 detection : TPR@FPR on radar/V2V residual + causal correction vs oracle;
                 plus Kalman-residual and plausibility BASELINE detectors
  H6 ordering  : PD > IF ~= PDr > JM > JO variant ranking

DhakaSim side: facts_v4.json (vol/mix cells) + v3_ttc gap traces for the
detector/TTC recomputation.

Usage:
    python analysis/notebooks/14_sumo_compare.py \
        --sumo-root C:/sumo-env/campaign \
        --dhk-run timing-the-chaos/experiments/results/run_20260808_015724_v3_ttc \
        --facts timing-the-chaos/analysis/results/facts_v4.json \
        --outdir timing-the-chaos/analysis
"""
import argparse
import csv
import json
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                "../../defenses/schmidt_kalman")))
from tau_estimator import estimate_tau_series

WARMUP = 60
RADAR_SIGMA, RADAR_N = 0.3, 3
STALE_THRESH = 0.25
MIN_SPEED = 0.5
FPR_TARGETS = [0.001, 0.01, 0.05]
SEED = 7
TTC_THRESH = 2.0


# --------------------------------------------------------------------------
# Shared measurement primitives (identical for both simulators)
# --------------------------------------------------------------------------
def ttc_series(df, gap_col):
    """TTC = gap / closing speed (only where follower is closing in)."""
    closing = (df["followerSpeed"] - df["leaderSpeed"]).values
    gap = df[gap_col].values
    mask = closing > 0.01
    ttc = np.full(len(df), np.nan)
    ttc[mask] = np.maximum(gap[mask], 0) / closing[mask]
    return ttc, mask


def radar_synthesize(true, seed):
    rng = np.random.default_rng(seed)
    return np.array([min(rng.normal(t, RADAR_SIGMA, RADAR_N))
                     for t in true])


def velocity_compensated_stale(perceived, leader_speeds, tau_series, stale):
    n = len(perceived)
    corrected = perceived.copy()
    for i in range(n):
        ts = max(1, int(round(float(tau_series[i]))))
        j = i - ts
        if j < 0 or not stale[i]:
            continue
        corrected[i] = perceived[i] + leader_speeds[j] * tau_series[i]
    return corrected


def kalman_residual_score(gap):
    """1-D constant-velocity Kalman filter; score = |normalized innovation|.

    Fixed, documented noise levels (not tuned per dataset): process q=0.05,
    measurement r=RADAR_SIGMA^2. Causal (forward pass only).
    """
    q, r = 0.05, RADAR_SIGMA ** 2
    x = np.array([gap[0], 0.0])
    P = np.eye(2)
    F = np.array([[1.0, 1.0], [0.0, 1.0]])
    H = np.array([1.0, 0.0])
    Q = np.array([[0.25, 0.5], [0.5, 1.0]]) * q
    out = np.zeros(len(gap))
    for t, z in enumerate(gap):
        x = F @ x
        P = F @ P @ F.T + Q
        S = H @ P @ H + r
        innov = z - H @ x
        out[t] = abs(innov) / np.sqrt(S)
        K = (P @ H) / S
        x = x + K * innov
        P = (np.eye(2) - np.outer(K, H)) @ P
    return out


def plausibility_score(df):
    """s = |gap jump| - (|vf|+|vl|)*dt: physical plausibility margin (dt=1).

    A value-domain plausibility gate in the VANET tradition: alarms only on
    physically impossible jumps. Smooth staleness passes by design.
    """
    g = df["perceivedGap"].values.astype(float)
    vf = df["followerSpeed"].values.astype(float)
    vl = df["leaderSpeed"].values.astype(float)
    dg = np.zeros(len(g))
    dg[1:] = np.abs(g[1:] - g[:-1])
    return dg - (np.abs(vf) + np.abs(vl)) * 1.0


def collect_series(gap_path, is_baseline, seed):
    """Per (leader,follower) series dicts with radar + labels (cf. 11_*)."""
    df = pd.read_csv(gap_path)
    if df.empty or "perceivedGap" not in df.columns:
        return []
    if "sameEdge" in df.columns:
        df = df[df["sameEdge"] == 1]
    df = df[df["simStep"] >= WARMUP]
    rng = np.random.default_rng(seed)
    recs = []
    for _k, grp in df.groupby(["leaderId", "followerId"], sort=False):
        if len(grp) < 5:
            continue
        g = grp.sort_values("simStep")
        gh = g["perceivedGap"].to_numpy(float)
        gt = g["trueGap"].to_numpy(float)
        vl = g["leaderSpeed"].to_numpy(float)
        vf = g["followerSpeed"].to_numpy(float)
        gr = np.array([min(rng.normal(t, RADAR_SIGMA, RADAR_N))
                       for t in gt])
        recs.append(dict(stale=np.abs(gh - gt) > STALE_THRESH,
                         moving=np.abs(vl) > MIN_SPEED,
                         resid=gr - gh, vl=vl, vf=vf, gh=gh, gt=gt, gr=gr,
                         err=gh - gt, base=is_baseline))
    return recs


def pool(rs, key, gate=None):
    if not rs:
        return np.array([])
    if gate is None:
        return np.concatenate([r[key] for r in rs])
    return np.concatenate([r[key][r[gate]] for r in rs])


def tpr_at_fpr(base_scores, att_scores, att_labels, tgt):
    th = float(np.quantile(base_scores, 1 - tgt)) if len(base_scores) else np.inf
    if len(att_scores) == 0 or not att_labels.any():
        return None, th
    return float((att_scores > th)[att_labels].mean()), th


# --------------------------------------------------------------------------
# SUMO side
# --------------------------------------------------------------------------
def load_sumo_cells(root):
    """{ (block, attack, delay, vol, mix, seed): gap_path } + sidecars."""
    runs = os.path.join(root, "runs")
    cells = {}
    for d in sorted(os.listdir(runs)):
        gp = os.path.join(runs, d, "gap_errors.csv")
        sp = os.path.join(runs, d, "sidecar.json")
        if not (os.path.exists(gp) and os.path.exists(sp)):
            continue
        with open(sp) as f:
            sc = json.load(f)
        # parse factors from config_id: S1_pd_d3_v1_balanced_seed7 etc.
        m = re.match(r"(S\d)_([a-z]+)(?:_d([\d.]+))?(?:_p([\d.]+))?"
                     r"(?:_j([\d.]+))?_v([\d.]+)_([a-z]+)_seed(\d+)", d)
        if not m:
            continue
        blk, atk = m.group(1), m.group(2)
        atk = {"baseline": "none"}.get(atk, atk)
        cells[(blk, atk, float(m.group(3) or 0), float(m.group(6)),
               m.group(7), int(m.group(8)))] = (gp, sc)
    return cells


def cell_stats(gap_path):
    df = pd.read_csv(gap_path)
    if "sameEdge" in df.columns:
        df = df[df["sameEdge"] == 1]
    df = df[df["simStep"] >= WARMUP]
    if df.empty:
        return None
    mv = df[df["leaderSpeed"] > MIN_SPEED]
    return dict(mae=float(df["gapError"].abs().mean()),
                mean_lead=float(df["leaderSpeed"].mean()),
                negfrac_mov=float((mv["gapError"] < 0).mean()) if len(mv) else np.nan,
                n=len(df))


# --------------------------------------------------------------------------
# Main comparison
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo-root", required=True)
    ap.add_argument("--sumo-lc-root", default=None,
                    help="optional second campaign root (e.g. SL2015 "
                         "lane-change sensitivity subset) for H2b")
    ap.add_argument("--dhk-run", required=True)
    ap.add_argument("--facts", required=True)
    ap.add_argument("--outdir", required=True)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    os.makedirs(os.path.join(args.outdir, "figures"), exist_ok=True)
    facts = {}
    verdicts = {}

    cells = load_sumo_cells(args.sumo_root)
    print(f"sumo cells: {len(cells)}")

    def sel(block=None, attack=None, delay=None, vol=None, mix=None):
        return {k: v for k, v in cells.items()
                if (block is None or k[0] == block)
                and (attack is None or k[1] == attack)
                and (delay is None or k[2] == delay)
                and (vol is None or k[3] == vol)
                and (mix is None or k[4] == mix)}

    def agg(keys):
        maes, leads, neg = [], [], []
        for k in keys:
            s = cell_stats(cells[k][0])
            if s:
                maes.append(s["mae"])
                leads.append(s["mean_lead"])
                neg.append(s["negfrac_mov"])
        return (float(np.mean(maes)), float(np.mean(leads)),
                float(np.mean(neg)), len(maes))

    # ---- H1: sign at reference point (S1 PD3, 1x balanced) ----
    ref = sel("S1", "pd", 3.0, 1.0, "balanced")
    mae, lead, neg, n = agg(ref.keys())
    facts.update({"sumo.ref_pd3_mae": round(mae, 2),
                  "sumo.ref_pd3_meanlead": round(lead, 2),
                  "sumo.ref_pd3_negfrac_mov": round(neg, 4),
                  "sumo.ref_pd3_taueff": round(mae / lead, 3)})
    verdicts["H1"] = bool(mae > 0 and neg >= 0.75
                          and abs(mae - 3.0 * lead) / (3.0 * lead) <= 0.25)

    # ---- H2: collapse over S2 grid ----
    pts = []
    for vol in [0.5, 1.0, 2.0, 3.0]:
        for mix in ["slowdom", "balanced", "fastdom"]:
            sub = sel("S2", "pd", 3.0, vol, mix)
            m, l, _n, _c = agg(sub.keys())
            pts.append((l, m))
    pts = np.array(pts)
    slope, icept, r, p, _se = stats.linregress(pts[:, 0], pts[:, 1])
    facts.update({"sumo.collapse_slope": round(float(slope), 3),
                  "sumo.collapse_r2": round(float(r ** 2), 4),
                  "sumo.collapse_n": len(pts)})
    # DhakaSim side from facts_v4 (vol + mix PD cells, Shahbagh)
    fd = json.load(open(args.facts))
    dhk = []
    for vol in ["0.5", "1.0", "2.0", "3.0"]:
        k1, k2 = f"vol.shahbagh.{vol}.PD.mae", f"vol.shahbagh.{vol}.PD.leader_speed"
        if k1 in fd:
            dhk.append((fd[k2], fd[k1]))
    for mx in ["(80, 15, 5)", "(65, 20, 15)", "(50, 25, 25)", "(20, 30, 50)"]:
        k1, k2 = f"mix.shahbagh.{mx}.PD.mae", f"mix.shahbagh.{mx}.PD.leader_speed"
        if k1 in fd:
            dhk.append((fd[k2], fd[k1]))
    dhk = np.array(dhk)
    dslope, _i, dr, _p, _s = stats.linregress(dhk[:, 0], dhk[:, 1])
    dhk_ratios = dhk[:, 1] / dhk[:, 0]
    sumo_ratios = pts[:, 1] / pts[:, 0]
    facts.update({"sumo.dhk_collapse_slope": round(float(dslope), 3),
                  "sumo.dhk_collapse_r2": round(float(dr ** 2), 4),
                  "sumo.dhk_collapse_n": len(dhk),
                  "sumo.dhk_collapse_icept": round(float(_i), 3),
                  "sumo.dhk_taueff_mean": round(float(dhk_ratios.mean()), 3),
                  "sumo.collapse_icept": round(float(icept), 3),
                  "sumo.collapse_taueff_mean": round(float(sumo_ratios.mean()),
                                                     3)})
    verdicts["H2"] = bool(r ** 2 >= 0.95 and abs(slope - 3.0) / 3.0 <= 0.20
                          and dr ** 2 >= 0.95)

    # ---- H2b: bootstrap 95% CI on the SUMO collapse slope ----
    # Resample seeds within each of the 12 S2 cells (10 seeds each), refit.
    rng_bs = np.random.default_rng(SEED)
    per_seed = {}
    for vol in [0.5, 1.0, 2.0, 3.0]:
        for mix in ["slowdom", "balanced", "fastdom"]:
            vals = []
            for k in sel("S2", "pd", 3.0, vol, mix).keys():
                s = cell_stats(cells[k][0])
                if s:
                    vals.append((s["mean_lead"], s["mae"]))
            if vals:
                per_seed[(vol, mix)] = vals
    boot_slopes = []
    cell_list = list(per_seed.values())
    for _ in range(2000):
        xs, ys = [], []
        for vals in cell_list:
            idx = rng_bs.integers(0, len(vals), len(vals))
            xs.append(float(np.mean([vals[i][0] for i in idx])))
            ys.append(float(np.mean([vals[i][1] for i in idx])))
        boot_slopes.append(float(stats.linregress(xs, ys)[0]))
    lo, hi = float(np.quantile(boot_slopes, 0.025)), float(
        np.quantile(boot_slopes, 0.975))
    facts.update({"sumo.collapse_slope_ci95": [round(lo, 3), round(hi, 3)],
                  "sumo.collapse_slope_boot_n": 2000})

    # ---- H2c: lane-change-model sensitivity (SL2015 subset, if provided) --
    if args.sumo_lc_root:
        lc_cells = load_sumo_cells(args.sumo_lc_root)

        def sel_lc(block=None, attack=None, delay=None, vol=None, mix=None):
            return {k: v for k, v in lc_cells.items()
                    if (block is None or k[0] == block)
                    and (attack is None or k[1] == attack)
                    and (delay is None or k[2] == delay)
                    and (vol is None or k[3] == vol)
                    and (mix is None or k[4] == mix)}

        def agg_lc(keys):
            maes, leads = [], []
            for k in keys:
                s = cell_stats(lc_cells[k][0])
                if s:
                    maes.append(s["mae"])
                    leads.append(s["mean_lead"])
            return (float(np.mean(maes)), float(np.mean(leads)), len(maes))

        lpts = []
        for vol in [0.5, 1.0, 2.0, 3.0]:
            for mix in ["slowdom", "balanced", "fastdom"]:
                m, l, _c = agg_lc(sel_lc("S2", "pd", 3.0, vol, mix).keys())
                lpts.append((l, m))
        lpts = np.array(lpts)
        lslope, licept, lr, _p, _se = stats.linregress(lpts[:, 0], lpts[:, 1])
        # reference MAE on the shared S1-equivalent cell (1x balanced PD3)
        lm, _ll, _ln = agg_lc(sel_lc("S2", "pd", 3.0, 1.0, "balanced").keys()) \
            if sel_lc("S2", "pd", 3.0, 1.0, "balanced") else (None, None, 0)
        facts.update({"sumo.lc_collapse_slope": round(float(lslope), 3),
                      "sumo.lc_collapse_r2": round(float(lr ** 2), 4),
                      "sumo.lc_collapse_icept": round(float(licept), 3),
                      "sumo.lc_collapse_n": len(lpts),
                      "sumo.lc_ref_mae": round(lm, 2) if lm else None})
        verdicts["H2c_lc"] = bool(
            lr ** 2 >= 0.90 and abs(lslope - slope) / slope <= 0.15)

    # ---- H3: dose monotonicity (S1) ----
    doses = []
    for d in [1.0, 2.0, 3.0, 5.0, 7.0]:
        m, _l, _n, _c = agg(sel("S1", "pd", d, 1.0, "balanced").keys())
        doses.append((d, m))
    rho, _p = stats.spearmanr([x[0] for x in doses], [x[1] for x in doses])
    facts.update({"sumo.dose_spearman": round(float(rho), 4),
                  "sumo.dose_maes": {str(k): round(v, 2) for k, v in doses}})
    verdicts["H3"] = bool(rho >= 0.9)

    # ---- H4: misjudged risk (TTC) + collisions ----
    def ttc_rates(keys):
        tv, pv, n, biases = [], [], 0, []
        for k in keys:
            df = pd.read_csv(cells[k][0])
            if "sameEdge" in df.columns:
                df = df[df["sameEdge"] == 1]
            df = df[df["simStep"] >= WARMUP]
            tt, mask = ttc_series(df, "trueGap")
            tp, _ = ttc_series(df, "perceivedGap")
            if mask.sum() == 0:
                continue
            tv.append(float(np.nanmean(tt[mask] < TTC_THRESH)))
            pv.append(float(np.nanmean(tp[mask] < TTC_THRESH)))
            n += int(mask.sum())
            biases.append(float(np.nanmedian(tp[mask] - tt[mask])))
        return float(np.mean(tv)), float(np.mean(pv)), n, \
            round(float(np.mean(biases)), 2) if biases else None

    t_true, t_perc, ncl, med_bias = ttc_rates(
        sel("S1", "pd", 3.0, 1.0, "balanced").keys())
    facts.update({"sumo.ttc_true_viol": round(t_true, 4),
                  "sumo.ttc_perc_viol": round(t_perc, 4),
                  "sumo.ttc_nclose": ncl,
                  "sumo.ttc_median_bias": med_bias})
    summ = pd.read_csv(os.path.join(args.sumo_root, "sumo_results_summary.csv"))
    att_nj = int(summ[summ["attack_scenario"] != "none"]["collision_count"].sum())
    base_nj = int(summ[summ["attack_scenario"] == "none"]["collision_count"].sum())
    facts.update({"sumo.coll_nonjunction_att": att_nj,
                  "sumo.coll_nonjunction_base": base_nj})
    verdicts["H4"] = bool(t_perc - t_true >= 0.05 and att_nj == 0)

    # ---- H5: detection + correction on SUMO traces + baselines ----
    srecs, brecs = [], []
    for k, (gp, _sc) in cells.items():
        if k[0] not in ("S1", "S3"):
            continue
        recs = collect_series(gp, k[1] == "none", seed=SEED)
        (brecs if k[1] == "none" else srecs).extend(recs)
    # pooled delay-family TPR (S1 PD + S3 IF/PDr cells)
    fam = []
    for k, (gp, _sc) in cells.items():
        if k[0] == "S1" and k[1] == "pd":
            fam.append(gp)
        if k[0] == "S3" and k[1] in ("if", "pdr"):
            fam.append(gp)
    fr = []
    for gp in fam:
        fr.extend(collect_series(gp, False, SEED))
    br = []
    for k, (gp, _sc) in cells.items():
        if k[1] == "none":
            br.extend(collect_series(gp, True, SEED))
    det = {}
    for tgt in FPR_TARGETS:
        bs = pool(br, "resid")
        s = pool(fr, "resid")
        lab = pool(fr, "stale")
        tpr, th = tpr_at_fpr(bs, s, lab, tgt)
        det[str(tgt)] = {"thresh": round(th, 3),
                         "tpr_delayfam": round(tpr, 4) if tpr else None,
                         "n_pos": int(lab.sum()), "n_base": len(bs)}
    facts["sumo.det_tpr_001"] = det["0.001"]["tpr_delayfam"]
    facts["sumo.det_thresh_001"] = det["0.001"]["thresh"]
    # TPR at the looser 1% FPR operating point (same procedure)
    bs1 = pool(br, "resid")
    s1 = pool(fr, "resid")
    lab1 = pool(fr, "stale")
    tpr01, _th01 = tpr_at_fpr(bs1, s1, lab1, 0.01)
    facts["sumo.det_tpr_01"] = round(tpr01, 4) if tpr01 else None
    verdicts["H5det"] = bool(det["0.001"]["tpr_delayfam"] is not None
                             and det["0.001"]["tpr_delayfam"] >= 0.90)
    # Conditional performance on the population the signed detector targets:
    # negative stale errors. Plus the positive-error mass that escapes it.
    r_all = pool(fr, "resid")
    errs = pool(fr, "err")
    pos_frac = float((errs[lab1] > 0).mean()) if lab1.any() else 0.0
    neg_mask = lab1 & (errs < 0)
    tpr_neg = float((r_all[neg_mask] > det["0.001"]["thresh"]).mean()
                    ) if neg_mask.any() else None
    facts["sumo.poserr_frac_stale"] = round(pos_frac, 4)
    facts["sumo.det_tpr_001_negsigned"] = round(tpr_neg, 4) if tpr_neg else None
    verdicts["H5det_cond"] = bool(tpr_neg is not None and tpr_neg >= 0.95)
    # Absolute-residual variant (catches both signs; paper's design
    # discussion predicts it pays the radar-bias FPR cost).
    def abs_tpr(bs_scores, att_scores, att_labels, tgt):
        th = float(np.quantile(np.abs(bs_scores), 1 - tgt))
        if len(att_scores) == 0 or not att_labels.any():
            return None, th
        return float((np.abs(att_scores) > th)[att_labels].mean()), th
    tpr_abs, _th_abs = abs_tpr(bs1, s1, lab1, 0.001)
    facts["sumo.det_abs_tpr_001"] = round(tpr_abs, 4) if tpr_abs else None
    # Dense FPR grid for the SUMO signed-residual curve (reviewer request:
    # TPR at additional operating points, same calibration procedure).
    curve = {}
    for tgt in [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]:
        tpr_g, th_g = tpr_at_fpr(bs1, s1, lab1, tgt)
        curve[str(tgt)] = {"tpr": round(tpr_g, 4) if tpr_g else None,
                           "thresh": round(th_g, 3)}
    facts["sumo.det_curve_signed"] = curve

    # causal correction vs oracle on one reference cell batch
    ref_keys = list(sel("S1", "pd", 3.0, 1.0, "balanced").keys())
    maes = {"raw": [], "fixed": [], "oracle": [], "causal": []}
    for k in ref_keys:
        df = pd.read_csv(cells[k][0])
        if "sameEdge" in df.columns:
            df = df[df["sameEdge"] == 1]
        df = df[df["simStep"] >= WARMUP]
        for _kk, grp in df.groupby(["leaderId", "followerId"], sort=False):
            if len(grp) < 5:
                continue
            g = grp.sort_values("simStep")
            perc = g["perceivedGap"].to_numpy(float)
            true = g["trueGap"].to_numpy(float)
            lv = g["leaderSpeed"].to_numpy(float)
            n = len(perc)
            radar = radar_synthesize(true, SEED)
            stale = np.abs(perc - true) > STALE_THRESH
            tau_est = estimate_tau_series(perc, lv, radar)
            maes["raw"].append(float(np.nanmean(np.abs(perc - true))))
            maes["fixed"].append(float(np.nanmean(np.abs(
                velocity_compensated_stale(
                    perc, lv, np.full(n, 1.0), stale) - true))))
            maes["oracle"].append(float(np.nanmean(np.abs(
                velocity_compensated_stale(
                    perc, lv, np.full(n, 3.0), stale) - true))))
            maes["causal"].append(float(np.nanmean(np.abs(
                velocity_compensated_stale(
                    perc, lv, tau_est, stale) - true))))
    red = {k: round(1 - float(np.mean(v)) / float(np.mean(maes["raw"])), 4)
           for k, v in maes.items() if k != "raw"}
    facts.update({"sumo.vcomp_oracle": red["oracle"],
                  "sumo.vcomp_causal": red["causal"],
                  "sumo.vcomp_fixed": red["fixed"]})
    verdicts["H5corr"] = bool(red["causal"] > 0 and
                              red["causal"] >= red["oracle"] - 0.10)

    # ---- Step 5: experimental baseline detectors (same traces, same rule) --
    def eval_detector(score_fn, name):
        out = {}
        for tgt in FPR_TARGETS:
            bs = np.concatenate([
                score_fn(r) for r in br]) if br else np.array([])
            th = float(np.quantile(bs, 1 - tgt)) if len(bs) else np.inf
            ss = np.concatenate([score_fn(r) for r in fr])
            lab = np.concatenate([r["stale"] for r in fr])
            tpr, _ = tpr_at_fpr(bs, ss, lab, tgt)
            out[str(tgt)] = round(tpr, 4) if tpr else None
        return out

    def kalman_score(r):
        return kalman_residual_score(r["gh"])

    def plaus_score_wrap(r):
        df = pd.DataFrame({"perceivedGap": r["gh"], "followerSpeed": r["vf"],
                           "leaderSpeed": r["vl"]})
        return plausibility_score(df)

    # wrap recs with vf for plausibility (collect_series stores vf already)
    kal = eval_detector(kalman_score, "kalman")
    pla = eval_detector(plaus_score_wrap, "plaus")
    facts["sumo.kalman_tpr_001"] = kal["0.001"]
    facts["sumo.plaus_tpr_001"] = pla["0.001"]
    # expectation (pre-registered directionally): value-domain baselines are
    # near-blind to smooth staleness; record, do not gate a verdict on them
    verdicts["H5base_documented"] = True

    # ---- H6: variant ordering (S3 + S1 PD3) ----
    order = {}
    for atk in ["pd", "if", "pdr", "jo"]:
        kk = list(sel("S3" if atk != "pd" else "S1", atk,
                      3.0 if atk in ("pd", "if") else None,
                      1.0, "balanced").keys())
        m, _l, _n, _c = agg(kk)
        order[atk] = round(m, 2)
    facts["sumo.variant_maes"] = order
    rank = sorted(order, key=order.get, reverse=True)
    verdicts["H6"] = bool(rank[0] == "pd" and rank[-1] == "jo")

    # ---- DhakaSim detector/TTC reference recomputation (same code) ----
    dhk_dir = args.dhk_run
    dhk_series = {"pd": [], "if": [], "pdr": [], "jo": [], "base": []}
    for root, _d, files in os.walk(dhk_dir):
        if "gap_errors.csv" not in files:
            continue
        cid = os.path.basename(root)
        key = ("base" if cid.startswith("baseline")
               else {"sc0": "pd", "sc1": "if", "sc3": "jo",
                     "sc4": "pdr"}.get(cid[:3], None))
        if key is None:
            continue  # jm/ss configs have no SUMO counterpart; skip
        dhk_series[key].extend(collect_series(os.path.join(root, "gap_errors.csv"),
                                       key == "base", SEED))
    dres = {}
    derrs = []
    for tgt in [0.001]:
        bs = pool(dhk_series["base"], "resid")
        for atk in ["pd", "if", "pdr", "jo"]:
            s = pool(dhk_series[atk], "resid")
            lab = pool(dhk_series[atk], "stale")
            tpr, th = tpr_at_fpr(bs, s, lab, tgt)
            dres[atk] = round(tpr, 4) if tpr else None
    facts["sumo.dhk_recomputed_tpr_001"] = dres
    # DhakaSim positive-error mass + absolute-residual variant, same code.
    # Restrict to the delay family (pd/if/pdr), matching the SUMO pool:
    # jitter-only is symmetric by construction and would inflate both sides.
    derr_all, dlab_all, dbase_all = [], [], []
    for atk in ["pd", "if", "pdr"]:
        derr_all.append(pool(dhk_series[atk], "err"))
        dlab_all.append(pool(dhk_series[atk], "stale"))
    derr_all = np.concatenate(derr_all)
    dlab_all = np.concatenate(dlab_all)
    dbase_all = pool(dhk_series["base"], "resid")
    facts["sumo.dhk_poserr_frac_stale"] = round(
        float((derr_all[dlab_all] > 0).mean()), 4)
    _t_abs, _th = abs_tpr(
        dbase_all,
        np.concatenate([pool(dhk_series[a], "resid")
                        for a in ["pd", "if", "pdr", "jo"]]),
        np.concatenate([pool(dhk_series[a], "stale")
                        for a in ["pd", "if", "pdr", "jo"]]), 0.001)
    facts["sumo.dhk_abs_tpr_001"] = round(_t_abs, 4) if _t_abs else None

    # ---- outputs ----
    facts["h_verdicts"] = verdicts
    with open(os.path.join(args.outdir, "sumo_comparison.json"), "w") as f:
        json.dump(facts, f, indent=1)
    cell_rows = []
    for k in sorted(cells):
        s = cell_stats(cells[k][0])
        if s:
            cell_rows.append({"block": k[0], "attack": k[1], "delay": k[2],
                              "vol": k[3], "mix": k[4], "seed": k[5], **s})
    with open(os.path.join(args.outdir, "sumo_comparison.csv"), "w",
              newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(cell_rows[0].keys()))
        w.writeheader()
        w.writerows(cell_rows)

    # ---- figure: side-by-side collapse + dose ----
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.6))
    # (recompute cleanly)
    sl, sm = [], []
    for vol in [0.5, 1.0, 2.0, 3.0]:
        for mix in ["slowdom", "balanced", "fastdom"]:
            m, l, _n, _c = agg(sel("S2", "pd", 3.0, vol, mix).keys())
            sl.append(l)
            sm.append(m)
    sl, sm = np.array(sl), np.array(sm)
    ax[0].scatter(sl, sm, s=18, label="SUMO S2 cells", color="#0f766e")
    xx = np.linspace(sl.min(), sl.max(), 50)
    ax[0].plot(xx, slope * xx + icept, color="#0f766e", lw=1.2,
               label=f"SUMO slope={slope:.2f}, $R^2$={r**2:.4f}")
    ax[0].scatter(dhk[:, 0], dhk[:, 1], s=18, marker="s",
                  label="DhakaSim vol+mix cells", color="#b45309", alpha=0.8)
    ax[0].plot(xx, dslope * xx + _i, color="#b45309", lw=1.2, ls="--",
               label=f"DhakaSim slope={dslope:.2f}, $R^2$={dr**2:.4f}")
    ax[0].set_xlabel("cell-mean leader speed (m/s)")
    ax[0].set_ylabel("cell-mean |error| (m)")
    ax[0].set_title("(a) Leader-speed collapse, both simulators")
    ax[0].legend(fontsize=6.5, frameon=False)
    dd = sorted(doses)
    ax[1].plot([x[0] for x in dd], [x[1] for x in dd], "o-",
               label="SUMO S1 PD", color="#0f766e")
    ax[1].set_xlabel("configured delay (s)")
    ax[1].set_ylabel("mean |error| (m)")
    ax[1].set_title("(b) Dose-response at reference point")
    ax[1].legend(fontsize=6.5, frameon=False)
    fig.tight_layout(pad=0.4)
    fig.savefig(os.path.join(args.outdir, "figures",
                             "sumo_vs_dhaka.png"), dpi=200)
    plt.close(fig)

    print(json.dumps({"facts": {k: v for k, v in facts.items()
                                if not isinstance(v, dict)},
                      "verdicts": verdicts}, indent=1))


if __name__ == "__main__":
    main()
