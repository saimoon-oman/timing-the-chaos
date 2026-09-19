"""
timing-the-chaos v4 campaign harness.

Runs DhakaSim across a structured parameter design that varies
  * road topology,
  * traffic volume (demand scaling),
  * traffic mix (slow / medium / fast composition),
  * attack variant and attack parameters,
  * replication seed,
  * simulation horizon,

and reduces every run's per-vehicle gap-error trace to a compact summary
(gap error, TTC safety, radar-residual detector, causal tau estimator and
velocity-compensated corrector) before deleting the trace.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, asdict, replace, field
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import lfilter

ROOT = Path(os.environ.get("TC_ROOT",
                           Path(__file__).resolve().parents[1]))
SIM_TEMPLATE = ROOT / "dhakasim"
WORK = ROOT / "work"
OUT = ROOT / "results"
JAVA = "java"

# ---------------------------------------------------------------- topologies

TOPOLOGIES = {
    "shahbagh": SIM_TEMPLATE / "topologies" / "shahbagh_main",
    "shankar_palashi": SIM_TEMPLATE / "topologies" / "shankar_palashi_updated",
    "straight_line": SIM_TEMPLATE / "topologies" / "straight_line_with_light" / "input",
}

# --------------------------------------------------------------- radar model
# DhakaSim's spatial fault-tolerance mode 1 reports the minimum of N noisy
# radar readings.  The detector in Section VII sees exactly that quantity.
RADAR_N = 3
RADAR_SIGMA = 0.3
WARMUP_S = 60          # discarded from every trace-derived statistic
TTC_THRESHOLD = 2.0    # s, "close following" criterion
CLOSING_SPEED_MIN = 0.1  # m/s
MAX_TRACE_ROWS = 400_000


# ------------------------------------------------------------------- config

@dataclass
class Cfg:
    # design coordinates
    block: str = "core"
    topology: str = "shahbagh"
    volume: float = 1.0            # demand multiplier
    mix: tuple = (50, 25, 25)      # slow / medium / fast percentages
    variant: str = "none"          # none|PD|IF|SS|JO|PDr|JM
    horizon: int = 300
    seed: int = 0

    # attack parameters
    delay_mean: float = 3.0
    delay_probability: float = 1.0
    jitter_magnitude: float = 0.5
    jitter_only_magnitude: int = 2
    packet_loss_rate: float = 0.2
    single_shot_delay: float = 5.0
    single_shot_time: int = 60
    delay_spread: float = 1.0
    attack_start_time: int = 1
    attack_end_time: int = -1
    attack_strip_mod: int = -1

    # fixed model parameters
    strip_width: float = 0.5
    position_buffer_size: int = 10
    cf_model: int = 0
    ttc_threshold_sim: float = 0.6

    def scenario_id(self) -> int:
        return {"none": -1, "PD": 0, "IF": 1, "SS": 2,
                "JO": 3, "PDr": 4, "JM": 5}[self.variant]

    def key(self) -> str:
        d = asdict(self)
        d["mix"] = list(self.mix)
        return hashlib.sha1(json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]

    def label(self) -> str:
        parts = [self.topology, f"v{self.volume:g}",
                 "m" + "-".join(str(x) for x in self.mix),
                 self.variant, f"T{self.horizon}", f"s{self.seed}"]
        if self.variant in ("PD", "IF", "JM"):
            parts.insert(4, f"d{self.delay_mean:g}")
        if self.variant == "IF":
            parts.insert(5, f"p{self.delay_probability:g}")
        if self.variant in ("PDr", "JM"):
            parts.insert(5, f"l{self.packet_loss_rate:g}")
        if self.variant in ("JO", "JM"):
            parts.insert(5, f"j{self.jitter_magnitude:g}")
        if self.variant == "SS":
            parts.insert(4, f"d{self.single_shot_delay:g}")
        return "_".join(parts)

    def param_lines(self) -> list[str]:
        slow, med, fast = self.mix
        sc = self.scenario_id()
        m = {
            "RandomSeed": str(self.seed),
            "SimulationSpeed": "1",
            "SimulationEndTime": str(self.horizon),
            "PixelPerMeter": "15",
            "EncounterPerAccident": "0.01",
            "StripWidth": f"{self.strip_width:.1f}",
            "FootpathStripWidth": "0.5",
            "MaximumSpeed": "100.0",
            "GUIMode": "Off",
            "DebugMode": "Off",
            "ObjectMode": "On",
            "TraceMode": "Off",
            "SignalChangeDuration": "1",
            "DefaultTranslateX": "-1340",
            "DefaultTranslateY": "-580",
            "CenteredView": "On",
            "SlowVehicle": str(slow),
            "MediumVehicle": str(med),
            "FastVehicle": str(fast),
            "DemandType": "2",
            "TTC_Threshold": f"{self.ttc_threshold_sim:.1f}",
            "LowRate": "200",
            "MediumRate": "600",
            "HighRate": "1000",
            "VehicleGenerationRate": "1",
            "ErrorMode": "Off" if self.variant == "none" else "On",
            "FTMethod": "4",
            "NoOfReadings": "2",
            "MFactor": "0.0",
            "ALPHA": "0.92",
            "BETA": "0.1",
            "ETA": "0",
            "DLC_model": "0",
            "CF_model": str(self.cf_model),
            "AcrossPedestrianMode": "On",
            "AlongPedestrianMode": "On",
            "AcrossPedestrianLimit": "1",
            "AcrossPedestrianPerHour": "400",
            "AlongPedestrianPerHour": "50",
            "AlongPedestrianPercentage": "99",
            "DensityPercentage": "50",
            "PedestrianWeight": "2",
            "PedestrianRandomLaneChangePercentage": "20",
            "PedestrianLeftBiasPercentage": "40",
            "PenaltyWait": "On",
            "ConsiderMinimum": "On",
            "NoOfRoutes": "12",
            "BrakeHard": "Off",
            "AttackScenario": str(sc),
            "DelayProbability": f"{self.delay_probability:.2f}",
            "JitterMagnitude": f"{self.jitter_magnitude:.2f}",
            "DelayMean": f"{self.delay_mean:.2f}",
            "DelaySpread": f"{self.delay_spread:.2f}",
            "PacketLossRate": f"{self.packet_loss_rate:.2f}",
            "AttackStripMod": str(self.attack_strip_mod),
            "AttackStartTime": str(self.attack_start_time),
            "AttackEndTime": str(self.attack_end_time),
            "JitterOnlyMagnitude": str(self.jitter_only_magnitude),
            "SingleShotDelay": f"{self.single_shot_delay:.2f}",
            "SingleShotTime": str(self.single_shot_time),
            "PositionBufferSize": str(self.position_buffer_size),
        }
        return [f"{k} {v}" for k, v in m.items()]


# ------------------------------------------------------------- sim workspace

def make_workspace(wid: int) -> Path:
    # keyed by pid as well as worker id, so two concurrent campaign processes
    # can never share an input/ or statistics/ directory
    d = WORK / f"p{os.getpid()}_w{wid}"
    if d.exists():
        shutil.rmtree(d)
    (d / "input").mkdir(parents=True)
    (d / "statistics").mkdir(parents=True)
    (d / "libraries").mkdir(parents=True)
    (d / "out" / "artifacts" / "DhakaSim_jar").mkdir(parents=True)
    shutil.copy(SIM_TEMPLATE / "out/artifacts/DhakaSim_jar/DhakaSim.jar",
                d / "out/artifacts/DhakaSim_jar/DhakaSim.jar")
    for j in (SIM_TEMPLATE / "libraries").glob("*.jar"):
        shutil.copy(j, d / "libraries" / j.name)
    return d


def write_demand(src: Path, dst: Path, scale: float):
    lines = src.read_text().split("\n")
    n = int(lines[0].strip())
    out = [str(n)]
    for ln in lines[1:1 + n]:
        p = ln.split()
        out.append(f"{p[0]} {p[1]} {max(1, int(round(float(p[2]) * scale)))}")
    dst.write_text("\n".join(out) + "\n")


def stage_topology(ws: Path, cfg: Cfg):
    src = TOPOLOGIES[cfg.topology]
    for f in ("link.txt", "node.txt", "path.txt"):
        shutil.copy(src / f, ws / "input" / f)
    write_demand(src / "demand.txt", ws / "input" / "demand.txt", cfg.volume)
    (ws / "input" / "parameter.txt").write_text("\n".join(cfg.param_lines()) + "\n")


# --------------------------------------------------------------- trace maths

def _summarise_trace(path: Path, cfg: Cfg, rng: np.random.Generator) -> dict:
    """Reduce gap_errors.csv to gap-error, TTC, detector and corrector stats."""
    if not path.exists():
        return {}
    try:
        arr = pd.read_csv(path, engine="c", on_bad_lines="skip",
                          dtype=np.float64).to_numpy(np.float64, copy=False)
    except Exception:
        try:
            arr = np.genfromtxt(path, delimiter=",", skip_header=1,
                                dtype=np.float64, invalid_raise=False)
        except Exception:
            return {}
    if arr.ndim != 2 or arr.shape[0] == 0:
        return {}

    step, leader, follower = arr[:, 0], arr[:, 1], arr[:, 2]
    gap_err, perceived, true_gap = arr[:, 3], arr[:, 4], arr[:, 5]
    v_lead, v_foll = arr[:, 6], arr[:, 7]

    # discard warm-up, sentinel leaders and the "no leader ahead" sentinel gap
    ok = (step > WARMUP_S) & (leader >= 0) & (np.abs(true_gap) < 1e6) \
        & (np.abs(perceived) < 1e6) & np.isfinite(gap_err)
    if ok.sum() < 50:
        return {"trace_rows": int(arr.shape[0]), "usable_rows": int(ok.sum())}

    idx = np.flatnonzero(ok)
    if idx.size > MAX_TRACE_ROWS:
        idx = idx[rng.choice(idx.size, MAX_TRACE_ROWS, replace=False)]
        idx.sort()

    step = step[idx]; follower = follower[idx]
    gap_err = gap_err[idx]; perceived = perceived[idx]; true_gap = true_gap[idx]
    v_lead = v_lead[idx]; v_foll = v_foll[idx]

    res: dict = {}
    a = np.abs(gap_err)
    res.update(
        trace_rows=int(arr.shape[0]),
        usable_rows=int(idx.size),
        gap_err_mean=float(gap_err.mean()),
        gap_err_mae=float(a.mean()),
        gap_err_median_abs=float(np.median(a)),
        gap_err_p95_abs=float(np.percentile(a, 95)),
        gap_err_max=float(a.max()),
        gap_err_std=float(gap_err.std()),
        gap_err_neg_frac=float((gap_err < 0).mean()),
        # strip-quantisation displacement, Eq. (5), in the cell's own units
        strip_offset_mean=float(np.abs(np.floor(perceived / cfg.strip_width)
                                       - np.floor(true_gap / cfg.strip_width)).mean()),
        # and in two fixed reference units, so cells with different ws compare
        offset_units_half=float(np.abs(np.floor(perceived / 0.5)
                                       - np.floor(true_gap / 0.5)).mean()),
        offset_units_lane=float(np.abs(np.floor(perceived / 3.5)
                                       - np.floor(true_gap / 3.5)).mean()),
        leader_speed_mean=float(v_lead.mean()),
        follower_speed_mean=float(v_foll.mean()),
        moving_frac=float((v_lead > 0.5).mean()),
    )

    # ---- time-to-collision, true vs perceived ---------------------------
    closing = v_foll - v_lead
    cl = closing > CLOSING_SPEED_MIN
    if cl.sum() > 10:
        ttc_true = np.clip(true_gap[cl], 0, None) / closing[cl]
        ttc_perc = np.clip(perceived[cl], 0, None) / closing[cl]
        res.update(
            ttc_pairs=int(cl.sum()),
            ttc_true_median=float(np.median(ttc_true)),
            ttc_perc_median=float(np.median(ttc_perc)),
            ttc_bias_median=float(np.median(ttc_perc) - np.median(ttc_true)),
            ttc_true_viol=float((ttc_true < TTC_THRESHOLD).mean()),
            ttc_perc_viol=float((ttc_perc < TTC_THRESHOLD).mean()),
        )

    # ---- radar residual detector, Section VII-A -------------------------
    # radar = min of RADAR_N Gaussian readings about the true range
    noise = rng.normal(0.0, RADAR_SIGMA, size=(RADAR_N, idx.size))
    radar = true_gap + noise.min(axis=0)
    residual = radar - perceived            # positive under delay, Eq. (4)
    gate = np.abs(v_lead) > 0.5             # motion gate
    res.update(
        resid_mean=float(residual[gate].mean()) if gate.any() else float("nan"),
        resid_p50=float(np.median(residual[gate])) if gate.any() else float("nan"),
        gate_frac=float(gate.mean()),
    )
    # label: the attack actually displaced this sample
    displaced = np.abs(gap_err) > 0.25
    res["displaced_frac"] = float(displaced.mean())
    # bounded sample of (residual, displaced) on gated steps, for pooled ROC
    gi = np.flatnonzero(gate)
    if gi.size > 60_000:
        gi = rng.choice(gi, 60_000, replace=False)
    res["_roc"] = (residual[gi].astype(np.float32),
                   displaced[gi].astype(np.bool_))

    # ---- causal tau estimator + velocity-compensated correction ---------
    # Evaluated on a bounded random sample of complete follower time series so
    # that the causal (per-vehicle, trailing-window) filters stay tractable.
    res.update(_defence_metrics(step, follower, perceived, true_gap, gap_err,
                                v_lead, radar, gate, cfg, rng))
    return res


DEF_MAX_FOLLOWERS = 120
DEF_TAU_WIN = 30
DEF_TAU_ALPHA = 0.2
DEF_MEDIAN_K = 5


def _causal_rolling_median(x: np.ndarray, k: int) -> np.ndarray:
    """Trailing median over the last <=k samples, causal, O(n log k)."""
    n = x.size
    if n == 0:
        return x
    k = min(k, n)
    pad = np.concatenate([np.full(k - 1, x[0]), x])
    win = np.lib.stride_tricks.sliding_window_view(pad, k)
    out = np.median(win, axis=1)
    # first k-1 samples must only see what has arrived so far
    for i in range(min(k - 1, n)):
        out[i] = np.median(x[:i + 1])
    return out


def _ewma(x: np.ndarray, alpha: float) -> np.ndarray:
    if x.size == 0:
        return x
    y = lfilter([alpha], [1.0, -(1.0 - alpha)], x, zi=[(1.0 - alpha) * x[0]])[0]
    return y


def _defence_metrics(step, follower, perceived, true_gap, gap_err,
                     v_lead, radar, gate, cfg, rng) -> dict:
    ids = np.unique(follower)
    if ids.size > DEF_MAX_FOLLOWERS:
        ids = rng.choice(ids, DEF_MAX_FOLLOWERS, replace=False)
    sel = np.isin(follower, ids)
    if sel.sum() < 100:
        return {}
    order = np.flatnonzero(sel)[np.lexsort((step[sel], follower[sel]))]
    f_s = follower[order]
    st_s = step[order]
    ge_s = gap_err[order]
    vl_s = v_lead[order]
    gh_s = perceived[order]
    g_s = true_gap[order]
    rd_s = radar[order]
    gt_s = gate[order]

    resid = rd_s - gh_s
    tau_inst = np.clip(np.where(gt_s, resid / np.maximum(vl_s, 0.5), 0.0), 0.0, 15.0)

    tau_hat = np.empty_like(tau_inst)
    med_gap = np.empty_like(gh_s)
    bounds = np.flatnonzero(np.diff(f_s)).tolist() + [f_s.size - 1]
    start = 0
    for b in bounds:
        sl = slice(start, b + 1)
        tau_hat[sl] = _ewma(_causal_rolling_median(tau_inst[sl], DEF_TAU_WIN),
                            DEF_TAU_ALPHA)
        med_gap[sl] = _causal_rolling_median(gh_s[sl], DEF_MEDIAN_K)
        start = b + 1

    # detector gate: fire only where the residual exceeds one strip
    fire = gt_s & (resid > cfg.strip_width)
    tau_gated = np.where(fire, tau_hat, 0.0)

    raw = np.abs(ge_s).mean()
    oracle = np.abs(ge_s + vl_s * _oracle_tau(cfg)).mean()
    causal = np.abs(ge_s + vl_s * tau_hat).mean()
    gated = np.abs(ge_s + vl_s * tau_gated).mean()
    fixed1 = np.abs(ge_s + vl_s * 1.0).mean()
    median_mae = np.abs(med_gap - g_s).mean()

    def red(x):
        return float(100.0 * (raw - x) / raw) if raw > 1e-9 else float("nan")

    return dict(
        def_samples=int(f_s.size),
        def_followers=int(ids.size),
        tau_hat_median=float(np.median(tau_hat[gt_s])) if gt_s.any() else float("nan"),
        def_raw_mae=float(raw),
        def_oracle_mae=float(oracle),
        def_causal_mae=float(causal),
        def_gated_mae=float(gated),
        def_fixed1_mae=float(fixed1),
        def_median_mae=float(median_mae),
        red_oracle=red(oracle),
        red_causal=red(causal),
        red_gated=red(gated),
        red_fixed1=red(fixed1),
        red_median=red(median_mae),
        detector_fire_rate=float(fire.mean()),
    )


def _oracle_tau(cfg: Cfg) -> float:
    if cfg.variant == "PD":
        return cfg.delay_mean
    if cfg.variant == "IF":
        return cfg.delay_mean * cfg.delay_probability
    if cfg.variant == "JM":
        return cfg.delay_mean * cfg.delay_probability
    return 0.0


# ------------------------------------------------------------------ run one

CONSOLE_KEYS = [
    ("net_speed", "speed:"),
    ("net_waiting", "waiting time:"),
    ("mot_speed", "motorized speed:"),
    ("mot_waiting", "motorized waiting time:"),
    ("nonmot_speed", "non-motorized speed:"),
    ("nonmot_waiting", "non-motorized waiting time:"),
]


def parse_console(text: str) -> dict:
    out = {}
    for line in text.splitlines():
        s = line.strip()
        for key, pref in CONSOLE_KEYS:
            if s.startswith(pref) and key not in out:
                try:
                    out[key] = float(s.split(":", 1)[1])
                except ValueError:
                    pass
        if s.startswith("Collision count:"):
            out["collisions"] = int(float(s.split(":")[1]))
        if s.startswith("Accident count:"):
            out["accidents"] = int(float(s.split(":")[1]))
        if s.startswith("Gap error count:"):
            out["sim_gap_count"] = int(float(s.split(":")[1]))
        if s.startswith("Mean absolute gap error:"):
            out["sim_gap_mae"] = float(s.split(":")[1])
    return out


def run_one(cfg: Cfg, ws: Path, keep_roc: bool = False) -> dict:
    stats = ws / "statistics"
    for f in stats.glob("*"):
        try:
            f.unlink()
        except OSError:
            pass
    stage_topology(ws, cfg)

    cp = os.pathsep.join([
        str(ws / "out/artifacts/DhakaSim_jar/DhakaSim.jar"),
        str(ws / "libraries/commons-math3-3.6.1.jar"),
        str(ws / "libraries/jmathio.jar"),
    ])
    env = dict(os.environ, JAVA_TOOL_OPTIONS="")
    t0 = time.time()
    rec = {**{k: v for k, v in asdict(cfg).items() if k != "mix"},
           "mix_slow": cfg.mix[0], "mix_medium": cfg.mix[1], "mix_fast": cfg.mix[2],
           "label": cfg.label(), "key": cfg.key()}
    try:
        p = subprocess.run([JAVA, "-Xmx3g", "-Djava.awt.headless=true",
                            "-cp", cp, "thesisfinal.DhakaSim"],
                           cwd=ws, capture_output=True, text=True,
                           timeout=max(600, cfg.horizon * 3), env=env)
        rec.update(parse_console(p.stdout + p.stderr))
        rec["status"] = "ok"
    except subprocess.TimeoutExpired:
        rec["status"] = "timeout"
        rec["elapsed"] = round(time.time() - t0, 2)
        return rec
    except Exception as e:                                   # pragma: no cover
        rec["status"] = f"error:{e}"
        rec["elapsed"] = round(time.time() - t0, 2)
        return rec

    rng = np.random.default_rng(abs(hash(cfg.key())) % (2 ** 32))
    summ = _summarise_trace(stats / "gap_errors.csv", cfg, rng)
    # the parsed trace must contain exactly the rows the simulator says it
    # wrote; any shortfall means the file was truncated or interleaved with
    # another writer, and the run is not usable
    declared = rec.get("sim_gap_count")
    parsed = summ.get("trace_rows")
    if declared is not None and parsed is not None and int(parsed) != int(declared):
        rec["status"] = f"trace-mismatch:{parsed}!={declared}"
        rec["elapsed"] = round(time.time() - t0, 2)
        for f in stats.glob("*"):
            try:
                f.unlink()
            except OSError:
                pass
        return rec
    roc = summ.pop("_roc", None)
    rec.update(summ)
    rec["elapsed"] = round(time.time() - t0, 2)

    if keep_roc and roc is not None:
        d = OUT / "roc"
        d.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(d / f"{cfg.key()}.npz",
                            residual=roc[0], displaced=roc[1])
        rec["roc_file"] = f"{cfg.key()}.npz"

    for f in stats.glob("*"):
        try:
            f.unlink()
        except OSError:
            pass
    return rec
