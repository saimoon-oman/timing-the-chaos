"""Single-run TraCI driver for the SUMO cross-validation campaign.

Perception-in-the-loop replication of DhakaSim FT_METHOD=4: every vehicle
that has an immediate leader is driven by the external Gipps controller
(scripts/gipps_controller.py) in BOTH baseline and attacked runs. The ONLY
difference between baseline and attack is the position-history lookback
(delay 0 vs configured staleness) -- exactly the isolation FT_METHOD=4
provides in DhakaSim. Leaders without a leader ahead, and all junction /
signal / lane-change behavior, remain SUMO-native.

Speed application (see --apply):
  slowdown (default): traci.vehicle.slowDown(v, 1.0) under the default
    speed mode, so signals, right-of-way and SUMO's own safe-speed floor
    stay enforced. Mild 1-step smoothing dilutes the attack effect
    uniformly in baseline and attack: any confirmed effect is a LOWER
    bound, which is the conservative direction for cross-validation.
  setspeed: traci.vehicle.setSpeed (exact actuation). Pilot (S0) compares
    both and locks one in; see docs.

Output per run directory:
  gap_errors.csv  -- simStep,leaderId,followerId,gapError,perceivedGap,
                     trueGap,leaderSpeed,followerSpeed  (DhakaSim schema)
                     + sameEdge flag (1 when ego and leader share an edge;
                     metrics use sameEdge==1 only)
  sidecar.json    -- run config, aggregates, wall time, SUMO version

Usage:
    python run_sumo.py --sumocfg ... --demand ... --outdir ... --attack pd
        --delay 3 --seed 7 [--apply slowdown|setspeed]
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gipps_controller as G

WARMUP = 60            # steps discarded in metrics (matches paper)
LEADER_DIST = 120.0    # max lookahead for immediate leader (m)
SS_STEP = 150          # single-shot firing step (of a 300-step horizon)

SCENARIOS = {"none", "pd", "if", "ss", "jo", "pdr", "jm"}


def parse_collisions(coll_path):
    """Count collisions by type. Junction collisions are OSM-geometry
    right-of-way artifacts, unrelated to longitudinal control; the
    attack-relevant class is non-junction (rear-end/following) collisions,
    which is what H4 tests. Both counts are recorded."""
    types = []
    if coll_path and os.path.exists(coll_path):
        try:
            for ev in ET.parse(coll_path).getroot().iter("collision"):
                types.append(ev.get("type", "unknown"))
        except ET.ParseError:
            pass
    return {"total": len(types),
            "junction": sum(1 for t in types if t == "junction"),
            "non_junction": sum(1 for t in types if t != "junction")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo-bin", required=True)
    ap.add_argument("--net", required=True)
    ap.add_argument("--demand", required=True)
    ap.add_argument("--vtypes", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--attack", default="none", choices=sorted(SCENARIOS))
    ap.add_argument("--delay", type=float, default=3.0)
    ap.add_argument("--prob", type=float, default=0.5)
    ap.add_argument("--jitter", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--horizon", type=float, default=300.0)
    ap.add_argument("--apply", default="slowdown", choices=["slowdown",
                                                            "setspeed"])
    ap.add_argument("--ss-step", type=int, default=SS_STEP)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--port", type=int, default=0,
                    help="TraCI port; 0 = pick a free port (required for "
                         "parallel runs sharing one machine)")
    ap.add_argument("--lateral-resolution", type=float, default=None,
                    help="SUMO --lateral-resolution (required for SL2015 "
                         "sublane model; e.g. 0.5 to mirror strip width)")
    args = ap.parse_args()

    G.check_vtypes_against_xml(args.vtypes)
    os.makedirs(args.outdir, exist_ok=True)
    rng = random.Random(args.seed)
    delay_steps = max(1, int(round(args.delay / G.TIME_STEP)))

    # --- write a concrete sumocfg for this run (ABSOLUTE paths: SUMO
    # resolves relative paths against the caller CWD, not the config dir) ---
    coll_path = os.path.join(args.outdir, "collisions.xml")
    cfg_path = os.path.join(args.outdir, "run.sumocfg")
    net_abs = os.path.abspath(args.net)
    dem_abs = os.path.abspath(args.demand)
    vty_abs = os.path.abspath(args.vtypes)
    with open(cfg_path, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<configuration>\n'
                '    <input>\n'
                f'        <net-file value="{net_abs}"/>\n'
                f'        <route-files value="{dem_abs}"/>\n'
                f'        <additional-files value="{vty_abs}"/>\n'
                '    </input>\n'
                '    <time>\n        <begin value="0"/>\n'
                f'        <end value="{args.horizon}"/>\n'
                '        <step-length value="1.0"/>\n    </time>\n'
                '    <processing>\n        <time-to-teleport value="120"/>\n'
                '        <collision.check-junctions value="true"/>\n'
                '        <collision.mingap-factor value="0"/>\n'
                f'        <collision-output value="{coll_path}"/>\n'
                + (f'        <lateral-resolution value="{args.lateral_resolution}"/>\n'
                   if args.lateral_resolution is not None else '') +
                '    </processing>\n'
                '    <random_number>\n'
                f'        <seed value="{args.seed}"/>\n'
                '    </random_number>\n</configuration>\n')

    import sumo as sumo_pkg  # noqa: E402  (sets SUMO_HOME)
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
    import traci  # noqa: E402

    binary = os.path.join(os.path.dirname(args.sumo_bin),
                          "sumo-gui.exe" if args.gui else "sumo.exe")
    if not os.path.exists(binary):
        binary = args.sumo_bin
    port = args.port
    if port == 0:
        import socket
        with socket.socket() as s:
            s.bind(("", 0))
            port = s.getsockname()[1]
    traci.start([binary, "-c", cfg_path, "--no-warnings", "--no-step-log"],
                port=port)
    try:
        hist = {}   # vid -> HistoryBuffer
        rows = []
        n_controlled = 0
        t0 = time.time()
        step = 0
        while traci.simulation.getTime() < args.horizon:
            vids = sorted(traci.vehicle.getIDList())
            # 1) record: positions at top of loop = pre-move state of this
            #    step (mirrors recordPosition-before-movement in DhakaSim).
            snap = {}
            for vid in vids:
                try:
                    x, y = traci.vehicle.getPosition(vid)
                    sp = traci.vehicle.getSpeed(vid)
                    ed = traci.vehicle.getRoadID(vid)
                except traci.TraCIException:
                    continue
                snap[vid] = (x, y, sp, ed)
                hist.setdefault(vid, G.HistoryBuffer()).record(
                    step, x, y, sp)
            # 2) control + log every follower with an immediate leader.
            for vid in vids:
                if vid not in snap:
                    continue
                try:
                    lead = traci.vehicle.getLeader(vid, LEADER_DIST)
                except traci.TraCIException:
                    continue
                if lead is None:
                    # no leader: free-flow Gipps term keeps the controller
                    # uniform across baseline and attack (no logging: no pair)
                    vtype = traci.vehicle.getTypeID(vid)
                    a, _b, vd, _ln = G.CLASS_PARAMS.get(
                        vtype, G.CLASS_PARAMS["med"])
                    try:
                        va = G.gipps_free(snap[vid][2], a, vd)
                        apply_speed(vid, va, args.apply)
                    except traci.TraCIException:
                        pass
                    continue
                lid, true_gap = lead
                if lid not in snap:
                    continue
                ex, ey, ev, eedge = snap[vid]
                lx, ly, lv, ledge = snap[lid]
                same_edge = 1 if (eedge == ledge
                                  and not eedge.startswith(":")) else 0
                vtype = traci.vehicle.getTypeID(vid)
                ltype = traci.vehicle.getTypeID(lid)
                a, b, vd, _ln = G.CLASS_PARAMS.get(
                    vtype, G.CLASS_PARAMS["med"])
                _la, lb, _lvd, llen = G.CLASS_PARAMS.get(
                    ltype, G.CLASS_PARAMS["med"])
                bhat = (lb + b) / 2.0
                lb_ = hist.get(lid)
                entry = (lb_.lookup(step, G.scenario_delay(
                    args.attack, delay_steps)) if lb_ is not None else None)
                perc, err = G.perceived_gap(
                    args.attack, rng, true_gap, (ex, ey), llen, entry,
                    args.jitter, prob=args.prob,
                    single_shot_step=args.ss_step, step=step)
                v_next = G.gipps_next(ev, perc, lv, a, b, bhat, vd)
                try:
                    apply_speed(vid, v_next, args.apply)
                    n_controlled += 1
                except traci.TraCIException:
                    pass
                rows.append((step, lid, vid, round(err, 4), round(perc, 4),
                             round(true_gap, 4), round(lv, 2),
                             round(ev, 2), same_edge))
            traci.simulationStep()
            step += 1
        wall = time.time() - t0
    finally:
        traci.close()

    gap_path = os.path.join(args.outdir, "gap_errors.csv")
    with open(gap_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["simStep", "leaderId", "followerId", "gapError",
                    "perceivedGap", "trueGap", "leaderSpeed",
                    "followerSpeed", "sameEdge"])
        w.writerows(rows)

    n_coll = parse_collisions(coll_path)
    post = [r for r in rows if r[0] >= WARMUP and r[8] == 1]
    errs = [abs(r[3]) for r in post]
    sidecar = {
        "attack": args.attack, "delay": args.delay, "prob": args.prob,
        "jitter": args.jitter, "seed": args.seed, "horizon": args.horizon,
        "apply": args.apply, "warmup": WARMUP,
        "vtypes_file": os.path.basename(args.vtypes),
        "lateral_resolution": args.lateral_resolution,
        "n_gap_rows": len(rows), "n_postwarm_sameedge": len(post),
        "mae_all": (sum(errs) / len(errs)) if errs else None,
        "n_controlled": n_controlled, "collisions": n_coll,
        "wall_seconds": round(wall, 1),
        "sumo_version": "1.27.1",
    }
    with open(os.path.join(args.outdir, "sidecar.json"), "w") as f:
        json.dump(sidecar, f, indent=1)
    print(f"[{os.path.basename(args.outdir)}] rows={len(rows)} "
          f"post={len(post)} mae={sidecar['mae_all']} "
          f"coll={n_coll} wall={wall:.1f}s")


def apply_speed(vid, v_next, mode):
    import traci
    if mode == "setspeed":
        traci.vehicle.setSpeed(vid, v_next)
    else:
        traci.vehicle.slowDown(vid, v_next, 1.0)


if __name__ == "__main__":
    main()
