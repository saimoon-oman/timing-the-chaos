"""Parallel campaign runner for the SUMO cross-validation (S1/S2/S3).

Builds the full cell list, generates missing demand files, runs each cell
as an isolated subprocess (free TraCI port each), skips completed cells
(resume), and aggregates sidecars into sumo_results_summary.csv whose
columns mirror the DhakaSim results_summary.csv fields consumed by the
analysis notebooks.

Cell design (matches the pre-registered plan):
  S1 dose+sign : PD delays {1,2,3,5,7}s + matched baseline, 10 seeds
                 (demand: 1x volume, balanced mix)
  S2 collapse  : volumes {0.5,1,2,3}x x mixes {slow-dom,balanced,fast-dom}
                 x {baseline, PD 3s}, 10 seeds
  S3 variants  : IF(p=0.5), PDr(p=0.3), JO(sigma=1.0) at reference point,
                 10 seeds (baselines shared with S1)

Usage:
    python run_campaign.py --workers 4 [--dry-run] [--only S1]
Outputs under <root>/results_sumo/ (default: this repo's results_sumo/).
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
NET = os.path.join(REPO, "networks", "shahbagh.net.xml")
VTYPES = os.path.join(REPO, "configs", "vtypes.add.xml")
SUMO_BIN = os.environ.get("SUMO_BIN", "sumo")

VOLUMES = {0.5: 4.0, 1.0: 2.0, 2.0: 1.0, 3.0: 0.67}   # multiplier -> period
MIXES = {"slowdom": "80,15,5",
         "balanced": "50,25,25",
         "fastdom": "20,30,50"}
SEEDS = [7, 11, 13, 17, 19, 23, 29, 31, 37, 41]
DOSES = [1.0, 2.0, 3.0, 5.0, 7.0]


def demand_file(root, vol, mix, seed):
    return os.path.join(root, "demand",
                        f"demand_{vol}vol_{mix}_seed{seed}.rou.xml")


def ensure_demand(root, vol, mix, seed, py, duarouter):
    path = demand_file(root, vol, mix, seed)
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cmd = [py, os.path.join(HERE, "make_demand.py"),
           "--net", NET, "--out", path, "--horizon", "300",
           "--period", str(VOLUMES[vol]), "--mix", MIXES[mix],
           "--seed", str(seed), "--duarouter", duarouter]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(path):
        raise RuntimeError(f"demand failed {vol}/{mix}/{seed}:\n"
                           + r.stdout[-1500:] + r.stderr[-1500:])
    return path


def build_cells():
    cells = []
    # S1: dose-response at reference point (1x, balanced)
    for s in SEEDS:
        cells.append(dict(block="S1", attack="none", delay=0.0, prob=0.5,
                          jitter=1.0, vol=1.0, mix="balanced", seed=s))
        for d in DOSES:
            cells.append(dict(block="S1", attack="pd", delay=d, prob=0.5,
                              jitter=1.0, vol=1.0, mix="balanced", seed=s))
    # S2: collapse grid
    for vol in VOLUMES:
        for mix in MIXES:
            for s in SEEDS:
                cells.append(dict(block="S2", attack="none", delay=0.0,
                                  prob=0.5, jitter=1.0, vol=vol, mix=mix,
                                  seed=s))
                cells.append(dict(block="S2", attack="pd", delay=3.0,
                                  prob=0.5, jitter=1.0, vol=vol, mix=mix,
                                  seed=s))
    # S3: variant spot-checks (share S1 reference baselines? No: own seeds
    # reuse the SAME demand files as S1 reference for matching).
    for s in SEEDS:
        cells.append(dict(block="S3", attack="if", delay=3.0, prob=0.5,
                          jitter=1.0, vol=1.0, mix="balanced", seed=s))
        cells.append(dict(block="S3", attack="pdr", delay=3.0, prob=0.3,
                          jitter=1.0, vol=1.0, mix="balanced", seed=s))
        cells.append(dict(block="S3", attack="jo", delay=3.0, prob=0.5,
                          jitter=1.0, vol=1.0, mix="balanced", seed=s))
    return cells


def cell_id(c):
    a = c["attack"]
    if a == "none":
        tag = "baseline"
    elif a == "pd":
        tag = f"pd_d{c['delay']:g}"
    elif a == "if":
        tag = f"if_d{c['delay']:g}_p{c['prob']:g}"
    elif a == "pdr":
        tag = f"pdr_p{c['prob']:g}"
    elif a == "jo":
        tag = f"jo_j{c['jitter']:g}"
    else:
        tag = f"{a}_d{c['delay']:g}"
    return f"{c['block']}_{tag}_v{c['vol']:g}_{c['mix']}_seed{c['seed']}"


def run_one(args):
    cell, root, py, vtypes, latres, suffix = args
    outdir = os.path.join(root, "runs", cell_id(cell) + suffix)
    sidecar = os.path.join(outdir, "sidecar.json")
    if os.path.exists(sidecar):
        try:
            with open(sidecar) as f:
                sc = json.load(f)
            if sc.get("n_gap_rows", 0) > 0:
                return (cell_id(cell), "skipped", 0.0)
        except (json.JSONDecodeError, OSError):
            pass
    dem = demand_file(root, cell["vol"], cell["mix"], cell["seed"])
    cmd = [py, os.path.join(HERE, "run_sumo.py"),
           "--sumo-bin", SUMO_BIN, "--net", NET, "--demand", dem,
           "--vtypes", vtypes, "--outdir", outdir,
           "--attack", cell["attack"], "--delay", str(cell["delay"]),
           "--prob", str(cell["prob"]), "--jitter", str(cell["jitter"]),
           "--seed", str(cell["seed"]), "--horizon", "300",
           "--apply", "slowdown", "--port", "0"]
    if latres is not None:
        cmd += ["--lateral-resolution", str(latres)]
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True)
    dt = time.time() - t0
    if r.returncode != 0:
        return (cell_id(cell) + suffix,
                "FAILED: " + (r.stderr[-500:] or r.stdout[-500:]), dt)
    return (cell_id(cell) + suffix, "done", dt)


def aggregate(root):
    rows = []
    for d in sorted(os.listdir(os.path.join(root, "runs"))):
        sc_path = os.path.join(root, "runs", d, "sidecar.json")
        if not os.path.exists(sc_path):
            continue
        with open(sc_path) as f:
            sc = json.load(f)
        # recover cell factors from directory name
        parts = d.split("_")
        rows.append({
            "config_id": d,
            "simulator": "sumo",
            "block": parts[0],
            "attack_scenario": sc["attack"],
            "delay_mean": sc["delay"],
            "random_seed": sc["seed"],
            "status": "completed",
            "gap_error_count": sc["n_gap_rows"],
            "mean_abs_gap_error": sc["mae_all"],
            "collision_count": sc["collisions"]["non_junction"],
            "collision_junction": sc["collisions"]["junction"],
            "elapsed_seconds": sc["wall_seconds"],
        })
    out = os.path.join(root, "sumo_results_summary.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return out, len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(REPO, "results_sumo"))
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", default=None, help="S1, S2 or S3")
    ap.add_argument("--duarouter", default="duarouter")
    ap.add_argument("--vtypes", default=None,
                    help="vType file override (default: configs/vtypes.add.xml)")
    ap.add_argument("--latres", type=float, default=None,
                    help="lateral resolution for SL2015 (omit for LC2013)")
    ap.add_argument("--suffix", default="",
                    help="suffix appended to run dir names (e.g. _SL2015)")
    ap.add_argument("--seeds", default=None,
                    help="comma-separated seed subset (default: all 10)")
    args = ap.parse_args()

    vtypes = args.vtypes or VTYPES
    if args.seeds:
        want = {int(s) for s in args.seeds.split(",")}

    py = sys.executable
    cells = build_cells()
    if args.only:
        cells = [c for c in cells if c["block"] == args.only.upper()]
    if args.seeds:
        cells = [c for c in cells if c["seed"] in want]
    print(f"cells: {len(cells)} "
          f"(S1={sum(1 for c in cells if c['block']=='S1')}, "
          f"S2={sum(1 for c in cells if c['block']=='S2')}, "
          f"S3={sum(1 for c in cells if c['block']=='S3')})"
          + (f" vtypes={os.path.basename(vtypes)}" if args.vtypes else "")
          + (f" latres={args.latres}" if args.latres else "")
          + (f" suffix={args.suffix!r}" if args.suffix else ""))
    if args.dry_run:
        for c in cells[:5]:
            print(" ", cell_id(c))
        print("  ...")
        return

    os.makedirs(os.path.join(args.root, "runs"), exist_ok=True)
    # ensure all demand files first (serial, fast)
    needed = {(c["vol"], c["mix"], c["seed"]) for c in cells}
    print(f"ensuring {len(needed)} demand files...")
    for i, (vol, mix, seed) in enumerate(sorted(needed)):
        ensure_demand(args.root, vol, mix, seed, py, args.duarouter)
        if (i + 1) % 20 == 0:
            print(f"  {i+1}/{len(needed)}")

    print(f"running {len(cells)} cells on {args.workers} workers...")
    done = failed = skipped = 0
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(run_one, (c, args.root, py, vtypes,
                                    args.latres, args.suffix)): c
                for c in cells}
        for fut in as_completed(futs):
            cid, status, dt = fut.result()
            if status == "done":
                done += 1
            elif status == "skipped":
                skipped += 1
            else:
                failed += 1
                print(f"  FAIL {cid}: {status[:200]}")
            if (done + failed + skipped) % 20 == 0:
                el = time.time() - t0
                print(f"  progress {done+failed+skipped}/{len(cells)} "
                      f"done={done} skip={skipped} fail={failed} "
                      f"elapsed={el/60:.1f}min")
    out, n = aggregate(args.root)
    print(f"finished: done={done} skipped={skipped} failed={failed} "
          f"in {(time.time()-t0)/60:.1f}min -> {out} ({n} rows)")


if __name__ == "__main__":
    main()
