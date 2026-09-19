"""Campaign design for timing-the-chaos v4."""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from harness import Cfg, OUT, WORK, make_workspace, run_one   # noqa: E402

TOPOS = ["shahbagh", "shankar_palashi", "straight_line"]

# traffic volume: demand multipliers, with the labels used in the paper
VOLUMES = [0.5, 1.0, 2.0, 3.0]
VOL_LABEL = {0.5: "light", 1.0: "moderate", 2.0: "heavy", 3.0: "saturated"}

# traffic mix: (slow, medium, fast) percentage split
MIXES = [(80, 15, 5), (65, 20, 15), (50, 25, 25), (20, 30, 50)]
MIX_LABEL = {(80, 15, 5): "slow-dominant", (65, 20, 15): "slow-heavy",
             (50, 25, 25): "balanced", (20, 30, 50): "fast-dominant"}

DELAYS = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]
VARIANTS = ["none", "PD", "IF", "SS", "JO", "PDr", "JM"]

# lateral discretisation: 0.5 m is DhakaSim's non-lane-based default, 3.5 m is
# a standard lane; 1.75 m is the half-lane at which a lane-based model changes
# a vehicle's discrete lateral state.
STRIP_WIDTHS = [0.5, 1.0, 1.75, 3.5]
STRIP_VOLUMES = [0.25, 0.5, 1.0, 2.0]

BASE_MIX = (50, 25, 25)
BASE_VOL = 1.0
BASE_DELAY = 3.0
HORIZON = 300
SEEDS = list(range(10))
CONV_SEEDS = list(range(40))


def variant_cfg(base: Cfg, variant: str, delay: float = BASE_DELAY) -> Cfg:
    """Canonical parameterisation of each attack variant (Section III-C)."""
    c = replace(base, variant=variant)
    if variant == "none":
        return c
    if variant == "PD":
        return replace(c, delay_mean=delay, delay_probability=1.0)
    if variant == "IF":
        return replace(c, delay_mean=delay, delay_probability=0.5)
    if variant == "SS":
        # must fire after the 60 s warm-up, else the event is discarded
        return replace(c, single_shot_delay=delay, single_shot_time=150,
                       delay_probability=0.3)
    if variant == "JO":
        return replace(c, jitter_magnitude=1.0, jitter_only_magnitude=2,
                       delay_probability=0.3)
    if variant == "PDr":
        return replace(c, packet_loss_rate=0.3, delay_probability=0.3,
                       delay_mean=delay)
    if variant == "JM":
        return replace(c, delay_mean=delay, delay_probability=0.5,
                       jitter_magnitude=1.0, packet_loss_rate=0.3)
    raise ValueError(variant)


def build_design() -> list[Cfg]:
    cfgs: list[Cfg] = []

    def add(block, topo, vol, mix, variant, seed, delay=BASE_DELAY,
            horizon=HORIZON, **kw):
        base = Cfg(block=block, topology=topo, volume=vol, mix=mix,
                   horizon=horizon, seed=seed)
        c = variant_cfg(base, variant, delay)
        if kw:
            c = replace(c, **kw)
        cfgs.append(c)

    # --- B1  replication-convergence study -------------------------------
    for t in TOPOS:
        for v in ("none", "PD"):
            for s in CONV_SEEDS:
                add("conv", t, BASE_VOL, BASE_MIX, v, s)

    # --- B2  traffic-volume main effect, all attack variants -------------
    for t in TOPOS:
        for vol in VOLUMES:
            for v in VARIANTS:
                for s in SEEDS:
                    add("volume", t, vol, BASE_MIX, v, s)

    # --- B3  traffic-mix main effect, all attack variants ----------------
    for t in TOPOS:
        for mix in MIXES:
            for v in VARIANTS:
                for s in SEEDS:
                    add("mix", t, BASE_VOL, mix, v, s)

    # --- B4  delay dose-response crossed with traffic volume -------------
    for t in TOPOS:
        for vol in VOLUMES:
            for d in DELAYS:
                for s in SEEDS:
                    add("delay_x_volume", t, vol, BASE_MIX, "PD", s, delay=d)

    # --- B5  delay dose-response crossed with traffic mix ----------------
    for t in TOPOS:
        for mix in MIXES:
            for d in DELAYS:
                for s in SEEDS:
                    add("delay_x_mix", t, BASE_VOL, mix, "PD", s, delay=d)

    # --- B6  volume x mix full two-way factorial -------------------------
    for t in TOPOS:
        for vol in VOLUMES:
            for mix in MIXES:
                for v in ("none", "PD"):
                    for s in SEEDS:
                        add("volume_x_mix", t, vol, mix, v, s)

    # --- B7  attack-parameter sweeps -------------------------------------
    for t in TOPOS:
        for p in (0.1, 0.3, 0.5, 0.7):
            for s in SEEDS:
                add("atk_param", t, BASE_VOL, BASE_MIX, "IF", s,
                    delay_probability=p)
                add("atk_param", t, BASE_VOL, BASE_MIX, "PDr", s,
                    packet_loss_rate=p)
        for j in (0.5, 1.0, 2.0):
            for s in SEEDS:
                add("atk_param", t, BASE_VOL, BASE_MIX, "JO", s,
                    jitter_magnitude=j)

    # --- B9  paired strip-vs-lane ablation --------------------------------
    # Lateral discretisation is crossed with demand, because changing the strip
    # width also changes how many vehicles fit abreast and therefore the
    # traffic state itself.  Crossing the two lets the behavioural cost be
    # compared at matched traffic states rather than at matched demand.
    for t in TOPOS:
        for ws in STRIP_WIDTHS:
            for vol in STRIP_VOLUMES:
                for v, dly in [("none", 0.0), ("PD", 1.0), ("PD", 3.0),
                               ("PD", 5.0), ("PD", 7.0)]:
                    for s in SEEDS:
                        add("strip", t, vol, BASE_MIX, v, s, delay=dly,
                            strip_width=ws)

    # --- B9b  demand extension for the strip ablation ---------------------
    # A 3.5 m lane model fits far fewer vehicles abreast, so at equal demand it
    # is far more congested than a 0.5 m strip model.  These extra cells push
    # the fine-discretisation networks into the same baseline-speed range as
    # the coarse ones, giving the two a common support to compare on.
    for t in TOPOS:
        for ws in (0.5, 1.0, 1.75):
            for vol in (3.0,):
                for v, dly in [("none", 0.0), ("PD", 1.0), ("PD", 3.0),
                               ("PD", 5.0), ("PD", 7.0)]:
                    for s in SEEDS:
                        add("strip", t, vol, BASE_MIX, v, s, delay=dly,
                            strip_width=ws)
        for vol in (0.05, 0.1):
            for v, dly in [("none", 0.0), ("PD", 1.0), ("PD", 3.0),
                           ("PD", 5.0), ("PD", 7.0)]:
                for s in SEEDS:
                    add("strip", t, vol, BASE_MIX, v, s, delay=dly,
                        strip_width=3.5)

    # --- B8  extended horizon --------------------------------------------
    for t in TOPOS:
        for vol in (1.0, 3.0):
            for v in ("none", "PD"):
                for s in range(5):
                    add("horizon", t, vol, BASE_MIX, v, s, horizon=900)

    # de-duplicate on the full configuration key
    seen, uniq = set(), []
    for c in cfgs:
        k = c.key()
        if k in seen:
            continue
        seen.add(k)
        uniq.append(c)
    return uniq


# ------------------------------------------------------------------ runner

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--label", default="v4")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--blocks", default="")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    cfgs = build_design()
    if args.blocks:
        want = set(args.blocks.split(","))
        cfgs = [c for c in cfgs if c.block in want]
    if args.limit:
        cfgs = cfgs[:args.limit]

    from collections import Counter
    print("configurations:", len(cfgs))
    for b, n in sorted(Counter(c.block for c in cfgs).items()):
        print(f"  {b:16s} {n}")
    if args.dry_run:
        return

    run_dir = OUT / args.label
    run_dir.mkdir(parents=True, exist_ok=True)
    out_csv = run_dir / "runs.csv"

    # refuse to start if another campaign process is already writing here:
    # concurrent runs share worker directories and silently corrupt traces
    lockfile = run_dir / "campaign.lock"
    if lockfile.exists():
        pid = lockfile.read_text().strip()
        alive = Path(f"/proc/{pid}").exists()
        if alive:
            print(f"ERROR: campaign already running as pid {pid}; refusing to "
                  f"start a second one. Remove {lockfile} only if that pid "
                  f"is dead.")
            sys.exit(1)
        print(f"note: clearing stale lock from dead pid {pid}")
    lockfile.write_text(str(os.getpid()))
    import atexit
    atexit.register(lambda p=lockfile: p.unlink(missing_ok=True))

    done = set()
    if out_csv.exists():
        with open(out_csv) as f:
            for row in csv.DictReader(f):
                done.add(row["key"])
        print(f"resuming: {len(done)} runs already present")
    todo = [c for c in cfgs if c.key() not in done]
    print(f"to run: {len(todo)}")

    q: "queue.Queue[Cfg]" = queue.Queue()
    for c in todo:
        q.put(c)

    lock = threading.Lock()
    results: list[dict] = []
    t_start = time.time()
    counter = {"n": 0}
    # keep the full residual sample only for a manageable subset
    roc_blocks = {"volume", "mix", "delay_x_volume", "conv"}

    def worker(wid: int):
        ws = make_workspace(wid)
        while True:
            try:
                cfg = q.get_nowait()
            except queue.Empty:
                return
            keep = cfg.block in roc_blocks and cfg.seed < 2
            try:
                rec = run_one(cfg, ws, keep_roc=keep)
            except Exception as e:                            # pragma: no cover
                rec = {"key": cfg.key(), "label": cfg.label(),
                       "status": f"harness-error:{e}", "block": cfg.block}
            with lock:
                results.append(rec)
                counter["n"] += 1
                n = counter["n"]
                if n % 25 == 0 or n == len(todo):
                    el = time.time() - t_start
                    rate = n / el
                    eta = (len(todo) - n) / rate / 60 if rate else 0
                    print(f"[{n}/{len(todo)}] {el/60:.1f} min elapsed, "
                          f"{rate:.2f} runs/s, ETA {eta:.0f} min", flush=True)
                if n % 100 == 0:
                    flush(run_dir, results)
            q.task_done()

    threads = [threading.Thread(target=worker, args=(i,), daemon=True)
               for i in range(args.workers)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    flush(run_dir, results)
    print(f"\ndone: {len(results)} runs in {(time.time()-t_start)/60:.1f} min")
    print("summary:", out_csv)


FIELDS_CACHE: list[str] = []


def flush(run_dir: Path, results: list[dict]):
    out_csv = run_dir / "runs.csv"
    existing = []
    if out_csv.exists():
        with open(out_csv) as f:
            existing = list(csv.DictReader(f))
    allrows = existing + results
    if not allrows:
        return
    fields = sorted({k for r in allrows for k in r})
    tmp = out_csv.with_suffix(".tmp")
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(allrows)
    tmp.replace(out_csv)
    results.clear()


if __name__ == "__main__":
    main()
