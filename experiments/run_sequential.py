"""Sequential experiment runner - avoids Windows multiprocessing issues."""

import csv, os, re, shutil, subprocess, sys, time, argparse
from datetime import datetime
sys.path.insert(0, os.path.dirname(__file__))
from configs.experiment_configs import (DhakaSimConfig, PROJECT_ROOT,
                                        generate_fractional_factorial,
                                        generate_extended_matrix,
                                        generate_baseline_configs)

JAVA = r"C:\Program Files\Eclipse Adoptium\jdk-21.0.12.8-hotspot\bin\java"
DHAKASIM = os.path.join(PROJECT_ROOT, "dhakasim")
RESULTS = os.path.join(PROJECT_ROOT, "experiments", "results")
DEFAULT_PARAMS = os.path.join(DHAKASIM, "input", "parameter.txt")

def run_one(config: DhakaSimConfig, run_dir: str, rep: int) -> dict:
    if not config.error_mode:
        parts = ["baseline"]
    else:
        parts = [f"sc{config.attack_scenario}"]
        if config.attack_scenario in (0,1,2,5): parts.append(f"delay{config.delay_mean}")
        if config.attack_scenario in (3,5): parts.append(f"jit{config.jitter_magnitude}")
        if config.attack_scenario in (4,5): parts.append(f"plr{config.packet_loss_rate}")
        if config.attack_scenario == 1: parts.append(f"prob{config.delay_probability}")
    cid = "_".join(parts)
    exp_dir = os.path.join(run_dir, f"{cid}_rep{rep}")
    os.makedirs(exp_dir, exist_ok=True)

    # Write parameter file
    with open(DEFAULT_PARAMS, "w") as f:
        f.write("\n".join(config.to_param_lines()))

    # Clear stats
    stats_dir = os.path.join(DHAKASIM, "statistics")
    os.makedirs(stats_dir, exist_ok=True)
    for fname in os.listdir(stats_dir):
        try: os.remove(os.path.join(stats_dir, fname))
        except: pass

    cp = os.pathsep.join([
        os.path.join(DHAKASIM, "out", "artifacts", "DhakaSim_jar", "DhakaSim.jar"),
        os.path.join(DHAKASIM, "libraries", "commons-math3-3.6.1.jar"),
        os.path.join(DHAKASIM, "libraries", "jmathio.jar"),
    ])

    # Timeout scales with simulation duration; generous headroom for slower topologies
    timeout_s = max(300, int(config.simulation_end_time * 0.4) + 120)

    t0 = time.time()
    try:
        r = subprocess.run([JAVA, "-cp", cp, "thesisfinal.DhakaSim"],
                          cwd=DHAKASIM, capture_output=True, text=True, timeout=timeout_s)
        elapsed = time.time() - t0
        output = r.stdout + r.stderr
        with open(os.path.join(exp_dir, "output.log"), "w") as f:
            f.write(output)

        # Parse results
        results = {"config_id": cid, "repetition": rep,
                   "elapsed_seconds": round(elapsed, 2), "status": "completed",
                   "attack_scenario": config.attack_scenario,
                   "delay_mean": config.delay_mean,
                   "error_mode": config.error_mode,
                   "simulation_end_time": config.simulation_end_time,
                   "random_seed": config.random_seed}
        for key, pat in [("avg_speed", r"speed:\s*([\d.]+)"),
                          ("avg_waiting_time", r"waiting time:\s*([\d.]+)"),
                          ("motorized_speed", r"motorized speed:\s*([\d.]+)"),
                          ("motorized_waiting_time", r"motorized waiting time:\s*([\d.]+)"),
                          ("non_motorized_speed", r"non-motorized speed:\s*([\d.]+)"),
                          ("non_motorized_waiting_time", r"non-motorized waiting time:\s*([\d.]+)"),
                          ("gap_error_count", r"Gap error count:\s*(\d+)"),
                          ("mean_abs_gap_error", r"Mean absolute gap error:\s*([\d.]+)"),
                          ("collision_count", r"Collision count:\s*(\d+)"),
                          ("accident_count", r"Accident count:\s*(\d+)")]:
            m = re.search(pat, output)
            if m: results[key] = float(m.group(1)) if "." in m.group(1) else int(m.group(1))

        # Copy statistics
        for fname in os.listdir(stats_dir):
            try: shutil.copy2(os.path.join(stats_dir, fname), os.path.join(exp_dir, fname))
            except: pass

        # Parse gap errors
        gcsv = os.path.join(stats_dir, "gap_errors.csv")
        if os.path.exists(gcsv):
            errors = []
            with open(gcsv) as f:
                for row in csv.DictReader(f):
                    errors.append(float(row["gapError"]))
            if errors:
                ae = [abs(e) for e in errors]
                results["gap_error_count"] = len(errors)
                results["gap_error_mean_abs"] = sum(ae) / len(ae)
                results["gap_error_max"] = max(ae)

        print(f"  [{cid}] completed in {elapsed:.1f}s, gap_error={results.get('mean_abs_gap_error', 'N/A')}, "
              f"collisions={results.get('collision_count', 'N/A')}")
        return results

    except subprocess.TimeoutExpired:
        print(f"  [{cid}] TIMEOUT after {timeout_s}s")
        return {"config_id": cid, "repetition": rep, "status": "timeout",
                "elapsed_seconds": round(time.time() - t0, 2),
                "error_mode": config.error_mode, "simulation_end_time": config.simulation_end_time}

    except Exception as e:
        elapsed = time.time() - t0
        print(f"  [{cid}] FAILED: {e}")
        return {"config_id": cid, "repetition": rep, "status": f"error: {e}",
                "elapsed_seconds": round(elapsed, 2),
                "error_mode": config.error_mode, "simulation_end_time": config.simulation_end_time}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="fractional",
                    choices=["fractional", "extended", "baseline", "long", "full"])
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--label", default=None)
    args = ap.parse_args()

    reps = args.reps
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.label:
        run_dir = os.path.join(RESULTS, f"run_{timestamp}_{args.label}")
    else:
        run_dir = os.path.join(RESULTS, f"run_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)

    configs = []
    if args.mode in ("fractional", "full"):
        configs += generate_fractional_factorial(DhakaSimConfig())
    if args.mode in ("extended", "full"):
        configs += generate_extended_matrix(DhakaSimConfig())
    if args.mode in ("baseline", "full"):
        configs += generate_baseline_configs()
    if args.mode == "long":
        for d in [1.0, 3.0, 5.0, 7.0]:
            configs.append(DhakaSimConfig(attack_scenario=0, delay_mean=d,
                                          delay_probability=1.0, simulation_end_time=1800))
        configs += generate_baseline_configs(simulation_end_time=1800)

    all_tasks = []
    for i, (cfg, rep) in enumerate([(cfg, rep) for cfg in configs for rep in range(reps)]):
        cfg.random_seed = 42 + i * 7 + rep
        all_tasks.append((cfg, rep))

    print(f"Running {len(all_tasks)} experiments in {run_dir}")
    print("-" * 60)

    all_results = []
    for i, (cfg, rep) in enumerate(all_tasks):
        print(f"[{i+1}/{len(all_tasks)}] ", end="")
        r = run_one(cfg, run_dir, rep)
        all_results.append(r)

    # Write summary
    spath = os.path.join(run_dir, "results_summary.csv")
    if all_results:
        fn = sorted(set().union(*(r.keys() for r in all_results)))
        with open(spath, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fn)
            w.writeheader(); w.writerows(all_results)
    print(f"\nSummary: {spath}")
    print(f"Completed: {sum(1 for r in all_results if r.get('status')=='completed')}/{len(all_tasks)}")

if __name__ == "__main__":
    main()
