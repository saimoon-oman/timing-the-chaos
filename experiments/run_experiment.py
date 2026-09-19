"""
Multi-processing experiment runner for timing-the-chaos.
Runs DhakaSim simulations across attack parameter configurations
and collects results into structured output directories.
"""

import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(__file__))
from configs.experiment_configs import (
    DhakaSimConfig,
    PROJECT_ROOT,
    generate_fractional_factorial,
    generate_full_factorial,
)

JAVA_PATH = r"C:\Program Files\Eclipse Adoptium\jdk-21.0.11.10-hotspot\bin\java"
DHAKASIM_DIR = os.path.join(PROJECT_ROOT, "dhakasim")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "experiments", "results")
LOGS_DIR = os.path.join(PROJECT_ROOT, "experiments", "logs")


def get_config_id(config: DhakaSimConfig) -> str:
    """Generate a unique, human-readable config ID."""
    parts = [
        f"sc{config.attack_scenario}",
        f"delay{config.delay_mean:.1f}" if config.attack_scenario in (0, 1, 2, 5) else "",
        f"jit{config.jitter_magnitude:.2f}" if config.attack_scenario in (3, 5) else "",
        f"plr{config.packet_loss_rate:.2f}" if config.attack_scenario in (4, 5) else "",
        f"prob{config.delay_probability:.2f}" if config.attack_scenario == 1 else "",
    ]
    return "_".join(filter(None, parts))


def write_parameter_file(config: DhakaSimConfig, dest_dir: str):
    """Write parameter.txt for a given config into dest_dir."""
    os.makedirs(dest_dir, exist_ok=True)
    param_path = os.path.join(dest_dir, "parameter.txt")
    with open(param_path, "w") as f:
        f.write("\n".join(config.to_param_lines()))
    return param_path


def parse_console_output(output: str) -> Dict:
    """Parse key metrics from DhakaSim stdout."""
    result = {}
    patterns = {
        "avg_speed": r"speed:\s*([\d.]+)",
        "avg_waiting_time": r"waiting time:\s*([\d.]+)",
        "motorized_speed": r"motorized speed:\s*([\d.]+)",
        "motorized_waiting_time": r"motorized waiting time:\s*([\d.]+)",
        "non_motorized_speed": r"non-motorized speed:\s*([\d.]+)",
        "non_motorized_waiting_time": r"non-motorized waiting time:\s*([\d.]+)",
        "gap_error_count": r"Gap error count:\s*(\d+)",
        "mean_abs_gap_error": r"Mean absolute gap error:\s*([\d.]+)",
    }
    for key, pat in patterns.items():
        m = re.search(pat, output)
        if m:
            result[key] = float(m.group(1)) if "." in m.group(1) else int(m.group(1))
    return result


def parse_stats_csv(csv_path: str) -> Dict:
    """Parse aggregated statistics CSV files produced by DhakaSim."""
    result = {}
    if not os.path.exists(csv_path):
        return result
    with open(csv_path) as f:
        lines = f.readlines()
    if lines:
        values = [v.strip() for v in lines[-1].strip().split(",") if v.strip()]
        result[os.path.basename(csv_path).replace(".csv", "")] = values
    return result


def parse_gap_errors(csv_path: str) -> Dict:
    """Parse gap_errors.csv and compute summary statistics."""
    if not os.path.exists(csv_path):
        return {}
    errors = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            errors.append(float(row["gapError"]))

    if not errors:
        return {}

    abs_errors = [abs(e) for e in errors]
    return {
        "gap_error_count": len(errors),
        "gap_error_mean": sum(errors) / len(errors),
        "gap_error_mean_abs": sum(abs_errors) / len(abs_errors),
        "gap_error_max": max(abs_errors),
        "gap_error_std": (
            (sum((e - sum(errors) / len(errors)) ** 2 for e in errors) / len(errors)) ** 0.5
        ),
    }


def run_single_experiment(config: DhakaSimConfig, run_dir: str, seed: int = 1) -> Tuple[str, Dict]:
    """Run a single DhakaSim experiment with the given config.
    
    Returns:
        Tuple of (config_id, results_dict)
    """
    config_id = get_config_id(config)
    exp_dir = os.path.join(run_dir, config_id)
    os.makedirs(exp_dir, exist_ok=True)

    # Write config-specific parameter.txt into the actual DhakaSim input dir
    write_parameter_file(config, os.path.join(DHAKASIM_DIR, "input"))

    # Prepare logging
    log_path = os.path.join(exp_dir, "output.log")
    stats_dir = os.path.join(exp_dir, "statistics")
    os.makedirs(stats_dir, exist_ok=True)

    # Build classpath
    classpath = os.pathsep.join([
        os.path.join(DHAKASIM_DIR, "out", "artifacts", "DhakaSim_jar", "DhakaSim.jar"),
        os.path.join(DHAKASIM_DIR, "libraries", "commons-math3-3.6.1.jar"),
        os.path.join(DHAKASIM_DIR, "libraries", "jmathio.jar"),
    ])

    # Run simulation
    start_time = time.time()
    try:
        result = subprocess.run(
            [JAVA_PATH, "-cp", classpath, "thesisfinal.DhakaSim"],
            cwd=DHAKASIM_DIR,
            capture_output=True,
            text=True,
            timeout=600,
        )
        elapsed = time.time() - start_time
        output = result.stdout + result.stderr

        # Save output
        with open(log_path, "w") as f:
            f.write(output)

        # Parse results
        results = parse_console_output(output)
        results["elapsed_seconds"] = round(elapsed, 2)
        results["config_id"] = config_id

        # Copy statistics files
        for fname in os.listdir(os.path.join(DHAKASIM_DIR, "statistics")):
            src = os.path.join(DHAKASIM_DIR, "statistics", fname)
            dst = os.path.join(stats_dir, fname)
            try:
                shutil.copy2(src, dst)
            except (OSError, shutil.Error):
                pass

        # Parse gap errors if present
        gap_errors_csv = os.path.join(DHAKASIM_DIR, "statistics", "gap_errors.csv")
        if os.path.exists(gap_errors_csv):
            gap_results = parse_gap_errors(gap_errors_csv)
            results.update(gap_results)

        results["status"] = "completed"

    except subprocess.TimeoutExpired:
        elapsed = time.time() - start_time
        results = {"status": "timeout", "elapsed_seconds": round(elapsed, 2), "config_id": config_id}
        with open(log_path, "a") as f:
            f.write(f"\nTIMEOUT after {elapsed:.1f}s\n")

    except Exception as e:
        elapsed = time.time() - start_time
        results = {"status": f"error: {str(e)}", "elapsed_seconds": round(elapsed, 2), "config_id": config_id}
        with open(log_path, "a") as f:
            f.write(f"\nERROR: {str(e)}\n")

    # Also save a copy of the parameter file in the experiment dir for reproducibility
    shutil.copy2(
        os.path.join(DHAKASIM_DIR, "input", "parameter.txt"),
        os.path.join(exp_dir, "parameter.txt"),
    )

    return config_id, results


def main():
    parser = argparse.ArgumentParser(description="Run timing-the-chaos experiments")
    parser.add_argument("--mode", choices=["fractional", "full", "single"], default="fractional",
                        help="Experiment design mode")
    parser.add_argument("--repetitions", type=int, default=5,
                        help="Number of repetitions per config (for stochastic runs)")
    parser.add_argument("--workers", type=int, default=4,
                        help="Number of parallel workers")
    parser.add_argument("--seed", type=int, default=42,
                        help="Base random seed")
    parser.add_argument("--config-id", type=str, default=None,
                        help="Single config ID to run (only for single mode)")
    args = parser.parse_args()

    # Create run directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(RESULTS_DIR, f"run_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)

    # Generate configs
    base = DhakaSimConfig()
    if args.mode == "single":
        configs = [base]
        if args.config_id:
            print(f"Single mode: config_id={args.config_id} not yet implemented, using default.")
    elif args.mode == "fractional":
        configs = generate_fractional_factorial(base)
    else:
        configs = generate_full_factorial(base)

    # Add repetitions
    all_tasks = []
    for cfg in configs:
        for rep in range(args.repetitions):
            all_tasks.append((cfg, rep))

    print(f"Experiment Run: {timestamp}")
    print(f"Mode: {args.mode}")
    print(f"Configurations: {len(configs)}")
    print(f"Repetitions per config: {args.repetitions}")
    print(f"Total experiments: {len(all_tasks)}")
    print(f"Workers: {args.workers}")
    print(f"Output: {run_dir}")
    print("-" * 60)

    # Run experiments in parallel
    results_summary = []
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(run_single_experiment, cfg, run_dir, args.seed + rep): (cfg, rep)
            for cfg, rep in all_tasks
        }

        for i, future in enumerate(as_completed(futures)):
            cfg, rep = futures[future]
            try:
                config_id, results = future.result()
                results["repetition"] = rep
                results_summary.append(results)
                status = results.get("status", "unknown")
                elapsed = results.get("elapsed_seconds", 0)
                gap = results.get("mean_abs_gap_error", "N/A")
                print(
                    f"[{i + 1}/{len(all_tasks)}] {config_id} (rep {rep}): "
                    f"{status} [{elapsed:.1f}s] gap_error={gap}"
                )
            except Exception as e:
                print(f"[{i + 1}/{len(all_tasks)}] Task failed: {e}")

    # Write summary CSV
    summary_path = os.path.join(run_dir, "results_summary.csv")
    if results_summary:
        fieldnames = set()
        for r in results_summary:
            fieldnames.update(r.keys())
        fieldnames = sorted(fieldnames)
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results_summary)

    print(f"\nSummary written to: {summary_path}")
    print(f"Total completed: {sum(1 for r in results_summary if r.get('status') == 'completed')}/{len(all_tasks)}")


if __name__ == "__main__":
    main()
