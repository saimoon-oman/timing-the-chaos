# Timing the Chaos

<!--

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![Website](https://img.shields.io/badge/website-live-brightgreen)

**Timing the Chaos: Temporal Sensor Desynchronization Attacks in Non-Lane-Based Traffic Systems**


Cooperative vehicles brake on the basis of positions their neighbours broadcast
over the air. A decade of security work has asked whether those positions are
**correct**. This project asks whether they are **current**: a network-level
adversary that delays, drops, or jitters the *arrival timing* of V2V position
updates — while every value stays authentic and every signature verifies —
corrupts gap perception, biases safety estimates, and costs throughput, with no
integrity check on content able to see it.

> **Course project (CSE 6207 — Advanced Dependable and Fault-Tolerant Computer
> Systems, BUET MSc).** 
---

## Table of Contents

- [Threat Model](#threat-model)
- [Attack Variants](#attack-variants)
- [Key Findings](#key-findings)
- [Experimental Campaign](#experimental-campaign)
- [Defense Stack](#defense-stack)
- [Repository Structure](#repository-structure)
- [Build and Run](#build-and-run)
- [Requirements](#requirements)
- [Reproducibility](#reproducibility)
- [Documentation](#documentation)
- [Citation](#citation)
- [License](#license)

---

## Threat Model

**Temporal sensor desynchronization.** The adversary sits on the V2V network
path (IEEE 802.11p/DSRC or C-V2X) and manipulates only the *timing* of position
updates — delaying, dropping, or jittering them. It cannot forge content and
cannot touch the follower's forward radar, which stays fresh. The follower is
forced to act on a stale leader position, producing a perceived gap error of

$$\varepsilon(t) = \hat{g}(t) - g(t) = -\int_{t-\tau}^{t} v_{\text{leader}}(s)\,ds \;\approx\; -v_{\text{leader}}(t)\cdot\tau$$

which scales with **leader speed** rather than delay alone, and is **negative**
in flowing traffic — followers under-estimate their gaps and brake when they
need not.

Why the standards do not stop it: **IEEE 1609.2** leaves the freshness
tolerance to the application, **ETSI EN 302 637-2** specifies no receiver-side
validity window, and no freshness policy can reject an update that was never
sent (a drop). The tolerance non-lane-based geometry actually requires —
**147 ms** at the measured mean leader speed of a dense Dhaka network — is an
order of magnitude tighter than lane-based practice would suggest.

The evaluation ground is **DhakaSim**, a microsimulator of Dhaka's
non-lane-based traffic, where vehicles negotiate **0.5 m lateral strips**
rather than 3.5 m lanes.

## Attack Variants

Six attack variants implemented as a parameter-driven fault-injection mode
(`FT_METHOD 4`) in DhakaSim:

| ID | Variant | Description |
|----|---------|-------------|
| 0 | Persistent delay (PD) | Constant delay on V2V position updates (1–10 s sweep) |
| 1 | Intermittent faults | Delay/faults at a duty cycle |
| 2 | Single-shot | One delayed update, then resynchronization |
| 3 | Jitter-only | Timing jitter on update arrival, no mean delay |
| 4 | Packet drop (PDr) | Fraction of updates dropped (stale holdover) |
| 5 | Joint multi-sensor | Combined delay + jitter |

## Key Findings

From a campaign of **8,100 runs** across traffic volume, fleet mix, attack
variant, magnitude, topology, and seed — cross-validated by **330 further runs
in SUMO**:

1. **Staleness alone corrupts gap perception.** A 3 s delay displaces the
   perceived gap by **11.29 m** at the reference cell, with a strong
   dose–response on every topology. Strip geometry multiplies how that error
   registers in the model's discrete lateral state, but a paired ablation
   finds **no corresponding multiplier on aggregate mobility damage** — we
   explicitly retract the earlier behavioural reading of the strip effect.
2. **The attack's strength is a computable function of the traffic state.**
   The gap error spans a factor of **2.5×** across the volume × mix grid, yet
   mean leader speed collapses it onto one line at **r = 0.9984**
   (cell-mean gap error tracks cell-mean leader speed).
3. **The damage is misjudged risk, not realised risk.** The attack biases
   time-to-collision pessimistically by **−5.09 s** and raises the perceived
   rate of sub-two-second situations to **20.0%** against a true **8.6%** —
   vehicles brake when they need not, costing throughput (mobility cost
   1.2–18.8% of speed, growing with congestion), with **0 collisions in
   8,100 runs**.
4. **The same asymmetry that enables the attack defeats and bounds it.** The
   untouched forward radar makes delay observable: a signed radar/V2V residual
   detects delay-family attacks at **99.1% TPR for 0.1% FPR (AUC 0.992)** and
   drives a causal correction removing **67.4%** of the gap error (oracle
   bound: 54.7%), bounding an adaptive adversary to **113 ms** of staleness.
   Temporal median filtering instead **worsens** the error in most attacked
   runs — a documented negative result.

SUMO replication reproduces the sign, ordering, and leader-speed collapse
(R² = 0.996).

## Experimental Campaign

- `experiments/configs/` — campaign configuration definitions;
- `experiments/run_sequential.py` — full campaign runner (Windows-safe,
  sequential; per-run statistics, timeouts);
- `experiments/run_experiment.py` — single-run harness;
- `dhakasim/input/` — active parameter/topology files;
- `dhakasim/topologies/` — all topologies (OSM-derived + demand files);
- `sumo_experiments/` — SUMO cross-validation (`configs/`, `networks/`,
  `scripts/`; `results_sumo/` is regenerable output).

Run outputs (`experiments/results/`, `experiments/logs/`, `**/gap_errors.csv`,
`dhakasim/statistics/`, `sumo_experiments/results_sumo/` — gigabytes of
regenerable traces) are **not** tracked in git; re-run the campaign to
regenerate them (see [Reproducibility](#reproducibility)).

## Defense Stack

- `defenses/dtw_detection/strip_aware_dtw.py` — strip-aware detector
  (signed radar/V2V residual; delay-family detection);
- `defenses/schmidt_kalman/tau_estimator.py` — causal radar-discrepancy delay
  estimator $\hat{\tau}$;
- `defenses/schmidt_kalman/gap_estimator.py` — velocity-compensated gap
  correction $\hat{g}_{corr}(t) = \hat{g}(t) + v_{leader}(t-\hat{\tau})\cdot\hat{\tau}$;
- `defenses/consensus_ft/temporal_median.py` — temporal median consensus
  (evaluated; **negative result** — increases error on real attack traces).

## Repository Structure

```
timing-the-chaos/
├── dhakasim/              # Modified DhakaSim (Java) microsimulator
│   ├── src/thesisfinal/   # Java source (FT_METHOD 4 = temporal desync attacks)
│   ├── input/             # Active parameter/topology files
│   ├── topologies/        # All topologies (OSM-derived + demand files)
│   ├── out/               # Prebuilt JAR (no rebuild needed)
│   ├── libraries/         # Third-party JAR dependencies
│   ├── build.bat          # Build (javac + jar; uses subst T: for path spaces)
│   └── run.bat            # Run simulator with input/parameter.txt
├── experiments/           # Campaign infrastructure (configs + runners)
├── analysis/              # Analysis pipeline (notebooks + figures)
│   ├── notebooks/         # Analysis scripts 01–10 (statistics, TTC, defenses)
│   └── figures/           # Generated figures
├── defenses/              # Defense algorithm implementations
├── sumo_experiments/      # SUMO cross-validation (configs, networks, scripts)
├── docs/                  # Project website (GitHub Pages)
├── tools/                 # Maintainer scripts (figure archive, wiki publish, CI)
└── README.md
```

Local-only directories (course materials, submission PDFs/drafts, demos,
presentations) are intentionally **not** tracked — see `.gitignore`.

## Build and Run

```bash
# Build DhakaSim (from dhakasim/; build.bat maps the tree to T: to survive path spaces)
build.bat

# Run with default parameters (no attack)
run.bat

# Run with a persistent-delay attack — edit input/parameter.txt:
#   ErrorMode On
#   FTMethod 4
#   AttackScenario 0
#   DelayMean 3.0
```

Reproduce the campaign:

```bash
python experiments/run_sequential.py --help
python experiments/run_sequential.py --mode full --reps 2 --label my_run
```

Analyze results:

```bash
python analysis/notebooks/01_gap_error_analysis.py --results <run>/results_summary.csv
python analysis/notebooks/09_tau_eval.py --gapdir <run>
```

## Requirements

- JDK 21+ (Eclipse Adoptium Temurin recommended)
- Python 3.10+ with numpy, scipy, pandas, matplotlib, seaborn
- (Website only) Jekyll, for local preview of `docs/`

## Reproducibility

Every number in the paper is produced by a template → facts → analysis
pipeline, not typed by hand: analysis scripts derive a facts file from the run
CSVs, and the manuscript renders placeholders from those facts, so a missing
or stale value aborts the render instead of silently surviving into the PDF.
An independent verification pass recomputes the headline results directly from
the run data through its own code path (**23 of 23 checks pass**).

Known caveats carried into the paper: an early single-shot pass fired inside
the 60 s warm-up window and was superseded by a corrected pass (excluded runs
documented in the analysis code); the 10-step position buffer caps realisable
staleness at ~10 s, so configured delays above ~7 s under-deliver (reported
delays are configured, not realised — high-delay results are conservative).

## Documentation

- **Project website:** <https://saimoon-oman.github.io/timing-the-chaos/>
- **GitHub Wiki:** <https://github.com/saimoon-oman/timing-the-chaos/wiki>
  (Home, Threat Model, Attack Variants, Experimental Design, Results,
  Defences, Negative Results, Reproducibility, FAQ)

## Citation

A machine-readable `CITATION.cff` is included — GitHub renders a
"Cite this repository" button from it.

---

## License

The novel code in this repository (attack scenarios, experiment framework,
analysis pipeline, defenses, documentation) is released under the
[MIT License](LICENSE). DhakaSim itself is a third-party academic
microsimulator and is **not** covered by that license; it retains its
original terms. See `dhakasim/libraries/` for third-party dependencies.

-->
