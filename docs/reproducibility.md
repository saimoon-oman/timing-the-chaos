---
title: Reproducibility
---

# Reproducibility

## Requirements

JDK 21+, Python 3.10+ (`numpy scipy pandas matplotlib seaborn statsmodels`).

```bash
git clone https://github.com/saimoon-oman/timing-the-chaos.git
cd timing-the-chaos
pip install numpy scipy pandas matplotlib seaborn statsmodels
```

## A single simulation

From `dhakasim/` (`build.bat` compiles; `run.bat` runs). Configure via
`dhakasim/input/parameter.txt`:

```
ErrorMode        On     # Off = no attack
FTMethod         4      # 4 = temporal desynchronization
AttackScenario   0      # 0=PD 1=IF 2=SS 3=JO 4=PDr 5=JM
DelayMean        3.0
SimulationEndTime 300
```

Switch topology by copying `node.txt`, `link.txt`, `path.txt`, `demand.txt`
from `dhakasim/topologies/<name>/` into `dhakasim/input/` (all four files —
a missing `path.txt` crashes the simulator).

## The full campaign

```bash
python experiments/campaign.py --label v4 --workers 3
python experiments/campaign.py --dry-run
```

Resumable by configuration hash; lockfile-guarded; trace-integrity asserted.
The 330-run SUMO replication lives in `sumo_experiments/`.

## Analysis

Scripts in `analysis/notebooks/` (01–14: gap error, delay impact, defenses,
topology, TTC/statistics, estimator, detector, SUMO comparison) write figures
to `analysis/figures/`.

## How the numbers are kept honest

No manuscript number is typed by hand: section templates carry `@fact.key@`
placeholders rendered from a facts file derived from the run CSVs, and an
independent verification pass recomputes every headline number through its
own code path — **23 of 23 checks pass**. Raw traces (~GBs) are gitignored
and regenerable; the manuscript PDFs ship via
[Releases](https://github.com/saimoon-oman/timing-the-chaos/releases).

Known caveats: an early single-shot pass firing inside warm-up was superseded
(excluded runs documented in the analysis code); the 10-step position buffer
caps realisable staleness at ~10 s, so configured delays above ~7 s
under-deliver (conservative).
