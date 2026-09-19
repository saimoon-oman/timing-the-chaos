# `results/` — campaign outputs

Everything in this folder is machine-generated. Nothing here is hand-edited,
and every number quoted in the README, on the project website and in the wiki
traces back to `runs.csv` through the pipeline below.

```
runs.csv ──▶ analysis/analyse.py ──▶ facts_v4.json ──▶ (README / site / wiki / paper)
                                └──▶ analysis/verify.py ──▶ VERIFICATION.md
```

## Files

| File | What it is |
|---|---|
| `runs.csv` | One row per simulation run — the full design coordinate plus every reduced metric. 8,340 rows, 79 columns, 6.7 MB. |
| `facts_v4.json` | 5,700 derived quantities, keyed (`vol.shahbagh.1.0.PD.mae`, `collapse.shahbagh.r`, …). This is what the prose is rendered from. |
| `facts_compare.json` | The prior-work comparison (our traces recomputed on P(TTC<1 s) and P(DRAC>2 m/s²) at the 2–4 s delay band). |
| `VERIFICATION.md` | Independent recomputation of every headline number, straight from `runs.csv`. 23 of 23 checks pass. |
| `cells.csv` | Per-cell aggregates for the volume × mix grid. |
| `strip_cells.csv` | Per-cell aggregates for the lateral-resolution ablation (3,150 runs). |
| `strip_matched_pairs.csv` | The four independent (topology, delay) matched pairs the ablation's significance tests are computed on. |
| `compare_cells.csv` | Per-run metrics behind `facts_compare.json`. |

Raw per-vehicle traces (`gap_errors.csv`, ~1.8 GB across the campaign) are
**not** tracked. They are regenerable — the runner is keyed by a hash of the
full parameter set, so re-running a configuration reproduces its trace exactly.

## Row count: 8,340 in the file, 8,100 in the results

`analysis/analyse.py::load()` drops 240 rows before anything else touches them:

```python
d = d[d.status == "ok"]
# the first single-shot pass fired at t = 60 s, inside the warm-up window,
# and was superseded by a corrected pass at t = 150 s
d = d[~((d.variant == "SS") & (d.single_shot_time == 60))]
```

The superseded single-shot rows are kept in the file rather than deleted, so
the mistake and its correction are both auditable. Every analysis, and the
"8,100 runs" figure, is computed after the filter.

## Blocks

| Block | Runs | What it varies |
|---|---:|---|
| `conv` | 240 | Replication seed only — the convergence and power analysis. |
| `volume` | 960 | Demand multiplier × variant × topology. |
| `mix` | 960 | Fleet composition × variant × topology. |
| `volume_x_mix` | 960 | The full 4 × 4 demand × composition grid. |
| `delay_x_volume` | 840 | Delay magnitude crossed with demand. |
| `delay_x_mix` | 840 | Delay magnitude crossed with composition. |
| `atk_param` | 330 | Per-variant attack parameters (drop rate, jitter σ, duty cycle). |
| `horizon` | 60 | Evaluation horizon, 300 s vs longer. |
| `strip` | 3,150 | Lateral resolution `w_s ∈ {0.5, 1.0, 1.75, 3.5}` m × demand. |

**The `strip` block is not comparable with the rest.** Changing `w_s` changes
the road model itself — it sets both the perception quantum and the lateral
occupancy, so it moves capacity as well as resolution. `analyse.py` therefore
passes `d[d.strip_width == 0.5]` to every analysis except the ablation's own,
and `verify.py` asserts that isolation structurally. That assertion exists
because pooling the ablation's baselines *did* silently deflate every
speed-loss figure in an earlier draft (12.56 % → 8.11 %), and nothing but an
explicit check caught it.

## Column groups in `runs.csv`

- **Design:** `block, topology, volume, mix_slow, mix_medium, mix_fast,
  variant, horizon, seed, strip_width, position_buffer_size, cf_model, key`
- **Attack parameters:** `delay_mean, delay_probability, delay_spread,
  jitter_magnitude, jitter_only_magnitude, packet_loss_rate,
  single_shot_delay, single_shot_time, attack_start_time, attack_end_time,
  attack_strip_mod`
- **Perception error:** `gap_err_mae, gap_err_mean, gap_err_median_abs,
  gap_err_p95_abs, gap_err_max, gap_err_std, gap_err_neg_frac,
  strip_offset_mean, offset_units_half, offset_units_lane, displaced_frac`
- **Traffic outcome:** `net_speed, net_waiting, mot_speed, mot_waiting,
  nonmot_speed, nonmot_waiting, moving_frac, follower_speed_mean,
  leader_speed_mean, collisions, accidents`
- **Surrogate safety:** `ttc_true_viol, ttc_perc_viol, ttc_true_median,
  ttc_perc_median, ttc_bias_median, ttc_pairs, ttc_threshold_sim`
- **Detection:** `resid_mean, resid_p50, detector_fire_rate, gate_frac,
  tau_hat_median, roc_file`
- **Defence:** `def_raw_mae, def_fixed1_mae, def_median_mae, def_causal_mae,
  def_gated_mae, def_oracle_mae, def_followers, def_samples,
  red_fixed1, red_median, red_causal, red_gated, red_oracle`
- **Integrity:** `status, elapsed, trace_rows, sim_gap_count, sim_gap_mae,
  usable_rows, label`

`trace_rows` and `sim_gap_count` are the integrity pair: the harness asserts
that the number of rows it parsed equals the number the simulator reported
writing, and marks the run `trace-mismatch:<parsed>!=<declared>` otherwise. It
was added after two concurrent campaign processes sharing worker directories
interleaved their trace writes. No mismatched row reached this file.

## Reproducing

```bash
python3 experiments/campaign.py --label v4 --workers 3
python3 analysis/trace_study.py
python3 analysis/compare.py
python3 analysis/analyse.py
python3 analysis/verify.py
```

`verify.py` exits non-zero on any mismatch, so it is safe to run in CI.
