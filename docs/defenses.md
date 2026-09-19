---
title: Defenses
---

# Defenses

The threat model supplies its own defence: the adversary cannot touch the
forward radar, so the residual between radar range and the V2V-derived gap is
a direct, on-board signature of staleness.

## Stage 1 — Detection from the radar residual

```
a(t) = 1[ |v_leader(t)| > v_min ] · 1[ r(t) > θ ],   r(t) = g_r(t) − ĝ(t)
```

Signed, not absolute: the error is negative in flowing traffic, so a stale
V2V gap reads *short* against radar.

| Variant | TPR at 0.1% FPR | AUC |
|---|---:|---:|
| Persistent delay | 99.1% | 0.992 |
| Intermittent faults | 99.2% | 0.993 |
| Packet drop | 98.0% | 0.980 |
| Single shot | 98.5% | 0.985 |
| Joint multi-sensor | 69.5% | 0.771 |
| Jitter only | 33.0% | 0.500 |

Thresholds calibrated on one topology's baselines land within centimetres on
all three (0.386 / 0.385 / 0.389 m).

## Stage 2 — Causal delay estimation

```
τ̂(t) = ( g_r(t) − ĝ(t) ) / v_leader( t − τ̂(t−1) )
```

Trailing 30-step median + exponential smoothing (α = 0.2), clamped [0, 15] s,
gated on leader speed > 0.5 m/s. Every operation is causal.

![Estimator accuracy]({{ site.baseurl }}/assets/tau_estimator_accuracy.png){: .fig }

## Stage 3 — Detector-gated velocity compensation

```
ĝ_corr(t) = ĝ(t) + v_leader(t − τ̂) · τ̂     (flagged samples only)
```

| Candidate | Gap-error reduction (persistent delay) |
|---|---:|
| Sliding-window median (k = 5) | **−20%** (worse than nothing) |
| Fixed τ̂ = 1 s | 26.1% |
| Oracle (true delay) | 54.7% |
| **Causal τ̂ + detector gate** | **67.4%** |

![Defense comparison]({{ site.baseurl }}/assets/defense_mae_reduction.png){: .fig }

**The estimator beats the oracle**: the residual measures realised
displacement, while the oracle delay needs a constant-velocity assumption
that fails in stop-and-go traffic. **Blindness to jitter is a safety
property**: with no staleness to compensate, the gate correctly never fires.

## Evasion bound

An adversary staying silent needs `v_leader · τ ≤ θ`, so its induced error is
bounded by the defender's threshold: at mean leader speed, **113 ms** of
staleness (47 ms at 90th-percentile speed) — 4.8% of the unmitigated effect,
next to the geometry's own 147 ms tolerance budget.

Cost: no new hardware, no message-format change — one subtraction, one
comparison, a 30-sample running median and an exponential update per pair
per step.
