---
title: Results
---

# Results

Five findings, in the order they constrain each other. Full detail in the
[manuscript](https://github.com/saimoon-oman/timing-the-chaos/releases) and
the [wiki](https://github.com/saimoon-oman/timing-the-chaos/wiki).

## 1. Staleness corrupts gap perception

A 3 s persistent delay displaces the perceived gap by **11.29 m** at the
reference cell, with a strong dose–response on every topology.

![Gap error vs delay]({{ site.baseurl }}/assets/gap_error_vs_delay.png){: .fig }
*Mean absolute gap error grows with configured delay.*

## 2. Traffic volume splits the attack in two

| Demand | Baseline speed | Perception error | Throughput cost |
|---|---:|---:|---:|
| ×0.5 (light) | 6.17 m/s | 11.05 m | 8.9% |
| ×1 (moderate) | 5.85 m/s | 11.29 m | 12.6% |
| ×2 (heavy) | 5.21 m/s | 10.48 m | 16.4% |
| ×3 (saturated) | 4.69 m/s | 9.58 m | 20.3% |

Perception error **falls** with congestion (a stale photograph of a barely
moved leader is nearly current) while throughput cost **rises** (near
capacity, premature braking propagates instead of being absorbed).

## 3. Mix is the stronger factor — and one scalar predicts everything

Perceived-gap error more than doubles from slow- to fast-dominant fleets at a
fixed 3 s delay. Volume and mix are two routes to one quantity: effective
staleness `τ_eff = E[|ε|] / E[v_leader]` is constant across the grid
(CV 3.3–6.8%) while the error itself varies 2.5×; gap error tracks mean
leader speed at **r = 0.9984** (Shahbagh; 0.9938 / 0.9921 on the other two).

![Topology comparison]({{ site.baseurl }}/assets/topology_comparison.png){: .fig }
*The same attack across three topologies.*

## 4. Misjudged risk, not realised risk

Median TTC bias **−5.09 s**; followers believe they are in a TTC < 2 s
situation in **20.0%** of close-following steps vs **8.6%** truly.
**Zero collisions in all 8,100 runs** — a stealthy denial of throughput, not
a crash exploit.

![TTC CDF]({{ site.baseurl }}/assets/ttc_cdf.png){: .fig }
![TTC violation rate]({{ site.baseurl }}/assets/ttc_violation_rate.png){: .fig }

## 5. Cross-validated in SUMO

A 330-run replication reproduces the sign, ordering, and leader-speed
collapse (R² = 0.996).

![SUMO vs DhakaSim]({{ site.baseurl }}/assets/sumo_vs_dhaka.png){: .fig }
