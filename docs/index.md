---
title: Temporal Sensor Desynchronization Attacks in Non-Lane-Based Traffic
---

<div class="hero" markdown="1">

# Timing the Chaos

**Temporal sensor desynchronization attacks in non-lane-based traffic systems** —
Saimoon Al Farshi Oman and A. B. M. Alim Al Islam (BUET).
Submitted to **IEEE Transactions on Dependable and Secure Computing (TDSC)**.

Cooperative vehicles brake on positions their neighbours broadcast. Security
research asks whether those positions are **correct**. We ask whether they are
**current**.

[Read the paper (Releases)](https://github.com/saimoon-oman/timing-the-chaos/releases)
· [Repository](https://github.com/saimoon-oman/timing-the-chaos)
· [Wiki](https://github.com/saimoon-oman/timing-the-chaos/wiki)

</div>

## The error model in one line

Under a delay `τ`, the follower's V2V channel supplies the leader's position
from `τ` seconds ago while its own position is current:

```
ε(t) = ĝ(t) − g(t) = −∫[t−τ,t] v_leader(s) ds  ≈  −v_leader(t) · τ
```

It scales with **leader speed**, and it is **negative** in flowing traffic —
followers under-estimate their gaps and brake when they need not.

<div class="cards" markdown="1">

**8,100**
Simulation runs across volume, mix, variant, magnitude, topology, seed

**11.29 m**
Perceived-gap displacement from a 3 s delay (reference cell)

**−5.09 s**
Median TTC bias; 0 collisions in 8,100 runs

**99.1% / 67.4%**
Detection TPR @ 0.1% FPR; gap error removed by causal correction

</div>

## Headline numbers

| Quantity | Value |
|---|---|
| Simulation runs | 8,100 (+ 330 SUMO replication runs) |
| Gap error at 3 s delay | 11.29 m |
| Leader-speed collapse | r = 0.9984 |
| Perceived vs true TTC < 2 s | 20.0% vs 8.6% |
| Collisions, entire campaign | **0** |
| Detection at 0.1% FPR | 99.1% (AUC 0.992) |
| Gap error removed by correction | 67.4% (oracle: 54.7%) |
| Adaptive adversary confined to | 113 ms of staleness |

## What is new here

1. **A timing-only adversary** — delay, drop, or jitter arrival timing while
   every value stays authentic and every signature verifies. IEEE 1609.2
   leaves the freshness window to the application; ETSI specifies none; drops
   defeat freshness by construction.
2. **Non-lane-based geometry** — DhakaSim's 0.5 m strips instead of 3.5 m
   lanes, evaluated on three Dhaka topologies plus a SUMO replication.
3. **A defence from the surviving channel** — the untouched forward radar
   makes delay observable: signed residual → detector → causal estimator →
   gated correction, with an evasion bound set by the defender.
4. **Reported negative results** — including a retraction of our own earlier
   strip-amplification claim and proof that median filtering worsens
   temporal faults.

Start with the [threat model]({{ site.baseurl }}/threat-model/), then
[results]({{ site.baseurl }}/results/) and [defenses]({{ site.baseurl }}/defenses/).
