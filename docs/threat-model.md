---
title: Threat Model
---

# Threat Model

## Adversary position

The adversary occupies the vehicle-to-vehicle communication path (IEEE
802.11p/DSRC or C-V2X): a compromised roadside unit, a store-and-forward
relay, or malware intercepting messages between the radio stack and the
perception module.

## Capabilities

For a chosen victim and time window: **delay** a position update by `τ`
seconds, **drop** it (receiver holds its last value), **jitter** delivery, and
**target selectively** (strips, time window). Six variants span continuous
through single-strike adversaries:

| ID | Variant | Mechanism |
|----|---------|-----------|
| PD | Persistent delay (0.5–10 s) | Pure staleness |
| IF | Intermittent faults | Random staleness |
| SS | Single shot (t = 150 s) | Isolated event |
| JO | Jitter only | Value perturbation, **no** staleness |
| PDr | Packet drop | Geometric staleness |
| JM | Joint multi-sensor | Both together |

## Limitations — and why each one matters

- **No physical influence.** Only the follower's picture is stale — invisible
  to monitors watching the road rather than the perception stack.
- **No content modification.** Every delivered coordinate was genuinely
  occupied; authentication and plausibility filters pass unchanged.
- **No radar access.** The forward radar stays fresh — the surviving channel
  that exposes the attack, and the load-bearing assumption of the defence.
- **No control of traffic state.** Demand and fleet mix are set by the city,
  which is why the evaluation sweeps them.

## Why the standards do not close the gap

1. The tolerance is not standardised (IEEE 1609.2 leaves `V` to the
   application; ETSI EN 302 637-2 specifies no receiver-side window).
2. Any plausible tolerance is already too wide: one strip of error needs a
   ~147 ms bound at Dhaka mean leader speed — an order of magnitude below
   lane-based practice.
3. Dropping defeats freshness by construction: a check can only reject a
   message that arrives.

![Delay–scenario heatmap]({{ site.baseurl }}/assets/delay_scenario_heatmap.png){: .fig }
*Gap error across the delay × scenario grid.*
