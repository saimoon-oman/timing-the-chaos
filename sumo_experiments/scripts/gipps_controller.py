"""Gipps perception-in-the-loop controller (pure functions, no TraCI import).

Faithful port of DhakaSim's FT_METHOD=4 path for the SUMO cross-validation:
  * Vehicle.getSpeedForAcceleration  -> gipps_free()
  * Vehicle.getSpeedForBraking       -> gipps_safe(), with
    getLeadersPerceivedDeceleration = (b_lead + b_foll) / 2
  * getNewSpeedGippsModel           -> gipps_next(): max(0, min(v_a, v_b)),
    rounded to 2 decimals (Utilities.precision2)
  * getDxTemporalDesync (scenarios 0,1,3,4,5) -> perceived_gap()
  * recordPosition / getHistoricalDistanceInSegment (B-entry FIFO,
    latest entry with step <= t-k else oldest) -> HistoryBuffer

Key semantic notes (mirroring Vehicle.java):
  - Only the GAP is stale under attack; the leader speed used by the safe
    term stays current (getSpeedForBraking reads leader.speed live).
  - Packet drop (scenario 4) uses a FIXED 5-step lookback on drop steps,
    not an accumulating staleness.
  - Joint (scenario 5): jittered position always; replaced by the pure
    historical position (no jitter) on delay-draw steps.
  - Jitter is truncated Gaussian(0, sigma, +/-5).

Class parameters MUST match configs/vtypes.add.xml; run_sumo.py asserts
this at startup (fail-fast on mapping drift).
"""

import math
from collections import deque

TIME_STEP = 1.0
REACTION_TIME = 1.0
HISTORY_CAP = 10          # POSITION_BUFFER_SIZE
DROP_LOOKBACK = 5         # maxStale = round(5.0 / TIME_STEP)
JITTER_TRUNC = 5.0        # +/-5 m truncation

# Documented mapping from DhakaSim vehicle classes. MUST match vtypes.add.xml
# (accel/desired-speed/length compared directly; braking compared by
# ABSOLUTE value because DhakaSim stores maxBraking NEGATIVE --
# Vehicle.java: "private double maxBraking; // negative(must)", default -6 --
# while SUMO vTypes use positive decel. The safe-speed formula below takes
# b_foll and b_hat NEGATIVE, exactly as Vehicle.getSpeedForBraking does.
# (maxAccel, maxBraking[negative], desiredSpeed, length)
CLASS_PARAMS = {
    "slow": (1.0, -3.5, 5.0, 3.0),
    "med": (1.8, -4.5, 9.0, 4.5),
    "fast": (2.2, -5.5, 13.9, 7.0),
}


def gipps_free(v, a_max, v_des, T=REACTION_TIME):
    """Vehicle.getSpeedForAcceleration."""
    return v + 2.5 * a_max * T * (1.0 - v / v_des) * math.sqrt(
        0.025 + v / v_des)


def gipps_safe(v, gap, v_lead, b_foll, b_hat, T=REACTION_TIME):
    """Vehicle.getSpeedForBraking(leader, follower, Dx).

    b_foll and b_hat are NEGATIVE (DhakaSim convention). With negative
    braking the radicand stays positive in all feasible car-following
    regimes; the max(0.0, .) guard only fires in infeasible-braking
    regimes where DhakaSim's unguarded sqrt would yield NaN. That guard is
    a documented safety deviation: the published 8,100-run campaign never
    reaches such regimes, so it cannot move any validation statistic.
    """
    dT = b_foll * T
    inside = (dT * dT - b_foll * (2.0 * gap - v * T
                                  - (v_lead * v_lead) / b_hat))
    return dT + math.sqrt(max(0.0, inside))


def gipps_next(v, gap, v_lead, a_max, b_foll, b_hat, v_des):
    """Vehicle.getNewSpeedGippsModel: min(free, safe), floored at 0."""
    v_a = gipps_free(v, a_max, v_des)
    v_b = gipps_safe(v, gap, v_lead, b_foll, b_hat)
    return round(max(0.0, min(v_a, v_b)), 2)


def truncated_gaussian(rng, sigma, bound=JITTER_TRUNC):
    """Utilities.truncatedGaussian(sigma, -5, 5): rejection sampling."""
    while True:
        x = rng.gauss(0.0, sigma)
        if abs(x) <= bound:
            return x


class HistoryBuffer:
    """Per-vehicle FIFO position history (Vehicle.recordPosition).

    Entry: (step, x, y, speed). Cap = HISTORY_CAP oldest-evicted, exactly
    like positionHistory with POSITION_BUFFER_SIZE.
    """

    def __init__(self, cap=HISTORY_CAP):
        self.buf = deque(maxlen=cap)

    def record(self, step, x, y, speed):
        self.buf.append((step, x, y, speed))

    def lookup(self, step, delay_steps):
        """getHistoricalDistanceInSegment: latest entry with
        entry.step <= step - delay_steps, else oldest entry, else None."""
        if not self.buf:
            return None
        target = step - delay_steps
        best = None
        for e in reversed(self.buf):
            if e[0] <= target:
                best = e
                break
        if best is None:
            best = self.buf[0]
        return best


def scenario_delay(scenario, delay_steps, drop_steps=DROP_LOOKBACK):
    """Lookback used by HistoryBuffer.lookup for each scenario.

    Packet drop always reads the fixed DROP_LOOKBACK-old entry on drop
    steps (mirroring getDxWithDelay(leader, follower, maxStale)); every
    other delay-family scenario reads the configured delay.
    """
    if scenario == "pdr":
        return drop_steps
    return delay_steps


def perceived_gap(scenario, rng, true_gap, ego_xy, leader_len,
                  hist_entry, jitter_sigma,
                  prob=0.5, single_shot_step=None, step=None):
    """Mirror of Vehicle.getDxTemporalDesync. Returns (perceived_gap, error).

    hist_entry: (step, x, y, speed) tuple from
    HistoryBuffer.lookup(step, scenario_delay(...)) performed by the CALLER
    with the scenario's lookback (the lookup encodes staleness); None (no
    history yet) falls back to the true gap, mirroring the Java null-guard.
    ego_xy: follower (x, y) NOW. leader_len: leader vehicle length, since
    the stale euclidean range must be converted to a bumper-to-bumper gap
    (DhakaSim subtracts follower length + THRESHOLD_DISTANCE from the
    leader's historical distance; SUMO's getLeader gap is already
    bumper-to-bumper).
    RNG draw order matches the Java switch (Bernoulli first, jitter draws
    only on taken branches).
    """
    if hist_entry is None:
        return true_gap, 0.0

    def stale():
        return stale_range_at(hist_entry, ego_xy, leader_len)

    if scenario == "none":
        return true_gap, 0.0
    if scenario == "pd":
        g = stale()
        return g, g - true_gap
    if scenario == "if":
        if rng.random() < prob:
            g = stale()
            return g, g - true_gap
        return true_gap, 0.0
    if scenario == "ss":
        if step is not None and step == single_shot_step:
            g = stale()
            return g, g - true_gap
        return true_gap, 0.0
    if scenario == "jo":
        g = true_gap + truncated_gaussian(rng, jitter_sigma)
        return g, g - true_gap
    if scenario == "pdr":
        if rng.random() < prob:
            g = stale()
            return g, g - true_gap
        return true_gap, 0.0
    if scenario == "jm":
        j = true_gap + truncated_gaussian(rng, jitter_sigma)
        if rng.random() < prob:
            g = stale()
            return g, g - true_gap
        return j, j - true_gap
    raise ValueError(f"unknown scenario {scenario}")


def stale_range_at(hist_entry, ego_xy, leader_len):
    """Perceived gap from an ALREADY-LOOKUPED historical entry.

    The caller performs HistoryBuffer.lookup(step, delay_steps) first (the
    lookup encodes the delay); this helper only converts the stored stale
    position into a bumper-to-bumper gap. Kept separate so unit tests can
    inject entries directly.
    """
    hx, hy = hist_entry[1], hist_entry[2]
    return math.hypot(ego_xy[0] - hx, ego_xy[1] - hy) - leader_len


def check_vtypes_against_xml(vtypes_path):
    """Fail-fast consistency check: CLASS_PARAMS vs configs/vtypes.add.xml."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(vtypes_path)
    mism = []
    for vt in tree.getroot().findall("vType"):
        vid = vt.get("id")
        if vid not in CLASS_PARAMS:
            mism.append(f"unknown vType {vid}")
            continue
        a, b, vd, ln = CLASS_PARAMS[vid]
        got = (float(vt.get("accel")), float(vt.get("decel")),
               float(vt.get("maxSpeed")), float(vt.get("length")))
        want = (a, abs(b), vd, ln)  # xml decel is positive; code keeps it negative
        if any(abs(x - y) > 1e-9 for x, y in zip(got, want)):
            mism.append(f"{vid}: xml={got} code={want}")
    if mism:
        raise SystemExit("CLASS_PARAMS drift vs vtypes.add.xml:\n"
                         + "\n".join(mism))
    return True
