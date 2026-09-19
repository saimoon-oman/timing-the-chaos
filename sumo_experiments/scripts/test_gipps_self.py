"""Unit self-test for gipps_controller.py (no SUMO needed).

Run: python scripts/gipps_controller.py  (or pytest-style import)
Checks:
  1. Gipps free/safe/next match hand-computed values.
  2. HistoryBuffer FIFO cap + lookup semantics (latest <= t-k else oldest).
  3. perceived_gap branches against a scripted RNG.
"""
import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gipps_controller import (
    gipps_free, gipps_safe, gipps_next, HistoryBuffer, perceived_gap,
    scenario_delay, truncated_gaussian, CLASS_PARAMS,
)


def test_gipps():
    # free term: v=5, a=1.8, vdes=9 -> 5 + 2.5*1.8*1*(1-5/9)*sqrt(0.025+5/9)
    v_a = gipps_free(5.0, 1.8, 9.0)
    exp_a = 5.0 + 2.5 * 1.8 * (1 - 5.0 / 9.0) * math.sqrt(0.025 + 5.0 / 9.0)
    assert abs(v_a - exp_a) < 1e-9, (v_a, exp_a)
    # safe term with DhakaSim's NEGATIVE braking convention:
    # v=5, gap=20, vl=5, b=-4.5, bhat=-4.5
    v_b = gipps_safe(5.0, 20.0, 5.0, -4.5, -4.5)
    dT = -4.5
    exp_b = dT + math.sqrt(dT * dT - (-4.5) * (40.0 - 5.0 - 25.0 / -4.5))
    assert abs(v_b - exp_b) < 1e-9, (v_b, exp_b)
    assert v_b > 0, v_b  # feasible regime: positive safe speed
    # next = min, floored, rounded
    assert gipps_next(5.0, 20.0, 5.0, 1.8, -4.5, -4.5, 9.0) == round(
        max(0.0, min(exp_a, exp_b)), 2)
    # stopped follower stays stopped-ish, huge gap -> free term
    assert gipps_next(0.0, 500.0, 8.0, 1.8, -4.5, -4.5, 9.0) > 0
    # class params use negative braking like DhakaSim
    for _c, (_a, b, _vd, _ln) in CLASS_PARAMS.items():
        assert b < 0, _c
    print("gipps OK")


def test_history():
    h = HistoryBuffer(cap=10)
    for t in range(12):
        h.record(t, 100.0 + t, 0.0, 5.0)
    assert len(h.buf) == 10 and h.buf[0][0] == 2 and h.buf[-1][0] == 11
    # latest entry with step <= 11-3=8 -> entry step 8, x=108
    e = h.lookup(11, 3)
    assert e[0] == 8 and e[1] == 108.0, e
    # target 11-12<0 -> oldest entry (step 2)
    e = h.lookup(11, 12)
    assert e[0] == 2, e
    # empty buffer -> None
    assert HistoryBuffer().lookup(5, 3) is None
    print("history OK")


class ScriptRNG:
    """Deterministic draw script for branch tests."""

    def __init__(self, uniforms, gausses):
        self.u = list(uniforms)
        self.g = list(gausses)

    def random(self):
        return self.u.pop(0)

    def gauss(self, mu, sigma):
        assert mu == 0.0
        return self.g.pop(0)


def test_branches():
    entry = (7, 90.0, 0.0, 5.0)  # leader was at x=90 at t=7
    ego = (100.0, 0.0)
    true_gap, L = 8.0, 4.5
    stale_gap = 10.0 - 4.5  # |100-90| - L = 5.5
    assert scenario_delay("pdr", 3) == 5
    assert scenario_delay("pd", 3) == 3
    g, e = perceived_gap("pd", ScriptRNG([], []), true_gap, ego, L,
                         entry, 1.0)
    assert (g, e) == (stale_gap, stale_gap - true_gap), (g, e)
    g, e = perceived_gap("none", ScriptRNG([], []), true_gap, ego, L,
                         entry, 1.0)
    assert (g, e) == (true_gap, 0.0)
    # intermittent: draw 0.2 < 0.5 -> stale; draw 0.9 -> current
    g, _ = perceived_gap("if", ScriptRNG([0.2], []), true_gap, ego, L,
                         entry, 1.0, prob=0.5)
    assert g == stale_gap
    g, _ = perceived_gap("if", ScriptRNG([0.9], []), true_gap, ego, L,
                         entry, 1.0, prob=0.5)
    assert g == true_gap
    # jitter-only: gauss draw used
    g, e = perceived_gap("jo", ScriptRNG([], [1.0]), true_gap, ego, L,
                         entry, 1.0)
    assert g == true_gap + 1.0 and e == 1.0
    # joint: jitter then bernoulli; bernoulli true -> pure stale
    g, _ = perceived_gap("jm", ScriptRNG([0.1], [2.0]), true_gap, ego, L,
                         entry, 1.0, prob=0.5)
    assert g == stale_gap
    # joint bernoulli false -> jittered value
    g, _ = perceived_gap("jm", ScriptRNG([0.9], [2.0]), true_gap, ego, L,
                         entry, 1.0, prob=0.5)
    assert g == true_gap + 2.0
    # single shot fires only at the configured step
    g, _ = perceived_gap("ss", ScriptRNG([], []), true_gap, ego, L,
                         entry, 1.0, single_shot_step=150, step=150)
    assert g == stale_gap
    g, _ = perceived_gap("ss", ScriptRNG([], []), true_gap, ego, L,
                         entry, 1.0, single_shot_step=150, step=149)
    assert g == true_gap
    # no history -> true gap
    g, e = perceived_gap("pd", ScriptRNG([], []), true_gap, ego, L,
                         None, 1.0)
    assert (g, e) == (true_gap, 0.0)
    # truncation respected
    rng = random.Random(0)
    for _ in range(2000):
        assert abs(truncated_gaussian(rng, 2.0)) <= 5.0
    print("branches OK")


if __name__ == "__main__":
    test_gipps()
    test_history()
    test_branches()
    print("CLASS_PARAMS classes:", sorted(CLASS_PARAMS))
    print("ALL SELF-TESTS PASSED")
