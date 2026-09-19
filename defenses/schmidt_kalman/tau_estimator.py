"""
Causal delay (tau) estimator for the velocity-compensated defense.

The delay is directly observable from the discrepancy between the FRESH
radar range g_radar(t) (DhakaSim senses the immediate leader via
min-of-N radar readings, unaffected by the V2V timing attack) and the
perceived V2V-derived gap g_hat(t):

    g_hat(t) - g_radar(t)  ~=  -v_leader(t - tau) * tau        (Eq. 5)

so a per-step instantaneous estimate is

    tau_hat(t) = (g_radar(t) - g_hat(t)) / v_leader(t - tau_hat_prev)

clamped to a physically plausible range and smoothed with an
exponentially weighted median over a trailing window (causal: only
data up to and including step t is used).

Reference context: Schmidt-Kalman treatment of timing uncertainty
(Schmidt 1966); the estimator here actively estimates the delay tau
that the reduced-order Schmidt-Kalman filter treats as a consider
parameter. The estimator requires only signals available on a vehicle
with forward radar + V2V: no ground truth is used.
"""

import numpy as np


class RadarTauEstimator:
    """
    Causal online estimator of the V2V timing offset tau using the
    fresh radar range and the perceived (delayed) V2V gap.

    Attributes:
        tau_hat (float): current delay estimate (seconds)
        history (list): per-step instantaneous estimates (for warm-up
            and analysis)
    """

    def __init__(self, tau_init=1.0, window=30, min_speed=0.5,
                 tau_min=0.0, tau_max=15.0, ema_alpha=0.2):
        self.tau = tau_init
        self.window = window
        self.min_speed = min_speed
        self.tau_min = tau_min
        self.tau_max = tau_max
        self.alpha = ema_alpha
        self.history = []

    def step(self, g_radar, g_hat, v_leader):
        """
        Update the estimate with one new sample (all causally available).

        Args:
            g_radar: fresh radar range to the immediate leader (m)
            g_hat:   perceived V2V-derived gap (m)
            v_leader: leader speed at the stale step, from the
                      position-history buffer (m/s)
        """
        inst = np.nan
        if abs(v_leader) > self.min_speed:
            inst = (g_radar - g_hat) / v_leader
            if not (self.tau_min <= inst <= self.tau_max):
                inst = np.nan
        self.history.append(inst)
        windowed = [x for x in self.history[-self.window:] if not np.isnan(x)]
        if windowed:
            # Median of the trailing window, EMA-smoothed for stability
            med = float(np.median(windowed))
            self.tau = (1 - self.alpha) * self.tau + self.alpha * med
            self.tau = min(max(self.tau, self.tau_min), self.tau_max)
        return self.tau


def estimate_tau_series(perceived, leader_speeds, radar_gaps=None,
                        tau_init=1.0, window=30, min_speed=0.5):
    """
    Causal per-step delay estimate for a full series.

    Args:
        perceived: perceived (delayed V2V) gap series
        leader_speeds: leader speed series as stored in the buffer
        radar_gaps: fresh radar range series; if None, the perceived
            gap is used as its own reference (degrades to the stale
            self-comparison; pass radar data for real deployments)
        tau_init: initial delay guess (s)
        window: trailing window for the median
        min_speed: below this leader speed, the sample is uninformative
    """
    n = len(perceived)
    tau_hat = np.full(n, np.nan)
    if radar_gaps is None:
        radar_gaps = perceived  # fall back (only correct if attack absent)
    est = RadarTauEstimator(tau_init=tau_init, window=window,
                            min_speed=min_speed)
    for t in range(n):
        # Use the leader speed at the stale step (tau_hat steps back)
        j = t - max(1, int(round(est.tau)))
        v_j = leader_speeds[j] if j >= 0 else leader_speeds[0]
        tau_hat[t] = est.step(radar_gaps[t], perceived[t], v_j)
    return tau_hat


def estimate_staleness(perceived, tau_hat_series, leader_speeds,
                       fs=1.0, threshold=0.25):
    """
    Causal staleness mask: a sample is stale when the gap differs from
    the tau-compensated prediction by more than `threshold` meters.

    stale(t) = |p(t) - (p(t - tau) + v_leader(t - tau) * tau)| > threshold
    """
    n = len(perceived)
    mask = np.zeros(n, dtype=bool)
    for t in range(n):
        tau = tau_hat_series[t]
        tau_steps = max(1, int(round(tau / fs)))
        j = t - tau_steps
        if j < 0:
            continue
        pred = perceived[j] + leader_speeds[j] * tau
        if abs(perceived[t] - pred) > threshold:
            mask[t] = True
    return mask
