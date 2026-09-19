"""
Car-following model interaction with temporal desync attacks.
Shows how different CF models (Gipps, IDM, Krauss) respond differently
to delayed/jittered gap estimates.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import os

FIGURES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../figures"))
os.makedirs(FIGURES_DIR, exist_ok=True)


def simulate_cf_response(cf_type: str, true_gaps: np.ndarray, attacked_gaps: np.ndarray, dt: float = 1.0) -> np.ndarray:
    """Simulate a car-following model's response to attacked gaps."""
    n = len(true_gaps)
    speed = np.zeros(n)
    speed[0] = 5.0
    max_accel = 3.0
    max_brake = -6.0
    max_speed = 15.0

    for t in range(1, n):
        gap = attacked_gaps[t - 1] if t > 0 else true_gaps[0]
        v_lead = 5.0  # approximate

        if cf_type == "gipps":
            b = -4.0
            bcap = -4.0
            s = 2.0
            term1 = speed[t - 1] + 2.5 * max_accel * dt * (1 - speed[t - 1] / max_speed) * np.sqrt(0.025 + speed[t - 1] / max_speed)
            term2 = (b * b) * (dt * dt) - b * (2 * (gap + 4.5 - s) - speed[t - 1] * dt - (v_lead * v_lead) / bcap)
            if term2 >= 0:
                Vnew = min(term1, b * dt + np.sqrt(term2))
            else:
                Vnew = term1
            speed[t] = max(0, min(max_speed, Vnew))

        elif cf_type == "idm":
            a = 3.0
            b = 4.0
            s0 = 2.0
            T = 1.5
            c = 2 * np.sqrt(a * b)
            s_star = s0 + max(0, speed[t - 1] * T + speed[t - 1] * (speed[t - 1] - v_lead) / c)
            acc = a * (1 - (speed[t - 1] / max_speed) ** 4 - (s_star / max(gap, 0.1)) ** 2)
            acc = max(max_brake, min(max_accel, acc))
            speed[t] = max(0, min(max_speed, speed[t - 1] + acc * dt))

        else:  # krauss
            a = max_accel
            b = 4.0
            tau = 0.3
            gap_des = v_lead * tau
            v_safe = v_lead + (gap - gap_des) / ((speed[t - 1] + v_lead) / (2 * b) + tau)
            v_des = min(speed[t - 1] + a * dt, min(v_safe, max_speed))
            speed[t] = max(0, v_des)

    return speed


def plot_cf_model_comparison(save: bool = True):
    """Compare how CF models react to persistent delay."""
    np.random.seed(42)
    n_steps = 300
    dt = 1.0

    # Generate realistic gap scenario: leader brakes at t=100
    leader_pos = np.zeros(n_steps)
    leader_speed = np.ones(n_steps) * 5.0
    leader_speed[100:130] = np.linspace(5, 2, 30)
    leader_speed[130:160] = np.linspace(2, 5, 30)
    for t in range(1, n_steps):
        leader_pos[t] = leader_pos[t - 1] + leader_speed[t] * dt

    follower_pos = leader_pos - 12.0
    true_gaps = leader_pos - follower_pos - 4.5

    # Attacked: 3s delay
    attacked_gaps = np.copy(true_gaps)
    for t in range(3, n_steps):
        attacked_gaps[t] = leader_pos[t - 3] - follower_pos[t] - 4.5

    cf_types = ["gipps", "idm", "krauss"]
    colors = {"gipps": "blue", "idm": "green", "krauss": "orange"}

    fig, axes = plt.subplots(2, 1, figsize=(14, 10))

    ax = axes[0]
    ax.plot(true_gaps, 'g-', linewidth=2, label="True Gap", alpha=0.8)
    ax.plot(attacked_gaps, 'r--', linewidth=1.5, label="Attacked Gap (3s delay)", alpha=0.7)
    ax.set_ylabel("Gap (m)")
    ax.set_title("Gap Profile: Leader Braking Maneuver with 3s Sensor Delay")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    for cf in cf_types:
        speed_true = simulate_cf_response(cf, true_gaps, true_gaps, dt)
        speed_att = simulate_cf_response(cf, true_gaps, attacked_gaps, dt)
        ax.plot(speed_true, color=colors[cf], linestyle='-', linewidth=1.5,
                label=f"{cf.upper()} (true gap)", alpha=0.6)
        ax.plot(speed_att, color=colors[cf], linestyle='--', linewidth=1.5,
                label=f"{cf.upper()} (attacked)", alpha=0.8)

    ax.set_xlabel("Time Step")
    ax.set_ylabel("Follower Speed (m/s)")
    ax.set_title("CF Model Response: True Gap vs. Attacked Gap")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "cf_model_interaction.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def plot_delay_speed_impact_by_cf(save: bool = True):
    """Final speed impact of delay for each CF model."""
    cf_types = ["gipps", "idm", "krauss"]
    delays = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0]
    np.random.seed(42)
    n_steps = 500
    dt = 1.0

    fig, ax = plt.subplots(figsize=(10, 6))

    for cf in cf_types:
        final_speeds = []
        for delay in delays:
            leader_pos = np.cumsum(np.random.normal(5.0, 0.5, n_steps)) + 20
            follower_pos = leader_pos - 10.0
            true_gaps = leader_pos - follower_pos - 4.5
            attacked_gaps = np.copy(true_gaps)
            delay_int = int(round(delay))
            for t in range(delay_int, n_steps):
                attacked_gaps[t] = leader_pos[t - delay_int] - follower_pos[t] - 4.5

            speed = simulate_cf_response(cf, true_gaps, attacked_gaps, dt)
            final_speeds.append(np.mean(speed[-100:]))

        ax.plot(delays, final_speeds, '-o', linewidth=2, markersize=8, label=cf.upper())

    ax.set_xlabel("Sensor Delay (seconds)")
    ax.set_ylabel("Average Speed (last 100 steps, m/s)")
    ax.set_title("Long-Run Speed Impact of Delay by CF Model")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save:
        path = os.path.join(FIGURES_DIR, "cf_delay_speed_impact.png")
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    plt.close()


def main():
    plot_cf_model_comparison()
    plot_delay_speed_impact_by_cf()
    print(f"CF model figures saved to: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
