"""
13_defense_figures.py - Figures for the radar-referenced detector and the
detector-gated correction stack.

Inputs : detector2_<topo>.json    (from 11_detector_eval.py)
         defense_stack_<topo>.json (from 12_defense_stack.py)
Outputs: paper/figures/detector_roc.png    single column
         paper/figures/defense_stack.png   single column
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

NAMES = {0: "Persistent", 1: "Intermittent", 2: "Single-Shot",
         3: "Jitter-Only", 4: "Packet Drop", 5: "Joint"}
TOPO = {"shahbagh": "Shahbagh", "sp": "Shankar-Palashi", "sll": "Straight-Line"}
CLR = ["#0f766e", "#c2410c", "#4338ca"]
OUT = "paper/figures"

# ---- Figure: detection rate per variant at three operating points ---------
det = {t: json.load(open(f"detector2_{t}.json")) for t in TOPO}
fig, ax = plt.subplots(figsize=(5.0, 3.1))
x = np.arange(6)
w = 0.26
for i, (k, lab) in enumerate(zip(["fpr0.001", "fpr0.01", "fpr0.05"],
                                 ["0.1%", "1%", "5%"])):
    ps = det["shahbagh"]["task_A"]["moving"]["d1"][k]["per_scenario"]
    ax.bar(x + (i - 1) * w, [ps[str(s)]["tpr"] * 100 for s in range(6)], w,
           label=f"FPR = {lab}", color=["#0f766e", "#2563eb", "#d97706"][i],
           edgecolor="black", linewidth=0.4)
aucs = det["shahbagh"]["task_A"]["moving"]["d1"]["auc_by_scenario"]
for s in range(6):
    ax.text(s, 3, f"AUC {aucs[str(s)]:.3f}", ha="center", fontsize=6.2,
            rotation=90, va="bottom", color="black")
ax.set_xticks(x)
ax.set_xticklabels([NAMES[s] for s in range(6)], rotation=20, ha="right",
                   fontsize=8)
ax.set_ylabel("True-positive rate (%)", fontsize=9)
ax.set_ylim(0, 128)
ax.legend(fontsize=7.5, loc="upper center", ncol=3, framealpha=0.95)
ax.tick_params(labelsize=8)
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(f"{OUT}/detector_roc.png", dpi=240)
plt.close(fig)
print("saved detector_roc.png")

# ---- Figure: correction vs the oracle-delay bound across delay levels -----
ds = {t: json.load(open(f"defense_stack_{t}.json")) for t in TOPO}
fig, ax = plt.subplots(figsize=(5.0, 3.1))
for i, t in enumerate(TOPO):
    d = ds[t]["persistent_by_delay"]
    ks = sorted(d, key=float)
    ax.plot([float(k) for k in ks], [d[k]["red_causal_det"] for k in ks],
            "-o", ms=4, lw=1.6, color=CLR[i], label=TOPO[t])
    ax.plot([float(k) for k in ks], [d[k]["red_oracle_oracle"] for k in ks],
            "--s", ms=3, lw=1.1, color=CLR[i], alpha=0.55)
ax.set_xlabel("Configured delay $\\tau$ (s)", fontsize=9)
ax.set_ylabel("Gap-error reduction (%)", fontsize=9)
ax.set_ylim(18, 92)
ax.legend(fontsize=7.5, title="solid: causal $\\hat{\\tau}$ + detector\n"
                              "dashed: oracle $\\tau$",
          title_fontsize=7, loc="upper right", framealpha=0.95)
ax.tick_params(labelsize=8)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUT}/defense_stack.png", dpi=240)
plt.close(fig)
print("saved defense_stack.png")
