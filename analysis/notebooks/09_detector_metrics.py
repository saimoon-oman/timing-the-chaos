"""Quick TPR/FPR of a strip-deviation detector on real experiment data."""

import os, sys
import numpy as np, pandas as pd

GAP = sys.argv[1] if len(sys.argv) > 1 else r"experiments\results\run_20260808_015724_v3_ttc"
rows = []
for root, _d, files in os.walk(GAP):
    if "gap_errors.csv" not in files:
        continue
    cid = os.path.basename(root)
    df = pd.read_csv(os.path.join(root, "gap_errors.csv"))
    if df.empty or "perceivedGap" not in df.columns:
        continue
    err = (df["perceivedGap"] - df["trueGap"]).abs().values
    stale = err > 0.25  # oracle ground truth: any non-trivial desync
    for thr in [0.5, 1.0, 2.0, 3.0]:
        alarm = err > thr
        tp = np.sum(alarm & stale); fn = np.sum(~alarm & stale)
        fp = np.sum(alarm & ~stale); tn = np.sum(~alarm & ~stale)
        tpr = tp / (tp + fn) if (tp + fn) else 1.0
        fpr = fp / (fp + tn) if (fp + tn) else 0.0
        rows.append({"cid": cid, "thr_m": thr, "tpr": tpr, "fpr": fpr})

dfr = pd.DataFrame(rows)
dfr["sc"] = dfr["cid"].str.extract(r"sc(\d)")[0].fillna("base")

print("Strip-deviation detector on real data (alarm = |perceived-true| > threshold):")
for thr in [0.5, 1.0, 2.0, 3.0]:
    sub = dfr[dfr["thr_m"] == float(thr)]
    attacked = sub[sub["sc"] != "base"]
    base = sub[sub["sc"] == "base"]
    print(f"  threshold {thr:g} m: attacked TPR={attacked['tpr'].mean()*100:5.1f}% "
          f"(FPR={attacked['fpr'].mean()*100:4.1f}%), baseline FPR={base['fpr'].mean()*100:4.1f}%")

print("\nPer-scenario TPR @ 2 m threshold:")
sub = dfr[dfr["thr_m"] == 2.0]
for scv in sorted(sub["sc"].unique()):
    g = sub[sub["sc"] == scv]
    print(f"  {scv:>4}: TPR={g['tpr'].mean()*100:5.1f}%  FPR={g['fpr'].mean()*100:4.1f}%")
