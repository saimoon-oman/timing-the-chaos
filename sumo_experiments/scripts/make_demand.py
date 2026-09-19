"""Seeded demand generator for the SUMO cross-validation campaign.

Samples (origin, destination) edge pairs from the largest weakly connected
component of the network, assigns departure times at a fixed insertion rate
(volume multiplier semantics, mirroring the paper's demand scaling) and a
fleet class per trip (mix semantics), routes with duarouter, and writes the
final .rou.xml. Every random draw comes from one seeded Generator; the
sidecar JSON records counts for the run manifest.

Usage:
    python make_demand.py --net shahbagh.net.xml --out demand_1x_bal_seed7.rou.xml \
        --horizon 300 --period 2.0 --mix 50,25,25 --seed 7 --duarouter <path>
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def lane_usable_by_passenger(lane):
    """A lane admits vClass=passenger iff it has no restrictive allow-list,
    or the allow-list names passenger. (Dhaka OSM ways often carry
    pedestrian/bicycle-only allow-lists; duarouter enforces the same rule,
    so the OD pool must respect it to keep the routed fraction high.)"""
    allow = lane.get("allow")
    if allow is None:
        return True  # only a disallow-list (or nothing): passenger permitted
    return "passenger" in allow.split()


def load_edges(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {}
    for e in root.findall("edge"):
        eid = e.get("id", "")
        if eid.startswith(":") or e.get("function") == "internal":
            continue
        frm, to = e.get("from"), e.get("to")
        lanes = e.findall("lane")
        if not frm or not to or not lanes:
            continue
        if not any(lane_usable_by_passenger(l) for l in lanes):
            continue
        try:
            shp = lanes[0].get("shape", "")
            pts = [tuple(map(float, p.split(","))) for p in shp.split()]
        except ValueError:
            continue
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        length = float(e.get("length", 0.0))
        edges[eid] = {"from": frm, "to": to, "mx": mx, "my": my,
                      "length": length}
    return edges


def largest_cc(edges):
    adj = {}
    for eid, e in edges.items():
        adj.setdefault(e["from"], set()).add(e["to"])
        adj.setdefault(e["to"], set()).add(e["from"])
    seen = set()
    best = set()
    for n in adj:
        if n in seen:
            continue
        stack, comp = [n], set()
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            seen.add(x)
            comp.add(x)
            stack.extend(adj[x] - seen)
        if len(comp) > len(best):
            best = comp
    keep = {eid for eid, e in edges.items()
            if e["from"] in best and e["to"] in best}
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--horizon", type=float, default=300.0)
    ap.add_argument("--period", type=float, default=2.0,
                    help="mean seconds between insertions (volume scale)")
    ap.add_argument("--mix", default="50,25,25",
                    help="slow,med,fast percentages")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--min-trip-dist", type=float, default=300.0,
                    help="min euclidean OD distance (m), avoids trivial trips")
    ap.add_argument("--duarouter", default="duarouter")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    mix = [float(x) for x in args.mix.split(",")]
    tot = sum(mix)
    cum = [mix[0] / tot, (mix[0] + mix[1]) / tot]
    classes = ["slow", "med", "fast"]

    edges = load_edges(args.net)
    keep = largest_cc(edges)
    pool = [eid for eid in edges if eid in keep]
    print(f"edges total={len(edges)} in-largest-CC={len(pool)}")

    n_trip = int(args.horizon / args.period)
    departs = sorted(rng.uniform(0, args.horizon) for _ in range(n_trip))
    trips = []
    for i, dep in enumerate(departs):
        # NOTE: vType is assigned AFTER routing (stratified, below), so the
        # realized mix matches the target exactly; the placeholder is unused.
        vtype = None
        for _ in range(50):  # rejection-sample a non-trivial OD pair
            o = rng.choice(pool)
            d = rng.choice(pool)
            if o == d:
                continue
            dx = edges[o]["mx"] - edges[d]["mx"]
            dy = edges[o]["my"] - edges[d]["my"]
            if math.hypot(dx, dy) >= args.min_trip_dist:
                break
        else:
            continue
        trips.append((f"v{i}", dep, o, d, vtype))

    with tempfile.TemporaryDirectory() as tmp:
        trippath = os.path.join(tmp, "trips.xml")
        with open(trippath, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n<trips>\n')
            for vid, dep, o, d, _vt in trips:
                f.write(f'    <trip id="{vid}" depart="{dep:.2f}" '
                        f'from="{o}" to="{d}"/>\n')
            f.write('</trips>\n')
        routepath = os.path.join(tmp, "routes.xml")
        cp = subprocess.run(
            [args.duarouter, "-n", args.net, "--trip-files", trippath,
             "-o", routepath, "--remove-loops", "--ignore-errors", "--no-warnings"],
            capture_output=True, text=True)
        if cp.returncode != 0:
            print(cp.stderr[-2000:])
            raise SystemExit("duarouter failed")
        routed = {}
        for node in ET.parse(routepath).getroot().iter():
            if node.tag in ("vehicle", "trip"):
                vid = node.get("id")
                if node.find("route") is not None or node.get("route"):
                    routed[vid] = True

    kept = [t for t in trips if t[0] in routed]
    # Stratified class assignment AFTER routing, so the realized mix matches
    # the target exactly (routing failures must not skew the composition).
    n_kept = len(kept)
    want = [int(round(n_kept * m)) for m in
            [mix[0] / tot, mix[1] / tot, mix[2] / tot]]
    want[-1] += n_kept - sum(want)  # fix rounding drift on the last class
    bag = ([classes[0]] * want[0] + [classes[1]] * want[1] +
           [classes[2]] * want[2])
    rng.shuffle(bag)
    kept = [(vid, dep, o, d, vt) for (vid, dep, o, d, _), vt
            in zip(kept, bag)]
    counts = {c: 0 for c in classes}
    for _vid, _dep, _o, _d, vt in kept:
        counts[vt] += 1

    # Re-run duarouter on the kept subset so routes match 1:1, then stamp types.
    with tempfile.TemporaryDirectory() as tmp:
        trippath = os.path.join(tmp, "trips2.xml")
        with open(trippath, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n<trips>\n')
            for vid, dep, o, d, _vt in kept:
                f.write(f'    <trip id="{vid}" depart="{dep:.2f}" '
                        f'from="{o}" to="{d}"/>\n')
            f.write('</trips>\n')
        routepath = os.path.join(tmp, "routes2.xml")
        subprocess.run(
            [args.duarouter, "-n", args.net, "--trip-files", trippath,
             "-o", routepath, "--remove-loops", "--ignore-errors", "--no-warnings"],
            capture_output=True, check=True)
        tree = ET.parse(routepath)
        vt_of = {vid: vt for vid, _dep, _o, _d, vt in kept}
        with open(args.out, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n<routes>\n')
            for veh in tree.getroot().findall("vehicle"):
                vid = veh.get("id")
                rt = veh.find("route")
                if rt is None:
                    continue
                dep = veh.get("depart")
                f.write(f'    <vehicle id="{vid}" type="{vt_of[vid]}" '
                        f'depart="{dep}">\n')
                edges_attr = rt.get("edges")
                if edges_attr:
                    f.write(f'        <route edges="{edges_attr}"/>\n')
                else:
                    for ch in list(rt):
                        f.write("        " + ET.tostring(
                            ch, encoding="unicode"))
                f.write('    </vehicle>\n')
            f.write('</routes>\n')

    sidecar = {"attempted": len(trips), "routed": len(kept),
               "per_class": counts, "seed": args.seed,
               "period": args.period, "mix": args.mix,
               "horizon": args.horizon}
    with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
        json.dump(sidecar, f, indent=1)
    print(f"attempted={len(trips)} routed={len(kept)} {counts}")


if __name__ == "__main__":
    main()
