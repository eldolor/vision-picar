"""Spacing + densify near disagreements."""
import pickle, sys, statistics as st
sys.path.insert(0, sys.argv[1])
from evalp import *

def policy2(run, spacing, c0, p, smin, band):
    m = run["map"]; free, clr = grids(m); res = m["resolution_m"]
    cost = costmap(clr, res, c0, p)
    seeds, names = [], []
    bylab = {}
    for t, x, y, room in run["trace"]:
        if room == "unknown": continue
        j, i = cellof(m, x, y)
        if not (0 <= j < free.shape[0] and 0 <= i < free.shape[1]) or not free[j, i]: continue
        ds = sorted(d[j, i] for d in bylab.values())
        ask = not ds or ds[0] > spacing or (len(ds) > 1 and ds[0] > smin and ds[1] - ds[0] < band)
        if ask:
            seeds.append((j, i)); names.append(room)
            bylab[room] = wdist(free, cost, [s for s, n in zip(seeds, names) if n == room])[0]
    return seeds, names, free, cost

def evaluate2(runs, *a):
    rows = []
    for run in runs:
        m = run["map"]
        seeds, names, free, cost = policy2(run, *a)
        rows.append((len(seeds), score_labels(build_world(run["house"]), label(m, seeds, names, free, cost), m["width"])))
    calls = [r[0] for r in rows]; per = [v for _, s in rows for v in s["per_room"].values()]
    return (st.median(calls), max(calls), sum(v >= .95 for v in per), len(per), round(st.median(per), 3),
            st.median(s["wrong_deep"] for _, s in rows))

runs = pickle.load(open(sys.argv[2], "rb"))
for spec in sys.argv[3:]:
    print(spec, evaluate2(runs, *[float(v) for v in spec.split(",")]), flush=True)
