"""Spacing + a penalty for crossing a boundary of the watershed cut."""
import pickle, sys, statistics as st
sys.path.insert(0, sys.argv[1])
from evalp import *
from brain.room_labels import segment

def seg_edges(m, ratio):
    seg, _ = segment(m, ratio)
    e = np.zeros(seg.shape, bool)
    a, b = seg[1:, :], seg[:-1, :]
    x = (a != b) & (a >= 0) & (b >= 0); e[1:, :] |= x; e[:-1, :] |= x
    a, b = seg[:, 1:], seg[:, :-1]
    x = (a != b) & (a >= 0) & (b >= 0); e[:, 1:] |= x; e[:, :-1] |= x
    return e

def run_all(runs, spacing, ratio, pen, c0=0.0, p=1.0):
    rows = []
    for run in runs:
        m = run["map"]; free, clr = grids(m); res = m["resolution_m"]
        cost = costmap(clr, res, c0, p) + pen * seg_edges(m, ratio)
        seeds, names, d = [], [], None
        for t, x, y, room in run["trace"]:
            if room == "unknown": continue
            j, i = cellof(m, x, y)
            if not (0 <= j < free.shape[0] and 0 <= i < free.shape[1]) or not free[j, i]: continue
            if d is None or d[j, i] > spacing:
                seeds.append((j, i)); names.append(room); d, _ = wdist(free, cost, seeds)
        rows.append((len(seeds), score_labels(build_world(run["house"]), label(m, seeds, names, free, cost), m["width"])))
    calls = [r[0] for r in rows]; per = [v for _, s in rows for v in s["per_room"].values()]
    return (st.median(calls), max(calls), sum(v >= .95 for v in per), len(per), round(st.median(per), 3),
            st.median(s["wrong_deep"] for _, s in rows))

runs = pickle.load(open(sys.argv[2], "rb"))
for spec in sys.argv[3:]:
    print(spec, run_all(runs, *[float(v) for v in spec.split(",")]), flush=True)
