"""Spacing + clearance cost + virtual lines continuing walls across openings."""
import pickle, sys, statistics as st
sys.path.insert(0, sys.argv[1])
import evalp
from evalp import *

def wall_lines(m, min_wall_m, max_gap_m):
    c = np.asarray(m["cells"]).reshape(m["height"], m["width"]); res = m["resolution_m"]
    occ = c == CELL_OCCUPIED; h, w = occ.shape
    L, G = max(1, round(min_wall_m / res)), round(max_gap_m / res)
    line = np.zeros_like(occ)
    for dj, di in ((1,0),(-1,0),(0,1),(0,-1)):
        for j, i in zip(*np.nonzero(occ)):
            nj, ni = j + dj, i + di
            if not (0 <= nj < h and 0 <= ni < w) or occ[nj, ni]: continue
            # a wall run of >= L behind this end, along the axis
            if not all(0 <= j - k*dj < h and 0 <= i - k*di < w and occ[j - k*dj, i - k*di] for k in range(L)):
                continue
            path = []
            for k in range(1, G + 1):
                pj, pi = j + k*dj, i + k*di
                if not (0 <= pj < h and 0 <= pi < w): path = None; break
                if occ[pj, pi]: break
                path.append((pj, pi))
            else:
                path = None
            if path:
                for q in path: line[q] = True
    return line & (c == CELL_FREE)

def run_all(runs, spacing, c0, p, lw, lg, pen, pen_ask=None):
    rows = []
    for run in runs:
        m = run["map"]; free, clr = grids(m); res = m["resolution_m"]
        wl = wall_lines(m, lw, lg); cost = costmap(clr, res, c0, p) + pen * wl
        acost = cost if pen_ask is None else costmap(clr, res, c0, p) + pen_ask * wl
        seeds, names, d = [], [], None
        for t, x, y, room in run["trace"]:
            if room == "unknown": continue
            j, i = cellof(m, x, y)
            if not (0 <= j < free.shape[0] and 0 <= i < free.shape[1]) or not free[j, i]: continue
            if d is None or d[j, i] > spacing:
                seeds.append((j, i)); names.append(room); d, _ = wdist(free, acost, seeds)
        rows.append((len(seeds), score_labels(build_world(run["house"]), label(m, seeds, names, free, cost), m["width"])))
    calls = [r[0] for r in rows]; per = [v for _, s in rows for v in s["per_room"].values()]
    return (st.median(calls), max(calls), sum(v >= .95 for v in per), len(per), round(st.median(per), 3),
            st.median(s["wrong_deep"] for _, s in rows))

runs = pickle.load(open(sys.argv[2], "rb"))
for spec in sys.argv[3:]:
    print(spec, run_all(runs, *[float(v) for v in spec.split(",")]), flush=True)
