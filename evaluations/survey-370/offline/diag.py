import pickle, sys
sys.path.insert(0, sys.argv[1])
from evalp import *
from tests.demo_explore import reachable_cells
runs = pickle.load(open(sys.argv[2], "rb"))
sp, c0, p = [float(v) for v in sys.argv[3].split(",")]
from collections import Counter
cls = Counter()
for run in runs:
    m = run["map"]; w = build_world(run["house"])
    seeds, names, free, cost = policy(run, sp, c0, p)
    sc = score_labels(w, label(m, seeds, names, free, cost), m["width"])
    visited = set(r for *_, r in run["trace"])
    reach = reachable_cells(w)
    for room, v in sc["per_room"].items():
        if v >= .95: continue
        n = len([c for c in w.rooms[room] if c in reach])
        k = "unvisited" if room not in visited else ("unseeded" if room not in names else "boundary")
        cls[k] += 1
        print(run["house"][:16], room, k, "share", v, "cells", n)
print(cls)
