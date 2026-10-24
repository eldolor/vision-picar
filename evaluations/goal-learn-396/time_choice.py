"""
evaluations/goal-learn-396/time_choice.py -- 3.96 criterion 6: one whole
goal choice under F-large on 3.77's benchmark (the six end-of-run maps of
3.65's live `home_first_floor` runs at 0.05 m, 20 poses each from 3.77's
seed, camera-seen set empty: the most view gaps).

A choice is what `_next_frontier` would do under F-large: one shared
`reach`, `find_frontiers`, `find_view_gaps`, the ten features per
candidate, `segment()` of the map and the room features, and the network's
score. The room labels are taken as absent (a find mission with no survey:
`type_known` 0), and the trail as the 30 nearest free cells to the pose
(found before the clock starts: the brain keeps its trail). Prints
p50/p90/max of the whole choice and of `room_features()` (the resample, the
room cut and the feature loop) over the poses that have candidates.

    python evaluations/goal-learn-396/time_choice.py [OUT.json]
"""
import json
import os
import platform
import random
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, HERE)
from brain import frontier as fr                              # noqa: E402
from brain.goal_policy import features                        # noqa: E402
from brain.room_labels import segment                         # noqa: E402
from features import coarse, room_features                    # noqa: E402
from model import score_np, to_np                             # noqa: E402
from tests import frontier_compare as fc                      # noqa: E402
from world.interface import CELL_FREE                         # noqa: E402

POSES = 20
BEST = os.path.join(HERE, "cloud", "ckpt", "F-large", "best.json")


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(0.9 * len(xs)))]


def trail_near(m, x, y, n=30):
    w, res, ox, oy = m["width"], m["resolution_m"], m["origin_x_m"], m["origin_y_m"]
    cells = np.asarray(m["cells"])
    free = np.flatnonzero(cells == CELL_FREE)
    px, py = ox + (free % w + 0.5) * res, oy + (free // w + 0.5) * res
    near = np.argsort((px - x) ** 2 + (py - y) ** 2)[:n]
    return list(zip(px[near].tolist(), py[near].tolist()))


def choice(m, x, y, heading, P, trail):
    pose = {"x_m": x, "y_m": y, "heading_deg": heading}
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m, size_m=f.size_m)
             for f in fs]
    feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m, area_m2=g.area_m2)
              for g in gs]
    goals = [f.goal for f in fs] + [g.goal for g in gs]
    if not goals:
        return 0, 0.0
    t = time.perf_counter()
    room = room_features(m, pose, goals, trail=trail, types=[None] * len(goals),
                         prior={})
    seg_ms = (time.perf_counter() - t) * 1000
    X = np.asarray([a + b for a, b in zip(feats, room)], dtype=np.float32)
    int(np.argmax(score_np(P, "F", X)))
    return len(goals), seg_ms


def main(argv):
    out = next((a for a in argv if a.endswith(".json")), None)
    P = to_np(json.load(open(BEST))["params"])
    rng = random.Random(fc.SEED)
    whole, rooms, counts, empty = [], [], [], 0
    for path in fc.MAPS:
        m = fc.load(path)
        segment(coarse(m))               # warm once, as a running brain would be
        for x, y in fc.poses(m, POSES, rng):
            trail, heading = trail_near(m, x, y), rng.uniform(0, 360)
            t = time.perf_counter()
            n, seg_ms = choice(m, x, y, heading, P, trail)
            if not n:
                empty += 1
                continue
            whole.append((time.perf_counter() - t) * 1000)
            rooms.append(seg_ms)
            counts.append(n)
    summary = {"host": platform.node(), "machine": platform.machine(), "poses": len(whole),
               "no_candidates": empty, "candidates_median": statistics.median(counts)}
    for name, xs in (("whole_choice", whole), ("room_features", rooms)):
        summary[name] = {"p50_ms": round(statistics.median(xs), 1), "p90_ms": round(p90(xs), 1),
                         "max_ms": round(max(xs), 1)}
        print(f"{name:14s} n={len(xs)}  p50 {summary[name]['p50_ms']} ms  "
              f"p90 {summary[name]['p90_ms']} ms  max {summary[name]['max_ms']} ms")
    print(f"candidates per choice: median {summary['candidates_median']}, max {max(counts)}")
    if out:
        with open(out, "w") as f:
            f.write(json.dumps(summary, indent=1) + "\n")


if __name__ == "__main__":
    main(sys.argv[1:])
