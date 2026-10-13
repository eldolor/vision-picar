"""
evaluations/explore-365/time_choice.py -- 3.65 amendment 3 check C: how long
explore's goal choice takes on a real SLAM map (0.05 m), under the rule and
under the network. For each saved live-run map (`final_map` in a
tests/demo_explore.py status file) and POSES free cells drawn by a fixed
seed, it times what `_next_frontier` computes:

* rule: find_frontiers, plus find_view_gaps when no frontier is left;
* network: find_frontiers + find_view_gaps + each candidate's features and
  score (`_learned_goal`'s work).

The camera-seen set is not in the status file, so it is taken as empty:
every known cell counts as unseen, which makes the most view gaps (the
slow case). Prints p50/p90/max per policy and the candidate counts.

    python evaluations/explore-365/time_choice.py STATUS.json [STATUS.json ...]
"""
import json
import random
import statistics
import sys
import time

sys.path.insert(0, ".")
from brain.explore import leave_clearance_m                    # noqa: E402
from brain.frontier import find_frontiers, find_view_gaps     # noqa: E402
from brain.goal_policy import NetGoalPolicy, features         # noqa: E402
from world.interface import CELL_FREE                         # noqa: E402

POSES, SEED = 20, 3655
LEAVE = leave_clearance_m(20)      # config/robot.yaml safety.min_distance_cm


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(0.9 * len(xs)))]


def main(paths):
    net, rng = NetGoalPolicy.load(), random.Random(SEED)
    rule_ms, net_ms, counts = [], [], []
    for path in paths:
        m = json.load(open(path))["final_map"]
        res, w = m["resolution_m"], m["width"]
        free = [i for i, v in enumerate(m["cells"]) if v == CELL_FREE]
        for i in rng.sample(free, POSES):
            x = m["origin_x_m"] + (i % w + 0.5) * res
            y = m["origin_y_m"] + (i // w + 0.5) * res
            pose = {"x_m": x, "y_m": y, "heading_deg": rng.uniform(0, 360)}
            t = time.perf_counter()
            fr = find_frontiers(m, x, y, clearance_m=LEAVE)
            if not fr:
                find_view_gaps(m, x, y, set(), clearance_m=LEAVE)
            rule_ms.append((time.perf_counter() - t) * 1000)
            t = time.perf_counter()
            fr = find_frontiers(m, x, y, clearance_m=LEAVE)
            gaps = find_view_gaps(m, x, y, set(), clearance_m=LEAVE)
            feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m,
                              size_m=f.size_m) for f in fr]
            feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m,
                               area_m2=g.area_m2) for g in gaps]
            net.best(feats)
            net_ms.append((time.perf_counter() - t) * 1000)
            counts.append(len(feats))
    for name, xs in (("rule", rule_ms), ("network", net_ms)):
        print(f"{name:8s} n={len(xs)}  p50 {statistics.median(xs):.1f} ms  p90 {p90(xs):.1f} ms"
              f"  max {max(xs):.1f} ms")
    print(f"candidates per choice: median {statistics.median(counts)}, max {max(counts)}")


if __name__ == "__main__":
    main(sys.argv[1:])
