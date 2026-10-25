"""
evaluations/goal-learn-3101/time_3101.py -- 3.101, reported: one whole
goal choice under F-large and under F-rich on 3.96's benchmark (3.77's six
live maps at 0.05 m, 20 poses each; 3.96's `time_choice.choice()`), on this
machine. F-rich's time is F-large's choice plus `rich_features()` and its
wider network. The Jetson's 100 ms bar is owed (the board is not reachable
from here).

    python evaluations/goal-learn-3101/time_3101.py OUT.json
"""
import json
import os
import platform
import random
import statistics
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-396"))
sys.path.insert(0, HERE)
import time_choice as T396                                    # noqa: E402
from brain import frontier as fr                              # noqa: E402
from brain.goal_policy import features                        # noqa: E402
from features import coarse, room_features                    # noqa: E402
from model import score_np, to_np                             # noqa: E402
from rich import rich_features                                # noqa: E402
from brain.room_labels import segment                         # noqa: E402
from tests import frontier_compare as fc                      # noqa: E402

F_RICH = os.path.join(os.environ.get("RUN_3101", os.path.join(HERE, "run")), "F-rich", "best.json")


def rich_choice(m, x, y, heading, P, trail):
    """3.96's choice, then the 25 features and F-rich's score."""
    pose = {"x_m": x, "y_m": y, "heading_deg": heading}
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m, size_m=f.size_m)
             for f in fs]
    feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m, area_m2=g.area_m2)
              for g in gs]
    pool = [("frontier", f, i) for i, f in enumerate(fs)] + [("view", g, None) for g in gs]
    if not pool:
        return 0, 0.0
    goals = [c.goal for _, c, _ in pool]
    room = room_features(m, pose, goals, trail=trail, types=[None] * len(goals), prior={})
    t = time.perf_counter()
    agent = types.SimpleNamespace(seen=set(), goals_sent=5)
    extra = rich_features(agent, m, pose, pool, feats, trail, 30.0)
    rich_ms = (time.perf_counter() - t) * 1000
    X = np.asarray([a + b + c for a, b, c in zip(feats, room, extra)], dtype=np.float32)
    int(np.argmax(score_np(P, "F", X)))
    return len(pool), rich_ms


def main(out: str) -> None:
    P_large = to_np(json.load(open(T396.BEST))["params"])
    P_rich = to_np(json.load(open(F_RICH))["params"])
    summary = {"host": platform.node(), "machine": platform.machine()}
    for arm in ("F-large", "F-rich"):
        rng = random.Random(fc.SEED)
        whole, rich = [], []
        for path in fc.MAPS:
            m = fc.load(path)
            segment(coarse(m))
            for x, y in fc.poses(m, T396.POSES, rng):
                trail, heading = T396.trail_near(m, x, y), rng.uniform(0, 360)
                t = time.perf_counter()
                if arm == "F-large":
                    n, ms = T396.choice(m, x, y, heading, P_large, trail)
                else:
                    n, ms = rich_choice(m, x, y, heading, P_rich, trail)
                if n:
                    whole.append((time.perf_counter() - t) * 1000)
                    rich.append(ms)
        summary[arm] = {"poses": len(whole), "whole_p50_ms": round(statistics.median(whole), 1),
                        "whole_p90_ms": round(T396.p90(whole), 1), "whole_max_ms": round(max(whole), 1)}
        if arm == "F-rich":
            summary[arm]["rich_features_p90_ms"] = round(T396.p90(rich), 1)
        print(arm, summary[arm], flush=True)
    json.dump(summary, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
