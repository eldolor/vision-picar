"""
evaluations/goal-learn-3103/jetson_time.py -- 3.103 criterion 5's Jetson half
(the user, 2026-10-26: "everything running on the board"): every learned
arm's whole goal choice on 3.96's benchmark (the six live 0.05 m maps, 20
poses each, the same seed), on the board, quiet or under the full stack
(`tools/jetson/headroom_identity.sh`'s window).

At each pose every arm is timed once, in an order that rotates pose by pose,
so a change in load reaches every arm alike. F-large is 3.96's own choice
(its record: p90 312.9 ms at 15 W). F-pool-W32's choice is also split into
its parts at the same poses, to show what a rewrite would have to cut.

    python evaluations/goal-learn-3103/jetson_time.py <label> <out.json>
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
sys.path.insert(0, HERE)
import pool as POOL                                           # noqa: E402
import run as B                                               # noqa: E402  (3.96's path set up)

sys.path.insert(0, os.path.join(B.ROOT_REPO, "evaluations", "goal-learn-396"))
import time_choice as T396                                    # noqa: E402
from model import to_np                                       # noqa: E402
from brain.room_labels import segment                         # noqa: E402

fc, fr, features, room_features = B.fc, B.fr, B.features, B.room_features


def _load(path):
    return {k: np.asarray(v, dtype=np.float32) for k, v in json.load(open(path))["params"].items()}


def arms() -> dict:
    """name -> f(m, x, y, heading, trail) returning the pool's size."""
    P_large = to_np(json.load(open(T396.BEST))["params"])
    P_lean = _load(os.path.join(HERE, "run", "F-lean-S64", "best.json"))
    out = {"F-large": lambda m, x, y, h, t: T396.choice(m, x, y, h, P_large, t)[0],
           "F-lean": lambda m, x, y, h, t: B.lean_choice(m, x, y, h, t, P_lean)}
    for w, d in ((32, "cloud/ckpt"), (64, "run"), (128, "cloud/ckpt")):
        P = _load(os.path.join(HERE, d, f"F-pool-W{w}", "best.json"))
        out[f"F-pool-W{w}"] = (lambda P: lambda m, x, y, h, t:
                               B.lean_choice(m, x, y, h, t, P, POOL.score_np))(P)
    return out


def parts(m, x, y, heading, trail, P) -> dict:
    """F-pool's whole choice as B.lean_choice makes it, each part timed (ms)."""
    ms = {}
    t = time.perf_counter()
    pose = {"x_m": x, "y_m": y, "heading_deg": heading}
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    ms["map_work"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m, size_m=f.size_m)
             for f in fs]
    feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m, area_m2=g.area_m2)
              for g in gs]
    ms["ten_facts"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    pool = [("frontier", f, i) for i, f in enumerate(fs)] + [("view", g, None) for g in gs]
    if not pool:
        return {}
    goals = [c.goal for _, c, _ in pool]
    room = room_features(m, pose, goals, trail=trail, types=[None] * len(goals), prior={})
    ms["room_facts"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    extra = B.lean_features(None, m, pose, pool, feats, trail, 0.0)
    ms["lidar_gain"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    X = np.asarray([a + b + c for a, b, c in zip(feats, room, extra)], dtype=np.float32)
    int(np.argmax(POOL.score_np(P, "F", X)))
    ms["network"] = (time.perf_counter() - t) * 1000
    return ms


def stats(xs) -> dict:
    return {"p50_ms": round(statistics.median(xs), 2), "p90_ms": round(T396.p90(xs), 2),
            "max_ms": round(max(xs), 2)}


def main(label: str, out_path: str) -> None:
    A = arms()
    if os.environ.get("ARMS"):              # a subset, to fit a loaded window
        A = {n: A[n] for n in os.environ["ARMS"].split(",")}
    P32 = _load(os.path.join(HERE, "cloud", "ckpt", "F-pool-W32", "best.json"))
    names = list(A)
    whole = {n: [] for n in names}
    split, counts, i = {}, [], 0
    t0 = time.time()
    rng = random.Random(fc.SEED)
    for path in fc.MAPS:
        m = fc.load(path)
        segment(T396.coarse(m))                 # warm once, as a running brain would be
        for x, y in fc.poses(m, T396.POSES, rng):
            trail, heading = T396.trail_near(m, x, y), rng.uniform(0, 360)
            for n in names[i % len(names):] + names[:i % len(names)]:
                t = time.perf_counter()
                k = A[n](m, x, y, heading, trail)
                if k:
                    whole[n].append((time.perf_counter() - t) * 1000)
                if n == names[0]:
                    counts.append(k)        # every arm builds the same pool
            for part, v in parts(m, x, y, heading, trail, P32).items():
                split.setdefault(part, []).append(v)
            i += 1
        print(f"{time.strftime('%T')} {os.path.basename(path)} done", flush=True)
    out = {"label": label, "host": platform.node(), "machine": platform.machine(),
           "start_unix": round(t0), "end_unix": round(time.time()),
           "loadavg": os.getloadavg(), "poses": i,
           "candidates_median": statistics.median(c for c in counts if c),
           "whole_choice": {n: {"choices": len(v), **stats(v)} for n, v in whole.items()},
           "F-pool-W32_parts": {p: stats(v) for p, v in split.items()}}
    with open(out_path, "w") as f:
        f.write(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:3])
