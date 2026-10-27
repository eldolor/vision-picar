"""
evaluations/goal-learn-3103/run.py -- 3.103.B: F-lean in four shapes, by
3.96's own stages (`goal-learn-396/train.py`) with the shape and the one
extra fact patched in.

    python evaluations/goal-learn-3103/run.py clone SHAPE ROOT    # oracle clone only -> ROOT/F-lean-SHAPE/clone.pt
    python evaluations/goal-learn-3103/run.py pick ROOT            # the two best by held-out agreement -> ROOT/picked.json
    python evaluations/goal-learn-3103/run.py ppo SHAPE ROOT       # PPO from the clone, 3.101's budget -> best.json
    python evaluations/goal-learn-3103/run.py choose ROOT          # the better PPO arm by mode-R selection -> ROOT/chosen.json
    python evaluations/goal-learn-3103/run.py test ROOT            # the chosen arm on the 588 test houses, R and U
    python evaluations/goal-learn-3103/run.py score ROOT           # criteria 1-4 against 3.101's F-large rows
    python evaluations/goal-learn-3103/run.py time ROOT            # criterion 5: one choice, F-large vs the chosen arm

ROOT holds `oracle-large` and `oracle-select` (3.101's shards, all 42
columns); the clone reads only LEAN_COLS. Every stage resumes as 3.96's do.
"""
import json
import multiprocessing
import os
import platform
import random
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT_REPO)
sys.path.insert(0, os.path.join(ROOT_REPO, "evaluations", "goal-learn-396"))
sys.path.insert(0, HERE)
os.environ.setdefault("WORKERS", str(os.cpu_count() or 4))
import c as C396                                              # noqa: E402
import train as T                                             # noqa: E402
from lean import LEAN_COLS, SHAPES, lean_features, make_torch_net, score_np, size  # noqa: E402
from brain import frontier as fr                              # noqa: E402
from brain.goal_policy import features, goal_context          # noqa: E402
from features import room_features                            # noqa: E402
from tests import frontier_compare as fc                      # noqa: E402

F_ROWS = os.path.join(ROOT_REPO, "evaluations", "goal-learn-3101", "run", "test")
SEED = 3103
_load_choices = T.load_choices


def use_shape(shape: str) -> None:
    """Patch 3.96's trainer before the pool forks (start method pinned to fork)."""
    T.EXTRA_FEATURES = lean_features
    T.torch_net = make_torch_net(shape)
    T.score_np = score_np
    T.load_choices = lambda shards, kind: [(x[:, LEAN_COLS], c, y, p)
                                           for x, c, y, p in _load_choices(shards, kind)]


def arm(shape: str) -> str:
    return f"F-lean-{shape}"


def cmd_clone(shape: str, root: str) -> None:
    use_shape(shape)
    out = os.path.join(root, arm(shape))
    os.makedirs(out, exist_ok=True)
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    with ProcessPoolExecutor(T.WORKERS) as ex:
        shards = T.stage_oracle("large", root, ex)        # all exist: nothing is run
        st = T.stage_clone(arm(shape), out, shards, ex)
    print(f"{arm(shape)}: {size(shape)} numbers, held-out agreement {st['held_out_agreement']}", flush=True)


def cmd_pick(root: str) -> None:
    import torch
    agree = {s: torch.load(os.path.join(root, arm(s), "clone.pt"), weights_only=False)["held_out_agreement"]
             for s in SHAPES}
    top = sorted(SHAPES, key=lambda s: -agree[s])[:2]
    T.save_atomic(os.path.join(root, "picked.json"),
                  lambda p: json.dump({"agreement": agree, "picked": top}, open(p, "w"), indent=1))
    print("clone agreement", agree, "-> PPO for", top, flush=True)


def cmd_ppo(shape: str, root: str) -> None:
    use_shape(shape)
    out = os.path.join(root, arm(shape))
    if os.path.exists(os.path.join(out, "best.json")):
        print(f"{arm(shape)}: already done")
        return
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    clone = torch.load(os.path.join(out, "clone.pt"), weights_only=False)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo(arm(shape), out, clone, ex)


def cmd_choose(root: str) -> None:
    picked = json.load(open(os.path.join(root, "picked.json")))["picked"]
    sel = {s: json.load(open(os.path.join(root, arm(s), "best.json")))["best_selection"] for s in picked}
    chosen = max(picked, key=lambda s: sel[s]["R"]["mean_reward"])
    T.save_atomic(os.path.join(root, "chosen.json"),
                  lambda p: json.dump({"selection": sel, "chosen": chosen}, open(p, "w"), indent=1))
    print("selection", {s: (v["R"]["mean_reward"], v["U"]["mean_reward"]) for s, v in sel.items()},
          "-> test", chosen, flush=True)


def _job(args):
    shape, s, h, mode, params = args
    use_shape(shape)
    try:
        r = T.mission(s, h, mode, ("greedy", "F", params), 0, 0.0)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def cmd_test(root: str) -> None:
    shape = json.load(open(os.path.join(root, "chosen.json")))["chosen"]
    params = json.load(open(os.path.join(root, arm(shape), "best.json")))["params"]
    hs = C396.test_houses()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in ("R", "U"):
            path = os.path.join(root, "test", f"F-lean-{mode}.json")
            if os.path.exists(path):
                continue
            os.makedirs(os.path.dirname(path), exist_ok=True)
            rows = list(ex.map(_job, [(shape, s, h, mode, params) for s, h in hs], chunksize=2))
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
            print(f"F-lean ({shape}) {mode}: {T.summary(rows)}", flush=True)


def _rows(path):
    return {r["index"]: r for r in json.load(open(path))}


def _boot(a, b, rng, n=10000):
    ks = sorted(k for k in a if k in b and a[k]["reward"] is not None and b[k]["reward"] is not None)
    d = np.array([a[k]["reward"] - b[k]["reward"] for k in ks])
    bs = np.array([d[rng.integers(0, len(d), len(d))].mean() for _ in range(n)])
    lo, hi = np.percentile(bs, [2.5, 97.5])
    return {"diff": round(float(d.mean()), 4), "ci95": [round(float(lo), 4), round(float(hi), 4)]}


def cmd_score(root: str) -> dict:
    rng = np.random.default_rng(SEED)
    want = sorted(h for _, h in C396.test_houses())
    rep = {"chosen": json.load(open(os.path.join(root, "chosen.json"))),
           "picked": json.load(open(os.path.join(root, "picked.json")))}
    for m in ("R", "U"):
        n = _rows(os.path.join(root, "test", f"F-lean-{m}.json"))
        L = _rows(os.path.join(F_ROWS, f"F-large-{m}.json"))
        R = _rows(os.path.join(F_ROWS, f"F-rich-{m}.json"))
        for name, rows in (("F-lean", n), ("F-large", L), ("F-rich", R)):
            if sorted(rows) != want:
                raise SystemExit(f"{name}-{m}: not the {len(want)} test houses -- not scoring")
        sn, sl = T.summary(list(n.values())), T.summary(list(L.values()))
        both = [k for k in n if n[k]["outcome"] == "found" and L[k]["outcome"] == "found"]
        rep[m] = {"F-lean": sn, "F-large": sl, "F-rich": T.summary(list(R.values())),
                  "vs_F_large": _boot(n, L, rng), "vs_F_rich": _boot(n, R, rng),
                  "fewer_goals_than_F_large": sum(n[k]["goals"] < L[k]["goals"] for k in both),
                  "more_goals_than_F_large": sum(n[k]["goals"] > L[k]["goals"] for k in both),
                  "finds_ok": sn["found"] >= 0.99 * sl["found"],
                  "trips_ok": sn["median_goals_found"] <= sl["median_goals_found"],
                  "unclean": sum(1 for r in n.values() if r["outcome"] != "no_target"
                                 and (r["outcome"] == "exception" or r.get("capped")))}
    rep["criteria"] = {"1_better_R": rep["R"]["vs_F_large"]["ci95"][0] > 0,
                       "2_finds": rep["R"]["finds_ok"] and rep["U"]["finds_ok"],
                       "3_trips": rep["R"]["trips_ok"] and rep["U"]["trips_ok"],
                       "4_clean": rep["R"]["unclean"] + rep["U"]["unclean"] == 0}
    T.save_atomic(os.path.join(root, "test", "score.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps({k: rep[k] for k in ("criteria", "R", "U")}, indent=1))
    return rep


def lean_choice(m, x, y, heading, trail, P, score=score_np) -> int:
    """One whole choice on F-lean's 18 facts (3.96's 17 plus lidar_gain),
    scored by `score(P, "F", X)`; returns the pool's size (0: no choice)."""
    pose = {"x_m": x, "y_m": y, "heading_deg": heading}
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    ctx = goal_context(m)                # 3.109: once per choice, as explore does
    feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m, size_m=f.size_m,
                      ctx=ctx) for f in fs]
    feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m, area_m2=g.area_m2,
                       ctx=ctx) for g in gs]
    pool = [("frontier", f, i) for i, f in enumerate(fs)] + [("view", g, None) for g in gs]
    if not pool:
        return 0
    goals = [c.goal for _, c, _ in pool]
    room = room_features(m, pose, goals, trail=trail, types=[None] * len(goals), prior={})
    extra = lean_features(None, m, pose, pool, feats, trail, 0.0)
    X = np.asarray([a + b + c for a, b, c in zip(feats, room, extra)], dtype=np.float32)
    int(np.argmax(score(P, "F", X)))
    return len(pool)


def bench(arms: dict) -> dict:
    """3.96's benchmark: every arm's whole choice at the same poses.
    arms: name -> f(m, x, y, heading, trail) returning the pool's size."""
    sys.path.insert(0, os.path.join(ROOT_REPO, "evaluations", "goal-learn-396"))
    import time_choice as T396
    from brain.room_labels import segment
    from features import coarse
    out = {"host": platform.node(), "machine": platform.machine()}
    for name, choose in arms.items():
        rng, whole = random.Random(fc.SEED), []
        for path in fc.MAPS:
            m = fc.load(path)
            segment(coarse(m))
            for x, y in fc.poses(m, T396.POSES, rng):
                trail, heading = T396.trail_near(m, x, y), rng.uniform(0, 360)
                t = time.perf_counter()
                n = choose(m, x, y, heading, trail)
                if n:
                    whole.append((time.perf_counter() - t) * 1000)
        out[name] = {"poses": len(whole), "p50_ms": round(statistics.median(whole), 1),
                     "p90_ms": round(T396.p90(whole), 1), "max_ms": round(max(whole), 1)}
        print(name, out[name], flush=True)
    return out


def cmd_time(root: str) -> None:
    """Criterion 5 on 3.96's benchmark: F-large's whole choice, and the
    chosen arm's (the same choice plus lidar_gain and its network)."""
    sys.path.insert(0, os.path.join(ROOT_REPO, "evaluations", "goal-learn-396"))
    import time_choice as T396
    from model import to_np
    shape = json.load(open(os.path.join(root, "chosen.json")))["chosen"]
    P_lean = {k: np.asarray(v, dtype=np.float32)
              for k, v in json.load(open(os.path.join(root, arm(shape), "best.json")))["params"].items()}
    P_large = to_np(json.load(open(T396.BEST))["params"])
    out = bench({"F-large": lambda m, x, y, h, t: T396.choice(m, x, y, h, P_large, t)[0],
                 "F-lean": lambda m, x, y, h, t: lean_choice(m, x, y, h, t, P_lean)})
    out["shape"] = shape
    out["5_cost"] = {"bar_ms": round(1.1 * out["F-large"]["p90_ms"], 1),
                     "met": out["F-lean"]["p90_ms"] <= 1.1 * out["F-large"]["p90_ms"]}
    print(out["5_cost"])
    T.save_atomic(os.path.join(root, "time-container.json"), lambda p: json.dump(out, open(p, "w"), indent=1))


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd, *rest = sys.argv[1:]
    {"clone": cmd_clone, "pick": cmd_pick, "ppo": cmd_ppo, "choose": cmd_choose,
     "test": cmd_test, "score": cmd_score, "time": cmd_time}[cmd](*rest)
