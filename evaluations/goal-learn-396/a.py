"""
evaluations/goal-learn-396/a.py -- 3.96 A: can richer features tell the
oracle's pick from the others? (criterion 1, amendment 1)

    python evaluations/goal-learn-396/a.py prior            # prior.json from the training houses
    python evaluations/goal-learn-396/a.py collect U|R      # oracle-driven missions, training + selection
    python evaluations/goal-learn-396/a.py fit              # criterion 1 on R; U reported -> a.json

Houses (the plan's): training = usable `val` 300-569 + usable `train`
0-299; selection = usable `val` 100-199. The oracle drives (3.88's), and
every choice of two or more candidates records the ten old features, the
seven room features, the oracle's costs and its pick.
"""
import json
import math
import os
import random
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-tap-388"))
sys.path.insert(0, HERE)
import oracle as O                                          # noqa: E402
from b import agreement, fit                                # noqa: E402
from features import ROOM_FEATURES, Labeller, room_features, room_type  # noqa: E402
from harness import CRASH_COST, GOAL_COST, METRE_COST, MeteredNav     # noqa: E402
from brain.goal_policy import FEATURES, LinearGoalPolicy   # noqa: E402
from control.mission_runner import MissionRunner           # noqa: E402
from sim.grid_world import CELL_WALL                       # noqa: E402
from sim.maps import build_world                           # noqa: E402
from sim.maps.procthor import TARGET, usable              # noqa: E402
from sim.mock_robot import MockRobot                       # noqa: E402
from sim.mock_world import MockWorld                       # noqa: E402
from tests.procthor_sweep import MAX_STEPS, TICK_CAP       # noqa: E402
from tests.test_explore import CELL                        # noqa: E402

WORKERS = int(os.environ.get("A_WORKERS", "4"))
# Amendment 1's mix: sure, split and spread categories.
MIX = ("basketball", "plunger", "garbage bag", "cart", "laundry hamper",
       "dog bed", "box", "house plant", "garbage can")


def houses(name: str) -> list:
    if name == "train":
        return ([("val", h) for h in usable("val")[300:570]]
                + [("train", h) for h in usable("train")[:300]])
    if name == "select":
        return [("val", h) for h in usable("val")[100:200]]
    raise ValueError(name)


def distance_field(grid, target: str) -> dict:
    """oracle.distance_field for any target name."""
    from collections import deque
    seeds = [c for c, n in grid.objects.items() if n == target]
    dist = {c: 0 for c in seeds}
    q = deque(seeds)
    while q:
        c = q.popleft()
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in dist and grid._cell(*n) != CELL_WALL and n not in grid.objects:
                dist[n] = dist[c] + 1
                q.append(n)
    return dist


def pick_target(grid, index: int, mode: str):
    """(target name, distance field), or (None, None) when no category of the
    mix is reachable from the start (recorded, left out)."""
    if mode == "U":
        return TARGET, distance_field(grid, TARGET)
    start = (int(grid.x), int(grid.y))
    have = sorted({n for n in grid.objects.values() if n in MIX})
    rng = random.Random(3960 + index)
    rng.shuffle(have)
    for name in have:
        dist = distance_field(grid, name)
        if start in dist:
            return name, dist
    return None, None


# ---------- the prior: P(target in a room | the room's type), training houses ----------

def _prior_house(args):
    split, h = args
    grid = build_world(f"procthor:{split}:{h}")
    out = {}
    names = set(MIX) | {TARGET}
    for room, cells in grid.rooms.items():
        t = room_type(room)
        inside = {grid.objects.get(c) for c in cells} & names
        for name in names:
            if name in grid.objects.values():
                k = out.setdefault(name, {}).setdefault(t, [0, 0])
                k[0] += name in inside
                k[1] += 1
    return out


def cmd_prior() -> None:
    with ProcessPoolExecutor(WORKERS) as ex:
        parts = list(ex.map(_prior_house, houses("train")))
    tot = {}
    for p in parts:
        for name, by in p.items():
            for t, (a, b) in by.items():
                k = tot.setdefault(name, {}).setdefault(t, [0, 0])
                k[0] += a
                k[1] += b
    prior = {name: {t: round(a / b, 4) for t, (a, b) in sorted(by.items())}
             for name, by in sorted(tot.items())}
    json.dump(prior, open(os.path.join(HERE, "prior.json"), "w"), indent=1)
    for name, by in prior.items():
        print(f"{name:14s}", by)


# ---------- one oracle-driven mission ----------

def run(index: int, split: str, mode: str, prior_all: dict) -> dict:
    grid = build_world(f"procthor:{split}:{index}")
    target, dist = pick_target(grid, index, mode)
    row = {"index": index, "split": split, "mode": mode, "target": target}
    if target is None:
        row["outcome"] = "no_target"
        return row
    prior = prior_all.get(target, {})
    robot = MockRobot(grid, render=False)
    nav = MeteredNav(robot)
    runner = MissionRunner(robot, target_object=target, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    agent = runner.agent
    agent.goal_policy = LinearGoalPolicy([0.0] * len(FEATURES))
    labeller = Labeller(grid, CELL)
    trail = []
    records = []

    def _learned_goal(self, m, pose, frontiers, leave, known, now, skipped, rch=None):
        pool, feats = self._pool(m, pose, frontiers, leave, known, now, skipped, rch)
        if not pool:
            return None
        costs = [O.cost_at(dist, c.goal[0], c.goal[1]) for _, c, _ in pool]
        o = O.oracle_pick(costs)
        if len(pool) > 1:
            goals = [c.goal for _, c, _ in pool]
            types = labeller.types(m, goals)
            room = room_features(m, pose, goals, trail=trail, types=types, prior=prior)
            records.append({"feats": [[round(x, 5) for x in f] for f in feats],
                            "room": [[round(x, 5) for x in f] for f in room],
                            "costs": [None if c == O.INF else c for c in costs],
                            "oracle": o})
        kind, c, rank = pool[o]
        return self._send_pooled(kind, c, rank, now, len(pool), skipped,
                                 oracle_cost=None if costs[o] == O.INF else costs[o])

    import types as _t
    agent._learned_goal = _t.MethodType(_learned_goal, agent)
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
            trail.append((grid.x * CELL, grid.y * CELL))
        s = runner.status()
        row.update(outcome=s["outcome"], ticks=ticks, capped=ticks >= TICK_CAP,
                   goals=s.get("explore", {}).get("goals_sent") or 0,
                   metres=round(nav.metres + robot._path_m, 2))
    except Exception as e:  # noqa: BLE001 -- counted, never hidden
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}", goals=0, metres=0.0)
    row["choices"] = records
    return row


def _job(args):
    O.yield_to_live_runs()
    (split, h), mode, prior = args
    return run(h, split, mode, prior)


def cmd_collect(mode: str) -> None:
    prior = json.load(open(os.path.join(HERE, "prior.json")))
    for name in ("train", "select"):
        hs = houses(name)
        with ProcessPoolExecutor(WORKERS) as ex:
            rows = list(ex.map(_job, [(h, mode, prior) for h in hs]))
        out = os.path.join(HERE, f"choices-{mode}-{name}.json")
        json.dump(rows, open(out, "w"))
        n = sum(len(r.get("choices", [])) for r in rows)
        oc = {}
        for r in rows:
            oc[r["outcome"]] = oc.get(r["outcome"], 0) + 1
        print(f"{mode} {name}: {len(rows)} houses {oc}, {n} choices -> {out}", flush=True)


# ---------- criterion 1 ----------

def _choices(mode: str, name: str, with_room: bool) -> list:
    rows = json.load(open(os.path.join(HERE, f"choices-{mode}-{name}.json")))
    out = []
    for r in rows:
        for c in r.get("choices", []):
            f = [a + b for a, b in zip(c["feats"], c["room"])] if with_room else c["feats"]
            out.append({"feats": f, "costs": c["costs"], "oracle": c["oracle"]})
    return out


def fit_mlp(choices: list, hidden: int = 32, steps: int = 3000, lr: float = 0.01,
            l2: float = 1e-3, seed: int = 396):
    """score = v . tanh(W f + b) + w . f, softmax over each choice's
    candidates (Adam, full batch)."""
    rng = np.random.default_rng(seed)
    F = np.vstack([np.asarray(c["feats"], float) for c in choices])
    sizes = [len(c["feats"]) for c in choices]
    seg = np.cumsum([0] + sizes)
    owner = np.repeat(np.arange(len(choices)), sizes)
    picked = np.array([seg[k] + c["oracle"] for k, c in enumerate(choices)])
    d = F.shape[1]
    P = {"W": rng.normal(0, 1 / math.sqrt(d), (hidden, d)), "b": np.zeros(hidden),
         "v": rng.normal(0, 1 / math.sqrt(hidden), hidden), "w": np.zeros(d)}
    M = {k: np.zeros_like(x) for k, x in P.items()}
    V = {k: np.zeros_like(x) for k, x in P.items()}
    n = len(choices)
    for t in range(1, steps + 1):
        H = np.tanh(F @ P["W"].T + P["b"])
        s = H @ P["v"] + F @ P["w"]
        smax = np.maximum.reduceat(s, seg[:-1])
        e = np.exp(s - smax[owner])
        p = e / np.add.reduceat(e, seg[:-1])[owner]
        ds = p.copy()
        ds[picked] -= 1.0
        ds /= n
        G = {"v": H.T @ ds, "w": F.T @ ds}
        dH = np.outer(ds, P["v"]) * (1 - H * H)
        G["W"] = dH.T @ F
        G["b"] = dH.sum(axis=0)
        for k in P:
            g = G[k] + 2 * l2 * P[k]
            M[k] = 0.9 * M[k] + 0.1 * g
            V[k] = 0.999 * V[k] + 0.001 * g * g
            P[k] = P[k] - lr * (M[k] / (1 - 0.9 ** t)) / (np.sqrt(V[k] / (1 - 0.999 ** t)) + 1e-8)
    return P


def mlp_strict(choices: list, P) -> float:
    hit = 0
    for c in choices:
        F = np.asarray(c["feats"], float)
        s = np.tanh(F @ P["W"].T + P["b"]) @ P["v"] + F @ P["w"]
        hit += int(np.argmax(s)) == c["oracle"]
    return round(hit / len(choices), 4)


def cmd_fit() -> None:
    out = {}
    for mode in ("R", "U"):
        if not os.path.exists(os.path.join(HERE, f"choices-{mode}-select.json")):
            continue
        res = {}
        for label, room in (("old10", False), ("old10+room", True)):
            tr, se = _choices(mode, "train", room), _choices(mode, "select", room)
            w = fit(tr, init=[0.0] * len(tr[0]["feats"][0]))
            P = fit_mlp(tr)
            res[label] = {"linear_select": agreement(se, w), "linear_train": agreement(tr, w)["strict"],
                          "mlp_select": mlp_strict(se, P), "mlp_train": mlp_strict(tr, P),
                          "weights": w, "n_train": len(tr), "n_select": len(se)}
            print(mode, label, "linear", res[label]["linear_select"]["strict"],
                  "(train", res[label]["linear_train"], ") mlp", res[label]["mlp_select"],
                  "(train", res[label]["mlp_train"], ")", flush=True)
        out[mode] = res
    out["features"] = list(FEATURES) + list(ROOM_FEATURES)
    json.dump(out, open(os.path.join(HERE, "a.json"), "w"), indent=1)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "prior":
        cmd_prior()
    elif cmd == "collect":
        cmd_collect(sys.argv[2])
    elif cmd == "fit":
        cmd_fit()
    else:
        raise SystemExit(__doc__)
