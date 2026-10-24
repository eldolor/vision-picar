"""
evaluations/goal-learn-396/train.py -- 3.96 B2-B4 (amendment 2): clone the
oracle, then PPO, for one arm; every stage resumable after a spot
interruption.

    python evaluations/goal-learn-396/train.py ARM OUT_DIR      # ARM: F-small | F-large | MAP-large | PONI-large
    python evaluations/goal-learn-396/train.py rule OUT_DIR     # the rule on the selection houses
    python evaluations/goal-learn-396/train.py oracle ROOT      # the shared oracle sets, first

Stages, each skipped when its output exists in OUT_DIR:

1. **oracle**: oracle-driven missions on the arm's training houses, one in
   mode R and one in mode U each, shared by the arms on the same house set
   (`OUT_DIR/../oracle-<small|large>/oracle-<k>.pkl`, a shard every SHARD
   houses, so an interruption loses at most one shard). `train.py oracle
   ROOT` makes them all first, so the arms never race to write them.
2. **clone**: supervised softmax cross-entropy on the oracle's picks, or
   PONI's potentials (`clone.pt`; `clone-ckpt.pt` after every epoch).
3. **ppo**: ITERS iterations of HOUSES houses x ROLLOUTS missions, each
   mission's mode drawn R or U half and half; group-relative advantages and
   PPO's clipped objective (3.65's, unchanged). A checkpoint (`ckpt.pt`:
   net, optimiser, both RNGs, the log, the best so far) is written
   atomically after every iteration, so a restart resumes at the next one.
   Every SELECT_EVERY iterations the greedy policy runs on the 100
   selection houses in modes R and U; the best by mode-R mean reward is
   kept (`best.json`).

Budgets are the same for every arm, fixed here before the first run.
`CKPT_S3=s3://...` copies each checkpoint there too.
"""
import json
import math
import os
import pickle
import random
import subprocess
import sys
import time
import types
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-tap-388"))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "explore-365"))
sys.path.insert(0, HERE)
import oracle as O                                            # noqa: E402
from a import houses as a_houses, pick_target                 # noqa: E402
from features import Labeller, room_features, room_type       # noqa: E402
from harness import CRASH_COST, GOAL_COST, METRE_COST, MeteredNav  # noqa: E402
from model import crops, layers, score_np, to_np, torch_net   # noqa: E402
from brain.frontier import _cell_of                            # noqa: E402
from brain.goal_policy import FEATURES, LinearGoalPolicy, NetGoalPolicy  # noqa: E402
from control.mission_runner import MissionRunner              # noqa: E402
from sim.maps import build_world                              # noqa: E402
from sim.maps.procthor import usable                          # noqa: E402
from sim.mock_robot import MockRobot                          # noqa: E402
from sim.mock_world import MockWorld                          # noqa: E402
from tests.procthor_sweep import MAX_STEPS, TICK_CAP          # noqa: E402
from tests.test_explore import CELL                           # noqa: E402

SEED = 3961
# The budget, the same for every arm.
HOUSES, ROLLOUTS, ITERS, EPOCHS = 48, 8, 60, 4
LR, CLIP, ENTROPY = 1e-3, 0.2, 0.01
SELECT_EVERY = 10
CLONE_EPOCHS, CLONE_BATCH, CLONE_LR = 30, 256, 1e-3
SHARD = 250
WORKERS = int(os.environ.get("WORKERS", max(1, (os.cpu_count() or 4) - 2)))
# SMOKE=1: a tiny run end to end, for checking the plumbing only.
SMOKE = os.environ.get("SMOKE") == "1"
if SMOKE:
    HOUSES, ROLLOUTS, ITERS, EPOCHS, SELECT_EVERY, CLONE_EPOCHS, SHARD = 2, 2, 3, 1, 2, 1, 3
PRIOR = json.load(open(os.path.join(HERE, "prior.json")))


def train_houses(arm: str) -> list:
    if SMOKE:
        return a_houses("train")[:4] + ([] if arm == "F-small" else [("train", usable("train")[-1])])
    small = a_houses("train")
    if arm == "F-small":
        return small
    extra = [("train", h) for h in usable("train") if h >= 2000]
    if len(extra) < 1000:
        # B1's 10,000-house import has not landed in this checkout.
        raise SystemExit(f"only {len(extra)} usable train houses from 2000 on; import B1 first")
    return small + extra


def select_houses() -> list:
    return a_houses("select")[:3] if SMOKE else a_houses("select")


# ---------- one mission under any chooser ----------

def dropped_rooms(grid, index: int, share: float) -> frozenset:
    """Criterion 3: `share` of the house's rooms (rounded) whose labels are
    withheld, chosen by a draw seeded by the house index."""
    if share <= 0:
        return frozenset()
    names = sorted(grid.rooms)
    return frozenset(random.Random(SEED * 7919 + index).sample(names, round(share * len(names))))


def mission(split: str, index: int, mode: str, spec, seed: int = 0, drop: float = 0.0) -> dict:
    """spec: "rule" | "net365" (3.65's shipped network, its own ten
    features) | "oracle" | ("sample"|"greedy", kind, params). `drop`: the
    share of rooms whose labels are withheld (criterion 3)."""
    grid = build_world(f"procthor:{split}:{index}")
    target, dist = pick_target(grid, index, mode)
    row = {"split": split, "index": index, "mode": mode, "target": target}
    if target is None:
        row.update(outcome="no_target", reward=None, choices=[])
        return row
    robot = MockRobot(grid, render=False)
    nav = MeteredNav(robot)
    runner = MissionRunner(robot, target_object=target, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    trail, records = [], []
    if spec == "net365":
        runner.agent.goal_policy = NetGoalPolicy.load()
    elif spec != "rule":
        agent = runner.agent
        agent.goal_policy = LinearGoalPolicy([0.0] * len(FEATURES))
        labeller = Labeller(grid, CELL, drop=dropped_rooms(grid, index, drop))
        prior = PRIOR.get(target, {})
        kind = None if spec == "oracle" else spec[1]
        params = None if spec == "oracle" else to_np(spec[2])
        rng = random.Random(seed)
        want_crops = kind == "MAP" or spec == "oracle"

        def _learned_goal(self, m, pose, frontiers, leave, known, now, skipped, rch=None):
            pool, feats = self._pool(m, pose, frontiers, leave, known, now, skipped, rch)
            if not pool:
                return None
            goals = [c.goal for _, c, _ in pool]
            known = labeller.known_rooms(m)
            types_ = labeller.types(m, goals, known)
            room = room_features(m, pose, goals, trail=trail, types=types_, prior=prior)
            X = np.asarray([a + b for a, b in zip(feats, room)], dtype=np.float32)
            C = None
            if want_crops:
                pc = {}
                for name in known:
                    p = prior.get(room_type(name), 0.0)
                    for (i, j) in labeller.cells[name]:
                        pc[_cell_of(m, (i + 0.5) * CELL, (j + 0.5) * CELL)] = p
                L = layers(m, self.seen, trail, pc)
                C = crops(m, L, pose, goals)
            costs = [O.cost_at(dist, g[0], g[1]) for g in goals]
            if spec == "oracle":
                i, lp = O.oracle_pick(costs), 0.0
            else:
                s = score_np(params, kind, X, C)
                s = np.where(np.isfinite(s), s, -1e9)
                if spec[0] == "greedy":
                    i, lp = int(np.argmax(s)), 0.0
                else:
                    p = np.exp(s - s.max())
                    p /= p.sum()
                    i = int(min(np.searchsorted(np.cumsum(p), rng.random()), len(p) - 1))
                    lp = float(math.log(max(p[i], 1e-12)))
            if len(pool) > 1 and (spec == "oracle" or spec[0] == "sample"):
                records.append({"X": X, "C": C if want_crops else None, "i": i, "lp": lp,
                                "oracle": O.oracle_pick(costs),
                                "costs": np.asarray([-1.0 if c == O.INF else c for c in costs],
                                                    dtype=np.float32)})
            k_, c, rank = pool[i]
            return self._send_pooled(k_, c, rank, now, len(pool), skipped)

        agent._learned_goal = types.MethodType(_learned_goal, agent)
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
    if row["outcome"] == "exception":
        row["reward"] = -CRASH_COST
    else:
        row["reward"] = round((row["outcome"] == "found") - GOAL_COST * row["goals"]
                              - METRE_COST * row["metres"], 4)
    row["choices"] = records
    return row


def _job(args):
    return mission(*args)


# ---------- persistence ----------

def save_atomic(path: str, write) -> None:
    # Per-process tmp: arms rebuilding one shared file at once never mix bytes.
    tmp = f"{path}.{os.getpid()}.tmp"
    write(tmp)
    os.replace(tmp, path)
    s3 = os.environ.get("CKPT_S3")
    if s3:
        dest = (f"{s3.rstrip('/')}/{os.path.basename(os.path.dirname(path))}/"
                f"{os.path.basename(path)}")
        try:
            ok = subprocess.run(["aws", "s3", "cp", "--only-show-errors", path, dest]).returncode == 0
        except OSError as e:
            ok = False
            print(f"WARNING: {e}", flush=True)
        if not ok:
            # The local copy is good; only a spot interruption now would lose it.
            print(f"WARNING: upload of {path} to {dest} FAILED", flush=True)


def summary(rows: list) -> dict:
    rows = [r for r in rows if r["outcome"] != "no_target"]
    found = [r for r in rows if r["outcome"] == "found"]
    med = lambda xs: float(np.median(xs)) if xs else None  # noqa: E731
    return {"n": len(rows), "found": len(found),
            "exceptions": sum(r["outcome"] == "exception" for r in rows),
            "capped": sum(bool(r.get("capped")) for r in rows),
            "median_goals_found": med([r["goals"] for r in found]),
            "median_metres_found": med([r["metres"] for r in found]),
            "mean_reward": round(float(np.mean([r["reward"] for r in rows])), 4) if rows else None}


# ---------- stages ----------

def house_set(arm: str) -> str:
    return "small" if arm == "F-small" else "large"


def stage_oracle(which: str, root: str, ex) -> list:
    """Oracle shards for a house set, in `root/oracle-<set>/`, shared by every
    arm on that set (remote.sh runs this once, before the arms)."""
    hs = train_houses("F-small" if which == "small" else "F-large")
    out = os.path.join(root, f"oracle-{which}")
    os.makedirs(out, exist_ok=True)
    shards = []
    for k in range(0, len(hs), SHARD):
        path = os.path.join(out, f"oracle-{k // SHARD:03d}.pkl")
        shards.append(path)
        if os.path.exists(path):
            continue
        jobs = [(s, h, mode, "oracle") for s, h in hs[k:k + SHARD] for mode in ("R", "U")]
        rows = list(ex.map(_job, jobs, chunksize=4))
        save_atomic(path, lambda p: pickle.dump(rows, open(p, "wb")))
        print(f"oracle {which} shard {k // SHARD}: {summary(rows)}", flush=True)
    return shards


# Amendment 3: PONI's potential, 1 at the pool's best true path, falling to 0
# 10 m (in true-grid cells) worse; unreachable is 0.
PONI_SPAN_CELLS = 10.0 / CELL


def potential(costs: np.ndarray) -> np.ndarray:
    ok = costs >= 0
    if not ok.any():
        return np.zeros_like(costs)
    best = costs[ok].min()
    return np.where(ok, 1.0 - np.minimum(1.0, (costs - best) / PONI_SPAN_CELLS), 0.0).astype(np.float32)


def load_choices(shards: list, kind: str) -> list:
    out = []
    for p in shards:
        for r in pickle.load(open(p, "rb")):
            for c in r["choices"]:
                if not (c["costs"] >= 0).any():
                    # The target unreachable from every candidate: the oracle's
                    # "pick" is just the pool's first, which would teach the rule.
                    continue
                out.append((c["X"], c["C"] if kind == "MAP" else None, c["oracle"],
                            potential(c["costs"])))
    return out


def _batch_loss(net, batch, kind, torch, poni=False):
    """Mean softmax cross-entropy of the oracle's pick over a batch of choices;
    `poni` (amendment 3): binary cross-entropy of every candidate's score to
    its potential, averaged per choice."""
    X = torch.tensor(np.concatenate([b[0] for b in batch]))
    C = torch.tensor(np.concatenate([b[1] for b in batch])) if kind == "MAP" else None
    s = net(X, C)
    if poni:
        y = torch.tensor(np.concatenate([b[3] for b in batch]))
        w = torch.tensor(np.concatenate([np.full(len(b[0]), 1.0 / len(b[0]), dtype=np.float32)
                                         for b in batch]))
        bce = torch.nn.functional.binary_cross_entropy_with_logits(s, y, reduction="none")
        return (bce * w).sum() / len(batch)
    loss, k = 0.0, 0
    for x, _, y, _ in batch:
        n = len(x)
        loss = loss - torch.log_softmax(s[k:k + n], dim=0)[y]
        k += n
    return loss / len(batch)


def stage_clone(arm: str, out: str, shards: list, ex):
    import torch
    kind = "F" if arm.startswith("F") else "MAP"
    poni = arm.startswith("PONI")
    path = os.path.join(out, "clone.pt")
    if os.path.exists(path):
        return torch.load(path, weights_only=False)
    torch.manual_seed(SEED)
    data = load_choices(shards, kind)
    held = load_choices(_select_oracle(os.path.dirname(out), ex), kind)
    if not data or not held:
        raise SystemExit(f"{arm}: no oracle choices ({len(data)} training, {len(held)} held out)")
    net = torch_net(kind)
    opt = torch.optim.Adam(net.parameters(), lr=CLONE_LR)
    first = 0
    cpath = os.path.join(out, "clone-ckpt.pt")
    if os.path.exists(cpath):
        st = torch.load(cpath, weights_only=False)
        net.load_state_dict(st["net"])
        opt.load_state_dict(st["opt"])
        torch.set_rng_state(st["torch_rng"])
        first = st["epoch"] + 1
        print(f"clone: resuming at epoch {first}", flush=True)
    for ep in range(first, CLONE_EPOCHS):
        # Each epoch's order from its own seed, so a resumed clone runs the
        # same batches as an uninterrupted one.
        order = list(range(len(data)))
        random.Random(SEED * 1000 + ep).shuffle(order)
        for k in range(0, len(order), CLONE_BATCH):
            loss = _batch_loss(net, [data[q] for q in order[k:k + CLONE_BATCH]], kind, torch, poni)
            opt.zero_grad()
            loss.backward()
            opt.step()
        if ep % 5 == 4 or ep == CLONE_EPOCHS - 1:
            with torch.no_grad():
                hl = float(sum(_batch_loss(net, held[k:k + 512], kind, torch, poni) * len(held[k:k + 512])
                               for k in range(0, len(held), 512)) / len(held))
            print(f"clone epoch {ep}: held-out loss {hl:.4f}", flush=True)
        cst = {"net": net.state_dict(), "opt": opt.state_dict(),
               "torch_rng": torch.get_rng_state(), "epoch": ep}
        save_atomic(cpath, lambda p: torch.save(cst, p))
    agree = _agreement(net, held, kind, torch)
    state = {"kind": kind, "net": net.state_dict(), "held_out_agreement": agree,
             "n_train": len(data), "n_held": len(held)}
    save_atomic(path, lambda p: torch.save(state, p))
    print(f"clone: {len(data)} choices, held-out agreement {agree}", flush=True)
    return state


def _agreement(net, held, kind, torch) -> float:
    hit = 0
    with torch.no_grad():
        for x, c, y, _ in held:
            s = net(torch.tensor(x), torch.tensor(c) if kind == "MAP" else None)
            hit += int(torch.argmax(s)) == y
    return round(hit / len(held), 4)


def _select_oracle(root: str, ex) -> list:
    """Oracle missions on the selection houses, both modes (held-out choices
    for the clone's agreement, reported only), in `root/oracle-select/`."""
    out = os.path.join(root, "oracle-select")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "oracle-select.pkl")
    if not os.path.exists(path):
        rows = list(ex.map(_job, [(s, h, m, "oracle") for s, h in select_houses()
                                  for m in ("R", "U")], chunksize=2))
        save_atomic(path, lambda p: pickle.dump(rows, open(p, "wb")))
    return [path]


def run_selection(kind: str, params: dict, ex) -> dict:
    jobs = [(s, h, m, ("greedy", kind, params)) for s, h in select_houses() for m in ("R", "U")]
    rows = list(ex.map(_job, jobs, chunksize=2))
    return {m: summary([r for r in rows if r["mode"] == m]) for m in ("R", "U")}


def ppo_update(net, opt, missions, kind, torch):
    seg, X, C, chosen, old, adv = [], [], [], [], [], []
    at = 0
    for m in missions:
        for c in m["choices"]:
            seg.append((at, len(c["X"])))
            at += len(c["X"])
            X.append(c["X"])
            if kind == "MAP":
                C.append(c["C"])
            chosen.append(c["i"])
            old.append(c["lp"])
            adv.append(m["adv"])
    if not seg:
        return {}
    Xt = torch.tensor(np.concatenate(X))
    Ct = torch.tensor(np.concatenate(C)) if kind == "MAP" else None
    old_t, adv_t = torch.tensor(old, dtype=torch.float32), torch.tensor(adv, dtype=torch.float32)
    CH = 8192        # candidates per chunk; gradients summed, so one step per epoch
    for _ in range(EPOCHS):
        opt.zero_grad()
        tot_loss, ents, ratios = 0.0, [], []
        k = 0
        while k < len(seg):
            j, n = k, 0
            while j < len(seg) and (n == 0 or n + seg[j][1] <= CH):
                n += seg[j][1]
                j += 1
            a0 = seg[k][0]
            s = net(Xt[a0:a0 + n], Ct[a0:a0 + n] if Ct is not None else None)
            new, ent = [], []
            for q in range(k, j):
                st, ln = seg[q][0] - a0, seg[q][1]
                lp = torch.log_softmax(s[st:st + ln], dim=0)
                new.append(lp[chosen[q]])
                ent.append(-(lp.exp() * lp).sum())
            new, ent = torch.stack(new), torch.stack(ent)
            ratio = torch.exp(new - old_t[k:j])
            a = adv_t[k:j]
            loss = (-torch.min(ratio * a, ratio.clamp(1 - CLIP, 1 + CLIP) * a).sum()
                    - ENTROPY * ent.sum()) / len(seg)
            loss.backward()
            tot_loss += float(loss.detach())
            ents.append(ent.detach())
            ratios.append(ratio.detach())
            k = j
        opt.step()
    ent, ratio = torch.cat(ents), torch.cat(ratios)
    return {"choices": len(seg), "loss": round(tot_loss, 4), "entropy": round(float(ent.mean()), 3),
            "clipped": round(float(((ratio - 1).abs() > CLIP).float().mean()), 3)}


def _reward(sel: dict) -> float:
    """Mode R's mean selection reward; none counts as the worst."""
    x = sel["R"]["mean_reward"]
    return -math.inf if x is None else x


def stage_ppo(arm: str, out: str, clone: dict, ex) -> None:
    import torch
    kind = clone["kind"]
    ckpt = os.path.join(out, "ckpt.pt")
    net = torch_net(kind)
    opt = torch.optim.Adam(net.parameters(), lr=LR)
    if os.path.exists(ckpt):
        st = torch.load(ckpt, weights_only=False)
        net.load_state_dict(st["net"])
        opt.load_state_dict(st["opt"])
        rng = random.Random()
        rng.setstate(st["rng"])
        torch.set_rng_state(st["torch_rng"])
        log, best, start = st["log"], st["best"], st["it"] + 1
        print(f"resuming at iteration {start}", flush=True)
    else:
        net.load_state_dict(clone["net"])
        torch.manual_seed(SEED)
        rng = random.Random(SEED)
        log = {"arm": arm, "budget": {"houses": HOUSES, "rollouts": ROLLOUTS, "iters": ITERS,
                                      "epochs": EPOCHS, "lr": LR, "clip": CLIP,
                                      "entropy": ENTROPY},
               "clone_agreement": clone["held_out_agreement"], "iters": [], "selection": []}
        sel = run_selection(kind, net.export(), ex)
        best = {"it": -1, "selection": sel, "params": net.export()}
        log["selection"].append({"it": -1, **sel})
        print(f"selection (clone): {sel}", flush=True)
        start = 0
    train = train_houses(arm)
    t0 = time.time()
    for it in range(start, ITERS):
        hs = rng.sample(train, HOUSES)
        params = net.export()
        jobs = []
        for s, h in hs:
            for _ in range(ROLLOUTS):
                jobs.append((s, h, rng.choice(("R", "U")), ("sample", kind, params),
                             rng.randrange(2 ** 31)))
        missions = [m for m in ex.map(_job, jobs, chunksize=2) if m["reward"] is not None]
        for s, h in hs:
            for mode in ("R", "U"):
                g = [m for m in missions if (m["split"], m["index"], m["mode"]) == (s, h, mode)]
                if not g:
                    continue
                rs = [m["reward"] for m in g]
                mu, sd = float(np.mean(rs)), float(np.std(rs))
                for m in g:
                    m["adv"] = (m["reward"] - mu) / (sd + 1e-6) if sd > 1e-6 else 0.0
        stats = ppo_update(net, opt, missions, kind, torch)
        entry = {"it": it, "mean_reward": round(float(np.mean([m["reward"] for m in missions])), 4),
                 "found": sum(m["outcome"] == "found" for m in missions), "n": len(missions),
                 "s": round(time.time() - t0), **stats}
        log["iters"].append(entry)
        print(f"it {it}: {entry}", flush=True)
        if (it + 1) % SELECT_EVERY == 0:
            sel = run_selection(kind, net.export(), ex)
            log["selection"].append({"it": it, **sel})
            print(f"selection it {it}: {sel}", flush=True)
            if _reward(sel) > _reward(best["selection"]):
                best = {"it": it, "selection": sel, "params": net.export()}
        st = {"net": net.state_dict(), "opt": opt.state_dict(), "rng": rng.getstate(),
              "torch_rng": torch.get_rng_state(), "log": log, "best": best, "it": it}
        save_atomic(ckpt, lambda p: torch.save(st, p))
    jsonable = lambda P: {k: np.asarray(v).round(6).tolist() for k, v in P.items()}  # noqa: E731
    res = {"arm": arm, "kind": kind, "best_it": best["it"], "best_selection": best["selection"],
           "params": jsonable(best["params"]), "log": log}
    save_atomic(os.path.join(out, "best.json"), lambda p: json.dump(res, open(p, "w")))
    print(f"done: best it {best['it']} {best['selection']}", flush=True)


def main(arm: str, out: str) -> None:
    os.makedirs(out, exist_ok=True)
    import torch
    # Three arms share one machine; the workers do the simulating.
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    with ProcessPoolExecutor(WORKERS) as ex:
        if arm == "rule":
            rows = list(ex.map(_job, [(s, h, m, "rule") for s, h in select_houses()
                                      for m in ("R", "U")], chunksize=2))
            res = {m: summary([r for r in rows if r["mode"] == m]) for m in ("R", "U")}
            save_atomic(os.path.join(out, "rule-select.json"),
                        lambda p: json.dump({"summary": res, "rows": rows}, open(p, "w")))
            print("rule:", res)
            return
        if arm == "oracle":
            # `train.py oracle ROOT`: every shared oracle set, before the arms.
            stage_oracle("small", out, ex)
            stage_oracle("large", out, ex)
            _select_oracle(out, ex)
            return
        if os.path.exists(os.path.join(out, "best.json")):
            print("already done")
            return
        root = os.path.dirname(os.path.abspath(out))
        shards = stage_oracle(house_set(arm), root, ex)
        clone = stage_clone(arm, out, shards, ex)
        stage_ppo(arm, out, clone, ex)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
