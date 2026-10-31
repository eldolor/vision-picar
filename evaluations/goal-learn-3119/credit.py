"""
evaluations/goal-learn-3119/credit.py -- 3.119: F-pool-W32's PPO with
per-choice credit. 3.96's trainer gives every choice in a mission the
mission's group-relative advantage; here each choice gets its own, by GAE
over a learned critic (A1: the chooser's own facts; A2: those plus the
truth; A3: A2 plus potential-based shaping, only by the section's rule).
The chooser, its facts, the clone it starts from and 3.96's budget are
W32's (3.103.C). Desktop only; none on the 588 test houses.

    python evaluations/goal-learn-3119/credit.py check1     # group mode reproduces W32's clone selection and iteration 0
    python evaluations/goal-learn-3119/credit.py check23    # the reward split and the truth inputs, 8 missions
    python evaluations/goal-learn-3119/credit.py ppo ARM    # ARM: A1 | A2 | A3 -> run/F-pool-W32-ARM/best.json
    python evaluations/goal-learn-3119/credit.py diag ARM   # the best on 3.111's 200 diagnostic houses, R and U
    python evaluations/goal-learn-3119/credit.py all        # checks, then ppo and diag for A1 and A2, in turn
    python evaluations/goal-learn-3119/credit.py report     # criteria 1-2 per arm -> run/rows/report.json

`run/W32-clone.pt` is a hard link to 3.103.C's `cloud/ckpt/F-pool-W32/clone.pt`
(git does not keep it). Every stage resumes: a checkpoint, `best.json` or a
diagnostic row on disk is not run again.
"""
import json
import hashlib
import math
import multiprocessing
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3114"))
import tie                                                    # noqa: E402  (3.111's and 3.103's harness, POOL_WIDTH=32)
from brain.explore import ExploreAgent                        # noqa: E402  (on the path tie's harness set up)

C, B, T, PR = tie.C, tie.B, tie.T, tie.PR
O = T.O
RUN = os.environ.get("CREDIT_RUN") or os.path.join(HERE, "run")    # CREDIT_RUN: a smoke run's own folder
ROWS = os.path.join(RUN, "rows")
CLONE = os.path.join(RUN, "W32-clone.pt")
W32_BEST = os.path.join(REPO, C.BEST["F-pool-W32"])
SEED = 3119                   # the bootstrap's; training keeps 3.96's SEED (3961)
ARMS = ("A1", "A2", "A3")
MODES = C.MODES
CELL_M = C.CELL_M             # the oracle's costs and distance field are true-grid cells
GAMMA, LAM = 1.0, 0.95
CRITIC_WIDTH, CRITIC_LR, CRITIC_BATCH = 64, 1e-3, 256
SHAPE_PER_M = 0.03            # A3: Phi(s) = -0.03 x the true path distance (m)
FAR_M = 30.0                  # an unreachable distance, as an input and for Phi
CHECK_HOUSES = 8


def arm(a: str) -> str:
    return f"F-pool-W32-{a}"   # "F": 3.96 reads the kind off the name


# ---------- what each choice is made from: a snapshot in the worker ----------

_now = {}


def _capture() -> None:
    """Patch 3.96's mission (before the pool forks) so each recorded choice
    carries goals sent and metres so far, the true distance robot -> target
    and whether the robot stands in a room holding the target."""
    bw, pt, mn, inner = T.build_world, T.pick_target, T.MeteredNav, T._mission
    send = ExploreAgent._send_pooled

    def build_world(name):
        g = bw(name)
        _now.clear()
        _now.update(grid=g, snaps=[], room_of={c: r for r, cells in g.rooms.items() for c in cells})
        return g

    def pick_target(grid, index, mode):
        t = pt(grid, index, mode)
        if t[0] is not None:
            cells = [c for c, n in grid.objects.items() if n == t[0]]
            _now.update(dist=t[1], rooms={_now["room_of"][c] for c in cells if c in _now["room_of"]})
        return t

    class Nav(mn):
        def __init__(self, robot):
            super().__init__(robot)
            _now["nav"] = self

    def _send_pooled(self, kind, c, rank, now, n, skipped, **info):
        if "nav" in _now:
            _now["snaps"].append(_snap(self, n))
        return send(self, kind, c, rank, now, n, skipped, **info)

    def _mission(*args, **kwargs):
        try:
            row = inner(*args, **kwargs)
            snaps = [s for s in _now.get("snaps", []) if s["n"] > 1]
            if row.get("choices"):
                if len(snaps) != len(row["choices"]):
                    # Counted and left out of the update (never trained on), not a dead run.
                    print(f"misaligned: {row['split']}:{row['index']} {row['mode']}: "
                          f"{len(snaps)} snapshots for {len(row['choices'])} choices", flush=True)
                    row.update(misaligned=True, choices=[])
                else:
                    for c, s in zip(row["choices"], snaps):
                        c["snap"] = s
                    row["end_d_m"] = _snap(None, 0)["d_m"]
            return row
        finally:
            # 3.113: nothing of a finished house may outlive it into _collect()'s gc.freeze().
            _now.clear()

    T.build_world, T.pick_target, T.MeteredNav, T._mission = build_world, pick_target, Nav, _mission
    ExploreAgent._send_pooled = _send_pooled


def _snap(agent, n: int) -> dict:
    g, nav = _now["grid"], _now["nav"]
    d = O.cost_at(_now["dist"], g.x * T.CELL, g.y * T.CELL)
    return {"n": n, "goals": agent.goals_sent if agent else None, "metres": nav.metres + nav.robot._path_m,
            "d_m": -1.0 if d == O.INF else d * CELL_M,
            "in_room": _now["room_of"].get((math.floor(g.x), math.floor(g.y))) in _now.get("rooms", ())}


# ---------- per-choice rewards, the critic's inputs, GAE ----------

def choice_rewards(m: dict, shape: bool = False) -> list:
    """Choice t pays for the goals and metres from it to choice t+1 (the
    last, to the mission's end) and the last takes `found`; a crash puts
    -CRASH_COST on the last choice alone. Costs before the first choice
    belong to none. `shape`: plus Phi(s') - Phi(s), Phi at the mission's end
    from where the robot stopped (0 m when found beside the target), so a
    mission that ends far away earns no bonus on its last choice."""
    cs = m["choices"]
    if m["outcome"] == "exception":
        r = [0.0] * len(cs)
        r[-1] = -T.CRASH_COST
    else:
        g = [c["snap"]["goals"] for c in cs] + [m["goals"]]
        x = [c["snap"]["metres"] for c in cs] + [m["metres"]]
        r = [-T.GOAL_COST * (g[t + 1] - g[t]) - T.METRE_COST * (x[t + 1] - x[t]) for t in range(len(cs))]
        r[-1] += float(m["outcome"] == "found")
    if shape:
        phi = [-SHAPE_PER_M * _dist(c["snap"]) for c in cs] + [-SHAPE_PER_M * _dist({"d_m": m["end_d_m"]})]
        r = [r[t] + GAMMA * phi[t + 1] - phi[t] for t in range(len(cs))]
    return r


def _dist(s: dict) -> float:
    return FAR_M if s["d_m"] < 0 else min(s["d_m"], FAR_M)


def critic_inputs(m: dict, c: dict, truth: bool) -> np.ndarray:
    X, s = c["X"], c["snap"]
    v = [*X.mean(axis=0), *X.max(axis=0), len(X) / 10.0, s["goals"] / 10.0, s["metres"] / 10.0]
    if truth:
        ok = c["costs"] >= 0
        best = float(c["costs"][ok].min()) * CELL_M if ok.any() else FAR_M
        v += [float(m["mode"] == "R"), _dist(s) / 10.0, min(best, FAR_M) / 10.0, float(s["in_room"])]
    return np.asarray(v, dtype=np.float32)


def gae(rewards: list, values: list) -> tuple:
    """Advantages and lambda-returns of one mission; V = 0 after its last choice."""
    adv, run = [0.0] * len(rewards), 0.0
    for t in reversed(range(len(rewards))):
        nxt = values[t + 1] if t + 1 < len(values) else 0.0
        run = rewards[t] + GAMMA * nxt - values[t] + GAMMA * LAM * run
        adv[t] = run
    return adv, [a + v for a, v in zip(adv, values)]


def critic_net(n_in: int, torch):
    nn = torch.nn
    return nn.Sequential(nn.Linear(n_in, CRITIC_WIDTH), nn.Tanh(), nn.Linear(CRITIC_WIDTH, CRITIC_WIDTH),
                         nn.Tanh(), nn.Linear(CRITIC_WIDTH, 1))


# ---------- PPO: 3.103.C's update with an advantage per choice ----------

def ppo_update(net, opt, missions, kind, torch):
    """`pool_run.ppo_update` itself (patched in by `use_pool`), each choice
    handed over as a one-choice mission carrying its own `adv`; the order
    of choices, and so every chunk and sum, is unchanged."""
    return T.ppo_update(net, opt, [{"choices": [c], "adv": c["adv"]} for m in missions for c in m["choices"]],
                        kind, torch)


def group_advantages(hs: list, missions: list) -> None:
    """3.96's: each mission against its house and mode's rollouts, on every choice."""
    for s, h in hs:
        for mode in MODES:
            g = [m for m in missions if (m["split"], m["index"], m["mode"]) == (s, h, mode)]
            if not g:
                continue
            rs = [m["reward"] for m in g]
            mu, sd = float(np.mean(rs)), float(np.std(rs))
            for m in g:
                m["adv"] = (m["reward"] - mu) / (sd + 1e-6) if sd > 1e-6 else 0.0
                for c in m["choices"]:
                    c["adv"] = m["adv"]


def critic_advantages(missions: list, critic, torch, truth: bool, shape: bool) -> dict:
    """GAE on each mission's choices, normalised over the batch; returns the
    critic's training set and what is reported of it."""
    xs, rets, vals, every = [], [], [], []
    for m in missions:
        if not m["choices"]:
            continue
        x = np.stack([critic_inputs(m, c, truth) for c in m["choices"]])
        with torch.no_grad():
            v = critic(torch.tensor(x)).squeeze(-1).tolist()
        a, ret = gae(choice_rewards(m, shape), v)
        for c, ai in zip(m["choices"], a):
            c["adv"] = ai
        every.extend(m["choices"])
        xs.append(x)
        rets.extend(ret)
        vals.extend(v)
    A = np.array([c["adv"] for c in every])
    mu, sd = float(A.mean()), float(A.std())
    for c in every:
        c["adv"] = (c["adv"] - mu) / (sd + 1e-8)
    # The spread PPO sees: of the normalised advantages, within each mission.
    spreads = [float(np.std([c["adv"] for c in m["choices"]])) for m in missions if len(m["choices"]) > 1]
    y, v = np.array(rets), np.array(vals)
    ev = 1.0 - float(np.var(y - v)) / float(np.var(y)) if np.var(y) > 0 else None
    return {"X": np.concatenate(xs), "y": np.asarray(rets, dtype=np.float32),
            "explained_var": None if ev is None else round(ev, 4),
            "adv_spread_in_mission": round(float(np.mean(spreads)), 4) if spreads else None}


def fit_critic(critic, copt, data: dict, gen, torch) -> float:
    X, y = torch.tensor(data["X"]), torch.tensor(data["y"])
    tot, n = 0.0, 0
    for _ in range(T.EPOCHS):
        perm = torch.randperm(len(X), generator=gen)
        for k in range(0, len(X), CRITIC_BATCH):
            b = perm[k:k + CRITIC_BATCH]
            loss = ((critic(X[b]).squeeze(-1) - y[b]) ** 2).mean()
            copt.zero_grad()
            loss.backward()
            copt.step()
            tot, n = tot + float(loss.detach()) * len(b), n + len(b)
    return round(tot / max(n, 1), 5)


def stage_ppo(name: str, out: str, clone: dict, ex, credit: str, iters: int = None) -> dict:
    """3.96's `stage_ppo` (as 3.103.C ran it) with the advantages by `credit`:
    "group" (3.96's own; check 1) or an arm A1 / A2 / A3."""
    import torch
    iters = T.ITERS if iters is None else iters
    truth, shape = credit in ("A2", "A3"), credit == "A3"
    kind = clone["kind"]
    ckpt = os.path.join(out, "ckpt.pt")
    net = T.torch_net(kind)
    opt = torch.optim.Adam(net.parameters(), lr=T.LR)
    n_in = 2 * PR.POOL.N_FEAT + 3 + (4 if truth else 0)
    with torch.random.fork_rng():                 # the critic leaves torch's stream as 3.96's
        torch.manual_seed(SEED)
        critic = critic_net(n_in, torch)
    copt = torch.optim.Adam(critic.parameters(), lr=CRITIC_LR)
    gen = torch.Generator().manual_seed(SEED)
    if os.path.exists(ckpt):
        st = torch.load(ckpt, weights_only=False)
        net.load_state_dict(st["net"])
        opt.load_state_dict(st["opt"])
        critic.load_state_dict(st["critic"])
        copt.load_state_dict(st["copt"])
        gen.set_state(st["gen"])
        rng = random.Random()
        rng.setstate(st["rng"])
        torch.set_rng_state(st["torch_rng"])
        log, best, start = st["log"], st["best"], st["it"] + 1
        print(f"resuming at iteration {start}", flush=True)
    else:
        net.load_state_dict(clone["net"])
        torch.manual_seed(T.SEED)
        rng = random.Random(T.SEED)
        log = {"arm": name, "credit": credit,
               "budget": {"houses": T.HOUSES, "rollouts": T.ROLLOUTS, "iters": iters, "epochs": T.EPOCHS,
                          "lr": T.LR, "clip": T.CLIP, "entropy": T.ENTROPY},
               "critic": None if credit == "group" else
               {"inputs": n_in, "width": CRITIC_WIDTH, "lr": CRITIC_LR, "batch": CRITIC_BATCH,
                "gamma": GAMMA, "lambda": LAM, "shape_per_m": SHAPE_PER_M if shape else None},
               "clone_agreement": clone["held_out_agreement"], "iters": [], "selection": []}
        sel = T.run_selection(kind, net.export(), ex)
        best = {"it": -1, "selection": sel, "params": net.export()}
        log["selection"].append({"it": -1, **sel})
        print(f"selection (clone): {sel}", flush=True)
        start = 0
    train = T.train_houses(name)
    t0 = time.time()
    # The desktop lock (3.128-3.129): training is shared work; between
    # iterations it steps aside for a queued live run, then carries on.
    sys.path.append(os.path.join(HERE, "..", "..", "tools"))
    import desktop_lock
    desktop_lock.hold_for_life("shared", f"PPO {os.path.basename(out)} ({__file__})")
    for it in range(start, iters):
        desktop_lock.step_aside()
        hs = rng.sample(train, T.HOUSES)
        params = net.export()
        jobs = []
        for s, h in hs:
            for _ in range(T.ROLLOUTS):
                jobs.append((s, h, rng.choice(MODES), ("sample", kind, params), rng.randrange(2 ** 31)))
        missions = [m for m in ex.map(T._job, jobs, chunksize=2) if m["reward"] is not None]
        if credit == "group":
            group_advantages(hs, missions)
            cstats = {}
        else:
            data = critic_advantages(missions, critic, torch, truth, shape)
        stats = ppo_update(net, opt, missions, kind, torch)
        if credit != "group":
            cstats = {"explained_var": data["explained_var"], "adv_spread": data["adv_spread_in_mission"],
                      "value_loss": fit_critic(critic, copt, data, gen, torch)}
        entry = {"it": it, "mean_reward": round(float(np.mean([m["reward"] for m in missions])), 4),
                 "found": sum(m["outcome"] == "found" for m in missions), "n": len(missions),
                 "misaligned": sum(bool(m.get("misaligned")) for m in missions),
                 "s": round(time.time() - t0), **stats, **cstats}
        log["iters"].append(entry)
        print(f"it {it}: {entry}", flush=True)
        if (it + 1) % T.SELECT_EVERY == 0:
            sel = T.run_selection(kind, net.export(), ex)
            log["selection"].append({"it": it, **sel})
            print(f"selection it {it}: {sel}", flush=True)
            if T._reward(sel) > T._reward(best["selection"]):
                best = {"it": it, "selection": sel, "params": net.export()}
        st = {"net": net.state_dict(), "opt": opt.state_dict(), "critic": critic.state_dict(),
              "copt": copt.state_dict(), "gen": gen.get_state(), "rng": rng.getstate(),
              "torch_rng": torch.get_rng_state(), "log": log, "best": best, "it": it}
        T.save_atomic(ckpt, lambda p: torch.save(st, p))
    jsonable = lambda P: {k: np.asarray(v).round(6).tolist() for k, v in P.items()}  # noqa: E731
    res = {"arm": name, "kind": kind, "best_it": best["it"], "best_selection": best["selection"],
           "params": jsonable(best["params"]), "log": log}
    T.save_atomic(os.path.join(out, "best.json"), lambda p: json.dump(res, open(p, "w")))
    print(f"done: best it {best['it']} {best['selection']}", flush=True)
    return res


# ---------- commands ----------

def _torch():
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    return torch


def _clone(torch) -> dict:
    st = torch.load(CLONE, weights_only=False)
    got = {x: st[x] for x in tie.N_CHOICES}
    if got != tie.N_CHOICES or int(st["net"]["lin.weight"].shape[1]) != PR.POOL.N_FEAT:
        raise SystemExit(f"{CLONE}: {got} -- not 3.103.C's W32 clone")
    return st


def _done(name: str) -> str:
    return os.path.join(RUN, f"{name}.json")


def _code() -> str:
    """This file's hash: a check passed by other code is no pass."""
    return hashlib.sha256(open(os.path.abspath(__file__), "rb").read()).hexdigest()[:16]


def _passed(name: str) -> bool:
    return os.path.exists(_done(name)) and json.load(open(_done(name))).get("code") == _code()


def _need_checks() -> None:
    # Check 1 compares against W32's full-budget run: a SMOKE run cannot pass it.
    for name in ("check23",) if T.SMOKE else ("check1", "check23"):
        if not _passed(name):
            raise SystemExit(f"run `{name}` first (none passed for this code)")


def cmd_check1() -> None:
    """Check 1: this trainer in group mode reproduces W32's logged iteration
    0 (3.103.C, on AWS) exactly, and the patch moves no mission: the clone's
    selection under it equals the clone's selection without it, on today's
    code. (W32's own logged selection is reported beside it: code merged to
    `dev` after 3.103.C moves its mode U, 0.1108 -> 0.1120, with or without
    the patch.)"""
    if _passed("check1"):
        return
    PR.use_pool()
    torch = _torch()
    net = T.torch_net("F")
    net.load_state_dict(_clone(torch)["net"])
    with ProcessPoolExecutor(T.WORKERS) as ex:     # forked and gone before _capture patches
        plain = T.run_selection("F", net.export(), ex)
    _capture()
    out = os.path.join(RUN, "check1-group")
    if os.path.exists(os.path.join(out, "ckpt.pt")):
        os.remove(os.path.join(out, "ckpt.pt"))    # the check runs whole, never resumed
    os.makedirs(out, exist_ok=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        got = stage_ppo("F-pool-W32", out, _clone(torch), ex, "group", iters=1)["log"]
    want = json.load(open(W32_BEST))["log"]
    keys = ("mean_reward", "found", "n", "choices", "loss", "entropy", "clipped")
    a, b = got["iters"][0], want["iters"][0]
    diff = {k: (a[k], b[k]) for k in keys if a[k] != b[k]}
    sa = got["selection"][0]
    sdiff = {m: (sa[m], plain[m]) for m in MODES if sa[m] != plain[m]}
    res = {"code": _code(), "iteration_0": {k: a[k] for k in keys}, "w32": {k: b[k] for k in keys},
           "differences": diff, "selection_patched_vs_plain": sdiff,
           "selection_vs_w32_log": {m: (sa[m]["mean_reward"], want["selection"][0][m]["mean_reward"]) for m in MODES}}
    print(json.dumps(res), flush=True)
    if diff or sdiff:
        raise SystemExit("check 1 FAILED: group mode does not reproduce W32, or the patch moved a mission -- nothing runs")
    T.save_atomic(_done("check1"), lambda p: json.dump(res, open(p, "w"), indent=1))


def cmd_check23() -> None:
    """Checks 2 and 3 on 8 sampled training missions under the clone:
    the split sums to the reward (within its 4-place rounding), and the
    true distance moves no more between choices than the robot drove
    (plus two cells' rounding), finite at every choice."""
    if _passed("check23"):
        return
    PR.use_pool()
    _capture()
    torch = _torch()
    params = T.torch_net("F")
    params.load_state_dict(_clone(torch)["net"])
    params = params.export()
    rng = random.Random(SEED)
    hs = rng.sample(T.train_houses("F-pool-W32"), CHECK_HOUSES)
    jobs = [(s, h, MODES[k % 2], ("sample", "F", params), rng.randrange(2 ** 31)) for k, (s, h) in enumerate(hs)]
    with ProcessPoolExecutor(min(8, T.WORKERS)) as ex:
        ms = list(ex.map(T._job, jobs))
    bad, n = [], 0
    for m in ms:
        if m["reward"] is None or m["outcome"] == "exception" or not m["choices"]:
            continue
        cs = m["choices"]
        n += len(cs)
        pre = T.GOAL_COST * cs[0]["snap"]["goals"] + T.METRE_COST * cs[0]["snap"]["metres"]
        total = sum(choice_rewards(m)) - pre
        if abs(total - m["reward"]) > 1e-4 + T.METRE_COST * 0.005:
            bad.append((m["index"], "split", round(total, 5), m["reward"]))
        if any(c["snap"]["d_m"] < 0 for c in cs) or m["end_d_m"] < 0:
            bad.append((m["index"], "unreachable"))
        for p, q in zip(cs, cs[1:]):
            a, b = p["snap"], q["snap"]
            if a["d_m"] < 0 or b["d_m"] < 0:
                continue
            if abs(b["d_m"] - a["d_m"]) > (b["metres"] - a["metres"]) + 2 * CELL_M + 1e-6:
                bad.append((m["index"], "distance", a["d_m"], b["d_m"], round(b["metres"] - a["metres"], 2)))
        # the shaped split telescopes: shaping adds exactly Phi(end) - Phi(first choice)
        sh = sum(choice_rewards(m, shape=True)) - sum(choice_rewards(m))
        if abs(sh - SHAPE_PER_M * (_dist(cs[0]["snap"]) - _dist({"d_m": m["end_d_m"]}))) > 1e-9:
            bad.append((m["index"], "shaping"))
    res = {"code": _code(), "missions": len(ms), "choices": n, "outcomes": [m["outcome"] for m in ms],
           "differences": len(bad), "first": bad[:5]}
    print(json.dumps(res), flush=True)
    if bad or not n:
        raise SystemExit("checks 2-3 FAILED -- nothing runs")
    T.save_atomic(_done("check23"), lambda p: json.dump(res, open(p, "w"), indent=1))


def cmd_ppo(a: str) -> None:
    out = os.path.join(RUN, arm(a))
    if os.path.exists(os.path.join(out, "best.json")):
        print(f"{arm(a)}: PPO already done", flush=True)
        return
    _need_checks()
    if a == "A3":
        rep = os.path.join(ROWS, "report.json")
        rule = json.load(open(rep)).get("A3_rule") if os.path.exists(rep) else None
        if not (rule and rule["run_A3"]):
            raise SystemExit(f"A3 runs only by the section's rule (A2 fails criterion 1 at R >= +0.02): {rule}")
    PR.use_pool()
    _capture()
    torch = _torch()
    os.makedirs(out, exist_ok=True)
    if not os.path.exists(os.path.join(out, "ckpt.pt")):
        # A fresh run: diagnostic rows of an earlier policy under this name are not its rows.
        for mode in MODES:
            stale = os.path.join(ROWS, f"diag-{arm(a)}-{mode}.json")
            if os.path.exists(stale):
                os.remove(stale)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        stage_ppo(arm(a), out, _clone(torch), ex, a)


def cmd_diag(a: str) -> None:
    PR.use_pool()
    sp = ("greedy_log", "F", json.load(open(os.path.join(RUN, arm(a), "best.json")))["params"])
    hs = C.diag_houses()
    os.makedirs(ROWS, exist_ok=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in MODES:
            path = os.path.join(ROWS, f"diag-{arm(a)}-{mode}.json")
            if os.path.exists(path):
                continue
            rows = list(ex.map(C._job, [(s, h, mode, sp) for s, h in hs], chunksize=2))
            got = T.summary(rows)
            print(f"{arm(a)} {mode}: {got}", flush=True)
            if got["exceptions"]:
                raise SystemExit(f"{arm(a)} {mode}: {got['exceptions']} missions raised -- not saving")
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))


def _stage(fn, *args) -> None:
    """One stage in its own process: each patches 3.96's module globals."""
    p = multiprocessing.get_context("fork").Process(target=fn, args=args)
    p.start()
    p.join()
    if p.exitcode:
        raise SystemExit(f"{fn.__name__}{args}: exit {p.exitcode}")


def cmd_all() -> None:
    _stage(cmd_check1)
    _stage(cmd_check23)
    for a in ("A1", "A2"):
        _stage(cmd_ppo, a)
        _stage(cmd_diag, a)


def cmd_report() -> dict:
    boot = lambda x, y: B._boot(x, y, np.random.default_rng(SEED))   # noqa: E731  (one stream per comparison)
    w32 = {m: tie._rows(os.path.join(tie.ROWS_3111, f"F-pool-W32-{m}.json")) for m in MODES}
    orc = {m: tie._rows(os.path.join(tie.ROWS_3111, f"oracle-{m}.json")) for m in MODES}
    w32_best = json.load(open(W32_BEST))
    rep = {"W32": {"best_it": w32_best["best_it"], "best_selection": w32_best["best_selection"],
                   **{m: T.summary(list(w32[m].values())) for m in MODES},
                   "own": tie._own({(m, i): r for m in MODES for i, r in w32[m].items()})},
           "oracle_minus_W32": {m: boot(orc[m], w32[m]) for m in MODES}, "arms": {}, "criteria": {}}
    for a in ARMS:
        name = arm(a)
        paths = {m: os.path.join(ROWS, f"diag-{name}-{m}.json") for m in MODES}
        if not all(os.path.exists(p) for p in paths.values()):
            rep["criteria"][name] = None          # not run, or its drives still to run
            continue
        d = {m: tie._rows(paths[m]) for m in MODES}
        b = json.load(open(os.path.join(RUN, name, "best.json")))
        lg = b["log"]
        e = {"best_it": b["best_it"], "best_selection": b["best_selection"],
             "selection_R": [(s["it"], s["R"]["mean_reward"]) for s in lg["selection"]],
             "explained_var": [(i["it"], i.get("explained_var")) for i in lg["iters"]],
             "adv_spread": [(i["it"], i.get("adv_spread")) for i in lg["iters"]],
             **{m: T.summary(list(d[m].values())) for m in MODES},
             "own": tie._own({(m, i): r for m in MODES for i, r in d[m].items()}),
             "minus_W32_R": boot(d["R"], w32["R"]), "minus_W32_U": boot(d["U"], w32["U"]),
             "oracle_minus": {m: boot(orc[m], d[m]) for m in MODES}}
        rep["arms"][name] = e
        rep["criteria"][name] = {"1_better_R": e["minus_W32_R"]["ci95"][0] > 0,
                                 "2_U_not_worse": e["minus_W32_U"]["ci95"][0] > -0.05}
    a2 = rep["arms"].get(arm("A2"))
    rep["A3_rule"] = None if a2 is None else {
        "A2_fails_1": not rep["criteria"][arm("A2")]["1_better_R"],
        "A2_R_point": a2["minus_W32_R"]["diff"],
        "run_A3": (not rep["criteria"][arm("A2")]["1_better_R"]) and a2["minus_W32_R"]["diff"] >= 0.02}
    rep["test_gate"] = [n for n, c in rep["criteria"].items() if c and all(c.values())]
    os.makedirs(ROWS, exist_ok=True)
    T.save_atomic(os.path.join(ROWS, "report.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps(rep, indent=1))
    return rep


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd = sys.argv[1]
    if cmd in ("ppo", "diag"):
        if sys.argv[2] not in ARMS:
            raise SystemExit(f"arm {sys.argv[2]}: 3.119's arms are {ARMS}")
        {"ppo": cmd_ppo, "diag": cmd_diag}[cmd](sys.argv[2])
    else:
        {"check1": cmd_check1, "check23": cmd_check23, "all": cmd_all, "report": cmd_report}[cmd]()
