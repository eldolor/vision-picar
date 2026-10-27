"""
evaluations/goal-learn-3111/checks.py -- 3.111: why copying the oracle
better does not search better. Three checks on the desktop, none on the
588 test houses.

    python evaluations/goal-learn-3111/checks.py diag MODEL      # MODEL greedy on the 200 diagnostic houses, R and U, every choice logged
    python evaluations/goal-learn-3111/checks.py ppo_more        # 2c: F-pool-W32's PPO, iterations 60-119
    python evaluations/goal-learn-3111/checks.py oracle3c        # 3c: the oracle sets again, plus `target_room`
    python evaluations/goal-learn-3111/checks.py same3c          # 3c's check: counts and the first 42 columns against 3.110's sets
    python evaluations/goal-learn-3111/checks.py clone3c [jobs]  # 3c: clones on 42 and 43 columns, three seeds each
    python evaluations/goal-learn-3111/checks.py report          # checks 1-3 and criteria 1-6 -> run/rows/report.json

MODEL: oracle, F-large, F-rich, F-lean-S64, F-pool-W32, F-pool-W32-clone,
F-pool-W32-more. `data/` holds hard links to files git does not keep:
3.110's `run/` (`data/r3110`) and 3.103.C's `cloud/ckpt/F-pool-W32`
(`data/W32`).
"""
import json
import math
import multiprocessing
import os
import pickle
import statistics
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3110"))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3103"))
os.environ["POOL_WIDTH"] = "32"          # the arm is F-pool-W32, whatever the shell says
DATA = os.path.join(HERE, "data")
RUN = os.path.join(HERE, "run")
ROWS = os.path.join(RUN, "rows")
C3 = os.path.join(RUN, "c3")
MORE = os.path.join(RUN, "W32-more")
SEED = 3111
SEEDS = (3961, 3962, 3963)
CELL_M = 0.30                    # the oracle's costs are true-grid cells
NEAR_M = 1.0
MODES = ("R", "U")
BEST = {"F-large": "evaluations/goal-learn-396/cloud/ckpt/F-large/best.json",
        "F-rich": "evaluations/goal-learn-3101/run/F-rich/best.json",
        "F-lean-S64": "evaluations/goal-learn-3103/cloud/ckpt/F-lean-S64/best.json",
        "F-pool-W32": "evaluations/goal-learn-3103/cloud/ckpt/F-pool-W32/best.json"}
CHOOSERS = tuple(BEST)
MODELS = ("oracle",) + CHOOSERS + ("F-pool-W32-clone", "F-pool-W32-more")


def _harness():
    """3.103's and 3.101's modules, in 3.110's import order: 3.103's run.py
    before lean's path edits put 3.101's ahead, and a pip `rich` (httpx's
    command line) set aside while 3.101's rich.py loads."""
    try:
        import httpx  # noqa: F401
    except ImportError:
        pass
    pip_rich = {k: sys.modules.pop(k) for k in list(sys.modules) if k == "rich" or k.startswith("rich.")}
    import run as B
    import lean
    import pool_run as PR
    rich101 = sys.modules["rich"]
    sys.modules.pop("rich", None)
    sys.modules.update(pip_rich)
    assert hasattr(B, "_load_choices") and hasattr(rich101, "rich_features"), (B.__file__, rich101)
    return B, lean, PR, rich101


B, LEAN, PR, RICH = _harness()
T = B.T


def diag_houses() -> list:
    """ProcTHOR `val` 0-99 and 200-299: no chooser since 3.96 trained,
    selected or was tested there (3.96: `val` 300-569 train, 100-199 select)."""
    from sim.maps.procthor import usable
    v = usable("val")
    return [("val", h) for h in v[0:100] + v[200:300]]


def use(model: str) -> None:
    """The model's harness patch, as its own test run had it (before the pool forks)."""
    if model == "F-rich":
        T.EXTRA_FEATURES = RICH.rich_features
    elif model == "F-lean-S64":
        B.use_shape("S64")
    elif model.startswith("F-pool-W32"):
        PR.use_pool()


def _jsonable(P: dict) -> dict:
    return {k: np.asarray(v).tolist() for k, v in P.items()}


def spec(model: str):
    if model == "oracle":
        return "oracle"
    if model in BEST:
        b = json.load(open(os.path.join(REPO, BEST[model])))
        return ("greedy_log", b["kind"], b["params"])
    if model == "F-pool-W32-more":
        return ("greedy_log", "F", json.load(open(os.path.join(MORE, "best.json")))["params"])
    import torch
    net = PR.POOL.torch_net("F")
    net.load_state_dict(torch.load(os.path.join(DATA, "W32", "clone.pt"), weights_only=False)["net"])
    return ("greedy_log", "F", _jsonable(net.export()))


def _job(args):
    s, h, mode, sp = args
    try:
        r = T.mission(s, h, mode, sp, 0, 0.0)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST, "choices": []}
    r["choices"] = [{"costs": [float(x) for x in c["costs"]], "i": int(c["i"]), "oracle": int(c["oracle"])}
                    for c in r.get("choices", [])]
    return r


def cmd_diag(model: str) -> None:
    if model not in MODELS:
        raise SystemExit(f"unknown model {model}")
    use(model)
    sp = spec(model)
    os.makedirs(ROWS, exist_ok=True)
    hs = diag_houses()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in MODES:
            path = os.path.join(ROWS, f"{model}-{mode}.json")
            if os.path.exists(path):
                continue
            rows = list(ex.map(_job, [(s, h, mode, sp) for s, h in hs], chunksize=2))
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
            print(f"{model} {mode}: {T.summary(rows)}", flush=True)


# ---------- 2c: more PPO ----------

def cmd_ppo_more() -> None:
    """F-pool-W32's PPO resumed from 3.103.C's iteration 59 for 60 more, into
    run/W32-more/. The resumed checkpoint's best is cleared, so best.json is
    the best of the six new selection checks, not iteration 59's."""
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    use("F-pool-W32")
    T.ITERS = 120
    os.makedirs(MORE, exist_ok=True)
    ckpt = os.path.join(MORE, "ckpt.pt")
    if not os.path.exists(ckpt):
        st = torch.load(os.path.join(DATA, "W32", "ckpt.pt"), weights_only=False)
        if st["it"] != 59:
            raise SystemExit(f"3.103.C's checkpoint is at iteration {st['it']}, not 59")
        st["best"] = {"it": None, "selection": {m: {"mean_reward": None} for m in MODES}, "params": None}
        T.save_atomic(ckpt, lambda p: torch.save(st, p))
    clone = torch.load(os.path.join(DATA, "W32", "clone.pt"), weights_only=False)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo("F-pool-W32", MORE, clone, ex)
    b = json.load(open(os.path.join(MORE, "best.json")))
    out = {"best_it": b["best_it"], "best_selection": b["best_selection"],
           "selection": [s for s in b["log"]["selection"] if s["it"] >= 60],
           "iters": [{k: e[k] for k in ("it", "mean_reward", "found", "n")} for e in b["log"]["iters"] if e["it"] >= 60]}
    T.save_atomic(os.path.join(ROWS, "ppo-more.json"), lambda p: json.dump(out, open(p, "w"), indent=1))
    print(json.dumps({k: out[k] for k in ("best_it", "best_selection")}), flush=True)


# ---------- 3c: one fact the robot can never have ----------

_now = {}


def _capture():
    """Keep the grid and target of the mission in this process (train.py
    builds both inside `mission`); `target_room` reads them."""
    bw, pt = T.build_world, T.pick_target

    def build_world(name):
        g = bw(name)
        _now.clear()
        _now["grid"] = g
        _now["room_of"] = {c: r for r, cells in g.rooms.items() for c in cells}
        return g

    def pick_target(grid, index, mode):
        t = pt(grid, index, mode)
        room_of = _now["room_of"]
        cells = [c for c, n in grid.objects.items() if n == t[0]]
        rooms = {room_of[c] for c in cells if c in room_of}
        if not rooms:
            # Only when no target cell is a room's floor cell: the rooms beside it.
            rooms = {room_of[(x + dx, y + dy)] for x, y in cells
                     for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)) if (x + dx, y + dy) in room_of}
        _now["rooms"] = rooms
        return t

    T.build_world, T.pick_target = build_world, pick_target


def target_room(pool: list) -> list:
    """1 when the candidate's goal lies in a room holding the target (a
    target cell among that room's floor cells; `pick_target` above)."""
    room_of, rooms = _now["room_of"], _now["rooms"]
    out = []
    for _, c, _ in pool:
        cell = (math.floor(c.goal[0] / CELL_M), math.floor(c.goal[1] / CELL_M))
        out.append(1.0 if room_of.get(cell) in rooms else 0.0)
    return out


def rich_plus(agent, m, pose, pool, feats, trail, now):
    return [r + [t] for r, t in zip(RICH.rich_features(agent, m, pose, pool, feats, trail, now),
                                    target_room(pool))]


def cmd_oracle3c() -> None:
    if abs(T.CELL - CELL_M) > 1e-9:
        raise SystemExit(f"the training grid is {T.CELL} m, not {CELL_M}")
    _capture()
    T.EXTRA_FEATURES = rich_plus
    os.makedirs(C3, exist_ok=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_oracle("large", C3, ex)
        T._select_oracle(C3, ex)


def _shards(root: str) -> tuple:
    big = sorted(os.path.join(root, "oracle-large", f) for f in os.listdir(os.path.join(root, "oracle-large"))
                 if f.endswith(".pkl"))
    return big, [os.path.join(root, "oracle-select", "oracle-select.pkl")]


def cmd_same3c() -> dict:
    """Every shard's rows and choices as 3.110's, the first 42 columns equal."""
    old, new = _shards(os.path.join(DATA, "r3110")), _shards(C3)
    bad, n = [], 0
    for a, b in zip(old[0] + old[1], new[0] + new[1]):
        ra, rb = pickle.load(open(a, "rb")), pickle.load(open(b, "rb"))
        if [(r["index"], r["mode"], len(r["choices"])) for r in ra] != \
                [(r["index"], r["mode"], len(r["choices"])) for r in rb]:
            bad.append(os.path.basename(b) + ": rows")
            continue
        for x, y in zip(ra, rb):
            for p, q in zip(x["choices"], y["choices"]):
                n += 1
                if not (q["X"].shape[1] == 43 and np.array_equal(p["X"], q["X"][:, :42])
                        and p["oracle"] == q["oracle"] and np.array_equal(p["costs"], q["costs"])):
                    bad.append(f"{os.path.basename(b)}: {x['index']} {x['mode']}")
    out = {"shards": [len(old[0]), len(new[0])], "choices_compared": n, "differences": len(bad),
           "first": bad[:5], "same": not bad and len(old[0]) == len(new[0])}
    print(json.dumps(out), flush=True)
    return out


def clone3c_one(ncols: int, seed: int) -> None:
    LEAN.N_FEAT = ncols              # read by make_torch_net's Net at construction
    T.SEED = seed
    T.torch_net = LEAN.make_torch_net("S32x32")
    T.load_choices = lambda shards, kind: [(x[:, :ncols], c, y, p)
                                           for x, c, y, p in B._load_choices(shards, kind)]
    import torch
    torch.set_num_threads(1)
    arm = f"F-c{ncols}-s{seed}"
    out = os.path.join(C3, arm)
    os.makedirs(out, exist_ok=True)
    with ProcessPoolExecutor(1) as ex:
        st = T.stage_clone(arm, out, T.stage_oracle("large", C3, ex), ex)
    inputs = int(st["net"]["lin.weight"].shape[1])
    if inputs != ncols:
        raise SystemExit(f"{arm}: the net reads {inputs} facts, not {ncols}")
    row = {"cols": ncols, "seed": seed, "held_out_agreement": st["held_out_agreement"],
           "n_train": st["n_train"], "n_held": st["n_held"], "net_inputs": inputs}
    T.save_atomic(os.path.join(ROWS, f"{arm}.json"), lambda p: json.dump(row, open(p, "w"), indent=1))
    print(json.dumps(row), flush=True)


def cmd_clone3c(jobs: int = 6) -> None:
    if not cmd_same3c()["same"]:
        raise SystemExit("3c's oracle sets differ from 3.110's (or are unfinished): not cloning")
    env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
    todo = [(c, s) for c in (42, 43) for s in SEEDS
            if not os.path.exists(os.path.join(ROWS, f"F-c{c}-s{s}.json"))]

    def go(cs):
        log = os.path.join(C3, f"F-c{cs[0]}-s{cs[1]}.log")
        with open(log, "w") as f:
            r = subprocess.run([sys.executable, __file__, "clone3c_one", str(cs[0]), str(cs[1])],
                               stdout=f, stderr=subprocess.STDOUT, env=env, cwd=REPO)
        print(f"F-c{cs[0]}-s{cs[1]}: exit {r.returncode}", flush=True)

    os.makedirs(C3, exist_ok=True)
    with ThreadPoolExecutor(jobs) as ex:
        list(ex.map(go, todo))


# ---------- the report ----------

def _choices_with_costs(shards: list) -> list:
    """(X, costs, oracle) in `load_choices`'s order and filter."""
    out = []
    for p in shards:
        for r in pickle.load(open(p, "rb")):
            for c in r["choices"]:
                if (c["costs"] >= 0).any():
                    out.append((c["X"], c["costs"], c["oracle"]))
    return out


def regret(costs, i: int):
    """(regret in metres, or None for a dead pick); None, None with nothing reachable."""
    c = np.asarray(costs, dtype=float)
    ok = c >= 0
    if not ok.any():
        return None, None
    if c[i] < 0:
        return None, "dead"
    return float((c[i] - c[ok].min()) * CELL_M), None


def _spearman(a, b) -> float:
    from scipy.stats import spearmanr
    return round(float(spearmanr(a, b).statistic), 4) if len(a) > 2 else None


def check1() -> dict:
    oracle = {(r["index"], m): r for m in MODES for r in json.load(open(os.path.join(ROWS, f"oracle-{m}.json")))}
    res = {}
    for model in CHOOSERS + ("F-pool-W32-clone",):
        per = {}
        by_mode = {m: json.load(open(os.path.join(ROWS, f"{model}-{m}.json"))) for m in MODES}
        for m in MODES + ("all",):
            rows = [r for mm in (MODES if m == "all" else (m,)) for r in by_mode[mm]]
            regs, dead, exact, near, n, mis_reg, short, deads = [], 0, 0, 0, 0, [], [], []
            first2 = 0.0
            for r in rows:
                tot, d0 = 0.0, dead
                for k, c in enumerate(r["choices"]):
                    g, flag = regret(c["costs"], c["i"])
                    if g is None and flag is None:
                        continue
                    n += 1
                    exact += c["i"] == c["oracle"]
                    if flag == "dead":
                        dead += 1
                        continue
                    regs.append(g)
                    near += g <= NEAR_M
                    tot += g
                    if k < 2:
                        first2 += g
                o = oracle.get((r["index"], r["mode"]))
                if o and o["reward"] is not None and r["reward"] is not None:
                    mis_reg.append(tot)
                    short.append(o["reward"] - r["reward"])
                    deads.append(dead - d0)
            srt = sorted(regs, reverse=True)
            top = sum(srt[:math.ceil(0.1 * len(srt))])
            total = sum(srt)
            per[m] = {"missions": len(rows), "choices": n, "dead_picks": dead,
                      "exact": round(exact / n, 4) if n else None,
                      "near_1m": round(near / n, 4) if n else None,
                      "mean_regret_m": round(total / max(1, len(regs)), 3),
                      "top10_share": round(top / total, 4) if total else None,
                      "first2_share": round(first2 / total, 4) if total else None,
                      "spearman_regret_shortfall": _spearman(mis_reg, short),
                      # Reported only: dead picks carry no number in the criteria.
                      "missions_with_dead_pick": sum(x > 0 for x in deads),
                      "spearman_no_dead_missions": _spearman(
                          [a for a, x in zip(mis_reg, deads) if not x],
                          [b for b, x in zip(short, deads) if not x]),
                      "mean_reward": T.summary(rows)["mean_reward"]}
        res[model] = per
    return res


def _rows_by_index(model: str, mode: str) -> dict:
    return {r["index"]: r for r in json.load(open(os.path.join(ROWS, f"{model}-{mode}.json")))}


def check2() -> dict:
    rec = {}
    for p in sorted(set(BEST.values()) | {
            "evaluations/goal-learn-396/cloud/ckpt/F-small/best.json",
            "evaluations/goal-learn-396/cloud/ckpt/MAP-large/best.json",
            "evaluations/goal-learn-396/cloud/ckpt/PONI-large/best.json",
            "evaluations/goal-learn-3103/cloud/ckpt/F-pool-W64/best.json",
            "evaluations/goal-learn-3103/cloud/ckpt/F-pool-W128/best.json",
            "evaluations/goal-learn-3103/run/F-lean-S32x32/best.json"}):
        b = json.load(open(os.path.join(REPO, p)))
        s = b["log"]["selection"]
        rec[b["arm"] if "arm" in b else p] = {
            "clone_R": s[0]["R"]["mean_reward"], "best_R": b["best_selection"]["R"]["mean_reward"],
            "lift_R": round(b["best_selection"]["R"]["mean_reward"] - s[0]["R"]["mean_reward"], 4),
            "best_it": b["best_it"], "clone_agreement": b["log"]["clone_agreement"]}
    rng = np.random.default_rng(SEED)
    out = {"record": rec}
    ppo, clone = _rows_by_index("F-pool-W32", "R"), _rows_by_index("F-pool-W32-clone", "R")
    out["2b_ppo_minus_clone_R"] = B._boot(ppo, clone, rng)
    if os.path.exists(os.path.join(ROWS, "F-pool-W32-more-R.json")):
        more = _rows_by_index("F-pool-W32-more", "R")
        out["2c_more_minus_it59_R"] = B._boot(more, ppo, rng)
        out["2c_more_minus_it59_U"] = B._boot(_rows_by_index("F-pool-W32-more", "U"),
                                              _rows_by_index("F-pool-W32", "U"), rng)
        out["2c_ppo_more"] = json.load(open(os.path.join(ROWS, "ppo-more.json")))
    return out


def check3() -> dict:
    import ladder
    import torch
    big, sel = _shards(os.path.join(DATA, "r3110"))
    train, held = _choices_with_costs(big), _choices_with_costs(sel)
    out = {"n_train": len(train), "n_held": len(held)}
    gaps = [(np.sort(c[c >= 0])[1] - c[c >= 0].min()) * CELL_M if (c >= 0).sum() > 1 else math.inf
            for _, c, _ in held]
    out["3b_near_tie_share"] = round(float(np.mean([g <= NEAR_M for g in gaps])), 4)
    fit = {}
    for k in (0, 25):
        cols = ladder.cols(k)
        LEAN.N_FEAT = len(cols)
        for s in SEEDS:
            net = LEAN.make_torch_net("S32x32")("F")
            st = torch.load(os.path.join(DATA, "r3110", ladder.arm(k, s), "clone.pt"), weights_only=False)
            net.load_state_dict(st["net"])
            with torch.no_grad():
                def picks(ch):
                    return [int(torch.argmax(net(torch.tensor(x[:, cols]), None))) for x, _, _ in ch]
                pt, ph = picks(train), picks(held)
            nr = [regret(c, i)[0] for (_, c, _), i in zip(held, ph)]
            near = [g is not None and g <= NEAR_M for g in nr]
            tie = [g <= NEAR_M for g in gaps]
            fit[f"k{k}-s{s}"] = {
                "facts": len(cols),
                "train": round(float(np.mean([i == y for i, (_, _, y) in zip(pt, train)])), 4),
                "held": round(float(np.mean([i == y for i, (_, _, y) in zip(ph, held)])), 4),
                "held_recorded": st["held_out_agreement"],
                "held_near_1m": round(float(np.mean(near)), 4),
                "held_near_1m_on_ties": round(float(np.mean([a for a, t in zip(near, tie) if t])), 4),
                "held_exact_on_ties": round(float(np.mean([i == y for i, (_, _, y), t in zip(ph, held, tie) if t])), 4),
                "held_exact_off_ties": round(float(np.mean([i == y for i, (_, _, y), t in zip(ph, held, tie) if not t])), 4)}
    out["3a_fit"] = fit
    rows = [json.load(open(os.path.join(ROWS, f"F-c{c}-s{s}.json"))) for c in (42, 43) for s in SEEDS
            if os.path.exists(os.path.join(ROWS, f"F-c{c}-s{s}.json"))]
    out["3c_clones"] = rows
    return out


def cmd_report() -> dict:
    c1, c2, c3 = check1(), check2(), check3()
    crit = {}
    crit["1_few_costly"] = all(c1[m]["all"]["top10_share"] is not None and c1[m]["all"]["top10_share"] >= 0.5
                               for m in CHOOSERS)
    sp = [c1[m]["all"]["spearman_regret_shortfall"] for m in CHOOSERS]
    crit["2_regret_tracks"] = all(x is not None and x >= 0.5 for x in sp)
    crit["2_reads_not_paid_for"] = all(x is not None and x < 0.3 for x in sp)
    crit["3_ppo_helps"] = c2["2b_ppo_minus_clone_R"]["ci95"][0] > 0
    if "2c_more_minus_it59_R" in c2:
        crit["4_budget_short"] = c2["2c_more_minus_it59_R"]["ci95"][0] > 0
    fit = c3["3a_fit"]
    crit["5_not_memorising"] = all(abs(v["train"] - v["held"]) <= 0.05 for v in fit.values())
    by = {c: [r["held_out_agreement"] * 100 for r in c3["3c_clones"] if r["cols"] == c] for c in (42, 43)}
    if all(len(v) == len(SEEDS) for v in by.values()):
        d = statistics.mean(by[43]) - statistics.mean(by[42])
        c3["3c_gain_points"] = round(d, 2)
        crit["6_target_decides"] = d >= 10.0 and min(by[43]) > max(by[42])
        crit["6_reads_not_missing"] = d < 3.0
    rep = {"check1": c1, "check2": c2, "check3": c3, "criteria": crit}
    T.save_atomic(os.path.join(ROWS, "report.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps(crit, indent=1))
    return rep


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd, *rest = sys.argv[1:]
    if cmd == "diag":
        cmd_diag(rest[0])
    elif cmd == "clone3c_one":
        clone3c_one(int(rest[0]), int(rest[1]))
    elif cmd == "clone3c":
        cmd_clone3c(*(int(x) for x in rest))
    else:
        r = {"ppo_more": cmd_ppo_more, "oracle3c": cmd_oracle3c, "same3c": cmd_same3c,
             "report": cmd_report}[cmd]()
        if cmd == "same3c" and not r["same"]:
            raise SystemExit("3c's oracle sets differ from 3.110's: not cloning")
