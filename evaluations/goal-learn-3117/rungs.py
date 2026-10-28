"""
evaluations/goal-learn-3117/rungs.py -- 3.117: 3.110's stage 2 on the
desktop. F-pool-W32 (3.103.C's network) on 3.110's rungs k = 7 (24 facts)
and k = 17 (34 facts): clone, PPO with 3.96's budget, then 3.111's 200
diagnostic houses. None on the 588 test houses.

    python evaluations/goal-learn-3117/rungs.py check K   # the rung's drive-time facts against the oracle sets' sliced columns
    python evaluations/goal-learn-3117/rungs.py clone K   # the oracle clone, the rung's columns
    python evaluations/goal-learn-3117/rungs.py ppo K     # PPO from the clone, 3.96's budget -> best.json
    python evaluations/goal-learn-3117/rungs.py diag K    # the best on the 200 diagnostic houses, R and U
    python evaluations/goal-learn-3117/rungs.py all       # check, clone, ppo, diag for every rung, in turn
    python evaluations/goal-learn-3117/rungs.py report    # criteria 1-2 per arm -> run/rows/report.json

`run/oracle-large` and `run/oracle-select` are hard links to 3.110's sets
(42 columns: 3.96's 17, then 3.101's 25). At drive time the facts are
3.101's `rich_features`, the rung's columns kept in 3.110's order, so a
choice's row is the stored row sliced by `ladder.cols(k)`. Every stage
resumes: a clone, `best.json` or a diagnostic row on disk is not run again.
"""
import json
import multiprocessing
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3114"))
import tie                                                    # noqa: E402  (3.111's and 3.103's harness, POOL_WIDTH=32)
import ladder                                                 # noqa: E402  (3.110's ORDER and cols; on 3.111's path)

C, B, T, PR = tie.C, tie.B, tie.T, tie.PR
RUN = os.environ.get("RUNGS_RUN") or os.path.join(HERE, "run")    # RUNGS_RUN: a smoke run's own folder
ROWS = os.path.join(RUN, "rows")
SEED = 3116
RUNGS = (7, 17)
MODES = C.MODES


def arm(k: int) -> str:
    return f"F-pool-W32-k{k:02d}"   # "F": 3.96 reads the kind off the name


def use_rung(k: int) -> None:
    """3.103.C's patch with the rung's facts (before the pool forks)."""
    PR.use_pool()
    c = ladder.cols(k)
    idx = [C.RICH.RICH_FEATURES.index(f) for f in ladder.ORDER[:k]]
    PR.POOL.N_FEAT = len(c)           # read by torch_net's Net at construction

    def extra(agent, m, pose, pool, feats, trail, now):
        return [[r[i] for i in idx] for r in C.RICH.rich_features(agent, m, pose, pool, feats, trail, now)]
    T.EXTRA_FEATURES = extra
    T.load_choices = lambda shards, kind: [(x[:, c], cc, y, p)
                                           for x, cc, y, p in B._load_choices(shards, kind)]
    T.SEED = tie.SEEDS[0]


def _torch():
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    return torch


CHECK_HOUSES = 8


def _checked(k: int) -> str:
    return os.path.join(RUN, f"check-k{k:02d}.json")


def cmd_check(k: int) -> None:
    """Oracle missions on shard 0's first 8 houses (both modes), re-run
    under the rung's patch, against the stored 42-column choices sliced to
    the rung: facts, picks and costs equal exactly. Passing writes a marker
    that clone and PPO require."""
    if os.path.exists(_checked(k)):
        return
    PR.check_oracle(RUN)
    use_rung(k)
    c = ladder.cols(k)
    stored = pickle.load(open(os.path.join(RUN, "oracle-large", "oracle-000.pkl"), "rb"))
    houses = list(dict.fromkeys(r["index"] for r in stored))[:CHECK_HOUSES]
    stored = [r for r in stored if r["index"] in houses]
    jobs = [(r["split"], r["index"], r["mode"], "oracle") for r in stored]
    with ProcessPoolExecutor(min(8, T.WORKERS)) as ex:
        again = list(ex.map(T._job, jobs))
    n, bad = 0, []
    for a, b in zip(stored, again):
        if len(a["choices"]) != len(b["choices"]):
            bad.append((a["index"], a["mode"], "choice count"))
            continue
        for p, q in zip(a["choices"], b["choices"]):
            n += 1
            if (q["X"].shape != (len(p["X"]), len(c)) or p["oracle"] != q["oracle"]
                    or not np.array_equal(p["costs"], q["costs"])
                    or not np.array_equal(p["X"][:, c], q["X"])):
                bad.append((a["index"], a["mode"], "facts"))
    out = {"rung": k, "facts": len(c), "houses": len(houses), "missions": len(jobs), "choices": n,
           "differences": len(bad), "first": bad[:5]}
    print(json.dumps(out), flush=True)
    if bad or not n:
        raise SystemExit(f"rung {k}: the drive-time facts differ from the oracle sets' columns: not running")
    T.save_atomic(_checked(k), lambda p: json.dump(out, open(p, "w")))


def _need_check(k: int) -> None:
    if not os.path.exists(_checked(k)):
        raise SystemExit(f"rung {k}: run `check {k}` first")


def _clone_ok(k: int, st: dict) -> None:
    got = {x: st[x] for x in tie.N_CHOICES}
    inputs = int(st["net"]["lin.weight"].shape[1])      # the patch took effect
    if inputs != len(ladder.cols(k)) or (got != tie.N_CHOICES and not T.SMOKE):
        raise SystemExit(f"{arm(k)}: {got}, {inputs} facts -- not 3.110's sets or not the rung's facts")


def cmd_clone(k: int) -> dict:
    _need_check(k)
    PR.check_oracle(RUN)
    use_rung(k)
    out = os.path.join(RUN, arm(k))
    os.makedirs(out, exist_ok=True)
    _torch()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        shards = T.stage_oracle("large", RUN, ex)        # all exist (check_oracle): nothing is run
        st = T.stage_clone(arm(k), out, shards, ex)
    print(f"{arm(k)}: {PR.POOL.size()} numbers, {st['net']['lin.weight'].shape[1]} facts, "
          f"{ {x: st[x] for x in tie.N_CHOICES} }, held-out agreement {st['held_out_agreement']}", flush=True)
    _clone_ok(k, st)
    return st


def cmd_ppo(k: int) -> None:
    use_rung(k)
    out = os.path.join(RUN, arm(k))
    if os.path.exists(os.path.join(out, "best.json")):
        print(f"{arm(k)}: PPO already done", flush=True)
        return
    _need_check(k)
    torch = _torch()
    st = torch.load(os.path.join(out, "clone.pt"), weights_only=False)
    _clone_ok(k, st)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo(arm(k), out, st, ex)


def cmd_diag(k: int) -> None:
    use_rung(k)
    sp = ("greedy_log", "F", json.load(open(os.path.join(RUN, arm(k), "best.json")))["params"])
    hs = C.diag_houses()
    os.makedirs(ROWS, exist_ok=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in MODES:
            path = os.path.join(ROWS, f"diag-{arm(k)}-{mode}.json")
            if os.path.exists(path):
                continue
            rows = list(ex.map(C._job, [(s, h, mode, sp) for s, h in hs], chunksize=2))
            got = T.summary(rows)
            print(f"{arm(k)} {mode}: {got}", flush=True)
            if got["exceptions"]:
                raise SystemExit(f"{arm(k)} {mode}: {got['exceptions']} missions raised -- not saving")
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))


def _stage(cmd: str, k: int) -> None:
    """One stage in its own process: each rung patches 3.96's module globals."""
    p = multiprocessing.get_context("fork").Process(target=CMDS[cmd], args=(k,))
    p.start()
    p.join()
    if p.exitcode:
        raise SystemExit(f"{cmd} {k}: exit {p.exitcode}")


def cmd_all() -> None:
    for k in RUNGS:                   # every rung's facts check before any training
        _stage("check", k)
    for k in RUNGS:
        for cmd in ("clone", "ppo", "diag"):
            _stage(cmd, k)


def cmd_report() -> dict:
    boot = lambda a, b: B._boot(a, b, np.random.default_rng(SEED))   # noqa: E731  (one stream per comparison)
    w32 = {m: tie._rows(os.path.join(tie.ROWS_3111, f"F-pool-W32-{m}.json")) for m in MODES}
    orc = {m: tie._rows(os.path.join(tie.ROWS_3111, f"oracle-{m}.json")) for m in MODES}
    w32_best = json.load(open(os.path.join(REPO, C.BEST["F-pool-W32"])))
    rep = {"W32": {"best_it": w32_best["best_it"], "best_selection": w32_best["best_selection"],
                   **{m: T.summary(list(w32[m].values())) for m in MODES},
                   "own": tie._own({(m, i): r for m in MODES for i, r in w32[m].items()})},
           "oracle_minus_W32": {m: boot(orc[m], w32[m]) for m in MODES}, "arms": {}, "criteria": {}}
    for k in RUNGS:
        a = arm(k)
        paths = {m: os.path.join(ROWS, f"diag-{a}-{m}.json") for m in MODES}
        if not all(os.path.exists(p) for p in paths.values()):
            rep["criteria"][a] = None          # not yet: PPO or its diagnostic drives still to run
            continue
        d = {m: tie._rows(paths[m]) for m in MODES}
        b = json.load(open(os.path.join(RUN, a, "best.json")))
        st = b["log"]
        e = {"facts": len(ladder.cols(k)), "clone_agreement": st.get("clone_agreement"),
             "best_it": b["best_it"], "best_selection": b["best_selection"],
             "selection_R": [(s["it"], s["R"]["mean_reward"]) for s in st["selection"]],
             **{m: T.summary(list(d[m].values())) for m in MODES},
             "own": tie._own({(m, i): r for m in MODES for i, r in d[m].items()}),
             "minus_W32_R": boot(d["R"], w32["R"]), "minus_W32_U": boot(d["U"], w32["U"]),
             "oracle_minus": {m: boot(orc[m], d[m]) for m in MODES}}
        rep["arms"][a] = e
        rep["criteria"][a] = {"1_better_R": e["minus_W32_R"]["ci95"][0] > 0,
                              "2_U_not_worse": e["minus_W32_U"]["ci95"][0] > -0.05}
    rep["test_gate"] = [a for a, c in rep["criteria"].items() if c and all(c.values())]
    os.makedirs(ROWS, exist_ok=True)
    T.save_atomic(os.path.join(ROWS, "report.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps(rep, indent=1))
    return rep


CMDS = {"check": cmd_check, "clone": cmd_clone, "ppo": cmd_ppo, "diag": cmd_diag}

if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd = sys.argv[1]
    if cmd in CMDS:
        k = int(sys.argv[2])
        if k not in RUNGS:
            raise SystemExit(f"rung {k}: 3.117's rungs are {RUNGS}")
        CMDS[cmd](k)
    else:
        {"all": cmd_all, "report": cmd_report}[cmd]()
