"""
evaluations/goal-learn-3114/tie.py -- 3.114: F-pool-W32's clone with the
oracle's near ties labelled as ties. Desktop only, none on the 588 test
houses.

    python evaluations/goal-learn-3114/tie.py clones [jobs]   # stage 1: hard / soft / set x seeds 3961-3963
    python evaluations/goal-learn-3114/tie.py select          # stage 2: every clone on the 100 selection houses; the choice
    python evaluations/goal-learn-3114/tie.py diag NAME       # stage 2: NAME (a clone arm, or "ppo") on 3.111's 200 diagnostic houses
    python evaluations/goal-learn-3114/tie.py ppo             # stage 3: PPO from the chosen clone, 3.96's budget
    python evaluations/goal-learn-3114/tie.py report          # criteria 1-3 -> run/rows/report.json

`run/oracle-large` and `run/oracle-select` are hard links to 3.110's sets
(`goal-learn-3110/run/`), the 42 columns 3.103.C's W32 trained on; F-lean's
18 are sliced from them. Every stage resumes: a clone, a selection row or
a diagnostic row on disk is not run again.
"""
import json
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
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3111"))
import checks as C                                            # noqa: E402  (3.103's and 3.111's harness, POOL_WIDTH=32)

B, T, PR, LEAN = C.B, C.T, C.PR, C.LEAN
RUN = os.environ.get("TIE_RUN") or os.path.join(HERE, "run")    # TIE_RUN: a smoke run's own folder
ROWS = os.path.join(RUN, "rows")
ROWS_3111 = os.path.join(REPO, "evaluations", "goal-learn-3111", "run", "rows")
SEED = 3114
SEEDS = (3961, 3962, 3963)
LOSSES = ("hard", "soft", "set")
NEAR_CELLS = C.NEAR_M / C.CELL_M + 1e-9
N_CHOICES = {"n_train": 37064, "n_held": 609}     # 3.103.C's (and 3.110's) sets


def arm(loss: str, seed: int) -> str:
    return f"F-W32-{loss}-s{seed}"   # "F": 3.96 reads the kind off the name


def near_set(costs) -> np.ndarray:
    """The reachable candidates within 1.0 m of the lowest oracle cost."""
    c = np.asarray(costs, dtype=float)
    ok = c >= 0
    return ok & (c - c[ok].min() <= NEAR_CELLS)


def load_choices(shards: list, kind: str) -> list:
    """3.96's `load_choices` order and filter, F-lean's columns, and the
    near set in the slot PONI's potential used."""
    out = []
    for p in shards:
        for r in pickle.load(open(p, "rb")):
            for c in r["choices"]:
                if not (c["costs"] >= 0).any():
                    continue
                out.append((c["X"][:, LEAN.LEAN_COLS], None, c["oracle"], near_set(c["costs"])))
    return out


def make_loss(loss: str):
    def _batch_loss(net, batch, kind, torch, poni=False):
        X = torch.tensor(np.concatenate([b[0] for b in batch]))
        s = net(X, seg=[len(b[0]) for b in batch])
        out, k = 0.0, 0
        for x, _, y, near in batch:
            n = len(x)
            lp = torch.log_softmax(s[k:k + n], dim=0)
            if loss == "hard":
                out = out - lp[y]
            else:
                m = torch.tensor(near)
                out = out - (lp[m].mean() if loss == "soft" else torch.logsumexp(lp[m], dim=0))
            k += n
        return out / len(batch)
    return _batch_loss


def clone_one(loss: str, seed: int) -> None:
    import torch
    torch.set_num_threads(1)
    PR.use_pool()
    T.SEED = seed
    T.load_choices = load_choices
    T._batch_loss = make_loss(loss)
    name = arm(loss, seed)
    out = os.path.join(RUN, name)
    os.makedirs(out, exist_ok=True)
    with ProcessPoolExecutor(1) as ex:
        shards = T.stage_oracle("large", RUN, ex)       # all exist (check_oracle): nothing is run
        # The counts before training, so a wrong set never leaves a clone.pt.
        got = {"n_train": len(load_choices(shards, "F")),
               "n_held": len(load_choices([os.path.join(RUN, "oracle-select", "oracle-select.pkl")], "F"))}
        if got != N_CHOICES and not T.SMOKE:
            raise SystemExit(f"{name}: {got}, not 3.103.C's {N_CHOICES}")
        st = T.stage_clone(name, out, shards, ex)
    held = load_choices([os.path.join(RUN, "oracle-select", "oracle-select.pkl")], "F")
    net = PR.POOL.torch_net("F")
    net.load_state_dict(st["net"])
    with torch.no_grad():
        picks = [int(torch.argmax(net(torch.tensor(x), None))) for x, _, _, _ in held]
    row = {"arm": name, "loss": loss, "seed": seed, **got,
           "held_exact": round(float(np.mean([i == y for i, (_, _, y, _) in zip(picks, held)])), 4),
           "held_near_1m": round(float(np.mean([nr[i] for i, (_, _, _, nr) in zip(picks, held)])), 4),
           "held_out_agreement": st["held_out_agreement"]}
    os.makedirs(ROWS, exist_ok=True)
    T.save_atomic(os.path.join(ROWS, f"clone-{name}.json"), lambda p: json.dump(row, open(p, "w"), indent=1))
    print(json.dumps(row), flush=True)


def cmd_clones(jobs: int = 9) -> None:
    # 3.110's 42-column sets must be in place: a missing one would be rebuilt
    # under use_pool's 18-column patch, by every clone at once.
    PR.check_oracle(RUN)
    env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
    todo = [(l, s) for l in LOSSES for s in SEEDS
            if not os.path.exists(os.path.join(ROWS, f"clone-{arm(l, s)}.json"))]

    def go(ls):
        log = os.path.join(RUN, f"{arm(*ls)}.log")
        with open(log, "a") as f:
            r = subprocess.run([sys.executable, __file__, "clone_one", ls[0], str(ls[1])],
                               stdout=f, stderr=subprocess.STDOUT, env=env, cwd=REPO)
        print(f"{arm(*ls)}: exit {r.returncode}", flush=True)
        return r.returncode

    with ThreadPoolExecutor(jobs) as ex:
        bad = [ls for ls, rc in zip(todo, ex.map(go, todo)) if rc]
    if bad:
        raise SystemExit(f"clones failed: {[arm(*ls) for ls in bad]} (their logs in run/)")


def _params(name: str) -> dict:
    import torch
    net = PR.POOL.torch_net("F")
    net.load_state_dict(torch.load(os.path.join(RUN, name, "clone.pt"), weights_only=False)["net"])
    return net.export()


def cmd_select() -> dict:
    PR.use_pool()
    path = os.path.join(ROWS, "select.json")
    sel = json.load(open(path)) if os.path.exists(path) else {}
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for l in LOSSES:
            for s in SEEDS:
                name = arm(l, s)
                if name in sel:
                    continue
                sel[name] = T.run_selection("F", _params(name), ex)
                T.save_atomic(path, lambda p: json.dump(sel, open(p, "w"), indent=1))
                print(f"{name}: {sel[name]}", flush=True)
    return choose(sel)


def choose(sel: dict) -> dict:
    """The loss with the higher mean selection R over its seeds, its best
    seed; hard's best seed by the same rule."""
    R = {n: v["R"]["mean_reward"] for n, v in sel.items()}
    mean = {l: statistics.mean(R[arm(l, s)] for s in SEEDS) for l in LOSSES}
    loss = max(("soft", "set", "hard"), key=lambda l: mean[l])   # ties go to a tie label, then soft
    best = lambda l: max((arm(l, s) for s in SEEDS), key=lambda n: R[n])  # noqa: E731
    out = {"mean_selection_R": {l: round(v, 4) for l, v in mean.items()},
           "chosen_loss": loss, "chosen_clone": best(loss), "hard_clone": best("hard")}
    print(json.dumps(out), flush=True)
    return out


def cmd_diag(name: str) -> None:
    PR.use_pool()
    if name == "ppo":
        b = json.load(open(os.path.join(RUN, "ppo", "best.json")))
        sp = ("greedy_log", "F", b["params"])
    else:
        sp = ("greedy_log", "F", C._jsonable(_params(name)))
    os.makedirs(ROWS, exist_ok=True)
    hs = C.diag_houses()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in C.MODES:
            path = os.path.join(ROWS, f"diag-{name}-{mode}.json")
            if os.path.exists(path):
                continue
            rows = list(ex.map(C._job, [(s, h, mode, sp) for s, h in hs], chunksize=2))
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
            print(f"{name} {mode}: {T.summary(rows)}", flush=True)


def cmd_ppo() -> None:
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    ch = choose(json.load(open(os.path.join(ROWS, "select.json"))))
    if ch["chosen_loss"] == "hard":
        raise SystemExit("hard was chosen: stage 3 is not run (the section's rule)")
    PR.use_pool()
    T.SEED = SEEDS[0]
    out = os.path.join(RUN, "ppo")
    os.makedirs(out, exist_ok=True)
    # A checkpoint resumes only the clone it started from.
    mark = os.path.join(out, "from.json")
    if os.path.exists(mark):
        if json.load(open(mark)) != ch["chosen_clone"]:
            raise SystemExit(f"run/ppo started from {json.load(open(mark))}, not {ch['chosen_clone']}")
    else:
        T.save_atomic(mark, lambda p: json.dump(ch["chosen_clone"], open(p, "w")))
    clone = torch.load(os.path.join(RUN, ch["chosen_clone"], "clone.pt"), weights_only=False)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo(f"F-pool-W32-{ch['chosen_loss']}", out, clone, ex)


# ---------- the report ----------

def _rows(path: str) -> dict:
    return {r["index"]: r for r in json.load(open(path))}


def _own(rows: dict) -> dict:
    """Near agreement and mean regret on the chooser's own choices (3.111 check 1)."""
    n, near, regs = 0, 0, []
    for r in rows.values():
        for c in r["choices"]:
            g, flag = C.regret(c["costs"], c["i"])
            if g is None and flag is None:
                continue
            n += 1
            if flag == "dead":
                continue
            regs.append(g)
            near += g <= C.NEAR_M
    return {"choices": n, "near_1m": round(near / n, 4) if n else None,
            "mean_regret_m": round(sum(regs) / max(1, len(regs)), 3)}


def cmd_report() -> dict:
    boot = lambda a, b: B._boot(a, b, np.random.default_rng(SEED))   # noqa: E731  (one stream per comparison)
    clones = {}
    for l in LOSSES:
        for s in SEEDS:
            p = os.path.join(ROWS, f"clone-{arm(l, s)}.json")
            if os.path.exists(p):
                clones[arm(l, s)] = json.load(open(p))
    rep = {"clones": clones, "criteria": {}}
    crit = rep["criteria"]
    sel_path = os.path.join(ROWS, "select.json")
    if not os.path.exists(sel_path):
        print("no selection yet")
        return rep
    sel = json.load(open(sel_path))
    want = [arm(l, s) for l in LOSSES for s in SEEDS]
    missing = [n for n in want if n not in sel or n not in clones]
    if missing:
        print(f"not every clone selected and rowed yet: {missing}")
        return rep
    rep["selection"] = sel
    ch = rep["choice"] = choose(sel)
    loss = ch["chosen_loss"]
    near = {l: [clones[arm(l, s)]["held_near_1m"] for s in SEEDS] for l in LOSSES}
    rep["held_near_mean"] = {l: round(statistics.mean(v), 4) for l, v in near.items()}
    # Hard chosen: no tie label won, so 1 FAILED, and 2-3 are not run.
    crit["1_label_learned"] = (loss != "hard"
                               and statistics.mean(near[loss]) - statistics.mean(near["hard"]) >= 0.05
                               and min(near[loss]) > max(near["hard"]))
    diag = {}
    for name in (ch["chosen_clone"], ch["hard_clone"], "ppo"):
        if all(os.path.exists(os.path.join(ROWS, f"diag-{name}-{m}.json")) for m in C.MODES):
            diag[name] = {m: _rows(os.path.join(ROWS, f"diag-{name}-{m}.json")) for m in C.MODES}
    w32 = {m: _rows(os.path.join(ROWS_3111, f"F-pool-W32-{m}.json")) for m in C.MODES}
    rep["diag"] = {n: {m: {**T.summary(list(d[m].values())), **_own(d[m])} for m in C.MODES}
                   for n, d in diag.items()}
    rep["diag"]["F-pool-W32 (3.111)"] = {m: {**T.summary(list(w32[m].values())), **_own(w32[m])}
                                        for m in C.MODES}
    a, h = ch["chosen_clone"], ch["hard_clone"]
    if a in diag and h in diag and a != h:
        rep["2_chosen_minus_hard_R"] = boot(diag[a]["R"], diag[h]["R"])
        rep["2_chosen_minus_hard_U"] = boot(diag[a]["U"], diag[h]["U"])
        crit["2_clone_drives_better"] = rep["2_chosen_minus_hard_R"]["ci95"][0] > 0
    if "ppo" in diag:
        rep["3_ppo_minus_W32_R"] = boot(diag["ppo"]["R"], w32["R"])
        rep["3_ppo_minus_W32_U"] = boot(diag["ppo"]["U"], w32["U"])
        crit["3_survives_ppo"] = (rep["3_ppo_minus_W32_R"]["ci95"][0] > 0
                                  and rep["3_ppo_minus_W32_U"]["ci95"][0] > -0.05)
        b = json.load(open(os.path.join(RUN, "ppo", "best.json")))
        rep["ppo"] = {"best_it": b["best_it"], "best_selection": b["best_selection"],
                      "selection_R": [(s["it"], s["R"]["mean_reward"]) for s in b["log"]["selection"]]}
    T.save_atomic(os.path.join(ROWS, "report.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in rep.items() if k not in ("clones", "selection", "diag")}, indent=1))
    return rep


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd, *rest = sys.argv[1:]
    if cmd == "clone_one":
        clone_one(rest[0], int(rest[1]))
    elif cmd == "clones":
        cmd_clones(*(int(x) for x in rest))
    elif cmd == "diag":
        cmd_diag(rest[0])
    else:
        {"select": cmd_select, "ppo": cmd_ppo, "report": cmd_report}[cmd]()
