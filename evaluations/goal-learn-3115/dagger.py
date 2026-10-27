"""
evaluations/goal-learn-3115/dagger.py -- 3.115: DAgger for F-pool-W32 on
its own states. Desktop only, none on the 588 test houses.

    python evaluations/goal-learn-3115/dagger.py check    # the learner's 18 facts against the oracle sets' sliced columns
    python evaluations/goal-learn-3115/dagger.py rounds   # rounds 1-5: drive, label, aggregate, clone, select
    python evaluations/goal-learn-3115/dagger.py diag     # the kept clone (and "ppo" once run) on 3.111's 200 diagnostic houses
    python evaluations/goal-learn-3115/dagger.py ppo      # PPO from the kept clone, 3.96's budget
    python evaluations/goal-learn-3115/dagger.py report   # criteria 1-3 -> run/rows/report.json

`run/oracle-large` and `run/oracle-select` are hard links to 3.110's sets,
as 3.114's; `run/round-0/clone.pt` a hard link to 3.103.C's W32 clone.
Every stage resumes: a round's drives, clone or selection on disk is not
run again.
"""
import json
import multiprocessing
import os
import pickle
import random
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3114"))
import tie                                                    # noqa: E402  (3.114: 3.111's and 3.103's harness, F-lean's columns)

C, B, T, PR, LEAN = tie.C, tie.B, tie.T, tie.PR, tie.LEAN
RUN = os.environ.get("DAGGER_RUN") or os.path.join(HERE, "run")   # DAGGER_RUN: a smoke run's own folder
ROWS = os.path.join(RUN, "rows")
SEED = 3115
ROUNDS = 5
HOUSES = 1000
HELD = "oracle-select"
MODES = C.MODES


def rdir(k: int) -> str:
    return os.path.join(RUN, f"round-{k}")


def _params(k: int) -> dict:
    import torch
    net = PR.POOL.torch_net("F")
    net.load_state_dict(torch.load(os.path.join(rdir(k), "clone.pt"), weights_only=False)["net"])
    return net.export()


def round_houses(k: int) -> list:
    return random.Random(SEED + k).sample(T.train_houses("F-large"), HOUSES)


def learner_choices(upto: int) -> list:
    """Every logged learner choice of rounds 1..upto, in 3.96's tuple form
    (X, crops, oracle, -) with a reachable candidate (3.96's filter)."""
    out = []
    for k in range(1, upto + 1):
        for r in pickle.load(open(os.path.join(rdir(k), "learner.pkl"), "rb")):
            for c in r["choices"]:
                if (c["costs"] >= 0).any():
                    out.append((c["X"], None, int(c["oracle"]), None))
    return out


def use_data(upto: int) -> None:
    """3.96's `load_choices` as 3.114's (the oracle's sets, F-lean's
    columns) plus, for the training shards only, rounds 1..upto's learner
    choices. The held-out set stays the oracle's 609."""
    extra = learner_choices(upto) if upto else []

    def load_choices(shards, kind):
        got = tie.load_choices(shards, kind)
        return got if any(HELD in p for p in shards) else got + extra
    T.load_choices = load_choices


def drive(k: int, ex) -> None:
    """Round k's learner (round k-1's clone) on its 1,000 houses, both modes."""
    path = os.path.join(rdir(k), "learner.pkl")
    if os.path.exists(path):
        return
    sp = ("greedy_log", "F", _params(k - 1))
    jobs = [(s, h, m, sp) for s, h in round_houses(k) for m in MODES]
    rows = list(ex.map(T._job, jobs, chunksize=4))
    os.makedirs(rdir(k), exist_ok=True)
    T.save_atomic(path, lambda p: pickle.dump(rows, open(p, "wb")))
    print(f"round {k} drives: {T.summary(rows)}, {sum(len(r['choices']) for r in rows)} choices", flush=True)


def clone(k: int, ex) -> dict:
    T.SEED = tie.SEEDS[0]
    T._batch_loss = tie.make_loss("hard")
    use_data(k)
    out = rdir(k)
    os.makedirs(out, exist_ok=True)
    shards = T.stage_oracle("large", RUN, ex)        # all exist (check_oracle): nothing is run
    return T.stage_clone(f"F-W32-dagger-r{k}", out, shards, ex)


def select(k: int, ex) -> dict:
    path = os.path.join(ROWS, "rounds.json")
    log = json.load(open(path)) if os.path.exists(path) else {}
    if str(k) in log:
        return log
    import torch
    st = torch.load(os.path.join(rdir(k), "clone.pt"), weights_only=False)
    entry = {"round": k, "n_train": st.get("n_train"), "n_held": st.get("n_held"),
             "held_out_agreement": st.get("held_out_agreement"),
             "selection": T.run_selection("F", _params(k), ex)}
    if k:
        rows = pickle.load(open(os.path.join(rdir(k), "learner.pkl"), "rb"))
        entry["drives"] = T.summary(rows)
        entry["learner_choices"] = sum(len(r["choices"]) for r in rows)
    log[str(k)] = entry
    os.makedirs(ROWS, exist_ok=True)
    T.save_atomic(path, lambda p: json.dump(log, open(p, "w"), indent=1))
    print(f"round {k}: {json.dumps({x: entry[x] for x in entry if x != 'selection'})} "
          f"selection R {entry['selection']['R']['mean_reward']} U {entry['selection']['U']['mean_reward']}",
          flush=True)
    return log


def kept(log: dict) -> int:
    """The round with the best mode-R selection reward; a tie keeps the earlier."""
    return max(sorted(int(k) for k in log), key=lambda k: log[str(k)]["selection"]["R"]["mean_reward"])


def cmd_rounds() -> None:
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "8")))
    PR.check_oracle(RUN)
    if not os.path.exists(os.path.join(rdir(0), "clone.pt")):
        raise SystemExit("run/round-0/clone.pt: hard-link 3.103.C's W32 clone there first")
    PR.use_pool()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        select(0, ex)
        for k in range(1, ROUNDS + 1):
            drive(k, ex)
            clone(k, ex)
            log = select(k, ex)
    print(f"kept round {kept(log)}", flush=True)


def cmd_check() -> None:
    """The learner's facts are the oracle sets' F-lean columns: oracle
    missions on shard 0's first houses, re-run under the learner's patch,
    against the stored 42-column choices."""
    PR.use_pool()
    stored = pickle.load(open(os.path.join(RUN, "oracle-large", "oracle-000.pkl"), "rb"))[:8]
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
            if q["X"].shape[1] != len(LEAN.LEAN_COLS) or not np.allclose(p["X"][:, LEAN.LEAN_COLS], q["X"]):
                bad.append((a["index"], a["mode"], "facts"))
    out = {"missions": len(jobs), "choices": n, "differences": len(bad), "first": bad[:5]}
    print(json.dumps(out), flush=True)
    if bad or not n:
        raise SystemExit("the learner's facts differ from the oracle sets' columns: not running DAgger")


def cmd_diag() -> None:
    PR.use_pool()
    log = json.load(open(os.path.join(ROWS, "rounds.json")))
    k = kept(log)
    todo = [(f"kept-r{k}", ("greedy_log", "F", C._jsonable(_params(k))))]
    if os.path.exists(os.path.join(RUN, "ppo", "best.json")):
        todo.append(("ppo", ("greedy_log", "F", json.load(open(os.path.join(RUN, "ppo", "best.json")))["params"])))
    hs = C.diag_houses()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for name, sp in todo:
            for mode in MODES:
                path = os.path.join(ROWS, f"diag-{name}-{mode}.json")
                if os.path.exists(path):
                    continue
                rows = list(ex.map(C._job, [(s, h, mode, sp) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{name} {mode}: {T.summary(rows)}", flush=True)


def cmd_ppo() -> None:
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    k = kept(json.load(open(os.path.join(ROWS, "rounds.json"))))
    if k == 0:
        raise SystemExit("round 0 was kept: PPO is not run (the section's rule)")
    PR.use_pool()
    T.SEED = tie.SEEDS[0]
    out = os.path.join(RUN, "ppo")
    os.makedirs(out, exist_ok=True)
    mark = os.path.join(out, "from.json")
    if os.path.exists(mark):
        if json.load(open(mark)) != k:
            raise SystemExit(f"run/ppo started from round {json.load(open(mark))}, not {k}")
    else:
        T.save_atomic(mark, lambda p: json.dump(k, open(p, "w")))
    st = torch.load(os.path.join(rdir(k), "clone.pt"), weights_only=False)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo(f"F-pool-W32-dagger-r{k}", out, st, ex)


def cmd_report() -> dict:
    boot = lambda a, b: B._boot(a, b, np.random.default_rng(SEED))   # noqa: E731  (one stream per comparison)
    log = json.load(open(os.path.join(ROWS, "rounds.json")))
    k = kept(log)
    rep = {"rounds": log, "kept": k, "criteria": {}}
    crit = rep["criteria"]
    w32 = {m: tie._rows(os.path.join(tie.ROWS_3111, f"F-pool-W32-{m}.json")) for m in MODES}
    w32c = {m: tie._rows(os.path.join(tie.ROWS_3111, f"F-pool-W32-clone-{m}.json")) for m in MODES}
    diag = {}
    for name in (f"kept-r{k}", "ppo"):
        if all(os.path.exists(os.path.join(ROWS, f"diag-{name}-{m}.json")) for m in MODES):
            diag[name] = {m: tie._rows(os.path.join(ROWS, f"diag-{name}-{m}.json")) for m in MODES}

    def own(d):
        return tie._own({(m, i): r for m in MODES for i, r in d[m].items()})
    rep["diag"] = {n: {**{m: {**T.summary(list(d[m].values())), **tie._own(d[m])} for m in MODES},
                       "all": own(d)} for n, d in diag.items()}
    rep["diag"]["W32 clone (3.111)"] = {"all": own(w32c)}
    rep["diag"]["W32 best (3.111)"] = {"all": own(w32)}
    kc = diag.get(f"kept-r{k}")
    if kc:
        base = own(w32c)["near_1m"]
        crit["1_states_covered"] = own(kc)["near_1m"] >= base + 0.10
        rep["2_kept_minus_clone_R"] = boot(kc["R"], w32c["R"])
        rep["2_kept_minus_clone_U"] = boot(kc["U"], w32c["U"])
        crit["2_clone_drives_better"] = rep["2_kept_minus_clone_R"]["ci95"][0] > 0
        rep["kept_minus_W32_best_R"] = boot(kc["R"], w32["R"])
    if k == 0:
        crit["3_survives_ppo"] = False         # not run: DAgger did not help on the selection houses
    elif "ppo" in diag:
        rep["3_ppo_minus_W32_R"] = boot(diag["ppo"]["R"], w32["R"])
        rep["3_ppo_minus_W32_U"] = boot(diag["ppo"]["U"], w32["U"])
        crit["3_survives_ppo"] = (rep["3_ppo_minus_W32_R"]["ci95"][0] > 0
                                  and rep["3_ppo_minus_W32_U"]["ci95"][0] > -0.05)
        b = json.load(open(os.path.join(RUN, "ppo", "best.json")))
        rep["ppo"] = {"best_it": b["best_it"], "best_selection": b["best_selection"],
                      "selection_R": [(s["it"], s["R"]["mean_reward"]) for s in b["log"]["selection"]]}
    T.save_atomic(os.path.join(ROWS, "report.json"), lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps({x: v for x, v in rep.items() if x != "rounds"}, indent=1))
    return rep


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    {"check": cmd_check, "rounds": cmd_rounds, "diag": cmd_diag, "ppo": cmd_ppo,
     "report": cmd_report}[sys.argv[1]]()
