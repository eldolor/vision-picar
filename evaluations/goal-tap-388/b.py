"""
evaluations/goal-tap-388/b.py -- 3.88 B: clone the oracle's goal choices
into explore's ten linear weights, then DAgger; judge on fresh houses.
The setup is written in the plan's status (2026-10-21) before any run.

    python evaluations/goal-tap-388/b.py collect          # oracle on training + selection houses
    python evaluations/goal-tap-388/b.py clone            # criterion 5; clone.json
    python evaluations/goal-tap-388/b.py dagger           # 10 rounds; dagger.json
    python evaluations/goal-tap-388/b.py select SPEC...   # selection sweep of rule|oracle|FILE.json
    python evaluations/goal-tap-388/b.py test SPEC...     # the 600 fresh `train` houses

Training houses: usable `val` 300-569; selection: usable `val` 100-199
(3.65's). Test: the first 600 of usable `train`.
"""
import json
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from oracle import run, yield_to_live_runs               # noqa: E402
from brain.goal_policy import FEATURES, NetGoalPolicy    # noqa: E402
from sim.maps.procthor import usable                     # noqa: E402

WORKERS = int(os.environ.get("B_WORKERS", "4"))
L2 = 1e-3
DAGGER_ROUNDS = 10


def houses(name: str) -> list:
    if name == "train":
        return [("val", h) for h in usable("val")[300:570]]
    if name == "select":
        return [("val", h) for h in usable("val")[100:200]]
    if name == "test":
        return [("train", h) for h in usable("train")[:600]]
    raise ValueError(name)


def _job(args):
    yield_to_live_runs()
    (split, h), spec = args
    if spec == "rule":
        return run(h, split, label=False)
    if spec == "oracle":
        return run(h, split)
    if spec[0] == "net":                       # clone then PPO: the network drives
        net = spec[1]
        return run(h, split, policy=NetGoalPolicy(net["weights"], [(l["W"], l["b"]) for l in net["layers"]]),
                   label=False)
    if spec[0] == "learner":                   # DAgger: the learner drives, the oracle labels
        return run(h, split, weights=spec[1])
    return run(h, split, weights=spec[1], label=False)


def sweep(spec, hs, ex) -> list:
    return list(ex.map(_job, [(h, spec) for h in hs]))


def summary(rows: list) -> dict:
    found = [r for r in rows if r["outcome"] == "found"]
    return {"n": len(rows), "found": len(found),
            "exceptions": sum(r["outcome"] == "exception" for r in rows),
            "capped": sum(bool(r.get("capped")) for r in rows),
            "median_goals_found": statistics.median(r["goals"] for r in found) if found else None,
            "median_metres_found": statistics.median(r["metres"] for r in found) if found else None,
            "mean_goals_found": round(statistics.mean(r["goals"] for r in found), 2) if found else None,
            "mean_reward": round(sum(r["reward"] for r in rows) / len(rows), 4)}


def choices_of(rows: list) -> list:
    return [c for r in rows for c in r.get("choices", [])]


# ---------- cloning ----------

def fit(choices: list, init=None, steps: int = 3000, lr: float = 0.05) -> list:
    """Ten weights minimising the mean softmax cross-entropy of the oracle's
    pick over each choice's candidates, plus L2 (Adam, full batch)."""
    X = [np.asarray(c["feats"], dtype=float) for c in choices]
    y = [c["oracle"] for c in choices]
    w = np.zeros(len(FEATURES)) if init is None else np.asarray(init, dtype=float)
    m = np.zeros_like(w)
    v = np.zeros_like(w)
    # One stacked matrix with segment offsets, so a step is a few numpy calls.
    F = np.vstack(X)
    seg = np.cumsum([0] + [len(x) for x in X])
    picked = np.array([seg[k] + y[k] for k in range(len(X))])
    owner = np.repeat(np.arange(len(X)), [len(x) for x in X])
    for t in range(1, steps + 1):
        s = F @ w
        smax = np.maximum.reduceat(s, seg[:-1])
        e = np.exp(s - smax[owner])
        z = np.add.reduceat(e, seg[:-1])
        p = e / z[owner]
        # d(-log p_y)/dw = sum_j p_j f_j - f_y, averaged over choices.
        g = (F.T @ p - F[picked].sum(axis=0)) / len(X) + 2 * L2 * w
        m = 0.9 * m + 0.1 * g
        v = 0.999 * v + 0.001 * g * g
        w = w - lr * (m / (1 - 0.9 ** t)) / (np.sqrt(v / (1 - 0.999 ** t)) + 1e-8)
    return [round(float(x), 5) for x in w]


def nll(choices: list, w) -> float:
    w = np.asarray(w)
    tot = 0.0
    for c in choices:
        s = np.asarray(c["feats"]) @ w
        s = s - s.max()
        tot += -(s[c["oracle"]] - np.log(np.exp(s).sum()))
    return round(tot / len(choices), 4)


def agreement(choices: list, w) -> dict:
    """Criterion 5: strict (the oracle's own index), tie-aware (same finite
    cost) and the rule's (index 0), over choices of two or more candidates.
    A pool the oracle cannot rank (every cost infinite) is counted in `n`
    and `unranked`, and never as a tie-aware match."""
    w = np.asarray(w)
    strict = tie = rule = rule_tie = unranked = 0
    for c in choices:
        s = np.asarray(c["feats"]) @ w
        i = int(np.argmax(s))          # first of equals: the pool's order
        costs = [np.inf if x is None else x for x in c["costs"]]
        o = c["oracle"]
        strict += i == o
        rule += o == 0
        if costs[o] == np.inf:
            unranked += 1
            continue
        tie += costs[i] == costs[o]
        rule_tie += costs[0] == costs[o]
    n = len(choices)
    return {"n": n, "unranked": unranked, "strict": round(strict / n, 4),
            "tie_aware": round(tie / n, 4),
            "rule_strict": round(rule / n, 4), "rule_tie_aware": round(rule_tie / n, 4)}


# ---------- commands ----------

def path(name: str) -> str:
    return os.path.join(HERE, name)


def dump(name: str, data) -> None:
    tmp = path(name) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path(name))


def cmd_collect(ex) -> None:
    for name in ("train", "select"):
        t = time.time()
        rows = sweep("oracle", houses(name), ex)
        dump(f"oracle-{name}.json", {"houses": name, "summary": summary(rows), "rows": rows})
        print(f"oracle {name}: {summary(rows)}  choices {len(choices_of(rows))}  "
              f"({time.time() - t:.0f} s)", flush=True)


def load_rows(name: str) -> list:
    return json.load(open(path(name)))["rows"]


def cmd_clone(ex) -> None:
    train = choices_of(load_rows("oracle-train.json"))
    held = choices_of(load_rows("oracle-select.json"))
    w = fit(train)
    out = {"features": list(FEATURES), "weights": w, "l2": L2,
           "train_choices": len(train), "train_nll": nll(train, w),
           "train_agreement": agreement(train, w),
           "held_out_agreement": agreement(held, w), "held_out_nll": nll(held, w)}
    rows = sweep(("weights", w), houses("select"), ex)
    out["selection"] = summary(rows)
    out["selection_rows"] = rows
    dump("clone.json", out)
    print(json.dumps({k: v for k, v in out.items() if k != "selection_rows"}, indent=1))


def cmd_dagger(ex) -> None:
    data = choices_of(load_rows("oracle-train.json"))
    w = json.load(open(path("clone.json")))["weights"]
    held = choices_of(load_rows("oracle-select.json"))
    log = {"features": list(FEATURES), "rounds": []}
    best = None
    t0 = time.time()
    for k in range(1, DAGGER_ROUNDS + 1):
        rows = sweep(("learner", w), houses("train"), ex)
        new = choices_of(rows)
        data += new
        w = fit(data, init=w)
        sel = sweep(("weights", w), houses("select"), ex)
        s = summary(sel)
        entry = {"round": k, "driven": summary(rows), "new_choices": len(new),
                 "choices": len(data), "weights": w, "selection": s,
                 "held_out_agreement": agreement(held, w)}
        log["rounds"].append(entry)
        if best is None or s["mean_reward"] > best["selection"]["mean_reward"]:
            best = entry
        log["best"] = {**best, "features": list(FEATURES)}
        dump("dagger.json", log)
        print(f"round {k}: driven {summary(rows)['found']} found; selection {s}; "
              f"agree {entry['held_out_agreement']['strict']}  ({time.time() - t0:.0f} s)",
              flush=True)


def spec_of(name: str):
    if name in ("rule", "oracle"):
        return name
    data = json.load(open(name))
    best = data.get("best", {})
    if "net" in best:                          # train_net.py's output
        return ("net", best["net"])
    if "weights" in best:                      # dagger.json
        return ("weights", best["weights"])
    return ("weights", data["weights"])        # clone.json


def cmd_sweep(which: str, names: list, ex) -> None:
    for name in names:
        t = time.time()
        rows = sweep(spec_of(name), houses(which), ex)
        tag = os.path.splitext(os.path.basename(name))[0]
        dump(f"{which}-{tag}.json", {"houses": which, "spec": name, "summary": summary(rows),
                                     "rows": rows})
        print(f"{which} {tag}: {summary(rows)}  ({time.time() - t:.0f} s)", flush=True)


def _nice() -> None:
    os.nice(10)


def main(argv) -> None:
    with ProcessPoolExecutor(max_workers=WORKERS, initializer=_nice) as ex:
        cmd = argv[0]
        if cmd == "collect":
            cmd_collect(ex)
        elif cmd == "clone":
            cmd_clone(ex)
        elif cmd == "dagger":
            cmd_dagger(ex)
        elif cmd in ("select", "test"):
            cmd_sweep(cmd, argv[1:], ex)
        else:
            raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
