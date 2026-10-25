"""
evaluations/goal-learn-3102/cem.py -- 3.102: the evolutionary method (3.65's
cross-entropy method) on 3.96's and 3.101's fact sets, against their
gradient twins.

    python evaluations/goal-learn-3102/cem.py train ARM OUT_DIR   # ARM: E-small | E-large | E-rich
    python evaluations/goal-learn-3102/cem.py test OUT_DIR        # every E arm on the 588 test houses, R and U
    python evaluations/goal-learn-3102/cem.py score OUT_DIR       # the criteria, from OUT_DIR and 3.101's rows

The policy is a weighted sum of the arm's facts: 3.96's F network with its
MLP zeroed, so `score_np` and `mission()` run it unchanged. Training is
3.65's loop (POP guesses from N(mean, std), the ELITE best by mean reward
refit the mean and std) with 3.96's houses, targets and reward. Each
generation's guesses all play the same BATCH houses in the same modes.
Resumable: the state is saved after every generation.
"""
import json
import math
import multiprocessing
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-396"))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-3101"))
import c as C396                                              # noqa: E402
import model as M                                             # noqa: E402
import train as T                                             # noqa: E402
from rich import RICH_FEATURES, rich_features                 # noqa: E402

SEED = 3102
POP, ELITE, BATCH = 24, 6, 40                 # 3.65's
ITERS = 24                                    # 24 x 24 x 40 = 23,040 searches, the gradient arms' 60 x 48 x 8
SELECT_EVERY = 4
INIT_STD, MIN_STD = 2.0, 0.2                  # 3.65's
# 3.65's rule-like start for the ten old facts; every newer fact starts at 0.
INIT10 = [-10.0, 0.0, -10.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARMS = {"E-small": ("F-small", 17), "E-large": ("F-large", 17), "E-rich": ("F-large", 42)}
WORKERS = int(os.environ.get("WORKERS", os.cpu_count() or 4))
SMOKE = os.environ.get("SMOKE") == "1"
if SMOKE:
    POP, ELITE, BATCH, ITERS, SELECT_EVERY = 4, 2, 2, 2, 1
F_RICH_ROWS = os.path.join(ROOT, "evaluations", "goal-learn-3101", "run", "test")


def linear_params(w) -> dict:
    """3.96's F network with its MLP zeroed: the score is w . f exactly."""
    n, h = len(w), M.HIDDEN
    return {"w": [float(x) for x in w], "W1": np.zeros((h, n)).tolist(), "b1": [0.0] * h,
            "W2": np.zeros((h, h)).tolist(), "b2": [0.0] * h, "W3": np.zeros((1, h)).tolist(),
            "b3": [0.0]}


def _job(args):
    arm, s, h, mode, w, drop = args
    T.EXTRA_FEATURES = rich_features if ARMS[arm][1] == 42 else None
    try:
        r = T.mission(s, h, mode, ("greedy", "F", linear_params(w)), 0, drop)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def selection(ex, arm, w) -> dict:
    rows = list(ex.map(_job, [(arm, s, h, m, w, 0.0) for s, h in T.select_houses()
                              for m in ("R", "U")], chunksize=2))
    return {m: T.summary([r for r in rows if r["mode"] == m]) for m in ("R", "U")}


def _r(sel) -> float:
    x = sel["R"]["mean_reward"]
    return -math.inf if x is None else x


def cmd_train(arm: str, out: str) -> None:
    houses_of, n = ARMS[arm]
    os.makedirs(out, exist_ok=True)
    state_path = os.path.join(out, f"{arm}.json")
    if os.path.exists(state_path):
        st = json.load(open(state_path))
        if st.get("done"):
            print(f"{arm}: already done")
            return
        rng = random.Random()
        rng.setstate(tuple(st["rng"][:1]) + (tuple(st["rng"][1]),) + tuple(st["rng"][2:]))
        print(f"{arm}: resuming at generation {st['it'] + 1}", flush=True)
    else:
        rng = random.Random(SEED)
        st = {"arm": arm, "features": n, "seed": SEED, "pop": POP, "elite": ELITE, "batch": BATCH,
              "iters_planned": ITERS, "it": -1, "mean": INIT10 + [0.0] * (n - 10),
              "std": [INIT_STD] * n, "iters": [], "selection": [], "best": None}
    train = T.train_houses(houses_of)
    if SMOKE:
        train = train[:4]
    t0 = time.time()
    with ProcessPoolExecutor(WORKERS) as ex:
        if st["best"] is None:
            sel = selection(ex, arm, st["mean"])
            st["best"] = {"it": -1, "selection": sel, "w": st["mean"]}
            st["selection"].append({"it": -1, **sel})
            print(f"{arm} selection (start): R {sel['R']['mean_reward']} U {sel['U']['mean_reward']}",
                  flush=True)
        for it in range(st["it"] + 1, ITERS):
            batch = [(s, h, rng.choice(("R", "U"))) for s, h in rng.sample(train, BATCH)]
            samples = [[rng.gauss(m, s) for m, s in zip(st["mean"], st["std"])] for _ in range(POP)]
            rows = list(ex.map(_job, [(arm, s, h, mode, w, 0.0) for w in samples
                                      for s, h, mode in batch], chunksize=2))
            scores = []
            for i in range(POP):
                rs = [r["reward"] for r in rows[i * BATCH:(i + 1) * BATCH] if r["reward"] is not None]
                scores.append(sum(rs) / len(rs) if rs else -math.inf)
            order = sorted(range(POP), key=lambda i: -scores[i])[:ELITE]
            elite = [samples[i] for i in order]
            st["mean"] = [sum(e[k] for e in elite) / ELITE for k in range(n)]
            st["std"] = [max(MIN_STD, (sum((e[k] - st["mean"][k]) ** 2 for e in elite) / ELITE) ** 0.5)
                         for k in range(n)]
            entry = {"it": it, "best": round(scores[order[0]], 4),
                     "elite_mean": round(sum(scores[i] for i in order) / ELITE, 4),
                     "pop_mean": round(sum(scores) / POP, 4), "s": round(time.time() - t0)}
            st["iters"].append(entry)
            print(f"{arm} gen {it}: {entry}", flush=True)
            if (it + 1) % SELECT_EVERY == 0 or it == ITERS - 1:
                sel = selection(ex, arm, st["mean"])
                st["selection"].append({"it": it, **sel})
                print(f"{arm} selection gen {it}: R {sel['R']['mean_reward']} U {sel['U']['mean_reward']}",
                      flush=True)
                if _r(sel) > _r(st["best"]["selection"]):
                    st["best"] = {"it": it, "selection": sel, "w": list(st["mean"])}
            st["it"] = it
            st["rng"] = list(rng.getstate())
            T.save_atomic(state_path, lambda p: json.dump(st, open(p, "w")))
    st["done"] = True
    T.save_atomic(state_path, lambda p: json.dump(st, open(p, "w")))
    print(f"{arm} done: best gen {st['best']['it']} {st['best']['selection']}", flush=True)


def cmd_test(out: str) -> None:
    hs = C396.test_houses()
    with ProcessPoolExecutor(WORKERS) as ex:
        for arm in ARMS:
            w = json.load(open(os.path.join(out, f"{arm}.json")))["best"]["w"]
            for mode in ("R", "U"):
                path = os.path.join(out, "test", f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                os.makedirs(os.path.dirname(path), exist_ok=True)
                rows = list(ex.map(_job, [(arm, s, h, mode, w, 0.0) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode}: {T.summary(rows)}", flush=True)


def _rows(path):
    return {r["index"]: r for r in json.load(open(path))}


def _boot(a, b, rng, n=10000):
    ks = sorted(k for k in a if a[k]["reward"] is not None and k in b)
    d = np.array([a[k]["reward"] - b[k]["reward"] for k in ks])
    bs = np.array([d[rng.integers(0, len(d), len(d))].mean() for _ in range(n)])
    lo, hi = np.percentile(bs, [2.5, 97.5])
    return {"diff": round(float(d.mean()), 4), "ci95": [round(float(lo), 4), round(float(hi), 4)],
            "verdict": "level" if lo <= 0 <= hi else ("better" if lo > 0 else "worse")}


def cmd_score(out: str) -> dict:
    rng = np.random.default_rng(SEED)
    rule = {m: _rows(os.path.join(F_RICH_ROWS, f"rule-{m}.json")) for m in ("R", "U")}
    twin = {"E-small": None, "E-large": "F-large", "E-rich": "F-rich"}
    want = sorted(h for _, h in C396.test_houses())
    report = {}
    for arm in ARMS:
        st = json.load(open(os.path.join(out, f"{arm}.json")))
        res = {"best_gen": st["best"]["it"], "selection": st["best"]["selection"]}
        for m in ("R", "U"):
            e = _rows(os.path.join(out, "test", f"{arm}-{m}.json"))
            if sorted(e) != want:
                raise SystemExit(f"{arm}-{m}: not the {len(want)} test houses -- not scoring")
            s, r = T.summary(list(e.values())), T.summary(list(rule[m].values()))
            g = C396._saving(s["median_goals_found"], r["median_goals_found"])
            d = C396._saving(s["median_metres_found"], r["median_metres_found"])
            found_ok = s["found"] >= 0.99 * r["found"]
            saving_ok = (((g or -1) >= 0.20 or (d or -1) >= 0.20) if m == "R"
                         else g is not None and g >= 0)
            res[m] = {"summary": s, "vs_rule": {"goal_saving": g, "metre_saving": d,
                                                "found_ok": found_ok, "saving_ok": saving_ok,
                                                "met": found_ok and saving_ok}}
            if twin[arm]:
                f = _rows(os.path.join(F_RICH_ROWS, f"{twin[arm]}-{m}.json"))
                both = [k for k in e if e[k]["outcome"] == "found" and f[k]["outcome"] == "found"]
                res[m]["vs_twin"] = {"twin": twin[arm], "twin_summary": T.summary(list(f.values())),
                                     "reward": _boot(e, f, rng),
                                     "fewer_goals": sum(e[k]["goals"] < f[k]["goals"] for k in both),
                                     "more_goals": sum(e[k]["goals"] > f[k]["goals"] for k in both)}
        res["met_20pct"] = res["R"]["vs_rule"]["met"] and res["U"]["vs_rule"]["met"]
        res["weights"] = [round(x, 3) for x in st["best"]["w"]]
        report[arm] = res
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({a: {"met_20pct": r["met_20pct"],
                          **{m: {"reward": r[m]["summary"]["mean_reward"], "found": r[m]["summary"]["found"],
                                 "goals": r[m]["summary"]["median_goals_found"],
                                 "vs_twin": r[m].get("vs_twin", {}).get("reward")} for m in ("R", "U")}}
                      for a, r in report.items()}, indent=1))
    return report


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd = sys.argv[1]
    if cmd == "train":
        cmd_train(sys.argv[2], sys.argv[3])
    else:
        {"test": cmd_test, "score": cmd_score}[cmd](sys.argv[2])
