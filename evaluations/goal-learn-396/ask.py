"""
evaluations/goal-learn-396/ask.py -- 3.116.A: how much of F-large's
labelled saving survives labels asked as a find mission would ask them.

    python evaluations/goal-learn-396/ask.py run OUT_DIR select|test   # resumable
    python evaluations/goal-learn-396/ask.py score OUT_DIR             # criteria 1-4 (test)

Arms, each in target modes R and U, all on one commit:

* **rule**: today's default.
* **no-label**: F-large (`cloud/ckpt/F-large/best.json`) with every label
  withheld -- what a find mission runs today.
* **stand-in**: F-large with 3.96's stand-in labeller.
* **ask**: F-large with `ask_labels.AskLabeller` (asks as a survey does,
  each answer the true room at the robot).
* **ask-noisy**: the same, 15% of answers a different type.

Each arm-mode pair is one file (`<arm>-<mode>.json`), written once it is
complete, so a rerun skips what is done.
"""
import functools
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c as C                                                 # noqa: E402
import train as T                                             # noqa: E402
from a import pick_target                                     # noqa: E402
from ask_labels import AskLabeller                            # noqa: E402
from brain.goal_policy import rooms_prior_informative         # noqa: E402
from features import Labeller                                 # noqa: E402
from tests.test_explore import CELL                           # noqa: E402

ARMS = ("rule", "no-label", "stand-in", "ask", "ask-noisy")
# 3.116 amendment 7: `ask` with no labels for a flat-prior target. Not in
# ARMS (A's run is done); run it with `run OUT test ask-rule,ask,no-label`.
RULE_ARM = "ask-rule"
NOISE = 0.15
MODES = ("R", "U")


def _ask(grid, index, noise):
    return AskLabeller(grid, CELL, index, noise=noise)


def _ask_rule(grid, index, mode):
    """`ask`, or no labels at all when the target's prior is flat (the
    brain's rule: `goal_policy.prior_is_informative`)."""
    target, _dist = pick_target(grid, index, mode)
    # The brain's own decision, on the shipped prior (review).
    if not rooms_prior_informative(target):
        return Labeller(grid, CELL, drop=frozenset(grid.rooms))
    return AskLabeller(grid, CELL, index)


def _spec(arm: str, mode: str = None):
    """(spec, drop, labels factory) for an arm."""
    if arm == "rule":
        return "rule", 0.0, None
    f_large = C._spec("F-large")[0]
    if arm == "no-label":
        return f_large, 1.0, None
    if arm == "stand-in":
        return f_large, 0.0, None
    if arm == RULE_ARM:
        return f_large, 0.0, functools.partial(_ask_rule, mode=mode)
    if arm not in ("ask", "ask-noisy"):
        raise ValueError(f"unknown arm {arm!r}")       # a typo never runs `ask` (review)
    return f_large, 0.0, functools.partial(_ask, noise=NOISE if arm == "ask-noisy" else 0.0)


def houses(which: str) -> list:
    return T.select_houses() if which == "select" else C.test_houses()


def _job(args):
    s, h, mode, arm = args
    spec, drop, labels = _spec(arm, mode)
    try:
        r = T.mission(s, h, mode, spec, 0, drop, labels=labels)
    except Exception as e:  # noqa: BLE001 -- a house that cannot even start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def cmd_run(out: str, which: str, arms=ARMS) -> None:
    os.makedirs(out, exist_ok=True)
    hs = houses(which)
    for arm in arms:
        _spec(arm, "R")                                  # refuse a bad name before any run
    meta = os.path.join(out, "run.json")
    if os.path.exists(meta):
        # Never overwrite an earlier run's record (review): add to it.
        old = json.load(open(meta))
        if old["houses"] != which:
            raise SystemExit(f"{out} holds {old['houses']} houses, not {which}")
        arms_all = list(dict.fromkeys(old.get("arms", ARMS) + list(arms)))
    else:
        arms_all = list(arms)
    T.save_atomic(meta, lambda p: json.dump({"houses": which, "n": len(hs), "noise": NOISE,
                                             "arms": arms_all}, open(p, "w")))
    print(f"{len(hs)} {which} houses", flush=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for arm in arms:
            for mode in MODES:
                path = os.path.join(out, f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                rows = list(ex.map(_job, [(s, h, mode, arm) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode}: {T.summary(rows)}", flush=True)


def _asks(rows: list) -> dict:
    a = [r["asks"] for r in rows if r["outcome"] == "found" and r.get("asks") is not None]
    return {"median": float(np.median(a)) if a else None,
            "p90": float(np.percentile(a, 90)) if a else None}


def cmd_score(out: str) -> dict:
    which = json.load(open(os.path.join(out, "run.json")))["houses"]
    rows = {(a, m): json.load(open(os.path.join(out, f"{a}-{m}.json"))) for a in ARMS for m in MODES}
    want = sorted(h for _, h in houses(which))
    for k, rs in rows.items():
        if sorted(r["index"] for r in rs) != want:
            raise SystemExit(f"{k}: not the {len(want)} {which} houses -- not scoring")
    res = {f"{a}-{m}": T.summary(r) for (a, m), r in rows.items()}
    g = {k: v["median_goals_found"] for k, v in res.items()}
    met = {k: v["median_metres_found"] for k, v in res.items()}
    v = {}
    # 1. Recovery, mode R: half of the stand-in's goal saving over no-label.
    gap = g["no-label-R"] - g["stand-in-R"]
    kept = g["no-label-R"] - g["ask-R"]
    v["1_recovery"] = {"stand_in_saving": gap, "ask_saving": kept,
                       "share": round(kept / gap, 4) if gap else None,
                       "met": bool(gap > 0 and kept >= 0.5 * gap)}
    # 2. Finds: the rule's minus 1%, both ask arms, both modes.
    v["2_finds"] = {f"{a}-{m}": {"found": res[f"{a}-{m}"]["found"],
                                 "bar": round(0.99 * res[f"rule-{m}"]["found"], 2),
                                 "met": res[f"{a}-{m}"]["found"] >= 0.99 * res[f"rule-{m}"]["found"]}
                    for a in ("ask", "ask-noisy") for m in MODES}
    # 3. Noise not harmful, mode R.
    v["3_noise"] = {"goals": g["ask-noisy-R"], "goals_bar": g["no-label-R"],
                    "metres": met["ask-noisy-R"], "metres_bar": round(1.10 * met["no-label-R"], 2),
                    "met": bool(g["ask-noisy-R"] <= g["no-label-R"]
                                and met["ask-noisy-R"] <= 1.10 * met["no-label-R"])}
    # 4. Asks per found mission (judged in R).
    asks = {f"{a}-{m}": _asks(rows[(a, m)]) for a in ("ask", "ask-noisy") for m in MODES}
    v["4_asks"] = {"asks": asks, "met": bool(asks["ask-R"]["median"] is not None
                                            and asks["ask-R"]["median"] <= 20
                                            and asks["ask-R"]["p90"] <= 40)}
    noisy = [r for m in MODES for r in rows[("ask-noisy", m)] if "asks" in r]
    report = {"houses": which, "summary": res, "verdict": v,
              "lost_vs_rule": {f"{a}-{m}": sorted({r["index"] for r in rows[("rule", m)] if r["outcome"] == "found"}
                                                  - {r["index"] for r in rows[(a, m)] if r["outcome"] == "found"})
                               for a in ("ask", "ask-noisy") for m in MODES},
              "noisy_asks": sum(r["asks"] for r in noisy)}
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report["verdict"], indent=1))
    return report


def cmd_score_rule(out: str) -> dict:
    """3.116 criteria 11-12: `ask-rule` against `ask` (R) and no labels (U),
    house for house on (outcome, goals, metres)."""
    def key(rs):
        return {r["index"]: [r["outcome"], r.get("goals"), r.get("metres")] for r in rs}
    rows = {(a, m): key(json.load(open(os.path.join(out, f"{a}-{m}.json"))))
            for a, m in ((RULE_ARM, "R"), ("ask", "R"), (RULE_ARM, "U"), ("no-label", "U"))}
    v = {}
    for n, m, ref in (("11", "R", "ask"), ("12", "U", "no-label")):
        a, b = rows[(RULE_ARM, m)], rows[(ref, m)]
        if sorted(a) != sorted(b):
            raise SystemExit(f"criterion {n}: different houses -- not scoring")
        differ = sorted(h for h in a if a[h] != b[h])
        v[n] = {"mode": m, "against": ref, "houses": len(a), "differ": differ,
                "met": not differ}
    T.save_atomic(os.path.join(out, "score-rule.json"), lambda p: json.dump(v, open(p, "w"), indent=1))
    print(json.dumps(v, indent=1))
    return v


if __name__ == "__main__":
    if sys.argv[1] == "run":
        cmd_run(sys.argv[2], sys.argv[3], *([sys.argv[4].split(",")] if len(sys.argv) > 4 else []))
    elif sys.argv[1] == "score-rule":
        cmd_score_rule(sys.argv[2])
    else:
        cmd_score(sys.argv[2])
