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
from ask_labels import AskLabeller                            # noqa: E402
from tests.test_explore import CELL                           # noqa: E402

ARMS = ("rule", "no-label", "stand-in", "ask", "ask-noisy")
NOISE = 0.15
MODES = ("R", "U")


def _ask(grid, index, noise):
    return AskLabeller(grid, CELL, index, noise=noise)


def _spec(arm: str):
    """(spec, drop, labels factory) for an arm."""
    if arm == "rule":
        return "rule", 0.0, None
    f_large = C._spec("F-large")[0]
    if arm == "no-label":
        return f_large, 1.0, None
    if arm == "stand-in":
        return f_large, 0.0, None
    return f_large, 0.0, functools.partial(_ask, noise=NOISE if arm == "ask-noisy" else 0.0)


def houses(which: str) -> list:
    return T.select_houses() if which == "select" else C.test_houses()


def _job(args):
    s, h, mode, arm = args
    spec, drop, labels = _spec(arm)
    try:
        r = T.mission(s, h, mode, spec, 0, drop, labels=labels)
    except Exception as e:  # noqa: BLE001 -- a house that cannot even start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def cmd_run(out: str, which: str) -> None:
    os.makedirs(out, exist_ok=True)
    hs = houses(which)
    json.dump({"houses": which, "n": len(hs), "noise": NOISE}, open(os.path.join(out, "run.json"), "w"))
    print(f"{len(hs)} {which} houses", flush=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for arm in ARMS:
            for mode in MODES:
                path = os.path.join(out, f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                rows = list(ex.map(_job, [(s, h, mode, arm) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode}: {T.summary(rows)}", flush=True)


def _asks(rows: list) -> dict:
    a = [r["asks"] for r in rows if r["outcome"] == "found" and "asks" in r]
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


if __name__ == "__main__":
    if sys.argv[1] == "run":
        cmd_run(sys.argv[2], sys.argv[3])
    else:
        cmd_score(sys.argv[2])
