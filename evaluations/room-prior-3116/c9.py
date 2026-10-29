"""
evaluations/room-prior-3116/c9.py -- 3.116 criterion 9: F-large with the
stand-in labeller, mode R, on the 588 test houses, reading the learned prior
(`learned`) or the LLM's (`llm`, `llm_prior.json`) for 3.96's nine
categories; both arms on one commit.

    python evaluations/room-prior-3116/c9.py run OUT_DIR    # resumable
    python evaluations/room-prior-3116/c9.py score OUT_DIR
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-396"))
import ask as A                                               # noqa: E402
import train as T                                             # noqa: E402

sys.path.insert(0, HERE)
from llm_prior import MODEL_ID, PROCTHOR, _load               # noqa: E402

ARMS = ("learned", "llm")


def llm_prior() -> dict:
    got = _load()[MODEL_ID]
    out = {}
    for t in PROCTHOR:
        p = got[t]["prior"]
        if p is None:
            raise SystemExit(f"no parsed LLM prior for {t!r}")
        out[t] = p
    return out


def _use(arm: str) -> None:
    """The pool's initializer: each worker sets its own prior, whatever the
    start method (spawn re-imports `train`; review)."""
    if arm == "llm":
        T.PRIOR.update(llm_prior())


def _job(args):
    s, h, arm = args
    spec, drop, labels = A._spec("stand-in")
    r = T.mission(s, h, "R", spec, 0, drop, labels=labels)
    r.pop("choices", None)
    # Which prior this row read, so a file can never pass for the other arm.
    r.update(arm=arm, prior=T.PRIOR.get(r.get("target")))
    return r


def cmd_run(out: str) -> None:
    os.makedirs(out, exist_ok=True)
    hs = A.houses("test")
    for arm in ARMS:
        path = os.path.join(out, f"{arm}-R.json")
        if os.path.exists(path):
            continue
        with ProcessPoolExecutor(T.WORKERS, initializer=_use, initargs=(arm,)) as ex:
            rows = list(ex.map(_job, [(s, h, arm) for s, h in hs], chunksize=2))
        T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
        print(arm, T.summary(rows), flush=True)


def cmd_score(out: str) -> dict:
    rows = {a: json.load(open(os.path.join(out, f"{a}-R.json"))) for a in ARMS}
    want = llm_prior()
    for r in rows["llm"]:
        if r.get("target") in want and r.get("prior") != want[r["target"]]:
            raise SystemExit(f"llm row {r['index']} read another prior -- not scoring")
    for r in rows["learned"]:
        if r.get("target") in want and r.get("prior") == want[r["target"]]:
            raise SystemExit(f"learned row {r['index']} read the LLM's prior -- not scoring")
    res = {a: T.summary(rs) for a, rs in rows.items()}
    lr, lm = res["learned"], res["llm"]
    v = {"found": lm["found"], "found_bar": round(0.99 * lr["found"], 2),
         "goals": lm["median_goals_found"], "goals_bar": lr["median_goals_found"] + 1}
    v["met"] = v["found"] >= v["found_bar"] and v["goals"] <= v["goals_bar"]
    report = {"summary": res, "criterion_9": v}
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    {"run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
