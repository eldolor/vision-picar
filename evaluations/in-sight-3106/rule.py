"""
evaluations/in-sight-3106/rule.py -- 3.106 criterion 2: the rule arm on
3.96's 588 test houses (`train` 1000-1999, modes R and U), with 3.106's
bound, against 3.96's recorded rows (evaluations/goal-learn-396/test-c/).

    python evaluations/in-sight-3106/rule.py OUT_DIR
"""
import json
import os
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
C396 = os.path.join(os.path.dirname(HERE), "goal-learn-396")
sys.path.insert(0, C396)
import c  # noqa: E402

WORKERS = int(os.environ.get("WORKERS", "7"))


def summary(rows):
    found = [r for r in rows if r.get("outcome") == "found"]
    return {"found": len(found),
            "median_goals": statistics.median(r["goals"] for r in found) if found else None,
            "capped": sum(1 for r in rows if r.get("capped")),
            "missions": sum(1 for r in rows if r.get("outcome") != "no_target")}


def main(out):
    os.makedirs(out, exist_ok=True)
    houses = c.test_houses()
    result = {}
    for mode in ("R", "U"):
        jobs = [(s, h, mode, "rule", 0.0) for s, h in houses]
        with ProcessPoolExecutor(WORKERS) as pool:
            rows = list(pool.map(c._job, jobs, chunksize=4))
        with open(os.path.join(out, f"rule-{mode}.json"), "w") as f:
            json.dump(rows, f)
        with open(os.path.join(C396, "test-c", f"rule-{mode}.json")) as f:
            before = json.load(f)
        old = {(r["split"], r["index"]): r for r in before}
        changed = [{"index": r["index"], "before": {k: o.get(k) for k in ("outcome", "goals")},
                    "after": {k: r.get(k) for k in ("outcome", "goals")}}
                   for r in rows if (o := old.get((r["split"], r["index"]))) is not None
                   and (r.get("outcome"), r.get("goals")) != (o.get("outcome"), o.get("goals"))]
        result[mode] = {"before": summary(before), "after": summary(rows), "changed": changed}
        print(mode, json.dumps(result[mode]), flush=True)
    with open(os.path.join(out, "score.json"), "w") as f:
        json.dump(result, f, indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
