"""
evaluations/goal-learn-396/wired.py -- 3.96.C: the wired policy is the
tested one. `explore_goal_policy: rooms` (built by MissionRunner from the
flag, scored by `brain/goal_policy.py`) against the harness's own F-large
with every room label withheld (a find mission has no labels: the brain's
map carries none without a survey), mission for mission, on the 100
selection houses in both target modes -- never the test houses, which are
run once.

    python evaluations/goal-learn-396/wired.py OUT.json

Writes each pair's (outcome, goals, metres) and the count that differ.
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c as C                                                 # noqa: E402
import train as T                                             # noqa: E402


def _pair(args):
    s, h, mode = args
    out = {"split": s, "index": h, "mode": mode}
    for name, spec, drop in (("harness", C._spec("F-large")[0], 1.0), ("wired", "wired", 0.0)):
        r = T.mission(s, h, mode, spec, 0, drop)
        out[name] = [r["outcome"], r["goals"], r["metres"]]
    return out


def main(path: str) -> None:
    jobs = [(s, h, m) for s, h in T.select_houses() for m in ("R", "U")]
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4"))) as ex:
        rows = list(ex.map(_pair, jobs, chunksize=2))
    differ = [r for r in rows if r["harness"] != r["wired"]]
    found = {k: sum(r[k][0] == "found" for r in rows) for k in ("harness", "wired")}
    report = {"pairs": len(rows), "differ": len(differ), "found": found, "rows": rows}
    T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))
    for r in differ:
        print("DIFF", r)


if __name__ == "__main__":
    main(sys.argv[1])
