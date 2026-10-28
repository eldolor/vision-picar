"""
evaluations/goal-learn-396/wired_ask.py -- 3.116 criterion 5: the wired
`find_room_labels` is the measured one. On the 100 selection houses, mission
for mission (outcome, goals, metres):

`find_room_labels` through MissionRunner (spec "wired-ask") against 3.116.A's
harness arm `ask` (the same F-large, the same asks), in both target modes.
(Amendment 2 first compared mode U with no labels, on the belief that the
backpack has no prior; it has a near-flat one, so the brain reads its types
too, and amendment 3 compares U like R.)

    python evaluations/goal-learn-396/wired_ask.py OUT.json
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ask as A                                               # noqa: E402
import train as T                                             # noqa: E402


def _pair(args):
    s, h, mode = args
    out = {"split": s, "index": h, "mode": mode}
    spec, drop, labels = A._spec("ask")
    ref = T.mission(s, h, mode, spec, 0, drop, labels=labels)
    new = T.mission(s, h, mode, "wired-ask", 0)
    for name, r in (("reference", ref), ("wired", new)):
        out[name] = [r["outcome"], r.get("goals"), r.get("metres")]
        out[f"{name}_asks"] = r.get("asks")
    return out


def main(path: str) -> None:
    jobs = [(s, h, m) for s, h in T.select_houses() for m in ("R", "U")]
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4"))) as ex:
        rows = list(ex.map(_pair, jobs, chunksize=2))
    report = {"pairs": len(rows)}
    for m in ("R", "U"):
        rs = [r for r in rows if r["mode"] == m and r["reference"][0] != "no_target"]
        report[m] = {"houses": len(rs),
                     "same": sum(r["reference"] == r["wired"] for r in rs),
                     "found": {k: sum(r[k][0] == "found" for r in rs) for k in ("reference", "wired")}}
    report["rows"] = rows
    T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))
    for r in rows:
        if r["reference"] != r["wired"] and r["reference"][0] != "no_target":
            print("DIFF", r)


if __name__ == "__main__":
    main(sys.argv[1])
