"""
evaluations/explore-365/evaluate.py -- 3.65 phase 2: run the rule or a set of
weights over usable houses [lo, hi), one JSON row per house.

    python evaluations/explore-365/evaluate.py LO HI rule|learned|network|WEIGHTS.json OUT.json [val|test|train]

LO HI index the split's usable list; `test` is amendment 3's fresh houses.
"""
import json
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, ".")
from harness import run_house                      # noqa: E402
from brain.goal_policy import WEIGHTS_PATH, NetGoalPolicy  # noqa: E402
from sim.maps.procthor import usable               # noqa: E402


def _job(args):
    house, w, spec, split = args
    # The network is loaded in the worker (amendment 2's committed file).
    return run_house(house, policy=NetGoalPolicy.load(), split=split) if spec == "network" \
        else run_house(house, w, split=split)


def weights_of(spec: str):
    if spec in ("rule", "network"):
        return None
    data = json.load(open(WEIGHTS_PATH if spec == "learned" else spec))
    return data["weights"] if "weights" in data else data["best"]["weights"]


def summary(rows: list) -> dict:
    found = [r for r in rows if r["outcome"] == "found"]
    return {"n": len(rows), "found": len(found),
            "clean": sum(r["outcome"] not in ("exception", "running") and not r.get("capped")
                         for r in rows),
            "median_goals_found": statistics.median(r["goals"] for r in found) if found else None,
            "median_metres_found": statistics.median(r["metres"] for r in found) if found else None,
            "mean_reward": round(sum(r["reward"] for r in rows) / len(rows), 4)}


def main(lo: int, hi: int, spec: str, out: str, split: str = "val") -> None:
    t = time.time()
    w = weights_of(spec)
    with ProcessPoolExecutor(max_workers=max(1, (os.cpu_count() or 4) - 2)) as ex:
        rows = list(ex.map(_job, [(h, w, spec, split) for h in usable(split)[lo:hi]]))
    s = summary(rows)
    json.dump({"range": [lo, hi], "split": split, "policy": spec, "weights": w, "summary": s, "rows": rows},
              open(out, "w"), indent=1)
    print(f"[{lo},{hi}) {spec}: {s}  ({time.time() - t:.0f} s)")


if __name__ == "__main__":
    main(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3], sys.argv[4], *sys.argv[5:])
