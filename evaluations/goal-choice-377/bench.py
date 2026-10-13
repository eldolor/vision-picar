"""
evaluations/goal-choice-377/bench.py -- 3.77 criterion 2: how long one of
explore's goal choices takes on a live SLAM map, before and after.

The six end-of-run maps of 3.65's live `home_first_floor` runs (0.05 m,
`maps/`), 20 free-cell poses each from one seeded generator, camera-seen set
empty (every free cell unseen: the most view-gap work). One choice's map
work on its worst path -- every frontier skipped, so the view gaps and the
boxed-in check run too:

* old: `tests/frontier_reference.py` (the pure-Python code 3.77 replaced):
  `find_frontiers`, `find_view_gaps` and `reachable`, each with its own
  reachability search, as `_next_frontier` called them;
* new: one `reach`, shared by `find_frontiers` and `find_view_gaps`.

    python evaluations/goal-choice-377/bench.py [--new-only] [OUT.json]
"""
import json
import platform
import random
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain import frontier as new                    # noqa: E402
from tests import frontier_compare as fc             # noqa: E402
from tests import frontier_reference as old          # noqa: E402

POSES = 20


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(0.9 * len(xs)))]


def run_old(m, x, y):
    old.find_frontiers(m, x, y, clearance_m=fc.LEAVE)
    old.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE)
    old.reachable(m, x, y, clearance_m=fc.LEAVE)


def run_new(m, x, y):
    r = new.reach(m, x, y, clearance_m=fc.LEAVE)
    new.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    new.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    len(r)


def main(argv):
    new_only = "--new-only" in argv
    out = next((a for a in argv if a.endswith(".json")), None)
    rng = random.Random(fc.SEED)
    times = {"old": [], "new": []}
    for path in fc.MAPS:
        m = fc.load(path)
        for x, y in fc.poses(m, POSES, rng):
            for name, fn in (("old", run_old), ("new", run_new)):
                if name == "old" and new_only:
                    continue
                t = time.perf_counter()
                fn(m, x, y)
                times[name].append((time.perf_counter() - t) * 1000)
    summary = {"host": platform.node(), "machine": platform.machine(), "poses": len(times["new"])}
    for name, xs in times.items():
        if xs:
            summary[name] = {"p50_ms": round(statistics.median(xs), 1),
                             "p90_ms": round(p90(xs), 1), "max_ms": round(max(xs), 1)}
            print(f"{name:4s} n={len(xs)}  p50 {summary[name]['p50_ms']} ms  "
                  f"p90 {summary[name]['p90_ms']} ms  max {summary[name]['max_ms']} ms")
    if out:
        Path(out).write_text(json.dumps(summary, indent=1) + "\n")


if __name__ == "__main__":
    main(sys.argv[1:])
