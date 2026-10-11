"""
evaluations/explore-369/sweep.py -- 3.69's before/after sweeps: `explore`
over usable ProcTHOR houses [lo, hi) with tests/procthor_sweep.py's harness,
in parallel processes, one JSON row per house.

    python -m evaluations.explore-369.sweep LO HI OUT.json   (run as a file)
"""
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, ".")
from sim.maps.procthor import usable          # noqa: E402
from tests.procthor_sweep import run_house    # noqa: E402


def main(lo: int, hi: int, out: str) -> None:
    t = time.time()
    houses = usable()[lo:hi]
    with ProcessPoolExecutor(max_workers=6) as ex:
        rows = list(ex.map(run_house, houses))
    json.dump(rows, open(out, "w"), indent=1)
    found = sum(r["outcome"] == "found" for r in rows)
    clean = sum(r["outcome"] not in ("exception", "running") and not r.get("capped") for r in rows)
    print(f"[{lo},{hi}) found {found}/{len(rows)}  clean {clean}/{len(rows)}  ({time.time() - t:.0f} s)")


if __name__ == "__main__":
    main(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
