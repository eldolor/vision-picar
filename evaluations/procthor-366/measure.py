"""
evaluations/procthor-366/measure.py -- 3.66's criteria 1-4 over every
ProcTHOR `val` house (or `test`, 3.65 amendment 3). Writes
`conversion.json` (`conversion-test.json`) beside itself and the usable list.

    python evaluations/procthor-366/measure.py [val|test|train]
"""

import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from sim.maps.procthor import convert, houses, usable_path  # noqa: E402
from tests.chassis_fit import SAFETY_MARGIN_M, UGV_ROVER, fit  # noqa: E402

def main(split="val"):
    out = os.path.join(os.path.dirname(__file__),
                       "conversion.json" if split == "val" else f"conversion-{split}.json")
    rows = []
    for index in range(len(houses(split))):
        t = time.perf_counter()
        try:
            cv = convert(index, split)
        except ValueError as e:
            # A house the conversion cannot start a robot in (none in `val`;
            # `test` has three): recorded, never usable, and counted as a
            # failure below. Any other conversion error still stops the run.
            if "no free cell to start on" not in str(e):
                raise
            rows.append({"index": index, "error": str(e), "rooms": None, "usable": False,
                         "lost": ["not converted"], "leaked": [], "area_err": None,
                         "floor_objects": 0, "placed_objects": 0, "build_s": None})
            continue
        build_s = time.perf_counter() - t
        got = cv.grid_joins()
        row = {"index": index, "rooms": len(cv.room_ids), "build_s": round(build_s, 4),
               "lost": sorted(map(list, cv.joins - got)), "leaked": sorted(map(list, got - cv.joins)),
               "area_err": round(cv.outline_area_m2 / cv.source_area_m2 - 1, 4),
               "floor_objects": cv.floor_objects, "placed_objects": cv.placed_objects,
               "target": any(v == "red backpack" for v in cv.world.objects.values())}
        if row["rooms"] >= 3:
            f = fit(f"procthor:{split}:{index}", *UGV_ROVER, SAFETY_MARGIN_M)
            row["start_free"] = f["start_free"]
            row["unreached"] = sorted(n for n, r in f["rooms"].items() if not r["reached"])
            row["usable"] = f["start_free"] and not row["unreached"]
        rows.append(row)
        if index % 100 == 0:
            print(index, flush=True)
    json.dump(rows, open(out, "w"), indent=0)
    with open(usable_path(split), "w") as f:
        json.dump([r["index"] for r in rows if r.get("usable")], f)

    failed = [r for r in rows if "error" in r]
    print(f"not converted (counted as failing 1, 2 and 4): {len(failed)} "
          f"{[r['index'] for r in failed][:20]}")
    n = len(rows)
    topo = sum(1 for r in rows if not r["lost"] and not r["leaked"])
    area = sum(1 for r in rows if r["area_err"] is not None and abs(r["area_err"]) <= 0.05)
    fo = sum(r["floor_objects"] for r in rows)
    po = sum(r["placed_objects"] for r in rows)
    # An unconverted house's rooms are unknown: it counts in criterion 4's
    # denominator as not usable rather than vanishing from it.
    big = [r for r in rows if r["rooms"] is None or r["rooms"] >= 3]
    usable = sum(1 for r in big if r["usable"])
    print(f"1 topology kept: {topo}/{n} = {topo / n:.1%} (bar 95%)")
    print(f"2 outline within 5%: {area}/{n} = {area / n:.1%} (bar 95%)")
    print(f"3 furniture placed: {po}/{fo} = {po / fo:.2%} (bar 98%)")
    print(f"4 usable (>= 3 rooms): {usable}/{len(big)} = {usable / len(big):.1%} (bar 80%)")
    print(f"6 build median {statistics.median(r['build_s'] for r in rows if r['build_s'] is not None) * 1000:.0f} ms (bar 500)")


if __name__ == "__main__":
    main(*sys.argv[1:])
