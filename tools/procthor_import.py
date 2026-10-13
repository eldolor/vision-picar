"""
tools/procthor_import.py -- ProcTHOR-10K houses, reduced to floor plans
(PLAN-ros-alignment.md 3.66).

Downloads a pinned split of ProcTHOR-10K (`val`, or `test` since 3.65
amendment 3) and AI2's asset database (both Apache-2.0, checked by
sha256), keeps what a 2D grid can use, and writes
`sim/maps/procthor_data/<split>.json.gz`, which `sim/maps/procthor.py`
reads. The suite reads only that committed file; nothing here runs in it.

Kept per house, in metres (ProcTHOR's x and z):
* rooms: id, type, floor polygon;
* walls: each drawn wall once, with its door holes as [t0, t1] metres
  along it; `empty` (open-plan) walls and the exterior copies are dropped;
* joins: the room pairs ProcTHOR itself connects (interior doors and
  `empty` walls), the reference 3.66's criterion 1 checks the grid against;
* objects: what stands on the floor (bottom below FLOOR_BAND_M), as type,
  centre, yaw and the asset's footprint;
* start: the agent's position.

    python -m tools.procthor_import [val|test]   # fetch, verify, write
"""

import gzip
import hashlib
import json
import math
import os
import sys
import urllib.request

SOURCES = {
    "val": ("https://media.githubusercontent.com/media/allenai/procthor-10k/"
            "439193522244720b86d8c81cde2e51e3a4d150cf/val.jsonl.gz",
            "d808540514e26b6726cd2790490e669b572eeb94febb5188a2f403591dd21721"),
    # 3.65 amendment 3: the same commit's `test` split, for houses no model
    # or rule tuning has seen. Hash taken on first fetch, 2026-10-12.
    "test": ("https://media.githubusercontent.com/media/allenai/procthor-10k/"
             "439193522244720b86d8c81cde2e51e3a4d150cf/test.jsonl.gz",
             "9a9fa6f134e76fe87f3fd92c00883651cf9fadf4e9ad4072d6d73be229f001dc"),
    "assets": ("https://raw.githubusercontent.com/allenai/procthor/"
               "53d5bd4c8c96a699e6a615dc390abb670cc9d353/procthor/databases/asset-database.json",
               "a292c54a227e16b27db81e6bc105c863e674cd6c1ec892d231886ea7604395d4"),
}
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "sim", "maps", "procthor_data")

# An object whose bottom is below this reaches the band the chassis and the
# D500 meet; a painting or a wall-mounted TV does not.
FLOOR_BAND_M = 0.25


def fetch(name: str, cache_dir: str) -> bytes:
    url, sha = SOURCES[name]
    path = os.path.join(cache_dir, os.path.basename(url))
    if not os.path.exists(path):
        with urllib.request.urlopen(url, timeout=120) as r, open(path, "wb") as f:
            f.write(r.read())
    data = open(path, "rb").read()
    got = hashlib.sha256(data).hexdigest()
    if got != sha:
        # Never keep a file that failed: a truncated download would
        # otherwise fail every later run without fetching again.
        os.remove(path)
        raise ValueError(f"{name}: sha256 {got} is not the pinned {sha}")
    return data


def _r(v: float) -> float:
    return round(v, 3)


def _ends(wall):
    p = wall["polygon"]
    return (_r(p[0]["x"]), _r(p[0]["z"])), (_r(p[1]["x"]), _r(p[1]["z"]))


def reduce_house(house: dict, sizes: dict) -> dict:
    rooms = [{"id": r["id"], "type": r["roomType"],
              "poly": [[_r(p["x"]), _r(p["z"])] for p in r["floorPolygon"]]}
             for r in house["rooms"]]
    walls = {w["id"]: w for w in house["walls"] if w["roomId"] != "exterior"}

    # The two rooms' copies of one wall share their ends (checked on all
    # 1,000 val houses: every `empty` wall has exactly one empty twin).
    twins = {}
    for w in walls.values():
        twins.setdefault(tuple(sorted(_ends(w))), []).append(w)

    joins = set()
    for group in twins.values():
        if any(w.get("empty") for w in group) and len(group) == 2:
            joins.add(tuple(sorted(w["roomId"] for w in group)))

    holes = {}   # wall id -> [(t0, t1)] metres from the wall's first end
    for d in house["doors"]:
        if d["room0"] == d["room1"]:
            continue    # the front door: the house stays closed
        joins.add(tuple(sorted((d["room0"], d["room1"]))))
        (ax, az), (bx, bz) = _ends(walls[d["wall0"]])
        h = d["holePolygon"]
        t0, t1 = sorted((h[0]["x"], h[1]["x"]))
        # The hole is measured along wall0; the same span cut from wall1,
        # which may run the other way.
        ux, uz = bx - ax, bz - az
        n = math.hypot(ux, uz)
        p0 = (ax + ux * t0 / n, az + uz * t0 / n)
        p1 = (ax + ux * t1 / n, az + uz * t1 / n)
        for wid in (d["wall0"], d["wall1"]):
            (cx, cz), (ex, ez) = _ends(walls[wid])
            vx, vz = ex - cx, ez - cz
            m = math.hypot(vx, vz)
            s = sorted(((p[0] - cx) * vx + (p[1] - cz) * vz) / m for p in (p0, p1))
            holes.setdefault(wid, []).append([_r(s[0]), _r(s[1])])

    drawn = []
    for group in twins.values():
        if any(w.get("empty") for w in group):
            continue
        # One copy is drawn; the holes of every copy are cut from it, moved
        # into its direction.
        w0 = group[0]
        (ax, az), (bx, bz) = _ends(w0)
        cut = list(holes.get(w0["id"], []))
        for w in group[1:]:
            if _ends(w)[0] != (ax, az):
                length = math.hypot(bx - ax, bz - az)
                cut += [[_r(length - t1), _r(length - t0)] for t0, t1 in holes.get(w["id"], [])]
            else:
                cut += holes.get(w["id"], [])
        drawn.append([ax, az, bx, bz, sorted(cut)])

    objects = []
    for o in house["objects"]:
        size = sizes[o["assetId"]]
        bottom = o["position"]["y"] - size["box"]["y"] / 2
        if bottom >= FLOOR_BAND_M:
            continue
        objects.append({"type": size["type"], "x": _r(o["position"]["x"]),
                        "z": _r(o["position"]["z"]), "yaw": _r(o["rotation"]["y"]),
                        "sx": _r(size["box"]["x"]), "sz": _r(size["box"]["z"])})

    agent = house["metadata"]["agent"]["position"]
    return {"rooms": rooms, "walls": drawn, "joins": sorted(list(j) for j in joins),
            "objects": objects, "start": [_r(agent["x"]), _r(agent["z"])]}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    split = argv[0] if argv else "val"
    if split not in ("val", "test"):
        raise SystemExit(f"split must be val or test; got {split!r}")
    cache = os.environ.get("PROCTHOR_CACHE", os.path.expanduser("~/.cache/vision-picar/procthor"))
    os.makedirs(cache, exist_ok=True)
    assets = json.loads(fetch("assets", cache))
    sizes = {a["assetId"]: {"type": a["objectType"], "box": a["boundingBox"]}
             for group in assets.values() for a in group}
    lines = gzip.decompress(fetch(split, cache)).splitlines()
    houses = [reduce_house(json.loads(line), sizes) for line in lines]
    out = os.path.normpath(os.path.join(OUT_DIR, f"{split}.json.gz"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    payload = json.dumps({"source": {k: SOURCES[k][0] for k in (split, "assets")},
                          "houses": houses}, separators=(",", ":")).encode()
    # mtime 0: the same sources give the same bytes.
    with open(out, "wb") as f:
        f.write(gzip.compress(payload, mtime=0))
    print(f"{len(houses)} houses -> {out} ({os.path.getsize(out) / 1e3:.0f} kB)")


if __name__ == "__main__":
    sys.exit(main())
