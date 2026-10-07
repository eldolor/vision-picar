"""
python evaluations/slam-340/judge.py run.json [run.json ...]

PLAN-ros-alignment.md 3.40's criteria 1 and 2, read off demo_slam_home's
per-run JSON the same way every time. East rooms are every room east of the
foyer the 2026-10-05/06 runs bent in.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from sim.maps import build_world  # noqa: E402

EAST = {"family room", "breakfast", "kitchen", "hall", "garage hall", "laundry", "pantry", "garage"}
MAP_ROOMS = ("kitchen", "breakfast", "family room", "garage")
JUMP_M = 0.3


def judge(path, drift=False):
    d = json.load(open(path))
    g = build_world("home_first_floor")
    room = {c: n for n, cs in g.rooms.items() for c in cs}
    s = d["series"]
    east = [r for r in s if r[4] is not None
            and room.get((int(r[4] // 0.3), int(r[5] // 0.3))) in EAST]
    jumps = [(round(b[0] - s[0][0]), round(b[1] - a[1], 2)) for a, b in zip(s, s[1:])
             if b[0] - a[0] <= 2.0 and b[1] - a[1] > JUMP_M]
    rooms = (d.get("map") or {}).get("rooms", {})
    map_ok = {r: rooms.get(r, (0, None))[1] for r in MAP_ROOMS}
    max_all = max(r[1] for r in s)
    out = {"run": os.path.basename(path), "drift": d.get("drift"),
           "east_samples": len(east), "east_max_m": round(max((r[1] for r in east), default=0), 3),
           "max_m": round(max_all, 3), "final_m": d.get("final_error_m"),
           "odom_max_m": d.get("max_odom_error_m"), "jumps": jumps, "map_rooms": map_ok,
           "goals": d.get("goal_states"), "coverage": d.get("coverage")}
    if drift:   # criterion 2
        out["pass"] = (not jumps and max_all <= 0.30 and (d.get("final_error_m") or 9) <= 0.15
                       and (d.get("max_odom_error_m") or 0) > 0.30)
    else:       # criterion 1
        out["pass"] = (not jumps and out["east_max_m"] <= 0.20 and len(east) > 0
                       and (d.get("final_error_m") or 9) <= 0.10
                       and all(v is not None and v >= 0.90 for v in map_ok.values()))
    return out


if __name__ == "__main__":
    for p in sys.argv[1:]:
        d = json.load(open(p))
        print(json.dumps(judge(p, drift=bool(d.get("drift")))))
