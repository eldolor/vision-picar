"""3.77: compare `brain/frontier.py` with the pure-Python version it replaced
(`tests/frontier_reference.py`, frozen at 3.65's head). Shared by
`test_frontier_fast.py` (fast tier) and `test_frontier_fast_sweep.py`."""
import gzip
import json
import random
from pathlib import Path

from brain import frontier as new
from brain.explore import leave_clearance_m
from tests import frontier_reference as old
from world.interface import CELL_FREE

MAPS = sorted((Path(__file__).parent.parent / "evaluations" / "goal-choice-377" / "maps")
              .glob("*.json.gz"))
SEED = 3770
LEAVE = leave_clearance_m(20)          # config/robot.yaml safety.min_distance_cm


def load(path) -> dict:
    with gzip.open(path, "rt") as f:
        return json.load(f)["final_map"]


def centre(m: dict, i: int) -> tuple:
    w, res = m["width"], m["resolution_m"]
    return m["origin_x_m"] + (i % w + 0.5) * res, m["origin_y_m"] + (i // w + 0.5) * res


def poses(m: dict, n: int, rng: random.Random) -> list:
    free = [i for i, v in enumerate(m["cells"]) if v == CELL_FREE]
    return [centre(m, i) for i in rng.sample(free, n)]


def seen_set(m: dict, n: int, rng: random.Random) -> set:
    """The cells the camera saw from `n` seeded poses and headings."""
    out = set()
    for x, y in poses(m, n, rng):
        out |= old.camera_seen(m, x, y, rng.uniform(0, 360))
    return out


def differences(m: dict, x: float, y: float, clearance_m: float, seens) -> list:
    """Every way the new code's answer differs from the reference's."""
    bad = []
    a, b = old.reachable(m, x, y, clearance_m), new.reachable(m, x, y, clearance_m)
    if list(a[0].items()) != list(b[0].items()) or a[1:] != b[1:]:
        bad.append("reachable")
    r = new.reach(m, x, y, clearance_m)
    fa = old.find_frontiers(m, x, y, clearance_m=clearance_m)
    for fb in (new.find_frontiers(m, x, y, clearance_m=clearance_m),
               new.find_frontiers(m, x, y, clearance_m=clearance_m, reach_map=r)):
        if [vars(f) for f in fa] != [vars(f) for f in fb]:
            bad.append("find_frontiers")
    for seen in seens:
        ga = old.find_view_gaps(m, x, y, seen, clearance_m=clearance_m)
        gb = new.find_view_gaps(m, x, y, seen, clearance_m=clearance_m, reach_map=r)
        if [vars(g) for g in ga] != [vars(g) for g in gb]:
            bad.append(f"find_view_gaps ({len(seen)} seen)")
    return bad
