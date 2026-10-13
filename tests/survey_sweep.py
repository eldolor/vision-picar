"""
tests/survey_sweep.py -- 3.70 S1/S2: surveys in process, with
`tests/test_explore.py`'s `FakeNav` standing in for nav2 (a goal succeeds
iff the true floor connects it). What it measures is the survey's decisions
-- when it ends, how much floor it saw, where its room labels landed and how
many label calls it made -- not driving (the live run does that).

    python -m tests.survey_sweep houses            # 3 spread starts in each named house
    python -m tests.survey_sweep procthor [A B]    # usable ProcTHOR val houses [A, B), default 200-300

Labels are scored against the simulator's own rooms (`build_world(...).rooms`)
on truth-reachable floor (`tests/demo_explore.reachable_cells`):
* per room, the share of its reachable cells carrying its own label (bar 95%);
* `wrong_deep`: cells carrying another room's label that lie more than one
  0.3 m cell inside their own room (bar 0).
"""

import json
import math
import random
import sys
import time

from control.mission_runner import EXHAUSTED, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.demo_explore import reachable_cells
from tests.test_explore import FakeNav

HOUSES = ("scaled_house", "home_first_floor", "complex_house")
STARTS_PER_HOUSE = 3
TIME_LIMIT_S = 1800.0          # S1: 30 min (proposed; twice 3.45's 900 s)
TICK_CAP = 40000


def starts(house: str, n: int = STARTS_PER_HOUSE, seed: int = 370) -> list:
    """Spread starts: rooms in turn (largest first), a free cell whose eight
    neighbours are in the same room and free, a random heading."""
    world = build_world(house)
    rng = random.Random(seed)
    rooms = sorted(world.rooms, key=lambda r: -len(world.rooms[r]))
    out = []
    for room in rooms:
        cells = sorted(c for c in world.rooms[room]
                       if all((c[0] + dx, c[1] + dy) in world.rooms[room]
                              and (c[0] + dx, c[1] + dy) not in world.objects
                              for dx in (-1, 0, 1) for dy in (-1, 0, 1)))
        if cells:
            out.append((rng.choice(cells), rng.uniform(-math.pi, math.pi), room))
        if len(out) == n:
            break
    return out


def score_labels(world, layer: dict, width: int) -> dict:
    reach = reachable_cells(world)
    room_of = {c: r for r, cs in world.rooms.items() for c in cs}
    names = (layer or {}).get("names") or []
    cells = (layer or {}).get("cells") or []

    def label(x, y):
        k = cells[y * width + x] if cells else -1
        return names[k] if 0 <= k < len(names) else None

    per = {}
    for room, cs in world.rooms.items():
        rc = [c for c in cs if c in reach]
        if rc:
            per[room] = round(sum(label(x, y) == room for x, y in rc) / len(rc), 3)
    wrong_deep = 0
    for x, y in reach:
        truth, got = room_of.get((x, y)), label(x, y)
        if truth and got and got != truth and all(
                room_of.get((x + dx, y + dy)) == truth for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
            wrong_deep += 1
    return {"per_room": per, "rooms_below_95": sorted(r for r, v in per.items() if v < 0.95),
            "wrong_deep": wrong_deep}


def coverage(world, m: dict) -> float:
    reach = reachable_cells(world)
    w = m["width"]
    return round(sum(m["cells"][y * w + x] != -1 for x, y in reach) / len(reach), 4)


def run(house: str, start=None, label_max_calls=None, mode="rooms") -> dict:
    grid = build_world(house)
    if start is not None:
        (cx, cy), theta, _room = start
        grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    world = MockWorld(grid)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=world, mission_time_limit_s=TIME_LIMIT_S,
                           room_label_max_calls=label_max_calls, room_label_mode=mode,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    row = {"house": house, "start": start[2] if start else None}
    t0 = grid.sim_time
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
        s = runner.status()
        m = world.get_map()
        layer = runner.room_labels()
        row.update(outcome=s["outcome"], sim_s=round(grid.sim_time - t0, 1),
                   capped=ticks >= TICK_CAP, goals=(s.get("explore") or {}).get("goals_sent"),
                   coverage=coverage(build_world(house), m),
                   label_calls=s["survey"]["label_calls"],
                   **score_labels(build_world(house), layer, m["width"]))
    except Exception as e:  # noqa: BLE001 -- the sweep counts these
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}")
    return row


def main(argv):
    label_mode = "rooms"
    if argv and argv[0].startswith("--mode="):
        label_mode, argv = argv[0].split("=", 1)[1], argv[1:]
    mode = argv[0] if argv else "houses"
    t = time.time()
    if mode == "houses":
        rows = [run(h, st, mode=label_mode) for h in HOUSES for st in starts(h)]
    else:
        from sim.maps.procthor import usable
        a, b = (int(argv[1]), int(argv[2])) if len(argv) > 2 else (200, 300)
        rows = [run(f"procthor:val:{i}", mode=label_mode) for i in usable()[a:b]]
    for r in rows:
        print(json.dumps(r), flush=True)
    ex = sum(r["outcome"] == EXHAUSTED for r in rows)
    print(f"exhausted {ex}/{len(rows)}  ({time.time() - t:.0f} s)")


if __name__ == "__main__":
    main(sys.argv[1:])
