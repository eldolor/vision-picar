"""
tests/floor_band_sweep.py -- the instrument for PLAN-ros-alignment.md 3.74
criterion B1: a low object stops the robot, judged on ground truth.

An empty room, one 5 cm object (a "sock", under the lidar's plane) placed
ahead of the chassis at a seeded distance, lateral offset and heading, and
a standing FORWARD command through the wheel loop's two calls
(`vet_wheel_velocity()` then `advance()`), as `tests/footprint_sweep.py`
does. The truth is that module's: the URDF rectangle against the occupied
cells, written out without the sim's or the safety layer's code.

`band=False` reproduces the robot before 3.74 (the safety layer never
reads the band), which is the red half of the criterion: the scan passes
over the sock, so some runs must make contact.
"""

import math
import random

from robot.safety import SafetyController
from sim.grid_world import GridWorld
from sim.mock_robot import WHEEL_RADIUS_M, MockRobot
from tests.footprint_sweep import (CELL_CM, HALF_LENGTH, HALF_WIDTH, PERIOD_S, T_BAR_CM,
                                   truth)

ROOM = 24                       # cells a side: walls >= 3 m from any start
RUN_S = 6.0                     # 1 m of travel at 0.2 m/s; more than any sock is away
SPEEDS = (0.1, 0.2)             # m/s: R2b's test speed and nav2's cap
AHEAD_CM = (35.0, 100.0)        # bumper to the sock's nearest face, before rounding to a cell
SOCK = {"sock": 0.05}


def _room():
    return ["#" * ROOM] + ["#" + "." * (ROOM - 2) + "#" for _ in range(ROOM - 2)] + ["#" * ROOM]


def placements(n, seed=0):
    """`n` seeded (x, y, theta, sock_cell): the robot mid-room at a random
    heading, the sock's cell centred a random distance ahead and a random
    lateral offset off the centre-line, up to the corridor's half-width
    plus half a cell, so some sit at its edge."""
    rng = random.Random(f"floor-band-{seed}")
    out = []
    while len(out) < n:
        x, y = ROOM / 2 + rng.uniform(-1, 1), ROOM / 2 + rng.uniform(-1, 1)
        theta = rng.uniform(-math.pi, math.pi)
        ahead = (HALF_LENGTH * CELL_CM + rng.uniform(*AHEAD_CM) + CELL_CM / 2) / CELL_CM
        side = rng.uniform(-1, 1) * (HALF_WIDTH + 0.5)
        cx = x + ahead * math.cos(theta) - side * math.sin(theta)
        cy = y + ahead * math.sin(theta) + side * math.cos(theta)
        cell = (int(cx), int(cy))
        world = _world(x, y, theta, cell)
        T0, G0, _ = truth(world)
        if G0 * 1.0 < 25.0 or T0 >= 2.5 * CELL_CM:
            continue   # the sock's cell rounded too near, or out of the path entirely
        out.append((x, y, theta, cell))
    return out


def _world(x, y, theta, cell):
    world = GridWorld(_room(), {"room": set()}, objects={cell: "sock"}, object_heights=SOCK)
    world.x, world.y, world.theta = x, y, theta
    return world


def run(x, y, theta, cell, speed_m_s, band=True):
    world = _world(x, y, theta, cell)
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    if not band:
        safety.floor_clearance = lambda *a, **k: (None, "no_floor_band")
    w = speed_m_s / WHEEL_RADIUS_M
    T0, G0, _ = truth(world)
    worst_T, min_G = math.inf, G0
    for _ in range(int(RUN_S / PERIOD_S)):
        left, right, _reason = safety.vet_wheel_velocity(w, w)
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        robot.advance(PERIOD_S)
        T, G, _ = truth(world)
        min_G = min(min_G, G)
        if math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9:
            break   # stopped; static room, same command: every later period is this one
        worst_T = min(worst_T, T)
    return {"T0": T0, "G0": G0, "T_after_move": worst_T, "min_G": min_G,
            "speed": speed_m_s, "band": band}


def sweep(n, seed=0, band=True, speeds=SPEEDS):
    return [run(*p, s, band) for p in placements(n, seed) for s in speeds]


def verdict(results):
    under = [r for r in results if r["T_after_move"] < T_BAR_CM]
    touched = [r for r in results if r["min_G"] < 1.0]
    return {"runs": len(results), "under_bar": len(under), "touched": len(touched),
            "worst_T": min(r["T_after_move"] for r in results)}




# ---------------- B2a: no cost where nothing is low (amendment 1) ----------------

HOUSES = ("starter_house", "scaled_house", "home_first_floor", "complex_house")
B2_RUN_S = 3.0                  # 3.18's RUN_S
NOISE = {"noise_stddev_cm": 3.0, "dropout_rate": 0.2}


def progress_run(house, x, y, heading_deg, speed_m_s, band=True, noise=False, seed=0):
    """3.18's forward run, with the band on or off and S5's noise on or
    off. With noise the stop is not final (a later reading may let the
    robot on), so the run lasts its full time, as 3.36's lagged runs do."""
    from sim.maps import build_world
    from sim.sensors import DistanceSensorModel

    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    sensor = (DistanceSensorModel(rng=random.Random(f"{house}-{x}-{y}-{heading_deg}-{seed}"), **NOISE)
              if noise else None)
    robot = MockRobot(world, render=False, sensor=sensor)
    safety = SafetyController(robot, 20.0)
    if not band:
        safety.floor_clearance = lambda *a, **k: (None, "no_floor_band")
    w = speed_m_s / WHEEL_RADIUS_M
    T0, G0, _ = truth(world)
    worst_T, x0, y0 = math.inf, world.x, world.y
    band_bound = 0
    for _ in range(int(B2_RUN_S / PERIOD_S)):
        left, right, reason = safety.vet_wheel_velocity(w, w)
        band_bound += "floor_band" in str(reason)
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        robot.advance(PERIOD_S)
        if math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9:
            if not noise:
                break
            continue
        worst_T = min(worst_T, truth(world)[0])
    return {"house": house, "x": x, "y": y, "heading": heading_deg, "speed": speed_m_s,
            "band": band, "noise": noise, "T0": T0, "T_after_move": worst_T,
            "travel": math.hypot(world.x - x0, world.y - y0) * CELL_CM, "band_bound": band_bound}


def b2_sweep(starts_per_house, seed=0):
    """Every start, heading and speed, in all four configurations; rows keyed
    so band-on and band-off of the same run can be paired."""
    from tests.footprint_sweep import HEADINGS, starts
    out = []
    for house in HOUSES:
        for x, y in starts(house, starts_per_house, seed):
            for h in range(HEADINGS):
                for s in SPEEDS:
                    for noise in (False, True):
                        for band in (True, False):
                            out.append(progress_run(house, x, y, h * 15, s, band, noise, seed))
    return out


def b2_verdict(rows):
    from tests.footprint_sweep import PROGRESS_MIN_CM, PROGRESS_START_CM
    out = {}
    for noise in (False, True):
        on = [r for r in rows if r["noise"] == noise and r["band"]]
        off = {(r["house"], r["x"], r["y"], r["heading"], r["speed"]): r
               for r in rows if r["noise"] == noise and not r["band"]}

        def share(rs):
            el = [r for r in rs if r["T0"] >= PROGRESS_START_CM]
            return sum(r["travel"] >= PROGRESS_MIN_CM for r in el) / len(el) if el else 1.0
        earlier = [r for r in on if off[(r["house"], r["x"], r["y"], r["heading"], r["speed"])]["travel"]
                   - r["travel"] > 3.0]
        out["noise" if noise else "exact"] = {
            "runs": len(on),
            "i_under_18cm": sum(r["T_after_move"] < T_BAR_CM for r in on),
            "worst_T": round(min(r["T_after_move"] for r in on), 1),
            "ii_progress_on": round(share(on), 4), "ii_progress_off": round(share(off.values()), 4),
            "iii_earlier_by_3cm": len(earlier), "iii_share": round(len(earlier) / len(on), 4),
            "band_refused_runs": sum(r["band_bound"] > 0 for r in on),  # refusals naming the band; a slow-down names nothing
        }
    return out


if __name__ == "__main__":
    import json
    import sys
    if sys.argv[1:2] == ["b2"]:
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        print(json.dumps(b2_verdict(b2_sweep(n)), indent=1))
    else:
        n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
        for band in (True, False):
            print("band" if band else "no band", verdict(sweep(n, band=band)))
