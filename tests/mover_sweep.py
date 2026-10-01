"""
tests/mover_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.30, criterion 2: no contact
the robot caused, with a mover crossing its path. Shared by
`tests/test_mover_safety.py` (a sample, pinned) and
`tests/demo_mover_sweep.py` (the full sweep, printed).

3.18's method, on purpose: the geometry that JUDGES is
`tests/footprint_sweep.py`'s, written from scratch and sharing no code with
the sim's collision or `robot/safety.py`; the loop under test is the robot
server's wheel loop, `vet_wheel_velocity()` then `advance()`. What is new is
only the mover: a person pacing a straight run of cells across the robot's
heading, a few cells ahead, so the robot meets it moving.

**Why "after the robot moved" is the right bar here.** A mover never hops
within `MOVER_KEEPOUT_M` (20 cm) of the robot's turning circle, which puts
it at least ~24 cm ahead of the front edge when it lands. So a reading under
18 cm can only come from the robot's own motion -- the thing the safety
layer is responsible for -- and never from a mover appearing.
"""

import math
import random

from robot.safety import SafetyController
from sim import renderer
from sim.maps import build_world
from sim.mock_robot import MockRobot, WHEEL_RADIUS_M
from sim.movers import Mover
from tests.footprint_sweep import (CELL_CM, G_BAR_CM, HALF_LENGTH, HALF_WIDTH,
                                   T_BAR_CM, chassis, gap, square, truth)

PERIOD_S = 0.05            # robot/server.py's WHEEL_LOOP_INTERVAL_S
RUN_S = 8.0
SPEEDS_M_S = (0.1, 0.3)    # R2b's test speed, and a verb's typical speed
HEADINGS = 24              # every 15 degrees
NEAR_MOVER_CM = 30.0       # "exercised": the mover came this close


def _free(world, c):
    return (renderer._cell_at(world.layout, c[0], c[1]) != "#"
            and c not in world.objects)


def crossing_path(world, x, y, theta, ahead_cells):
    """A straight run of free cells across the heading, centred on the cell
    `ahead_cells` in front of (x, y): along y when the heading is nearer
    east/west, along x when nearer north/south. Returned as a closed walk
    (out and back), or None if fewer than 3 free cells line up."""
    cx = math.floor(x + math.cos(theta) * ahead_cells)
    cy = math.floor(y + math.sin(theta) * ahead_cells)
    if not _free(world, (cx, cy)):
        return None
    along_y = abs(math.cos(theta)) >= abs(math.sin(theta))
    step = (0, 1) if along_y else (1, 0)
    run = [(cx, cy)]
    for sign in (-1, 1):
        for k in range(1, 4):
            c = (cx + sign * step[0] * k, cy + sign * step[1] * k)
            if not _free(world, c):
                break
            run.append(c) if sign > 0 else run.insert(0, c)
    if len(run) < 3:
        return None
    return run + run[-2:0:-1]


def starts(house, n, seed=0):
    """`n` seeded starts on free floor whose turning circle is clear."""
    rng = random.Random(f"mover-{house}-{seed}")
    world = build_world(house)
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c != "#" and (x, y) not in world.objects]
    radius = math.hypot(HALF_LENGTH, HALF_WIDTH)
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        x, y = cx + 0.2 + 0.6 * rng.random(), cy + 0.2 + 0.6 * rng.random()
        r = int(math.ceil(radius)) + 1
        if any(not _free(world, (i, j))
               and math.hypot(max(i - x, 0, x - i - 1), max(j - y, 0, y - j - 1)) < radius
               for i in range(cx - r, cx + r + 1) for j in range(cy - r, cy + r + 1)):
            continue
        out.append((x, y))
    return out


def run(house, x, y, heading_deg, speed_m_s, hop_s, ahead_cells, clamp=True):
    """One standing forward command through the wheel loop's two calls,
    with a mover crossing ahead. None if no crossing fits this start."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    path = crossing_path(world, x, y, world.theta, ahead_cells)
    if path is None:
        return None
    try:
        world.add_mover(Mover("person", path, hop_s=hop_s))
    except ValueError:
        return None
    person = world.movers[0]
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    w = speed_m_s / WHEEL_RADIUS_M
    T0, G0, _ = truth(world)
    worst_T, min_G, max_P, nearest_mover = math.inf, G0, 0.0, math.inf
    x0, y0 = world.x, world.y
    for _ in range(int(RUN_S / PERIOD_S)):
        left, right = w, w
        if clamp:
            left, right, _reason = safety.vet_wheel_velocity(w, w)
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        hops_before = person.hops
        robot.advance(PERIOD_S)
        T, G, P = truth(world)
        min_G, max_P = min(min_G, G), max(max_P, P)
        A = chassis(world.x, world.y, world.theta)
        nearest_mover = min(nearest_mover, gap(A, square(*person.cell)) * CELL_CM)
        # T judges the robot's own motion: a period in which the mover hopped
        # (after the move, inside the same step) is the mover's doing, never
        # closer than it was -- contact and penetration still count there.
        if (math.hypot(world.x - before[0], world.y - before[1]) > 1e-9
                and person.hops == hops_before):
            worst_T = min(worst_T, T)
    return {"house": house, "x": x, "y": y, "heading": heading_deg, "speed": speed_m_s,
            "hop_s": hop_s, "ahead": ahead_cells, "T0": T0, "G0": G0,
            "T_after_move": worst_T, "min_G": min_G, "max_P": max_P,
            "nearest_mover_cm": nearest_mover, "hops": person.hops, "waits": person.waits,
            "travel": math.hypot(world.x - x0, world.y - y0) * CELL_CM}


def sweep(houses, starts_per_house, seed=0, headings=HEADINGS, speeds=SPEEDS_M_S, clamp=True):
    rng = random.Random(f"mover-sweep-{seed}")
    out = []
    for house in houses:
        for x, y in starts(house, starts_per_house, seed):
            for h in range(headings):
                for speed in speeds:
                    r = run(house, x, y, h * 360 / headings, speed,
                            hop_s=rng.choice((0.4, 0.7, 1.0)), ahead_cells=rng.choice((2, 3)),
                            clamp=clamp)
                    if r is not None:
                        out.append(r)
    return out


def verdicts(results):
    """Criterion 2 over a sweep: (ok, detail) per bar, plus the counts that
    say whether the bars were exercised at all."""
    c_t = [r for r in results if r["T_after_move"] < T_BAR_CM]
    c_g = [r for r in results if r["min_G"] < min(r["G0"], G_BAR_CM) - 1e-6]
    c_p = [r for r in results if r["max_P"] > 0.0]
    met = [r for r in results if r["nearest_mover_cm"] <= NEAR_MOVER_CM]
    return {
        "travel-to-contact after a robot move >= 18 cm": (
            not c_t, f"{len(c_t)}/{len(results)} runs under"
            + (f", worst {min(r['T_after_move'] for r in c_t):.1f}" if c_t else "")),
        "no contact (gap >= 1 cm)": (not c_g, f"{len(c_g)}/{len(results)} runs touched"),
        "no penetration": (not c_p, f"{len(c_p)}/{len(results)} runs inside something"),
        "exercised": (bool(met), f"{len(met)}/{len(results)} runs had the mover within "
                                 f"{NEAR_MOVER_CM:.0f} cm; median travel "
                                 f"{sorted(r['travel'] for r in results)[len(results) // 2]:.0f} cm"),
    }
