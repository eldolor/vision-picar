"""
tests/creep_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.44 criterion 2 -- a creep
never touches. Shared by `tests/test_speed_clearance.py` (a sample, pinned)
and `python -m tests.creep_sweep` (the full sweep, printed).

3.44 lets a straight move ASKED at `CREEP_M_S` or slower come within
`CREEP_MARGIN_CM` (3 cm) where every other move keeps 20 cm. This sweep
starts the chassis close to something -- truth's travel-to-contact within
`NEAR_CM` in the direction it will creep -- and creeps FORWARD or REVERSE
until the safety layer refuses, through the two paths a creep really takes:

* `verb`: `SafetyController.check_and_execute(action, speed=CREEP_SPEED)`,
  direct mode's path (and the escape's, `brain/explore.py`);
* `wheels`: the wheel loop's `vet_wheel_velocity()` at the creep speed while
  the creep verb is the one asked (`_asking()`), which is how a creep verb's
  twists are vetted under `drive: ros`.

Each in both sensing modes: `instant` (the simulator's scan and depth grid)
and `lidar` (`footprint_sweep._LidarTimed`: the D500's timed scan through
the real driver, no depth grid -- 3.42 criterion 4).

**Ground truth**, as in 3.18 and 3.22: `footprint_sweep.truth()`, sampled
after every movement the simulator makes. A run fails if the chassis ever
comes within `G_BAR_CM` (2 cm) of anything it did not start closer to --
3.18's `min(G0, bar)` convention -- or touches.
"""

import logging
import math
import random
import sys
import time

from robot.safety import CREEP_M_S, SafetyController, SafetyViolation, verb_speed_m_s
from sim.maps import build_world
from sim.mock_robot import MockRobot, WHEEL_RADIUS_M
from tests import footprint_sweep as fs
from tests import verb_sweep as vs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
NEAR_CM = 25.0                 # starts with something this close ahead
CREEP_SPEED = 5                # a verb's speed 0-100: 0.03 m/s, the creep bar
G_BAR_CM = 2.0                 # criterion 2: no sample under 2 cm of true gap
# The progress half (CLAUDE.md 7.4), stated before the full run: the margin
# is used -- half the creeps end within twice it of contact. Today's 20 cm
# bar leaves every start under 20 cm where it began (median ~12 cm).
PROGRESS_MEDIAN_CM = 2 * 3.0
MAX_VERBS = 3                  # a creep verb plans a whole cell; repeat until refused
PERIOD_S = fs.PERIOD_S
WHEEL_RUN_S = 12.0             # NEAR_CM at 3 cm/s, with room to spare

assert verb_speed_m_s(CREEP_SPEED) <= CREEP_M_S


def near_starts(house, n, seed=0):
    """`n` seeded (x, y, theta, direction): the chassis legal (no contact)
    and truth's travel-to-contact in `direction` within NEAR_CM."""
    rng = random.Random(f"creep-{house}-{seed}")
    world = build_world(house)
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c != "#" and (x, y) not in world.solid_cells]
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        world.x, world.y = cx + rng.random(), cy + rng.random()
        world.theta = rng.uniform(-math.pi, math.pi)
        d = rng.choice((+1, -1))
        T, G, P = fs.truth(world, d)
        if G <= 0.0 or P > 0.0 or T > NEAR_CM:
            continue
        out.append((world.x, world.y, world.theta, d))
    return out


class _LidarVerb(fs._LidarTimed):
    """`_LidarTimed` for the verb path: the verb loop advances the body
    itself, so the scan is brought up to date after every advance -- the
    same order as the wheel loop's (record, vet, command, advance)."""

    def advance(self, dt):
        self._robot.advance(dt)
        self.record()


def run(house, x, y, theta, direction, path="verb", lidar=False):
    world = build_world(house)
    world.x, world.y, world.theta = x, y, theta
    robot = MockRobot(world, render=False)
    if lidar:
        sensed = _LidarVerb(robot) if path == "verb" else fs._LidarTimed(robot)
    else:
        sensed = robot
    safety = SafetyController(sensed, 20.0)
    T0, G0, _ = fs.truth(world, direction)
    samples = []
    vs._sampled(world, direction, samples)
    x0, y0 = world.x, world.y
    refused = None
    if path == "verb":
        action = "FORWARD" if direction > 0 else "REVERSE"
        for _ in range(MAX_VERBS):
            try:
                safety.check_and_execute(action, speed=CREEP_SPEED, duration=0.5)
            except SafetyViolation as e:
                refused = str(e)
                break
    else:
        w = direction * verb_speed_m_s(CREEP_SPEED) / WHEEL_RADIUS_M
        with safety._asking(direction * verb_speed_m_s(CREEP_SPEED)):
            for _ in range(int(WHEEL_RUN_S / PERIOD_S)):
                if lidar:
                    sensed.record()
                left, right, _why = safety.vet_wheel_velocity(w, w)
                robot.set_wheel_velocity(left, right)
                before = (world.x, world.y)
                robot.advance(PERIOD_S)
                # Stopped in a static house on instant readings: every later
                # period is this one again (3.18's rule). A timed scan may
                # free it a period later, so the lidar run goes on.
                if not lidar and math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9:
                    break
    T1, G1, _ = fs.truth(world, direction)
    return {"house": house, "x": x, "y": y, "theta": theta, "dir": direction,
            "path": path, "lidar": lidar, "T0": T0, "G0": G0, "T_end": T1,
            "min_G": min([G0] + [g for _t, g, _p in samples]),
            "max_P": max([0.0] + [p for _t, _g, p in samples]),
            "travel": math.hypot(world.x - x0, world.y - y0) * fs.CELL_CM,
            "refused": refused}


def sweep(houses=HOUSES, starts_per_house=480, seed=0, path="verb", lidar=False):
    return [run(h, x, y, th, d, path, lidar)
            for h in houses for x, y, th, d in near_starts(h, starts_per_house, seed)]


def verdicts(results):
    """Criterion 2, plus the progress half (CLAUDE.md 7.4): creeping
    closes the gap, or the margin is not being used."""
    touched = [r for r in results if r["max_P"] > 0.0 or r["min_G"] <= 0.0]
    under = [r for r in results if r["min_G"] < min(r["G0"], G_BAR_CM) - 1e-6]
    ends = sorted(r["T_end"] for r in results)
    median_end = ends[len(ends) // 2] if ends else math.inf
    worst = min((r["min_G"] for r in results), default=math.inf)
    return {
        "2 creep never touches": (not touched,
            f"{len(touched)}/{len(results)} runs touched"),
        "2 creep keeps 2 cm": (not under,
            f"{len(under)}/{len(results)} runs came under {G_BAR_CM}cm "
            f"of true gap (closest {worst:.2f}cm)"),
        "progress: creep closes in": (median_end <= PROGRESS_MEDIAN_CM,
            f"median travel-to-contact at the end {median_end:.1f}cm "
            f"(bar {PROGRESS_MEDIAN_CM:g})"),
    }


def main(argv):
    logging.getLogger("safety").setLevel(logging.ERROR)   # one line per refusal
    n = int(argv[1]) if len(argv) > 1 else 480
    for path in ("verb", "wheels"):
        for lidar in (False, True):
            t = time.monotonic()
            res = sweep(starts_per_house=n, path=path, lidar=lidar)
            mode = "lidar" if lidar else "instant"
            print(f"{path:6s} {mode:7s} {len(res)} runs, {time.monotonic() - t:.0f}s")
            for name, (ok, detail) in verdicts(res).items():
                print(f"   {'PASS' if ok else 'FAIL'}  {name}: {detail}")
            sys.stdout.flush()


if __name__ == "__main__":
    main(sys.argv)
