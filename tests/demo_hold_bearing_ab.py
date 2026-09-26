"""P25's A/B, answered -- and the answer was about turn SIZE, not memory.

    python -m tests.demo_hold_bearing_ab

P25 asked whether dead-reckoning a bearing between detections stops the
tier's command changing every frame. This harness could not answer while
the sim turned in 90-degree quanta; R0 made the pose continuous, and the
first honest run showed the premise was incomplete: every LEFT/RIGHT the
brain sent was STILL the executor's default 90 degrees, which against a
10-degree centre band overshoots any target inside an 80-degree cone. So
R1 (`PLAN-ros-alignment.md`) made a turn chosen from a bearing turn BY that
bearing, and this now prints the comparison that justified it.

Everything runs through the WHOLE mission path -- `MissionRunner` ->
`VisionAgent` -> `robot/safety.py`'s collar -> `MockRobot` -- because the
defect lived in the seam between the tier and the executor. Perception is
`FrameReportedPipeline`, 1.12's synthetic detections, optionally withheld on
a schedule to model a detector that lands only some frames. Scored on
distance closed and turn reversals, never `median_command_run` alone.

Measured 2026-09-25, twelve clear-line starts plus four whose line clips
the door jamb (see `tests/test_bearing_turns.py`):

    sized turns     closed ~4.1 cells, ~1 reversal, all clear starts arrive
    quarter turns   ends FURTHER away than it started, ~5 reversals

Dead-reckoning (`tier_hold_bearing`) is worth little once turns are sized:
each sighting fully corrects the heading, so there is less for memory to
bridge. It stays OFF.
"""

import logging
import math
import statistics

from brain.perceive import ABSENT, FrameReportedPipeline, Perception
from brain.tiered import TieredVision
from control.mission_runner import MissionRunner
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import (
    GOAL, STARTS, STEPS, TARGET, _DropTurnSize, _quiet_cloud)


class _Intermittent(FrameReportedPipeline):
    """A detector that lands one frame in `every` -- P25's actual premise."""

    def __init__(self, target, every):
        super().__init__(target)
        self.every, self.i = every, -1

    def perceive(self, frame):
        self.i += 1
        if self.i % self.every:
            return Perception(status=ABSENT, synthesised=True)
        return super().perceive(frame)


def run(start, *, sized, every, hold):
    x, y, off = start
    grid = build_starter_world()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(_Intermittent(TARGET, every), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True,
                        hold_bearing=hold, hold_bearing_max_m=1.0)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=STEPS,
                           policy="tiered",
                           vision_fn=tier if sized else _DropTurnSize(tier),
                           world=mock_world_for(robot))
    before = math.dist((grid.x, grid.y), GOAL)
    runner.start()
    while runner.tick():
        pass
    return before - math.dist((grid.x, grid.y), GOAL), runner.status()["turns"]["reversals"]


def main():
    logging.disable(logging.WARNING)  # the collar's vetoes are expected here
    print(f"{len(STARTS)} starts, {STEPS} steps, target {TARGET!r}\n")
    print(f"{'turns':>8} {'detector':>9} {'hold':>5} {'closed':>7} {'reversals':>10}")
    for every in (1, 3):
        for sized in (True, False):
            for hold in (False, True):
                rs = [run(s, sized=sized, every=every, hold=hold) for s in STARTS]
                print(f"{'sized' if sized else 'quarter':>8} "
                      f"{'1 in ' + str(every):>9} {str(hold):>5} "
                      f"{statistics.mean(r[0] for r in rs):>7.2f} "
                      f"{statistics.mean(r[1] for r in rs):>10.1f}")


if __name__ == "__main__":
    main()
