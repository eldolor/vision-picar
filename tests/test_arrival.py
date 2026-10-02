"""
tests/test_arrival.py

P7e's first half (`PLAN-ros-alignment.md` 3.11) -- a mission that reaches its
target ends `found`. One test per acceptance criterion, written into the plan
before the rule was built.

Before this, every tiered mission that reached the backpack was labelled
`blocked` or `max_steps`: the trace that motivated it arrived at step 6 and
pushed FORWARD into the collar until step 10. Measured through the whole
mission path -- `MissionRunner` -> `VisionAgent` (whose `_review_scene()`
applies the rule) -> `robot/safety.py` -> `MockRobot` -- with the same starts
and the same flaky detector R1b/R1c were measured on, so "arrived" means
exactly what it meant there. Ground truth is read HERE, by the test, and
never by the rule.
"""

import logging
import math

import pytest

from brain.arrival import (
    APPROACHING, ARRIVAL_CENTRE_DEG, ARRIVED, NOT_JUDGED, ArrivalCheck)
from brain.perceive import FrameReportedPipeline
from brain.tiered import STEER_BAND_DEG, TieredVision
from control.mission_runner import FOUND, MissionRunner
from robot.interface import unusable_scan
from sim.mock_robot import DEFAULT_CELL_M, MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import (
    CLEAR_STARTS, GOAL, JAMB_STARTS, SEARCH_OFFSETS, SEARCH_START, TARGET, _Flaky,
    _build,
    _quiet_cloud)

from tests.test_bearing_turns import ARRIVED_CELLS  # noqa: E402 -- corrected 3.31, see there
FALSE_ARRIVAL_M = 0.60        # criterion 2
SWEEP = CLEAR_STARTS + [(*SEARCH_START, o) for o in SEARCH_OFFSETS]


@pytest.fixture(autouse=True)
def _quiet_logs():
    # Every stopped mission logs its refused FORWARDs; thousands of them
    # bury a failure's own output.
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def _run(start, pipeline, *, robot_wrapper=None):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    body = robot_wrapper(robot) if robot_wrapper else robot
    tier = TieredVision(pipeline, _quiet_cloud, steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(body, target_object=TARGET, max_steps=60,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    cells = math.dist((grid.x, grid.y), GOAL)
    return runner.status()["outcome"], cells


def _sweep(detection_p, seeds):
    rows = []
    for start in SWEEP:
        for seed in seeds:
            pipeline = (FrameReportedPipeline(TARGET) if detection_p == 1.0
                        else _Flaky(TARGET, detection_p, seed))
            rows.append(_run(start, pipeline))
    return rows


def _assert_recognised(rows, label):
    arrived = [r for r in rows if r[1] <= ARRIVED_CELLS]
    found = [r for r in arrived if r[0] == FOUND]
    assert arrived, f"{label}: nothing arrived at all"
    assert len(found) / len(arrived) >= 0.95, (
        f"{label}: {len(found)}/{len(arrived)} arrivals ended found")
    false = [r for r in rows if r[0] == FOUND and r[1] * DEFAULT_CELL_M > FALSE_ARRIVAL_M]
    assert not false, f"{label}: found from {[round(r[1], 2) for r in false]} cells"


# ---------- criteria 1 + 2: recognition, and never a false arrival ----------

def test_every_arrival_at_perfect_detection_ends_found():
    _assert_recognised(_sweep(1.0, [0]), "perfect detection")


@pytest.mark.parametrize("p", [0.9, 0.8])
def test_arrival_is_recognised_under_an_unreliable_detector(p):
    _assert_recognised(_sweep(p, range(10)), f"{p:.0%} detection")


def test_jamb_starts_never_claim_an_arrival_they_did_not_make():
    """The straight line clips the door jamb: the robot stops against the
    wall with the target in view and nearly centred, and something IS within
    the radius -- the jamb. The false arrival the rule must not make."""
    rows = [_run(s, FrameReportedPipeline(TARGET)) for s in JAMB_STARTS]
    false = [r for r in rows if r[0] == FOUND and r[1] * DEFAULT_CELL_M > FALSE_ARRIVAL_M]
    assert not false, rows


# ---------- criterion 4: refuse to judge rather than guess ----------

class _NoScan:
    """The body with its lidar removed -- teleop and replay look like this."""

    def __init__(self, robot):
        self._robot = robot

    def __getattr__(self, name):
        return getattr(self._robot, name)

    def get_scan(self, max_range_m=None):
        return unusable_scan()


def test_no_range_sensor_never_ends_found_locally():
    outcome, cells = _run(CLEAR_STARTS[0], FrameReportedPipeline(TARGET),
                          robot_wrapper=_NoScan)
    assert cells <= ARRIVED_CELLS, "it still gets there"
    assert outcome != FOUND, "but nothing could measure that it had"


class _FakeRobot:
    def __init__(self, ranges):
        self.ranges = ranges

    def get_scan(self, max_range_m=None):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.0, "range_max_m": 4.2, "ranges_m": self.ranges}


def _scene(bearing, pan=0.0, status="detected"):
    return {"_perception": {"status": status, "bearing_deg": bearing, "pan_deg": pan}}


def test_a_panned_camera_is_not_judged():
    check = ArrivalCheck()
    robot = _FakeRobot([0.2] * 360)
    for _ in range(3):
        assert check.observe(_scene(0.0, pan=30.0), robot)["state"] == NOT_JUDGED


def test_a_policy_without_local_perception_is_not_judged():
    assert ArrivalCheck().observe({}, _FakeRobot([0.2] * 360))["state"] == NOT_JUDGED


# ---------- the rule's own edges ----------

def test_two_frames_are_needed_and_a_miss_resets_the_streak():
    check, robot = ArrivalCheck(), _FakeRobot([0.2] * 360)
    assert check.observe(_scene(0.0), robot)["state"] == APPROACHING
    assert check.observe(_scene(0.0, status="absent"), robot)["state"] == APPROACHING
    assert check.observe(_scene(0.0), robot)["state"] == APPROACHING
    assert check.observe(_scene(0.0), robot)["state"] == ARRIVED


def test_off_centre_or_out_of_reach_never_arrives():
    check = ArrivalCheck()
    near, far = _FakeRobot([0.2] * 360), _FakeRobot([0.5] * 360)
    for _ in range(3):
        assert check.observe(_scene(ARRIVAL_CENTRE_DEG + 1), near)["state"] == APPROACHING
        assert check.observe(_scene(0.0), far)["state"] == APPROACHING


def test_the_range_is_read_at_the_bearing_not_dead_ahead():
    """Clockwise-positive, like a perception bearing: a target 2 degrees
    right is read from the beams 0-4 degrees right, and an obstacle dead
    ahead does not stand in for it."""
    ranges = [3.0] * 360
    ranges[180] = 0.1           # dead ahead: something else, close
    check = ArrivalCheck()
    for _ in range(3):
        assert check.observe(_scene(-2.5), _FakeRobot(ranges))["state"] != ARRIVED
    ranges = [3.0] * 360
    ranges[180 + 1:180 + 6] = [0.3] * 5   # an object 1-5 degrees right
    check = ArrivalCheck()
    check.observe(_scene(2.5), _FakeRobot(ranges))
    assert check.observe(_scene(2.5), _FakeRobot(ranges))["state"] == ARRIVED


def test_an_edge_beside_the_target_is_not_the_target():
    """Criterion 2's regression, from the sweep that found it: stuck on a
    door jamb 95 cm out, target dead centre, the jamb's edge 1-5 degrees to
    the left at 0.195 m. The nearest return in the window was the jamb; the
    target is the median."""
    ranges = [3.0] * 360
    ranges[175:180] = [0.195] * 5
    ranges[180:186] = [0.81] * 6
    check = ArrivalCheck()
    for _ in range(3):
        assert check.observe(_scene(0.13), _FakeRobot(ranges))["state"] != ARRIVED


def test_a_jamb_edge_straddling_the_window_is_not_the_target():
    """3.32 criterion 1, from the sweep that found it once missions used
    guarded verbs: stopped 37 cm from a door jamb, the target visible
    through the doorway at 0.4 deg, and the five-beam window -- 3 cm wide
    at that range -- three beams on the jamb and two past it. The median
    was the jamb, and the mission ended found 1.1 m short."""
    ranges = [3.0] * 360
    # Lidar ranges; the rule measures from base_link, 4 cm behind the lidar
    # (3.27), so 0.33 here is the 0.37 m the mission read.
    ranges[178:181] = [0.33] * 3        # the jamb, on one side of the bearing
    ranges[181:183] = [1.08] * 2        # through the doorway, the target
    check = ArrivalCheck()
    for _ in range(3):
        assert check.observe(_scene(0.4), _FakeRobot(ranges))["state"] != ARRIVED


def test_a_face_that_fills_the_window_still_arrives():
    """The edge rule must not cost a real arrival: a target face at 35 cm
    reads within a few millimetres across the window."""
    ranges = [3.0] * 360
    ranges[177:184] = [0.352, 0.351, 0.350, 0.350, 0.350, 0.351, 0.352]
    check = ArrivalCheck()
    states = [check.observe(_scene(0.0), _FakeRobot(ranges))["state"] for _ in range(2)]
    assert states[-1] == ARRIVED, states


def test_the_centre_band_is_the_tiers_steering_band():
    assert ARRIVAL_CENTRE_DEG == STEER_BAND_DEG
