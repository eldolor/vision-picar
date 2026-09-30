"""
tests/test_ros_verb_safety.py

`PLAN-ros-alignment.md` 3.24, G2: the ROS verb path meets 3.18's and 3.19's
ground-truth bars, and reports a blocked verb as direct mode does. One test
per criterion, all confirmed red against the code before the fix, and each
fix mutation-checked against its own criterion (look-ahead -> 1, the ROS
refusal -> 3, the executor's settle tolerance -> 4).

The instrument is `tests/ros_verb_sweep.py`: the real `RosDriveRobot`
executor, entered as `/action` enters it, through a fake chain with R4's
measured timing and the robot server's own vet, on a virtual clock. The
full sweep runs once per module (~100 s): criterion 1's failure was one verb
in 1440 and criterion 4's margin is under three points, so a sample small
enough to be quick could not be trusted to catch a regression.
"""

import math

import pytest

from tests import footprint_sweep as fs
from tests import ros_verb_sweep as rv

HOUSES = ("starter_house", "scaled_house", "home_first_floor")


@pytest.fixture(scope="module")
def sweep():
    return rv.straight_sweep(HOUSES, 10), rv.pivot_sweep(HOUSES, 20)


def test_the_start_that_broke_the_stopping_distance():
    """The one verb of 1440 the unfixed ROS path drove under 18 cm: a
    full-speed REVERSE in the scaled house (17.93 cm with the look-ahead
    removed). Pinned on its own, so it fails fast and says where."""
    r = rv.run("scaled_house", 4.2323, 4.3963, 1.0472, "REVERSE", 100)
    assert r["worst_T_after_move"] >= fs.T_BAR_CM, r


def test_criterion_1_the_stopping_distance_holds_through_ros(sweep):
    ok, detail = rv.verdicts(*sweep)["1 stopping distance"]
    assert ok, detail


def test_criterion_2_no_contact_through_ros(sweep):
    ok, detail = rv.verdicts(*sweep)["2 no contact"]
    assert ok, detail


def test_criterion_3_a_verb_the_vet_held_is_a_refusal(sweep):
    ok, detail = rv.verdicts(*sweep)["3 a blocked verb is a refusal"]
    assert ok, detail


def test_criterion_4_clear_verbs_cover_their_move(sweep):
    ok, detail = rv.verdicts(*sweep)["4 progress"]
    assert ok, detail


def test_criterion_4_clear_turns_land_on_their_angle():
    """The turn half of criterion 4 -- measured separately because the
    sweep's pivots all start with something inside the turning circle. Red
    before the executor's floor rate and tolerance were cut (80%)."""
    ok, detail = rv.turn_verdict(rv.clear_turns())
    assert ok, detail


def test_a_clear_verb_through_ros_means_what_it_meant():
    """The progress half, at both speeds, where nothing is near: a FORWARD
    covers 30 cm within 3.13's 4.4 mm and a 45-degree turn its angle within
    0.64 degrees -- the executor's own accuracy, with the vet in the chain."""
    for speed in rv.SPEEDS:
        r = rv.run("scaled_house", 6.5, 4.5, 0.0, "FORWARD", speed, seed=3)
        assert abs(r["travel_cm"] - 30.0) <= 0.44, r
        assert r["refused"] is None
    r = rv.run("scaled_house", 6.5, 4.5, 0.0, "LEFT", seed=3)
    assert abs(r["turned_deg"] - rv.TURN_DEG) <= 0.64, r
