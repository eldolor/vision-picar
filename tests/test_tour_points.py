"""
tests/test_tour_points.py -- PLAN-ros-alignment.md 3.55 criterion 1.

Every point of the furnished-home tour (tests/demo_nav_goals.HOME_GOALS) must
be somewhere the robot can stand and leave, on ground truth: the chassis
overlaps nothing at any heading, and at every heading forward or reverse is
allowed by the real safety layer. Goal 6's old point (31.0, 30.0) ft overlapped
the dining furniture, so the tour failed it in every run since 3.40 and its
"8 of 9" meant "every other goal".
"""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture(scope="module")
def tour():
    from tests import demo_nav_goals as ng     # HOME_GOALS does not read SIM_MAP
    spec = importlib.util.spec_from_file_location(
        "slam345_goal_sweep", os.path.join(HERE, "..", "evaluations", "slam-345", "goal_sweep.py"))
    gs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gs)
    return ng, gs


def test_every_tour_point_can_be_stood_on_and_left(tour):
    ng, gs = tour
    bad = {}
    for name, x, y in ng.HOME_GOALS:
        ok, kinds = gs.leavable(x, y)
        if not ok or "overlaps" in kinds:
            bad[name] = kinds
    assert not bad, bad


def test_the_old_dining_room_point_could_not(tour):
    ng, gs = tour
    ok, kinds = gs.leavable(*ng._home_m(31.0, 30.0))
    assert "overlaps" in kinds          # the defect this section removed
