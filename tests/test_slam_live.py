"""
tests/test_slam_live.py

Phase R5 (`PLAN-ros-alignment.md` 3.14), live: one lap through the real
stack, asserting what held on EVERY one of the eighteen laps measured on
2026-09-26 -- the map is the house, SLAM ends where the robot truly is, and
SLAM's position stays inside 10 cm throughout. Criteria 2 and 3's tighter
bars (5 cm / 2 degrees without drift, 3 degrees with it) did NOT hold and are
recorded in the plan with their distribution, not asserted here.

Skips unless the robot server answers with a SLAM world (WORLD_MODE=ros)
and a secret is available; see tests/demo_slam_lap.py for how to bring the
stack up. It drives the robot a lap.
"""

import os

import pytest

from tests.demo_slam_lap import client, run_lap, score_map


@pytest.fixture(scope="module")
def lap():
    try:
        robot = client()
        pose = robot.get("/world/pose").json()
    except Exception:  # noqa: BLE001
        pytest.skip("no robot server")
    if not str(pose.get("map_id") or "").startswith("slam-"):
        pytest.skip("the robot server's world is not SLAM (WORLD_MODE=ros)")
    samples, blocked = run_lap(robot)
    return robot, samples, run_lap.rest, blocked


def test_the_map_is_the_house(lap):
    robot, *_ = lap
    m = score_map(robot)
    assert m["precision"] >= 0.90, m
    assert m["free_bad_rate"] <= 0.01, m


def test_slam_ends_where_the_robot_truly_is(lap):
    _, _, rest, _ = lap
    assert rest[-1][0] <= 0.10, rest[-1]


def test_slams_position_stays_inside_ten_centimetres_at_rest(lap):
    _, _, rest, _ = lap
    assert max(r[0] for r in rest) <= 0.10, [round(r[0], 3) for r in rest]
