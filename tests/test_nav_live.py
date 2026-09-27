"""
tests/test_nav_live.py

Phase R6 (`PLAN-ros-alignment.md` 3.15), live: nav2 driving the simulated
robot to six goals through the whole chain, judged by GROUND TRUTH. One test
per criterion, over one run of `tests/demo_nav_goals.py` (a mapping lap, the
six goals, an unreachable goal, a D-pad tap mid-goal).

Measured on the SCALED house (`SIM_MAP=scaled_house`): 90 cm doors, because
the starter house's 30 cm doors leave the chosen chassis about 5 cm a side
and nav2 seals them whenever SLAM draws a jamb one cell thick (3.15).

Skips unless the robot server answers with a SLAM world and nav2 is up; it
drives the robot for several minutes.
"""

import os

import pytest

from tests import demo_nav_goals as nav


@pytest.fixture(scope="module")
def run():
    try:
        robot, bridge = nav.client(), nav.bridge()
        pose = robot.get("/world/pose").json()
        goal = robot.get("/world/goal")
    except Exception:  # noqa: BLE001
        pytest.skip("no robot server")
    if not str(pose.get("map_id") or "").startswith("slam-") or goal.status_code != 200:
        pytest.skip("no SLAM world with nav2 (WORLD_MODE=ros, ROBOT_DRIVE=ros)")
    if robot.get("/health").json().get("sim_map") != "scaled_house" or nav.HOUSE != "scaled_house":
        pytest.skip("R6 is judged on SIM_MAP=scaled_house (see the module docstring)")
    nav.map_first(robot)
    bridge.post("/nav/stats/reset", json={})
    before = dict(robot.get("/health").json().get("refusal_counts") or {})
    rows = [nav.run_goal(robot, x, y) for _, x, y in nav.GOALS]
    stats = bridge.get("/nav/stats").json()
    after = dict(robot.get("/health").json().get("refusal_counts") or {})
    return {"rows": rows, "stats": stats, "robot": robot, "bridge": bridge,
            "safety_refusals": after.get("safety_distance", 0) - before.get("safety_distance", 0)}


def test_goals_are_reached(run):
    ok = [r for r in run["rows"] if r["state"] == "succeeded"]
    assert len(ok) >= 5, run["rows"]
    assert all(r["end_error_m"] <= 0.20 for r in ok), [r["end_error_m"] for r in ok]


def test_the_chassis_never_touches_anything(run):
    assert min(r["min_clearance_m"] for r in run["rows"]) >= nav.HALF_WIDTH_M


def test_it_does_not_flicker(run):
    s = run["stats"]
    assert s["metres"] > 5.0, s
    assert s["reversals"] / s["metres"] <= 1.0, s


def test_an_unreachable_goal_ends_with_the_robot_stopped(run):
    r = nav.unreachable(run["robot"])
    assert r["state"] in ("aborted", "rejected"), r
    assert r["seconds"] <= 60 and r["wheels_stopped"], r


def test_a_person_outranks_the_plan(run):
    r = nav.preempt(run["robot"], run["bridge"])
    assert r["tap_executed"], r
    assert r["seconds_from_tap_sent_to_canceled"] is not None, r
    assert r["seconds_from_tap_sent_to_canceled"] <= 1.0, r
