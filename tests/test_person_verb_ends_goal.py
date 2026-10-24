"""3.97 (the user, 2026-10-24: "cancel in both modes"): a person's move
ends a nav2 goal, as a Stop does, under every drive. Under `drive: ros` the
bridge already cancelled on a person's twist (3.15); under `drive: direct`
with a ROS world a person's verb only won while it ran, and nav2 drove on
to the goal once their authority lapsed. In-process, against
tests/test_ros_goals.py's fake bridge, as tests/test_stop_cancels_goal.py."""

import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.ros_drive import STOP_HOLD_S
from tests.test_stop_cancels_goal import served  # noqa: F401 -- fixture


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.01)
    return cond()


@pytest.mark.parametrize("driver", ["twin-dpad", "teleop-operator", ""])
@pytest.mark.parametrize("verb", ["FORWARD", "LEFT"])
def test_a_persons_move_ends_the_goal(served, driver, verb):
    client, bridge, nav2_tick, wheel_speeds = served
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    headers = {"x-driver": driver} if driver else {}
    r = client.post("/action", json={"action": verb, "duration": 0.1, "angle": 15}, headers=headers)
    assert r.json().get("executed"), r.json()
    assert _wait(lambda: bridge.goal is None), "the goal outlived a person's move"
    # And nav2 does not drive on once the person lets go.
    time.sleep(STOP_HOLD_S + 0.1)
    nav2_tick()
    assert wheel_speeds() == (0.0, 0.0)


def test_a_persons_look_leaves_the_goal(served):
    """Panning the camera does not move the robot: not a take-over."""
    client, bridge, nav2_tick, wheel_speeds = served
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "twin-dpad"})
    time.sleep(0.3)
    assert bridge.goal is not None and bridge.goal["state"] == "active"


def test_a_persons_move_does_not_latch_the_brain_out(served):
    """A move is not a Stop: the brain is not refused `person_stop` after
    it (3.84's latch is a Stop's alone)."""
    client, bridge, nav2_tick, wheel_speeds = served
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/action", json={"action": "FORWARD", "duration": 0.1},
                headers={"x-driver": "twin-dpad"})
    r = client.post("/action", json={"action": "FORWARD", "duration": 0.1},
                    headers={"x-driver": "brain"}).json()
    assert r.get("reason") != "person_stop", r


def test_a_brain_move_leaves_the_goal(served):
    """Only a person's move ends a goal; the brain cannot drive while a
    goal holds the robot (3.23), and its refusal must not end it."""
    client, bridge, nav2_tick, wheel_speeds = served
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/action", json={"action": "FORWARD", "duration": 0.1},
                headers={"x-driver": "brain"})
    time.sleep(0.3)
    assert bridge.goal is not None and bridge.goal["state"] == "active"


def test_a_tap_after_a_move_still_drives(served):
    """The move's ending loop never cancels the person's next goal."""
    client, bridge, nav2_tick, wheel_speeds = served
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/action", json={"action": "FORWARD", "duration": 0.1},
                headers={"x-driver": "twin-dpad"})
    assert _wait(lambda: bridge.goal is None)
    r = client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["accepted"] is True, r
    time.sleep(1.0)
    assert bridge.goal is not None and bridge.goal["state"] == "active"


def test_a_move_with_no_goal_counts_no_stop(served):
    """A press with no goal running must not read as a Stop to a goal sent
    right after it (3.82's rule refuses a goal a stop overtook)."""
    client, bridge, nav2_tick, wheel_speeds = served
    client.post("/action", json={"action": "FORWARD", "duration": 0.1},
                headers={"x-driver": "twin-dpad"})
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0},
                    headers={"x-driver": "twin-dpad"}).json()     # the person's own tap
    assert r["accepted"] is True, r
    time.sleep(0.5)
    assert bridge.goal is not None and bridge.goal["state"] == "active"
