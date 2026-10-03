"""
tests/test_stop_cancels_goal.py

Handoff 2026-10-02 1b (decided by the user): a stop ends a nav2 goal.

Before this, `POST /stop` only called `robot.stop()`, and the bridge cancels
a goal only on a non-zero `twin-dpad` twist -- so nav2 kept its goal and
drove again the moment the stop hold ended (under `drive: direct` with
`WORLD_MODE=ros`, within ~50 ms). Now `/stop` also cancels any active goal,
on a background thread after `robot.stop()`, and a person re-sends a goal to
resume.

In-process, against tests/test_ros_goals.py's fake bridge. nav2 is modelled
by what it does with an active goal: post a non-zero wheel command as driver
`ros`, the way `picar_sim_hardware` relays `diff_drive_controller`'s output.
"""

import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.ros_drive import STOP_HOLD_S
from tests.test_ros_goals import world_with
from tests.test_ros_world import FakeTruth


@pytest.fixture
def served(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = TestClient(server.create_app())

    def nav2_tick():
        """What nav2 does each period: drive while it holds an active goal."""
        if bridge.goal and bridge.goal.get("state") == "active":
            client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                        headers={"x-driver": "ros"})

    def wheel_speeds():
        w = client.get("/wheels").json()
        return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]

    return client, bridge, nav2_tick, wheel_speeds


def test_the_wheels_stay_at_zero_past_the_stop_hold(served):
    client, bridge, nav2_tick, wheel_speeds = served
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    nav2_tick()
    assert wheel_speeds() != (0.0, 0.0), "the fake nav2 never drove -- the test proves nothing"

    assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
    time.sleep(STOP_HOLD_S + 0.1)
    nav2_tick()

    assert wheel_speeds() == (0.0, 0.0), "nav2 resumed its goal after the stop"
    assert bridge.goal is None and bridge.cancelled == 1


def test_a_new_goal_can_be_set_after_a_stop(served):
    client, bridge, nav2_tick, wheel_speeds = served
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/stop")
    deadline = time.monotonic() + 2.0
    while bridge.goal is not None and time.monotonic() < deadline:
        time.sleep(0.01)
    r = client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()
    assert r["accepted"] is True and bridge.goal["state"] == "active"


def test_a_stop_never_waits_on_the_bridge(monkeypatch):
    """The cancel is on a background thread: a hung bridge costs the stop
    nothing."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    world.cancel_goal = lambda: time.sleep(2.0)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = TestClient(server.create_app())
    started = time.monotonic()
    assert client.post("/stop").json()["executed"] is True
    assert time.monotonic() - started < 0.5


def test_a_world_that_cannot_plan_still_stops(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = TestClient(server.create_app())
    assert client.post("/stop").json()["executed"] is True
