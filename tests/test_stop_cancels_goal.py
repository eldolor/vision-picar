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


# ---------------------------------------------------------------------------
# Spec review 3 (V1), rule decided by the user 2026-10-03: a stop from an
# autonomous driver other than `ros` zeroes the wheels but SPARES a goal in
# progress. While a goal holds the robot the brain cannot drive (3.23), so
# its stop can only be a loser's teardown -- and before this rule the
# preempted mission's exit stop cancelled the very goal that won.

def _wait_settled():
    time.sleep(0.3)          # long enough for a background cancel to land


def test_a_brain_stop_spares_a_goal_in_progress(served):
    client, bridge, _, _ = served
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    refused = client.post("/action", json={"action": "FORWARD"},
                          headers={"x-driver": "brain"}).json()
    assert refused["reason"] == "preempted", refused
    assert client.post("/stop", headers={"x-driver": "brain"}).json()["executed"] is True
    _wait_settled()
    assert bridge.goal is not None and bridge.goal["state"] == "active"
    assert bridge.cancelled == 0, "the loser's stop cancelled the goal that beat it"


@pytest.mark.parametrize("driver", ["twin-dpad", None, "teleop-operator"])
def test_a_person_stop_still_ends_the_goal(served, driver):
    client, bridge, _, _ = served
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/stop", headers={"x-driver": driver} if driver else {})
    _wait_settled()
    assert bridge.goal is None and bridge.cancelled == 1


def test_a_mission_preempted_by_a_goal_leaves_the_goal_running(monkeypatch):
    """The whole path: MissionRunner over RemoteRobot, refused by the goal,
    finishes `preempted` and stops the robot on its way out."""
    from control.mission_runner import MissionRunner
    from control.remote_robot import RemoteRobot
    from tests.conftest import ASGI_BASE_URL, asgi_client

    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = asgi_client(server.create_app())
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]

    runner = MissionRunner(RemoteRobot(ASGI_BASE_URL, client=client),
                           target_object="red backpack", max_steps=20)
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["outcome"] == "preempted", runner.status()
    _wait_settled()
    assert bridge.goal is not None and bridge.goal["state"] == "active"
    assert bridge.cancelled == 0


# ---------------------------------------------------------------------------
# Spec review 3 (V2): "the wheels stop and the goal is ended" must hold
# BETWEEN the stop and the cancel landing, when the cancel fails, and for a
# goal nav2 has not yet accepted. The fake nav2 ticks right after the stop
# here -- the first version of this file only ticked after STOP_HOLD_S, by
# which time the fast fake cancel had always landed.

import httpx  # noqa: E402

from tests.test_ros_goals import GoalBridge  # noqa: E402


class SlowBridge(GoalBridge):
    """A bridge with the real one's awkward cases, switchable per test."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.cancel_delay_s = 0.0
        self.cancel_fails = False
        self.accept_on_post = True        # False: a goal waits `pending` for accept()

    def handler(self, request):
        if request.url.path == "/goal" and request.method == "DELETE":
            if self.cancel_fails:
                raise httpx.ConnectError("bridge unreachable", request=request)
            time.sleep(self.cancel_delay_s)
            if self.goal and self.goal["state"] == "pending":
                # The real bridge (before its own fix) cancelled nothing here.
                self.cancelled += 1
                return httpx.Response(200, json={"cancelled": False})
        reply = super().handler(request)
        if request.url.path == "/goal" and request.method == "POST" and not self.accept_on_post:
            self.goal["state"] = "pending"
        return reply

    def accept(self):
        """nav2 accepts the pending goal."""
        if self.goal and self.goal["state"] == "pending":
            self.goal["state"] = "active"


@pytest.fixture
def slow(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    bridge = SlowBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = TestClient(server.create_app())

    def nav2_tick():
        if bridge.goal and bridge.goal.get("state") == "active":
            client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                        headers={"x-driver": "ros"})

    def wheel_speeds():
        w = client.get("/wheels").json()
        return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]

    def drive_for(seconds):
        """nav2 ticking at 20 Hz; the fastest wheel speed seen meanwhile."""
        worst, end = 0.0, time.monotonic() + seconds
        while time.monotonic() < end:
            nav2_tick()
            worst = max(worst, *map(abs, wheel_speeds()))
            time.sleep(0.05)
        return worst

    return client, bridge, nav2_tick, wheel_speeds, drive_for


def test_the_wheels_stay_stopped_while_the_cancel_is_in_flight(slow):
    client, bridge, nav2_tick, wheel_speeds, drive_for = slow
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    nav2_tick()
    assert wheel_speeds() != (0.0, 0.0)
    bridge.cancel_delay_s = 0.3
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    assert drive_for(STOP_HOLD_S + 0.5) == 0.0, "nav2 drove before the cancel landed"
    assert bridge.goal is None


def test_a_failed_cancel_keeps_the_wheels_stopped_and_is_retried(slow):
    client, bridge, nav2_tick, wheel_speeds, drive_for = slow
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    bridge.cancel_fails = True
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    assert drive_for(1.5) == 0.0, "nav2 drove on after a failed cancel"
    assert bridge.goal is not None, "the fake cancelled anyway -- the test proves nothing"
    bridge.cancel_fails = False                   # the bridge comes back
    deadline = time.monotonic() + 3.0
    while bridge.goal is not None and time.monotonic() < deadline:
        time.sleep(0.05)
    assert bridge.goal is None, "the cancel was never retried"


def test_a_pending_goal_is_ended_once_nav2_accepts_it(slow):
    client, bridge, nav2_tick, wheel_speeds, drive_for = slow
    bridge.accept_on_post = False
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    assert bridge.goal["state"] == "pending"
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    time.sleep(0.2)
    bridge.accept()                               # nav2 accepts after the stop
    assert drive_for(STOP_HOLD_S + 1.0) == 0.0, "the pending goal drove once accepted"
    assert bridge.goal is None, "the accepted goal was never cancelled"


def test_a_new_goal_after_a_stop_drives(slow):
    client, bridge, nav2_tick, wheel_speeds, drive_for = slow
    client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    time.sleep(0.1)
    assert client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()["accepted"]
    assert drive_for(0.3) > 0.0, "a person's new goal was held at zero"
