"""
tests/test_review_376a.py

`PLAN-ros-alignment.md` 3.76, batch A: the bugs the whole-file review of
`robot/safety.py` and `robot/server.py` confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names.
"""

import threading
import time

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.interface import unusable_wheels
from robot.safety import SafetyController
from tests.conftest import asgi_client


def _keep_robot(monkeypatch):
    """create_app() builds its own robot; keep a handle on it."""
    real, built = server.get_robot, {}

    def keep(*a, **kw):
        built["robot"] = real(*a, **kw)
        return built["robot"]

    monkeypatch.setattr(server, "get_robot", keep)
    return built


def _wheels(robot):
    w = robot.get_wheel_state()
    return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]


# ---------- A1: a long verb keeps its driver's authority ----------


def test_a_person_verb_longer_than_the_watchdog_keeps_the_brain_out(monkeypatch):
    """A person's verb that runs longer than `watchdog_timeout_s` (a 90
    degree turn at 1.2 rad/s is 1.3 s on the car) held the robot only for the
    first second: `driver_at` was stamped at the start and never again, so
    the brain passed arbitration mid-verb, queued on the motion lock, took
    the driver slot and moved the moment the person's verb ended."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    built = _keep_robot(monkeypatch)
    app = server.create_app()
    robot = built["robot"]
    inside, finish = threading.Event(), threading.Event()

    def long_verb(*a, **kw):
        inside.set()
        finish.wait(5.0)
        return {"ok": True}

    monkeypatch.setattr(robot, "verb_plan", lambda *a, **kw: None, raising=False)
    monkeypatch.setattr(robot, "look_left", long_verb, raising=False)

    person, brain = TestClient(app), TestClient(app)
    replies = {}
    t = threading.Thread(target=lambda: replies.setdefault(
        "person", person.post("/action", json={"action": "LOOK_LEFT"},
                              headers={"x-driver": "twin-dpad"}).json()))
    t.start()
    assert inside.wait(5.0), "the person's verb never started"
    time.sleep(1.2)                                   # the verb runs past the 1.0 s watchdog timeout
    replies["brain"] = brain.post("/action", json={"action": "LOOK_RIGHT"},
                                  headers={"x-driver": "brain"}).json()
    finish.set()
    t.join()
    assert replies["person"]["executed"] is True, replies
    assert replies["brain"]["executed"] is False, replies["brain"]
    assert replies["brain"]["reason"] == "preempted", replies["brain"]


# ---------- A2: the wheel loop never writes over a stop ----------


def test_a_stop_during_the_wheel_loops_vet_is_not_overwritten(monkeypatch):
    """The wheel loop read a standing command, vetted it (a scan: ms), and
    wrote the SLOWED command back. `/stop` does not take the motion lock, so
    a stop landing during the vet was overwritten and the robot drove on
    until the watchdog -- the race `carry_out_verb` already guards with
    `stop_count` (handoff 2c)."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    built = _keep_robot(monkeypatch)
    client = asgi_client(server.create_app())
    real_vet = SafetyController.vet_wheel_velocity
    armed = {"on": False, "fired": 0}

    def vet_with_a_stop_landing(self, left, right):
        if armed["on"]:
            armed["on"] = False
            armed["fired"] += 1
            self.robot.stop()                          # Stop pressed mid-vet
            return left * 0.5, right * 0.5, None       # slowed, not clamped
        return real_vet(self, left, right)

    monkeypatch.setattr(SafetyController, "vet_wheel_velocity", vet_with_a_stop_landing)
    with client:
        robot = built["robot"]
        r = client.post("/wheels", json={"left_rad_s": 1.0, "right_rad_s": 1.0},
                        headers={"x-driver": "ros"}).json()
        assert r["executed"] is True and _wheels(robot) != (0.0, 0.0), r
        armed["on"] = True
        deadline = time.monotonic() + 1.0
        while armed["fired"] == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        time.sleep(0.15)                               # the tick finishes
        assert armed["fired"] == 1, "the wheel loop never vetted -- the test proves nothing"
        assert _wheels(robot) == (0.0, 0.0), "the wheel loop wrote over the stop"


# ---------- A5: lost wheel feedback under drive: ros is named as such ----------


def test_lost_wheel_feedback_under_drive_ros_is_no_feedback_not_ros_unavailable(monkeypatch):
    """`WheelFeedbackLost` subclasses RuntimeError, so drive: ros's
    `except (httpx.HTTPError, RuntimeError)` caught it first: a lost encoder
    was reported as `ros_unavailable` ("the bridge failed"), and counted so
    in /health -- the opposite fault to look for."""
    from tests.test_ros_fallback import Stack

    stack = Stack(monkeypatch)
    try:
        stack.ros_up()
        time.sleep(0.2)
        monkeypatch.setattr(stack.inner, "get_wheel_state", unusable_wheels)
        stops0 = stack.inner.stop_count
        reply = stack.client.post("/action", json={"action": "FORWARD"},
                                  headers={"x-driver": "twin-dpad"}).json()
        assert reply["executed"] is False, reply
        assert reply["reason"] == "no_feedback", reply
        assert stack.inner.stop_count > stops0, "refused without stopping the robot"
    finally:
        stack.ros_down()
        stack.client.__exit__(None, None, None)


# ---------- A8: a stop never waits on the ROS bridge ----------


def test_a_second_stop_answers_while_the_bridge_hangs(monkeypatch):
    """`/stop` took `goal_stop_lock`, which the previous stop's goal-ending
    thread holds across bridge calls (2 s timeouts each). With the bridge
    hung, a second Stop zeroed the motors but its reply waited on the
    bridge -- long enough for the twin's request to time out and read as a
    failed stop."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    release = threading.Event()

    class HangingWorld:
        def get_goal(self):
            release.wait(5.0)                          # a hung bridge
            raise httpx.ReadTimeout("bridge hung")

        def cancel_goal(self):
            return {"cancelled": True}

    monkeypatch.setattr(server, "get_world", lambda *a, **kw: HangingWorld())
    client = TestClient(server.create_app())
    try:
        assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
        time.sleep(0.1)                                # the ending thread is in get_goal()
        t0 = time.monotonic()
        assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
        assert time.monotonic() - t0 < 0.5, "the second stop waited on the bridge"
    finally:
        release.set()


# ---------- T1: a superseded ROS verb does not stop the verb that superseded it ----------


def test_a_person_taking_over_a_ros_verb_is_not_stopped_by_the_verb_it_replaced(monkeypatch):
    """Under drive: ros `/action` holds no lock, so a person's verb
    supersedes the brain's in flight. The brain's verb, cut off before ROS's
    latency let it move, was judged "achieved nothing" -- a refusal -- and
    that refusal's `robot.stop()` superseded the PERSON's verb too and held
    its twists at zero. Both came back `safety_distance`."""
    from tests.test_ros_fallback import Stack

    stack = Stack(monkeypatch)
    try:
        stack.ros_up()
        time.sleep(0.2)
        replies = {}
        brain = threading.Thread(target=lambda: replies.setdefault(
            "brain", stack.client.post("/action", json={"action": "LEFT", "angle": 90},
                                       headers={"x-driver": "brain"}).json()))
        brain.start()
        time.sleep(0.03)                               # before ROS has moved it
        replies["person"] = stack.client.post(
            "/action", json={"action": "RIGHT", "angle": 45},
            headers={"x-driver": "twin-dpad"}).json()
        brain.join()
        person = replies["person"]
        assert person["executed"] is True, replies
        assert abs(person["result"]["turned_deg"]) > 30, replies
        # Cut off by the take-over: preempted, never a safety refusal.
        assert replies["brain"].get("reason") == "preempted", replies
    finally:
        stack.ros_down()
        stack.client.__exit__(None, None, None)
