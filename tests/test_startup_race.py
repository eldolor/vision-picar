"""
tests/test_startup_race.py

Handoff 2026-10-02, item 2a: the wheel plugin's start-up race.

If the ROS container activated before `HardwareRobot` had read the motor
board's first feedback frame, `GET /wheels` answered `usable: false`, the
plugin's `on_activate()` took that for "this body has no wheels" (a phone
walk, a replay) and refused -- and `ros2_control` never retried, so the
container had to be started after the first frame. Now a body that HAS
wheels but cannot measure them yet says `awaiting_feedback: true`, and the
plugin activates and waits.

The offline half pins the robot server's side of that contract. The live
half starts a real container against a board that stays silent for a while
(`SIM_BOARD_SILENT_S`) and skips unless that stack is up.
"""

import os
import time

import httpx
import pytest

from robot.hardware_robot import HardwareRobot
from robot.interface import unusable_wheels
from sim.fake_esp32 import FakeEsp32
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot
from sim.teleop_robot import TeleopRobot


def _wait(cond, timeout=3.0, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def test_a_board_before_its_first_frame_is_awaiting_feedback(monkeypatch):
    monkeypatch.setenv("SIM_BOARD_SILENT_S", "0.5")
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    try:
        before = robot.get_wheel_state()
        assert before["usable"] is False
        assert before.get("awaiting_feedback") is True, before
        assert _wait(lambda: robot.get_wheel_state()["usable"]), "no first frame"
        assert "awaiting_feedback" not in robot.get_wheel_state()
    finally:
        robot.close()
        board.close()


def test_a_body_with_no_wheels_is_not_awaiting_anything():
    """A phone walk must still read as 'no wheels', or the plugin would come
    up against a body it can never drive."""
    assert "awaiting_feedback" not in unusable_wheels()
    assert not TeleopRobot().get_wheel_state().get("awaiting_feedback")


# ---------- live: a container started before the board's first frame ----------

ROBOT_URL = os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000")


def _live_stack():
    secret = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET") or ""
    try:
        h = httpx.get(ROBOT_URL + "/health", headers={"x-app-secret": secret}, timeout=1.0).json()
    except Exception:  # noqa: BLE001
        return None
    if (h.get("drive") or {}).get("mode") != "ros" or h.get("mode") != "hardware":
        return None
    return secret


@pytest.mark.skipif(_live_stack() is None,
                    reason="needs a drive: ros robot server on the fake board, with "
                           "SIM_BOARD_SILENT_S set and the container started inside that window")
def test_live_a_container_started_first_drives_once_frames_arrive():
    secret = _live_stack()
    headers = {"x-app-secret": secret, "x-driver": "twin-dpad"}
    assert _wait(lambda: httpx.get(ROBOT_URL + "/health", headers=headers, timeout=2.0)
                 .json()["drive"].get("ros_up"), timeout=60.0), "ROS never came up"
    before = httpx.get(ROBOT_URL + "/odometry", headers=headers, timeout=2.0).json()
    reply = httpx.post(ROBOT_URL + "/action", json={"action": "LEFT", "angle": 45},
                       headers=headers, timeout=30.0).json()
    assert reply.get("executed") is True and reply["result"].get("via") == "ros", reply
    after = httpx.get(ROBOT_URL + "/odometry", headers=headers, timeout=2.0).json()
    turned = (before["heading_deg"] - after["heading_deg"]) % 360.0
    assert 40.0 <= turned <= 50.0, f"turned {turned:.1f} deg for a 45-degree LEFT"


# ---------- handoff 2b: a wheel command before the first feedback frame ----------
# Closed by 3.34 (the body refuses to move wheels it cannot measure); pinned
# here in the handoff's own terms: refused before the first frame, vetted as
# usual after it.

def test_a_wheel_command_before_the_first_frame_is_refused_and_after_it_vetted(monkeypatch):
    from fastapi.testclient import TestClient

    import robot.server as server

    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    monkeypatch.setenv("SIM_MOTOR_BOARD", "fake")
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("SIM_BOARD_SILENT_S", "1.0")
    with TestClient(server.create_app()) as client:
        cmd = {"left_rad_s": 3.0, "right_rad_s": 3.0}
        hdr = {"x-driver": "twin-dpad"}
        before = client.post("/wheels", json=cmd, headers=hdr).json()
        assert before["executed"] is False and before["reason"] == "no_feedback", before
        assert _wait(lambda: client.get("/wheels").json()["usable"]), "no first frame"
        after = client.post("/wheels", json=cmd, headers=hdr).json()
        assert after["executed"] is True and "clamped" in after, after
        client.post("/stop", headers=hdr)
