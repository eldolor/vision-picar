"""
Run with: pytest tests/test_teleop_robot.py -v

Covers Phase T1 of PLAN-teleop-robot.md: TeleopRobot behaves like a
RobotInterface with a live camera and no motor.
"""

import time

import pytest

from robot.safety import SafetyController
from sim.teleop_robot import NO_SENSOR_CM, TeleopRobot, TeleopStall


def test_get_camera_frame_returns_pushed_frame():
    robot = TeleopRobot()
    robot.push_frame("BASE64DATA", media_type="image/jpeg")

    frame = robot.get_camera_frame()

    assert frame["image_base64"] == "BASE64DATA"
    assert frame["media_type"] == "image/jpeg"
    # MissionMemory reads this; a live phone frame carries no room label.
    assert frame["room"] == "unknown"


def test_get_camera_frame_is_a_repeatable_read_not_a_single_use_cursor():
    """The bug this guards against shipped to production: a shared
    "consumed" cursor meant any second reader -- Settings' connect() health
    check, a passive observer, anything besides an active mission tick --
    blocked for the full stall_timeout_s and then failed, even seconds
    after a perfectly fresh push. Every read of a still-fresh frame must
    return immediately, however many times it's read."""
    robot = TeleopRobot(stall_timeout_s=5.0)
    robot.push_frame("FIRST")

    first = robot.get_camera_frame()
    start = time.monotonic()
    second = robot.get_camera_frame()
    elapsed = time.monotonic() - start

    assert first["image_base64"] == second["image_base64"] == "FIRST"
    assert elapsed < 0.1


def test_get_camera_frame_stalls_immediately_once_past_the_deadline():
    """Staleness is measured from when a frame was last pushed, not from
    whether anyone has read it since -- so a stale read fails right away,
    it does not sit blocked for stall_timeout_s first."""
    robot = TeleopRobot(stall_timeout_s=0.2)
    robot.push_frame("ONLY")
    time.sleep(0.3)  # let the push go stale

    start = time.monotonic()
    with pytest.raises(TeleopStall):
        robot.get_camera_frame()
    elapsed = time.monotonic() - start

    assert elapsed < 0.1


def test_get_camera_frame_stalls_when_nothing_was_ever_pushed():
    robot = TeleopRobot(stall_timeout_s=5.0)
    with pytest.raises(TeleopStall):
        robot.get_camera_frame()


def test_movement_methods_ack_without_moving_anything():
    robot = TeleopRobot()

    result = robot.drive_forward(speed=100, duration=1.0)

    assert result["action"] == "drive_forward"
    assert result["moved"] == 0
    assert any("DRIVE_FORWARD" in entry for entry in robot.log)


def test_look_methods_ack():
    robot = TeleopRobot()
    assert robot.look_left()["pan"] == -1
    assert robot.look_right()["pan"] == 1
    assert robot.look_center()["pan"] == 0


def test_get_distance_is_inert():
    robot = TeleopRobot()
    distance = robot.get_distance()
    assert distance == NO_SENSOR_CM


def test_safety_never_vetoes_teleop_robot():
    """Same claim replay_robot.py makes about itself: with no real
    ultrasonic, robot/safety.py must never block a move against this
    backend, at any realistic threshold."""
    robot = TeleopRobot()
    safety = SafetyController(robot, min_distance_cm=200)  # deliberately strict

    result = safety.check_and_execute("FORWARD", speed=100, duration=0.5)

    assert result["action"] == "drive_forward"
