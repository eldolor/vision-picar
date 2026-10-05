"""
tests/test_blind_reverse.py

A body that really moves and cannot see astern does not reverse -- decided by
the user 2026-10-02 (docs-review/SPEC-REVIEW.md finding 3).

Until then `rear_clearance()` answered `(None, "no_rear_sensor")` on a body
with no scan and every reverse path let the move through: on the car, before
its lidar driver lands, REVERSE and the guarded verb's settle passes backed
up blind. FORWARD on that body was already refused (its distance reads 0.0,
3.18's "unobserved is not clear"); this applies the same rule astern.

The body here is the real backend, `HardwareRobot`, over the fake ESP32 on a
pseudo-terminal, with NO sensors -- exactly the car's state today. Turns stay
allowed: they are how a robot turns away from something. A body with no
wheels to move (a phone walk) keeps its old behaviour.
"""

import time

import pytest

from robot.hardware_robot import HardwareRobot

from robot.safety import SafetyController, SafetyViolation
from sim.fake_esp32 import FakeEsp32
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot
from sim.teleop_robot import TeleopRobot


@pytest.fixture
def blind_car():
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path)               # the car today: motors, no sensors
    deadline = time.time() + 2.0
    while not robot.get_wheel_state().get("usable") and time.time() < deadline:
        time.sleep(0.02)
    assert robot.get_wheel_state()["usable"], "no feedback frame from the fake board"
    assert not robot.get_scan()["usable"]
    yield robot, body
    robot.close()
    board.close()


def test_reverse_is_refused_when_nothing_sees_astern(blind_car):
    robot, body = blind_car
    x0, y0 = body.world.x, body.world.y
    with pytest.raises(SafetyViolation, match="astern_not_observed"):
        SafetyController(robot).check_and_execute("REVERSE", speed=50, duration=0.5)
    time.sleep(0.2)
    assert (body.world.x, body.world.y) == (x0, y0)


def test_a_standing_reverse_command_is_clamped(blind_car):
    robot, _ = blind_car
    left, right, reason = SafetyController(robot).vet_wheel_velocity(-5.0, -5.0)
    assert (left, right) == (0.0, 0.0)
    assert "astern_not_observed" in reason


def test_turns_stay_allowed_blind(blind_car):
    robot, _ = blind_car
    _, _, reason = SafetyController(robot).vet_wheel_velocity(-3.0, 3.0)
    assert reason is None


def test_a_body_with_no_wheels_still_reverses():
    # A phone walk: REVERSE moves a person, who can see.
    robot = TeleopRobot()
    assert SafetyController(robot).reverse_clearance() == (None, "no_rear_sensor")


def test_a_body_with_a_scan_is_judged_by_it():
    robot = MockRobot(build_scaled_world(), render=False)
    clearance, source = SafetyController(robot).reverse_clearance()
    assert source != "astern_not_observed"
