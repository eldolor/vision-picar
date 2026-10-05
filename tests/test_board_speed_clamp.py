"""
tests/test_board_speed_clamp.py

Handoff 2026-10-02, item 4j: the host never clamped wheel commands to the
motor board's +/-2.0 m/s window. The firmware (`setGoalSpeed()`) DROPS an
out-of-range `T:1` and keeps running the previous setpoint, while the
command still feeds its heartbeat -- so an over-fast command left the robot
doing whatever it did before. Now the host scales both wheels into the
window together, which keeps the ratio (and so the turn) the same.
"""

import time

import pytest

from robot.hardware_robot import BOARD_MAX_WHEEL_M_S, WHEEL_RADIUS_M, HardwareRobot
from sim.fake_esp32 import FakeEsp32
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot


@pytest.fixture
def car():
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    end = time.monotonic() + 2.0
    while not robot.get_wheel_state()["usable"] and time.monotonic() < end:
        time.sleep(0.01)
    yield robot, board
    robot.set_wheel_velocity(0.0, 0.0)
    robot.close()
    board.close()


def _board_setpoint(board, want, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if [round(v, 3) for v in board.setpoint] == [round(w, 3) for w in want]:
            return True
        time.sleep(0.01)
    return False


def test_the_window_is_the_firmwares():
    assert BOARD_MAX_WHEEL_M_S == 2.0


def test_an_over_fast_command_is_scaled_into_the_window_not_dropped(car):
    robot, board = car
    robot.set_wheel_velocity(0.5 / WHEEL_RADIUS_M, 0.5 / WHEEL_RADIUS_M)
    assert _board_setpoint(board, [0.5, 0.5])
    fast = 3.2 / WHEEL_RADIUS_M                       # 3.2 m/s: out of range
    robot.set_wheel_velocity(fast, -fast / 2)
    assert _board_setpoint(board, [2.0, -1.0]), (
        f"board kept {board.setpoint} -- the out-of-range command was dropped")
