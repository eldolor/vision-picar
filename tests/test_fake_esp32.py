"""
tests/test_fake_esp32.py

Phase R7 (`PLAN-ros-alignment.md` 3.16) -- the fake ESP32 driver board and
`robot/hardware_robot.py` over a real pseudo-terminal. Since 3.25 the board
is the UGV Rover's ROS Driver; `tests/test_ros_driver_board.py` holds that
phase's criteria and these keep R7's.

Criterion 1 is that the fake IS the firmware, so each rule below cites the
function in `waveshareteam/ugv_base_ros`, `ROS_Driver/` it mirrors. A fake
that behaved better than the board would hide exactly what it exists to find.
"""

import json
import math
import os
import select
import time

import pytest

from robot.hardware_robot import HEARTBEAT_MS, HardwareRobot
from sim.fake_esp32 import NO_LOAD_WHEEL_M_S, FakeEsp32
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import WHEEL_RADIUS_M, MockRobot


@pytest.fixture
def board():
    body = MockRobot(build_starter_world(), render=False)
    b = FakeEsp32(body)
    fd = os.open(b.path, os.O_RDWR | os.O_NOCTTY)
    yield b, body, fd
    os.close(fd)
    b.close()


def send(fd, obj):
    os.write(fd, (json.dumps(obj) + "\n").encode())
    time.sleep(0.05)


def read_frames(fd, seconds=0.2):
    end, buf = time.time() + seconds, b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            buf += os.read(fd, 65536)
    return [json.loads(line) for line in buf.split(b"\n") if line.strip().startswith(b"{")]


# ---------- criterion 1: the fake is the firmware ----------

def test_t1_in_closed_loop_sets_wheel_speeds(board):
    b, _, fd = board                         # setGoalSpeed(), mainType 2
    send(fd, {"T": 1, "L": 0.2, "R": -0.1})
    assert b.setpoint == [0.2, -0.1] and b.use_pid


def test_t1_out_of_range_is_ignored_in_closed_loop(board):
    b, _, fd = board                         # if(inputLeft < -2.0 || > 2.0) return;
    send(fd, {"T": 1, "L": 0.2, "R": 0.2})
    send(fd, {"T": 1, "L": 2.5, "R": 0.0})
    assert b.setpoint == [0.2, 0.2]


def test_t13_is_ros_style_twist(board):
    b, _, fd = board                         # rosCtrl(): X -/+ Z * TRACK_WIDTH / 2
    send(fd, {"T": 13, "X": 0.1, "Z": 1.0})
    assert b.setpoint == pytest.approx([0.1 - 0.172 / 2, 0.1 + 0.172 / 2])


def test_t11_raw_pwm_turns_the_pid_off(board):
    b, _, fd = board                         # CMD_PWM_INPUT: usePIDCompute = false;
    send(fd, {"T": 11, "L": 100, "R": -300})
    assert not b.use_pid and b.pwm == [100, -255]


def test_t130_sends_one_base_frame(board):
    _, _, fd = board                         # baseInfoFeedback(), once
    send(fd, {"T": 131, "cmd": 0})           # the stream is on from boot
    read_frames(fd, 0.1)
    send(fd, {"T": 130})
    frames = [f for f in read_frames(fd) if f.get("T") == 1001]
    assert len(frames) == 1
    assert {"L", "R", "odl", "odr", "v"} <= set(frames[0])


def test_t131_turns_continuous_feedback_off_and_on(board):
    _, _, fd = board                         # if (baseFeedbackFlow) baseInfoFeedback(); every loop
    send(fd, {"T": 131, "cmd": 0})
    read_frames(fd, 0.1)
    assert not [f for f in read_frames(fd, 0.3) if f.get("T") == 1001]
    send(fd, {"T": 131, "cmd": 1})           # ... at most one per 50 ms
    assert len([f for f in read_frames(fd, 0.5) if f.get("T") == 1001]) >= 8


@pytest.mark.parametrize("main_type", [2, 3])
def test_the_heartbeat_stops_the_motors_in_both_modes(board, main_type):
    """heartBeatCtrl(): setGoalSpeed(0, 0) once HEART_BEAT_DELAY passes with no
    T=1/11 -- a PID target of zero whatever the mainType. The BOM's
    'believed' [U] is verified from source; this pins it."""
    b, body, fd = board
    send(fd, {"T": 900, "main": main_type, "module": 0})
    send(fd, {"T": 136, "cmd": 300})
    send(fd, {"T": 1, "L": 0.2, "R": 0.2})
    assert body.get_wheel_state()["left"]["velocity_rad_s"] != 0
    time.sleep(0.45)
    assert body.get_wheel_state()["left"]["velocity_rad_s"] == 0


# ---------- criterion 3: wheels over the wire ----------

def test_wheels_over_the_wire():
    body = MockRobot(build_starter_world(), render=False)
    b = FakeEsp32(body)
    robot = HardwareRobot(b.path, sensors=body)
    try:
        time.sleep(0.2)
        truth0 = body.get_wheel_state()["left"]["position_rad"]
        est0 = robot.get_wheel_state()["left"]["position_rad"]
        x0 = body.world.x
        t_first = time.time()
        robot.set_wheel_velocity(3.0, 3.0)
        end = t_first + 1.0
        while time.time() < end:              # a controller re-sends at 20 Hz
            robot.set_wheel_velocity(3.0, 3.0)
            time.sleep(0.05)
        robot.set_wheel_velocity(0.0, 0.0)
        commanded_s = time.time() - t_first   # what the host actually asked for
        time.sleep(0.2)
        moved_m = (body.world.x - x0) * 0.30
        truth = body.get_wheel_state()["left"]["position_rad"] - truth0
        est = robot.get_wheel_state()["left"]["position_rad"] - est0
        assert moved_m == pytest.approx(3.0 * WHEEL_RADIUS_M * commanded_s, rel=0.05)
        # Within the centimetre the board's odometer allows, plus one edge
        # (3.25 criterion 3). This was rel=0.02 against the General Driver
        # fake, which reported perfect speeds; the ROS Driver's measured
        # speeds and whole-centimetre odometers do not resolve finer.
        assert abs(est - truth) * WHEEL_RADIUS_M <= 0.0104, (est, truth)
    finally:
        robot.close()
        b.close()


# ---------- criterion 4: the heartbeat drill ----------

def test_a_severed_host_stops_on_the_boards_own_heartbeat():
    """The host goes silent mid-motion -- a crashed robot server, a pulled
    cable. Nothing on the host stops the wheels; the BOARD must."""
    body = MockRobot(build_starter_world(), render=False)
    b = FakeEsp32(body)
    robot = HardwareRobot(b.path, sensors=body)
    try:
        time.sleep(0.1)                        # the board reads the host's set-up
        assert b.heartbeat_ms == HEARTBEAT_MS, "the host set the board's heartbeat"
        robot.set_wheel_velocity(3.0, 3.0)
        t_silent = time.time()                # and never writes again
        moving = lambda: body.get_wheel_state()["left"]["velocity_rad_s"] != 0  # noqa: E731
        while not moving() and time.time() - t_silent < 0.5:
            time.sleep(0.002)                 # the board has to read it first
        assert moving(), "the command never reached the wheels"
        while moving() and time.time() - t_silent < 5:
            time.sleep(0.005)
        stopped_after = time.time() - t_silent
        assert HEARTBEAT_MS / 1000 <= stopped_after <= HEARTBEAT_MS / 1000 + 0.05, stopped_after
    finally:
        robot.close()
        b.close()


def test_the_open_loop_speed_ceiling_is_the_motor_s(board):
    b, body, fd = board
    send(fd, {"T": 11, "L": 255, "R": 255})
    time.sleep(0.05)
    assert body.get_wheel_state()["left"]["velocity_rad_s"] * WHEEL_RADIUS_M == pytest.approx(
        NO_LOAD_WHEEL_M_S)
