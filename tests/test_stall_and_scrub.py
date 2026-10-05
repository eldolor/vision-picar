"""
tests/test_stall_and_scrub.py

PLAN-ros-alignment.md 3.35: what can be done for gaps 2 and 4 of the car
body's failure modes without the car.

Gap 2: a verb on the wall clock (the real board) had no stall rule, so a
snagged wheel was pushed until the verb's cap (3x its expected time + 1 s).
It now ends `stalled` after `VERB_STALL_S` with no encoder progress -- the
ROS path's rule and number, in one place.

Gap 4: skid steer turns less than the geometric track predicts. The
correction, `track_scrub`, is the REAL chassis' and must never reach the
simulator. 1.0 everywhere until the Rover is measured.
"""

import math
import threading
import time

import pytest

from robot import hardware_robot
from robot.hardware_robot import TRACK_WIDTH_M, WHEEL_RADIUS_M, HardwareRobot
from robot.interface import VERB_STALL_S, RobotInterface, carry_out_verb
from robot.safety import SafetyController
from sim.fake_esp32 import FakeEsp32
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot

# A pose in the scaled house where a right pivot jams on furniture after
# ~12 degrees (found by search; the sim stops a pivot at contact, 3.19).
BLOCKED_PIVOT = (15.554, 8.248, 1.54)


def _wait(cond, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


# ---- criterion 1: a stalled wall-clock verb ends `stalled` ------------------

class _SnaggingWheels(RobotInterface):
    """Wheels that turn as commanded on the wall clock until `snag_after_s`,
    then stop advancing however hard they are driven -- a wheel caught on a
    rug, as the encoders see it."""

    def __init__(self, snag_after_s):
        self.snag_at = time.monotonic() + snag_after_s
        self.cmd = (0.0, 0.0)
        self.pos = [0.0, 0.0]
        self.at = time.monotonic()
        self.last_progress_at = None

    def _advance(self):
        now = time.monotonic()
        moving_until = min(now, self.snag_at)
        dt = max(0.0, moving_until - self.at)
        if dt > 0:
            self.pos[0] += self.cmd[0] * dt
            self.pos[1] += self.cmd[1] * dt
            if any(self.cmd):
                self.last_progress_at = moving_until
        self.at = max(self.at, moving_until) if now >= self.snag_at else now

    def set_wheel_velocity(self, left, right):
        self._advance()
        self.cmd = (left, right)
        return {}

    def get_wheel_state(self):
        self._advance()
        return {"usable": True,
                "left": {"position_rad": self.pos[0], "velocity_rad_s": self.cmd[0]},
                "right": {"position_rad": self.pos[1], "velocity_rad_s": self.cmd[1]},
                "wheel_radius_m": 0.04, "track_width_m": 0.172}

    # the rest of the contract, unused here
    def drive_forward(self, speed=50, duration=0.5): return {}
    def reverse(self, speed=50, duration=0.5): return {}
    def turn_left(self, angle=90): return {}
    def turn_right(self, angle=90): return {}
    def stop(self): return {}
    def look_left(self): return {}
    def look_right(self): return {}
    def look_center(self): return {}
    def get_camera_frame(self): return {"image": b"", "media_type": "image/jpeg"}
    def get_distance(self): return 200.0


def test_a_snagged_wall_clock_verb_ends_stalled_promptly():
    body = _SnaggingWheels(snag_after_s=0.3)
    plan = {"kind": "straight", "left_rad_s": 5.0, "right_rad_s": 5.0,
            "target": 0.30, "wall_clock": True}          # 1.5 s expected; cap ~5.5 s
    out = carry_out_verb(body, plan)
    ended_at = time.monotonic()
    assert out["ended"] == "stalled", out
    assert ended_at - body.last_progress_at <= VERB_STALL_S + 0.2
    assert body.cmd == (0.0, 0.0)


def test_a_blocked_pivot_on_the_fake_board_ends_stalled():
    body = MockRobot(build_scaled_world(), render=False)
    body.world.x, body.world.y, body.world.theta = BLOCKED_PIVOT
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path)              # no sensors: nothing vets the turn
    try:
        assert _wait(lambda: robot.get_wheel_state()["usable"])
        plan = robot.verb_plan("RIGHT", angle=90)   # 1.3 s expected; cap ~6 s
        started = time.monotonic()
        out = SafetyController(robot).run_verb(plan)
        took = time.monotonic() - started
        assert out["ended"] == "stalled", out
        assert out["done"] < 45, out                # it jammed early
        # Jam after ~12 deg (~0.2 s of turning), then the stall window.
        assert took <= 0.2 + VERB_STALL_S + 0.6, took
    finally:
        robot.close()
        board.close()


# ---- criterion 2: a clear verb never stalls ---------------------------------

@pytest.mark.parametrize("action,kw", [("FORWARD", {"speed": 50, "duration": 0.5}),
                                       ("LEFT", {"angle": 45}),
                                       ("RIGHT", {"angle": 90})])
def test_a_clear_fake_board_verb_completes(action, kw):
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    try:
        assert _wait(lambda: robot.get_wheel_state()["usable"])
        plan = robot.verb_plan(action, **kw)
        out = SafetyController(robot).run_verb(plan)
        assert out["ended"] == "complete", out
    finally:
        robot.close()
        board.close()


# ---- criterion 3: one number ------------------------------------------------

def test_the_ros_path_uses_the_same_stall_window():
    from robot import ros_drive
    src = open(ros_drive.__file__).read()
    assert "VERB_STALL_S" in src
    assert not hasattr(ros_drive, "STALL_S"), "a second stall constant"


# ---- criterion 4: the scrub reaches every direct-mode use -------------------

def _hardware(track_scrub, sensors=None):
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=sensors, track_scrub=track_scrub)
    assert _wait(lambda: robot.get_wheel_state()["usable"])
    return robot, board


def test_the_scrub_sizes_pivots_and_is_published():
    robot, board = _hardware(1.6)
    try:
        effective = TRACK_WIDTH_M * 1.6
        plan = robot.verb_plan("RIGHT", angle=90)
        assert plan["left_rad_s"] == pytest.approx(1.2 * effective / 2 / WHEEL_RADIUS_M)
        assert robot.get_wheel_state()["track_width_m"] == pytest.approx(effective)
    finally:
        robot.close()
        board.close()


def test_the_scrub_scales_the_odometry_heading():
    # The same wheel travel reads as less body rotation on a wider track.
    one, b1 = _hardware(1.0)
    two, b2 = _hardware(1.6)
    try:
        for r in (one, two):
            with r._lock:
                r._board_m = [r._board_m[0] - 0.05, r._board_m[1] + 0.05]
        h1 = (360 - one.get_odometry()["heading_deg"]) % 360
        h2 = (360 - two.get_odometry()["heading_deg"]) % 360
        assert h2 == pytest.approx(h1 / 1.6, rel=0.02)
    finally:
        for r, b in ((one, b1), (two, b2)):
            r.close()
            b.close()


# ---- criterion 5: never in the simulator -------------------------------------

def test_the_fake_board_ignores_a_scrub_setting(monkeypatch, sim_programs):
    import httpx
    # 3.36: the fake board's body runs as the split simulator's programs;
    # TRACK_SCRUB is set for both, as run.sh would pass it to both.
    programs = sim_programs(SIM_MAP="scaled_house", TRACK_SCRUB="1.6").apply(monkeypatch)
    monkeypatch.setenv("TRACK_SCRUB", "1.6")
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    from robot.factory import get_robot
    robot = get_robot()
    try:
        assert robot.track_scrub == 1.0
        assert _wait(lambda: robot.get_wheel_state()["usable"])
        def heading():
            return httpx.get(f"{programs.body_url}/truth").json()["heading_deg"]
        heading0 = heading()
        out = SafetyController(robot).run_verb(robot.verb_plan("RIGHT", angle=90))
        assert out["ended"] == "complete"
        time.sleep(0.2)
        turned = (heading() - heading0 + 180) % 360 - 180
        assert abs(abs(turned) - 90) <= 3, turned
    finally:
        robot.close()
        robot.sensors.close()


def test_a_real_board_reads_the_scrub_setting(monkeypatch):
    from robot import factory
    seen = {}

    class Spy:
        def __init__(self, port, sensors=None, track_scrub=1.0):
            seen.update(port=port, track_scrub=track_scrub)

    monkeypatch.setattr(hardware_robot, "HardwareRobot", Spy)
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    monkeypatch.delenv("SIM_MOTOR_BOARD", raising=False)
    monkeypatch.setenv("ROBOT_SERIAL", "/dev/picar-board")
    monkeypatch.setenv("TRACK_SCRUB", "1.45")
    factory.get_robot()
    assert seen == {"port": "/dev/picar-board", "track_scrub": 1.45}
    monkeypatch.delenv("TRACK_SCRUB")
    factory.get_robot()
    assert seen["track_scrub"] == 1.0


def test_the_scrub_is_reported_in_health():
    robot, board = _hardware(1.0)
    try:
        assert robot.feedback_status()["track_scrub"] == 1.0
    finally:
        robot.close()
        board.close()
