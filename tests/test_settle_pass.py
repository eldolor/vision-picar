"""
tests/test_settle_pass.py

`PLAN-ros-alignment.md` 3.29 -- a settle pass for direct-mode verbs on a
real board (wall-clock plans): after a verb, wait for the wheels and fresh
feedback, and correct a leftover error slowly, through the same safety
vetting as any move. One test (or group) per criterion, each red on the
code before 3.29 first. Ground truth is the simulated body's, never what
the host reports.
"""

import math
import threading
import time

import pytest

from robot.hardware_robot import HardwareRobot
from robot.safety import SafetyController
from sim.fake_esp32 import FakeEsp32
from sim.maps import build_world
from sim.mock_robot import MockRobot


def _rig(firmware):
    b = MockRobot(build_world("scaled_house"), render=False)
    board = FakeEsp32(b, firmware=firmware)
    robot = HardwareRobot(board.path, sensors=b)
    time.sleep(0.2)
    return robot, b, board


@pytest.fixture(params=["fork", "stock"])
def rig(request):
    robot, b, board = _rig(request.param)
    yield request.param, robot, b
    robot.close()
    board.close()


@pytest.fixture
def fork_rig():
    robot, b, board = _rig("fork")
    yield robot, b
    robot.close()
    board.close()


def turn_errors(robot, b, angle, n):
    safety = SafetyController(robot, 20.0)
    errs = []
    for i in range(n):
        th0 = b.world.theta
        safety.check_and_execute("RIGHT" if i % 2 == 0 else "LEFT", angle=angle)
        time.sleep(0.15)
        d = math.degrees(math.atan2(math.sin(b.world.theta - th0),
                                    math.cos(b.world.theta - th0)))
        errs.append(round(abs(d) - angle, 2))
    return errs


# ---------- criterion 1: turns land on their angle (fork) ----------

@pytest.mark.parametrize("angle", [15, 45, 90])
def test_1_fork_turns_land_within_a_degree(fork_rig, angle):
    errs = turn_errors(*fork_rig, angle, 10)
    assert all(abs(e) <= 1.0 for e in errs), errs


# ---------- criterion 3: straights still mean a cell ----------

def forward_cells(robot, b, n=5):
    safety = SafetyController(robot, 20.0)
    moved = []
    for _ in range(n):
        x0, y0 = b.world.x, b.world.y
        safety.check_and_execute("FORWARD", speed=50, duration=1.0)
        time.sleep(0.15)
        moved.append(math.hypot(b.world.x - x0, b.world.y - y0) * 30.0)
        safety.check_and_execute("REVERSE", speed=50, duration=1.0)
        time.sleep(0.15)
    return moved


def test_3_a_forward_is_a_cell_on_either_firmware(rig):
    firmware, robot, b = rig
    moved = forward_cells(robot, b)
    bar = 0.5 if firmware == "fork" else 1.0
    assert all(abs(m - 30.0) <= bar for m in moved), [round(m, 2) for m in moved]


# ---------- criterion 5: a correction is vetted like a move ----------

class _Board:
    """A wall-clock robot whose wheels obey each command `latency` seconds
    late -- what overshoots a real turn: the stop lands after the estimate
    said done -- with a scan that can be set to put furniture where the
    correction would go."""

    def __init__(self, latency=0.06):
        self.pos = [0.0, 0.0]
        self.latency = latency
        self.pending = [(0.0, (0.0, 0.0))]      # (effective at, velocity)
        self.stop_count = 0
        self.sent = []
        self.scan = {"usable": False}
        self._t = time.monotonic()
        self._lock = threading.Lock()

    def _tick(self):
        now = time.monotonic()
        t = self._t
        while t < now:
            vel = [v for at, v in self.pending if at <= t][-1]
            nxt = min([at for at, _ in self.pending if at > t] + [now])
            for i in range(2):
                self.pos[i] += vel[i] * (nxt - t)
            t = nxt
        self._t = now

    def set_wheel_velocity(self, left, right):
        with self._lock:
            self._tick()
            self.pending.append((time.monotonic() + self.latency, (left, right)))
            self.sent.append((left, right))

    def get_wheel_state(self):
        with self._lock:
            self._tick()
            return {"usable": True, "left": {"position_rad": self.pos[0]},
                    "right": {"position_rad": self.pos[1]},
                    "wheel_radius_m": 0.04, "track_width_m": 0.172}

    def stop(self):
        self.stop_count += 1
        self.set_wheel_velocity(0.0, 0.0)

    def get_scan(self, max_range_m=None):
        return self.scan

    def get_depth_grid(self):
        return {"usable": False, "zones": []}

    def get_distance(self):
        return 400.0

    def turned_deg(self):
        time.sleep(self.latency * 2)      # let any late command land
        self.get_wheel_state()            # advances the wheels to now
        return math.degrees((self.pos[1] - self.pos[0]) * 0.04 / 0.172)


def _turn_plan(deg, ccw=True):
    w = 1.2 * 0.172 / 2 / 0.04
    l, r = (-w, w) if ccw else (w, -w)
    return {"kind": "turn", "left_rad_s": l, "right_rad_s": r,
            "target": float(deg), "wall_clock": True, "settle": True}


def test_5_a_settle_corrects_an_overshoot_when_the_way_is_clear():
    board = _Board()
    SafetyController(board, 20.0).run_verb(_turn_plan(30))
    assert abs(board.turned_deg() - 30) <= 0.5, board.turned_deg()


def test_5_a_correction_into_furniture_is_clamped_not_driven():
    """The turn overshoots CCW; the correction turns back CLOCKWISE. Put a
    return just off the right front corner, where a clockwise turn closes on
    it: the correction must be refused, so the overshoot stays."""
    board = _Board()
    safety = SafetyController(board, 20.0)
    ranges = [None] * 360
    # 1 cm beside the right flank, near the front (body x ahead, y left, cm;
    # the chassis' right side is y = -11.55). Turning CLOCKWISE swings the
    # flank onto it; the counter-clockwise turn swings it away. The scan is
    # clockwise-positive, 0 ahead, measured from the lidar 4 cm ahead of
    # base_link (3.27).
    from robot.safety import LIDAR_X_M
    x, y = 10.0, -12.55
    lx = x - LIDAR_X_M * 100
    bearing = math.degrees(math.atan2(-y, lx))
    ranges[int(round(bearing)) + 180] = math.hypot(lx, y) / 100.0
    board.scan = {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                  "ranges_m": ranges}
    safety.run_verb(_turn_plan(30))
    assert board.turned_deg() > 30 + 3, "the overshoot should not have been corrected into furniture"
    clockwise = [s for s in board.sent if s[0] > 0 > s[1]]
    assert not clockwise, f"a clockwise correction was sent: {clockwise[:3]}"


# ---------- criterion 6: a stop wins ----------

def test_6_a_stop_during_the_settle_ends_it_and_nothing_follows():
    board = _Board()
    safety = SafetyController(board, 20.0)
    done = threading.Event()

    def run():
        safety.run_verb(_turn_plan(30))
        done.set()

    t = threading.Thread(target=run)
    t.start()
    # Stop INSIDE the settle wait: 50 ms after the main verb's own zero
    # command (the wait is 150 ms), not at a guessed time.
    deadline = time.monotonic() + 3
    while (0.0, 0.0) not in board.sent and time.monotonic() < deadline:
        time.sleep(0.005)
    assert (0.0, 0.0) in board.sent, "the main verb never ended"
    time.sleep(0.05)
    board.stop()
    n = len(board.sent)
    t.join(timeout=3)
    assert done.is_set()
    assert all(s == (0.0, 0.0) for s in board.sent[n:]), board.sent[n:]
