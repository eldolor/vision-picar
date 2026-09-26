"""
tests/test_ros_drive.py

Phase R4 (`PLAN-ros-alignment.md` 3.13), the always-run half: the verb
executor in `robot/ros_drive.py` against a FAKE ROS chain in-process, so the
properties hold on a machine with no container. The live half is
`tests/test_ros_chain_live.py`.

The fake chain is deliberately slow AND jittery: each twist lands on the
wheels 40-150 ms after it is sent (R4's first live run measured that
spread), and one in ten takes 300 ms. The jitter, not the delay, is what
made a 45-degree turn come out at 59-74 degrees live: a fixed 100 ms delay
was tried first and the original, overshooting tuning passed against it --
a fake that cannot fail tests nothing.
"""

import threading
import time
from collections import deque

import httpx
import pytest

from robot.ros_drive import RosDriveRobot, moves_for
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import TRACK_WIDTH_M, WHEEL_RADIUS_M, MockRobot

import random

DELAY_S = (0.04, 0.15)
SPIKE_S, SPIKE_P = 0.30, 0.10
LOOP_S = 0.05


class FakeChain:
    """twist_mux + diff_drive_controller + the robot server's wheel loop, in
    one thread: twists queue for DELAY_S, become wheel velocities, and the
    robot advances every LOOP_S."""

    def __init__(self, robot):
        self.robot = robot
        self.pending = deque()
        self.twists = []
        self.lock = threading.Lock()
        self.running = True
        self.rng = random.Random(7)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def handler(self, request: httpx.Request) -> httpx.Response:
        body = __import__("json").loads(request.content)
        with self.lock:
            self.twists.append(body)
            delay = (SPIKE_S if self.rng.random() < SPIKE_P
                     else self.rng.uniform(*DELAY_S))
            # In order, as a queue of messages is: a late one holds the rest.
            due = max(time.monotonic() + delay,
                      self.pending[-1][0] if self.pending else 0.0)
            self.pending.append((due, body))
        return httpx.Response(200, json={"published": body["driver"]})

    def _loop(self):
        last = time.monotonic()
        while self.running:
            time.sleep(LOOP_S)
            now = time.monotonic()
            with self.lock:
                while self.pending and self.pending[0][0] <= now:
                    _, t = self.pending.popleft()
                    v, w = t["linear_m_s"], t["angular_rad_s"]
                    half = w * TRACK_WIDTH_M / 2
                    self.robot.set_wheel_velocity((v - half) / WHEEL_RADIUS_M,
                                                  (v + half) / WHEEL_RADIUS_M)
                self.robot.advance(now - last)
            last = now

    def close(self):
        self.running = False
        self.thread.join()


class _NoVerbs(MockRobot):
    """The robot underneath, with its own verbs forbidden: under drive: ros
    a verb executed directly would be a second writer to the wheels."""

    def drive_forward(self, *a, **k):
        raise AssertionError("verb executed directly on the robot")

    reverse = turn_left = turn_right = drive_forward


@pytest.fixture
def rig():
    grid = build_starter_world()
    grid.x, grid.y, grid.theta = 5.5, 7.5, 0.0      # hallway, facing east: open floor
    inner = _NoVerbs(grid, render=False)
    chain = FakeChain(inner)
    robot = RosDriveRobot(inner, "http://bridge")
    robot._http = httpx.Client(base_url="http://bridge",
                               transport=httpx.MockTransport(chain.handler))
    yield robot, grid, chain
    chain.close()


def _heading_deg(grid):
    import math
    return math.degrees(grid.theta)


def test_a_move_is_one_move_through_a_slow_chain(rig):
    robot, grid, _ = rig
    x0 = grid.x
    robot.drive_forward()
    assert abs((grid.x - x0) * 0.30 - 0.30) <= 0.02


@pytest.mark.parametrize("angle", [15, 45, 90])
def test_a_turn_is_its_angle_through_a_slow_chain(rig, angle):
    robot, grid, _ = rig
    h0 = _heading_deg(grid)
    robot.turn_right(angle)
    assert abs((_heading_deg(grid) - h0) - angle) <= 2.0
    h1 = _heading_deg(grid)
    robot.turn_left(angle)
    assert abs((_heading_deg(grid) - h1) + angle) <= 2.0


def test_verbs_never_reach_the_robot_directly(rig):
    robot, _, chain = rig
    robot.drive_forward()
    robot.turn_left(30)
    robot.reverse()
    assert robot.verbs_through_ros == 3
    assert chain.twists, "every verb went out as twists"


def test_the_driver_rides_on_every_twist(rig):
    robot, _, chain = rig
    with robot.driving_as("twin-dpad"):
        robot.turn_left(15)
    assert {t["driver"] for t in chain.twists} == {"twin-dpad"}


def test_a_verb_means_what_it_meant_without_ros():
    """moves_for() mirrors MockRobot._speed_duration_to_cells() exactly."""
    probe = MockRobot(build_starter_world(), render=False)
    for speed in (0, 1, 25, 50, 75, 100):
        for duration in (0.0, 0.1, 0.25, 0.5, 1.0, 1.7):
            assert moves_for(speed, duration) == probe._speed_duration_to_cells(speed, duration)


def test_stop_does_not_depend_on_the_bridge():
    grid = build_starter_world()
    inner = MockRobot(grid, render=False)
    robot = RosDriveRobot(inner, "http://bridge")

    def dead(request):
        raise httpx.ConnectError("container gone")
    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(dead))
    inner.set_wheel_velocity(5.0, 5.0)
    robot.stop()
    w = inner.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0


def test_a_newer_verb_supersedes_one_in_flight(rig):
    robot, grid, chain = rig
    t = threading.Thread(target=robot.turn_left, args=(90,))
    t.start()
    time.sleep(0.3)
    with robot.driving_as("twin-dpad"):
        robot.turn_right(15)
    t.join(timeout=10)
    assert not t.is_alive()
    # The superseded brain turn stopped streaming once the newer verb began.
    after = [x for x in chain.twists[-10:]]
    assert all(x["driver"] == "twin-dpad" for x in after), after
