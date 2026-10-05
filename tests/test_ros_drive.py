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
        if request.method == "GET" and request.url.path == "/health":
            return httpx.Response(200, json={"ok": True})     # the bridge's liveness probe (1d)
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


# ---------- a stop never waits on the container (SPEC-REVIEW finding 1) ----------
#
# test_stop_does_not_depend_on_the_bridge covers a DEAD bridge, which fails
# fast. A HUNG one -- accepts the connection, never answers -- is the case
# that held a stop for ~6 s (three posts x the client's 2 s timeout) before
# the direct stop ran, on the server's event loop when the watchdog called it.

def _hung_robot(answer_after_s=3.0):
    grid = build_starter_world()
    inner = MockRobot(grid, render=False)
    robot = RosDriveRobot(inner, "http://bridge")
    posts = []

    def hung(request):
        posts.append(__import__("json").loads(request.content))
        time.sleep(answer_after_s)
        return httpx.Response(200, json={})
    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(hung))
    return robot, inner, posts


def test_a_hung_bridge_does_not_delay_the_stop():
    robot, inner, _ = _hung_robot()
    inner.set_wheel_velocity(5.0, 5.0)
    t0 = time.monotonic()
    robot.stop()
    elapsed = time.monotonic() - t0
    w = inner.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0
    assert elapsed < 0.1, f"stop took {elapsed:.2f}s waiting on the bridge"


def test_the_stop_still_zeroes_every_ros_input():
    robot, inner, posts = _hung_robot(answer_after_s=0.0)
    robot.stop()
    deadline = time.monotonic() + 2.0
    while len(posts) < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sorted(p["driver"] for p in posts) == ["brain", "ros", "twin-dpad"]
    assert all(p["linear_m_s"] == 0 and p["angular_rad_s"] == 0 for p in posts)


def test_repeated_stops_on_a_hung_bridge_do_not_pile_up_threads():
    def zeroing():
        return {t for t in threading.enumerate() if t.name == "ros-stop-zero"}
    already = zeroing()                 # earlier tests' hung threads, still sleeping
    robot, _, posts = _hung_robot(answer_after_s=0.5)
    for _ in range(20):                 # the watchdog stops every poll while silent
        robot.stop()
    time.sleep(0.1)
    assert len(zeroing() - already) <= 1
    assert len(posts) == 1              # the one in-flight zeroing, still on its first post


def test_a_stale_ros_command_cannot_undo_the_stop():
    # The stopped verb's last twist reaches the plugin AFTER the stop, and the
    # plugin posts it to /wheels: it must not restart the wheels.
    robot, inner, _ = _hung_robot()
    robot.stop()
    robot.set_wheel_velocity(5.0, 5.0)
    w = inner.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0


def test_the_hold_ends_on_its_own_and_on_the_next_verb():
    import robot.ros_drive as rd
    robot, inner, _ = _hung_robot()
    robot.stop()
    robot._begin()                      # the next verb starts: commands flow again
    robot.set_wheel_velocity(5.0, 5.0)
    assert inner.get_wheel_state()["left"]["velocity_rad_s"] == 5.0

    robot.stop()
    time.sleep(rd.STOP_HOLD_S + 0.05)   # no verb, but the hold has expired
    robot.set_wheel_velocity(4.0, 4.0)
    assert inner.get_wheel_state()["left"]["velocity_rad_s"] == 4.0


# ---------- spec review 3, V7: what marks the bridge down ----------
# Liveness is about reachability. A 4xx is the bridge answering -- the 400
# for an unknown driver, or a 401 for a wrong secret -- so it must not mark
# ROS down; the unauthenticated /health probe would then mark it up again
# and the state would flap.

def _robot_answering(status=None, exc=None):
    inner = MockRobot(build_starter_world(), render=False)
    robot = RosDriveRobot(inner, "http://bridge")

    def reply(request):
        if exc is not None:
            raise exc("bridge", request=request)
        return httpx.Response(status, json={})

    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(reply))
    return robot


@pytest.mark.parametrize("status", [400, 401, 404])
def test_a_4xx_from_the_bridge_does_not_mark_ros_down(status):
    robot = _robot_answering(status=status)
    with pytest.raises(httpx.HTTPStatusError):
        robot._send(0.0, 0.0, driver="teleop")
    assert robot.bridge_up() is True


@pytest.mark.parametrize("case", [{"status": 503}, {"exc": httpx.ConnectError},
                                  {"exc": httpx.ReadTimeout}])
def test_a_5xx_or_transport_failure_marks_ros_down(case):
    robot = _robot_answering(**case)
    with pytest.raises(httpx.HTTPError):
        robot._send(0.0, 0.0, driver="brain")
    assert robot.bridge_up() is False


# ---------- handoff 3a: every person drives on the D-pad's input ----------

def _bridge_inputs():
    """picar_bridge's DRIVER_TOPICS keys, read from bridge.py (which imports
    rclpy, so it cannot be imported here)."""
    import re
    from pathlib import Path
    text = (Path(__file__).resolve().parent.parent / "service" / "slam" / "src" /
            "picar_bridge" / "picar_bridge" / "bridge.py").read_text()
    block = text[text.index("DRIVER_TOPICS = {"):]
    block = block[:block.index("}")]
    return set(re.findall(r'"([a-z-]+)":', block))


@pytest.mark.parametrize("driver, expected", [
    ("twin-dpad", "twin-dpad"), ("teleop-operator", "twin-dpad"), ("", "twin-dpad"),
    ("someone-with-curl", "twin-dpad"), ("brain", "brain"), ("teleop", "brain"),
    ("ros", "ros"),
])
def test_a_driver_reaches_the_input_of_its_rank(driver, expected):
    from robot.ros_drive import ros_input_for
    assert ros_input_for(driver) == expected
    assert expected in _bridge_inputs()


def test_a_teleop_operator_verb_drives_through_ros():
    """It used to post driver 'teleop-operator', which the bridge has no
    input for -- a 400, and the verb refused `ros_unavailable`."""
    inner = _NoVerbs(build_starter_world())
    chain = FakeChain(inner)
    bot = RosDriveRobot(inner, "http://bridge")
    bot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(chain.handler))
    try:
        with bot.driving_as("teleop-operator"):
            bot.turn_left(15)
        drivers = {t["driver"] for t in chain.twists}
        assert drivers == {"twin-dpad"}, drivers
        assert drivers <= _bridge_inputs()
    finally:
        chain.close()
