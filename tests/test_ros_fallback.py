"""
tests/test_ros_fallback.py

`PLAN-ros-alignment.md` 3.24, G3 -- the fallback when the ROS stack dies,
decided by the user: only a PERSON may drive on it. One test per criterion,
written before the fix and confirmed red first.

A real robot-server app in `drive: ros`. "ROS is alive" is what it is on
the car: the actuator plugin posting `/wheels` as `ros` at 20 Hz, played
here by a thread; "the container dies" is that thread stopping and the
bridge refusing connections. While alive, R4's fake chain
(`tests/test_ros_drive.py`) turns twists into wheel motion, so ROS verbs
really move the robot. The mission is the brain's own: a `MissionRunner`
over `RemoteRobot`, exactly as `control/brain_server.py` drives it.
"""

import math
import threading
import time

import httpx
import pytest

import robot.server as server
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from tests.conftest import ASGI_BASE_URL, asgi_client
from tests.test_ros_drive import FakeChain


class _Plugin(FakeChain):
    """R4's chain, ending where the real one ends: in the ACTUATOR PLUGIN,
    whose 20 Hz `POST /wheels` (as `ros`) is the only way ROS moves the
    wheels -- vetted by the server on the way in. A twist lands after R4's
    measured delay as the plugin's current command; the heartbeat posts that
    command. The real server's wheel loop integrates; the chain never
    touches the robot. (The first harness wrote twists onto the wheels AND
    posted zeros as the pulse, so the two fought and a turn sometimes went
    nowhere -- and a dead container's queued twists landed after it died.)"""

    def __init__(self, robot):
        self.dead = False
        self.command = (0.0, 0.0)
        super().__init__(robot)

    def _loop(self):
        while self.running:
            time.sleep(0.05)
            with self.lock:
                now = time.monotonic()
                while not self.dead and self.pending and self.pending[0][0] <= now:
                    _, t = self.pending.popleft()
                    v, w = t["linear_m_s"], t["angular_rad_s"]
                    ws = self.robot.get_wheel_state()
                    half = w * ws["track_width_m"] / 2
                    self.command = ((v - half) / ws["wheel_radius_m"], (v + half) / ws["wheel_radius_m"])

    def die(self):
        with self.lock:
            self.dead = True
            self.pending.clear()
            self.command = (0.0, 0.0)

    def revive(self):
        with self.lock:
            self.dead = False


class Stack:
    """The robot server in drive: ros, and a ROS side that can die."""

    def __init__(self, monkeypatch):
        monkeypatch.setenv("ROBOT_DRIVE", "ros")
        monkeypatch.setenv("SIM_MAP", "starter_house")
        real_get_robot, self.built = server.get_robot, {}

        def keep(*a, **kw):
            r = real_get_robot(*a, **kw)
            self.built["robot"] = r
            return r

        monkeypatch.setattr(server, "get_robot", keep)
        self.client = asgi_client(server.create_app())
        self.client.__enter__()                          # run the lifespan: watchdog + wheel loop
        self.robot = self.built["robot"]                 # a RosDriveRobot
        self.inner = self.robot.inner                    # the MockRobot underneath
        self.chain = _Plugin(self.inner)
        self.bridge_up = True
        self.robot._http = httpx.Client(base_url="http://bridge",
                                        transport=httpx.MockTransport(self._bridge))
        self._beat = None
        self.alive = threading.Event()

    def _bridge(self, request):
        if not self.bridge_up:
            raise httpx.ConnectError("container is gone", request=request)
        return self.chain.handler(request)

    def _heartbeat(self):
        while self.alive.is_set():
            left, right = self.chain.command
            self.client.post("/wheels", json={"left_rad_s": left, "right_rad_s": right},
                             headers={"x-driver": "ros"})
            time.sleep(0.05)

    def ros_up(self):
        self.chain.revive()
        self.bridge_up = True
        self.alive.set()
        self._beat = threading.Thread(target=self._heartbeat, daemon=True)
        self._beat.start()

    def ros_down(self):
        """The container dies: the plugin's posts stop, the bridge refuses."""
        self.bridge_up = False
        self.chain.die()
        self.alive.clear()
        if self._beat:
            self._beat.join()

    def act(self, action, driver="twin-dpad", **kw):
        return self.client.post("/action", json={"action": action, **kw},
                                headers={"x-driver": driver}).json()

    def pose(self):
        w = self.inner.world
        return (w.x, w.y, w.theta)

    def close(self):
        self.ros_down()
        self.chain.close()
        self.client.__exit__(None, None, None)


@pytest.fixture
def stack(monkeypatch):
    s = Stack(monkeypatch)
    yield s
    s.close()


def _mission(stack):
    remote = RemoteRobot(ASGI_BASE_URL, client=stack.client)
    return MissionRunner(remote, target_object="red backpack", max_steps=200)


def test_criterion_1_the_mission_ends_when_ros_dies(stack):
    stack.ros_up()
    time.sleep(0.3)
    runner = _mission(stack)
    runner.start()
    done = threading.Event()

    def loop():
        while runner.tick():
            pass
        done.set()

    threading.Thread(target=loop, daemon=True).start()
    time.sleep(2.0)                                   # a few steps through ROS
    assert runner.is_running(), runner.status()
    killed = time.monotonic()
    stack.ros_down()
    assert done.wait(3.0), f"still running 3 s after ROS died: {runner.status()}"
    status = runner.status()
    assert status["outcome"] == "failed", status
    assert "ros" in (status.get("error") or "").lower(), status
    time.sleep(0.2)
    at_end = stack.pose()
    time.sleep(2.0)
    assert stack.pose() == at_end, "the robot moved after the mission ended"
    assert time.monotonic() - killed < 6.0


def test_criterion_2_a_person_can_drive_on_the_fallback(stack):
    stack.ros_up()
    time.sleep(0.3)
    stack.ros_down()
    time.sleep(0.6)                                   # ROS silent past the 0.5 s bar
    t0 = time.monotonic()
    before = stack.pose()
    reply = stack.act("FORWARD")                      # east from the start room: 2.85 m clear
    assert reply.get("executed") is True, reply
    assert time.monotonic() - t0 < 2.0
    moved = math.hypot(stack.pose()[0] - before[0], stack.pose()[1] - before[1]) * 30
    assert abs(moved - 30.0) <= 0.5, f"moved {moved:.2f} cm"


def test_criterion_2_the_fallback_is_vetted(stack):
    """North from the start the wall is 45 cm from the centre: a FORWARD on
    the fallback must stop at the line or be refused, as drive: direct."""
    from tests import footprint_sweep as fs
    stack.ros_down()
    time.sleep(0.6)
    assert stack.act("LEFT", angle=90).get("executed") is True
    stack.act("FORWARD")
    stack.act("FORWARD")
    T, G, _ = fs.truth(stack.inner.world)
    assert T >= fs.T_BAR_CM, f"drove to {T:.1f} cm of travel-to-contact on the fallback"


def test_criterion_3_autonomy_is_refused_while_ros_is_down(stack):
    stack.ros_up()
    time.sleep(0.3)
    stack.ros_down()
    time.sleep(0.6)
    for action in ("FORWARD", "LEFT", "LOOK_CENTER"):
        reply = stack.act(action, driver="brain", angle=15)
        assert reply.get("executed") is False and reply.get("reason") == "ros_unavailable", (action, reply)


def test_criterion_3_a_mission_started_during_the_outage_ends_at_once(stack):
    stack.ros_down()
    time.sleep(0.6)
    before = stack.pose()
    runner = _mission(stack)
    runner.start()
    runner.tick()
    status = runner.status()
    assert not runner.is_running() and status["outcome"] == "failed", status
    assert "ros" in (status.get("error") or "").lower(), status
    assert stack.pose() == before, "a mission moved the robot while ROS was down"


def test_criterion_4_back_without_a_restart(stack):
    stack.ros_down()
    time.sleep(0.6)
    assert stack.client.get("/health").json()["drive"].get("ros_up") is False
    t0 = time.monotonic()
    stack.ros_up()
    while time.monotonic() - t0 < 5.0 and not stack.client.get("/health").json()["drive"].get("ros_up"):
        time.sleep(0.05)
    assert stack.client.get("/health").json()["drive"].get("ros_up") is True
    reply = stack.act("LEFT", angle=15)
    assert reply.get("executed") is True and reply["result"].get("via") == "ros", reply


# ---------------------------------------------------------------------------
# Handoff 2026-10-02 1d (decided by the user): a failed send to the bridge
# marks ROS down. Before it, liveness was judged only from the plugin's
# /wheels posts, so a dead BRIDGE behind a live plugin refused every verb --
# a person's included -- as ros_unavailable, with no fallback at all.

def test_1d_a_dead_bridge_behind_a_live_plugin_marks_ros_down(stack):
    stack.ros_up()
    time.sleep(0.3)
    stack.bridge_up = False                           # the bridge dies; the plugin keeps posting
    first = stack.act("LEFT", angle=15)               # the send that finds it dead
    assert first.get("executed") is False and first.get("reason") == "ros_unavailable", first
    assert stack.client.get("/health").json()["drive"]["ros_up"] is False

    before = stack.pose()
    reply = stack.act("FORWARD")                      # a person drives on, direct
    assert reply.get("executed") is True and reply["result"].get("via") == "direct-fallback", reply
    moved = math.hypot(stack.pose()[0] - before[0], stack.pose()[1] - before[1]) * 30
    assert abs(moved - 30.0) <= 0.5, f"moved {moved:.2f} cm"

    # The person's claim must lapse first, or the brain is refused
    # `preempted` before ROS's liveness is ever asked.
    time.sleep(stack.client.get("/health").json()["watchdog_timeout_s"] + 0.2)
    auto = stack.act("LEFT", driver="brain", angle=15)
    assert auto.get("executed") is False and auto.get("reason") == "ros_unavailable", auto


def test_1d_ros_is_back_once_the_bridge_answers_again(stack):
    stack.ros_up()
    time.sleep(0.3)
    stack.bridge_up = False
    stack.act("LEFT", angle=15)
    assert stack.client.get("/health").json()["drive"]["ros_up"] is False
    stack.bridge_up = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0 and not stack.client.get("/health").json()["drive"]["ros_up"]:
        time.sleep(0.05)
    assert stack.client.get("/health").json()["drive"]["ros_up"] is True
    reply = stack.act("LEFT", angle=15)
    assert reply.get("executed") is True and reply["result"].get("via") == "ros", reply
