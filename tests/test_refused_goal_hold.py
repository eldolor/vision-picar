"""
tests/test_refused_goal_hold.py

`PLAN-ros-alignment.md` 3.82: a goal the bridge refuses must not lift a
Stop's hold, and a bridge that refuses the robot server's secret must end
an explore mission instead of being waited on for the whole time limit.

In process, against tests/test_stop_cancels_goal.py's fake bridge and fake
nav2 (nav2 drives while the bridge holds an active goal, posting `ros`
wheel commands the way `picar_sim_hardware` relays them).
"""

import threading
import time

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.ros_drive import STOP_HOLD_S
from tests.test_ros_goals import world_with
from tests.test_ros_world import FakeTruth
from tests.test_stop_cancels_goal import SlowBridge

START = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}


class RefusingBridge(SlowBridge):
    """SlowBridge whose `POST /goal` can be refused, or held in flight."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.refuse_post = None        # None, "transport", or an (status, body) answer
        self.post_gate = None          # a threading.Event a post waits on
        self.post_entered = threading.Event()
        self.refuse_reads = False      # True: GET/DELETE /goal answer 401

    def handler(self, request):
        if (request.url.path == "/goal" and request.method in ("GET", "DELETE")
                and self.refuse_reads):
            return httpx.Response(401, json={"error": "bad or missing x-app-secret"})
        if request.url.path == "/goal" and request.method == "POST":
            self.post_entered.set()
            if self.post_gate is not None:
                self.post_gate.wait(5.0)
            if self.refuse_post == "transport":
                raise httpx.ConnectError("bridge unreachable", request=request)
            if self.refuse_post == "html":
                return httpx.Response(200, text="<html>a proxy's page</html>")
            if self.refuse_post is not None:
                status, body = self.refuse_post
                return httpx.Response(status, json=body)
        return super().handler(request)


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    bridge = RefusingBridge(start_truth=START)
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = TestClient(server.create_app())

    def nav2_tick():
        if bridge.goal and bridge.goal.get("state") == "active":
            client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                        headers={"x-driver": "ros"})

    def wheel_speeds():
        w = client.get("/wheels").json()
        return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]

    def drive_for(seconds):
        """nav2 ticking at 20 Hz; the fastest wheel speed seen meanwhile."""
        worst, end = 0.0, time.monotonic() + seconds
        while time.monotonic() < end:
            nav2_tick()
            worst = max(worst, *map(abs, wheel_speeds()))
            time.sleep(0.05)
        return worst

    def wait_until(cond, seconds=3.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.02)
        return cond()

    return client, bridge, world, nav2_tick, wheel_speeds, drive_for, wait_until


def _goal_a_driving(client, bridge, nav2_tick, wheel_speeds):
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    nav2_tick()
    assert wheel_speeds() != (0.0, 0.0), "the fake nav2 never drove -- the test proves nothing"


# ---------- criterion 2: a refused goal keeps the Stop's hold ----------

REFUSALS = {
    "transport": "transport",
    "nav2 down": (503, {"error": "nav2 is not up (navigate_to_pose has no server)"}),
    "session changed": (409, {"error": "the SLAM session changed", "session": "other"}),
}


@pytest.mark.parametrize("refusal", list(REFUSALS) + ["no SLAM session"])
def test_a_refused_goal_after_a_stop_does_not_free_the_goal_the_stop_is_ending(rig, monkeypatch,
                                                                                refusal):
    """The goal route cleared the hold and bumped the generation BEFORE
    `set_goal()` answered, so a refused goal ended the Stop's goal-ending
    loop and released nav2 onto the goal the Stop had not yet cancelled."""
    client, bridge, world, nav2_tick, wheel_speeds, drive_for, wait_until = rig
    _goal_a_driving(client, bridge, nav2_tick, wheel_speeds)
    bridge.cancel_fails = True                       # the Stop's first cancels fail
    assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
    time.sleep(STOP_HOLD_S + 0.1)                    # past RosDriveRobot's own stop hold
    if refusal == "no SLAM session":
        monkeypatch.setattr(world, "set_goal",
                            lambda x, y: {"accepted": False, "reason": "no SLAM session yet"})
    else:
        bridge.refuse_post = REFUSALS[refusal]
    # Posted naming no one (ranked `ros`), so no person's authority holds
    # nav2's wheel commands out and masks the hold.
    r = client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()
    assert r["accepted"] is False, r
    assert bridge.goal is not None and bridge.goal["state"] == "active", \
        "goal A was cancelled anyway -- the test proves nothing"
    assert drive_for(1.5) == 0.0, f"a refused goal ({refusal}) released nav2 onto goal A"
    bridge.cancel_fails = False                      # the bridge takes cancels again
    assert wait_until(lambda: bridge.goal is None), "the Stop's loop gave up on goal A"


# ---------- criterion 3: handoff 3b, an accepted goal after a Stop ----------


def test_an_accepted_goal_after_a_stop_drives_and_is_never_cancelled_by_it(rig):
    client, bridge, world, nav2_tick, wheel_speeds, drive_for, wait_until = rig
    _goal_a_driving(client, bridge, nav2_tick, wheel_speeds)
    bridge.cancel_fails = True                       # the Stop's loop is still running
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    time.sleep(0.1)
    bridge.cancel_fails = False
    assert client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()["accepted"]
    cancelled = bridge.cancelled
    assert drive_for(0.3) > 0.0, "a person's new goal was held at zero"
    time.sleep(1.0)
    assert bridge.cancelled == cancelled and bridge.goal is not None, \
        "the Stop's loop cancelled the goal that superseded it"


# ---------- criterion 4: a Stop while set_goal() is in flight ----------


@pytest.mark.parametrize("accepted", [True, False])
def test_a_stop_during_a_goal_in_flight_ends_whichever_goal_nav2_holds(rig, accepted):
    """Guards the fix's design: bumping the generation AFTER an accepted
    answer, unconditionally, would supersede a Stop that landed while the
    call was in flight, and nav2 would drive the goal the Stop ended."""
    client, bridge, world, nav2_tick, wheel_speeds, drive_for, wait_until = rig
    _goal_a_driving(client, bridge, nav2_tick, wheel_speeds)
    bridge.post_gate, bridge.post_entered = threading.Event(), threading.Event()
    if not accepted:
        bridge.refuse_post = REFUSALS["nav2 down"]
    reply = {}
    poster = threading.Thread(target=lambda: reply.setdefault("r", client.post(
        "/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()))
    poster.start()
    try:
        assert bridge.post_entered.wait(5.0), "the goal never reached the bridge"
        t0 = time.monotonic()
        assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
        assert time.monotonic() - t0 < 0.5, "the Stop waited on the goal in flight (A8)"
    finally:
        bridge.post_gate.set()
    poster.join(5.0)
    assert reply["r"]["accepted"] is accepted, reply
    assert drive_for(STOP_HOLD_S + 0.5) == 0.0, "nav2 drove after the Stop"
    assert wait_until(lambda: bridge.goal is None), "the Stop never ended the goal nav2 held"


# ---------- criterion 5: a bridge that refuses the secret ends explore ----------

from control.mission_runner import FAILED, MissionRunner  # noqa: E402
from control.remote_navigator import RemoteNavigator  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.conftest import ASGI_BASE_URL, asgi_client  # noqa: E402

BRIDGE_FAULTS = {
    "401": ((401, {"error": "unauthorised"}), False),
    "403": ((403, {"error": "forbidden"}), False),
    "transport": ("transport", True),
    "nav2 down": (REFUSALS["nav2 down"], True),
}


def _server_over(monkeypatch, refuse_post):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    bridge = RefusingBridge(start_truth=START)
    bridge.refuse_post = refuse_post
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return asgi_client(server.create_app()), bridge


@pytest.mark.parametrize("fault", list(BRIDGE_FAULTS))
def test_the_goal_refusal_says_whether_waiting_can_help(monkeypatch, fault):
    """Every bridge error came back `ros_unavailable` with only prose to
    tell a rotated secret from a blip."""
    refuse_post, retryable = BRIDGE_FAULTS[fault]
    client, _ = _server_over(monkeypatch, refuse_post)
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()
    assert r["accepted"] is False and r["reason"] == "ros_unavailable", r
    assert r.get("retryable") is retryable, r
    expected_status = None if refuse_post == "transport" else refuse_post[0]
    assert r.get("bridge_status") == expected_status, r


def _explore_over_http(client):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = RemoteNavigator(ASGI_BASE_URL, client=client)
    return MissionRunner(robot, target_object="red backpack", policy="explore",
                         navigator=nav, world=MockWorld(grid), max_steps=400,
                         clock=lambda: grid.sim_time, idle=robot.pass_time)


def _ticks_after_first_goal(runner, bridge, limit=400, after=20):
    """Tick until the first goal reaches the bridge, then at most `after`
    more; returns the ticks run after it (None if no goal was ever sent)."""
    runner.start()
    seen = None
    for _ in range(limit):
        ticking = runner.tick()
        if seen is not None:
            seen += 1
        elif bridge.post_entered.is_set():
            seen = 1                             # the tick that sent it
        if not ticking or (seen is not None and seen >= after):
            break
    return seen


@pytest.mark.parametrize("fault", ["401", "403"])
def test_a_bridge_that_refuses_the_secret_ends_the_explore_mission(monkeypatch, fault):
    """`_send()` waited on every refusal, so a mismatched bridge secret sat
    the robot still for the whole time limit (1200 s) instead of failing."""
    client, bridge = _server_over(monkeypatch, BRIDGE_FAULTS[fault][0])
    runner = _explore_over_http(client)
    ticks = _ticks_after_first_goal(runner, bridge)
    assert ticks is not None, "explore never sent a goal -- the test proves nothing"
    status = runner.status()
    assert status["outcome"] == FAILED, status
    assert ticks <= 3, f"the mission ran {ticks} ticks after the refusal"
    assert f"HTTP {fault}" in str(status.get("end_reason")), status


@pytest.mark.parametrize("fault", ["transport", "nav2 down", "session changed"])
def test_a_transient_bridge_fault_is_still_waited_on(monkeypatch, fault):
    refuse_post = REFUSALS[fault]
    client, bridge = _server_over(monkeypatch, refuse_post)
    runner = _explore_over_http(client)
    ticks = _ticks_after_first_goal(runner, bridge)
    assert ticks == 20, ticks
    assert runner.is_running(), runner.status()
    runner.stop()


# ---------- review round 1: a real mismatched secret refuses EVERY route ----------
# Both reviewers: the bridge checks the secret on every route but /health,
# so before a goal is ever sent the map and pose reads are refused too --
# explore waited on "no map yet", the goal route answered "no SLAM session
# yet", and the start check blamed a missing nav2.

from control.remote_world import RemoteWorld  # noqa: E402
from world.bridge_faults import RETRYABLE_HEADER, STATUS_HEADER  # noqa: E402


class SecretBridge(RefusingBridge):
    """A bridge whose every route but /health refuses (or cannot be reached)."""

    def __init__(self, everything, **kw):
        super().__init__(**kw)
        self.everything = everything   # "401", "404" or "transport"

    def handler(self, request):
        if request.url.path != "/health":
            if self.everything == "transport":
                raise httpx.ConnectError("bridge unreachable", request=request)
            return httpx.Response(int(self.everything), json={"error": "refused"})
        return super().handler(request)


def _server_with_secret_fault(monkeypatch, everything):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    bridge = SecretBridge(everything, start_truth=START)
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return asgi_client(server.create_app())


def test_a_mismatched_bridge_secret_is_named_at_the_missions_start(monkeypatch):
    """`available()` read the goal route's 503 as "this world has no nav2"."""
    client = _server_with_secret_fault(monkeypatch, "401")
    with pytest.raises(PermissionError, match="HTTP 401"):
        RemoteNavigator(ASGI_BASE_URL, client=client).available()


@pytest.mark.parametrize("everything", ["401", "transport"])
def test_goal_reads_say_whether_waiting_can_help(monkeypatch, everything):
    client = _server_with_secret_fault(monkeypatch, everything)
    r = client.get("/world/goal")
    assert r.status_code == 503 and "ros_unavailable" in r.text, r.text
    assert r.headers.get(RETRYABLE_HEADER) == ("false" if everything == "401" else "true")
    assert r.headers.get(STATUS_HEADER) == ("401" if everything == "401" else None)


def test_a_goal_before_any_session_is_refused_as_a_setup_fault_not_no_session(monkeypatch):
    client = _server_with_secret_fault(monkeypatch, "401")
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()
    assert r["accepted"] is False and r.get("retryable") is False, r
    assert r.get("bridge_status") == 401, r


def _explore_world_over_http(client):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    return MissionRunner(robot, target_object="red backpack", policy="explore",
                         navigator=RemoteNavigator(ASGI_BASE_URL, client=client),
                         world=RemoteWorld(ASGI_BASE_URL, client=client), max_steps=400,
                         clock=lambda: grid.sim_time, idle=robot.pass_time)


def test_a_mismatched_bridge_secret_ends_explore_on_the_map_read(monkeypatch):
    """With every route refused, explore never reached a goal: it waited on
    "no map yet" for the whole time limit."""
    client = _server_with_secret_fault(monkeypatch, "401")
    runner = _explore_world_over_http(client)
    runner.start()
    ticks = 0
    while ticks < 20 and runner.tick():
        ticks += 1
    status = runner.status()
    assert status["outcome"] == FAILED, status
    assert ticks <= 3, ticks
    assert "HTTP 401" in str(status.get("end_reason")), status


def test_an_unreachable_bridge_is_still_waited_on_at_the_map(monkeypatch):
    client = _server_with_secret_fault(monkeypatch, "transport")
    runner = _explore_world_over_http(client)
    runner.start()
    for _ in range(20):
        assert runner.tick(), runner.status()
    assert runner.is_running()
    runner.stop()


def test_a_refused_tap_to_goal_does_not_take_the_robot_from_the_brain(monkeypatch):
    """code-review: a person's goal took the driver slot before `set_goal()`
    answered, so a goal the bridge refused still preempted the brain."""
    client, _ = _server_over(monkeypatch, REFUSALS["nav2 down"])
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["accepted"] is False, r
    a = client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"}).json()
    assert a.get("reason") != "preempted", a


def test_a_goal_reply_that_is_not_json_is_a_setup_fault(monkeypatch):
    """code-review: a 200 that is not a JSON object (a proxy's page) was
    classed retryable and waited on."""
    client, _ = _server_over(monkeypatch, "html")
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()
    assert r["accepted"] is False and r.get("retryable") is False, r


# ---------- review round 2 ----------


def test_a_tap_in_flight_still_holds_the_robot_against_the_brain(monkeypatch):
    """Round 1 moved the person's claim to after `set_goal()` answered:
    while the bridge answered, explore's own goal or a brain verb passed
    arbitration and then replaced or ran beside the person's goal."""
    client, bridge = _server_over(monkeypatch, None)
    bridge.post_gate = threading.Event()
    reply = {}
    tap = threading.Thread(target=lambda: reply.setdefault("r", client.post(
        "/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers={"x-driver": "twin-dpad"}).json()))
    tap.start()
    try:
        assert bridge.post_entered.wait(5.0)
        verb = client.post("/action", json={"action": "LOOK_LEFT"},
                           headers={"x-driver": "brain"}).json()
        assert verb.get("reason") == "preempted", verb
        bridge.post_entered.clear()
        goal = {}
        other = threading.Thread(target=lambda: goal.setdefault("r", client.post(
            "/world/goal", json={"x_m": 1.0, "y_m": 2.0}).json()))
        other.start()
        other.join(1.0)
        assert goal.get("r", {}).get("reason") == "preempted", goal
    finally:
        bridge.post_gate.set()
    tap.join(5.0)
    assert reply["r"]["accepted"] is True, reply


def test_a_bridge_404_is_not_blamed_on_the_secret(monkeypatch):
    client = _server_with_secret_fault(monkeypatch, "404")
    with pytest.raises(PermissionError) as e:
        RemoteNavigator(ASGI_BASE_URL, client=client).available()
    assert "HTTP 404" in str(e.value) and "ROS_BRIDGE_URL" in str(e.value), e.value
    assert "APP_SHARED_SECRET" not in str(e.value), e.value


def test_a_secret_fault_on_a_goal_read_is_named_as_one(monkeypatch):
    """`_bridge_blip` re-raised the robot server's bare 503: the mission
    ended reading like an outage, not a mismatched secret."""
    client, bridge = _server_over(monkeypatch, None)
    runner = _explore_over_http(client)
    runner.start()
    for _ in range(400):
        if bridge.goal is not None or not runner.tick():
            break
    assert bridge.goal is not None, "explore never sent a goal -- the test proves nothing"
    bridge.refuse_reads = True
    for _ in range(5):
        if not runner.tick():
            break
    status = runner.status()
    assert status["outcome"] == FAILED, status
    assert "HTTP 401" in str(status["end_reason"]), status
    assert "APP_SHARED_SECRET" in str(status["end_reason"]), status
