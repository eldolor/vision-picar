"""
tests/test_person_stop.py

`PLAN-ros-alignment.md` 3.84: a person's Stop is the last word over
autonomy. After a Stop from a person (the D-pad, an unnamed caller), the
brain's goals and verbs are refused by name and its mission ends
`cancelled`, `cancelled_by: "stop"`; a person's own next command always
works, and a new mission lifts it. And a verb under `drive: ros` lifts the
Stop's hold only under its lock, and only if no stop came after it.

In process, against 3.82's fake bridge and fake nav2
(tests/test_refused_goal_hold.py): nav2 drives while the bridge holds an
active goal, posting `ros` wheel commands as `picar_sim_hardware` relays them.
"""

import time

import pytest

from control.mission_runner import CANCELLED
from tests.test_refused_goal_hold import _explore_over_http, _server_over

PERSONS = {
    "twin-dpad": {"x-driver": "twin-dpad"},
    "unnamed": {},
    "teleop-operator": {"x-driver": "teleop-operator"},
}


def _nav2(client, bridge):
    def tick():
        if bridge.goal and bridge.goal.get("state") == "active":
            client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                        headers={"x-driver": "ros"})

    def speed():
        w = client.get("/wheels").json()
        return max(abs(w["left"]["velocity_rad_s"]), abs(w["right"]["velocity_rad_s"]))

    return tick, speed


def _explore_driving(monkeypatch):
    """An explore mission over HTTP whose first goal nav2 is driving."""
    client, bridge = _server_over(monkeypatch, None)
    runner = _explore_over_http(client)
    runner.start()
    for _ in range(400):
        assert runner.tick(), runner.status()
        if bridge.goal and bridge.goal.get("state") == "active":
            break
    assert bridge.goal and bridge.goal["state"] == "active", "explore never sent a goal"
    tick, speed = _nav2(client, bridge)
    tick()
    assert speed() > 0.0, "the fake nav2 never drove -- the test proves nothing"
    return client, bridge, runner, tick, speed


# ---------- criterion 2: a person's Stop during explore ----------


@pytest.mark.parametrize("who", list(PERSONS))
def test_a_persons_stop_during_explore_ends_the_mission_and_nav2_stays_stopped(monkeypatch, who):
    """The D-pad's STOP called only the robot server's /stop. The server
    ended nav2's goal; explore saw it end and posted its next goal, naming
    no one (ranked `ros`); the bridge took it, the route lifted the Stop's
    hold, and the robot drove again within seconds of a person's Stop."""
    client, bridge, runner, tick, speed = _explore_driving(monkeypatch)
    posted = len(bridge.posted)
    assert client.post("/stop", headers=PERSONS[who]).json()["executed"]
    time.sleep(0.1)
    worst, ticks_after, end = 0.0, 0, time.monotonic() + 3.0
    while time.monotonic() < end:
        if runner.is_running():
            runner.tick()
            ticks_after += 1
        tick()
        worst = max(worst, speed())
        time.sleep(0.05)
    assert worst == 0.0, f"nav2 drove {worst} rad/s after a person's Stop ({who})"
    assert len(bridge.posted) == posted, f"explore's goal reached the bridge: {bridge.posted}"
    status = runner.status()
    assert status["outcome"] == CANCELLED, status
    assert status.get("cancelled_by") == "stop", status
    assert "stop" in str(status.get("end_reason")).lower(), status


# ---------- criterion 3: the brain's verbs and named goals are refused too ----------

from fastapi.testclient import TestClient  # noqa: E402

import robot.server as server  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from control.remote_robot import RemoteRobot  # noqa: E402
from control.remote_world import RemoteWorld  # noqa: E402
from tests.conftest import ASGI_BASE_URL, asgi_client  # noqa: E402

BUDGET = 400


@pytest.fixture
def plain(monkeypatch):
    """The robot server over the plain sim world (no goals)."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = asgi_client(server.create_app())
    yield client
    client.close()


@pytest.mark.parametrize("who", list(PERSONS))
def test_a_brain_verb_after_a_persons_stop_is_refused_by_name(plain, who):
    client = plain
    assert client.post("/action", json={"action": "LOOK_LEFT"},
                       headers={"x-driver": "brain"}).json()["executed"]
    client.post("/stop", headers=PERSONS[who])
    r = client.post("/action", json={"action": "LOOK_RIGHT"},
                    headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == "person_stop", r
    assert who.replace("unnamed", "unknown") in r["detail"], r


def test_a_brain_named_goal_after_a_persons_stop_is_refused_by_name(monkeypatch):
    client, bridge = _server_over(monkeypatch, None)
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0},
                    headers={"x-driver": "brain"}).json()
    assert r["accepted"] is False and r["reason"] == "person_stop", r
    assert bridge.posted == [], "a refused goal reached the bridge"


def test_a_mission_over_http_ends_cancelled_by_a_persons_stop(plain):
    client = plain
    runner = MissionRunner(RemoteRobot(ASGI_BASE_URL, client=client),
                           target_object="red backpack", max_steps=BUDGET,
                           world=RemoteWorld(ASGI_BASE_URL, client=client))
    runner.start()
    for _ in range(3):
        assert runner.tick(), runner.status()
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    for _ in range(3):
        if not runner.tick():
            break
    status = runner.status()
    assert status["outcome"] == CANCELLED and status["cancelled_by"] == "stop", status
    assert "twin-dpad" in status["end_reason"], status


# ---------- criterion 4: what stays ----------


@pytest.mark.parametrize("who", ["brain", "ros"])
def test_an_autonomous_stop_latches_nothing(plain, who):
    client = plain
    client.post("/stop", headers={"x-driver": who})
    r = client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is True, r


def test_a_persons_own_verb_after_their_stop_works(plain):
    client = plain
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    r = client.post("/action", json={"action": "LEFT", "angle": 30},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["executed"] is True, r
    # ...and the brain stays out: the person driving does not resume autonomy.
    time.sleep(1.2)                                    # past the person's claim
    r = client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"}).json()
    assert r["reason"] == "person_stop", r


@pytest.mark.parametrize("named", [True, False])
def test_a_persons_goal_or_a_scripts_after_a_stop_is_accepted_and_drives(monkeypatch, named):
    client, bridge = _server_over(monkeypatch, None)
    tick, speed = _nav2(client, bridge)
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    time.sleep(0.1)
    headers = {"x-driver": "twin-dpad"} if named else {}
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers=headers).json()
    assert r["accepted"] is True, r
    # 1.5 s: under drive: direct a person's goal holds the driver slot for
    # `watchdog_timeout_s` after it is accepted (3.82's known sim-only limit).
    worst, end = 0.0, time.monotonic() + 1.5
    while time.monotonic() < end:
        tick()
        worst = max(worst, speed())
        time.sleep(0.05)
    assert worst > 0.0, "the goal after the Stop was held at zero"


def test_resume_lifts_the_latch(plain):
    client = plain
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    assert client.post("/autonomy/resume", headers={"x-driver": "brain"}).json()["resumed"]
    r = client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is True, r
    assert client.post("/autonomy/resume").json()["resumed"] is False


@pytest.fixture
def unpaced_config(tmp_path):
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_interval_s: 0.0\n")
    return str(config)


def _wait_finished(brain, seconds=60.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        status = brain.get("/mission/status").json()
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            return status
        time.sleep(0.05)
    raise AssertionError(f"the mission never finished: {status}")


def test_a_new_mission_lifts_a_persons_stop(plain, unpaced_config):
    from control.brain_server import create_app as brain_app

    robot_client = plain
    robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
    app = brain_app(config_path=unpaced_config,
                    robot_factory=lambda: RemoteRobot(ASGI_BASE_URL, client=robot_client))
    with TestClient(app) as brain:
        r = brain.post("/mission/start", json={"target_object": "red backpack",
                                                "max_steps": BUDGET})
        assert r.status_code == 200, r.text
        status = _wait_finished(brain)
    # It ran: never refused by the Stop pressed before it started.
    assert status["outcome"] != CANCELLED and status["step"] > 10, status


def test_resume_on_a_robot_server_older_than_3_84_answers_false():
    import httpx

    calls = []

    def handler(request):
        calls.append(request.url.path)
        return httpx.Response(404, json={"detail": "Not Found"})

    robot = RemoteRobot("http://old", client=httpx.Client(transport=httpx.MockTransport(handler)))
    assert robot.resume_autonomy() is False
    assert calls == ["/autonomy/resume"]


# ---------- criteria 5-6: a verb lifts the Stop's hold only under its lock ----------

import sys  # noqa: E402
import threading  # noqa: E402

from tests.test_refused_goal_hold import START, RefusingBridge  # noqa: E402
from tests.test_ros_goals import world_with  # noqa: E402
from tests.test_ros_world import FakeTruth  # noqa: E402


@pytest.fixture
def ros_goals(monkeypatch):
    """drive: ros (tests/test_ros_fallback.py's Stack, the ROS chain alive)
    over a world whose goals go to 3.82's fake bridge."""
    from tests.test_ros_fallback import Stack

    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    bridge = RefusingBridge(start_truth=START)
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    stack = Stack(monkeypatch)
    stack.ros_up()
    time.sleep(0.2)
    try:
        yield stack, bridge
    finally:
        stack.close()


def _nav2_applied(stack):
    """One nav2 command for its goal, as picar_sim_hardware relays it: what
    the robot server applied (a hold applies it as zero)."""
    r = stack.client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                          headers={"x-driver": "ros"}).json()
    return max(abs(r["applied"]["left_rad_s"]), abs(r["applied"]["right_rad_s"]))


def _nav2_for(stack, seconds):
    worst, end = 0.0, time.monotonic() + seconds
    while time.monotonic() < end:
        worst = max(worst, _nav2_applied(stack))
        time.sleep(0.05)
    return worst


def test_a_verb_arbitrated_before_a_stop_does_not_lift_its_hold(ros_goals, monkeypatch):
    """`_carry_out` cleared `goal_stop["hold"]` bare, after arbitration. A
    person's Stop that set the hold between the verb's arbitration and that
    clear was lost: nav2's `ros` commands for the goal the Stop was ending
    passed while its loop was still cancelling."""
    stack, bridge = ros_goals
    assert stack.client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    bridge.cancel_fails = True                      # the Stop's loop keeps going
    inside, release = threading.Event(), threading.Event()
    real_bridge_up = stack.robot.bridge_up

    def bridge_up():
        # Block the verb's own liveness check: after arbitration, before the clear.
        if (sys._getframe(1).f_code.co_name == "ros_up"
                and sys._getframe(2).f_code.co_name == "_carry_out"
                and not inside.is_set()):
            inside.set()
            release.wait(5.0)
        return real_bridge_up()

    monkeypatch.setattr(stack.robot, "bridge_up", bridge_up)
    reply = {}
    verb = threading.Thread(target=lambda: reply.setdefault(
        "r", stack.act("LEFT", angle=20)))
    verb.start()
    try:
        assert inside.wait(5.0), "the verb never reached the drive branch"
        assert stack.client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
    finally:
        release.set()
    verb.join(10.0)
    assert _nav2_for(stack, 0.5) == 0.0, "a verb from before the Stop lifted its hold"
    assert bridge.goal is not None, "the goal was cancelled anyway -- the test proves nothing"


def test_a_persons_verb_during_the_stops_loop_drives_then_the_hold_returns(ros_goals):
    """A person's own verb after their Stop always works; the hold it lifts
    for its twists comes back when it ends, while the Stop's loop is still
    ending the goal, so nav2 never drives that goal after the person's move."""
    stack, bridge = ros_goals
    assert stack.client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    bridge.cancel_fails = True
    assert stack.client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
    time.sleep(0.3)
    r = stack.act("LEFT", angle=45)
    assert r["executed"] is True, r
    assert abs(r["result"]["turned_deg"]) > 30, r
    assert _nav2_for(stack, 0.5) == 0.0, "nav2 drove the stopped goal after the person's verb"
    bridge.cancel_fails = False                     # the cancels land; the loop settles
    end, lifted = time.monotonic() + 3.0, False
    while time.monotonic() < end and not lifted:
        lifted = _nav2_applied(stack) > 0.0
        time.sleep(0.05)
    assert bridge.goal is None, "the Stop's loop never ended the goal"
    assert lifted, "the hold was never lifted after the Stop's loop ended the goal"
