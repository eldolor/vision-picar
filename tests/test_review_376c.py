"""
tests/test_review_376c.py

`PLAN-ros-alignment.md` 3.76, batch C: the bugs the whole-file review of
the mission loop (`control/mission_runner.py`, `control/brain_server.py`,
`control/remote_*.py`) confirmed. Each test was written before its fix and
seen to fail on `dev` for the reason it names.
"""

import threading
import time

import pytest
from fastapi.testclient import TestClient

from control.brain_server import create_app
from control.mission_runner import CANCELLED, MissionRunner
from robot.interface import Preempted
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.conftest import RecordingRobot, fresh_mock_robot
from tests.test_brain_server import slow_runner_factory
from tests.test_explore import FakeNav, _agent_with_refusal


class _StuckNav(FakeNav):
    """nav2 holding a goal it never finishes: what Stop has to end."""

    def get_goal(self):
        self.calls.append(("get",))
        return {"goal": self.goal, "plan": []}


def _explore_runner(nav_cls=_StuckNav, **nav_kw):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = nav_cls(robot, **nav_kw)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore",
                           navigator=nav, world=MockWorld(grid), max_steps=400,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    return runner, nav


def _soon(cond, timeout=2.0):
    """The goal is ended on a thread of its own (a Stop runs `_finish` on
    the brain's event loop)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.01)
    return cond()


def _tick_until(runner, cond, limit=400):
    runner.start()
    for _ in range(limit):
        if cond() or not runner.tick():
            break
    return cond()


# ---------- C1: a Stop ends the explore mission's own nav2 goal ----------


def test_stop_during_explore_cancels_the_missions_own_goal():
    """The runner stops the robot as driver `brain`, and the robot server
    spares the goal on a brain's stop (it reads one as a loser's teardown).
    Under explore the goal IS the brain's: nav2 drove on after Stop, feeding
    the watchdog, while the twin said the robot had stopped."""
    runner, nav = _explore_runner()
    assert _tick_until(runner, lambda: nav.goal is not None), "explore never sent a goal"
    runner.stop()
    assert _soon(lambda: nav.goal["state"] == "canceled"), \
        f"the goal is still {nav.goal['state']} after Stop"


def test_a_person_s_goal_is_not_cancelled_by_the_missions_end():
    """The cancel is for the mission's OWN goal: a goal someone else set
    since (a tap on the map) is not the mission's to end."""
    runner, nav = _explore_runner()
    assert _tick_until(runner, lambda: nav.goal is not None)
    nav.goal = {"state": "active", "x_m": 0.55, "y_m": 0.55}      # a person's tap
    runner.stop()
    time.sleep(0.3)                                    # the mission's end has run
    assert nav.goal["state"] == "active"


# ---------- C3: no goal is sent, or left, after the mission ended ----------


class _StopsDuringSend(_StuckNav):
    """A Stop landing while a tick's goal is on its way to nav2."""

    runner = None

    def set_goal(self, x_m, y_m):
        reply = super().set_goal(x_m, y_m)
        if self.runner is not None and self.runner.is_running():
            self.runner.stop()
        return reply


def test_a_goal_that_lands_after_the_stop_is_cancelled():
    runner, nav = _explore_runner(_StopsDuringSend)
    nav.runner = runner
    _tick_until(runner, lambda: not runner.is_running())
    assert nav.goal is not None, "no goal was ever sent -- the test proves nothing"
    assert _soon(lambda: nav.goal["state"] == "canceled"), "the in-flight goal drove on after the stop"


def test_no_goal_is_sent_once_the_mission_has_ended():
    runner, nav = _explore_runner()
    runner.start()
    runner.stop()
    sent = len([c for c in nav.calls if c[0] == "set"])
    runner.agent.navigator.set_goal(1.0, 1.0)          # a tick thread still inside its step
    assert len([c for c in nav.calls if c[0] == "set"]) == sent


class _SlowCloseableNav(_StuckNav):
    """A real navigator's two traits the in-process fake lacked: a client
    that can be closed, and a goal POST that takes time."""

    def __init__(self, robot):
        super().__init__(robot)
        self.closed = False
        self.inside, self.release = threading.Event(), threading.Event()

    def set_goal(self, x_m, y_m):
        self.inside.set()
        self.release.wait(5.0)
        return super().set_goal(x_m, y_m)

    def get_goal(self):
        if self.closed:
            raise RuntimeError("Cannot send a request, as the client has been closed")
        return super().get_goal()

    def cancel_goal(self):
        if self.closed:
            raise RuntimeError("Cannot send a request, as the client has been closed")
        return super().cancel_goal()

    def close(self):
        self.closed = True


def test_a_goal_on_its_way_when_stop_lands_on_another_thread_is_still_cancelled():
    """The fixes' own review: `_finish` closed the client at once, so a goal
    accepted just after Stop -- on the tick thread -- could not cancel
    itself, and nav2 drove on."""
    runner, nav = _explore_runner(_SlowCloseableNav)
    runner.start()
    tick = threading.Thread(target=runner.agent.navigator.set_goal, args=(1.5, 1.5))
    tick.start()
    assert nav.inside.wait(2.0)
    runner.stop()                                      # Stop, on another thread
    time.sleep(0.1)                                    # the mission's end runs
    nav.release.set()                                  # ...then nav2 accepts the goal
    tick.join(2.0)
    assert _soon(lambda: nav.goal["state"] == "canceled"), "the goal accepted after Stop drove on"
    assert _soon(lambda: nav.closed), "the client was never closed"


# ---------- C4: a person at the D-pad ends an explore mission ----------


class _PersonHolds(_StuckNav):
    def set_goal(self, x_m, y_m):
        self.calls.append(("set", round(x_m, 2), round(y_m, 2)))
        return {"accepted": False, "reason": "preempted",
                "detail": "twin-dpad is driving -- ros is lower priority and was refused"}


def test_a_person_at_the_dpad_ends_an_explore_mission():
    """A goal refused because a person holds the robot read as "wait and
    ask again", so the mission resumed under the person the moment their
    authority lapsed (AGENT-HARNESS.md section 4: a person ends it)."""
    runner, nav = _explore_runner(_PersonHolds)
    _tick_until(runner, lambda: not runner.is_running(), limit=60)
    st = runner.status()
    assert st["outcome"] == CANCELLED and st.get("cancelled_by") == "dpad", st


# ---------- C5: the refusals the mission must read correctly ----------


def test_explore_waits_while_its_own_last_goal_still_holds_the_robot():
    """Right after explore's goal ends, `ros` still holds authority for up
    to `watchdog_timeout_s` from nav2's last wheel post. A pan then is
    refused "ros is driving -- one autonomous driver at a time", which
    ended the mission `failed` as an arbitration fault."""
    agent = _agent_with_refusal("ros is driving -- one autonomous driver at a time, "
                                "so brain was refused until it lapses")
    action, executed, _ = agent._do_pending()
    assert action == "WAIT" and agent._pending == ["LOOK_LEFT"]


def test_a_take_over_refusal_names_who_took_over():
    """3.76 T1's refusal reads "X took over"; the preemption parser knew
    only "X is driving", so it credited every take-over to a person."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot, target_object="red backpack", max_steps=10)
    runner.start()
    runner._finish_preempted(Preempted("ros took over -- brain's FORWARD was cut off"))
    st = runner.status()
    assert st["outcome"] == "failed", st


# ---------- C2: one mission at a time, even while one is being built ----------


def _slow_build(delay):
    inner = slow_runner_factory(delay=0.1)

    def factory(robot, req):
        time.sleep(delay)                              # a model load, a vision service check
        return inner(robot, req)

    return factory


def test_two_starts_during_one_build_start_one_mission():
    """`start_mission` checked for a running mission, then awaited the build
    (seconds: a model load, a GET to the vision service), then installed
    whatever it built. Two starts in that window both passed: two loops on
    one robot, and Stop cancelled only the second."""
    app = create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()),
                     runner_factory=_slow_build(0.5))
    with TestClient(app) as client:
        codes = []

        def start():
            codes.append(client.post("/mission/start",
                                     json={"target_object": "red backpack"}).status_code)

        threads = [threading.Thread(target=start) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        client.post("/mission/stop")
        assert sorted(codes) == [200, 409], codes


def test_a_stop_during_the_build_means_the_mission_never_starts():
    app = create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()),
                     runner_factory=_slow_build(0.5))
    with TestClient(app) as client:
        reply = {}
        t = threading.Thread(target=lambda: reply.setdefault(
            "start", client.post("/mission/start", json={"target_object": "red backpack"})))
        t.start()
        time.sleep(0.2)
        client.post("/mission/stop")
        t.join()
        time.sleep(0.2)
        assert client.get("/mission/status").json()["running"] is False, \
            "the mission started after the operator pressed Stop"
        client.post("/mission/stop")


def test_the_robot_server_answers_ros_is_driving_just_after_nav2_drove(monkeypatch):
    """The server half of C5: nav2's last non-zero wheel post leaves `ros`
    holding the robot, so the brain's next move -- explore's pan after its
    goal -- is refused in exactly the words explore now waits on."""
    import robot.server as server
    from brain.explore import OWN_GOAL_HOLDER
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = TestClient(server.create_app())
    client.post("/wheels", json={"left_rad_s": 1.0, "right_rad_s": 1.0},
                headers={"x-driver": "ros"})
    client.post("/wheels", json={"left_rad_s": 0.0, "right_rad_s": 0.0},
                headers={"x-driver": "ros"})             # the goal ends
    reply = client.post("/action", json={"action": "LOOK_LEFT"},
                        headers={"x-driver": "brain"}).json()
    assert reply["executed"] is False and reply["detail"].startswith(OWN_GOAL_HOLDER), reply


# ---------- C6: the lows that held ----------


def test_an_explore_start_against_an_unreachable_robot_says_so(tmp_path, monkeypatch):
    """`RemoteNavigator.available()` raised httpx's error, which is not a
    ValueError: a bare 500."""
    monkeypatch.delenv("ROBOT_URL", raising=False)
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  world_url: http://127.0.0.1:9\n")     # discard: nothing answers
    app = create_app(str(config), robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        r = client.post("/mission/start", json={"target_object": "red backpack",
                                                "policy": "explore"})
    assert r.status_code == 400 and "could not reach" in r.json()["detail"], r.text


def test_a_wrong_secret_is_not_reported_as_no_nav2():
    import httpx
    from control.remote_navigator import RemoteNavigator
    nav = RemoteNavigator("http://robot", client=httpx.Client(
        base_url="http://robot", transport=httpx.MockTransport(lambda req: httpx.Response(401))))
    with pytest.raises(PermissionError):
        nav.available()
