"""
tests/test_review_376k2.py

`PLAN-ros-alignment.md` 3.76, step K2: the bugs the review of `control/` as
other sessions changed it after batches C and F-G (3.70 S1-S5, 3.77, 3.80
C-P, 3.81.E, 3.82, 3.84) confirmed. Each test was written before its fix
and seen to fail on `dev` for the reason it names.
"""

import threading
import time

import pytest

from control.brain_server import create_app
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from sim.mock_robot import MockRobot
from tests.conftest import ASGI_BASE_URL
from tests.test_arrival_reconfirm import (
    SwitchableCloud, _metrics_runner, _quiet_cloud, _wait_for)
from tests.test_brain_server import RecordingRobot, fresh_mock_robot, slow_runner_factory
from tests.test_cloud_health import TARGET, _build
from tests.test_review_376k1 import plain  # noqa: F401 -- the robot server fixture

from fastapi.testclient import TestClient


# ---------- K2.1: a Stop during the arrival's waits still bought a cloud call ----------


@pytest.mark.parametrize("wait", ["inflight", "previous confirmation"])
def test_a_stop_during_the_arrivals_wait_pays_for_no_confirmation(wait):
    """Only a local-led arrival re-read the mission after its waits. On the
    plain path (no local tier) a Stop while the confirmation waited for a
    trigger call still out, or for the last confirmation, was followed by a
    paid cloud call for a mission that had ended."""
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           vision_timeout_s=2.0, arrival_confirm_timeout_s=1.0)
    asked = []

    class Policy:
        def wait_inflight(self):
            if wait == "inflight":
                runner.stop("the operator pressed Stop")

        def confirm_arrival(self, frame):
            asked.append(frame)
            return {"confirmed": True, "cloud_called": True}

        def close(self):
            pass

    runner.vision_fn = Policy()
    runner.start()
    if wait == "previous confirmation":
        def out():
            time.sleep(0.2)
            runner.stop("the operator pressed Stop")
        prev = threading.Thread(target=out, daemon=True)
        prev.start()
        runner._confirm_thread = prev
    verdict = runner._guarded_confirm({})
    assert asked == [], "a cloud confirmation was sent after the Stop"
    assert verdict["cloud_called"] is False, verdict


# ---------- K2.2: a refused start lifted a person's Stop ----------


def _brain_over(robot_client, **kw):
    return create_app(robot_factory=lambda: RemoteRobot(ASGI_BASE_URL, client=robot_client),
                      **kw)


def _brain_refused(robot_client):
    reply = robot_client.post("/action", json={"action": "LOOK_LEFT"},
                              headers={"x-driver": "brain"}).json()
    return reply.get("reason") == "person_stop"


@pytest.mark.parametrize("request_body, brain_yaml, status", [
    ({"target_object": "red backpack", "policy": "vision"}, "", 400),      # no vision_url
    ({"target_object": "red backpack", "fault": "vision_error"},
     "brain:\n  allow_drills: false\n", 403),                             # drills off
])
def test_a_refused_start_leaves_a_persons_stop_latched(plain, tmp_path, request_body,  # noqa: F811
                                                       brain_yaml, status):
    """The latch is lifted first, before the build; a start then refused
    (400, 403) began no mission but left autonomy unlatched after the
    person's D-pad Stop."""
    config = tmp_path / "robot.yaml"
    config.write_text(brain_yaml)
    plain.post("/stop", headers={"x-driver": "twin-dpad"})
    with TestClient(_brain_over(plain, config_path=str(config))) as brain:
        r = brain.post("/mission/start", json=request_body)
    assert r.status_code == status, r.text
    assert _brain_refused(plain), "a refused start lifted the person's Stop"


def test_a_start_stopped_during_its_build_leaves_a_persons_stop_latched(plain):  # noqa: F811
    plain.post("/stop", headers={"x-driver": "twin-dpad"})
    inner = slow_runner_factory(delay=0.1)
    building = threading.Event()

    def slow(robot, req):
        building.set()
        time.sleep(0.5)
        return inner(robot, req)

    with TestClient(_brain_over(plain, runner_factory=slow)) as brain:
        reply = {}
        t = threading.Thread(target=lambda: reply.setdefault(
            "r", brain.post("/mission/start", json={"target_object": "red backpack"})))
        t.start()
        # Inside the build, not a fixed sleep: under the full suite's load a
        # Stop at 0.2 s landed before the start reached it (the gate, M2).
        assert building.wait(10)
        brain.post("/mission/stop")
        t.join()
    assert reply["r"].status_code == 409, reply["r"].text
    assert _brain_refused(plain), "a start stopped during its build lifted the person's Stop"


def test_a_started_mission_still_lifts_it(plain):  # noqa: F811
    plain.post("/stop", headers={"x-driver": "twin-dpad"})
    with TestClient(_brain_over(plain, runner_factory=slow_runner_factory(delay=0.1))) as brain:
        r = brain.post("/mission/start", json={"target_object": "red backpack"})
        assert r.status_code == 200, r.text
        assert not _brain_refused(plain)
        brain.post("/mission/stop")


# ---------- K2.3: a start during a stopped build's discard skipped 3.81.E's wait ----------


def test_a_start_during_a_discard_is_refused():
    """`building` was cleared before the stopped build's runner was
    discarded, so a second start in that await passed the 409 check, and
    its wait for the previous close read the OLD runner: two perception
    pipelines at once (3.81.E)."""
    inner = slow_runner_factory(delay=0.1)
    discarding = threading.Event()
    building = threading.Event()

    def factory(robot, req):
        building.set()
        time.sleep(0.4)
        runner = inner(robot, req)
        real = runner.discard

        def slow_discard():
            discarding.set()
            time.sleep(0.6)
            real()
        runner.discard = slow_discard
        return runner

    app = create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()),
                     runner_factory=factory)
    with TestClient(app) as client:
        first = threading.Thread(target=lambda: client.post(
            "/mission/start", json={"target_object": "red backpack"}))
        first.start()
        assert building.wait(10)               # inside the build (as above)
        client.post("/mission/stop")
        assert discarding.wait(3)
        second = client.post("/mission/start", json={"target_object": "red backpack"})
        first.join()
        client.post("/mission/stop")
    assert second.status_code == 409, second.text


# ---------- K2.5 (not a bug; a guard): a late call ending at its guard ----------


def test_a_late_call_ending_at_its_guard_after_a_cancel_ships_its_cost(monkeypatch):
    """The claim: `ask()`'s finally re-ships only if the guard already marked
    the call abandoned, so a call ending between the guard's give-up and its
    lock left the final row without the call's cloud time. Passes on `dev`:
    `late_ask()`'s own finally re-ships a final record after the guard, with
    the policy's counters as they are then."""
    import control.metrics_client as mc
    import control.mission_runner as mr
    original, shipped = [], []
    monkeypatch.setattr(mc, "ship_run_async", lambda url, row, **k: original.append(row))
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: shipped.append(row))
    cloud = SwitchableCloud()
    runner = _metrics_runner(cloud)
    runner.start()
    while runner.tick():
        pass
    cloud.up = True

    def cancelled_meanwhile(frame):
        runner.late_update(state="dropped", reason="a new mission started")
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = cancelled_meanwhile
    hold = threading.Event()

    def at_the_edge(fn, *args, timeout_s=None, on_start=None):
        done = threading.Event()

        def body():
            try:
                fn(*args)
            finally:
                done.set()
                hold.wait(5)      # the thread is still alive at the guard's look
        t = threading.Thread(target=body, daemon=True)
        if on_start is not None:
            on_start(t)
        t.start()
        done.wait(5)
        raise TimeoutError(f"call exceeded {timeout_s}s")
    monkeypatch.setattr(mr, "call_with_timeout", at_the_edge)
    try:
        with pytest.raises(TimeoutError):
            runner.late_ask()
    finally:
        hold.set()

    def timed(row):
        return (row["stats"].get("cloud_ms") or {}).get("n", 0)
    _wait_for(lambda: any(timed(r) > timed(original[0]) for r in shipped), timeout=3)
