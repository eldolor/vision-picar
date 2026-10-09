"""
tests/test_arrival_reconfirm.py

3.53 (`docs/plans/ros-alignment/3.53-arrival-reconfirm.md`): **after an
`arrived_unconfirmed` ending, ask the identity question again once the cloud
is back** -- free /health probes, ONE paid call on the stored arrival frame,
the answer recorded beside the outcome and never instead of it, the robot
never touched.

Through the real mission path (`MissionRunner` -> agent -> `robot/safety.py`
-> `MockRobot`, scaled house, async tier), with a cloud that is down until
the mission has ended and then comes back.
"""

import logging
import math
import time

import pytest
from fastapi.testclient import TestClient

from brain.navigate import CloudUnavailable
from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control.mission_runner import ARRIVED_UNCONFIRMED, MissionRunner
from control.reconfirm import Reconfirmer
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_arrival_confirmation import _disagreeing_cloud
from tests.test_bearing_turns import CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


class SwitchableCloud:
    """Down (CloudUnavailable, as brain/navigate.py raises) until `up` is
    set; then answers as `answer` does. Counts every call that goes out."""

    def __init__(self, answer=_quiet_cloud):
        self.up = False
        self.answer = answer
        self.calls = 0
        self.calls_while_up = 0

    def __call__(self, frame):
        self.calls += 1
        if not self.up:
            raise CloudUnavailable("ConnectError: no network")
        self.calls_while_up += 1
        return self.answer(frame)


class RobotSpy:
    """Counts every public method called on the raw robot from here on."""

    def __init__(self, robot):
        self.calls = []
        for name in dir(robot):
            if name.startswith("_"):
                continue
            fn = getattr(robot, name)
            if callable(fn):
                setattr(robot, name, self._wrap(name, fn))

    def _wrap(self, name, fn):
        def wrapped(*a, **k):
            self.calls.append(name)
            return fn(*a, **k)
        return wrapped


def _ended_unconfirmed(cloud, start, async_cloud=True):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, steer_on_sight=True,
                        hold_goal=True, async_cloud=async_cloud)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["outcome"] == ARRIVED_UNCONFIRMED, runner.status()["log_tail"][-2:]
    return runner, robot, grid


class FakeClockReconfirmer(Reconfirmer):
    """Waits advance a fake clock instead of sleeping."""

    def __init__(self, *a, **k):
        self.now = 0.0
        super().__init__(*a, clock=lambda: self.now, **k)

    def _wait(self, seconds):
        self.now += seconds
        return self._cancelled.is_set()


@pytest.mark.parametrize("answer,state,confirmed", [
    (_quiet_cloud, "confirmed", True),
    (_disagreeing_cloud, "refused", False),
], ids=["sees-it", "does-not"])
def test_1_3_it_asks_once_when_the_cloud_comes_back(answer, state, confirmed):
    """Criteria 1 and 3: on every clear start, exactly one paid call once
    the cloud is back; the verdict lands beside an unchanged outcome; no
    robot method is called during the late phase."""
    for start in CLEAR_STARTS:
        cloud = SwitchableCloud(answer)
        runner, robot, grid = _ended_unconfirmed(cloud, start)
        pose = (grid.x, grid.y, grid.theta)
        spy = RobotSpy(runner.robot)
        down_probes = 3
        probes = iter([False] * down_probes + [True] * 10)

        def probe():
            up = next(probes)
            cloud.up = up
            return up
        FakeClockReconfirmer(runner, probe, interval_s=15, window_s=600).run()
        status = runner.status()
        late = status["late_confirmation"]
        assert late["state"] == state, (start, late)
        assert late["confirmed"] is confirmed
        assert late["paid_calls"] == 1 and cloud.calls_while_up == 1, (start, late)
        assert late["probes"] == down_probes + 1
        assert status["outcome"] == ARRIVED_UNCONFIRMED and not runner.memory.found
        assert status["found"] is False
        assert spy.calls == [], f"the late phase touched the robot: {spy.calls}"
        assert (grid.x, grid.y, grid.theta) == pose
        assert status["tier"]["stats"]["triggers"][TRIGGER_ARRIVAL] >= 2


def test_2_probing_is_free_and_bounded():
    """Criterion 2: the cloud never comes back -- 0 paid calls, probes
    within window / interval + 1, then `expired`."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    before = cloud.calls
    rc = FakeClockReconfirmer(runner, lambda: False, interval_s=15, window_s=600)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "expired", late
    assert late["paid_calls"] == 0 and cloud.calls == before
    assert late["probes"] <= 600 / 15 + 1, late


def test_2_health_up_but_navigate_failing_pays_twice_then_stops():
    """/health answers while Bedrock is down (a server-side outage): two
    paid calls, then `failed` -- and the second waits half the window, so
    the attempts span the outage rather than 15 s of it (second review)."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    before = cloud.calls
    rc = FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600)
    times = []
    real_ask = runner.late_ask

    def timed():
        times.append(rc.now)
        return real_ask()
    runner.late_ask = timed
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed", late
    assert late["paid_calls"] == 2 and cloud.calls - before == 2, late
    assert times[1] - times[0] >= 300, times


def test_2_a_first_failure_late_in_the_window_still_gets_its_second_try():
    """Third review: /health first answers at t=400 of 600 with Bedrock
    still failing; the retry is capped inside the window, not lost past
    it, and the ending says a paid call failed -- not 'unreachable'."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    rc = FakeClockReconfirmer(runner, lambda: rc.now >= 400, interval_s=15, window_s=600)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed" and late["paid_calls"] == 2, late
    assert "unreachable" not in late["reason"], late


def test_2_a_window_that_runs_out_after_a_failed_call_is_failed_not_expired():
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    rc = FakeClockReconfirmer(runner, lambda: 400 <= rc.now < 420,
                              interval_s=15, window_s=600)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed" and late["paid_calls"] == 1, late
    assert "window ran out" in late["reason"], late


def test_review_no_check_before_finish_has_completed():
    """Third review: `_finish` closes the policy and builds the metrics row
    after it clears `_running`; a check must not start in that gap."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    assert runner.late_confirmation_pending()
    runner.finished.clear()
    assert not runner.late_confirmation_pending()


def test_2_a_server_side_outage_that_clears_in_the_window_is_confirmed():
    """Bedrock down for four minutes behind a healthy /health: the first
    paid call fails, the second (half the window later) is answered."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    rc = FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600)

    def probe():
        cloud.up = rc.now >= 240
        return True
    rc.probe = probe
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 2, late


def test_4_cancelled_while_waiting_never_pays():
    """Criterion 4 at the loop: a cancel (new mission, or Stop) while the
    cloud is down; the cloud then comes back; nothing is asked."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    rc = Reconfirmer(runner, lambda: cloud.up, interval_s=0.02, window_s=30).start()
    time.sleep(0.1)
    before = cloud.calls
    rc.cancel("a new mission started")
    cloud.up = True
    rc.join(2)
    late = runner.status()["late_confirmation"]
    assert late["state"] == "dropped" and "new mission" in late["reason"], late
    assert cloud.calls == before


def test_only_an_unconfirmed_arrival_records_anything():
    """A runner that ended any other way has no late confirmation, and a
    cancel on it records nothing."""
    grid = _build()
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud))
    runner.start()
    runner.stop()
    assert not runner.late_confirmation_pending()
    Reconfirmer(runner, lambda: True, interval_s=0.01, window_s=1).cancel("x")
    assert runner.status()["late_confirmation"] is None


def _metrics_runner(cloud):
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=TieredVision(FrameReportedPipeline(TARGET), cloud,
                                                  steer_on_sight=True, hold_goal=True,
                                                  async_cloud=True),
                           world=mock_world_for(robot))
    runner.metrics_url = "http://metrics.invalid"
    return runner


def _wait_for(pred, timeout=5):
    deadline = time.time() + timeout
    while not pred():
        assert time.time() < deadline
        time.sleep(0.02)


def test_review_the_late_row_lands_after_the_original_and_counts_the_late_call(monkeypatch):
    """Coordinator's Thermos pass, finding 3: both rows write one key, so the
    late one must land second even when the original's ship is slow; and its
    stats must include the late call (cloud_calls), not the frozen row's."""
    import threading
    import control.metrics_client as mc
    landed = []

    def slow_async(url, row, **k):
        def go():
            time.sleep(0.5)
            landed.append(("original", row))
        t = threading.Thread(target=go, daemon=True)
        t.start()
        return t
    monkeypatch.setattr(mc, "ship_run_async", slow_async)
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: landed.append(("late", row)))
    cloud = SwitchableCloud()
    runner = _metrics_runner(cloud)
    runner.start()
    while runner.tick():
        pass

    def probe():
        cloud.up = True
        return True
    FakeClockReconfirmer(runner, probe, interval_s=15, window_s=600).run()
    _wait_for(lambda: len(landed) == 2)
    assert [k for k, _ in landed] == ["original", "late"], landed
    original, late = landed[0][1], landed[1][1]
    assert late["stats"]["cloud_calls"] == original["stats"]["cloud_calls"] + 1


def test_review_a_cancel_while_the_call_is_out_ships_the_call(monkeypatch):
    """Coordinator's Thermos pass, finding 2: a cancel that lands while the
    paid call is out ships the `dropped` record -- and that record must
    already count the call, or the stored metrics understate the spend."""
    import control.metrics_client as mc
    original, shipped = [], []
    monkeypatch.setattr(mc, "ship_run_async", lambda url, row, **k: original.append(row))
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: shipped.append(row))
    cloud = SwitchableCloud()
    runner = _metrics_runner(cloud)
    runner.start()
    while runner.tick():
        pass
    rc = FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600)
    real = runner.vision_fn.confirm_arrival

    def cancelled_while_out(frame):
        rc.cancel("a new mission started")  # lands with the call committed
        cloud.up = True
        return real(frame)
    runner.vision_fn.confirm_arrival = cancelled_while_out
    rc.run()
    _wait_for(lambda: len(shipped) == 2)
    first, last = shipped
    assert first["stats"]["late_confirmation"]["state"] == "dropped"
    assert first["stats"]["late_confirmation"]["paid_calls"] == 1, first
    # Re-sent once the call returned (fifth review): its counters include it.
    assert last["stats"]["late_confirmation"]["paid_calls"] == 1
    assert last["stats"]["cloud_calls"] == original[0]["stats"]["cloud_calls"] + 1, last


def test_review_a_final_record_and_its_stored_row_agree(monkeypatch):
    """Fifth review: at the cap, with a cancel landing while confirm_arrival
    runs, the take-back must not rewrite the final `dropped` record after
    its row was shipped -- the status and the stored row must agree."""
    import control.metrics_client as mc
    shipped = []
    monkeypatch.setattr(mc, "ship_run_async", lambda url, row, **k: None)
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: shipped.append(row))
    cloud = SwitchableCloud()
    runner = _metrics_runner(cloud)
    runner.start()
    while runner.tick():
        pass
    runner.vision_fn.max_calls = 0  # the cap: confirm_arrival makes no call
    rc = FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600)
    real = runner.vision_fn.confirm_arrival

    def cancelled_while_out(frame):
        rc.cancel("a new mission started")
        return real(frame)
    runner.vision_fn.confirm_arrival = cancelled_while_out
    rc.run()
    _wait_for(lambda: shipped)
    time.sleep(0.2)
    status_count = runner.status()["late_confirmation"]["paid_calls"]
    assert shipped[-1]["stats"]["late_confirmation"]["paid_calls"] == status_count
    # Sixth review: and both say what happened -- no call was made.
    assert status_count == 0, runner.status()["late_confirmation"]


def test_6_metrics_row_is_resent_not_added(monkeypatch):
    """Criterion 6: the late verdict re-sends the mission's own row (same
    run_id and finished_at) with stats.late_confirmation."""
    sent = []
    import control.metrics_client as mc
    monkeypatch.setattr(mc, "ship_run_async", lambda url, row, **k: sent.append(row))
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: sent.append(row))
    cloud = SwitchableCloud()
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=TieredVision(FrameReportedPipeline(TARGET), cloud,
                                                  steer_on_sight=True, hold_goal=True,
                                                  async_cloud=True),
                           world=mock_world_for(robot))
    runner.metrics_url = "http://metrics.invalid"
    runner.start()
    while runner.tick():
        pass
    assert len(sent) == 1 and sent[0]["outcome"] == ARRIVED_UNCONFIRMED

    def probe():
        cloud.up = True
        return True
    FakeClockReconfirmer(runner, probe, interval_s=15, window_s=600).run()
    _wait_for(lambda: len(sent) == 2)
    first, second = sent
    assert second["run_id"] == first["run_id"]
    assert second["finished_at"] == first["finished_at"]
    assert second["outcome"] == ARRIVED_UNCONFIRMED
    assert second["stats"]["late_confirmation"]["state"] == "confirmed"
    assert "late_confirmation" not in first["stats"]


# ---------------------------------------------------------------------------
# Criterion 4 through brain_server: a new mission and Stop both drop it.

@pytest.fixture
def reconfirm_app(tmp_path, monkeypatch, request):
    interval = getattr(request, "param", 0.0)
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  tick_interval_s: {interval}\n"
                      "  vision_url: http://127.0.0.1:9\n"
                      "  reconfirm_probe_s: 0.02\n  reconfirm_window_s: 30\n")
    cloud = SwitchableCloud()
    runners = []

    def runner_factory(robot, req):
        if not req.target_object:
            # As the real factory does: a 400, before any runner exists.
            raise ValueError("The tiered policy searches for an object")
        x, y, off = CLEAR_STARTS[0]
        grid = _build()
        grid.x, grid.y = x, y
        grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
        body = MockRobot(grid, render=False)
        runner = MissionRunner(body, target_object=TARGET, max_steps=60, policy="tiered",
                               vision_fn=TieredVision(FrameReportedPipeline(TARGET), cloud,
                                                      steer_on_sight=True, hold_goal=True,
                                                      async_cloud=True),
                               world=mock_world_for(body))
        runners.append(runner)
        return runner

    import control.brain_server as bs
    monkeypatch.setattr(bs, "health_probe", lambda *a, **k: (lambda: cloud.up))
    app = bs.create_app(config_path=str(config),
                        robot_factory=lambda: MockRobot(_build(), render=False),
                        runner_factory=runner_factory)
    return app, cloud, runners


def _wait_outcome(client, timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        status = client.get("/mission/status").json()
        if not status["running"]:
            return status
        time.sleep(0.05)
    pytest.fail("mission did not end")


def _waiting(client, runners):
    client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
    assert _wait_outcome(client)["outcome"] == ARRIVED_UNCONFIRMED
    deadline = time.time() + 5
    while (runners[0].status()["late_confirmation"] or {}).get("probes", 0) < 2:
        assert time.time() < deadline, "the late confirmation never started"
        time.sleep(0.02)


def _final(runner, timeout=5):
    deadline = time.time() + timeout
    while True:
        late = runner.status()["late_confirmation"]
        if late and late["state"] != "waiting":
            return late
        assert time.time() < deadline, late
        time.sleep(0.02)


def test_4_a_new_mission_drops_it(reconfirm_app):
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        _waiting(client, runners)
        assert client.post("/mission/start",
                           json={"target_object": TARGET, "policy": "tiered"}
                           ).status_code == 200
        cloud.up = True
        time.sleep(0.3)
        late = runners[0].status()["late_confirmation"]
        assert late["state"] == "dropped" and late["paid_calls"] == 0, late
        client.post("/mission/stop")


def test_4_amended_a_stop_after_the_ending_leaves_it_and_it_still_asks(reconfirm_app):
    """Amendment 1: the Guide tab sends Stop on every close; the check
    never moves the robot, so a Stop after the ending must not drop it."""
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        _waiting(client, runners)
        client.post("/mission/stop")
        cloud.up = True
        late = _final(runners[0])
        assert late["state"] == "confirmed" and late["paid_calls"] == 1, late


@pytest.mark.parametrize("reconfirm_app", [1.0], indirect=True)
def test_4_amended_a_stop_in_the_loops_last_sleep_still_starts_it(reconfirm_app):
    """Second review: the mission ends inside tick(); the loop then sleeps
    tick_interval_s before it would start the check. A Stop in that sleep
    used to cancel the loop and lose the check without a trace."""
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        assert _wait_outcome(client, timeout=120)["outcome"] == ARRIVED_UNCONFIRMED
        client.post("/mission/stop")  # inside the 1 s post-tick sleep
        cloud.up = True
        late = _final(runners[0])
        assert late["state"] == "confirmed", late


@pytest.mark.parametrize("reconfirm_app", [1.0], indirect=True)
def test_review_a_later_stop_never_starts_a_check_an_earlier_stop_forbade(reconfirm_app):
    """Third review: a Stop that landed while the runner was running marks
    it (`operator_stopped`); a later Stop -- the Guide tab's close -- must
    not start the check, even though the in-flight tick ended the mission
    `arrived_unconfirmed`. That interleaving is not reproducible through
    TestClient, so the first Stop's mark is set by hand; the second Stop
    lands in the loop's post-tick sleep, where only the Stop path could
    start a check."""
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        assert _wait_outcome(client, timeout=120)["outcome"] == ARRIVED_UNCONFIRMED
        runners[0].operator_stopped = True
        client.post("/mission/stop")
        cloud.up = True
        time.sleep(0.5)
        assert cloud.calls_while_up == 0
        assert runners[0].status()["late_confirmation"] is None


def _count_reconfirmers(monkeypatch):
    import control.brain_server as bs
    made = []

    class Counting(Reconfirmer):
        def __init__(self, runner, *a, **k):
            made.append(runner)
            super().__init__(runner, *a, **k)
    monkeypatch.setattr(bs, "Reconfirmer", Counting)
    return made


async def _ended_over_asgi(client):
    await client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
    import asyncio
    for _ in range(2400):
        status = (await client.get("/mission/status")).json()
        if not status["running"]:
            assert status["outcome"] == ARRIVED_UNCONFIRMED
            return
        await asyncio.sleep(0.05)
    pytest.fail("mission did not end")


@pytest.mark.parametrize("reconfirm_app", [1.0], indirect=True)
def test_review_two_stops_at_once_start_one_check(reconfirm_app, monkeypatch):
    """Coordinator's Thermos pass, finding 1a: two Stops inside the loop's
    post-tick sleep both saw an empty slot and both started a check -- the
    first orphaned, both able to pay at once. The slot is claimed before
    the first await."""
    import asyncio
    import httpx
    made = _count_reconfirmers(monkeypatch)
    app, cloud, runners = reconfirm_app

    async def go():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),
                                     base_url="http://brain") as client:
            await _ended_over_asgi(client)
            await asyncio.gather(client.post("/mission/stop"), client.post("/mission/stop"))
            cloud.up = True
            await asyncio.sleep(0.5)
    asyncio.run(go())
    assert len(made) == 1, f"{len(made)} late checks started"
    assert cloud.calls_while_up == 1


@pytest.mark.parametrize("reconfirm_app", [1.0], indirect=True)
def test_review_a_start_during_a_stop_never_checks_the_old_mission(reconfirm_app, monkeypatch):
    """Coordinator's Thermos pass, finding 1b: a Start that lands while a
    Stop is awaiting must leave the Stop unable to start a check for the
    OLD runner, which would pay while the new mission drives. The Stop
    reads the generation at entry."""
    import asyncio
    import httpx
    made = _count_reconfirmers(monkeypatch)
    app, cloud, runners = reconfirm_app

    class SlowFinished:
        def wait(self, timeout=None):
            time.sleep(1.0)  # the Stop is parked here while the Start runs
            return True

        def is_set(self):
            return True

    async def go():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),
                                     base_url="http://brain") as client:
            await _ended_over_asgi(client)
            runners[0].finished = SlowFinished()

            async def start_later():
                await asyncio.sleep(0.3)
                return await client.post("/mission/start",
                                         json={"target_object": TARGET, "policy": "tiered"})
            stop, start = await asyncio.gather(client.post("/mission/stop"), start_later())
            assert start.status_code == 200, start.text
            await asyncio.sleep(0.5)
            await client.post("/mission/stop")
    asyncio.run(go())
    assert runners[0] not in made, "a late check was started for the old mission"


def test_4_amended_a_refused_start_leaves_it(reconfirm_app):
    """From review: a start refused with 400 must not drop the old check."""
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        _waiting(client, runners)
        assert client.post("/mission/start", json={"policy": "tiered"}).status_code == 400
        cloud.up = True
        assert _final(runners[0])["state"] == "confirmed"


def test_review_a_cancel_that_lands_before_the_call_stops_it():
    """From review: the cancel recorded between the probe and the call --
    `late_ask()` checks the record under the lock and sends nothing."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    before = cloud.calls
    rc = FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600)
    real_ask = runner.late_ask

    def cancelled_first():
        rc.cancel("a new mission started")
        cloud.up = True
        return real_ask()
    runner.late_ask = cancelled_first
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "dropped" and late["paid_calls"] == 0, late
    assert cloud.calls == before


def test_review_a_spent_cap_counts_no_paid_call():
    """From review: a cap refusal makes no call and must not be counted."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_fn.max_calls = 0

    def probe():
        cloud.up = True
        return True
    FakeClockReconfirmer(runner, probe, interval_s=15, window_s=600).run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "not_asked" and late["paid_calls"] == 0, late


def test_review_a_hung_call_is_never_doubled():
    """From review: a confirmation that times out is still in flight; no
    second one goes out beside it -- `failed` after exactly one."""
    import threading
    release = threading.Event()
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.2
    hung = {"n": 0}

    def hang(frame):
        hung["n"] += 1
        release.wait(5)
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = hang
    FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600).run()
    release.set()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed" and "hung" in late["reason"], late
    assert hung["n"] == 1 and late["paid_calls"] == 1, (hung, late)


def test_7_the_brain_asks_when_the_cloud_comes_back(reconfirm_app):
    """The same flow through brain_server's own loop, which criterion 7
    then runs live over HTTP."""
    app, cloud, runners = reconfirm_app
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        assert _wait_outcome(client)["outcome"] == ARRIVED_UNCONFIRMED
        time.sleep(0.1)
        cloud.up = True
        deadline = time.time() + 5
        while True:
            late = client.get("/mission/status").json()["late_confirmation"]
            if late and late["state"] != "waiting":
                break
            assert time.time() < deadline, late
            time.sleep(0.02)
        assert late["state"] == "confirmed" and late["paid_calls"] == 1, late
        assert client.get("/mission/status").json()["outcome"] == ARRIVED_UNCONFIRMED
