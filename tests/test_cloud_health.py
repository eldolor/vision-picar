"""
tests/test_cloud_health.py

3.56 (`docs/plans/ros-alignment/3.56-cloud-health.md`): **the operator sees
the cloud, and a hanging cloud parks the robot sooner.**

(a) `status.cloud`: the vision service's free `/health`, probed by one
`CloudWatch` per brain only while a cloud-policy mission runs, and by 3.53's
late check afterwards -- one prober at a time, never the robot, never B3.2.

(b) `arrival_confirm_timeout_s` (default 8 s, the user's choice): the
arrival confirmation's own deadline, above the slowest cloud call recorded.

Through the real path where it matters (`MissionRunner` -> agent ->
`robot/safety.py` -> `MockRobot`, scaled house; brain_server over a
TestClient), with the cloud faked.
"""

import logging
import math
import threading
import time

import pytest
import yaml
from fastapi.testclient import TestClient

from brain.navigate import CloudUnavailable
from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control import reconfirm
from control.brain_config import DEFAULTS, load_brain_config
from control.mission_runner import (
    ARRIVED_UNCONFIRMED, DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S, FOUND, MissionRunner)
from control.reconfirm import REACHABLE, UNKNOWN, UNREACHABLE, CloudWatch
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_arrival_reconfirm import RobotSpy, SwitchableCloud
from tests.test_bearing_turns import ARRIVED_CELLS, CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud

# The slowest /navigate call in the recorded walks (3.56's table): 98 calls.
SLOWEST_RECORDED_S = 6.783


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def _placed(start=CLEAR_STARTS[0]):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    return grid


# ---------------------------------------------------------------------------
# Criterion 1: honest states.

def test_1_unknown_then_reachable_then_unreachable_and_since_moves_only_on_a_change():
    t = {"now": 100.0}
    watch = CloudWatch(lambda: True, interval_s=15, active=lambda: False,
                       wall=lambda: t["now"])
    assert watch.snapshot()["state"] == UNKNOWN and watch.snapshot()["since"] is None
    watch.record(True)
    assert watch.snapshot()["state"] == REACHABLE and watch.snapshot()["since"] == 100.0
    t["now"] = 115.0
    watch.record(True)
    snap = watch.snapshot()
    assert snap["since"] == 100.0 and snap["checked_at"] == 115.0, snap
    t["now"] = 130.0
    watch.record(False)
    assert watch.snapshot()["state"] == UNREACHABLE and watch.snapshot()["since"] == 130.0


def test_1_a_probe_that_raises_reads_unreachable():
    def boom():
        raise RuntimeError("dns")
    watch = CloudWatch(boom, interval_s=15, active=lambda: False)
    assert watch.recording(boom)() is False
    assert watch.snapshot()["state"] == UNREACHABLE


def test_1_health_probe_only_ever_asks_health(monkeypatch):
    """Free: the probe's URL is /health, never /navigate."""
    seen = []

    class R:
        status_code = 503

    monkeypatch.setattr(reconfirm.httpx, "get", lambda url, **k: seen.append(url) or R())
    assert reconfirm.health_probe("http://cloud/x/")() is False
    assert seen == ["http://cloud/x/health"]


# ---------------------------------------------------------------------------
# Criterion 2: free and bounded -- probes only while a cloud mission runs.

def test_2_the_watch_probes_only_while_active_and_no_faster_than_its_interval():
    calls = []
    active = threading.Event()
    interval = 0.05
    watch = CloudWatch(lambda: calls.append(time.monotonic()) or True,
                       interval_s=interval, active=active.is_set).start()
    try:
        time.sleep(0.3)
        assert calls == [], "probed while no cloud mission ran"
        active.set()
        began = time.monotonic()
        time.sleep(0.5)
        active.clear()
        ended = time.monotonic()
        during = [c for c in calls if began <= c <= ended]
        assert during, "never probed while active"
        assert len(during) <= (ended - began) / interval + 1, (len(during), ended - began)
        time.sleep(interval * 2)       # one probe may have been mid-flight
        settled = len(calls)
        time.sleep(0.3)
        assert len(calls) == settled, "kept probing after the mission ended"
    finally:
        watch.stop()


def _app(tmp_path, monkeypatch, runner_factory=None, probe_s=0.05, health=lambda: True):
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_interval_s: 0.0\n"
                      "  vision_url: http://127.0.0.1:9\n"
                      f"  cloud_probe_s: {probe_s}\n"
                      "  reconfirm_probe_s: 0.02\n  reconfirm_window_s: 30\n")
    probes = []          # one counter per probe the brain builds, in order

    def fake_health_probe(*a, **k):
        n = []
        probes.append(n)

        def probe():
            n.append(1)
            return health()
        return probe

    import control.brain_server as bs
    monkeypatch.setattr(bs, "health_probe", fake_health_probe)
    robot = MockRobot(_build(), render=False)
    app = bs.create_app(config_path=str(config), robot_factory=lambda: robot,
                        world_factory=lambda r: mock_world_for(r),
                        runner_factory=runner_factory)
    return app, probes


def _wait_done(client, timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        status = client.get("/mission/status").json()
        if not status["running"]:
            return status
        time.sleep(0.05)
    raise AssertionError("mission never ended")


def test_2_a_cloudless_mission_makes_no_probes(tmp_path, monkeypatch):
    app, probes = _app(tmp_path, monkeypatch)
    with TestClient(app) as client:
        assert client.get("/mission/status").json()["cloud"]["state"] == UNKNOWN
        r = client.post("/mission/start", json={"target_object": TARGET,
                                                "policy": "frontier", "max_steps": 40})
        assert r.status_code == 200, r.text
        _wait_done(client)
        time.sleep(0.2)
        status = client.get("/mission/status").json()
    assert sum(len(p) for p in probes) == 0, probes
    assert status["cloud"]["state"] == UNKNOWN and status["cloud"]["probes"] == 0


def test_2_once_started_the_watch_still_skips_a_cloudless_mission(tmp_path, monkeypatch):
    """A tiered mission starts the watch's thread; a frontier mission after
    it must not be probed for (the `active` check, not only the start)."""
    spies = []
    tiered = _tiered_factory(_quiet_cloud, spies)

    def runner_factory(robot, req):
        if req.policy == "tiered":
            return tiered(robot, req)
        body = MockRobot(_build(), render=False)
        return SlowFrontier(body, target_object=TARGET, max_steps=40, policy="frontier",
                            world=mock_world_for(body))

    app, probes = _app(tmp_path, monkeypatch, runner_factory, probe_s=0.02)
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        assert _wait_done(client)["outcome"] == FOUND
        time.sleep(0.1)
        before = len(probes[0])
        assert before >= 1, "the tiered mission was never probed for"
        client.post("/mission/start", json={"target_object": TARGET, "policy": "frontier"})
        _wait_done(client)
    assert len(probes[0]) == before, (before, len(probes[0]))


class SlowFrontier(MissionRunner):
    """A frontier mission that lasts long enough (about a second) for a
    0.02 s watch to show itself if it probes for it."""

    def tick(self):
        time.sleep(0.03)
        return super().tick()


# ---------------------------------------------------------------------------
# Criterion 3: never the robot, never B3.2.

def _tiered_factory(cloud, spies):
    def runner_factory(robot, req):
        body = MockRobot(_placed(), render=False)
        runner = MissionRunner(body, target_object=TARGET, max_steps=60, policy="tiered",
                               vision_fn=TieredVision(FrameReportedPipeline(TARGET), cloud,
                                                      steer_on_sight=True, hold_goal=True),
                               world=mock_world_for(body))
        spies.append((RobotSpy(runner.robot), body))
        return runner
    return runner_factory


@pytest.mark.parametrize("probe_s", [0.02, 0])
def test_3_a_dead_health_with_a_live_navigate_changes_nothing(tmp_path, monkeypatch, probe_s):
    """/health down, /navigate up: the mission still ends `found` with no
    vision failure; the watch on (0.02 s) and off (0) give the same run."""
    spies = []
    app, probes = _app(tmp_path, monkeypatch, _tiered_factory(_quiet_cloud, spies),
                       probe_s=probe_s, health=lambda: False)
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        status = _wait_done(client)
    spy, body = spies[0]
    test_3_a_dead_health_with_a_live_navigate_changes_nothing.runs[probe_s] = (
        status["outcome"], status["step"], spy.calls, (body.world.x, body.world.y))
    assert status["outcome"] == FOUND, status["log_tail"][-3:]
    assert status["vision_failures"] == 0
    if probe_s:
        assert status["cloud"]["state"] == UNREACHABLE, status["cloud"]
        assert sum(len(p) for p in probes) >= 1
    runs = test_3_a_dead_health_with_a_live_navigate_changes_nothing.runs
    if len(runs) == 2:
        on, off = runs[0.02], runs[0]
        assert on == off, (on[:2], off[:2])


test_3_a_dead_health_with_a_live_navigate_changes_nothing.runs = {}


# ---------------------------------------------------------------------------
# Criterion 4: one prober at a time; the late check keeps the status moving.

def test_4_during_the_late_wait_only_the_late_check_probes(tmp_path, monkeypatch):
    cloud = SwitchableCloud()
    spies = []
    up = {"v": False}
    app, probes = _app(tmp_path, monkeypatch, _tiered_factory(cloud, spies),
                       probe_s=0.02, health=lambda: up["v"])
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        assert _wait_done(client)["outcome"] == ARRIVED_UNCONFIRMED
        watch_probes = len(probes[0])          # the first probe built is the watch's
        assert client.get("/mission/status").json()["cloud"]["state"] == UNREACHABLE
        time.sleep(0.3)
        assert len(probes[0]) == watch_probes, "the watch probed beside the late check"
        assert len(probes) >= 2 and probes[1], "the late check never probed"
        up["v"] = cloud.up = True
        deadline = time.time() + 5
        while time.time() < deadline:
            status = client.get("/mission/status").json()
            if status["cloud"]["state"] == REACHABLE:
                break
            time.sleep(0.05)
    assert status["cloud"]["state"] == REACHABLE, status["cloud"]
    assert len(probes[0]) == watch_probes


# ---------------------------------------------------------------------------
# Criterion 6: the default sits above the data.

def test_6_the_default_is_above_the_slowest_recorded_call():
    assert DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S == 8.0 > SLOWEST_RECORDED_S
    assert DEFAULTS["arrival_confirm_timeout_s"] == DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S
    assert load_brain_config()["arrival_confirm_timeout_s"] == DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S


# ---------------------------------------------------------------------------
# Criteria 7-9: a hanging cloud, scaled down 10x.

class HttpLike:
    """A cloud client with its own deadline, as `vision_fn_for(timeout_s=)`
    is: a call slower than the deadline ends at the deadline with
    CloudUnavailable (httpx's ReadTimeout), else it answers. Counts the
    peak number of calls in flight."""

    def __init__(self, latency_s, deadline_s, dark_after=None):
        self.latency_s, self.deadline_s = latency_s, deadline_s
        self.dark_after = dark_after          # (tier, n): slow once n confirmations began
        self.live = self.peak = self.calls = 0
        self.first_at = None
        self.lock = threading.Lock()

    def slow(self):
        if self.dark_after is None:
            return True
        tier, n = self.dark_after
        return tier.stats.triggers.get(TRIGGER_ARRIVAL, 0) >= n

    def __call__(self, frame):
        slow = self.slow()
        with self.lock:
            self.calls += 1
            self.live += 1
            self.peak = max(self.peak, self.live)
            if slow and self.first_at is None:
                self.first_at = time.monotonic()
        try:
            if not slow:
                return _quiet_cloud(frame)
            if self.latency_s > self.deadline_s:
                time.sleep(self.deadline_s)
                raise CloudUnavailable(f"ReadTimeout: no answer in {self.deadline_s}s")
            time.sleep(self.latency_s)
            return _quiet_cloud(frame)
        finally:
            with self.lock:
                self.live -= 1


def _hang_run(confirm_deadline_s, vision_timeout_s=2.0, hang_s=60.0):
    """The cloud goes dark at the first arrival confirmation and every call
    from then on hangs. Returns (status, parked seconds, client)."""
    grid = _placed()
    robot = MockRobot(grid, render=False)
    holder = {}
    triggers = HttpLike(hang_s, vision_timeout_s)
    tier = TieredVision(FrameReportedPipeline(TARGET), lambda f: holder["t"](f),
                        steer_on_sight=True, hold_goal=True,
                        confirm_vision_fn=lambda f: holder["c"](f))
    confirm = HttpLike(hang_s, confirm_deadline_s, dark_after=(tier, 1))
    triggers.dark_after = (tier, 1)
    holder["t"], holder["c"] = triggers, confirm
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot),
                           vision_timeout_s=vision_timeout_s,
                           arrival_confirm_timeout_s=confirm_deadline_s)
    runner.start()
    while runner.tick():
        pass
    parked = time.monotonic() - confirm.first_at if confirm.first_at else None
    return runner.status(), parked, confirm


def test_7_and_9_a_hanging_cloud_parks_sooner_with_one_call_in_flight():
    short, parked_short, client = _hang_run(confirm_deadline_s=0.8)
    long_, parked_long, _ = _hang_run(confirm_deadline_s=2.0)
    assert short["outcome"] == long_["outcome"] == ARRIVED_UNCONFIRMED, (
        short["log_tail"][-2:], long_["log_tail"][-2:])
    bar = 3 * (0.8 + 2.0) + 1.0
    assert parked_short <= bar, (parked_short, bar)
    assert parked_short < parked_long, (parked_short, parked_long)
    assert client.peak == 1, f"{client.peak} confirmations in flight at once"
    test_7_and_9_a_hanging_cloud_parks_sooner_with_one_call_in_flight.measured = (
        parked_short, parked_long)


def test_9_a_confirmation_still_in_flight_is_not_doubled():
    """The runner's backstop: a confirmation that outlived its guard is still
    out, so the next fails at once as the cloud's -- no second call."""
    release = threading.Event()
    calls = []

    class Policy:
        def confirm_arrival(self, frame):
            calls.append(1)
            release.wait(5)
            return {"confirmed": True, "cloud_called": True}

    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           arrival_confirm_timeout_s=0.1, vision_timeout_s=1.0)
    runner.vision_fn = Policy()
    from control.mission_runner import VisionUnavailable, _caused_by
    try:
        with pytest.raises(VisionUnavailable) as first:
            runner._guarded_confirm({})
        with pytest.raises(VisionUnavailable) as second:
            runner._guarded_confirm({})
    finally:
        release.set()
    assert "in flight" in str(second.value)
    assert _caused_by(first.value, CloudUnavailable) and _caused_by(second.value, CloudUnavailable)
    assert len(calls) == 1


def test_8_slow_but_alive_still_confirms_on_every_start():
    """Every arrival confirmation answers in 0.68 s against a 0.8 s deadline
    (the slowest recorded call, scaled 10x; amendment 1: triggers fast)."""
    for start in CLEAR_STARTS:
        grid = _placed(start)
        robot = MockRobot(grid, render=False)
        holder = {}
        tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                            steer_on_sight=True, hold_goal=True,
                            confirm_vision_fn=lambda f: holder["c"](f))
        holder["c"] = HttpLike(SLOWEST_RECORDED_S / 10, 0.8)
        runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                               vision_fn=tier, world=mock_world_for(robot),
                               arrival_confirm_timeout_s=0.8)
        runner.start()
        while runner.tick():
            pass
        status = runner.status()
        assert status["outcome"] == FOUND, (start, status["log_tail"][-2:])
        assert tier.stats.triggers.get(TRIGGER_ARRIVAL) == 1, (start, tier.stats.triggers)
        assert math.dist((grid.x, grid.y), GOAL) <= ARRIVED_CELLS


def test_7_the_brain_builds_the_confirmation_client_with_its_own_deadline(tmp_path, monkeypatch):
    """The wiring criterion 7 depends on: brain_server gives the tier a second
    cloud client whose HTTP deadline is min(arrival_confirm, vision)."""
    import control.brain_server as bs
    from tests.test_brain_server import (
        RecordingRobot, capture_tiered, fresh_mock_robot, tiered_config)
    monkeypatch.setattr(bs, "_frames_are_simulated", lambda robot: False)
    timeouts = []

    def fake_vision_fn_for(target, **kwargs):
        timeouts.append(kwargs.get("timeout_s"))
        fn = lambda frame: {}  # noqa: E731
        fn.timeout_s = kwargs.get("timeout_s")
        return fn
    monkeypatch.setattr(bs, "vision_fn_for", fake_vision_fn_for)
    seen = capture_tiered(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(config_path=tiered_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        r = client.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
        client.post("/mission/stop")
    assert r.status_code == 200, r.text
    config = load_brain_config(tiered_config(tmp_path))
    assert seen["cloud_vision_fn"].timeout_s == config["vision_timeout_s"]
    assert seen["confirm_vision_fn"].timeout_s == min(
        config["arrival_confirm_timeout_s"], config["vision_timeout_s"]) == 8.0


# ---------------------------------------------------------------------------
# Criterion 10: config moves together.

def _load(tmp_path, **brain):
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": brain}))
    return load_brain_config(str(path))


def test_10_the_yaml_and_the_defaults_agree():
    assert load_brain_config()["cloud_probe_s"] == DEFAULTS["cloud_probe_s"] == 15.0


@pytest.mark.parametrize("brain", [
    {"arrival_confirm_timeout_s": 0},
    {"arrival_confirm_timeout_s": -1},
    {"cloud_probe_s": -1},
])
def test_10_bad_values_are_refused(tmp_path, brain):
    with pytest.raises(ValueError):
        _load(tmp_path, **brain)


def test_10_lowering_only_the_vision_timeout_in_the_shipped_yaml_still_loads(tmp_path):
    """Amendment 2 (/code-review on 6c9934d): the SHIPPED yaml sets
    arrival_confirm_timeout_s, so the check must be made against it, not a
    stripped file. A deployment that lowers only vision_timeout_s loads (the
    walks Lambda reads this file at cold start) and is clamped."""
    from control.brain_config import _DEFAULT_CONFIG
    shipped = yaml.safe_load(open(_DEFAULT_CONFIG))
    assert "arrival_confirm_timeout_s" in shipped["brain"], "the trap this guards is gone"
    shipped["brain"]["vision_timeout_s"] = 5.0
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(shipped))
    config = load_brain_config(str(path))
    assert config["arrival_confirm_timeout_s"] == 8.0
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           vision_timeout_s=config["vision_timeout_s"],
                           arrival_confirm_timeout_s=config["arrival_confirm_timeout_s"])
    assert runner.confirm_timeout_s() == 5.0


def test_9_the_backstop_waits_for_the_call_in_flight_before_giving_up():
    """Self-review: the next confirmation waits for one still in flight (up
    to one guard) rather than failing on the next tick; if that call ends
    meanwhile, a fresh one goes out -- still one at a time."""
    release = threading.Event()
    live = {"n": 0, "peak": 0, "calls": 0}

    class Policy:
        def confirm_arrival(self, frame):
            live["n"] += 1
            live["calls"] += 1
            live["peak"] = max(live["peak"], live["n"])
            try:
                if live["calls"] == 1:
                    release.wait(5)
                return {"confirmed": True, "cloud_called": True}
            finally:
                live["n"] -= 1

    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           arrival_confirm_timeout_s=0.2, vision_timeout_s=1.0)
    runner.vision_fn = Policy()
    from control.mission_runner import VisionUnavailable
    with pytest.raises(VisionUnavailable):
        runner._guarded_confirm({})                 # the guard (0.25 s) fires
    threading.Timer(0.1, release.set).start()       # the call ends while we wait
    assert runner._guarded_confirm({})["confirmed"] is True
    assert live["calls"] == 2 and live["peak"] == 1, live


def test_review_a_slow_async_trigger_is_not_charged_to_the_confirmation():
    """/code-review on 3062488: under the shipped async tier the confirmation
    first waits for a trigger call still out, on the triggers' deadline. A
    healthy but slow trigger (0.5 s) ahead of a quick confirmation (0.2 s)
    must not trip the confirmation's 0.3 s deadline."""
    grid = _placed()
    robot = MockRobot(grid, render=False)
    holder = {}
    tier = TieredVision(FrameReportedPipeline(TARGET), lambda f: holder["t"](f),
                        steer_on_sight=True, hold_goal=True, async_cloud=True,
                        stale_after=1, confirm_vision_fn=lambda f: holder["c"](f))
    holder["t"] = HttpLike(0.5, 2.0)
    holder["c"] = HttpLike(0.2, 0.3)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot),
                           vision_timeout_s=2.0, arrival_confirm_timeout_s=0.3)
    runner.start()
    while runner.tick():
        pass
    status = runner.status()
    assert tier.stats.triggers.get(TRIGGER_ARRIVAL), "never reached arrival -- proves nothing"
    assert status["outcome"] == FOUND, status["log_tail"][-3:]
    assert not any("vision failure" in line for line in status["log_tail"]), status["log_tail"]


def test_review_the_late_check_never_sends_beside_a_confirmation_still_out():
    """/code-review on 3062488: 3.53's late_ask() ignored a confirmation
    still in flight. Now it waits (B3.2's timeout) and, still out, raises
    TimeoutError -- the Reconfirmer's "hung" -- with nothing sent."""
    from tests.test_arrival_reconfirm import _ended_unconfirmed
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    cloud.up = True
    release = threading.Event()
    out = threading.Thread(target=release.wait, args=(5,), daemon=True)
    out.start()
    runner._confirm_thread = out
    runner.vision_timeout_s = 0.1
    before = cloud.calls
    try:
        with pytest.raises(TimeoutError):
            runner.late_ask()
    finally:
        release.set()
    assert cloud.calls == before, "a second confirmation went out beside the first"
    late = runner.status().get("late_confirmation") or {}
    assert late.get("paid_calls", 0) == 0, late


def test_review_the_searched_rooms_reach_the_confirmation_client():
    """/code-review on 3062488: the confirmation's own client never got the
    searched rooms its /navigate call used to carry."""
    got = {}

    class Client:
        def __init__(self, name):
            self.name = name

        def __call__(self, frame):
            return {}

        def set_searched_rooms(self, rooms):
            got[self.name] = list(rooms)

    tier = TieredVision(FrameReportedPipeline(TARGET), Client("triggers"),
                        confirm_vision_fn=Client("confirm"))
    tier.set_searched_rooms(["kitchen", "hallway"])
    assert got == {"triggers": ["kitchen", "hallway"], "confirm": ["kitchen", "hallway"]}


@pytest.mark.parametrize("prev_out", [False, True], ids=["fresh", "one-still-out"])
def test_13_the_arrival_tick_stays_within_b32s_deadline(prev_out):
    """Amendment 2 (/code-review on 6c9934d): the in-flight wait, the
    backstop join and the call share one deadline, vision_timeout_s, so the
    step can never outlast B3.3's tick deadline. The trigger still out ends
    just inside B3.2's deadline (0.9 of 1.0 s); everything after it hangs."""
    hang = threading.Event()

    class Policy:
        def wait_inflight(self):
            time.sleep(0.9)

        def confirm_arrival(self, frame):
            hang.wait(5)
            return {"confirmed": True, "cloud_called": True}

    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           vision_timeout_s=1.0, arrival_confirm_timeout_s=0.8)
    runner.vision_fn = Policy()
    if prev_out:
        out = threading.Thread(target=hang.wait, args=(5,), daemon=True)
        out.start()
        runner._confirm_thread = out
    from control.mission_runner import VisionUnavailable, _caused_by
    began = time.monotonic()
    try:
        with pytest.raises(VisionUnavailable) as err:
            runner._guarded_confirm({})
    finally:
        hang.set()
    took = time.monotonic() - began
    assert took <= 1.0 + 0.5, took
    assert _caused_by(err.value, CloudUnavailable)


def test_review_a_spent_cap_refuses_without_waiting_for_a_call_still_out():
    """/code-review on 6c9934d: with the call cap spent, a slow trigger still
    out must not turn the free local refusal into a cloud failure."""
    from concurrent.futures import Future
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, max_calls=1,
                        async_cloud=True)
    tier.stats.cloud_calls = 1
    tier._inflight = Future()                 # never completes
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=tier,
                           vision_timeout_s=1.0, arrival_confirm_timeout_s=0.8)
    began = time.monotonic()
    verdict = runner._guarded_confirm({})
    assert time.monotonic() - began < 0.5
    assert verdict["cloud_called"] is False and "cap" in verdict["reason"], verdict


def test_10_a_confirmation_deadline_never_outlasts_b32():
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {},
                           vision_timeout_s=2.0, arrival_confirm_timeout_s=8.0)
    assert runner.confirm_timeout_s() == 2.0
    assert runner.confirm_guard_s() == 2.5
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=lambda f: {})
    assert (runner.confirm_timeout_s(), runner.confirm_guard_s()) == (8.0, 10.0)
