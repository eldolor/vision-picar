"""
Phase B3 -- the three failsafes.

Run with: pytest tests/test_failsafes.py -v

Three distinct failures, three distinct guards:

  B3.1  motors left running   -- robot/server.py's watchdog. Unchanged by
                                 this phase; its decision logic is tested
                                 in tests/test_server.py
                                 (test_watchdog_should_stop_pure_logic),
                                 and the async polling loop it drives is
                                 Phase S4's job to exercise.
  B3.2  the AWS link is dead  -- MissionRunner's vision timeout and
                                 consecutive-failure budget. Tested here.
  B3.3  the brain loop hangs  -- brain_server's per-tick dead-man, which
                                 B3.1 cannot see because a stuck loop is
                                 still a live process. Tested here.

In every case the assertion is the same one: **the robot was told to
stop.** A failsafe that ends the mission but leaves the car moving has
not done its job.
"""

import threading
import time

import pytest
from fastapi.testclient import TestClient

from control.brain_server import create_app
from control.mission_runner import FAILED, FOUND, MissionRunner
from tests.conftest import RecordingRobot, fresh_mock_robot

BUDGET = 150


def drive_to_completion(runner, max_ticks=500):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


# ---------- B3.2: the AWS link ----------


def test_consecutive_vision_failures_end_the_mission_with_the_robot_stopped():
    """The browser's autopilot logs a /navigate error and schedules the
    next tick. A robot that keeps moving while blind is the failure mode
    that matters on hardware."""
    calls = {"n": 0}

    def failing_vision(frame):
        calls["n"] += 1
        raise RuntimeError("navigate: connection reset")

    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(
        robot,
        target_object="red backpack",
        max_steps=BUDGET,
        vision_fn=failing_vision,
        max_vision_failures=3,
    )
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == FAILED
    assert "vision unavailable 3 times" in status["error"]
    assert calls["n"] == 3, "gave up too early or kept retrying past the budget"
    assert "stop" in robot.calls
    assert "drive_forward" not in robot.calls, "the robot moved while blind"


def test_a_single_vision_failure_is_survivable():
    """One dropped call is not a reason to end a mission -- but the car
    stops for it, and the counter resets on the next success."""
    from brain.vision import describe_grid_frame

    calls = {"n": 0}

    def flaky_vision(frame):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("navigate: 502")
        return describe_grid_frame(frame)

    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(
        robot, target_object="red backpack", max_steps=BUDGET, vision_fn=flaky_vision
    )
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == FOUND
    assert status["vision_failures"] == 0
    assert status["error"] is None


def test_a_hanging_vision_call_trips_the_same_budget():
    """A call that never returns is the failure the browser handles worst
    -- there is no error to log, so nothing ever fires."""
    release = threading.Event()
    calls = {"n": 0}

    def hanging_vision(frame):
        calls["n"] += 1
        release.wait(30)  # released in the finally block, never on its own
        return {}

    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(
        robot,
        target_object="red backpack",
        max_steps=BUDGET,
        vision_fn=hanging_vision,
        vision_timeout_s=0.15,
        max_vision_failures=3,
    )
    try:
        runner.start()
        started = time.monotonic()
        status = drive_to_completion(runner)
        elapsed = time.monotonic() - started

        assert status["outcome"] == FAILED
        assert "vision unavailable 3 times" in status["error"]
        assert "timed out" in status["error"]
        assert elapsed < 5, "the mission waited on the hung call instead of timing out"
        assert "stop" in robot.calls
        assert "drive_forward" not in robot.calls
    finally:
        release.set()


def test_a_healthy_mission_is_untouched_by_either_guard():
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot, target_object="red backpack", max_steps=BUDGET)
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == FOUND
    assert status["error"] is None
    assert status["vision_failures"] == 0
    assert "drive_forward" in robot.calls


def test_no_movement_command_reaches_the_robot_after_a_stop():
    """The gate behind POST /mission/stop's promise: a tick that was
    already deciding when the stop landed must not get its move out."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot, target_object="red backpack", max_steps=BUDGET)
    runner.start()
    for _ in range(5):
        runner.tick()
    runner.stop()

    robot.calls.clear()
    assert runner.tick() is False
    assert [c for c in robot.calls if c != "get_camera_frame"] == []


# ---------- B3.3: a hung brain loop ----------


class HungRunner(MissionRunner):
    """A runner whose tick() never returns -- the failure B3.1's watchdog
    cannot see, because the brain process is alive and the robot is idle
    rather than being commanded."""

    release = threading.Event()

    def tick(self) -> bool:
        self.release.wait(30)
        return True


@pytest.fixture
def fast_deadman_config(tmp_path):
    """A brain config whose per-tick dead-man is short enough to test."""
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_timeout_s: 0.3\n  robot_url: http://127.0.0.1:8000\n")
    return str(config)


def test_a_hung_tick_stops_the_robot_and_fails_the_mission(fast_deadman_config):
    robot = RecordingRobot(fresh_mock_robot())
    app = create_app(
        config_path=fast_deadman_config,
        robot_factory=lambda: robot,
        runner_factory=lambda r, req: HungRunner(r, target_object=req.target_object),
    )
    try:
        with TestClient(app) as client:
            client.post("/mission/start", json={"target_object": "red backpack"})

            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                status = client.get("/mission/status").json()
                if not status["running"]:
                    break
                time.sleep(0.05)
            else:
                pytest.fail("the hung tick was never caught")

            # Release before leaving the TestClient block: shutting the
            # event loop down waits on its executor threads, and the hung
            # tick is running on one. The mission loop has already exited,
            # so nothing resumes.
            HungRunner.release.set()

        assert status["outcome"] == FAILED
        assert "hung" in status["error"]
        assert "stop" in robot.calls, "the mission failed but the car was never stopped"
    finally:
        HungRunner.release.set()


def test_a_healthy_mission_is_not_killed_by_the_deadman(fast_deadman_config):
    """0.3s is a tight dead-man, and a real tick is far quicker than it --
    the guard must not fire on a working mission."""
    robot = RecordingRobot(fresh_mock_robot())
    app = create_app(
        config_path=fast_deadman_config,
        robot_factory=lambda: robot,
        runner_factory=lambda r, req: MissionRunner(
            r, target_object=req.target_object, max_steps=BUDGET
        ),
    )
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "red backpack"})
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            status = client.get("/mission/status").json()
            if not status["running"]:
                break
            time.sleep(0.05)
        else:
            pytest.fail("mission never finished")

    assert status["outcome"] == FOUND
    assert status["error"] is None


# ---------- the drills the twin fires these guards with ----------
#
# control/drills.py exists so B3.2 and B3.3 can be demonstrated from the
# UI. That makes the drills themselves load-bearing: a drill that quietly
# stopped exercising the real guard would turn the twin's demonstration
# into theatre. These tests pin what each one does.


def drill_app(fault_config, robot, **overrides):
    return create_app(config_path=fault_config, robot_factory=lambda: robot, **overrides)


def run_drill(client, fault, timeout=20):
    resp = client.post(
        "/mission/start", json={"target_object": "red backpack", "fault": fault}
    )
    assert resp.status_code == 200, resp.text
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.get("/mission/status").json()
        if not status["running"]:
            return status
        time.sleep(0.05)
    pytest.fail(f"the {fault} drill never ended")


@pytest.fixture
def drill_config(tmp_path):
    """Short deadlines so the drills finish inside a test."""
    config = tmp_path / "robot.yaml"
    config.write_text(
        "brain:\n"
        "  robot_url: http://127.0.0.1:8000\n"
        "  allow_drills: true\n"
        "  drill_vision_timeout_s: 0.2\n"
        "  drill_tick_timeout_s: 0.3\n"
    )
    return str(config)


@pytest.mark.parametrize("fault", ["vision_error", "vision_hang", "tick_hang"])
def test_every_drill_ends_failed_with_the_robot_stopped(drill_config, fault):
    robot = RecordingRobot(fresh_mock_robot())
    with TestClient(drill_app(drill_config, robot)) as client:
        status = run_drill(client, fault)

    assert status["outcome"] == FAILED, f"{fault} did not trip a failsafe"
    assert status["error"], f"{fault} reported no reason"
    assert status["fault"] == fault
    assert "stop" in robot.calls, f"{fault} ended the mission but not the car"


def test_the_drills_trip_the_guard_they_claim_to(drill_config):
    """Each drill has to fire its own failsafe, not just fail somehow."""
    with TestClient(drill_app(drill_config, RecordingRobot(fresh_mock_robot()))) as client:
        assert "vision unavailable" in run_drill(client, "vision_error")["error"]
        hang = run_drill(client, "vision_hang")["error"]
        assert "vision unavailable" in hang and "timed out" in hang
        assert "hung" in run_drill(client, "tick_hang")["error"]


def test_the_tick_hang_drill_runs_a_step_before_freezing(drill_config):
    """The twin has to show the mission alive first, or the dead-man looks
    like a mission that never started."""
    with TestClient(drill_app(drill_config, RecordingRobot(fresh_mock_robot()))) as client:
        assert run_drill(client, "tick_hang")["step"] >= 1


def test_no_fault_builds_exactly_the_ordinary_mission(drill_config):
    """The drill path and the production path must build the same object,
    or a drill is a different mission wearing a costume."""
    robot = RecordingRobot(fresh_mock_robot())
    with TestClient(drill_app(drill_config, robot)) as client:
        status = run_drill(client, "none", timeout=60)

    assert status["outcome"] == FOUND
    assert status["error"] is None
    assert status["fault"] == "none"


def test_an_unknown_fault_is_rejected(drill_config):
    with TestClient(drill_app(drill_config, RecordingRobot(fresh_mock_robot()))) as client:
        resp = client.post(
            "/mission/start", json={"target_object": "red backpack", "fault": "explode"}
        )
        assert resp.status_code == 400
        assert "Unknown fault" in resp.json()["detail"]


def test_drills_can_be_switched_off(tmp_path):
    """A brain reachable beyond the LAN should not accept 'end this
    mission' from anyone who can reach it."""
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  allow_drills: false\n")
    robot = RecordingRobot(fresh_mock_robot())

    with TestClient(drill_app(str(config), robot)) as client:
        resp = client.post(
            "/mission/start", json={"target_object": "red backpack", "fault": "vision_error"}
        )
        assert resp.status_code == 403
        assert client.get("/health").json()["drills_allowed"] is False
        # An ordinary mission is unaffected.
        assert client.post("/mission/start", json={"target_object": "red backpack"}).status_code == 200
        client.post("/mission/stop")
