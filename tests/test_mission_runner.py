"""
Phase B1 -- MissionRunner.

Run with: pytest tests/test_mission_runner.py -v

The point of these tests is that turning the loop inside out changed
nothing about the mission: a runner driven a tick at a time reaches the
same result, in the same number of steps, as ObjectSearchAgent's blocking
run_mission() -- which is what tests/demo_active_search.py exercises.
"""

import pytest

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from control.mission_runner import (
    FOUND,
    IDLE,
    MAX_STEPS,
    RUNNING,
    STOPPED,
    MissionRunner,
)
from tests.conftest import RecordingRobot, fresh_mock_robot

BUDGET = 150


def drive_to_completion(runner, max_ticks=500):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


def test_ticking_matches_the_blocking_run_mission():
    """demo_active_search.py's mission, both ways."""
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(fresh_mock_robot(), memory, min_distance_cm=30)
    demo_report = agent.run_mission(max_steps=BUDGET)

    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == FOUND
    assert status["found"] is True
    assert status["step"] == demo_report["steps_taken"]
    assert status["rooms_searched"] == demo_report["rooms_searched"]
    assert status["sighting"]["object_name"] == demo_report["sighting"].object_name
    assert status["sighting"]["room"] == demo_report["sighting"].room


def test_status_before_start_is_idle():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == IDLE
    assert status["step"] == 0
    assert status["found"] is False


def test_status_while_running_reports_progress():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    runner.start()
    assert runner.status()["outcome"] == RUNNING
    for _ in range(5):
        runner.tick()

    status = runner.status()
    assert status["running"] is True
    assert status["step"] == 5
    assert status["last_action"] in {"FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP",
                                     "LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER"}
    assert status["last_reasoning"]
    assert status["rooms_visited"]
    assert len(status["log_tail"]) >= 5


def test_stop_mid_mission_halts_and_stops_the_car():
    """Stopping the thinking is not stopping the robot -- stop() has to
    do both."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot, target_object="red backpack", max_steps=BUDGET)
    runner.start()
    for _ in range(5):
        runner.tick()

    robot.calls.clear()
    runner.stop()

    assert "stop" in robot.calls
    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == STOPPED
    assert status["complete"] is False
    assert runner.tick() is False, "a stopped runner must not keep stepping"
    assert runner.status()["step"] == 5


def test_stop_is_idempotent_and_keeps_the_first_outcome():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    runner.start()
    runner.tick()
    runner.stop()
    runner.abort("should not overwrite a completed mission")
    assert runner.status()["outcome"] == STOPPED


def test_step_budget_ends_the_mission():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=4)
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == MAX_STEPS
    assert status["step"] == 4
    assert status["found"] is False


def test_room_target_completes_on_arrival():
    runner = MissionRunner(fresh_mock_robot(), target_room="hallway", max_steps=BUDGET)
    runner.start()
    status = drive_to_completion(runner)

    assert status["room_reached"] is True
    assert status["outcome"] == "room_reached"
    assert "hallway" in status["rooms_visited"]


def test_double_start_is_rejected():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    runner.start()
    with pytest.raises(RuntimeError):
        runner.start()


def test_a_mission_with_no_target_is_rejected():
    """MissionMemory.is_complete() is never true without one, so such a
    mission can only ever end at the step budget."""
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot())


def test_vision_policy_requires_a_vision_fn():
    """The default vision_fn is the offline grid converter. Running the
    vision policy on top of it would produce a mission that looks like it
    used the model and did not."""
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot(), target_object="red backpack", policy="vision")


def test_unknown_policy_is_rejected():
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot(), target_object="x", policy="telepathy")


def test_runner_works_the_same_over_http(robot_over_asgi):
    """The runner never touches a backend directly -- swapping in a
    RemoteRobot must not change the mission."""
    local = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    local.start()
    local_status = drive_to_completion(local)

    remote = MissionRunner(robot_over_asgi, target_object="red backpack", max_steps=BUDGET)
    remote.start()
    remote_status = drive_to_completion(remote)

    assert remote_status["outcome"] == local_status["outcome"] == FOUND
    assert remote_status["step"] == local_status["step"]
    assert remote_status["sighting"] == local_status["sighting"]
