"""
Phase B0 (= S3) -- RemoteRobot.

Run with: pytest tests/test_remote_robot.py -v

The headline test is test_identical_action_sequence_over_a_real_socket:
the same mission, run in-process and again across a real uvicorn, must
produce the identical action sequence. That is the proof the HTTP
boundary is transparent, and therefore that "brain on the Pi" vs "brain
on the MacBook" is a base-URL difference and nothing more.
"""

import pytest

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from control.remote_robot import RemoteRobot, RobotTransportError
from robot.safety import SafetyViolation
from tests.conftest import fresh_mock_robot

MAX_STEPS = 150


def run_backpack_hunt(robot):
    """tests/demo_active_search.py's mission, driven programmatically."""
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30)
    report = agent.run_mission(max_steps=MAX_STEPS)
    return [r.action for r in agent.history], report


def test_identical_action_sequence_over_a_real_socket(live_robot_server):
    """The single most valuable test in PLAN-sim-hardening.md's S3."""
    local_actions, local_report = run_backpack_hunt(fresh_mock_robot())

    with RemoteRobot(live_robot_server) as remote:
        remote_actions, remote_report = run_backpack_hunt(remote)

    assert local_actions == remote_actions
    assert local_report["steps_taken"] == remote_report["steps_taken"]
    assert local_report["found"] is True and remote_report["found"] is True
    assert local_report["sighting"].room == remote_report["sighting"].room
    assert local_report["rooms_searched"] == remote_report["rooms_searched"]


def test_identical_action_sequence_in_process_vs_asgi(robot_over_asgi):
    """Same assertion without the subprocess, so the cheap suite still
    catches a regression in the HTTP layer."""
    local_actions, local_report = run_backpack_hunt(fresh_mock_robot())
    remote_actions, remote_report = run_backpack_hunt(robot_over_asgi)

    assert local_actions == remote_actions
    assert local_report["steps_taken"] == remote_report["steps_taken"]
    assert remote_report["found"] is True


def test_return_shapes_match_the_in_process_backend(robot_over_asgi):
    """Every method has to return what the interface's other
    implementations return -- same keys, same units, same types."""
    local = fresh_mock_robot()

    assert robot_over_asgi.get_camera_frame() == local.get_camera_frame()
    assert robot_over_asgi.get_distance() == local.get_distance()
    assert isinstance(robot_over_asgi.get_distance(), float)

    for call in ("look_left", "look_right", "look_center", "stop"):
        assert getattr(robot_over_asgi, call)() == getattr(local, call)()

    assert robot_over_asgi.turn_right(90) == local.turn_right(90)
    assert robot_over_asgi.drive_forward(50, 0.5) == local.drive_forward(50, 0.5)


def test_position_survives_json_as_a_tuple(robot_over_asgi):
    """JSON has no tuples, and MissionAgent keys a set with `position` --
    a list here is an unhashable-type crash several steps later."""
    frame = robot_over_asgi.get_camera_frame()
    assert isinstance(frame["position"], tuple)
    assert {frame["position"]}  # hashable, which is the actual requirement

    moved = robot_over_asgi.drive_forward(50, 0.5)
    assert isinstance(moved["position"], tuple)


def test_safety_veto_raises_the_same_exception_it_does_in_process(robot_over_asgi):
    """In-process a blocked action raises SafetyViolation; over HTTP the
    server answers 200 {"executed": false}. The agent must not be able to
    tell the difference."""
    with pytest.raises(SafetyViolation):
        for _ in range(20):
            robot_over_asgi.drive_forward(100, 1.0)


def test_unreachable_robot_raises_transport_error():
    from tests.conftest import free_port

    with RemoteRobot(f"http://127.0.0.1:{free_port()}", timeout=0.5) as remote:
        with pytest.raises(RobotTransportError):
            remote.get_distance()


def test_bad_secret_is_a_transport_error_not_a_silent_no_op(monkeypatch):
    """A 401 must be loud. A brain that reads a rejected command as a
    completed one would drive blind."""
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    from robot.server import create_app
    from tests.conftest import ASGI_BASE_URL, asgi_client

    base_url = ASGI_BASE_URL
    client = asgi_client(create_app())

    with RemoteRobot(base_url, secret="wrong", client=client) as remote:
        with pytest.raises(RobotTransportError):
            remote.get_distance()

    with RemoteRobot(base_url, secret="correct-horse-battery-staple", client=client) as remote:
        assert remote.get_distance() >= 0

    client.close()


def test_unknown_action_raises_value_error(robot_over_asgi):
    """robot/safety.py raises ValueError for an unknown verb; the server
    turns that into a 400; RemoteRobot turns it back."""
    with pytest.raises(ValueError):
        robot_over_asgi._action("FLY")
