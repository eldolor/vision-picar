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


# ---------- the depth grid over the wire (phase M2) ----------


def test_depth_grid_comes_from_the_robot_not_from_the_interface_default(robot_over_asgi):
    """`get_depth_grid()` is the one RobotInterface method with a default
    implementation, so a client that forgot to override it would answer
    all-unusable while talking to a robot that has a working sensor -- a
    silent lie in the one direction a safety consumer must not be lied to,
    and one nothing else in the suite would notice."""
    from robot.interface import ZONE_RANGE

    grid = robot_over_asgi.get_depth_grid()
    assert grid["rows"] == 1 and grid["cols"] >= 1
    assert any(z["status"] == ZONE_RANGE for z in grid["zones"]), (
        "the sim robot on the other end has a sensor -- reporting none means "
        "the interface default was inherited instead of the route being called"
    )
    local = fresh_mock_robot().get_depth_grid()
    assert {k: grid[k] for k in ("rows", "cols", "fov_deg", "zones")} == local, (
        "the wire must not change the grid: same robot, same start pose"
    )
    assert grid["fov_deg"] == local["fov_deg"], (
        "fov_deg has to survive the wire or brain-side safety silently falls "
        "back to the fraction-of-columns rule (PLAN-onboard-perception.md 5.1)"
    )
    # It may *add* to it, and since M3 it does: the route composes the
    # server's own safety reduction onto the backend's grid, so the twin can
    # draw which zones the veto reads without re-implementing that rule in
    # the browser. Additive only -- `zones` above is byte-for-byte what the
    # backend produced, which is what keeps this a passthrough.
    assert grid["path"]["source"] == "depth_grid"


def test_a_server_with_no_depth_route_reports_no_sensor_rather_than_failing():
    """The stacks are redeployed one at a time, so a brain talking to a
    pre-M2 robot server is a real state. It resolves to the same
    all-unusable grid any sensorless backend gives -- never a transport
    error that would spend a tick of the mission's failure budget, and
    never a fabricated clearance."""
    import httpx

    from robot.interface import ZONE_UNUSABLE

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/depth":
            return httpx.Response(404, json={"detail": "Not Found"})
        return httpx.Response(200, json={})

    client = httpx.Client(transport=httpx.MockTransport(handler))
    bot = RemoteRobot("http://robot.test", client=client)
    grid = bot.get_depth_grid()
    assert all(z["status"] == ZONE_UNUSABLE for z in grid["zones"])


def test_a_broken_depth_sensor_is_not_reported_as_an_absent_one():
    """Only 404 means "this server has no such route". A 500 is a robot
    that has a depth route and could not answer it, which the caller has to
    hear about -- collapsing the two would turn every sensor fault into a
    quiet "no sensor fitted"."""
    import httpx

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(500, text="sensor bus error")

    client = httpx.Client(transport=httpx.MockTransport(handler))
    bot = RemoteRobot("http://robot.test", client=client)
    with pytest.raises(RobotTransportError):
        bot.get_depth_grid()
