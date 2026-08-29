"""
Phase B2 -- the brain service.

Run with: pytest tests/test_brain_server.py -v

The headline test runs a whole mission through two HTTP boundaries at
once -- TestClient -> brain server -> RemoteRobot -> a live uvicorn
robot server -- and asserts the outcome matches the in-process demo.
That the mission keeps running across many separate status requests is
the property the whole phase exists for: the loop no longer belongs to
whoever started it.
"""

import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from brain.vision import describe_grid_frame
from control.brain_server import MissionStartRequest, create_app
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from tests.conftest import RecordingRobot, fresh_mock_robot

BUDGET = 150
POLL_TIMEOUT_S = 120.0


def demo_report():
    """The reference result: ObjectSearchAgent driving MockRobot directly."""
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(fresh_mock_robot(), memory, min_distance_cm=30)
    return agent.run_mission(max_steps=BUDGET)


def slow_runner_factory(robot_holder=None, delay=0.05):
    """Builds runners whose vision call takes `delay` seconds, so a
    mission stays reliably in-flight while the test pokes at it."""

    def factory(robot, req: MissionStartRequest) -> MissionRunner:
        def slow_vision(frame):
            time.sleep(delay)
            return describe_grid_frame(frame)

        return MissionRunner(
            robot,
            target_object=req.target_object,
            target_room=req.target_room,
            max_steps=req.max_steps or BUDGET,
            vision_fn=slow_vision,
        )

    return factory


def wait_until_finished(client, timeout=POLL_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.get("/mission/status").json()
        if not status["running"]:
            return status
        time.sleep(0.05)
    pytest.fail(f"mission still running after {timeout}s")


@pytest.fixture
def unpaced_config(tmp_path):
    """config/robot.yaml paces the loop so a sim mission is watchable in
    the twin; a test wants it as fast as it will go."""
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_interval_s: 0.0\n")
    return str(config)


def test_mission_runs_to_completion_through_two_http_hops(live_robot_server, unpaced_config):
    expected = demo_report()
    app = create_app(config_path=unpaced_config,
                     robot_factory=lambda: RemoteRobot(live_robot_server))

    with TestClient(app) as client:
        started = client.post(
            "/mission/start", json={"target_object": "red backpack", "max_steps": BUDGET}
        )
        assert started.status_code == 200
        assert started.json()["started"] is True

        status = wait_until_finished(client)

    assert status["outcome"] == "found"
    assert status["found"] is True
    assert status["step"] == expected["steps_taken"]
    assert status["sighting"]["room"] == expected["sighting"].room
    assert status["rooms_searched"] == expected["rooms_searched"]


def test_status_is_idle_before_any_mission():
    with TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as client:
        status = client.get("/mission/status").json()
        assert status["running"] is False
        assert status["outcome"] == "idle"


def test_health_reports_the_robot_url_it_will_use():
    with TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as client:
        body = client.get("/health").json()
        assert body["status"] == "ok"
        assert body["mission_running"] is False
        assert body["robot_url"]


def test_route_prefix_env_var_prepends_every_route(monkeypatch):
    """PLAN-teleop-robot.md: a second brain_server instance shares the
    twin's load balancer by claiming a distinct path prefix instead of the
    existing brain deployment's already-claimed literal paths. Unset (every
    other test in this file), this changes nothing -- confirmed here."""
    monkeypatch.setenv("ROUTE_PREFIX", "/teleop-brain")
    with TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as client:
        assert client.get("/teleop-brain/health").status_code == 200
        assert client.get("/teleop-brain/mission/status").status_code == 200
        # The bare, unprefixed paths must not also work -- otherwise this
        # instance would collide with the existing brain's ListenerRule.
        assert client.get("/health").status_code == 404
        assert client.get("/mission/status").status_code == 404


def test_second_start_while_running_is_rejected_not_raced():
    """Two loops driving one robot is the failure this service exists to
    prevent."""
    app = create_app(
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()),
        runner_factory=slow_runner_factory(delay=0.1),
    )
    with TestClient(app) as client:
        assert client.post("/mission/start", json={"target_object": "red backpack"}).status_code == 200
        second = client.post("/mission/start", json={"target_object": "red backpack"})
        assert second.status_code == 409

        client.post("/mission/stop")


def test_stop_halts_the_loop_and_stops_the_car():
    robot = RecordingRobot(fresh_mock_robot())
    app = create_app(robot_factory=lambda: robot, runner_factory=slow_runner_factory(delay=0.1))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "red backpack"})
        time.sleep(0.3)
        calls_before = len(robot.calls)

        stopped = client.post("/mission/stop")
        assert stopped.status_code == 200
        assert "stop" in robot.calls[calls_before:], "the car was never told to stop"

        status = stopped.json()["status"]
        assert status["running"] is False
        assert status["outcome"] == "stopped"
        assert status["complete"] is False

        # And it stays stopped -- no background task quietly resumes it.
        step_at_stop = client.get("/mission/status").json()["step"]
        time.sleep(0.3)
        assert client.get("/mission/status").json()["step"] == step_at_stop


def test_stop_works_with_no_mission_ever_started():
    """Always available -- an operator hitting stop must not need to know
    whether a mission is running."""
    robot = RecordingRobot(fresh_mock_robot())
    with TestClient(create_app(robot_factory=lambda: robot)) as client:
        resp = client.post("/mission/stop")
        assert resp.status_code == 200
        assert "stop" in robot.calls


def test_vision_policy_without_a_configured_service_is_rejected_at_start(tmp_path):
    """A misconfigured brain must say so before it starts a mission, not
    after the first paid call fails."""
    config = tmp_path / "robot.yaml"
    config.write_text('brain:\n  vision_url: ""\n')
    app = create_app(config_path=str(config),
                     robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        resp = client.post(
            "/mission/start", json={"target_object": "red backpack", "policy": "vision"}
        )
        assert resp.status_code == 400
        assert "vision_url" in resp.json()["detail"]


def test_vision_policy_needs_an_object_not_just_a_room(tmp_path):
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  vision_url: http://vision.test\n")
    app = create_app(config_path=str(config),
                     robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        resp = client.post(
            "/mission/start", json={"target_room": "kitchen", "policy": "vision"}
        )
        assert resp.status_code == 400
        assert "target_object" in resp.json()["detail"]


def test_mission_with_no_target_is_rejected_with_400():
    with TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as client:
        assert client.post("/mission/start", json={}).status_code == 400


def test_mission_routes_require_the_secret_when_set(monkeypatch):
    """Anyone who can start a mission can drive the robot, so the brain
    needs the same gate the robot server has. /health stays open."""
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    with TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as client:
        assert client.get("/mission/status").status_code == 401
        assert client.post("/mission/stop").status_code == 401
        assert client.post("/mission/start", json={"target_object": "x"}).status_code == 401
        assert client.get("/health").status_code == 200

        ok = client.get("/mission/status", headers={"x-app-secret": "correct-horse-battery-staple"})
        assert ok.status_code == 200


def test_the_brain_process_never_loads_the_simulator():
    """Definition-of-done item 3. The brain is an HTTP client of the
    robot and nothing else: if importing it drags in sim/ or a robot
    backend, "run the brain on the Pi" has quietly stopped being a
    config change. Checked in a subprocess so this test file's own
    imports don't mask it."""
    import subprocess
    import sys

    probe = (
        "import control.brain_server, sys; "
        "print(sorted(m for m in sys.modules if m.split('.')[0] in ('sim', 'robot')))"
    )
    result = subprocess.run([sys.executable, "-c", probe], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    loaded = eval(result.stdout)

    assert not [m for m in loaded if m.startswith("sim")], f"brain loaded the simulator: {loaded}"
    # robot.safety comes in with SafetyViolation, which is part of the
    # interface contract RemoteRobot re-raises. A backend or the robot
    # server itself would not be.
    assert set(loaded) <= {"robot", "robot.interface", "robot.safety"}, loaded


# ---------- recording a Robot-view walk ----------
#
# The frames the phone captures during a Robot view walk are worth keeping:
# replayed through sim/replay_robot.py they let the Python brain run the
# same real-pixel mission repeatedly, which is the only way to iterate on a
# prompt without walking the house again for every change.


import base64

FRAME_B64 = base64.b64encode(b"\xff\xd8\xff\xd9").decode()


def recording_app(tmp_path, allow=True):
    config = tmp_path / "robot.yaml"
    config.write_text(
        f"brain:\n  allow_recording: {'true' if allow else 'false'}\n"
        f"  recording_dir: {tmp_path / 'recordings'}\n"
    )
    return create_app(config_path=str(config),
                      robot_factory=lambda: RecordingRobot(fresh_mock_robot()))


def post_frame(client, seq, walk="walk-1", **overrides):
    body = {"walk": walk, "seq": seq, "image_base64": FRAME_B64,
            "media_type": "image/jpeg", "navigate": {"action": "FORWARD"}}
    body.update(overrides)
    return client.post("/recording/frame", json=body)


def test_a_walk_lands_where_replay_robot_can_read_it(tmp_path):
    from sim.replay_robot import ReplayRobot

    with TestClient(recording_app(tmp_path)) as client:
        for seq in range(3):
            resp = post_frame(client, seq)
            assert resp.status_code == 200, resp.text
        assert resp.json()["frames"] == 3
        walk_dir = resp.json()["dir"]

    # The point of the endpoint: this has to be directly replayable.
    robot = ReplayRobot(walk_dir)
    assert robot.get_camera_frame()["image_base64"] == FRAME_B64
    robot.drive_forward()
    assert robot.get_camera_frame()["metadata"]["index"] == 1


def test_each_frames_live_answer_is_kept_beside_it(tmp_path):
    """So a replayed run can be diffed against what the service said at the
    time, on the same pixels."""
    import json as _json

    with TestClient(recording_app(tmp_path)) as client:
        post_frame(client, 0, navigate={"action": "LEFT", "reasoning": "door"})
        walk_dir = post_frame(client, 1).json()["dir"]

    lines = [_json.loads(l) for l in (Path(walk_dir) / "walk.jsonl").read_text().splitlines()]
    assert [l["seq"] for l in lines] == [0, 1]
    assert lines[0]["navigate"]["reasoning"] == "door"
    assert lines[0]["file"] == "frame-0000.jpg"


def test_recording_can_be_switched_off(tmp_path):
    with TestClient(recording_app(tmp_path, allow=False)) as client:
        assert post_frame(client, 0).status_code == 403
        assert client.get("/health").json()["recording_allowed"] is False


def test_health_reports_recording_allowed_when_only_proxied(tmp_path):
    """Regression: a brain with allow_recording: false but a configured
    recording_proxy_url can still actually save a walk (see the proxy
    tests below) -- /health's recording_allowed has to say so, or
    web-twin/index.html's pre-flight check in startGuidance() rejects a
    "Record this walk" session that would have worked."""
    config = tmp_path / "robot.yaml"
    config.write_text(
        "brain:\n  allow_recording: false\n"
        "  recording_proxy_url: http://peer-brain.internal\n"
    )
    app = create_app(config_path=str(config),
                      robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/health").json()["recording_allowed"] is True


# ---------- recording proxy (teleop-brain -> main brain) ----------
#
# A brain with no storage of its own (allow_recording: false, e.g.
# teleop-brain -- no EFS mount) can forward POST /recording/frame to a peer
# brain that has one, instead of just rejecting it -- see
# PLAN-teleop-robot.md's "Recording proxy" section. The outbound call is a
# plain httpx.Client used synchronously inside asyncio.to_thread, so these
# tests fake control.brain_server.httpx.Client rather than spinning up a
# second live server -- record_frame()'s local-storage behavior is already
# covered above, and the proxy path only needs to prove it forwards the
# right request and relays the peer's response honestly.


def proxying_app(tmp_path, proxy_url="http://peer-brain.internal",
                  proxy_secret="peer-secret"):
    config = tmp_path / "robot.yaml"
    config.write_text(
        "brain:\n"
        "  allow_recording: false\n"
        f"  recording_proxy_url: {proxy_url}\n"
        f"  recording_proxy_secret: {proxy_secret}\n"
    )
    return create_app(config_path=str(config),
                      robot_factory=lambda: RecordingRobot(fresh_mock_robot()))


def fake_proxy_client(monkeypatch, post):
    """Patches control.brain_server.httpx.Client to a stub whose .post()
    is `post`, so a test can assert on the outbound call and control the
    (fake) peer's response without a real second server."""
    import control.brain_server as brain_server_module

    class FakeClient:
        def __init__(self, timeout=None):
            self.timeout = timeout

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def post(self, url, json=None, headers=None):
            return post(url, json=json, headers=headers)

    monkeypatch.setattr(brain_server_module.httpx, "Client", FakeClient)


class FakeProxyResponse:
    def __init__(self, status_code=200, json_body=None, text=""):
        self.status_code = status_code
        self._json_body = json_body or {}
        self.text = text

    def json(self):
        return self._json_body


def test_recording_proxies_to_a_peer_brain_when_disabled_locally(tmp_path, monkeypatch):
    calls = []

    def post(url, json, headers):
        calls.append({"url": url, "json": json, "headers": headers})
        return FakeProxyResponse(
            200, {"saved": "frame-0000.jpg", "walk": "walk-1", "frames": 1, "dir": "peer-dir"}
        )

    fake_proxy_client(monkeypatch, post)

    with TestClient(proxying_app(tmp_path)) as client:
        resp = post_frame(client, 0)

    assert resp.status_code == 200, resp.text
    # The proxy's response is relayed verbatim -- "peer-dir" could never
    # come from this brain's own (disabled) local-write path.
    assert resp.json()["dir"] == "peer-dir"
    assert len(calls) == 1
    assert calls[0]["url"] == "http://peer-brain.internal/recording/frame"
    assert calls[0]["headers"] == {"x-app-secret": "peer-secret"}
    assert calls[0]["json"]["walk"] == "walk-1"


def test_recording_proxy_surfaces_the_peers_own_error(tmp_path, monkeypatch):
    fake_proxy_client(monkeypatch, lambda url, json, headers: FakeProxyResponse(400, text="bad walk name"))

    with TestClient(proxying_app(tmp_path)) as client:
        resp = post_frame(client, 0)

    assert resp.status_code == 400


def test_recording_proxy_unreachable_peer_is_a_502_not_a_crash(tmp_path, monkeypatch):
    import httpx

    def post(url, json, headers):
        raise httpx.ConnectError("connection refused")

    fake_proxy_client(monkeypatch, post)

    with TestClient(proxying_app(tmp_path)) as client:
        resp = post_frame(client, 0)

    assert resp.status_code == 502


def test_a_walk_name_cannot_escape_the_recording_directory(tmp_path):
    with TestClient(recording_app(tmp_path)) as client:
        for bad in ["../evil", "a/b", "", ".hidden", "x" * 65]:
            assert post_frame(client, 0, walk=bad).status_code == 400, bad


def test_a_corrupt_or_oversized_frame_is_refused(tmp_path):
    with TestClient(recording_app(tmp_path)) as client:
        assert post_frame(client, 0, image_base64="not base64!!").status_code == 400
        assert post_frame(client, 0, image_base64="A" * (8 * 1024 * 1024)).status_code == 413
