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
from control.brain_config import load_brain_config
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
    # interface contract RemoteRobot re-raises. robot.identity is M5's
    # start-up line, which both servers log and which therefore has to live
    # in the lower half -- it is stdlib only, and importing it can reach no
    # backend. A backend or the robot server itself would not be allowed
    # here, which is the whole point of listing these by name rather than
    # allowing the `robot` package wholesale.
    assert set(loaded) <= {
        "robot", "robot.interface", "robot.safety", "robot.identity",
    }, loaded


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


# ---------- finishing a walk ----------
#
# Frames arrive one at a time and nothing in the stream says which is last,
# so a walk is otherwise only "finished" in the sense that no more frames
# turned up. POST /recording/finish is the explicit signal, and it is where
# the walk-level model_id lands -- the fact that makes a recorded walk
# comparable against another walk of the same room on a different model.


def test_finishing_a_walk_records_the_model_that_produced_it(tmp_path):
    import json as _json

    with TestClient(recording_app(tmp_path)) as client:
        post_frame(client, 0)
        walk_dir = post_frame(client, 1).json()["dir"]
        resp = client.post("/recording/finish", json={
            "walk": "walk-1",
            "model_id": "amazon.nova-lite-v1:0",
            "target_object": "red backpack",
        })

    assert resp.status_code == 200, resp.text
    meta = _json.loads((Path(walk_dir) / "meta.json").read_text())
    assert meta["model_id"] == "amazon.nova-lite-v1:0"
    assert meta["target_object"] == "red backpack"
    assert meta["frames"] == 2
    assert meta["finished_at"] > 0


def test_a_walk_records_the_resolution_it_was_captured_at(tmp_path):
    """So the corpus describes its own viewpoint instead of needing a
    document to interpret it.

    The whole Stage 0 corpus was 640x480 and nobody noticed for a month --
    the twin asked getUserMedia for a camera with no resolution constraint
    and got the browser's default, so every finding about small distant
    targets in PLAN-onboard-perception.md 4.10/4.11 was measured on VGA. The
    capture size is the one number the browser knows and a person reading
    the frames later cannot see, so the walk carries it.
    """
    import json as _json

    with TestClient(recording_app(tmp_path)) as client:
        post_frame(client, 0)
        walk_dir = post_frame(client, 1).json()["dir"]
        resp = client.post("/recording/finish", json={
            "walk": "walk-1", "capture_width": 1280, "capture_height": 720,
        })

    assert resp.status_code == 200, resp.text
    meta = _json.loads((Path(walk_dir) / "meta.json").read_text())
    assert meta["capture"] == {"width": 1280, "height": 720}


def test_a_walk_that_cannot_report_its_capture_size_still_finishes(tmp_path):
    """A phone that never produced a frame, or an older twin build: the
    marker still has to be written, because it is what tells a completed
    walk from one that merely stopped arriving."""
    import json as _json

    with TestClient(recording_app(tmp_path)) as client:
        post_frame(client, 0)
        walk_dir = post_frame(client, 1).json()["dir"]
        resp = client.post("/recording/finish", json={"walk": "walk-1"})

    assert resp.status_code == 200, resp.text
    meta = _json.loads((Path(walk_dir) / "meta.json").read_text())
    assert "capture" not in meta
    assert meta["frames"] == 2


def test_finishing_an_unknown_walk_is_a_404_not_a_new_directory(tmp_path):
    """A typo'd walk name must not conjure an empty walk into the listing."""
    with TestClient(recording_app(tmp_path)) as client:
        resp = client.post("/recording/finish", json={"walk": "never-recorded"})
    assert resp.status_code == 404
    assert not (tmp_path / "recordings" / "never-recorded").exists()


def test_finish_rejects_a_bad_walk_name(tmp_path):
    with TestClient(recording_app(tmp_path)) as client:
        assert client.post("/recording/finish", json={"walk": "../etc"}).status_code == 400


def test_finish_is_refused_when_recording_is_off(tmp_path):
    with TestClient(recording_app(tmp_path, allow=False)) as client:
        assert client.post("/recording/finish", json={"walk": "walk-1"}).status_code == 403


def test_finish_does_not_clobber_a_model_id_already_recorded(tmp_path):
    """Re-finishing (a double-tap on Stop) must not blank out metadata."""
    import json as _json

    with TestClient(recording_app(tmp_path)) as client:
        walk_dir = post_frame(client, 0).json()["dir"]
        client.post("/recording/finish", json={"walk": "walk-1", "model_id": "m-1"})
        client.post("/recording/finish", json={"walk": "walk-1"})

    assert _json.loads((Path(walk_dir) / "meta.json").read_text())["model_id"] == "m-1"


# ---------- choosing the model for a vision mission ----------
#
# The twin's model picker used to apply only to a one-off /navigate call, so
# turning on "Drive via brain" silently fell back to the service default --
# the bad kind of silent. The model is bound into the mission's vision_fn by
# the factory, which leaves the vision_fn(frame) -> scene contract untouched
# (AGENT-HARNESS.md section 10); nothing below asserts on that contract.


def vision_config(tmp_path, **extra):
    lines = ["brain:", "  vision_url: http://vision.invalid"]
    for k, v in extra.items():
        lines.append(f"  {k}: {v}")
    config = tmp_path / "robot.yaml"
    config.write_text("\n".join(lines) + "\n")
    return str(config)


def capture_model_id(monkeypatch):
    """Intercept vision_fn_for at the brain_server namespace, which is what
    the route actually calls."""
    import control.brain_server as bs

    seen = {}

    def fake_vision_fn_for(target, **kwargs):
        seen["model_id"] = kwargs.get("model_id")
        seen["prompt_variant"] = kwargs.get("prompt_variant")
        return lambda frame: {}

    monkeypatch.setattr(bs, "vision_fn_for", fake_vision_fn_for)
    return seen


def test_a_mission_binds_the_requested_model(tmp_path, monkeypatch):
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "red backpack", "policy": "vision",
            "model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0",
        })
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert seen["model_id"] == "us.anthropic.claude-opus-4-5-20251101-v1:0"


def test_the_configured_default_is_used_when_the_request_names_none(tmp_path, monkeypatch):
    """The headless case: once the brain runs on the Pi a mission can start
    with no twin to pick a model, so config has to be able to pin one."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_model_id="qwen.qwen3-vl-235b-a22b"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "x", "policy": "vision"})
        client.post("/mission/stop")

    assert seen["model_id"] == "qwen.qwen3-vl-235b-a22b"


def test_the_request_overrides_the_configured_default(tmp_path, monkeypatch):
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_model_id="amazon.nova-lite-v1:0"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "model_id": "qwen.qwen3-vl-235b-a22b"})
        client.post("/mission/stop")

    assert seen["model_id"] == "qwen.qwen3-vl-235b-a22b"


def test_no_model_anywhere_means_no_preference_not_a_guessed_name(tmp_path, monkeypatch):
    """None must reach the request as an absent field -- control/ never gets
    to invent a Bedrock model id."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "x", "policy": "vision"})
        client.post("/mission/stop")

    assert seen["model_id"] is None


def test_an_unknown_model_is_refused_at_start_not_after_three_failed_ticks(tmp_path, monkeypatch):
    """Without start-time validation this becomes a 400 inside a tick, which
    burns failsafe B3.2's budget and then reports "vision failed 3 times" --
    which says nothing about the real cause."""
    import control.brain_server as bs

    class FakeResponse:
        status_code = 200

        def raise_for_status(self):
            pass

        def json(self):
            return {"models": [{"id": "amazon.nova-lite-v1:0"}]}

    class FakeClient:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def get(self, url, headers=None):
            return FakeResponse()

    monkeypatch.setattr(bs.httpx, "Client", lambda **kw: FakeClient())
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "model_id": "made.up-model"})

    assert resp.status_code == 400
    assert "not offered" in resp.json()["detail"]
    assert "amazon.nova-lite-v1:0" in resp.json()["detail"]


def test_a_models_endpoint_outage_does_not_block_the_mission(tmp_path, monkeypatch):
    """An unknown model is a caller error; an unreachable models endpoint is
    an outage. Refusing to start because a *validation* call failed would
    turn a soft problem into a hard one -- B3.2 already covers what happens
    next."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)

    def boom(**kw):
        raise bs.httpx.ConnectError("vision service unreachable")

    monkeypatch.setattr(bs.httpx, "Client", boom)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "model_id": "amazon.nova-lite-v1:0"})
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert seen["model_id"] == "amazon.nova-lite-v1:0"


def test_a_mission_binds_the_requested_prompt_variant(tmp_path, monkeypatch):
    """The wording is the other half of what makes a walk attributable, and
    until now the brain dropped it on the floor."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "red backpack", "policy": "vision",
            "prompt_variant": "bearing-only",
        })
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert seen["prompt_variant"] == "bearing-only"


def test_the_twins_prompt_variant_is_no_longer_silently_dropped(tmp_path, monkeypatch):
    """This is the bug this field was added for, and it was live.

    web-twin/app.js has been sending prompt_variant on "Drive via brain"
    since the picker shipped. MissionStartRequest had no such field, and
    pydantic's default is to DISCARD an unknown one -- so the mission ran the
    service's default wording while the UI showed the operator's pick. That
    is exactly the NavigateModelId trap the Stage 0 notes record: a value
    that was passed, ignored, and then reasoned about as though it had
    applied. Re-introduce the missing field and this test goes red."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        # Byte-for-byte the body web-twin/app.js sends from Robot view.
        resp = client.post("/mission/start", json={
            "target_object": "red backpack", "policy": "vision",
            "model_id": None, "prompt_variant": "center-third-path"})
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert seen["prompt_variant"] == "center-third-path"


def test_an_unknown_field_is_refused_by_name_rather_than_ignored(tmp_path):
    """The generalisation of the bug above, and Microduck's own lesson
    (`duck-ipc-proto`): refuse per-field, by name, instead of accepting a
    request and quietly running something else. A misspelled parameter is a
    caller error and has to read as one."""
    import control.brain_server as bs

    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "prompt_varient": "bearing-only"})

    assert resp.status_code == 422
    assert "prompt_varient" in resp.text


def test_the_configured_prompt_variant_is_used_when_the_request_names_none(tmp_path, monkeypatch):
    """The headless case again: once B5 puts the brain on the Pi a mission can
    start with no twin in the loop to pick a wording."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_prompt_variant="bearing-only"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "x", "policy": "vision"})
        client.post("/mission/stop")

    assert seen["prompt_variant"] == "bearing-only"


def test_the_request_overrides_the_configured_prompt_variant(tmp_path, monkeypatch):
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_prompt_variant="default"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "prompt_variant": "bearing-only"})
        client.post("/mission/stop")

    assert seen["prompt_variant"] == "bearing-only"


def test_no_prompt_variant_anywhere_means_no_preference(tmp_path, monkeypatch):
    """None reaches the request as an absent field -- control/ never gets to
    invent a wording, the same way it never invents a model id."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "x", "policy": "vision"})
        client.post("/mission/stop")

    assert seen["prompt_variant"] is None


def _models_endpoint_serving(body: dict, monkeypatch):
    import control.brain_server as bs

    class FakeResponse:
        status_code = 200

        def raise_for_status(self):
            pass

        def json(self):
            return body

    class FakeClient:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def get(self, url, headers=None):
            return FakeResponse()

    monkeypatch.setattr(bs.httpx, "Client", lambda **kw: FakeClient())


def test_an_unknown_prompt_variant_is_refused_at_start(tmp_path, monkeypatch):
    """Same reasoning as the model check: a 400 inside a tick would burn
    B3.2's budget and then report "vision failed 3 times", which says nothing
    about a typo in a variant name."""
    import control.brain_server as bs

    _models_endpoint_serving(
        {"models": [{"id": "amazon.nova-lite-v1:0"}],
         "prompts": ["default", "bearing-only"]}, monkeypatch)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "prompt_variant": "bearing-onlyy"})

    assert resp.status_code == 400
    assert "not offered" in resp.json()["detail"]
    assert "bearing-only" in resp.json()["detail"]


def test_a_service_too_old_to_publish_prompts_does_not_block_the_mission(tmp_path, monkeypatch):
    """An empty list is "could not ask", not "nothing is allowed" -- the same
    rule that keeps a models-endpoint outage from turning a soft problem into
    a hard one. The deployed service predates several of these variants."""
    import control.brain_server as bs

    seen = capture_model_id(monkeypatch)
    _models_endpoint_serving({"models": [{"id": "amazon.nova-lite-v1:0"}]}, monkeypatch)
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision", "prompt_variant": "bearing-only"})
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert seen["prompt_variant"] == "bearing-only"


def test_both_axes_are_checked_in_one_round_trip(tmp_path, monkeypatch):
    """One call per mission start, not one per axis. The models endpoint
    publishes both lists together precisely so a client can ask once."""
    import control.brain_server as bs

    calls = []

    class FakeResponse:
        status_code = 200

        def raise_for_status(self):
            pass

        def json(self):
            return {"models": [{"id": "amazon.nova-lite-v1:0"}],
                    "prompts": ["default", "bearing-only"]}

    class FakeClient:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def get(self, url, headers=None):
            calls.append(url)
            return FakeResponse()

    capture_model_id(monkeypatch)
    monkeypatch.setattr(bs.httpx, "Client", lambda **kw: FakeClient())
    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "vision",
            "model_id": "amazon.nova-lite-v1:0", "prompt_variant": "bearing-only"})
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert len(calls) == 1


def test_health_reports_the_prompt_variant_a_vision_mission_would_run(tmp_path):
    """Shown, not inferred -- for the same reason the model id is. A walk
    attributed to the wrong wording is as wrong as one attributed to the
    wrong model."""
    import control.brain_server as bs

    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_prompt_variant="bearing-only"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/health").json()["navigate_prompt_variant"] == "bearing-only"

    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/health").json()["navigate_prompt_variant"] is None


def test_health_reports_the_model_a_vision_mission_would_run(tmp_path):
    """Shown, not inferred: the one real failure this project hit was a week
    of recorded walks attributed to a model that was never running."""
    import control.brain_server as bs

    app = bs.create_app(
        config_path=vision_config(tmp_path, navigate_model_id="qwen.qwen3-vl-235b-a22b"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/health").json()["navigate_model_id"] == "qwen.qwen3-vl-235b-a22b"


def test_health_reports_null_when_the_brain_pins_nothing(tmp_path):
    """null means "the vision service's default applies" -- deliberately not
    resolved here, which would put an HTTP call in a polled health check."""
    import control.brain_server as bs

    app = bs.create_app(config_path=vision_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/health").json()["navigate_model_id"] is None


# ---------- policy: "tiered" (PLAN-onboard-perception.md 4.10, phase P2) ----------
#
# The models run IN THIS PROCESS, and `ultralytics`/`torch` are a
# deliberately optional install (requirements-perception.txt -- a
# multi-gigabyte download no automated test may need). So the property
# that matters most here is a negative one: a brain that cannot load them
# must say so at mission start, naming the fix, rather than starting a
# mission that dies three ticks later reporting "vision unavailable 3
# times in a row" with a robot standing in a room throughout. That is the
# same diagnose-the-wrong-thing failure _validate_navigate_choices() was
# written to prevent for model ids.
#
# Nothing below installs or loads a model: brain_server's own
# _tiered_vision_fn is the seam, and it is patched.


def tiered_config(tmp_path, **extra):
    return vision_config(tmp_path, **extra)


def capture_tiered(monkeypatch, wrapped=None):
    """Intercept the pipeline build, which is the only line that needs the
    heavy dependencies."""
    import control.brain_server as bs

    seen = {}

    def fake_tiered(target, cloud_vision_fn, config):
        seen["target"] = target
        seen["cloud_vision_fn"] = cloud_vision_fn
        seen["config"] = config
        return wrapped or (lambda frame: {})

    monkeypatch.setattr(bs, "_tiered_vision_fn", fake_tiered)
    return seen


def test_the_tiered_policy_wraps_the_cloud_call_rather_than_replacing_it(tmp_path, monkeypatch):
    """2.6's invariant: the cloud vision_fn is unchanged and the tier goes
    in FRONT of it. If the tier replaced it, deliberation would be gone
    rather than rationed."""
    import control.brain_server as bs

    seen_model = capture_model_id(monkeypatch)
    seen_tier = capture_tiered(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(config_path=tiered_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "red backpack", "policy": "tiered",
            "model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0",
            "prompt_variant": "bearing-only",
        })
        client.post("/mission/stop")

    assert resp.status_code == 200, resp.text
    assert resp.json()["status"]["policy"] == "tiered"
    assert seen_tier["target"] == "red backpack"
    # The cloud fn the tier was handed is the one vision_fn_for built, with
    # this mission's model and wording bound into it -- a tiered walk has to
    # be as attributable as a vision one.
    assert seen_tier["cloud_vision_fn"] is not None
    assert seen_model["model_id"] == "us.anthropic.claude-opus-4-5-20251101-v1:0"
    assert seen_model["prompt_variant"] == "bearing-only"


def test_the_tiered_policy_passes_the_configured_trigger_discipline(tmp_path, monkeypatch):
    """6.1's hysteresis is the finding most likely to be skipped, so it has
    to be reachable without editing code."""
    import control.brain_server as bs

    capture_model_id(monkeypatch)
    seen = capture_tiered(monkeypatch)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    app = bs.create_app(
        config_path=tiered_config(tmp_path, tier_consecutive_frames=3,
                                  tier_cold_search_after=9, tier_max_calls=12),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "x", "policy": "tiered"})
        client.post("/mission/stop")

    assert seen["config"]["tier_consecutive_frames"] == 3
    assert seen["config"]["tier_cold_search_after"] == 9
    assert seen["config"]["tier_max_calls"] == 12


def test_a_brain_without_the_perception_models_refuses_at_start_not_mid_tick(tmp_path, monkeypatch):
    """The one that matters. `ultralytics`/`torch` are optional by design,
    so this is a normal state for a fresh checkout -- and the answer to it
    has to be a 400 naming the pip command, before any motor turns."""
    import control.brain_server as bs
    from brain.perceive import PerceptionUnavailable

    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)

    def no_models(target, **kwargs):
        raise PerceptionUnavailable(
            "ultralytics is not installed. `pip install -r "
            "requirements-perception.txt`")

    monkeypatch.setattr("brain.perceive.pipeline_for", no_models)
    app = bs.create_app(config_path=tiered_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "red backpack", "policy": "tiered"})
        status = client.get("/mission/status").json()

    assert resp.status_code == 400, resp.text
    detail = resp.json()["detail"]
    assert "requirements-perception.txt" in detail, detail
    # And no mission was left running behind the refusal.
    assert status["running"] is False


def test_a_broken_weights_file_is_also_a_start_time_refusal(tmp_path, monkeypatch):
    """Not only the ImportError path: a named detector that will not load is
    the same class of problem and must not become three vision failures."""
    import control.brain_server as bs

    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    monkeypatch.setattr(
        "brain.perceive.pipeline_for",
        lambda target, **kw: (_ for _ in ()).throw(OSError("no such file: nope.pt")))
    app = bs.create_app(config_path=tiered_config(tmp_path, perception_detector="nope.pt"),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))

    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "tiered"})

    assert resp.status_code == 400, resp.text
    assert "nope.pt" in resp.json()["detail"]


def test_the_tiered_policy_needs_a_vision_service_and_an_object(tmp_path, monkeypatch):
    """`mission_start` alone guarantees one paid call, so "cheaper" is not
    "offline" -- and the target string is what CLIP scores every crop
    against, so a room-only mission has nothing to perceive either."""
    import control.brain_server as bs

    capture_tiered(monkeypatch)
    no_vision = tmp_path / "robot.yaml"
    no_vision.write_text('brain:\n  vision_url: ""\n')
    app = bs.create_app(config_path=str(no_vision),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={
            "target_object": "x", "policy": "tiered"})
    assert resp.status_code == 400
    assert "vision_url" in resp.json()["detail"]

    app2 = bs.create_app(config_path=tiered_config(tmp_path),
                         robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app2) as client:
        resp = client.post("/mission/start", json={
            "target_room": "kitchen", "policy": "tiered"})
    assert resp.status_code == 400
    assert "target_object" in resp.json()["detail"]


def test_health_says_whether_the_tiered_policy_can_run_here(tmp_path):
    """So the twin can grey the policy out rather than offering one that
    will 400 -- and can name the detector on screen before anything is
    spent. Description, never verdict: a brain with no perception extras is
    perfectly healthy for the other two policies."""
    app = create_app(config_path=tiered_config(tmp_path),
                     robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        health = client.get("/health").json()

    assert isinstance(health["perception_available"], bool)
    assert health["perception_detector"] == "yolo11s.pt"
    assert health["perception_clip_model"] == "RN50"
    assert health["tier_consecutive_frames"] == 2


def test_health_names_the_detector_the_config_actually_pins(tmp_path):
    """*"Swap the HEF and the name on screen changes; that is the experiment
    loop made watchable."* It is only watchable if the name comes from what
    would really be loaded."""
    app = create_app(
        config_path=tiered_config(tmp_path, perception_detector="yolo11n.pt",
                                  perception_clip_model="ViT-B-32"),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        health = client.get("/health").json()

    assert health["perception_detector"] == "yolo11n.pt"
    assert health["perception_clip_model"] == "ViT-B-32"


def test_the_real_wrapper_builds_a_tiered_vision_fn_over_the_cloud_one(tmp_path, monkeypatch):
    """The wiring itself, with only `pipeline_for` faked -- every test above
    stubs `_tiered_vision_fn` wholesale, which leaves the line that actually
    assembles the tier unexercised. A fake pipeline is enough: what is under
    test is that the config's knobs reach brain/tiered.py and that the cloud
    fn ends up behind it rather than beside it."""
    import control.brain_server as bs
    from brain.perceive import ABSENT, Perception

    class FakePipeline:
        target = "red backpack"
        crop_source = "label_gate"
        detector = type("D", (), {"weights": "fake.pt"})()
        scorer = type("S", (), {"model_name": "FakeCLIP"})()

        def perceive(self, frame):
            return Perception(status=ABSENT, reason="fake")

    seen = {}

    def fake_pipeline_for(target, **kwargs):
        seen["target"] = target
        seen["kwargs"] = kwargs
        return FakePipeline()

    monkeypatch.setattr("brain.perceive.pipeline_for", fake_pipeline_for)

    config = load_brain_config(tiered_config(
        tmp_path, perception_detector="fake.pt", perception_clip_model="FakeCLIP",
        tier_consecutive_frames=3, tier_cold_search_after=9, tier_max_calls=5))
    cloud_calls = []

    tier = bs._tiered_vision_fn("red backpack", lambda frame: cloud_calls.append(frame) or {
        "safest_direction": "FORWARD", "_navigate": {"reasoning": "cloud"}}, config)

    assert seen["kwargs"] == {"weights": "fake.pt", "clip_model": "FakeCLIP"}
    assert tier.consecutive_frames == 3
    assert tier.cold_search_after == 9
    assert tier.max_calls == 5
    # Phase A is on by default since 2026-09-12, so mission_start is
    # DISPATCHED rather than waited on. 2.5 anticipated exactly this --
    # *"mission start is the one genuinely blocking call, or it is not, if
    # the opening default is 'look around'"* -- and the opening default is
    # now the stand-in's scan until the answer lands.
    assert tier.async_cloud is True
    scenes = [tier({"image_base64": "eA==", "image_width": 640}) for _ in range(4)]
    assert scenes[0]["_tier"]["in_flight"] == "mission_start"
    assert scenes[0]["_tier"]["cloud_called"] is False
    assert tier.stats.cloud_calls == 1, "one call, dispatched not repeated"
    assert scenes[-1]["_tier"]["cloud_called"] is False
    assert scenes[-1]["_tier"]["models"]["detector"] == "fake.pt"
    tier.close()


def test_the_blocking_path_is_still_available_and_still_blocks(tmp_path, monkeypatch):
    """Phase A is a default, not a removal. A mission configured
    synchronously must behave exactly as it did -- the failsafe drills and
    every pre-2026-09-12 walk were recorded against that path."""
    import control.brain_server as bs

    from brain.perceive import ABSENT, Perception

    class FakePipeline:
        target = "red backpack"
        crop_source = "label_gate"
        detector = type("D", (), {"weights": "fake.pt"})()
        scorer = type("S", (), {"model_name": "FakeCLIP"})()

        def perceive(self, frame):
            return Perception(status=ABSENT, reason="fake")

    monkeypatch.setattr("brain.perceive.pipeline_for",
                        lambda target, **kw: FakePipeline())
    config = load_brain_config(tiered_config(
        tmp_path, perception_detector="fake.pt", tier_async_cloud=False))
    cloud_calls = []
    tier = bs._tiered_vision_fn("red backpack", lambda f: cloud_calls.append(f) or {
        "safest_direction": "FORWARD", "_navigate": {"reasoning": "cloud"}}, config)

    assert tier.async_cloud is False
    scene = tier({"image_base64": "eA==", "image_width": 640})
    assert scene["_tier"]["trigger"] == "mission_start"
    assert scene["_tier"]["cloud_called"] is True
    assert len(cloud_calls) == 1
    assert tier._executor is None, "a synchronous mission started a thread"


def test_a_half_installed_perception_package_reads_as_unavailable(monkeypatch):
    """`find_spec` raises rather than returning None when a package is
    present but broken. Either way it cannot be used, and the health field
    must not become an exception inside a route the twin polls."""
    import control.brain_server as bs

    def boom(name):
        raise ValueError(f"{name}.__spec__ is None")

    monkeypatch.setattr("importlib.util.find_spec", boom)
    assert bs._perception_available() is False


def test_the_clip_margin_is_settable_without_editing_code(tmp_path, monkeypatch):
    """DEFAULT_MATCH_MARGIN is uncalibrated by its own admission, and the way
    to calibrate it is to vary it on a rig walk. That needs a config key, not
    a source edit -- and the default must stay put until a valid corpus says
    otherwise."""
    import control.brain_server as bs

    seen = {}
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    monkeypatch.setattr("brain.perceive.pipeline_for",
                        lambda target, **kw: seen.update(kw) or _FakePipeline())

    config = load_brain_config(tiered_config(tmp_path, perception_match_margin=0.02))
    bs._tiered_vision_fn("red backpack", lambda f: {}, config)
    assert seen["match_margin"] == 0.02

    seen.clear()
    config = load_brain_config(tiered_config(tmp_path, perception_match_probability=0.9))
    bs._tiered_vision_fn("red backpack", lambda f: {}, config)
    assert seen["match_probability"] == 0.9

    # Unset means "whatever brain/perceive.py says", passed as an absent
    # kwarg rather than a number this module invented.
    seen.clear()
    bs._tiered_vision_fn("red backpack", lambda f: {}, load_brain_config(tiered_config(tmp_path)))
    assert "match_margin" not in seen


class _FakePipeline:
    target = "red backpack"
    crop_source = "label_gate"
    detector = type("D", (), {"weights": "fake.pt"})()
    scorer = type("S", (), {"model_name": "FakeCLIP"})()

    def perceive(self, frame):
        from brain.perceive import ABSENT, Perception
        return Perception(status=ABSENT)


def test_health_reports_the_gate_a_mission_would_use(tmp_path):
    """The gate is the probability; the margin is an override and reads null
    unless one is set. Both are reported so a walk can say which produced
    it -- the raw margin is not comparable across targets, and a walk that
    cannot name its gate cannot be compared with another."""
    app = create_app(config_path=tiered_config(tmp_path),
                     robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        h = client.get("/health").json()
        assert h["perception_match_probability"] == 0.8
        assert h["perception_match_margin"] is None

    app2 = create_app(
        config_path=tiered_config(tmp_path, perception_match_probability=0.9,
                                  perception_match_margin=0.02),
        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app2) as client:
        h = client.get("/health").json()
        assert h["perception_match_probability"] == 0.9
        assert h["perception_match_margin"] == 0.02
