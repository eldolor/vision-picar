"""
The remaining small branches, mostly error paths and edges.

Nothing here is exotic; they are the lines that no scenario test happened to
walk through. Several are worth pinning on their own merits -- the mission
runner's refusal to be restarted, its swallowing of a policy exception, the
static assets the twin needs to install as a PWA -- and the rest are cheap.
"""

import json

import pytest
from fastapi.testclient import TestClient

from brain.memory import MissionMemory
from control.walk_eval import _entry_actions, _parse_json_reply, judge_walk
from sim.grid_world import GridWorld


# ---------- brain ----------


def test_memory_knows_which_rooms_it_has_searched():
    """Feeds /navigate's searched_rooms, which is the room-level step memory
    a single photograph cannot carry."""
    m = MissionMemory(mission="find it", target_object="red backpack")
    m.mark_room_searched("kitchen")
    m.mark_room_searched("unknown")  # never recorded -- it is not a room
    assert m.is_room_searched("kitchen")
    assert not m.is_room_searched("hallway")
    assert not m.is_room_searched("unknown")


def test_the_anthropic_client_is_built_once_and_reused(monkeypatch):
    """brain/vision.py's client is lazy so importing the module needs no API
    key -- the whole automated suite runs without one."""
    import anthropic

    import brain.vision as vision

    built = []
    monkeypatch.setattr(vision, "_client", None)
    monkeypatch.setattr(anthropic, "Anthropic", lambda *a, **k: built.append(1) or "client")

    assert vision._get_client() == "client"
    assert vision._get_client() == "client"
    assert len(built) == 1


# ---------- sim ----------


def test_reading_outside_the_grid_is_a_wall():
    """The raycaster and the distance sensor both walk off the map edge; a
    bounds check that returned None would crash them instead of stopping
    them."""
    from sim.grid_world import CELL_WALL

    w = GridWorld(layout=["...", "..."], rooms={})
    assert w._cell(-1, 0) == CELL_WALL
    assert w._cell(0, -1) == CELL_WALL
    assert w._cell(99, 0) == CELL_WALL
    assert w._cell(0, 99) == CELL_WALL
    assert not w._is_passable(-1, 0)


# ---------- walk_eval odds and ends ----------


def test_entry_actions_tolerates_a_missing_navigate_block():
    assert _entry_actions([{"seq": 0}, {"seq": 1, "navigate": {"action": "LEFT"}}]) == [None, "LEFT"]


def test_a_judge_reply_with_no_json_is_a_clear_error():
    with pytest.raises(ValueError) as e:
        _parse_json_reply("I'm afraid I can't do that")
    assert "no JSON object" in str(e.value)


def test_judging_a_walk_with_no_entries_says_so():
    out = judge_walk(None, "m", [], lambda e: b"x", "red backpack")
    assert out["judged"] == 0
    assert out["error"] == "no entries"


# ---------- robot/server.py static assets ----------


@pytest.mark.parametrize("path,content_type", [
    ("/manifest.json", "application/manifest+json"),
    ("/icons/icon-192.png", "image/png"),
    ("/icons/icon-512.png", "image/png"),
    ("/icons/apple-touch-icon.png", "image/png"),
])
def test_the_pwa_assets_are_served(path, content_type):
    """Without these the twin cannot be installed to a home screen, which is
    how it is actually used -- and they 404'd silently once already for want
    of a load-balancer path."""
    from robot.server import create_app

    with TestClient(create_app()) as client:
        resp = client.get(path)
        assert resp.status_code == 200, path
        assert resp.headers["content-type"].startswith(content_type.split("+")[0])


# ---------- brain_server validation ----------


def _recording_app(tmp_path, **extra):
    from control.brain_server import create_app

    lines = ["brain:", "  allow_recording: true", f"  recording_dir: {tmp_path / 'rec'}"]
    for k, v in extra.items():
        lines.append(f"  {k}: {v}")
    config = tmp_path / "robot.yaml"
    config.write_text("\n".join(lines) + "\n")
    return create_app(config_path=str(config))


def test_a_frame_sequence_number_is_bounded(tmp_path):
    """seq becomes a filename (frame-%04d) -- an unbounded one would either
    collide or escape the format."""
    import base64

    with TestClient(_recording_app(tmp_path)) as client:
        body = {"walk": "w", "seq": 10000,
                "image_base64": base64.b64encode(b"\xff\xd8").decode()}
        assert client.post("/recording/frame", json=body).status_code == 400
        body["seq"] = -1
        assert client.post("/recording/frame", json=body).status_code == 400


def test_finishing_a_walk_survives_a_corrupt_meta_file(tmp_path):
    """meta.json is rewritten on every finish; a truncated one must not stop
    a walk being marked complete."""
    import base64

    app = _recording_app(tmp_path)
    with TestClient(app) as client:
        client.post("/recording/frame", json={
            "walk": "w", "seq": 0, "image_base64": base64.b64encode(b"\xff\xd8").decode()})
        (tmp_path / "rec" / "w" / "meta.json").write_text("{truncated")
        resp = client.post("/recording/finish", json={"walk": "w", "model_id": "m-1"})

    assert resp.status_code == 200
    assert json.loads((tmp_path / "rec" / "w" / "meta.json").read_text())["model_id"] == "m-1"


# ---------- the watchdog's own loop, and the remaining brain guards ----------


def test_the_watchdog_decision_is_a_pure_function():
    """Failsafe B3.1. The decision is here; the live async loop that calls it
    is exercised against a real server in tests/test_watchdog_integration.py,
    because it needs real wall-clock time on a real event loop."""
    from robot.server import watchdog_should_stop

    assert watchdog_should_stop(last_command_at=0.0, now=2.0, timeout_s=1.0) is True
    assert watchdog_should_stop(last_command_at=0.0, now=0.5, timeout_s=1.0) is False
    assert watchdog_should_stop(last_command_at=0.0, now=1.0, timeout_s=1.0) is False


def test_a_bad_walk_name_is_refused_by_the_brains_finish_route(tmp_path):
    from control.brain_server import create_app

    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  allow_recording: true\n  recording_dir: {tmp_path / 'rec'}\n")
    with TestClient(create_app(config_path=str(config))) as client:
        assert client.post("/recording/finish", json={"walk": "../escape"}).status_code == 400


def test_a_brain_with_no_storage_proxies_the_finish_signal(tmp_path):
    """teleop-brain has no EFS mount of its own and forwards recording writes
    to the brain that does -- the finish signal has to travel the same road as
    the frames, or a proxied walk is never marked complete."""
    from control.brain_server import create_app

    config = tmp_path / "robot.yaml"
    config.write_text(
        "brain:\n  allow_recording: false\n"
        "  recording_proxy_url: http://peer-brain.internal\n")
    app = create_app(config_path=str(config))

    forwarded = {}

    class FakeResponse:
        status_code = 200

        def json(self):
            return {"walk": "w", "meta": {"model_id": "m"}}

    class FakeClient:
        def __init__(self, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            forwarded["url"] = url
            forwarded["json"] = json
            return FakeResponse()

    import control.brain_server as bs

    original = bs.httpx.Client
    bs.httpx.Client = FakeClient
    try:
        with TestClient(app) as client:
            resp = client.post("/recording/finish", json={"walk": "w", "model_id": "m"})
    finally:
        bs.httpx.Client = original

    assert resp.status_code == 200
    assert forwarded["url"].endswith("/recording/finish")
    assert forwarded["json"]["walk"] == "w"


def test_the_default_robot_is_a_remote_one(tmp_path):
    """control/ may not import a backend or the simulator -- the robot is
    only ever reachable over HTTP, which is what makes "brain on the Pi" and
    "brain on a MacBook" a config difference (tests/test_brain_server.py
    asserts the import surface). Every other test injects a robot, so this is
    the only cover for the factory the real deployment actually uses."""
    from control.brain_server import create_app
    from control.remote_robot import RemoteRobot

    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  robot_url: http://pi.local:8000\n")
    app = create_app(config_path=str(config))

    # Stopping a mission builds the robot to stop it, even with none running.
    with TestClient(app) as client:
        built = {}
        import control.brain_server as bs

        real = bs.RemoteRobot

        class Spy(real):
            def __init__(self, *a, **kw):
                built["url"] = a[0] if a else kw.get("base_url")
                super().__init__(*a, **kw)

            def stop(self):
                return {"stopped": True}

        bs.RemoteRobot = Spy
        try:
            client.post("/mission/stop")
        finally:
            bs.RemoteRobot = real

    assert built["url"] == "http://pi.local:8000"


def test_a_walk_hitting_the_frame_cap_is_refused(tmp_path, monkeypatch):
    """MAX_FRAMES_PER_WALK bounds what one recording can put on a shared
    volume -- a phone left running would otherwise fill it."""
    import base64

    import control.brain_server as bs

    monkeypatch.setattr(bs, "MAX_FRAMES_PER_WALK", 2)
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  allow_recording: true\n  recording_dir: {tmp_path / 'rec'}\n")

    img = base64.b64encode(b"\xff\xd8").decode()
    with TestClient(bs.create_app(config_path=str(config))) as client:
        for seq in range(2):
            assert client.post("/recording/frame", json={
                "walk": "w", "seq": seq, "image_base64": img}).status_code == 200
        resp = client.post("/recording/frame", json={"walk": "w", "seq": 2, "image_base64": img})

    assert resp.status_code == 409
    assert "2 frames" in resp.json()["detail"]


def test_a_walk_that_runs_out_of_steps_says_so():
    """The step budget, which is the only one of the three that fires on a
    mission that is working -- just not fast enough."""
    from control.mission_runner import MissionRunner

    from tests.conftest import fresh_mock_robot

    # A target that is not in the starter house, so the mission cannot end by
    # succeeding before the budget runs out.
    runner = MissionRunner(robot=fresh_mock_robot(),
                           target_object="a thing that is not in this house",
                           max_steps=2)
    runner.start()
    for _ in range(6):
        if not runner.tick():
            break

    status = runner.status()
    assert status["outcome"] == "max_steps"
    assert "budget" in " ".join(status["log_tail"]).lower()


def test_a_judged_frame_whose_image_is_gone_is_skipped():
    """walk.jsonl outlives a deleted frame; the judge and the collision check
    both have to tolerate an entry with no pixels behind it."""
    from control.walk_eval import check_collisions

    entries = [{"seq": 0, "file": "frame-0000.jpg",
                "navigate": {"action": "FORWARD"}}]
    out = check_collisions(object(), "m", entries, lambda e: None)
    assert out["checked"] == 0
    assert out["collisions"] == []
