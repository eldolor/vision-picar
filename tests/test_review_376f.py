"""
tests/test_review_376f.py

3.76 batch F: the whole-file review of recorded walks and their scoring
(`control/walk_*`, `recording_routes.py`, `admin_server.py`,
`perception_eval.py`, `label_assist.py`, `metrics_*`). One test per
confirmed finding, each red on `dev` before its fix; the ids are the
findings table's in `docs/plans/ros-alignment/3.76-legacy-review.md`.
"""

import json
import shutil
import sys
import tempfile
import time
import types
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from control import admin_server, label_assist, perception_eval, walk_eval
from control.metrics_client import row_for
from control.metrics_routes import register_metrics_routes
from control.perception_eval import FrameScore, save_records
from control.walk_store import LocalWalkStore


@pytest.fixture
def recordings(tmp_path):
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  recording_dir: {tmp_path / 'recordings'}\n"
                      f"  vision_url: http://vision.invalid\n")
    (tmp_path / "recordings").mkdir()
    return tmp_path, str(config)


@pytest.fixture
def client(recordings):
    root, config = recordings
    return TestClient(admin_server.create_app(config_path=config)), root


def make_walk(root, name, actions):
    walk_dir = root / "recordings" / name
    walk_dir.mkdir(parents=True)
    lines = []
    for seq, action in enumerate(actions):
        (walk_dir / f"frame-{seq:04d}.jpg").write_bytes(b"\xff\xd8\xff\xd9")
        nav = {"action": action, "target_visible": True, "target_reached": False,
               "obstacle_ahead": False, "reasoning": "r"}
        lines.append(json.dumps({"seq": seq, "file": f"frame-{seq:04d}.jpg",
                                 "media_type": "image/jpeg", "navigate": nav}))
    (walk_dir / "walk.jsonl").write_text("\n".join(lines) + "\n")
    return walk_dir


def reply(text):
    return {"output": {"message": {"content": [{"text": text}]}},
            "usage": {"inputTokens": 100, "outputTokens": 10}}


# ---------- F1: a judge or collision outage stored as a finished score ----------


class Throttled:
    """Bedrock throttling every call: the judge and the collision check both
    come back with nothing."""

    def converse(self, **kw):
        raise RuntimeError("ThrottlingException")


def test_f1_a_judge_outage_is_marked_and_kept_out_of_the_summary(client, monkeypatch):
    """Red on dev: the outage scored as a plain metrics-only card at the
    current schema, uncapped by a collision check that never ran, and counted
    in /recording/summary like any other walk."""
    c, root = client
    make_walk(root, "walk-out", ["FORWARD"] * 6)
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: Throttled())

    body = c.post("/recording/walks/walk-out/evaluate").json()
    assert "judge-incomplete" in body["flags"]
    assert "collisions-unchecked" in body["flags"]
    assert c.get("/recording/summary").json()["rows"] == []


def test_f1_a_partly_failed_collision_check_is_marked(client, monkeypatch):
    """One collision call throttled out of six: the check is not a clean bill."""
    c, root = client
    make_walk(root, "walk-part", ["FORWARD"] * 6)
    calls = {"n": 0}

    class OneFails:
        def converse(self, modelId, messages, inferenceConfig):
            text = messages[0]["content"][1]["text"]
            if "would_collide" in text:
                calls["n"] += 1
                if calls["n"] == 1:
                    raise RuntimeError("ThrottlingException")
                return reply('{"would_collide": false, "why": "clear"}')
            return reply('{"sensible": true, "better_action": null, "why": "ok"}')

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: OneFails())
    body = c.post("/recording/walks/walk-part/evaluate").json()
    assert body["collisions"]["failed"] == 1
    assert "collisions-unchecked" in body["flags"]
    assert "judge-incomplete" not in body["flags"]


def test_f1_a_replay_whose_collision_check_failed_is_marked(recordings, monkeypatch):
    root, config = recordings
    make_walk(root, "walk-rp", ["FORWARD"] * 4)
    monkeypatch.setattr(admin_server, "post_navigate",
                        lambda *a, **k: {"action": "FORWARD", "target_visible": True})
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: Throttled())
    c = TestClient(admin_server.create_app(config_path=config))

    body = c.post("/recording/walks/walk-rp/replay", json={"model_id": "m"}).json()
    assert "collisions-unchecked" in body["flags"]
    assert c.get("/recording/summary").json()["rows"] == []


# ---------- F2: the summary averaged two different formulas ----------


def test_f2_judged_and_metrics_only_scores_are_separate_rows(client):
    """Red on dev: one row, mean of a judged 30 and a metrics-only 90."""
    c, root = client
    for name, basis, score in (("walk-a", "judge+metrics", 30),
                               ("walk-b", "metrics-only", 90)):
        d = make_walk(root, name, ["FORWARD"] * 3)
        (d / "eval.json").write_text(json.dumps({
            "schema": walk_eval.SCHEMA_VERSION, "score": score, "basis": basis,
            "flags": [], "metrics": {"frames": 3}}))
    rows = c.get("/recording/summary").json()["rows"]
    assert sorted((r["basis"], r["mean_score"]) for r in rows) == \
        [("judge+metrics", 30), ("metrics-only", 90)]


# ---------- F3: a replay that outlives the Lambda leaves no record ----------


def test_f3_a_replay_stops_sending_frames_at_its_deadline(recordings, monkeypatch):
    """Red on dev: every frame waits its turn, however long the vision service
    takes, so on the Lambda the run is killed at 900 s with nothing stored.
    Scaled down: a 0.3 s deadline, 0.2 s calls, 90 frames over 6 workers --
    3 s without the deadline, ~0.5 s with it, so the 1.5 s bar holds under a
    loaded suite (30 frames put the red 1.0 s beside a 0.9 s bar)."""
    root, config = recordings
    make_walk(root, "walk-slow", ["FORWARD"] * 90)

    def slow(*a, **k):
        time.sleep(0.2)
        return {"action": "FORWARD"}

    monkeypatch.setattr(admin_server, "post_navigate", slow)
    monkeypatch.setattr(admin_server, "REPLAY_DEADLINE_S", 0.3, raising=False)
    c = TestClient(admin_server.create_app(config_path=config))

    started = time.monotonic()
    body = c.post("/recording/walks/walk-slow/replay", json={"model_id": "m"}).json()
    assert time.monotonic() - started < 1.5
    assert body["verdict"] == "unusable"
    assert any("deadline" in (d.get("error") or "") for d in body["diff"])
    stored = root / "recordings" / "walk-slow" / "replay-m.json"
    assert stored.exists(), "the attempt must leave a record"


def test_f3_post_navigate_starts_no_retry_after_the_deadline(monkeypatch):
    import httpx

    attempts = {"n": 0}

    class Down:
        def __init__(self, timeout=None):
            self.timeout = timeout

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            attempts["n"] += 1
            return httpx.Response(503, text="down")

    monkeypatch.setattr(httpx, "Client", Down)
    monkeypatch.setattr(admin_server, "REPLAY_MIN_ATTEMPT_S", 0.0, raising=False)
    real_sleep = time.sleep
    monkeypatch.setattr("time.sleep", lambda s: real_sleep(0.05))
    with pytest.raises(RuntimeError, match="503"):
        admin_server.post_navigate("http://vision.invalid", 5.0, b"jpg", "bag", None,
                                   deadline=time.monotonic() + 0.01)
    assert attempts["n"] == 1


# ---------- F4: deleting a frame ----------


def test_f4_a_malformed_manifest_line_does_not_half_delete_a_frame(client):
    """Red on dev: the image went, then json.loads raised (500), and walk.jsonl
    still pointed at the deleted frame."""
    c, root = client
    d = make_walk(root, "walk-m", ["FORWARD"] * 3)
    with (d / "walk.jsonl").open("a") as f:
        f.write('{"seq": 9, "file": "frame-00\n')
    resp = c.delete("/recording/walks/walk-m/frames/frame-0001.jpg")
    assert resp.status_code == 200, resp.text
    manifest = (d / "walk.jsonl").read_text()
    assert "frame-0001.jpg" not in manifest
    assert '{"seq": 9, "file": "frame-00' in manifest, "a line it cannot read is not its to drop"


def test_f4_a_deleted_frame_marks_the_scores_that_counted_it(client):
    """Red on dev: the scorecard and the replays kept presenting a score of
    frames the walk no longer has as current."""
    c, root = client
    d = make_walk(root, "walk-d", ["FORWARD"] * 3)
    c.post("/recording/walks/walk-d/evaluate?judge=false")
    (d / "replay-m.json").write_text(json.dumps({
        "schema": walk_eval.SCHEMA_VERSION, "model_id": "m", "score": 50, "flags": []}))
    assert c.delete("/recording/walks/walk-d/frames/frame-0001.jpg").status_code == 200

    walk = next(w for w in c.get("/recording/walks").json()["walks"] if w["walk"] == "walk-d")
    assert walk["eval"]["stale"] is True
    assert walk["replays"][0]["stale"] is True
    assert c.get("/recording/summary").json()["rows"] == []


# ---------- F5: a score or replay that outlives a walk delete ----------


def test_f5_a_replay_finishing_after_its_walk_was_deleted_does_not_bring_it_back(
        recordings, monkeypatch):
    """Red on dev: write_json recreated the directory, and the deleted walk
    came back as a 0-frame walk holding one replay sidecar."""
    root, config = recordings
    d = make_walk(root, "walk-gone", ["FORWARD"] * 2)

    def deleting(*a, **k):
        shutil.rmtree(d, ignore_errors=True)
        return {"action": "FORWARD"}

    monkeypatch.setattr(admin_server, "post_navigate", deleting)
    c = TestClient(admin_server.create_app(config_path=config))
    c.post("/recording/walks/walk-gone/replay", json={"model_id": "m"})
    assert not d.exists()


def test_f5_a_score_finishing_after_its_walk_was_deleted_does_not_bring_it_back(
        client, monkeypatch):
    c, root = client
    d = make_walk(root, "walk-gone2", ["FORWARD"] * 2)

    class Deleting:
        def converse(self, **kw):
            shutil.rmtree(d, ignore_errors=True)
            return reply('{"sensible": true, "would_collide": false}')

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: Deleting())
    c.post("/recording/walks/walk-gone2/evaluate")
    assert not d.exists()


# ---------- F6: a walk named like a metrics day ----------


def test_f6_the_write_routes_refuse_a_metrics_day_name(client):
    """Red on dev: stored, then invisible -- list_walks() leaves the metrics
    prefix out, so the walk could never be listed, scored or deleted."""
    c, root = client
    body = {"walk": "metrics-2026-10-14", "seq": 0, "image_base64": "/9j/2Q=="}
    assert c.post("/recording/frame", json=body).status_code == 400
    assert c.post("/recording/finish", json={"walk": "metrics-2026-10-14"}).status_code == 400
    assert not (root / "recordings" / "metrics-2026-10-14").exists()


# ---------- F7: a name with ".." is a 500 ----------


def test_f7_a_dotdot_walk_name_is_a_400_not_a_500(client):
    c, _ = client
    assert c.get("/recording/walks/a..b").status_code == 400
    assert c.delete("/recording/walks/a..b").status_code == 400


def test_f7_a_dotdot_run_id_is_a_400_not_a_500():
    app = FastAPI()
    register_metrics_routes(app, LocalWalkStore(tempfile.mkdtemp()), lambda: None)
    c = TestClient(app, raise_server_exceptions=False)
    assert c.post("/metrics/runs", json={"run_id": "x..y"}).status_code == 400


# ---------- F8: a stray file in a local walk ----------


def test_f8_a_stray_file_does_not_break_the_walk(client):
    """Red on dev: Finder's .DS_Store made every download of the walk a 500."""
    c, root = client
    d = make_walk(root, "walk-ds", ["FORWARD"] * 2)
    (d / ".DS_Store").write_bytes(b"\0")
    (root / "recordings" / "My Walk").mkdir()
    resp = c.get("/recording/walks/walk-ds/download")
    assert resp.status_code == 200
    assert c.get("/recording/walks").status_code == 200


# ---------- F9: label_assist paid again for a reply it could not read ----------


def _fake_boto3(monkeypatch, replies):
    calls = {"n": 0}

    class Client:
        def converse(self, **kw):
            calls["n"] += 1
            r = replies[min(calls["n"] - 1, len(replies) - 1)]
            if isinstance(r, Exception):
                raise r
            return reply(r)

    monkeypatch.setitem(sys.modules, "boto3",
                        types.SimpleNamespace(client=lambda *a, **k: Client()))
    monkeypatch.setattr("time.sleep", lambda s: None)
    return calls


def _label_walk(tmp_path):
    d = tmp_path / "w"
    d.mkdir()
    (d / "meta.json").write_text(json.dumps({"target_object": "red bag"}))
    (d / "frame-0000.jpg").write_bytes(b"\xff\xd8\xff\xd9")
    return d


def test_f9_an_unreadable_reply_is_not_paid_for_again(tmp_path, monkeypatch):
    """Red on dev: four paid calls for one frame whose reply had a null
    confidence, and none of their tokens in `usage`."""
    calls = _fake_boto3(monkeypatch, ['{"visible": true, "confidence": null}'])
    doc = label_assist.propose_cloud(_label_walk(tmp_path), "m", workers=1)
    assert calls["n"] == 1
    assert doc["proposed_by"]["errors"] == 1
    assert doc["usage"] == {"input_tokens": 100, "output_tokens": 10}


def test_f9_a_transport_error_is_still_retried(tmp_path, monkeypatch):
    calls = _fake_boto3(monkeypatch, [RuntimeError("throttled"),
                                      '{"visible": true, "confidence": 0.95}'])
    doc = label_assist.propose_cloud(_label_walk(tmp_path), "m", workers=1)
    assert calls["n"] == 2
    assert doc["proposed_by"]["errors"] == 0


# ---------- F10, F11: comparing saved perception runs ----------


def test_f10_compare_refuses_runs_over_different_frames(tmp_path, capsys):
    """Red on dev: a run scored before the corpus grew was tabled beside a
    newer one, recall over different denominators, and only the first file's
    frame count was printed."""
    a, b = tmp_path / "a.json", tmp_path / "b.json"
    save_records(a, [FrameScore("w", "frame-0000.jpg", True, "detected", 0.9)],
                 {"detector": "x"})
    save_records(b, [FrameScore("w", "frame-0000.jpg", True, "detected", 0.9),
                     FrameScore("w", "frame-0001.jpg", False, "absent", None)],
                 {"detector": "y"})
    assert perception_eval.main(["compare", str(a), str(b)]) == 1
    assert "different frames" in capsys.readouterr().out


def test_f11_a_saved_config_names_its_detector_confidence(tmp_path):
    args = types.SimpleNamespace(
        detector="yolo-world", clip="RN50", floor_mask=False, proposer="none",
        segmenter="sam", crop_path="hard", metric="confidence", max_crops=None,
        imgsz=None, vlm_max_pixels=None, no_vlm_boxes=False, detector_confidence=0.1)
    config = perception_eval._config_of(args)
    assert config["detector_confidence"] == 0.1
    assert config["segmenter"] == "sam"
    assert "0.1" in perception_eval._label_for(config)


# ---------- F12, F13: metrics ----------


def test_f12_a_store_failure_is_an_error_not_no_runs():
    """Red on dev: AccessDenied read as "No runs stored yet"."""
    class Broken(LocalWalkStore):
        def list_names(self, *a, **k):
            raise RuntimeError("AccessDenied")

    app = FastAPI()
    register_metrics_routes(app, Broken(tempfile.mkdtemp()), lambda: None)
    resp = TestClient(app).get("/metrics/summary?days=1")
    assert resp.status_code == 503


def test_f13_a_row_says_when_its_mission_started():
    """Red on dev: status has no started_at, so every row stored null. Read
    off a real runner's status, not a hand-built dict (the fixes' review:
    the first fix read a key the status does not publish)."""
    from control.mission_runner import MissionRunner
    from tests.conftest import fresh_mock_robot

    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=3)
    runner.start()
    runner.tick()
    runner.stop()
    status = runner.status()
    row = row_for(status)
    assert row["started_at"] is not None
    assert row["started_at"] == pytest.approx(
        row["finished_at"] - status["budget"]["elapsed_s"])


# ---------- the fixes' own review (both reviewers, round 2) ----------


def test_f8_the_s3_store_does_not_list_names_it_refuses():
    """The first F8 fix covered only the local store; the Lambda runs on S3."""
    from control.walk_store import S3WalkStore
    from tests.test_walk_store import FakeS3

    s3 = S3WalkStore("test-bucket", "recordings", client=FakeS3())
    s3.write_bytes("w1", "frame-0000.jpg", b"x")
    s3.client.put_object(Bucket="test-bucket", Key="recordings/w1/.DS_Store", Body=b"x")
    s3.client.put_object(Bucket="test-bucket", Key="recordings/My Walk/a.jpg", Body=b"x")
    assert s3.list_walks() == ["w1"]
    assert [n for n, _ in s3.list_files("w1")] == ["frame-0000.jpg"]


def test_f1_a_walk_that_hit_something_stays_in_the_summary(client):
    """A collision found is capped and shown, whatever else failed: dropping
    it let a model that hit something rank best."""
    c, root = client
    d = make_walk(root, "walk-hit", ["FORWARD"] * 3)
    (d / "eval.json").write_text(json.dumps({
        "schema": walk_eval.SCHEMA_VERSION, "score": 40, "basis": "judge+metrics",
        "flags": ["collision", "collisions-unchecked"], "metrics": {"frames": 3},
        "judge": {"judged": 8, "attempted": 8}, "collisions": {"failed": 1}}))
    rows = c.get("/recording/summary").json()["rows"]
    assert len(rows) == 1 and rows[0]["collisions"] == 1


def test_f1_a_judge_short_of_one_frame_is_still_a_judged_score(client, monkeypatch):
    c, root = client
    make_walk(root, "walk-7of8", ["FORWARD"] * 8)
    n = {"judge": 0}

    class OneJudgeFails:
        def converse(self, modelId, messages, inferenceConfig):
            text = messages[0]["content"][1]["text"]
            if "would_collide" in text:
                return reply('{"would_collide": false, "why": "clear"}')
            n["judge"] += 1
            if n["judge"] == 1:
                raise RuntimeError("ThrottlingException")
            return reply('{"sensible": true, "better_action": null, "why": "ok"}')

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: OneJudgeFails())
    body = c.post("/recording/walks/walk-7of8/evaluate").json()
    assert "judge-incomplete" not in body["flags"]
    assert len(c.get("/recording/summary").json()["rows"]) == 1


def test_f1_an_outage_card_stored_before_the_flags_is_left_out(client):
    """Schema 9 is unchanged, so a card written during an outage before this
    fix carries no flag; the summary reads the record itself."""
    c, root = client
    d = make_walk(root, "walk-old", ["FORWARD"] * 3)
    (d / "eval.json").write_text(json.dumps({
        "schema": walk_eval.SCHEMA_VERSION, "score": 80, "basis": "metrics-only",
        "flags": [], "metrics": {"frames": 3},
        "judge": {"judged": 0, "sensible_rate": None, "error": "throttled"},
        "collisions": None}))
    assert c.get("/recording/summary").json()["rows"] == []


def test_f3_the_collision_check_starts_no_call_after_the_deadline():
    calls = {"n": 0}

    class Counting:
        def converse(self, **kw):
            calls["n"] += 1
            return reply('{"would_collide": false}')

    entries = [{"seq": i, "file": f"frame-{i:04d}.jpg", "navigate": {"action": "FORWARD"}}
               for i in range(4)]
    out = walk_eval.check_collisions(Counting(), "m", entries, lambda e: b"jpg",
                                     deadline=time.monotonic() - 1)
    assert calls["n"] == 0
    assert out["failed"] == 4


def test_f3_no_attempt_starts_with_almost_no_time_left(monkeypatch):
    """An attempt with a second left is paid for and then abandoned."""
    import httpx

    sent = {"n": 0}

    class Counting:
        def __init__(self, timeout=None):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            sent["n"] += 1
            return httpx.Response(200, json={"action": "FORWARD"})

    monkeypatch.setattr(httpx, "Client", Counting)
    with pytest.raises(RuntimeError, match="deadline"):
        admin_server.post_navigate("http://vision.invalid", 5.0, b"jpg", "bag", None,
                                   deadline=time.monotonic() + 1)
    assert sent["n"] == 0


def test_f4_a_score_in_flight_across_a_frame_delete_is_marked(client, monkeypatch):
    """delete_frame marks the records already stored; a score computed from
    the manifest before the delete and stored after it was shown as current."""
    c, root = client
    d = make_walk(root, "walk-race", ["FORWARD"] * 3)

    import threading
    once = threading.Lock()

    class DeletesAFrame:
        def converse(self, **kw):
            # Once, through the route, as the console's Delete does.
            if once.acquire(blocking=False):
                assert c.delete("/recording/walks/walk-race/frames/frame-0001.jpg"
                                ).status_code == 200
            return reply('{"sensible": true, "would_collide": false}')

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: DeletesAFrame())
    c.post("/recording/walks/walk-race/evaluate")
    stored = json.loads((d / "eval.json").read_text())
    assert stored["frames_deleted"] == ["frame-0001.jpg"]


# ---------- the second review of the committed fixes ----------


def test_f4_an_unlisted_image_marks_no_score(client):
    """Second review (/code-review): deleting an image the manifest never
    listed -- one no score counted -- marked every score stale."""
    c, root = client
    d = make_walk(root, "walk-u", ["FORWARD"] * 2)
    c.post("/recording/walks/walk-u/evaluate?judge=false")
    (d / "frame-0099.jpg").write_bytes(b"\xff\xd8\xff\xd9")
    resp = c.delete("/recording/walks/walk-u/frames/frame-0099.jpg")
    assert resp.status_code == 200 and resp.json()["marked"] is False, resp.text
    assert "frames_deleted" not in json.loads((d / "eval.json").read_text())


def test_f5_a_store_read_failure_after_the_paid_work_still_stores_it(client, monkeypatch):
    """Second review (/code-review): F4/F5's reads before the write let an S3
    error (a ClientError, not OSError) escape after the paid work; nothing
    was stored and the route answered 500."""
    c, root = client
    d = make_walk(root, "walk-t", ["FORWARD"] * 2)
    real, n = admin_server._walk_entries, {"calls": 0}

    def throttled_second_read(store, walk):
        n["calls"] += 1
        if n["calls"] > 1:
            raise RuntimeError("ThrottlingException")
        return real(store, walk)
    monkeypatch.setattr(admin_server, "_walk_entries", throttled_second_read)
    resp = c.post("/recording/walks/walk-t/evaluate?judge=false")
    assert resp.status_code == 200, resp.text
    assert (d / "eval.json").exists()


def test_f11_a_default_segmenter_keeps_the_old_label():
    """Second review (/code-review): every label gained the default
    segmenter's name, so a default run no longer matched older records."""
    from brain.perceive import DEFAULT_SEGMENTER
    base = {"detector": "yoloe", "proposer": "none"}
    assert perception_eval._label_for({**base, "segmenter": DEFAULT_SEGMENTER}) == \
        perception_eval._label_for(base)
    assert "sam" in perception_eval._label_for({**base, "segmenter": "sam"})


def test_f4_a_retried_delete_still_marks_the_scores(client, monkeypatch):
    """The fixes' second review (both reviewers): the manifest was rewritten,
    then the image delete failed; the retry found the frame unlisted and
    marked nothing, so a score of a deleted frame read as current."""
    c, root = client
    d = make_walk(root, "walk-r", ["FORWARD"] * 3)
    c.post("/recording/walks/walk-r/evaluate?judge=false")
    real = LocalWalkStore.delete_file
    failing = {"once": True}

    def delete_file(self, walk, name):
        if failing.pop("once", False):
            raise RuntimeError("S3: InternalError")
        return real(self, walk, name)
    monkeypatch.setattr(LocalWalkStore, "delete_file", delete_file)
    with pytest.raises(RuntimeError):
        c.delete("/recording/walks/walk-r/frames/frame-0001.jpg")
    resp = c.delete("/recording/walks/walk-r/frames/frame-0001.jpg")
    assert resp.status_code == 200, resp.text
    assert resp.json()["marked"] is True, "the console marks from this"
    assert json.loads((d / "eval.json").read_text())["frames_deleted"] == ["frame-0001.jpg"]
    assert not (d / "frame-0001.jpg").exists()
    assert "frame-0001.jpg" not in (d / "walk.jsonl").read_text()


def test_f11_an_empty_segmenter_is_not_named():
    assert "seg" not in perception_eval._label_for(
        {"detector": "yoloe", "proposer": "none", "segmenter": ""})


def test_f4_a_retry_after_a_failed_mark_marks_every_score(client, monkeypatch):
    """The fixes' sixth review (both reviewers): with the manifest written
    first, a mark that failed could never be retried -- the frame was no
    longer listed -- so a replay of the deleted frame read as current."""
    c, root = client
    d = make_walk(root, "walk-p", ["FORWARD"] * 3)
    c.post("/recording/walks/walk-p/evaluate?judge=false")
    (d / "replay-m.json").write_text(json.dumps({
        "schema": walk_eval.SCHEMA_VERSION, "model_id": "m", "score": 50, "flags": []}))
    real = LocalWalkStore.write_json
    failing = {"once": True}

    def write_json(self, walk, name, record):
        if name.startswith("replay-") and failing.pop("once", False):
            raise RuntimeError("S3: SlowDown")
        return real(self, walk, name, record)
    monkeypatch.setattr(LocalWalkStore, "write_json", write_json)
    with pytest.raises(RuntimeError):
        c.delete("/recording/walks/walk-p/frames/frame-0001.jpg")
    resp = c.delete("/recording/walks/walk-p/frames/frame-0001.jpg")
    assert resp.status_code == 200 and resp.json()["marked"] is True, resp.text
    for name in ("eval.json", "replay-m.json"):
        assert json.loads((d / name).read_text())["frames_deleted"] == ["frame-0001.jpg"], name
