"""
Tests for control/admin_server.py's scorecard routes.

The viewer/deleter half of this service predates these tests and is
exercised by hand; what's covered here is the part with real logic --
scoring a walk, storing the result beside it, and the two rules that keep
the machine's opinion from being mistaken for the operator's:

  * the score is written to its own eval.json sidecar and NEVER to
    tags.json, so a scorer calibrated against six walks can't overwrite a
    human "training-ready" judgement;
  * a walk's model comes from the frames when they carry it and from
    meta.json otherwise, which is the only way the six walks recorded
    before /navigate echoed model_id back can ever be compared fairly.

No AWS: the judge tier is off unless WALK_JUDGE_ENABLED is set, and where
it is exercised the bedrock client factory is monkeypatched.
"""

import json

import pytest
from fastapi.testclient import TestClient

from control import admin_server


@pytest.fixture
def recordings(tmp_path):
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  recording_dir: {tmp_path / 'recordings'}\n")
    (tmp_path / "recordings").mkdir()
    return tmp_path, str(config)


def make_walk(root, name, actions, model=None, obstacle=False):
    walk_dir = root / "recordings" / name
    walk_dir.mkdir(parents=True)
    lines = []
    for seq, action in enumerate(actions):
        (walk_dir / f"frame-{seq:04d}.jpg").write_bytes(b"\xff\xd8\xff\xd9")
        nav = {"action": action, "target_visible": True, "target_direction": "center",
               "target_reached": False, "obstacle_ahead": obstacle, "reasoning": "r"}
        if model:
            nav["model_id"] = model
        lines.append(json.dumps({"seq": seq, "file": f"frame-{seq:04d}.jpg",
                                 "media_type": "image/jpeg", "navigate": nav}))
    (walk_dir / "walk.jsonl").write_text("\n".join(lines) + "\n")
    return walk_dir


@pytest.fixture
def client(recordings):
    root, config = recordings
    return TestClient(admin_server.create_app(config_path=config)), root


# ---------- scoring ----------


def test_evaluating_the_stalled_walk_flags_it_and_persists_the_scorecard(client):
    """The 2026-08-29 ottoman walk's shape: one FORWARD then 21 frames of
    turning. The whole point of the feature is that this is detected from
    the log without anyone reading 22 frames by hand."""
    c, root = client
    make_walk(root, "stalled-walk", ["FORWARD"] + ["RIGHT", "LEFT"] * 10 + ["STOP"])

    resp = c.post("/recording/walks/stalled-walk/evaluate?judge=false")
    assert resp.status_code == 200, resp.text
    body = resp.json()

    assert "stalled" in body["flags"]
    assert body["basis"] == "metrics-only"
    assert body["metrics"]["longest_no_forward_run"] == 21
    # persisted beside the frames, so the console doesn't re-pay for it
    stored = json.loads((root / "recordings" / "stalled-walk" / "eval.json").read_text())
    assert stored["score"] == body["score"]


def test_evaluating_the_degenerate_walk_flags_it(client):
    c, root = client
    make_walk(root, "blind-walk", ["FORWARD"] * 22)
    body = c.post("/recording/walks/blind-walk/evaluate?judge=false").json()
    assert "degenerate" in body["flags"]


def test_the_score_never_touches_the_operators_label(client):
    """The rule that keeps machine and human judgement separable."""
    c, root = client
    walk_dir = make_walk(root, "walk-a", ["FORWARD"] * 10)
    c.put("/recording/walks/walk-a/tag", json={"label": "training-ready"})

    c.post("/recording/walks/walk-a/evaluate?judge=false")

    assert json.loads((walk_dir / "tags.json").read_text())["label"] == "training-ready"
    assert c.get("/recording/walks").json()["walks"][0]["label"] == "training-ready"
    assert "label" not in json.loads((walk_dir / "eval.json").read_text())


def test_evaluation_is_computed_lazily_on_first_read(client):
    """The fallback half of the trigger design -- a walk whose finish signal
    never arrived (closed tab, dead battery) still gets scored."""
    c, root = client
    walk_dir = make_walk(root, "walk-b", ["FORWARD", "LEFT", "FORWARD"])
    assert not (walk_dir / "eval.json").exists()

    body = c.get("/recording/walks/walk-b/evaluation").json()
    assert "score" in body
    assert (walk_dir / "eval.json").exists()


def test_a_stored_evaluation_is_reused_not_recomputed(client, monkeypatch):
    c, root = client
    make_walk(root, "walk-c", ["FORWARD"] * 4)
    first = c.get("/recording/walks/walk-c/evaluation").json()

    def boom(*a, **k):
        raise AssertionError("should have been served from the sidecar")

    monkeypatch.setattr(admin_server.walk_eval, "compute_metrics", boom)
    assert c.get("/recording/walks/walk-c/evaluation").json()["score"] == first["score"]


def test_a_stale_schema_is_rescored_rather_than_served(client):
    c, root = client
    walk_dir = make_walk(root, "walk-d", ["FORWARD"] * 4)
    (walk_dir / "eval.json").write_text(json.dumps({"schema": 0, "score": 999}))
    assert c.get("/recording/walks/walk-d/evaluation").json()["score"] != 999


# ---------- the model a walk was recorded with ----------


def test_the_model_comes_from_the_frames_when_they_carry_it(client):
    c, root = client
    make_walk(root, "walk-e", ["FORWARD"] * 3, model="amazon.nova-lite-v1:0")
    assert c.get("/recording/walks").json()["walks"][0]["model_id"] == "amazon.nova-lite-v1:0"


def test_the_model_can_be_backfilled_for_older_walks(client):
    """Every walk recorded up to 2026-08-29 predates /navigate echoing the
    model back, and was in fact produced by Sonnet 4.5 rather than the model
    the docs named -- so this is the only way their history is comparable."""
    c, root = client
    make_walk(root, "walk-f", ["FORWARD"] * 3)
    assert c.get("/recording/walks").json()["walks"][0]["model_id"] is None

    c.put("/recording/walks/walk-f/meta",
          json={"model_id": "us.anthropic.claude-sonnet-4-5-20250929-v1:0"})

    listed = c.get("/recording/walks").json()["walks"][0]
    assert listed["model_id"] == "us.anthropic.claude-sonnet-4-5-20250929-v1:0"
    assert c.post("/recording/walks/walk-f/evaluate?judge=false").json()["model_id"] == \
        "us.anthropic.claude-sonnet-4-5-20250929-v1:0"


def test_meta_updates_merge_rather_than_replace(client):
    c, root = client
    make_walk(root, "walk-g", ["FORWARD"])
    c.put("/recording/walks/walk-g/meta", json={"model_id": "m-1"})
    c.put("/recording/walks/walk-g/meta", json={"target_object": "red backpack"})
    meta = c.get("/recording/walks/walk-g").json()["meta"]
    assert meta == {"model_id": "m-1", "target_object": "red backpack"}


def test_sidecars_are_not_counted_as_frames(client):
    """eval.json/meta.json/tags.json live in the walk directory; miscounting
    them would inflate every frame total in the console."""
    c, root = client
    make_walk(root, "walk-h", ["FORWARD"] * 3)
    c.put("/recording/walks/walk-h/meta", json={"model_id": "m"})
    c.post("/recording/walks/walk-h/evaluate?judge=false")
    c.put("/recording/walks/walk-h/tag", json={"label": "good"})
    assert c.get("/recording/walks").json()["walks"][0]["frames"] == 3


# ---------- the judge tier ----------


class FakeBedrock:
    def __init__(self):
        self.calls = 0

    def converse(self, modelId, messages, inferenceConfig):
        self.calls += 1
        return {"output": {"message": {"content": [
            {"text": '{"sensible": false, "better_action": "FORWARD", "why": "open floor"}'}]}}}


def test_the_judge_runs_when_enabled_and_lands_in_the_scorecard(client, monkeypatch):
    c, root = client
    make_walk(root, "walk-i", ["STOP"] * 6)
    fake = FakeBedrock()
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: fake)

    body = c.post("/recording/walks/walk-i/evaluate").json()

    assert fake.calls > 0
    assert body["basis"] == "judge+metrics"
    assert body["judge"]["sensible_rate"] == 0.0
    assert body["judge"]["frames"][0]["better_action"] == "FORWARD"


def test_a_judge_outage_degrades_to_metrics_rather_than_failing_the_walk(client, monkeypatch):
    """Bedrock being throttled must not make a walk unviewable."""
    c, root = client
    make_walk(root, "walk-j", ["FORWARD"] * 4)
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)

    def boom():
        raise RuntimeError("bedrock unreachable")

    monkeypatch.setattr(admin_server, "_bedrock_client", boom)
    resp = c.post("/recording/walks/walk-j/evaluate")

    assert resp.status_code == 200
    assert resp.json()["judge"]["sensible_rate"] is None
    assert "score" in resp.json()


def test_the_judge_is_skipped_when_disabled(client, monkeypatch):
    c, root = client
    make_walk(root, "walk-k", ["FORWARD"] * 4)
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", False)
    monkeypatch.setattr(admin_server, "_bedrock_client",
                        lambda: (_ for _ in ()).throw(AssertionError("must not be built")))
    assert c.post("/recording/walks/walk-k/evaluate").json()["basis"] == "metrics-only"


# ---------- path safety, inherited from the existing routes ----------


@pytest.mark.parametrize("route", [
    "/recording/walks/..%2F..%2Fetc/evaluate",
    "/recording/walks/nope/evaluation",
])
def test_bad_or_unknown_walks_are_refused(client, route):
    c, _root = client
    method = c.post if route.endswith("evaluate") else c.get
    assert method(route).status_code in (400, 404)


# ---------- the console's client script ----------
#
# admin.js was extracted out of admin.html. The failure mode that change
# introduces is quiet: the page still renders, the script 404s, and nothing
# on it works -- so the reference and the route are pinned here rather than
# left to be noticed in a browser.


def test_the_console_serves_its_script_and_links_to_it(client):
    c, _root = client

    page = c.get("/admin")
    assert page.status_code == 200
    assert 'src="admin.js"' in page.text
    assert "<script>" not in page.text, "the client should be external, not inlined again"

    script = c.get("/admin.js")
    assert script.status_code == 200
    assert "application/javascript" in script.headers["content-type"]
    # no-cache, so a redeploy can't leave a stale script against fresh HTML
    assert "no-cache" in script.headers.get("cache-control", "")
    assert "scorePendingWalks" in script.text
