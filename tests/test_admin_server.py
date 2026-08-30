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
from pathlib import Path

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


def test_walks_carry_a_recording_time_for_sorting(client):
    """The console's "Newest first" sorted on the walk NAME, which meant
    time order only while every name was "<target>-<timestamp>". Adding the
    model tag to the name silently turned it into "group by model", and a
    run of walks on one model buried the walks recorded either side of it on
    another -- which reads as the walks having gone missing."""
    c, root = client
    make_walk(root, "red-backpack-opus-4-5-20260830-104413", ["FORWARD"])
    make_walk(root, "red-backpack-qwen3-vl-235b-a22b-20260830-103934", ["FORWARD"])

    walks = {w["walk"]: w for w in c.get("/recording/walks").json()["walks"]}
    opus = walks["red-backpack-opus-4-5-20260830-104413"]["recorded_at"]
    qwen = walks["red-backpack-qwen3-vl-235b-a22b-20260830-103934"]["recorded_at"]

    assert opus is not None and qwen is not None
    # The opus walk was recorded LATER, though its name sorts earlier.
    assert opus > qwen
    assert "red-backpack-opus-4-5-20260830-104413" < "red-backpack-qwen3-vl-235b-a22b-20260830-103934"


def test_a_walk_with_no_timestamp_in_its_name_still_lists(client):
    """Old or hand-made directory names must not break the listing."""
    c, root = client
    make_walk(root, "some-old-walk", ["FORWARD"])
    assert c.get("/recording/walks").json()["walks"][0]["recorded_at"] is None


# ---------- replay: the same pixels, a different model ----------
#
# The only controlled model comparison this project has. Comparing two live
# walks instead mixes model quality with where the operator pointed the
# phone -- every model conclusion drawn that way has had to be retracted at
# least once.


def nav_reply(action, **over):
    d = {"action": action, "target_visible": True, "target_direction": "center",
         "target_reached": False, "obstacle_ahead": False, "reasoning": "r"}
    d.update(over)
    return d


@pytest.fixture
def vision(recordings, monkeypatch):
    """A client whose config names a vision service, with the HTTP call to it
    replaced. Returns (client, root, calls)."""
    root, config_path = recordings
    Path(config_path).write_text(
        f"brain:\n  recording_dir: {root / 'recordings'}\n"
        f"  vision_url: http://vision.invalid\n")
    calls = []

    def fake_post(vision_url, timeout_s, image_bytes, target_object, model_id,
                  prompt_variant=None):
        calls.append({"target": target_object, "model_id": model_id,
                      "prompt_variant": prompt_variant})
        return nav_reply("FORWARD")

    monkeypatch.setattr(admin_server, "post_navigate", fake_post)
    return TestClient(admin_server.create_app(config_path=config_path)), root, calls


def test_a_replay_re_asks_every_frame_under_the_chosen_model(vision):
    c, root, calls = vision
    make_walk(root, "walk-r", ["STOP", "STOP", "STOP"])

    body = c.post("/recording/walks/walk-r/replay",
                  json={"model_id": "qwen.qwen3-vl-235b-a22b"}).json()

    assert body["frames"] == 3
    assert body["metrics"]["forward_rate"] == 1.0
    assert len(calls) == 3
    assert {x["model_id"] for x in calls} == {"qwen.qwen3-vl-235b-a22b"}
    # The target comes from the walk, not from the caller.
    assert {x["target"] for x in calls} == {"walk r"}


def test_replay_scores_with_the_same_scorer_as_the_original(vision):
    """A replay and a recording must be comparable numbers, which means the
    replay is shaped as walk.jsonl entries and handed to the same scorer --
    nothing about replays is special-cased in walk_eval."""
    c, root, _calls = vision
    make_walk(root, "walk-s", ["STOP"] * 12)

    replay = c.post("/recording/walks/walk-s/replay", json={}).json()
    original = c.post("/recording/walks/walk-s/evaluate?judge=false").json()

    assert set(replay["metrics"]) == set(original["metrics"])
    assert "score" in replay and "verdict" in replay
    # The recording never moved; the replay moves every frame.
    assert replay["metrics"]["forward_rate"] > original["metrics"]["forward_rate"]


def test_replay_reports_how_much_it_disagreed(vision):
    c, root, _calls = vision
    make_walk(root, "walk-t", ["FORWARD", "FORWARD", "LEFT", "LEFT"])

    body = c.post("/recording/walks/walk-t/replay", json={}).json()

    assert body["agreement"] == 0.5
    changed = {d["seq"] for d in body["diff"]}
    assert changed == {2, 3}
    assert all(d["original"] == "LEFT" and d["replayed"] == "FORWARD" for d in body["diff"])


def test_one_failing_frame_does_not_lose_the_replay(recordings, monkeypatch):
    root, config_path = recordings
    Path(config_path).write_text(
        f"brain:\n  recording_dir: {root / 'recordings'}\n"
        f"  vision_url: http://vision.invalid\n")
    n = {"calls": 0}

    def flaky(vision_url, timeout_s, image_bytes, target_object, model_id,
              prompt_variant=None):
        n["calls"] += 1
        if n["calls"] == 2:
            raise RuntimeError("HTTP 502: vision service down")
        return nav_reply("FORWARD")

    monkeypatch.setattr(admin_server, "post_navigate", flaky)
    c = TestClient(admin_server.create_app(config_path=config_path))
    make_walk(root, "walk-u", ["STOP"] * 4)

    body = c.post("/recording/walks/walk-u/replay", json={}).json()
    assert body["errors"] == 1
    assert body["metrics"]["frames"] == 3


def test_replays_accumulate_per_model_rather_than_overwriting(vision):
    """A walk should build up a comparison table across models, not replace
    one result with the next."""
    c, root, _calls = vision
    make_walk(root, "walk-v", ["STOP", "STOP"])

    c.post("/recording/walks/walk-v/replay", json={"model_id": "amazon.nova-lite-v1:0"})
    c.post("/recording/walks/walk-v/replay", json={"model_id": "qwen.qwen3-vl-235b-a22b"})

    stored = c.get("/recording/walks/walk-v/replays").json()["replays"]
    assert {r["model_id"] for r in stored} == {"amazon.nova-lite-v1:0", "qwen.qwen3-vl-235b-a22b"}
    # and the sidecars must not be counted as frames
    assert c.get("/recording/walks").json()["walks"][0]["frames"] == 2


def test_replay_without_a_vision_service_is_a_clear_503(client):
    """The default test config names no vision service, which is the same
    state a misconfigured deployment is in -- it must say so once, rather
    than 500 or fail once per frame."""
    c, root = client
    make_walk(root, "walk-w", ["STOP"])
    resp = c.post("/recording/walks/walk-w/replay", json={})
    assert resp.status_code == 503
    assert "vision" in resp.json()["detail"].lower()


def test_a_replay_can_vary_the_prompt_instead_of_the_model(vision):
    """The other axis, and the more interesting one: the failure that started
    this project was a wording problem, not a model problem. Replaying the
    same frames under a different prompt is the experiment that could never
    be run before -- it needed a redeploy."""
    c, root, calls = vision
    make_walk(root, "walk-p", ["STOP", "STOP"])

    c.post("/recording/walks/walk-p/replay",
           json={"prompt_variant": "next-step-obstacle"})

    assert {x["prompt_variant"] for x in calls} == {"next-step-obstacle"}


def test_model_and_prompt_replays_are_stored_separately(vision):
    """A walk holds a grid across both axes, not one result per model: the
    same model under two prompts is two different experiments."""
    c, root, _calls = vision
    make_walk(root, "walk-q", ["STOP"])

    c.post("/recording/walks/walk-q/replay", json={"model_id": "amazon.nova-lite-v1:0"})
    c.post("/recording/walks/walk-q/replay",
           json={"model_id": "amazon.nova-lite-v1:0", "prompt_variant": "next-step-obstacle"})

    stored = c.get("/recording/walks/walk-q/replays").json()["replays"]
    assert len(stored) == 2
    assert {r["prompt_variant"] for r in stored} == {"default", "next-step-obstacle"}
    assert {r["model_id"] for r in stored} == {"amazon.nova-lite-v1:0"}


# ---------- the per-model summary ----------
#
# The aggregation that was being done by hand after every batch of walks --
# differently each time, and wrong twice.


def test_the_summary_groups_scored_walks_by_model(client):
    c, root = client
    for name, actions in (("red-backpack-a-20260830-100000", ["FORWARD"] * 6),
                          ("red-backpack-b-20260830-100100", ["STOP"] * 6)):
        make_walk(root, name, actions, model="m-1")
        c.post(f"/recording/walks/{name}/evaluate?judge=false")

    rows = c.get("/recording/summary").json()["rows"]
    recorded = [r for r in rows if r["source"] == "recorded"]
    assert len(recorded) == 1
    assert recorded[0]["model_id"] == "m-1"
    assert recorded[0]["walks"] == 2
    assert recorded[0]["best"] >= recorded[0]["worst"]


def test_recordings_and_replays_are_counted_separately(vision):
    """A replay is stronger evidence than a recording -- it holds the pixels
    fixed -- so the two must not be averaged into one number."""
    c, root, _calls = vision
    make_walk(root, "red-backpack-c-20260830-100200", ["STOP"] * 4, model="m-1")
    c.post("/recording/walks/red-backpack-c-20260830-100200/evaluate?judge=false")
    c.post("/recording/walks/red-backpack-c-20260830-100200/replay",
           json={"model_id": "m-2"})

    rows = c.get("/recording/summary").json()["rows"]
    assert {(r["model_id"], r["source"]) for r in rows} == {("m-1", "recorded"), ("m-2", "replay")}


def test_a_model_that_hit_something_is_never_ranked_best(client):
    """Ordering has to reflect that a collision is disqualifying, not just a
    lower score."""
    c, root = client
    make_walk(root, "red-backpack-d-20260830-100300", ["FORWARD"] * 4, model="safe")
    c.post("/recording/walks/red-backpack-d-20260830-100300/evaluate?judge=false")
    walk_dir = make_walk(root, "red-backpack-e-20260830-100400", ["FORWARD"] * 4, model="hitter")
    c.post("/recording/walks/red-backpack-e-20260830-100400/evaluate?judge=false")
    # Force a collision onto the second walk's stored scorecard.
    stored = json.loads((walk_dir / "eval.json").read_text())
    stored["flags"] = stored.get("flags", []) + ["collision"]
    stored["score"] = 99
    (walk_dir / "eval.json").write_text(json.dumps(stored))

    rows = [r for r in c.get("/recording/summary").json()["rows"] if r["source"] == "recorded"]
    assert rows[-1]["model_id"] == "hitter", [r["model_id"] for r in rows]
    assert rows[-1]["collisions"] == 1


def test_an_empty_recordings_dir_summarises_to_nothing(client):
    c, _root = client
    assert c.get("/recording/summary").json()["rows"] == []


def test_a_replay_is_checked_for_collisions_like_a_recording(vision, monkeypatch):
    """Without this a replay cannot answer the question the feature exists
    for -- would this model, or this wording, have driven into that wall --
    and would report a clean score for a run that hit something, which is
    worse than reporting nothing."""
    c, root, _calls = vision
    make_walk(root, "walk-x", ["FORWARD"] * 4)

    class Hitter:
        def converse(self, modelId, messages, inferenceConfig):
            return {"output": {"message": {"content": [
                {"text": '{"would_collide": true, "why": "wall fills the frame"}'}]}}}

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: Hitter())

    body = c.post("/recording/walks/walk-x/replay", json={}).json()

    assert body["collisions"]["collisions"], "the replay found no collision"
    assert "collision" in body["flags"]
    assert body["score"] <= 40


def test_the_target_is_recovered_from_a_name_carrying_a_prompt_tag(client):
    """Walk names now carry the model AND the prompt, and the target is
    parsed back out of the name for the judge's and collision check's
    prompts. Miss the prompt tag and the target becomes "blue bottle next
    step and walls", which then goes to the model."""
    c, root = client
    make_walk(root, "blue-bottle-opus-4-5-next-step-and-walls-20260830-171042", ["FORWARD"])
    body = c.post(
        "/recording/walks/blue-bottle-opus-4-5-next-step-and-walls-20260830-171042"
        "/evaluate?judge=false").json()
    assert body["target_object"] == "blue bottle"

    make_walk(root, "red-backpack-qwen3-vl-235b-a22b-20260830-104120", ["FORWARD"])
    body = c.post("/recording/walks/red-backpack-qwen3-vl-235b-a22b-20260830-104120"
                  "/evaluate?judge=false").json()
    assert body["target_object"] == "red backpack"
