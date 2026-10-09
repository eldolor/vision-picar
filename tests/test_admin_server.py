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


def test_a_stale_schema_is_marked_and_not_summed(client):
    """3.64 E2: only the evaluation read rescored a stale scorecard. The walk
    list showed its old score as current, and the per-model summary
    averaged it, replays included."""
    c, root = client
    walk_dir = make_walk(root, "walk-old", ["FORWARD"] * 4, model="m-old")
    stale = {"schema": admin_server.walk_eval.SCHEMA_VERSION - 1, "score": 999,
             "verdict": "good", "flags": [], "metrics": {}}
    (walk_dir / "eval.json").write_text(json.dumps(stale))
    (walk_dir / "replay-m-old.json").write_text(json.dumps(
        {**stale, "model_id": "m-old", "prompt_variant": "default"}))
    (row,) = [w for w in c.get("/recording/walks").json()["walks"] if w["walk"] == "walk-old"]
    # Listed, marked stale -- not auto-re-scored (a schema bump must not
    # start paid judge runs over the corpus on the next page load).
    assert row["eval"]["stale"] is True and row["eval"]["score"] == 999
    assert row["replays"][0]["stale"] is True
    assert c.get("/recording/summary").json()["rows"] == []


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


# ---------- a replay that mostly failed is not a result ----------
#
# Replay is the only controlled comparison this project has, which is
# exactly why a broken one is dangerous rather than merely useless: it
# comes back looking like a measurement. Three prompt variants of one
# 22-frame walk were once all scored 33, on 3, 9 and 2 surviving frames --
# read at the time as "the wording made no difference", and really "the
# vision service timed out". These three tests are that incident.


def timing_out(recordings, monkeypatch, fails, total):
    """A client whose vision service drops `fails` of `total` frames."""
    root, config_path = recordings
    Path(config_path).write_text(
        f"brain:\n  recording_dir: {root / 'recordings'}\n"
        f"  vision_url: http://vision.invalid\n")
    n = {"calls": 0}

    def flaky(vision_url, timeout_s, image_bytes, target_object, model_id,
              prompt_variant=None):
        n["calls"] += 1
        if n["calls"] <= fails:
            raise RuntimeError("ReadTimeout: timed out")
        return nav_reply("FORWARD")

    monkeypatch.setattr(admin_server, "post_navigate", flaky)
    return TestClient(admin_server.create_app(config_path=config_path)), root


def test_a_replay_that_lost_most_of_its_frames_is_not_given_a_score(recordings, monkeypatch):
    """The defect itself: 2 frames of 10 came back and the scorer produced a
    number anyway, indistinguishable in the console from a model that really
    had navigated badly."""
    c, root = timing_out(recordings, monkeypatch, fails=8, total=10)
    make_walk(root, "walk-timeout", ["STOP"] * 10)

    body = c.post("/recording/walks/walk-timeout/replay", json={}).json()

    assert body["errors"] == 8
    assert body["coverage"] == 0.2
    assert body["score"] is None, "a mostly-failed replay must not report a score"
    assert body["verdict"] == "unusable"
    assert body["flags"] == ["incomplete"]


def test_an_unscored_replay_is_kept_out_of_the_per_model_summary(recordings, monkeypatch):
    """A null score is the mechanism -- summary's add() already skips one --
    so this pins the two halves together. Averaging a timeout into a model's
    mean is how a harness failure becomes a model conclusion."""
    c, root = timing_out(recordings, monkeypatch, fails=8, total=10)
    make_walk(root, "walk-timeout-2", ["STOP"] * 10)

    c.post("/recording/walks/walk-timeout-2/replay",
           json={"model_id": "amazon.nova-lite-v1:0"})

    rows = c.get("/recording/summary").json()["rows"]
    assert not [r for r in rows if r["source"] == "replay"], \
        "an unusable replay must not appear as evidence about its model"


def test_a_replay_that_lost_only_a_frame_or_two_is_still_scored(recordings, monkeypatch):
    """The guard has to be a threshold, not a demand for perfection: a walk
    that dropped one frame of twenty is still the comparison the operator
    asked for."""
    c, root = timing_out(recordings, monkeypatch, fails=1, total=20)
    make_walk(root, "walk-nearly", ["STOP"] * 20)

    body = c.post("/recording/walks/walk-nearly/replay", json={}).json()

    assert body["errors"] == 1
    assert body["coverage"] == 0.95
    assert isinstance(body["score"], int)
    assert body["verdict"] != "unusable"


def test_replay_uses_its_own_timeout_not_the_missions(vision_timeouts):
    """A mission is impatient because a robot is standing in a room with its
    motors live. A replay has no robot, and is the burstiest caller here --
    sharing B3.2's 20s budget is what produced the timeouts above."""
    c, root, seen = vision_timeouts
    make_walk(root, "walk-budget", ["STOP"])

    c.post("/recording/walks/walk-budget/replay", json={})

    assert seen == [60.0], "replay should spend replay_timeout_s, not vision_timeout_s"


@pytest.fixture
def vision_timeouts(recordings, monkeypatch):
    """Like `vision`, but records the timeout each call was given."""
    root, config_path = recordings
    Path(config_path).write_text(
        f"brain:\n  recording_dir: {root / 'recordings'}\n"
        f"  vision_url: http://vision.invalid\n")
    seen = []

    def fake_post(vision_url, timeout_s, image_bytes, target_object, model_id,
                  prompt_variant=None):
        seen.append(timeout_s)
        return nav_reply("FORWARD")

    monkeypatch.setattr(admin_server, "post_navigate", fake_post)
    return TestClient(admin_server.create_app(config_path=config_path)), root, seen


# ---------- post_navigate's retry ----------


def test_a_read_timeout_is_retried_rather_than_losing_the_frame(monkeypatch):
    """The bug, exactly. httpx raises a timeout instead of returning a status
    code, so it fell straight out of post_navigate and the backoff written
    for this very burst never ran on the failure that dominates it."""
    import httpx

    attempts = {"n": 0}

    class FakeClient:
        def __init__(self, timeout=None):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            attempts["n"] += 1
            if attempts["n"] < 3:
                raise httpx.ReadTimeout("timed out")
            return httpx.Response(200, json={"action": "FORWARD"})

    monkeypatch.setattr(httpx, "Client", FakeClient)
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None, raising=False)

    out = admin_server.post_navigate("http://vision.invalid", 5.0, b"jpg",
                                     "red backpack", None)

    assert out == {"action": "FORWARD"}
    assert attempts["n"] == 3, "the timeout should have been retried, not raised"


def test_a_frame_that_times_out_every_attempt_reports_the_timeout(monkeypatch):
    """And when the retries genuinely run out, the error that reaches the
    replay's diff has to name the timeout -- diagnosing this incident
    depended on those strings being in the stored record."""
    import httpx

    class AlwaysTimesOut:
        def __init__(self, timeout=None):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            raise httpx.ReadTimeout("timed out")

    monkeypatch.setattr(httpx, "Client", AlwaysTimesOut)
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None, raising=False)

    with pytest.raises(RuntimeError, match="ReadTimeout"):
        admin_server.post_navigate("http://vision.invalid", 5.0, b"jpg",
                                   "red backpack", None)


def test_a_bad_request_is_not_retried(monkeypatch):
    """Unchanged behaviour, pinned because the retry block was restructured
    around it: a 400 is our own malformed call and no amount of backoff
    fixes it."""
    import httpx

    attempts = {"n": 0}

    class Rejects:
        def __init__(self, timeout=None):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            attempts["n"] += 1
            return httpx.Response(400, text="bad model_id")

    monkeypatch.setattr(httpx, "Client", Rejects)
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None, raising=False)

    with pytest.raises(RuntimeError, match="400"):
        admin_server.post_navigate("http://vision.invalid", 5.0, b"jpg",
                                   "red backpack", None)
    assert attempts["n"] == 1


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


# ---------- the viewer/deleter half ----------
#
# These routes predate the scorecard and had no tests at all, which for
# DELETE is a poor place to have none.


def test_a_frame_is_served_with_the_right_content_type(client):
    c, root = client
    make_walk(root, "walk-f1", ["FORWARD"])
    resp = c.get("/recording/walks/walk-f1/frames/frame-0000.jpg")
    assert resp.status_code == 200
    assert resp.headers["content-type"] == "image/jpeg"
    assert resp.content == b"\xff\xd8\xff\xd9"


@pytest.mark.parametrize("bad", ["../../etc/passwd", "notaframe.jpg", "frame-9.jpg"])
def test_a_frame_path_cannot_escape_its_walk(client, bad):
    """The one route that takes a filename from the caller."""
    c, root = client
    make_walk(root, "walk-f2", ["FORWARD"])
    assert c.get(f"/recording/walks/walk-f2/frames/{bad}").status_code in (400, 404)


def test_deleting_one_frame_removes_it_from_the_log_too(client):
    """A frame and its walk.jsonl entry have to go together, or a replay
    reads an entry whose image is gone."""
    c, root = client
    walk_dir = make_walk(root, "walk-d1", ["FORWARD", "LEFT", "STOP"])

    resp = c.delete("/recording/walks/walk-d1/frames/frame-0001.jpg")

    assert resp.status_code == 200
    assert not (walk_dir / "frame-0001.jpg").exists()
    remaining = [json.loads(l) for l in (walk_dir / "walk.jsonl").read_text().splitlines() if l.strip()]
    assert [e["file"] for e in remaining] == ["frame-0000.jpg", "frame-0002.jpg"]
    assert c.get("/recording/walks").json()["walks"][0]["frames"] == 2


def test_deleting_a_walk_removes_the_whole_directory(client):
    c, root = client
    walk_dir = make_walk(root, "walk-d2", ["FORWARD"])
    assert c.delete("/recording/walks/walk-d2").status_code == 200
    assert not walk_dir.exists()
    assert c.get("/recording/walks").json()["walks"] == []


def test_deleting_an_unknown_walk_is_a_404_not_a_wiped_directory(client):
    c, root = client
    make_walk(root, "walk-d3", ["FORWARD"])
    assert c.delete("/recording/walks/nope").status_code == 404
    assert c.delete("/recording/walks/..").status_code in (400, 404)
    # the real walk is untouched
    assert c.get("/recording/walks").json()["walks"][0]["walk"] == "walk-d3"


def test_a_walk_downloads_as_a_zip_of_its_files(client):
    import io
    import zipfile

    c, root = client
    make_walk(root, "walk-z", ["FORWARD", "LEFT"])
    c.put("/recording/walks/walk-z/tag", json={"label": "good"})

    resp = c.get("/recording/walks/walk-z/download")

    assert resp.status_code == 200
    assert resp.headers["content-type"] == "application/zip"
    names = set(zipfile.ZipFile(io.BytesIO(resp.content)).namelist())
    assert {"frame-0000.jpg", "frame-0001.jpg", "walk.jsonl", "tags.json"} <= names


def test_stats_totals_frames_and_bytes_across_walks(client):
    c, root = client
    make_walk(root, "walk-s1", ["FORWARD", "LEFT"])
    make_walk(root, "walk-s2", ["FORWARD"])
    body = c.get("/stats").json()
    assert body["walks"] == 2
    assert body["frames"] == 3
    assert body["bytes"] > 0


def test_stats_and_the_listing_survive_a_missing_recordings_dir(tmp_path):
    """A brand-new deployment, before any walk has been recorded."""
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  recording_dir: {tmp_path / 'never-created'}\n")
    c = TestClient(admin_server.create_app(config_path=str(config)))
    assert c.get("/stats").json() == {"walks": 0, "frames": 0, "bytes": 0}
    assert c.get("/recording/walks").json()["walks"] == []
    assert c.get("/recording/summary").json()["rows"] == []


def test_health_reports_where_the_recordings_live(client):
    c, _root = client
    body = c.get("/health").json()
    assert body["status"] == "ok"
    assert body["recording_dir_exists"] is True


def test_the_console_page_and_its_script_are_served(client):
    c, _root = client
    assert 'src="admin.js"' in c.get("/admin").text
    assert c.get("/admin.js").status_code == 200


def test_a_corrupt_sidecar_is_ignored_rather_than_fatal(client):
    """Sidecars are plain files on a shared volume; a truncated write must
    not take out the listing for every walk."""
    c, root = client
    walk_dir = make_walk(root, "walk-c1", ["FORWARD"])
    (walk_dir / "tags.json").write_text("{not json")
    (walk_dir / "eval.json").write_text("{also not json")
    (walk_dir / "meta.json").write_text("{nor this")
    (walk_dir / "replay-broken.json").write_text("{still not")

    listed = c.get("/recording/walks").json()["walks"][0]
    assert listed["label"] is None and listed["eval"] is None
    assert listed["replays"] == []
    assert c.get("/recording/walks/walk-c1/replays").json()["replays"] == []
    assert c.get("/recording/summary").json()["rows"] == []


def test_the_model_relay_says_so_when_no_vision_service_is_configured(client):
    c, _root = client
    body = c.get("/recording/models").json()
    assert body["models"] == []
    assert "vision" in body["detail"].lower()


def test_the_model_relay_degrades_to_an_empty_list_when_the_service_is_down(vision, monkeypatch):
    """An empty picker beats a broken page."""
    c, _root, _calls = vision

    class Boom:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def get(self, url, headers=None):
            raise RuntimeError("vision service unreachable")

    # httpx is imported inside the function, so patch the library itself.
    import httpx

    monkeypatch.setattr(httpx, "Client", lambda **kw: Boom())
    body = c.get("/recording/models").json()
    assert body["models"] == []


def test_an_unlabelled_walk_can_have_its_label_cleared(client):
    c, root = client
    walk_dir = make_walk(root, "walk-t1", ["FORWARD"])
    c.put("/recording/walks/walk-t1/tag", json={"label": "bad"})
    assert (walk_dir / "tags.json").exists()

    assert c.put("/recording/walks/walk-t1/tag", json={"label": None}).json()["label"] is None
    assert not (walk_dir / "tags.json").exists()


def test_an_unknown_label_is_refused(client):
    c, root = client
    make_walk(root, "walk-t2", ["FORWARD"])
    assert c.put("/recording/walks/walk-t2/tag", json={"label": "excellent"}).status_code == 400


# ---------- the outbound /navigate call replay makes ----------


class FakeResp:
    def __init__(self, status, payload=None, text=""):
        self.status_code = status
        self._payload = payload or {}
        self.text = text

    def json(self):
        return self._payload


def fake_httpx(responses, sent=None):
    """An httpx.Client stand-in returning `responses` in order."""
    seq = list(responses)

    class Client:
        def __init__(self, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            if sent is not None:
                sent.append({"url": url, "json": json, "headers": headers})
            return seq.pop(0)

    return Client


def test_post_navigate_sends_the_shape_the_service_expects(monkeypatch):
    import httpx

    sent = []
    monkeypatch.setattr(httpx, "Client", fake_httpx([FakeResp(200, {"action": "FORWARD"})], sent))
    monkeypatch.setenv("VISION_SHARED_SECRET", "vision-secret")

    out = admin_server.post_navigate("http://vision.test/", 10.0, b"\xff\xd8",
                                     "red backpack", "m-1", "next-step-and-walls")

    assert out == {"action": "FORWARD"}
    body = sent[0]["json"]
    assert sent[0]["url"] == "http://vision.test/navigate"
    assert body["target_object"] == "red backpack"
    assert body["model_id"] == "m-1"
    assert body["prompt_variant"] == "next-step-and-walls"
    assert body["image_base64"]  # base64 of the bytes, not the bytes
    # The VISION service's secret, not this service's own.
    assert sent[0]["headers"]["x-app-secret"] == "vision-secret"


def test_post_navigate_omits_model_and_prompt_when_not_chosen(monkeypatch):
    import httpx

    sent = []
    monkeypatch.setattr(httpx, "Client", fake_httpx([FakeResp(200, {})], sent))
    admin_server.post_navigate("http://v", 5.0, b"x", "target", None, None)
    assert "model_id" not in sent[0]["json"]
    assert "prompt_variant" not in sent[0]["json"]


def test_post_navigate_retries_a_throttle_and_then_succeeds(monkeypatch):
    """A replay fires every frame of a walk at the vision service at once,
    which is burstier than anything else here produces, and Bedrock throttles
    it -- raising the worker count once turned 0 errors into 10 of 22."""
    import httpx

    monkeypatch.setattr(httpx, "Client", fake_httpx([
        FakeResp(429, text="slow down"),
        FakeResp(503, text="unavailable"),
        FakeResp(200, {"action": "LEFT"}),
    ]))
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None)

    assert admin_server.post_navigate("http://v", 5.0, b"x", "t", None)["action"] == "LEFT"


def test_post_navigate_does_not_retry_its_own_bad_request(monkeypatch):
    """A 400 is this caller's mistake -- an unknown model, say. Retrying it
    three times just makes the same error slowly."""
    import httpx

    calls = {"n": 0}

    class Counting:
        def __init__(self, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def post(self, url, json=None, headers=None):
            calls["n"] += 1
            return FakeResp(400, text="unknown model_id")

    monkeypatch.setattr(httpx, "Client", Counting)
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None)

    with pytest.raises(RuntimeError) as e:
        admin_server.post_navigate("http://v", 5.0, b"x", "t", "made-up")

    assert calls["n"] == 1
    assert "400" in str(e.value)


def test_post_navigate_gives_up_after_retrying(monkeypatch):
    import httpx

    monkeypatch.setattr(httpx, "Client", fake_httpx([FakeResp(500, text="boom")] * 6))
    monkeypatch.setattr(admin_server.time, "sleep", lambda s: None)
    with pytest.raises(RuntimeError) as e:
        admin_server.post_navigate("http://v", 5.0, b"x", "t", None)
    assert "500" in str(e.value)


def test_the_bedrock_client_is_built_with_retries(monkeypatch):
    """Built per call site rather than at import, so this module stays
    importable (and testable) without boto3 configured."""
    import boto3

    captured = {}

    def fake_client(name, config=None, **kw):
        captured["name"] = name
        captured["attempts"] = config.retries["max_attempts"]
        return "client"

    monkeypatch.setattr(boto3, "client", fake_client)
    assert admin_server._bedrock_client() == "client"
    assert captured["name"] == "bedrock-runtime"
    assert captured["attempts"] >= 1


# ---------- remaining guards and degradation paths ----------


def test_the_secret_gate_refuses_a_wrong_one(recordings, monkeypatch):
    """Reviewing and deleting recordings is a different privilege from
    starting a mission, so this service has its own secret."""
    root, config_path = recordings
    monkeypatch.setenv("APP_SHARED_SECRET", "right")
    c = TestClient(admin_server.create_app(config_path=config_path))
    make_walk(root, "walk-g1", ["FORWARD"])

    assert c.get("/recording/walks").status_code == 401
    assert c.get("/recording/walks", headers={"x-app-secret": "wrong"}).status_code == 401
    assert c.get("/recording/walks", headers={"x-app-secret": "right"}).status_code == 200
    # /health and the page itself stay open -- the ALB check cannot send
    # headers, and a user must load the page before entering a secret.
    assert c.get("/health").status_code == 200
    assert c.get("/admin").status_code == 200


@pytest.mark.parametrize("route", [
    "/recording/walks/../secrets",
    "/recording/walks/bad%20name/replays",
])
def test_a_bad_walk_name_is_refused_everywhere_it_is_accepted(client, route):
    c, _root = client
    assert c.get(route).status_code in (400, 404)


def test_a_walk_with_no_jsonl_scores_as_empty(client):
    """Frames on disk but no log -- a recording interrupted before its first
    /navigate reply came back."""
    c, root = client
    walk_dir = root / "recordings" / "walk-nolog"
    walk_dir.mkdir(parents=True)
    (walk_dir / "frame-0000.jpg").write_bytes(b"\xff\xd8\xff\xd9")

    body = c.post("/recording/walks/walk-nolog/evaluate?judge=false").json()
    assert body["metrics"]["empty"] is True
    assert body["score"] == 0


def test_a_malformed_jsonl_line_is_skipped_not_fatal(client):
    c, root = client
    walk_dir = make_walk(root, "walk-badline", ["FORWARD", "LEFT"])
    with (walk_dir / "walk.jsonl").open("a") as f:
        f.write("{ this is not json\n")

    assert c.post("/recording/walks/walk-badline/evaluate?judge=false").json()["metrics"]["frames"] == 2


def test_the_target_comes_from_meta_when_it_is_recorded(client):
    """meta.json wins over parsing the name, which is only a fallback."""
    c, root = client
    make_walk(root, "some-walk-name", ["FORWARD"])
    c.put("/recording/walks/some-walk-name/meta", json={"target_object": "green mug"})
    assert c.post("/recording/walks/some-walk-name/evaluate?judge=false").json()["target_object"] \
        == "green mug"


def test_a_walk_with_an_unparseable_timestamp_falls_back_to_meta(client):
    c, root = client
    make_walk(root, "walk-notime", ["FORWARD"])
    listed = c.get("/recording/walks").json()["walks"][0]
    assert listed["recorded_at"] is None

    c.post("/recording/walks/walk-notime/evaluate?judge=false")
    # finished_at only lands via the brain's finish route; meta set by hand
    # is the other way a walk gets one.
    c.put("/recording/walks/walk-notime/meta", json={"model_id": "m"})
    assert c.get("/recording/walks").json()["walks"][0]["model_id"] == "m"


def test_a_replay_whose_judge_is_unavailable_still_scores(vision, monkeypatch):
    """Bedrock being down costs the collision check, not the replay."""
    c, root, _calls = vision
    make_walk(root, "walk-nj", ["FORWARD"] * 3)
    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client",
                        lambda: (_ for _ in ()).throw(RuntimeError("bedrock down")))

    body = c.post("/recording/walks/walk-nj/replay", json={}).json()
    assert body["collisions"] is None
    assert "score" in body


def test_the_summary_counts_a_walk_that_reached_its_target(vision):
    c, root, _calls = vision
    walk_dir = make_walk(root, "red-thing-20260830-090000", ["FORWARD"] * 3)
    entries = [json.loads(l) for l in (walk_dir / "walk.jsonl").read_text().splitlines() if l.strip()]
    entries[-1]["navigate"]["target_reached"] = True
    (walk_dir / "walk.jsonl").write_text("".join(json.dumps(e) + "\n" for e in entries))

    c.post("/recording/walks/red-thing-20260830-090000/evaluate?judge=false")
    row = [r for r in c.get("/recording/summary").json()["rows"] if r["source"] == "recorded"][0]
    assert row["reached"] == 1 and row["reach_rate"] == 1.0


def test_a_scorecard_that_cannot_be_written_is_still_returned(client, monkeypatch):
    """A read-only or full volume must not turn a computed score into a 500 --
    the caller gets the result, the persistence is best-effort."""
    c, root = client
    make_walk(root, "walk-ro", ["FORWARD"] * 3)

    # write_bytes, not write_text: every write goes through
    # control/walk_store.py's LocalWalkStore now, and it funnels text
    # through write_bytes so the two backends cannot drift on encoding.
    # Still patching the real filesystem, which is the point -- the
    # scenario is a read-only volume, not a misbehaving store.
    real = Path.write_bytes

    def refuse(self, *a, **kw):
        if self.name in ("eval.json", "replay-m.json"):
            raise OSError("read-only file system")
        return real(self, *a, **kw)

    monkeypatch.setattr(Path, "write_bytes", refuse)
    body = c.post("/recording/walks/walk-ro/evaluate?judge=false").json()
    assert "score" in body
    assert not (root / "recordings" / "walk-ro" / "eval.json").exists()


def test_a_replay_that_cannot_be_written_is_still_returned(vision, monkeypatch):
    c, root, _calls = vision
    make_walk(root, "walk-ro2", ["FORWARD"])

    # See the sibling test above for why this is write_bytes.
    real = Path.write_bytes

    def refuse(self, *a, **kw):
        if self.name.startswith("replay-"):
            raise OSError("no space left on device")
        return real(self, *a, **kw)

    monkeypatch.setattr(Path, "write_bytes", refuse)
    assert "score" in c.post("/recording/walks/walk-ro2/replay", json={}).json()
    # Asserted, not assumed: without this the test passes whether or not
    # the refusal ever fired, which is how it kept passing after the write
    # seam moved into control/walk_store.py.
    assert not list((root / "recordings" / "walk-ro2").glob("replay-*.json"))


def test_a_missing_frame_file_is_a_404(client):
    c, root = client
    make_walk(root, "walk-mf", ["FORWARD"])
    assert c.get("/recording/walks/walk-mf/frames/frame-0007.jpg").status_code == 404


def test_a_name_with_an_impossible_date_has_no_recorded_at(client):
    """The regex matches the SHAPE of a timestamp; strptime is what rejects
    the 30th of February."""
    c, root = client
    make_walk(root, "thing-20260230-999999", ["FORWARD"])
    assert c.get("/recording/walks").json()["walks"][0]["recorded_at"] is None


def test_the_model_relay_presents_the_vision_services_secret(vision, monkeypatch):
    c, _root, _calls = vision
    monkeypatch.setenv("VISION_SHARED_SECRET", "vision-secret")
    seen = {}

    class Client:
        def __init__(self, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def get(self, url, headers=None):
            seen["headers"] = headers

            class R:
                status_code = 200

                @staticmethod
                def raise_for_status():
                    return None

                @staticmethod
                def json():
                    return {"models": [{"id": "m"}], "default": "m"}

            return R()

    import httpx

    monkeypatch.setattr(httpx, "Client", Client)
    assert c.get("/recording/models").json()["models"] == [{"id": "m"}]
    assert seen["headers"]["x-app-secret"] == "vision-secret"


def test_stats_skips_files_sitting_beside_the_walk_directories(client):
    c, root = client
    make_walk(root, "walk-st", ["FORWARD"])
    (root / "recordings" / "stray.txt").write_text("not a walk")
    assert c.get("/stats").json()["walks"] == 1


def test_a_log_entry_naming_a_file_that_is_not_a_frame_is_skipped(client):
    """walk.jsonl is appended by the brain and read here; a line naming
    something outside the frame-NNNN convention must not be turned into a
    path this service then reads."""
    c, root = client
    walk_dir = make_walk(root, "walk-oddfile", ["FORWARD"])
    with (walk_dir / "walk.jsonl").open("a") as f:
        f.write(json.dumps({"seq": 1, "file": "../../etc/passwd",
                            "navigate": {"action": "FORWARD"}}) + "\n")

    body = c.post("/recording/walks/walk-oddfile/evaluate?judge=false").json()
    assert body["metrics"]["frames"] == 2  # both entries counted from the log
    assert "score" in body


def test_a_log_entry_naming_a_non_frame_file_yields_no_bytes(client, monkeypatch):
    """frame_bytes_for() is what stands between a walk.jsonl line and a file
    read. An entry naming something outside the frame-NNNN convention must
    produce no bytes rather than a path traversal -- so the judge and the
    collision check skip it."""
    c, root = client
    walk_dir = make_walk(root, "walk-badentry", ["FORWARD"])
    with (walk_dir / "walk.jsonl").open("a") as f:
        f.write(json.dumps({"seq": 1, "file": "../../../etc/passwd",
                            "navigate": {"action": "FORWARD"}}) + "\n")

    judged = []

    class Recorder:
        def converse(self, modelId, messages, inferenceConfig):
            judged.append(1)
            return {"output": {"message": {"content": [
                {"text": '{"sensible": true, "better_action": null, "why": "ok"}'}]}}}

    monkeypatch.setattr(admin_server, "JUDGE_ENABLED", True)
    monkeypatch.setattr(admin_server, "_bedrock_client", lambda: Recorder())

    body = c.post("/recording/walks/walk-badentry/evaluate").json()
    # Two entries in the log, but only the real frame is ever read.
    assert body["metrics"]["frames"] == 2
    assert body["judge"]["judged"] == 1


def test_a_replay_skips_a_log_entry_with_no_readable_frame(vision):
    c, root, calls = vision
    walk_dir = make_walk(root, "walk-badentry2", ["FORWARD"])
    with (walk_dir / "walk.jsonl").open("a") as f:
        f.write(json.dumps({"seq": 1, "file": "not-a-frame.txt",
                            "navigate": {"action": "FORWARD"}}) + "\n")

    body = c.post("/recording/walks/walk-badentry2/replay", json={}).json()
    assert body["errors"] == 1
    assert len(calls) == 1


def test_a_day_of_metrics_is_neither_listed_nor_deletable_as_a_walk(client):
    """Handoff 4c. Metrics rows live beside the walks as `metrics-<day>`; the
    console listed each day as an empty walk, and its Delete removed a whole
    day of mission rows."""
    c, root = client
    day = root / "recordings" / "metrics-2026-10-03"
    day.mkdir()
    (day / "run-1.json").write_text('{"run_id": "run-1"}')
    make_walk(root, "walk-m1", ["FORWARD"])
    names = [w["walk"] for w in c.get("/recording/walks").json()["walks"]]
    assert names == ["walk-m1"], names
    assert "metrics-2026-10-03" not in names
    assert c.delete("/recording/walks/metrics-2026-10-03").status_code == 404
    assert (day / "run-1.json").exists(), "a day of mission metrics was deleted"


def test_an_unusable_replay_never_replaces_a_scored_one(recordings, monkeypatch):
    """3.64 C3: one file per (model, prompt), and an unusable replay (score
    None) was written over a scored one -- a throttled re-run erased the
    comparison it was meant to add to. The scored record stays, and says
    when a later attempt came back incomplete, so the console can tell."""
    c, root = timing_out(recordings, monkeypatch, fails=0, total=10)
    make_walk(root, "walk-keep", ["STOP"] * 10)
    first = c.post("/recording/walks/walk-keep/replay", json={}).json()
    assert isinstance(first["score"], int)

    def down(*a, **kw):
        raise RuntimeError("ThrottlingException")
    monkeypatch.setattr(admin_server, "post_navigate", down)
    second = c.post("/recording/walks/walk-keep/replay", json={}).json()

    (stored,) = c.get("/recording/walks/walk-keep/replays").json()["replays"]
    assert stored["score"] == first["score"]
    assert stored["replayed_at"] == first["replayed_at"]
    assert stored["last_unusable"]["coverage"] == 0.0
    assert second["score"] == first["score"] and second["last_unusable"]


def test_an_unusable_replay_is_still_stored_when_there_is_nothing_to_keep(recordings, monkeypatch):
    c, root = timing_out(recordings, monkeypatch, fails=8, total=10)
    make_walk(root, "walk-first-bad", ["STOP"] * 10)
    c.post("/recording/walks/walk-first-bad/replay", json={})
    (stored,) = c.get("/recording/walks/walk-first-bad/replays").json()["replays"]
    assert stored["verdict"] == "unusable" and stored["score"] is None


def test_an_older_scorers_replay_is_kept_and_marked_stale(recordings, monkeypatch):
    """3.64, found by /code-review on C3: the kept record could be an older
    scorer's, sent back as a current, comparable score. Kept (its paid
    per-frame answers can be re-scored), but marked stale on the way out."""
    c, root = timing_out(recordings, monkeypatch, fails=8, total=10)
    walk_dir = make_walk(root, "walk-old-replay", ["STOP"] * 10)
    (walk_dir / "replay-_service_default_.json").write_text(json.dumps(
        {"schema": admin_server.walk_eval.SCHEMA_VERSION - 1, "score": 71,
         "model_id": "(service default)", "prompt_variant": "default", "replayed_at": 1.0}))
    body = c.post("/recording/walks/walk-old-replay/replay", json={}).json()
    assert body["score"] == 71 and body["stale"] is True and body["last_unusable"]
    (listed,) = c.get("/recording/walks/walk-old-replay/replays").json()["replays"]
    assert listed["stale"] is True
    stored = json.loads((walk_dir / "replay-_service_default_.json").read_text())
    assert "stale" not in stored
