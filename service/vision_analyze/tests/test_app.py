"""
Test suite for service/vision_analyze/app.py -- the real gap named in
CLAUDE.md section 5 item 1 and section 6: this service has run for a long
time verified only by manual `docker run` + curl and against the live
deployment, never by an automated suite the way lambda/vision_analyze/'s
test_handler.py covered the Lambda predecessor.

No AWS calls anywhere here. Every vision_core.describe_image_bytes*()
function is monkeypatched at the app module's own namespace (the name
app.py imported via `from vision_core import ...`, which is what the route
handlers actually call) -- what's under test is app.py's own routing,
request validation, decode/size handling, and error mapping, not the
Bedrock integration vision_core.py owns.

Run with: pytest service/vision_analyze/tests/ -q
"""

import base64

import pytest
from fastapi.testclient import TestClient

import app as app_module

IMAGE_B64 = base64.b64encode(b"not-a-real-jpeg-but-bytes-are-bytes").decode()


@pytest.fixture
def client():
    """The module-level app is a stateless FastAPI object (its only
    creation-time environment read is ALLOWED_ORIGINS, irrelevant to
    TestClient calls, which never send an Origin header) -- reusing it is
    safe, and route handlers resolve describe_image_bytes()/identify_room()
    etc. from app_module's globals at call time, so a test's
    monkeypatch.setattr(app_module, ...) is picked up with no reload."""
    return TestClient(app_module.app)


def scene_reply(**overrides):
    reply = {
        "obstacles_ahead": [],
        "free_space": "clear",
        "doorway_visible": False,
        "important_objects": ["red backpack"],
        "safest_direction": "FORWARD",
    }
    reply.update(overrides)
    return reply


def navigate_reply(**overrides):
    reply = {
        "target_visible": True,
        "target_direction": "center",
        "target_reached": False,
        "obstacle_ahead": False,
        "room_guess": "kitchen",
        "action": "FORWARD",
        "reasoning": "the backpack is ahead",
    }
    reply.update(overrides)
    return reply


def guidance_reply(**overrides):
    reply = {
        "target_visible": True,
        "position": "center",
        "proximity": "medium",
        "guidance": "Walk forward",
        "bounding_box": None,
    }
    reply.update(overrides)
    return reply


# ---------- health ----------


def test_health(client):
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok"}


# ---------- /analyze ----------


def test_analyze_returns_the_scene_plus_a_room_guess(client, monkeypatch):
    monkeypatch.setattr(app_module, "describe_image_bytes", lambda b, mt: scene_reply())
    monkeypatch.setattr(app_module, "identify_room", lambda objs: "living room")

    resp = client.post("/analyze", json={"image_base64": IMAGE_B64})

    assert resp.status_code == 200
    body = resp.json()
    assert body["important_objects"] == ["red backpack"]
    assert body["room_guess"] == "living room"


def test_analyze_missing_image_base64_is_a_400(client):
    resp = client.post("/analyze", json={})
    assert resp.status_code == 400


def test_analyze_bad_base64_is_a_400(client):
    resp = client.post("/analyze", json={"image_base64": "not valid base64!!"})
    assert resp.status_code == 400


def test_analyze_oversized_image_is_a_413(client, monkeypatch):
    monkeypatch.setattr(app_module, "MAX_IMAGE_BYTES", 4)
    resp = client.post("/analyze", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 413


def test_analyze_a_vision_failure_is_a_502_not_a_crash(client, monkeypatch):
    def boom(b, mt):
        raise RuntimeError("bedrock: throttled")

    monkeypatch.setattr(app_module, "describe_image_bytes", boom)
    resp = client.post("/analyze", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 502


def test_analyze_requires_the_shared_secret_when_configured(client, monkeypatch):
    monkeypatch.setenv("APP_SHARED_SECRET", "s3cret")
    monkeypatch.setattr(app_module, "describe_image_bytes", lambda b, mt: scene_reply())
    monkeypatch.setattr(app_module, "identify_room", lambda objs: "unknown")

    unauthorized = client.post("/analyze", json={"image_base64": IMAGE_B64})
    assert unauthorized.status_code == 401

    authorized = client.post(
        "/analyze", json={"image_base64": IMAGE_B64}, headers={"x-app-secret": "s3cret"}
    )
    assert authorized.status_code == 200


# ---------- /describe ----------


def test_describe_returns_the_person_facing_schema(client, monkeypatch):
    monkeypatch.setattr(
        app_module, "describe_image_bytes_person",
        lambda b, mt: {"summary": "a tidy kitchen", "room_type": "kitchen", "objects": ["kettle"]},
    )
    resp = client.post("/describe", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 200
    assert resp.json()["room_type"] == "kitchen"


def test_describe_a_vision_failure_is_a_502(client, monkeypatch):
    def boom(b, mt):
        raise RuntimeError("bedrock: down")

    monkeypatch.setattr(app_module, "describe_image_bytes_person", boom)
    resp = client.post("/describe", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 502


# ---------- /navigate ----------


def test_navigate_requires_a_target_object(client):
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 400


def test_navigate_returns_the_decision_verbatim(client, monkeypatch):
    monkeypatch.setattr(
        app_module, "describe_image_bytes_navigate",
        lambda b, target, mt, searched_rooms=None, model_id=None, prompt_variant=None: navigate_reply(),
    )
    resp = client.post(
        "/navigate", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["action"] == "FORWARD"
    assert body["room_guess"] == "kitchen"


def test_navigate_forwards_searched_rooms(client, monkeypatch):
    """The room-level step-memory wiring (AGENT-HARNESS.md section 12):
    whatever the client says it has already searched must reach
    vision_core.describe_image_bytes_navigate()'s searched_rooms kwarg."""
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None, prompt_variant=None):
        seen["searched_rooms"] = searched_rooms
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)

    client.post(
        "/navigate",
        json={
            "image_base64": IMAGE_B64,
            "target_object": "red backpack",
            "searched_rooms": ["kitchen", "hallway"],
        },
    )

    assert seen["searched_rooms"] == ["kitchen", "hallway"]


def test_navigate_defaults_searched_rooms_to_empty(client, monkeypatch):
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None, prompt_variant=None):
        seen["searched_rooms"] = searched_rooms
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)

    client.post(
        "/navigate", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )

    assert seen["searched_rooms"] == []


def test_navigate_ignores_a_malformed_searched_rooms(client, monkeypatch):
    """A client bug (wrong type, junk entries) should degrade to "nothing
    searched", never a 500 -- this route's own honesty matters more than
    a caller's mistake."""
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None, prompt_variant=None):
        seen["searched_rooms"] = searched_rooms
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)

    client.post(
        "/navigate",
        json={
            "image_base64": IMAGE_B64,
            "target_object": "red backpack",
            "searched_rooms": "kitchen",  # a string, not a list -- a plausible client bug
        },
    )
    assert seen["searched_rooms"] == []

    client.post(
        "/navigate",
        json={
            "image_base64": IMAGE_B64,
            "target_object": "red backpack",
            "searched_rooms": ["kitchen", 7, "", None],
        },
    )
    assert seen["searched_rooms"] == ["kitchen"]


def test_navigate_a_vision_failure_is_a_502(client, monkeypatch):
    def boom(b, target, mt, searched_rooms=None, model_id=None, prompt_variant=None):
        raise RuntimeError("bedrock: throttled")

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", boom)
    resp = client.post(
        "/navigate", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )
    assert resp.status_code == 502


# ---------- /navigate model selection ----------


def test_navigate_models_lists_the_allowed_set_and_default(client):
    resp = client.get("/navigate/models")
    assert resp.status_code == 200
    body = resp.json()
    assert body["default"] == app_module.NAVIGATE_MODEL_ID
    ids = {m["id"] for m in body["models"]}
    assert ids == set(app_module.NAVIGATE_MODEL_CHOICES)
    assert all(isinstance(m["label"], str) and m["label"] for m in body["models"])


def test_navigate_forwards_a_valid_model_id(client, monkeypatch):
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None, prompt_variant=None):
        seen["model_id"] = model_id
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)
    chosen = next(iter(app_module.NAVIGATE_MODEL_CHOICES))

    resp = client.post(
        "/navigate",
        json={"image_base64": IMAGE_B64, "target_object": "red backpack", "model_id": chosen},
    )

    assert resp.status_code == 200
    assert seen["model_id"] == chosen


def test_navigate_defaults_model_id_to_none(client, monkeypatch):
    """Omitting model_id must reach vision_core as None, not some guessed
    string -- describe_image_bytes_navigate()'s own default (NAVIGATE_MODEL_ID)
    is the single source of truth for what "no preference" means."""
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None, prompt_variant=None):
        seen["model_id"] = model_id
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)

    client.post(
        "/navigate", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )

    assert seen["model_id"] is None


def test_navigate_rejects_a_model_id_outside_the_allow_list(client, monkeypatch):
    """The allow-list check must happen before any Bedrock call -- an
    arbitrary client-supplied string must never reach modelId= in
    vision_core, whether that's a typo, a model this account can't invoke,
    or an attempt to run up the bill on something expensive."""
    def fail_if_called(*a, **k):
        raise AssertionError("describe_image_bytes_navigate must not be called for a bad model_id")

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fail_if_called)

    resp = client.post(
        "/navigate",
        json={
            "image_base64": IMAGE_B64,
            "target_object": "red backpack",
            "model_id": "anthropic.claude-sonnet-5",
        },
    )

    assert resp.status_code == 400


# ---------- /guidance ----------


def test_guidance_requires_a_target_object(client):
    resp = client.post("/guidance", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 400


def test_guidance_returns_the_result_verbatim(client, monkeypatch):
    monkeypatch.setattr(
        app_module, "describe_image_bytes_guidance",
        lambda b, target, mt: guidance_reply(),
    )
    resp = client.post(
        "/guidance", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )
    assert resp.status_code == 200
    assert resp.json()["guidance"] == "Walk forward"


def test_guidance_a_vision_failure_is_a_502(client, monkeypatch):
    def boom(b, target, mt):
        raise RuntimeError("bedrock: down")

    monkeypatch.setattr(app_module, "describe_image_bytes_guidance", boom)
    resp = client.post(
        "/guidance", json={"image_base64": IMAGE_B64, "target_object": "red backpack"}
    )
    assert resp.status_code == 502


# ---------- /navigate prompt variants ----------
#
# The prompt is a lever at least as strong as the model, and was the only one
# that could not be varied without a redeploy. The stall this project spent
# days on was a wording problem: "is there an obstacle directly ahead" read as
# "is there furniture anywhere in front of me", which stays true from every
# angle and so never lets FORWARD come back.


def test_navigate_models_also_publishes_the_prompt_variants(client):
    body = client.get("/navigate/models").json()
    assert body["default_prompt"] == app_module.DEFAULT_PROMPT_VARIANT
    assert set(body["prompts"]) == set(app_module.NAVIGATE_PROMPT_VARIANTS)
    assert "default" in body["prompts"]


def test_navigate_forwards_a_valid_prompt_variant(client, monkeypatch):
    seen = {}

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None,
             prompt_variant=None):
        seen["prompt_variant"] = prompt_variant
        return navigate_reply()

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake)
    variant = sorted(app_module.NAVIGATE_PROMPT_VARIANTS)[-1]

    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "target_object": "red backpack",
        "prompt_variant": variant})

    assert resp.status_code == 200
    assert seen["prompt_variant"] == variant


def test_navigate_rejects_an_unknown_prompt_variant(client, monkeypatch):
    """Validated before any Bedrock call, exactly like model_id -- a typo
    must not silently fall back to the default wording, or a whole recorded
    walk would be attributed to a prompt that never ran."""
    def fail_if_called(*a, **k):
        raise AssertionError("must not reach the model for a bad prompt_variant")

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fail_if_called)
    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "target_object": "x", "prompt_variant": "made-up"})
    assert resp.status_code == 400


# ---------- the ordinal distance estimate (Q2) ----------


def test_the_distance_variant_is_published_like_any_other(client):
    """Served from the allow-list rather than hardcoded in a client, so the
    twin's prompt picker offers it and a recorded walk says which wording
    produced it -- the same discipline the model picker follows."""
    body = client.get("/navigate/models").json()
    assert "default-with-distance" in body["prompts"]


def test_the_default_prompt_is_not_touched_by_the_distance_variant():
    """CLAUDE.md's 3x3 matrix measured the default's exact wording, and
    production serves the same file. A variant that edited the default
    would silently invalidate those numbers and change production's
    behaviour on the next deploy."""
    import vision_core

    assert (vision_core.NAVIGATE_PROMPT_VARIANTS["default"]
            is vision_core.NAVIGATE_PROMPT_TEMPLATE)
    assert "distance_estimate" not in vision_core.NAVIGATE_PROMPT_TEMPLATE


def test_the_distance_variant_asks_for_the_field_it_promises():
    """The prompt and the response schema have to agree, or the field comes
    back absent on every call and the veto can never fire."""
    import vision_core

    prompt = vision_core.NAVIGATE_PROMPT_VARIANTS["default-with-distance"]
    rendered = prompt.format(target_object="red backpack", searched_rooms_note="")
    assert "within_one_step" in rendered
    assert '"distance_estimate"' in rendered
    # The target is the one thing that must not read as an obstacle --
    # otherwise arriving looks exactly like being blocked.
    assert "does not count" in rendered


@pytest.mark.parametrize("raw,expected", [
    ("within_one_step", "within_one_step"),
    ("a_few_steps", "a_few_steps"),
    ("far", "far"),
    ("VERY CLOSE", "unknown"),      # off the allow-list
    ("12cm", "unknown"),            # a number, which this field never is
    (None, "unknown"),
])
def test_an_unrecognised_distance_estimate_becomes_unknown(raw, expected):
    """A caller may veto a move on this field, so anything unrecognised has
    to fail towards "do not act on it" -- never towards a made-up level of
    confidence."""
    import json as _json

    import vision_core

    payload = {"target_visible": False, "target_direction": "not_visible",
               "target_reached": False, "obstacle_ahead": False,
               "room_guess": "kitchen", "action": "FORWARD",
               "reasoning": "x"}
    if raw is not None:
        payload["distance_estimate"] = raw
    parsed = vision_core._parse_navigate_json(_json.dumps(payload))
    assert parsed["distance_estimate"] == expected


def test_distance_estimate_is_always_present_even_on_a_garbage_reply():
    """A client should never have to tell "the model said nothing" apart
    from "this variant does not ask"."""
    import vision_core

    assert vision_core._parse_navigate_json("not json at all")["distance_estimate"] == "unknown"


# ---------- the center-third path question (Q2, fourth attempt) ----------


def test_the_center_third_variant_is_published_like_any_other(client):
    """Same discipline as every other wording: served from the allow-list, so
    the twin's picker offers it without a client-side hardcode and a recorded
    walk can say which prompt produced it."""
    body = client.get("/navigate/models").json()
    assert "center-third-path" in body["prompts"]


def test_the_default_prompt_is_not_touched_by_the_center_third_variant():
    """The Stage 0 table measured the default's exact wording and production
    serves this file, so a variant that edited the default in place would
    both invalidate those numbers and change production on the next deploy."""
    import vision_core

    assert (vision_core.NAVIGATE_PROMPT_VARIANTS["default"]
            is vision_core.NAVIGATE_PROMPT_TEMPLATE)
    assert "path_ahead" not in vision_core.NAVIGATE_PROMPT_TEMPLATE


def test_the_center_third_variant_asks_for_the_field_it_promises():
    """The prompt and the response schema have to agree, or the field comes
    back absent on every call and there is nothing to measure."""
    import vision_core

    rendered = vision_core.NAVIGATE_PROMPT_VARIANTS["center-third-path"].format(
        target_object="red backpack", searched_rooms_note="")
    assert '"path_ahead"' in rendered
    assert "open_floor" in rendered
    # The region restriction IS the experiment -- without it this is just
    # another rewording of "is there an obstacle ahead".
    assert "bottom half of the center third" in rendered
    # Approaching the target must not read as being blocked by it, or
    # arriving looks exactly like a collision.
    assert "never counts as" in rendered


def test_the_center_third_variant_changes_question_2_and_nothing_else():
    """The point of this variant is attribution. "next-step-obstacle" moved
    question 2 and question 5 together and went degenerate, and there was no
    way to tell which half did it -- so this one moves question 2 alone, and
    that property is worth pinning rather than trusting to a comment."""
    import vision_core

    default = vision_core.NAVIGATE_PROMPT_TEMPLATE
    variant = vision_core.NAVIGATE_PROMPT_VARIANTS["center-third-path"]

    # Question 2 is gone, replaced.
    assert "2. Is there an obstacle directly ahead" in default
    assert "2. Is there an obstacle directly ahead" not in variant

    # Questions 1, 3, 4 and 5 survive verbatim. Question 5 especially: it is
    # the one the previous attempt also moved, and holding it fixed is what
    # makes a degenerate result here point at question 5 rather than at the
    # region change.
    for shared in (
        "1. Is the {target_object} visible in this image?",
        "3. Has the robot ARRIVED at the {target_object}?",
        "4. What kind of room does this look like",
        "5. Given the above, what is the single best next action to get closer to the\n"
        "   {target_object} while not colliding with anything?{searched_rooms_note}",
    ):
        assert shared in default
        assert shared in variant


@pytest.mark.parametrize("raw,expected", [
    ("open_floor", "open_floor"),
    ("blocked", "blocked"),
    ("unclear", "unclear"),
    ("clear", "unclear"),        # plausible, off the allow-list, still refused
    ("OPEN_FLOOR", "unclear"),   # case is not normalised on purpose
    (None, "unclear"),
])
def test_an_unrecognised_path_ahead_becomes_unclear(raw, expected):
    """Same contract as distance_estimate: anything unrecognised fails
    towards "do not act on it", never towards a made-up confidence."""
    import json as _json

    import vision_core

    payload = {"target_visible": False, "target_direction": "not_visible",
               "target_reached": False, "obstacle_ahead": False,
               "room_guess": "kitchen", "action": "FORWARD",
               "reasoning": "x"}
    if raw is not None:
        payload["path_ahead"] = raw
    parsed = vision_core._parse_navigate_json(_json.dumps(payload))
    assert parsed["path_ahead"] == expected


def test_path_ahead_is_always_present_even_on_a_garbage_reply():
    """A client should never have to tell "the model said nothing" apart from
    "this variant does not ask"."""
    import vision_core

    assert vision_core._parse_navigate_json("not json at all")["path_ahead"] == "unclear"


def test_obstacle_ahead_is_not_recomputed_from_path_ahead():
    """The prompt asks the model to keep the two in step, and whether it
    actually does is the measurement. Deriving one from the other in the
    parser would manufacture the agreement and hide the disagreement worth
    seeing -- the same mistake as grading a replay without reading its
    coverage."""
    import json as _json

    import vision_core

    parsed = vision_core._parse_navigate_json(_json.dumps({
        "target_visible": False, "target_direction": "not_visible",
        "target_reached": False, "obstacle_ahead": False,
        "room_guess": "kitchen", "action": "FORWARD",
        "path_ahead": "blocked", "reasoning": "x"}))
    assert parsed["path_ahead"] == "blocked"
    assert parsed["obstacle_ahead"] is False


# ---------- bearing-only: the obstacle question deleted (M1) ----------


def test_the_bearing_only_variant_is_published_like_any_other(client):
    """Same discipline as every other wording: served from the allow-list, so
    the twin's picker offers it without a client-side hardcode and a recorded
    walk can say which prompt produced it."""
    body = client.get("/navigate/models").json()
    assert "bearing-only" in body["prompts"]


def test_the_bearing_only_variant_removes_question_2_and_nothing_else():
    """The whole experiment is the subtraction. Four wordings of question 2
    have been measured and two of them produced never-FORWARD while two
    produced always-FORWARD; this variant asks whether the question is worth
    asking a photograph at all. If anything ELSE moved with it, the answer
    would not be attributable -- the same trap "next-step-obstacle" fell into
    by moving questions 2 and 5 together."""
    import difflib

    import vision_core

    default = vision_core.NAVIGATE_PROMPT_TEMPLATE
    variant = vision_core.NAVIGATE_PROMPT_VARIANTS["bearing-only"]

    removed = [line for line in difflib.ndiff(default.splitlines(), variant.splitlines())
               if line.startswith("- ")]
    added = [line for line in difflib.ndiff(default.splitlines(), variant.splitlines())
             if line.startswith("+ ")]
    assert added == []
    assert removed == [
        "- 2. Is there an obstacle directly ahead that would block moving forward?",
        '-   "obstacle_ahead": true | false,',
    ]


def test_the_bearing_only_variant_does_not_touch_the_default():
    """Production serves this file, and the Stage 0 table measured the
    default's exact wording."""
    import vision_core

    assert (vision_core.NAVIGATE_PROMPT_VARIANTS["default"]
            is vision_core.NAVIGATE_PROMPT_TEMPLATE)
    assert "obstacle_ahead" in vision_core.NAVIGATE_PROMPT_TEMPLATE


def test_variant_asks_obstacle_is_read_off_the_template():
    """Derived, not a second list. A variant that drops the field cannot
    forget to register itself, which is the failure mode a hand-maintained
    set of names would have."""
    import vision_core

    assert vision_core.variant_asks_obstacle("default") is True
    assert vision_core.variant_asks_obstacle("center-third-path") is True
    assert vision_core.variant_asks_obstacle("default-with-distance") is True
    assert vision_core.variant_asks_obstacle("bearing-only") is False
    # An unknown name resolves to the default template, which is what
    # describe_image_bytes_navigate() will actually send for it.
    assert vision_core.variant_asks_obstacle("made-up") is True


def _stub_converse(monkeypatch, reply_json: str):
    import vision_core

    class FakeClient:
        def converse(self, **kwargs):
            self.kwargs = kwargs
            return {"output": {"message": {"content": [{"text": reply_json}]}},
                    "usage": {"inputTokens": 1, "outputTokens": 2}}

    fake = FakeClient()
    # Takes the model id _get_client() is now called with (the region pin --
    # see vision_core.MODEL_REGIONS); the stub ignores it, since which client
    # comes back is what test_a_pinned_model_is_called_in_its_own_region
    # covers, not this.
    monkeypatch.setattr(vision_core, "_get_client", lambda *a, **kw: fake)
    return fake


def test_a_pinned_model_is_called_in_its_own_region(monkeypatch):
    """Claude Fable 5.1 answers only from us-east-1 on this account -- it is
    refused from us-east-2, where this service is deployed, and from
    us-west-2. The pin is the whole reason it can be in the picker, so it is
    pinned here rather than left to whatever region the task happens to run
    in."""
    import vision_core

    monkeypatch.setattr(vision_core, "_clients", {})
    built = []
    monkeypatch.setattr(vision_core.boto3, "client",
                        lambda svc, region_name=None: built.append(region_name) or object())

    vision_core._get_client("us.anthropic.claude-fable-5-1")
    vision_core._get_client("us.anthropic.claude-opus-4-5-20251101-v1:0")

    # The pinned model names its region; everything else takes the ambient
    # one, which is None here -- boto3 resolving it is the point.
    assert built == ["us-east-1", None]


def test_clients_are_cached_per_region_not_globally(monkeypatch):
    """A single cached client was rebuilt every time the region changed, so
    alternating traffic meant a fresh connection pool per request."""
    import vision_core

    monkeypatch.setattr(vision_core, "_clients", {})
    monkeypatch.setattr(vision_core.boto3, "client",
                        lambda svc, region_name=None: ("client", region_name))

    pinned = vision_core._get_client("us.anthropic.claude-fable-5-1")
    ambient = vision_core._get_client("amazon.nova-lite-v1:0")

    assert pinned is vision_core._get_client("us.anthropic.claude-fable-5-1")
    assert ambient is vision_core._get_client("amazon.nova-lite-v1:0")
    assert pinned is not ambient


def test_the_region_pins_can_be_emptied_by_env(monkeypatch):
    """Expected to be emptied: once Fable 5.1 is enabled in us-east-2 the pin
    buys a slower call and nothing else, and that must not need a redeploy of
    code to undo."""
    import vision_core

    monkeypatch.setenv("BEDROCK_MODEL_REGIONS", "")
    assert vision_core._load_model_regions() == {}

    monkeypatch.setenv("BEDROCK_MODEL_REGIONS", "some.model=eu-west-1")
    assert vision_core._load_model_regions() == {"some.model": "eu-west-1"}

    monkeypatch.delenv("BEDROCK_MODEL_REGIONS")
    assert vision_core._load_model_regions() == vision_core._DEFAULT_MODEL_REGIONS


def test_the_new_models_are_offered_by_the_picker(monkeypatch):
    """The three added on 2026-09-21, each confirmed with a real Converse
    call before being listed. The picker is populated from this set alone, so
    this is the whole contract between the allow-list and the UI."""
    import vision_core

    for model_id in ("us.anthropic.claude-fable-5-1",
                     "us.anthropic.claude-opus-5",
                     "us.openai.gpt-6-astra"):
        assert model_id in vision_core.NAVIGATE_MODEL_CHOICES

    # The default is deliberately NOT one of them -- nothing has replayed a
    # walk through them, and promoting on a guess is the NavigateModelId
    # mistake this repo already paid for once.
    assert vision_core.NAVIGATE_MODEL_ID == "us.anthropic.claude-opus-4-5-20251101-v1:0"


def test_bearing_only_omits_obstacle_ahead_from_the_reply(monkeypatch):
    """Absent, not false. _parse_navigate_json merges the empty schema over
    every reply, so without the strip this field would come back False on a
    variant that never asked -- indistinguishable from a model that looked at
    the frame and saw clear floor. A caller has to be able to tell those
    apart; brain/navigate.py reports the second as free_space "unknown"."""
    import json as _json

    import vision_core

    _stub_converse(monkeypatch, _json.dumps({
        "target_visible": True, "target_direction": "center",
        "target_reached": False, "room_guess": "hallway",
        "action": "FORWARD", "reasoning": "x"}))

    decision = vision_core.describe_image_bytes_navigate(
        b"jpegbytes", "red backpack", "image/jpeg", prompt_variant="bearing-only")

    assert "obstacle_ahead" not in decision
    assert decision["action"] == "FORWARD"
    assert decision["prompt_variant"] == "bearing-only"


def test_bearing_only_strips_obstacle_ahead_even_if_the_model_volunteers_it(monkeypatch):
    """The field is absent because nobody asked, so a model that answers a
    question it was not asked does not get to reintroduce the very signal
    this variant exists to remove."""
    import json as _json

    import vision_core

    _stub_converse(monkeypatch, _json.dumps({
        "target_visible": False, "target_direction": "not_visible",
        "target_reached": False, "obstacle_ahead": True,
        "room_guess": "hallway", "action": "STOP", "reasoning": "x"}))

    decision = vision_core.describe_image_bytes_navigate(
        b"jpegbytes", "red backpack", "image/jpeg", prompt_variant="bearing-only")

    assert "obstacle_ahead" not in decision


def test_a_variant_that_asks_still_reports_obstacle_ahead(monkeypatch):
    """The strip is keyed off the template, so it must not touch the
    variants the Stage 0 table is measured on."""
    import json as _json

    import vision_core

    _stub_converse(monkeypatch, _json.dumps({
        "target_visible": False, "target_direction": "not_visible",
        "target_reached": False, "obstacle_ahead": True,
        "room_guess": "hallway", "action": "STOP", "reasoning": "x"}))

    decision = vision_core.describe_image_bytes_navigate(
        b"jpegbytes", "red backpack", "image/jpeg", prompt_variant="default")

    assert decision["obstacle_ahead"] is True
