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
        lambda b, target, mt, searched_rooms=None, model_id=None: navigate_reply(),
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

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None):
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

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None):
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

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None):
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
    def boom(b, target, mt, searched_rooms=None, model_id=None):
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

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None):
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

    def fake(image_bytes, target, media_type, searched_rooms=None, model_id=None):
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
