"""
Run with: pytest lambda/vision_analyze/test_handler.py -v

Simulates AWS Lambda Function URL events directly against handler() --
no AWS deployment needed to validate the logic. describe_image_bytes is
mocked the same way brain/vision.py's tests mock the Anthropic client,
so this suite runs offline with no API key and no cost.
"""

import base64
import json
from unittest.mock import patch

import pytest

from handler import handler


TINY_PNG = bytes.fromhex(
    "89504e470d0a1a0a0000000d4948445200000001000000010802000000"
    "907753360000000a49444154789c6360000002000100ffff03000006"
    "0005579d7a09000000004945"
)


def make_event(body, origin="http://localhost:5173", secret=None, method="POST") -> dict:
    headers = {"origin": origin}
    if secret is not None:
        headers["x-app-secret"] = secret
    return {
        "headers": headers,
        "requestContext": {"http": {"method": method}},
        "body": json.dumps(body) if body is not None else None,
    }


@pytest.fixture(autouse=True)
def no_shared_secret_by_default(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.delenv("ALLOWED_ORIGINS", raising=False)


def test_options_preflight_returns_204_with_cors_headers():
    event = make_event(None, method="OPTIONS")
    resp = handler(event, None)
    assert resp["statusCode"] == 204
    assert "Access-Control-Allow-Origin" in resp["headers"]


def test_missing_image_field_returns_400():
    event = make_event({"media_type": "image/jpeg"})
    resp = handler(event, None)
    assert resp["statusCode"] == 400


def test_invalid_base64_returns_400():
    event = make_event({"image_base64": "not valid base64 !!!"})
    resp = handler(event, None)
    assert resp["statusCode"] == 400


def test_oversized_image_returns_413():
    huge = base64.b64encode(b"0" * (6 * 1024 * 1024)).decode()
    event = make_event({"image_base64": huge})
    resp = handler(event, None)
    assert resp["statusCode"] == 413


@patch("handler.describe_image_bytes")
def test_successful_analysis_includes_room_guess(mock_describe):
    mock_describe.return_value = {
        "obstacles_ahead": [],
        "free_space": "clear",
        "doorway_visible": False,
        "important_objects": ["red backpack", "refrigerator"],
        "safest_direction": "FORWARD",
    }
    image_b64 = base64.b64encode(TINY_PNG).decode()
    event = make_event({"image_base64": image_b64, "media_type": "image/png"})

    resp = handler(event, None)

    assert resp["statusCode"] == 200
    body = json.loads(resp["body"])
    assert body["important_objects"] == ["red backpack", "refrigerator"]
    assert body["room_guess"] == "kitchen"
    mock_describe.assert_called_once()


@patch("handler.describe_image_bytes")
def test_vision_api_failure_returns_502(mock_describe):
    mock_describe.side_effect = RuntimeError("upstream timeout")
    image_b64 = base64.b64encode(TINY_PNG).decode()
    event = make_event({"image_base64": image_b64})

    resp = handler(event, None)
    assert resp["statusCode"] == 502


def test_shared_secret_required_when_configured(monkeypatch):
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    image_b64 = base64.b64encode(TINY_PNG).decode()

    event_no_secret = make_event({"image_base64": image_b64})
    resp = handler(event_no_secret, None)
    assert resp["statusCode"] == 401

    event_wrong_secret = make_event({"image_base64": image_b64}, secret="wrong")
    resp = handler(event_wrong_secret, None)
    assert resp["statusCode"] == 401


@patch("handler.describe_image_bytes")
def test_shared_secret_accepted_when_correct(mock_describe, monkeypatch):
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    mock_describe.return_value = {
        "obstacles_ahead": [],
        "free_space": "clear",
        "doorway_visible": False,
        "important_objects": [],
        "safest_direction": "FORWARD",
    }
    image_b64 = base64.b64encode(TINY_PNG).decode()
    event = make_event({"image_base64": image_b64}, secret="correct-horse-battery-staple")

    resp = handler(event, None)
    assert resp["statusCode"] == 200


def test_disallowed_origin_gets_null_cors_origin():
    event = make_event({"image_base64": "x"}, origin="https://evil.example.com")
    resp = handler(event, None)
    assert resp["headers"]["Access-Control-Allow-Origin"] == "null"


def test_allowed_origins_configurable_via_env(monkeypatch):
    monkeypatch.setenv("ALLOWED_ORIGINS", "https://my-twin.example.com")
    event = make_event({"image_base64": "x"}, origin="https://my-twin.example.com")
    resp = handler(event, None)
    assert resp["headers"]["Access-Control-Allow-Origin"] == "https://my-twin.example.com"
