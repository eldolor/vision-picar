"""
handler.py

AWS Lambda handler for the "find the bag in my photo" feature: the web
twin uploads a phone photo, this calls the same describe_image()/
identify_room() logic already validated in brain/vision.py and
brain/rooms.py (just accepting bytes instead of a file path), and
returns the same structured scene schema plus a room guess.

Deploy behind a Function URL (not API Gateway) -- simplest setup for a
single endpoint. See README.md in this folder for exact deploy steps.

Environment variables:
    ANTHROPIC_API_KEY   required. Set as an encrypted Lambda env var --
                        never shipped in the deployment package or in
                        any client-side code.
    APP_SHARED_SECRET   optional but recommended. A Claude API call
                        costs real money per request; a public Function
                        URL with no auth is an open invitation to abuse
                        someone else's traffic. If set, requests must
                        include a matching `x-app-secret` header. If
                        unset, the check is skipped (fine for initial
                        local testing, not for a public deploy).
    ALLOWED_ORIGINS     optional, comma-separated. Defaults to allowing
                        only localhost for local dev -- set this to your
                        deployed web twin's actual origin(s).

Request body (JSON):
    {
      "image_base64": "...",
      "media_type": "image/jpeg"   // optional, defaults to image/jpeg
    }

Response body (JSON), same schema as brain/vision.py plus room_guess:
    {
      "obstacles_ahead": [...],
      "free_space": "...",
      "doorway_visible": bool,
      "important_objects": [...],
      "safest_direction": "...",
      "room_guess": "kitchen"
    }
"""

import json
import base64
import logging
import os

from vision_core import describe_image_bytes
from rooms_core import identify_room

logger = logging.getLogger()
logger.setLevel(logging.INFO)

MAX_IMAGE_BYTES = 5 * 1024 * 1024  # 5MB, matches Claude's API limit

_DEFAULT_ALLOWED_ORIGINS = {"http://localhost:5173", "http://localhost:8000"}


def _allowed_origins() -> set:
    configured = os.environ.get("ALLOWED_ORIGINS")
    if configured:
        return {o.strip() for o in configured.split(",") if o.strip()}
    return _DEFAULT_ALLOWED_ORIGINS


def _cors_headers(origin: str) -> dict:
    allow_origin = origin if origin in _allowed_origins() else "null"
    return {
        "Access-Control-Allow-Origin": allow_origin,
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type, x-app-secret",
        "Vary": "Origin",
    }


def _get_header(headers: dict, name: str) -> str:
    """Lambda Function URL headers arrive lowercased, but be defensive."""
    headers = headers or {}
    for key, value in headers.items():
        if key.lower() == name.lower():
            return value
    return ""


def _error(status: int, message: str, headers: dict) -> dict:
    return {"statusCode": status, "headers": headers, "body": json.dumps({"error": message})}


def handler(event, context):
    headers_in = event.get("headers") or {}
    origin = _get_header(headers_in, "origin")
    headers_out = _cors_headers(origin)

    method = event.get("requestContext", {}).get("http", {}).get("method", "")
    if method == "OPTIONS":
        return {"statusCode": 204, "headers": headers_out, "body": ""}

    expected_secret = os.environ.get("APP_SHARED_SECRET")
    if expected_secret:
        provided = _get_header(headers_in, "x-app-secret")
        if provided != expected_secret:
            return _error(401, "Missing or invalid x-app-secret header.", headers_out)

    try:
        body = json.loads(event.get("body") or "{}")
        image_b64 = body["image_base64"]
        media_type = body.get("media_type", "image/jpeg")
    except (KeyError, json.JSONDecodeError) as e:
        return _error(400, f"Bad request: {e}", headers_out)

    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception:
        return _error(400, "image_base64 did not decode.", headers_out)

    if len(image_bytes) > MAX_IMAGE_BYTES:
        return _error(413, "Image too large (5MB limit).", headers_out)

    try:
        scene = describe_image_bytes(image_bytes, media_type)
    except Exception as e:
        logger.exception("Vision API call failed")
        return _error(502, f"Vision analysis failed: {e}", headers_out)

    scene["room_guess"] = identify_room(scene.get("important_objects", []))

    return {"statusCode": 200, "headers": headers_out, "body": json.dumps(scene)}
