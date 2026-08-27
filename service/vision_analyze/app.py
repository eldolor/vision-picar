"""
app.py

ECS Fargate replacement for lambda/vision_analyze/handler.py: the web
twin uploads a phone photo, this calls the same describe_image_bytes()/
identify_room() logic, and returns the same structured scene schema
plus a room guess.

Same feature as the Lambda deployment, different ingress path -- see
cloudformation/README.md for why (Lambda's public entry points are
blocked by an account-level restriction; a container behind an
NLB -> ALB doesn't go through Lambda's invoke-permission model at all).

Environment variables:
    ANTHROPIC_API_KEY   required. Injected from Secrets Manager via the
                        ECS task definition's `secrets` block, never
                        baked into the image or this repo.
    APP_SHARED_SECRET   required in production. If set, requests must
                        include a matching `x-app-secret` header --
                        without it, a public load balancer is an open
                        invitation to run up the Anthropic bill. If
                        unset, the check is skipped (fine for local
                        `docker run` testing only).
    ALLOWED_ORIGINS     optional, comma-separated. Defaults to allowing
                        only localhost for local dev. A single "*"
                        allows any origin (matches robot/server.py's
                        own allowed_origins default) -- fine while
                        APP_SHARED_SECRET is the real access control.

POST /analyze  -- one-shot scene description of an uploaded photo, in the
                  robot's terms (obstacles, free space, safest direction).
POST /describe -- the same photo described for the person who took it.

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

POST /navigate -- given a camera frame and a target object, picks the
robot's single next move. Built for the digital twin's Vision Autopilot
panel (web-twin/index.html), which renders a synthetic first-person view
of the grid-world sim and drives it with real Claude Vision calls instead
of the twin's rule-based frontier-exploration mode. See vision_core.py's
describe_image_bytes_navigate() for the model call.

Request body (JSON):
    {
      "image_base64": "...",
      "media_type": "image/jpeg",  // optional, defaults to image/jpeg
      "target_object": "red backpack"   // required
    }

Response body (JSON):
    {
      "target_visible": bool,
      "target_direction": "left" | "center" | "right" | "not_visible",
      "obstacle_ahead": bool,
      "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
      "reasoning": "..."
    }

POST /guidance -- AR-style directional guidance for a human holding their
phone, searching for a target object with their own real camera (as
opposed to /navigate, which picks a discrete move for the simulated
robot). Built for the digital twin's "Guide me to..." panel
(web-twin/index.html), which overlays a directional arrow on the live
camera feed. See vision_core.py's describe_image_bytes_guidance().

Request body (JSON):
    {
      "image_base64": "...",
      "media_type": "image/jpeg",  // optional, defaults to image/jpeg
      "target_object": "red backpack"   // required
    }

Response body (JSON):
    {
      "target_visible": bool,
      "position": "far_left" | "left" | "center" | "right" | "far_right" | "not_visible",
      "proximity": "near" | "medium" | "far" | "unknown",
      "guidance": "...",
      "bounding_box": {"x_min": float, "y_min": float, "x_max": float, "y_max": float} | null
        -- normalized (0.0-1.0) coordinates, only present when the model is
           confident of the object's extent; null otherwise. Used by the
           web twin's Guide-me-to panel to draw an outline around the
           object once found, instead of a generic checkmark.
    }
"""

import base64
import logging
import os

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware

from vision_core import (
    describe_image_bytes,
    describe_image_bytes_person,
    describe_image_bytes_navigate,
    describe_image_bytes_guidance,
)
from rooms_core import identify_room

logger = logging.getLogger("vision_analyze")

MAX_IMAGE_BYTES = 5 * 1024 * 1024  # 5MB, matches Claude's API limit

_DEFAULT_ALLOWED_ORIGINS = ["http://localhost:5173", "http://localhost:8000"]


def _allowed_origins() -> list:
    configured = os.environ.get("ALLOWED_ORIGINS")
    if configured:
        return [o.strip() for o in configured.split(",") if o.strip()]
    return _DEFAULT_ALLOWED_ORIGINS


def _check_secret(request: Request):
    expected_secret = os.environ.get("APP_SHARED_SECRET")
    if expected_secret:
        provided = request.headers.get("x-app-secret", "")
        if provided != expected_secret:
            raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


async def _decode_image(request: Request) -> tuple:
    """Shared by /analyze and /navigate: auth check, JSON parse, base64
    decode, size check. Returns (image_bytes, media_type, body)."""
    _check_secret(request)

    try:
        body = await request.json()
        image_b64 = body["image_base64"]
        media_type = body.get("media_type", "image/jpeg")
    except (KeyError, ValueError) as e:
        raise HTTPException(status_code=400, detail=f"Bad request: {e}")

    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception:
        raise HTTPException(status_code=400, detail="image_base64 did not decode.")

    if len(image_bytes) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail="Image too large (5MB limit).")

    return image_bytes, media_type, body


def create_app() -> FastAPI:
    app = FastAPI(title="vision-picar photo analysis service")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=_allowed_origins(),
        allow_methods=["POST", "OPTIONS"],
        allow_headers=["Content-Type", "x-app-secret"],
    )

    @app.get("/health")
    def health():
        return {"status": "ok"}

    @app.post("/analyze")
    async def analyze(request: Request):
        image_bytes, media_type, _body = await _decode_image(request)

        try:
            scene = describe_image_bytes(image_bytes, media_type)
        except Exception as e:
            logger.exception("Vision API call failed")
            raise HTTPException(status_code=502, detail=f"Vision analysis failed: {e}")

        scene["room_guess"] = identify_room(scene.get("important_objects", []))
        return scene

    @app.post("/describe")
    async def describe(request: Request):
        """Person-facing sibling of /analyze.

        /analyze answers what a robot needs to know about a room (obstacles,
        free space, doorways, a safest direction). The twin's Camera tab is
        used by a person photographing their own room, for whom those fields
        are meaningless, so this returns a plain description instead. Its own
        route rather than a flag on /analyze, matching /navigate and
        /guidance.
        """
        image_bytes, media_type, _body = await _decode_image(request)

        try:
            return describe_image_bytes_person(image_bytes, media_type)
        except Exception as e:
            logger.exception("Vision API call failed")
            raise HTTPException(status_code=502, detail=f"Vision description failed: {e}")

    @app.post("/navigate")
    async def navigate(request: Request):
        image_bytes, media_type, body = await _decode_image(request)

        target_object = body.get("target_object")
        if not target_object:
            raise HTTPException(status_code=400, detail="Bad request: 'target_object' is required.")

        try:
            decision = describe_image_bytes_navigate(image_bytes, target_object, media_type)
        except Exception as e:
            logger.exception("Vision API call failed")
            raise HTTPException(status_code=502, detail=f"Vision navigation failed: {e}")

        return decision

    @app.post("/guidance")
    async def guidance(request: Request):
        image_bytes, media_type, body = await _decode_image(request)

        target_object = body.get("target_object")
        if not target_object:
            raise HTTPException(status_code=400, detail="Bad request: 'target_object' is required.")

        try:
            result = describe_image_bytes_guidance(image_bytes, target_object, media_type)
        except Exception as e:
            logger.exception("Vision API call failed")
            raise HTTPException(status_code=502, detail=f"Vision guidance failed: {e}")

        return result

    return app


app = create_app()
