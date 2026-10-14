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
      "target_object": "red backpack",  // required
      "searched_rooms": ["kitchen", "hallway"],  // optional, room-level step
                                                  // memory -- see below
      "model_id": "amazon.nova-lite-v1:0"  // optional, must be one of
                                            // GET /navigate/models's list;
                                            // omit for this route's default
                                            // (vision_core.NAVIGATE_MODEL_ID)
    }

GET /navigate/models -- the model A/B set above, plus which one is the
default, so a caller (the twin's model picker) doesn't hardcode ids that
drift as vision_core.NAVIGATE_MODEL_CHOICES changes:
    {"default": "amazon.nova-lite-v1:0", "models": [{"id": ..., "label": ...}]}

Response body (JSON):
    {
      "target_visible": bool,
      "target_direction": "left" | "center" | "right" | "not_visible",
      "target_reached": bool,
      "obstacle_ahead": bool,
      "room_guess": "kitchen" | "unclear",
      "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
      "reasoning": "...",
      "model_id": "amazon.nova-lite-v1:0",  // which model actually answered
      "usage": {"input_tokens": int | null, "output_tokens": int | null}
    }

`searched_rooms`/`room_guess` close the room-level step-memory gap noted in
AGENT-HARNESS.md section 12: a single photograph carries no history of its
own, so a vision-driven mission (control/brain_server.py's `policy:
"vision"`) feeds back what MissionMemory already knows it has searched, and
reads this route's own room guess to update that memory in turn --
brain/navigate.py and brain/agent.py:MissionAgent.step() are the client
side of this loop.

`target_reached` is the mission's termination signal, and is deliberately
NOT derived from `action`. "STOP" is what the model returns for a blocked
path, and is also this route's parse-failure default, so treating it as
success would end a mission on an obstacle or a malformed response. It is
coerced strictly (`is True`) and forced false when `target_visible` is
false: over-running by a few steps is cheap, stopping short is not.

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
import hmac
import logging
import os

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware

import vision_core
from vision_core import (
    describe_image_bytes,
    describe_image_bytes_person,
    describe_image_bytes_navigate,
    describe_image_bytes_guidance,
    NAVIGATE_MODEL_CHOICES,
    NAVIGATE_MODEL_ID,
    NAVIGATE_PROMPT_VARIANTS,
    DEFAULT_PROMPT_VARIANT,
)
from rooms_core import identify_room

logger = logging.getLogger("vision_analyze")

# The request gate is the LAMBDA payload, not Bedrock: a sync invocation
# carries at most 6 MB (6,291,456 bytes), and base64 grows bytes by 4/3, so
# 4.5 MB raw is ~6.0 MB encoded plus the JSON around it. Bedrock's 3.75 MB
# (vision_core.MAX_IMAGE_BYTES) is checked by vision_core AFTER any
# conversion: a 4.2 MB HEIC shrinks to fit, a passthrough JPEG over 3.75 MB
# is a 413 there (3.76 round 3, P3).
MAX_REQUEST_IMAGE_BYTES = 4_500_000

# 3.76 H2-8: caller text goes into the prompt (the target ~6 times), so it is
# bounded. A target is a few words (brain/navigate.py's MAX_TARGET_CHARS
# mirrors this, so the brain refuses at mission start instead). The brain
# sends searched rooms SORTED, so which 20 survive is arbitrary -- a house
# has fewer; a long room name (model text fed back) is cut, not dropped.
MAX_TARGET_CHARS = 200
MAX_SEARCHED_ROOMS = 20
MAX_ROOM_CHARS = 64

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
        # 3.76 H2-9: constant time, so the comparison leaks no prefix length.
        if not hmac.compare_digest(provided.encode(), expected_secret.encode()):
            raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


async def _decode_image(request: Request) -> tuple:
    """Shared by /analyze and /navigate: auth check, JSON parse, base64
    decode, size check. Returns (image_bytes, media_type, body)."""
    # 3.76 round 4 (G1): the Bedrock retry budget runs from here, so the
    # decode and any conversion count against Lambda's 30 s too.
    vision_core.mark_request_start()
    _check_secret(request)

    try:
        body = await request.json()
        # 3.76 H2-5: a JSON list or string body was a TypeError, i.e. a 500.
        if not isinstance(body, dict):
            raise ValueError("the body must be a JSON object")
        image_b64 = body["image_base64"]
        media_type = body.get("media_type", "image/jpeg")
        if not isinstance(media_type, str):
            raise ValueError("'media_type' must be a string")
    except (KeyError, ValueError) as e:
        raise HTTPException(status_code=400, detail=f"Bad request: {e}")

    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception:
        raise HTTPException(status_code=400, detail="image_base64 did not decode.")

    if len(image_bytes) > MAX_REQUEST_IMAGE_BYTES:
        raise HTTPException(status_code=413,
                            detail=f"Image too large ({MAX_REQUEST_IMAGE_BYTES} byte request limit).")

    return image_bytes, media_type, body


def _target_object(body: dict) -> str:
    target_object = body.get("target_object")
    # Whitespace-only is missing too: a paid call on a blank noun (round 2, S10).
    if not target_object or (isinstance(target_object, str) and not target_object.strip()):
        raise HTTPException(status_code=400, detail="Bad request: 'target_object' is required.")
    if not isinstance(target_object, str) or len(target_object) > MAX_TARGET_CHARS:
        raise HTTPException(
            status_code=400,
            detail=f"Bad request: 'target_object' must be a string of at most {MAX_TARGET_CHARS} characters.",
        )
    return target_object


def _vision_call(what: str, fn, *args, **kwargs):
    """One model call with the routes' shared error mapping (round 3, P5).

    An `ImageRejected` is the caller's image (400/413). EVERYTHING else is a
    502, Bedrock's own refusals included: a ValidationException is also what
    a deployed-wrong model id or a model not enabled in this region raise,
    and as a 4xx those would end an arrived mission `failed` without 3.47's
    late re-ask. Known limit: a malformed frame Bedrock rejects therefore
    reads as unreachable too (3.76 round 3, P1).
    """
    try:
        return fn(*args, **kwargs)
    except vision_core.ImageRejected as e:  # read at call time: one class
        raise HTTPException(status_code=e.status, detail=str(e))
    except Exception as e:
        logger.exception("Vision API call failed")
        raise HTTPException(status_code=502, detail=f"Vision {what} failed: {e}")


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
        # env_label: which deployment this is (3.64 B1). Behind CloudFront
        # "/health" routes here, so this is what the twin's and the console's
        # environment banners read. A label, not a credential; empty in prod.
        return {"status": "ok", "env_label": os.environ.get("ENV_LABEL", "").strip()}

    @app.post("/analyze")
    async def analyze(request: Request):
        image_bytes, media_type, _body = await _decode_image(request)

        scene = _vision_call("analysis", describe_image_bytes, image_bytes, media_type)

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

        return _vision_call("description", describe_image_bytes_person, image_bytes, media_type)

    @app.get("/navigate/models")
    def navigate_models():
        """The model A/B set /navigate will accept, so a client (the twin's
        model picker) never has to hardcode ids that drift as this service's
        allow-list changes. `default` is what an omitted model_id resolves
        to -- see vision_core.NAVIGATE_MODEL_ID."""
        return {
            "default": NAVIGATE_MODEL_ID,
            "models": [{"id": mid, "label": label} for mid, label in NAVIGATE_MODEL_CHOICES.items()],
            # The prompt is a lever at least as strong as the model, and is
            # published alongside it so one fetch tells a client both axes.
            "default_prompt": DEFAULT_PROMPT_VARIANT,
            "prompts": sorted(NAVIGATE_PROMPT_VARIANTS),
        }

    @app.post("/navigate")
    async def navigate(request: Request):
        image_bytes, media_type, body = await _decode_image(request)

        target_object = _target_object(body)

        searched_rooms = body.get("searched_rooms") or []
        if not isinstance(searched_rooms, list):
            searched_rooms = []
        searched_rooms = [r.strip()[:MAX_ROOM_CHARS] for r in searched_rooms
                          if isinstance(r, str) and r.strip()]
        searched_rooms = list(dict.fromkeys(searched_rooms))[:MAX_SEARCHED_ROOMS]

        model_id = body.get("model_id")
        # isinstance first: a list or dict is unhashable, a TypeError (500) in
        # the `in` test (3.76 round 2, S2).
        if model_id is not None and (not isinstance(model_id, str)
                                     or model_id not in NAVIGATE_MODEL_CHOICES):
            raise HTTPException(
                status_code=400,
                detail=f"Bad request: 'model_id' must be one of {sorted(NAVIGATE_MODEL_CHOICES)}.",
            )

        prompt_variant = body.get("prompt_variant")
        if prompt_variant is not None and (not isinstance(prompt_variant, str)
                                           or prompt_variant not in NAVIGATE_PROMPT_VARIANTS):
            raise HTTPException(
                status_code=400,
                detail=f"Bad request: 'prompt_variant' must be one of {sorted(NAVIGATE_PROMPT_VARIANTS)}.",
            )

        decision = _vision_call(
            "navigation", describe_image_bytes_navigate,
            image_bytes, target_object, media_type, searched_rooms=searched_rooms,
            model_id=model_id, prompt_variant=prompt_variant,
        )

        return decision

    @app.post("/guidance")
    async def guidance(request: Request):
        image_bytes, media_type, body = await _decode_image(request)

        target_object = _target_object(body)

        result = _vision_call("guidance", describe_image_bytes_guidance,
                              image_bytes, target_object, media_type)

        return result

    return app


app = create_app()
