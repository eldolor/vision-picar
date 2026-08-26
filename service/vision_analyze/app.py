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

import base64
import logging
import os

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware

from vision_core import describe_image_bytes
from rooms_core import identify_room

logger = logging.getLogger("vision_analyze")

MAX_IMAGE_BYTES = 5 * 1024 * 1024  # 5MB, matches Claude's API limit

_DEFAULT_ALLOWED_ORIGINS = ["http://localhost:5173", "http://localhost:8000"]


def _allowed_origins() -> list:
    configured = os.environ.get("ALLOWED_ORIGINS")
    if configured:
        return [o.strip() for o in configured.split(",") if o.strip()]
    return _DEFAULT_ALLOWED_ORIGINS


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
        expected_secret = os.environ.get("APP_SHARED_SECRET")
        if expected_secret:
            provided = request.headers.get("x-app-secret", "")
            if provided != expected_secret:
                raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")

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

        try:
            scene = describe_image_bytes(image_bytes, media_type)
        except Exception as e:
            logger.exception("Vision API call failed")
            raise HTTPException(status_code=502, detail=f"Vision analysis failed: {e}")

        scene["room_guess"] = identify_room(scene.get("important_objects", []))
        return scene

    return app


app = create_app()
