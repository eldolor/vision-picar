"""
navigate.py

The vision policy's perception step: a camera frame goes to the deployed
vision service's /navigate route, and comes back as one action.

This is the Python side of what has existed only in JavaScript since the
twin's Vision Autopilot was built (`PLAN-sim-hardening.md` Q1 and phase
S2b). It is a `vision_fn` in `MissionRunner`'s sense -- frame in, scene
out -- so the harness around it is unchanged: the per-call timeout and the
consecutive-failure budget already wrap whatever is passed there. **Do not
add retries in here.** The budget is the retry policy, and silently
retrying a failing vision call while the robot keeps moving is exactly the
behaviour `AGENT-HARNESS.md` section 6 exists to prevent.

## The two schemas, and the one interesting mapping

/navigate answers with the robot's own vocabulary:

    {target_visible, target_direction, target_reached, obstacle_ahead,
     action, reasoning}

`brain/agent.py` and `brain/memory.py` consume `brain/vision.py`'s scene
schema. Most of the mapping is mechanical. One decision is not:

**`important_objects` carries the target only when `target_reached` is
true, not when it is merely visible.** `MissionMemory.record_observation()`
treats a match there as "found", and found ends the mission. Ending on
first sight would stop the robot in a doorway across the room from the
backpack and call that success. /navigate has a dedicated arrival signal
precisely so that STOP-because-blocked and STOP-because-arrived stay
distinguishable (see the service's prompt), and Robot view already pauses
on it. This uses the same signal for the same reason.

Visibility is not lost -- it goes into `_navigate` for the log and the
status panel's reasoning line, where it belongs.

## What this requires of a frame

An image. `get_camera_frame()` must return `image_base64` and
`media_type`. `MockRobot` returns a grid description and no pixels, so the
vision policy cannot drive the simulator until phase S2 ports the twin's
raycaster into Python. Today the backends that work with it are
`sim/replay_robot.py` (a recorded walk) and, later, real hardware.
"""

import logging
import os
from typing import Optional

import httpx

logger = logging.getLogger("navigate")

DEFAULT_TIMEOUT_S = 60.0

# /navigate's action vocabulary is already the robot's -- FORWARD, LEFT,
# RIGHT, REVERSE, STOP -- so no translation table is needed. Anything else
# is treated as STOP, matching the service's own defaulting.
ACTIONS = {"FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP"}


class FrameHasNoImage(ValueError):
    """The backend returned a frame with no pixels in it. A vision policy
    has nothing to look at -- see this module's docstring."""


def navigate_scene(
    frame: dict,
    target_object: str,
    vision_url: str,
    secret: Optional[str] = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    client: Optional[httpx.Client] = None,
) -> dict:
    """One vision call. Returns `brain/vision.py`'s scene schema, with the
    raw /navigate response preserved under `_navigate`."""
    image_base64 = frame.get("image_base64")
    if not image_base64:
        raise FrameHasNoImage(
            "This frame carries no image_base64. The vision policy needs pixels; "
            "MockRobot returns a grid description until phase S2 lands. Use "
            "sim/replay_robot.py, or the rule-based policy."
        )

    body = {
        "image_base64": image_base64,
        "media_type": frame.get("media_type", "image/jpeg"),
        "target_object": target_object,
    }
    headers = {"Content-Type": "application/json"}
    if secret:
        headers["x-app-secret"] = secret

    url = vision_url.rstrip("/") + "/navigate"
    http = client or httpx.Client(timeout=timeout_s)
    try:
        response = http.post(url, headers=headers, json=body, timeout=timeout_s)
        if response.status_code != 200:
            raise RuntimeError(f"HTTP {response.status_code}: {response.text[:200]}")
        result = response.json()
    finally:
        if client is None:
            http.close()

    return to_scene(result, target_object)


def to_scene(result: dict, target_object: str) -> dict:
    """Map /navigate's answer onto brain/vision.py's scene schema.

    Pure, so the interesting decision above is testable without a network
    call or an API bill.
    """
    action = result.get("action")
    if action not in ACTIONS:
        action = "STOP"

    reached = result.get("target_reached") is True
    visible = result.get("target_visible") is True
    obstacle = result.get("obstacle_ahead") is True

    return {
        "obstacles_ahead": ["obstacle"] if obstacle else [],
        # The service reports whether the way ahead is blocked, not how far
        # away anything is -- there is no distance in a photograph. Two
        # states, honestly labelled, rather than a made-up number.
        "free_space": "none" if obstacle else "clear",
        "doorway_visible": False,  # /navigate does not report doorways
        "important_objects": [target_object] if reached else [],
        "safest_direction": action,
        "_navigate": {
            "target_visible": visible,
            "target_direction": result.get("target_direction", "not_visible"),
            "target_reached": reached,
            "obstacle_ahead": obstacle,
            "reasoning": result.get("reasoning", ""),
        },
    }


def vision_fn_for(
    target_object: str,
    vision_url: Optional[str] = None,
    secret: Optional[str] = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    client: Optional[httpx.Client] = None,
):
    """Bind a target and an endpoint into the one-argument `vision_fn(frame)`
    the harness expects."""
    url = vision_url or os.environ.get("VISION_URL", "")
    if not url:
        raise ValueError(
            "No vision service URL. Pass vision_url, set brain.vision_url in "
            "config/robot.yaml, or export VISION_URL."
        )
    app_secret = secret if secret is not None else os.environ.get("APP_SHARED_SECRET", "")

    def vision_fn(frame: dict) -> dict:
        return navigate_scene(
            frame, target_object, url, secret=app_secret, timeout_s=timeout_s, client=client
        )

    return vision_fn
