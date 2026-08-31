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
`media_type`. Since phase S2 every backend does, `MockRobot` included --
`sim/renderer.py` ports the twin's raycaster into Python, so the vision
policy drives the grid-world simulator as well as a recorded walk
(`sim/replay_robot.py`), a live phone walk (`sim/teleop_robot.py`) and,
later, real hardware. Read `sim/renderer.py`'s fidelity note before
treating a sim result as a statement about real rooms.

## Room-level step memory

A photograph carries no room label -- `AGENT-HARNESS.md` section 12's "no
room memory" gap. `vision_fn_for()` closes it without breaking the
`vision_fn(frame) -> scene` single-argument contract every other seam in
the harness relies on: the callable it returns also carries a
`set_searched_rooms(rooms)` attribute, which `control/mission_runner.py`'s
`_guarded_vision()` calls with the live `MissionMemory.searched_rooms`
immediately before every vision call, entirely outside the documented
contract. The rule-based policy's `vision_fn` has no such attribute, so
`getattr(..., "set_searched_rooms", None)` is `None` there and nothing
changes. `to_scene()` reads the service's own `room_guess` back out the
other side; `brain/agent.py:MissionAgent.step()` uses it to backfill
`frame["room"]` when the frame's own room is `"unknown"`, which is what
lets `MissionMemory` track rooms at all under this policy.
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
    searched_rooms: Optional[list] = None,
    model_id: Optional[str] = None,
) -> dict:
    """One vision call. Returns `brain/vision.py`'s scene schema, with the
    raw /navigate response preserved under `_navigate`.

    `model_id` picks which Bedrock model answers, from the allow-list the
    vision service publishes at GET /navigate/models. Omitted means "no
    preference" -- the service's own default -- and is NOT a model name this
    module gets to guess, which is why None is passed through as an absent
    field rather than a default string."""
    image_base64 = frame.get("image_base64")
    if not image_base64:
        raise FrameHasNoImage(
            "This frame carries no image_base64. The vision policy needs "
            "pixels, and every backend supplies them since phase S2 -- so a "
            "frame without them is either a MockRobot built with "
            "render=False (the free/offline path) or a backend that is not "
            "honouring RobotInterface.get_camera_frame()'s contract."
        )

    body = {
        "image_base64": image_base64,
        "media_type": frame.get("media_type", "image/jpeg"),
        "target_object": target_object,
    }
    if searched_rooms:
        body["searched_rooms"] = list(searched_rooms)
    if model_id:
        body["model_id"] = model_id
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


# Mirrors service/vision_analyze/vision_core.py's allow-list. Duplicated
# for the same reason the prompt is (CLAUDE.md section 6): this module must
# not import from the service, and a value the service stops sending should
# read as "unknown" here rather than pass through unchecked.
DISTANCE_ESTIMATES = ("within_one_step", "a_few_steps", "far", "unknown")


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
    room_guess = result.get("room_guess")
    if not isinstance(room_guess, str) or not room_guess.strip():
        room_guess = "unclear"

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
            "room_guess": room_guess,
            # Ordinal proximity, present only under the
            # "default-with-distance" prompt variant. Anything the service
            # did not vouch for arrives as "unknown", and "unknown" must
            # never be acted on -- see brain/agent.py's proximity veto.
            "distance_estimate": (
                result.get("distance_estimate")
                if result.get("distance_estimate") in DISTANCE_ESTIMATES
                else "unknown"
            ),
            "reasoning": result.get("reasoning", ""),
        },
    }


def vision_fn_for(
    target_object: str,
    vision_url: Optional[str] = None,
    secret: Optional[str] = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    client: Optional[httpx.Client] = None,
    model_id: Optional[str] = None,
):
    """Bind a target and an endpoint into the one-argument `vision_fn(frame)`
    the harness expects.

    `model_id` is bound here, in the factory, exactly like `vision_url` and
    `secret` -- so a mission can choose its model without the
    `vision_fn(frame) -> scene` contract growing a second argument. Nothing
    that only knows that contract (MissionRunner, the agents) needs to know
    a model was chosen at all.

    The returned callable also carries a `set_searched_rooms(rooms)`
    attribute -- not part of the `vision_fn(frame) -> scene` contract, so
    nothing that only expects that contract needs to know about it.
    `control/mission_runner.py`'s `_guarded_vision()` calls it with
    `MissionMemory.searched_rooms` before every vision call when present;
    see this module's docstring, "Room-level step memory"."""
    url = vision_url or os.environ.get("VISION_URL", "")
    if not url:
        raise ValueError(
            "No vision service URL. Pass vision_url, set brain.vision_url in "
            "config/robot.yaml, or export VISION_URL."
        )
    app_secret = secret if secret is not None else os.environ.get("APP_SHARED_SECRET", "")
    state = {"searched_rooms": []}

    def vision_fn(frame: dict) -> dict:
        return navigate_scene(
            frame, target_object, url, secret=app_secret, timeout_s=timeout_s, client=client,
            searched_rooms=state["searched_rooms"], model_id=model_id,
        )

    def set_searched_rooms(rooms) -> None:
        state["searched_rooms"] = list(rooms)

    vision_fn.set_searched_rooms = set_searched_rooms
    return vision_fn
