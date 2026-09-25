"""
vision.py

Vision LLM scene understanding (build plan Phase 1 sim).

Several entry points, all returning the same schema so brain/agent.py
never needs to know which one produced a given description:

- describe_image(image_path)
    Sends a real or stock photo to the Vision LLM. This is what Phase 1
    actually tests: can the model reliably interpret basic household
    scenes from static images, before any hardware or live camera exists.

- describe_frame(frame) / describe_base64(data, media_type)
    The same call for pixels already in memory -- a get_camera_frame()
    result, from any backend that has an image. Added by phase S2, which
    is when the simulator gained pixels of its own; before that a file
    path was the only way in and the sim had no file to offer.

`describe_grid_frame(frame)` used to live here too -- the free, offline
converter the rule-based policy ran on, reading a cell count and a doorway
flag off the simulator's frame. It was retired with the cell layer
(`PLAN-ros-alignment.md`); the rule-based policy now builds its scene from
the robot's DEPTH GRID (`brain/agent.py`'s `ConstrainedAgent.sensed_scene()`),
which is what a real robot has and works on any backend with a range sensor.
"""

import base64
import json
import logging
from pathlib import Path
from typing import Optional

import anthropic

logger = logging.getLogger("vision")

MODEL = "claude-sonnet-5"

SCENE_PROMPT = """You are viewing the world through a small indoor robot.
Look at the image and describe:
1. obstacles directly ahead
2. approximate free space
3. visible doorways
4. important objects
5. safest direction of travel

Respond with ONLY a JSON object, no other text, matching this schema:
{
  "obstacles_ahead": [list of strings],
  "free_space": "none" | "some" | "clear",
  "doorway_visible": true | false,
  "important_objects": [list of strings],
  "safest_direction": "FORWARD" | "LEFT" | "RIGHT" | "STOP"
}"""

_client: Optional["anthropic.Anthropic"] = None

_MEDIA_TYPES = {
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".png": "image/png",
    ".webp": "image/webp",
}

_EMPTY_SCHEMA = {
    "obstacles_ahead": [],
    "free_space": "unknown",
    "doorway_visible": False,
    "important_objects": [],
    "safest_direction": "STOP",
}


def _get_client() -> "anthropic.Anthropic":
    global _client
    if _client is None:
        _client = anthropic.Anthropic()  # reads ANTHROPIC_API_KEY from env
    return _client


def _image_to_base64(image_path: str) -> tuple[str, str]:
    path = Path(image_path)
    media_type = _MEDIA_TYPES.get(path.suffix.lower(), "image/jpeg")
    data = base64.standard_b64encode(path.read_bytes()).decode("utf-8")
    return data, media_type


def describe_image(image_path: str) -> dict:
    """Send a real or stock photo to the Vision LLM and return a
    structured scene description. This is the Phase 1 sim milestone."""
    data, media_type = _image_to_base64(image_path)
    return describe_base64(data, media_type)


def describe_frame(frame: dict) -> dict:
    """Describe an already-captured frame -- phase S2.

    A `get_camera_frame()` result carries `image_base64`/`media_type`
    directly on every backend that has pixels (`MockRobot` since S2,
    `ReplayRobot`, `TeleopRobot`, and hardware later), so there is nothing
    to read off disk. Before S2 the only way into this module was a file
    path, which is why the sim could not use it at all: the grid world had
    no file and no pixels to write to one.

    A frame with no image gets the empty schema back -- "nothing seen",
    which is the truth about a picture that does not exist -- rather than a
    KeyError. It used to fall back to the grid-fact converter, which no
    longer exists.
    """
    data = frame.get("image_base64")
    if not data:
        return dict(_EMPTY_SCHEMA)
    return describe_base64(data, frame.get("media_type", "image/jpeg"))


def describe_base64(data: str, media_type: str = "image/jpeg") -> dict:
    """The one place that actually calls the model. Split out of
    `describe_image()` so bytes already in memory never need a temp file."""
    client = _get_client()

    response = client.messages.create(
        model=MODEL,
        max_tokens=500,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": media_type,
                            "data": data,
                        },
                    },
                    {"type": "text", "text": SCENE_PROMPT},
                ],
            }
        ],
    )

    text = "".join(block.text for block in response.content if block.type == "text")
    return _parse_scene_json(text)


def _parse_scene_json(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    try:
        parsed = json.loads(text)
        # Fill in any keys the model omitted rather than trusting it blindly.
        return {**_EMPTY_SCHEMA, **parsed}
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse VLM response as JSON: {text!r}")
        return {**_EMPTY_SCHEMA, "_raw": text}
