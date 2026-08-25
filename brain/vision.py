"""
vision.py

Vision LLM scene understanding (build plan Phase 1 sim).

Two entry points, both returning the same schema so brain/agent.py never
needs to know which one produced a given description:

- describe_image(image_path)
    Sends a real or stock photo to the Vision LLM. This is what Phase 1
    actually tests: can the model reliably interpret basic household
    scenes from static images, before any hardware or live camera exists.

- describe_grid_frame(frame)
    Converts grid_world.frame_description() output into the same schema
    WITHOUT calling the LLM -- the grid-world frame is already ground
    truth, so there's nothing for a VLM to infer. Keeps agent-loop tests
    (Phase 2+) fast and free while still exercising the exact same
    downstream interface describe_image() would produce.
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


def describe_grid_frame(frame: dict) -> dict:
    """Convert grid_world.frame_description() output into the same
    schema describe_image() returns -- no LLM call needed."""
    cells = frame["free_space_cells"]
    if cells >= 3:
        free_space = "clear"
    elif cells >= 1:
        free_space = "some"
    else:
        free_space = "none"

    return {
        "obstacles_ahead": [] if free_space != "none" else ["wall"],
        "free_space": free_space,
        "doorway_visible": frame["doorway_ahead"],
        "important_objects": frame["objects_visible"],
        "safest_direction": "FORWARD" if free_space != "none" else "STOP",
    }


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
