"""
vision_core.py

Lambda-local copy of the image-analysis logic from brain/vision.py,
adapted to accept image bytes directly (a photo taken in the browser)
instead of a file path. Kept deliberately small and dependency-light so
the Lambda deployment package doesn't need the whole repo bundled in.

If you change the prompt or output schema in brain/vision.py, mirror the
change here -- these are meant to stay in sync but are packaged
separately for deployment simplicity.
"""

import base64
import json
import logging

import anthropic

logger = logging.getLogger()

MODEL = "claude-sonnet-5"

SCENE_PROMPT = """You are viewing a photo of a room, taken by the owner of a small indoor robot to help it understand the space.
Describe:
1. obstacles directly ahead
2. approximate free space
3. visible doorways
4. important objects (name each one specifically, e.g. "red backpack" not just "bag")
5. safest direction of travel

Respond with ONLY a JSON object, no other text, matching this schema:
{
  "obstacles_ahead": [list of strings],
  "free_space": "none" | "some" | "clear",
  "doorway_visible": true | false,
  "important_objects": [list of strings],
  "safest_direction": "FORWARD" | "LEFT" | "RIGHT" | "STOP"
}"""

_EMPTY_SCHEMA = {
    "obstacles_ahead": [],
    "free_space": "unknown",
    "doorway_visible": False,
    "important_objects": [],
    "safest_direction": "STOP",
}

_client = None


def _get_client():
    global _client
    if _client is None:
        _client = anthropic.Anthropic()  # reads ANTHROPIC_API_KEY from Lambda env
    return _client


def describe_image_bytes(image_bytes: bytes, media_type: str = "image/jpeg") -> dict:
    data = base64.standard_b64encode(image_bytes).decode("utf-8")
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
                        "source": {"type": "base64", "media_type": media_type, "data": data},
                    },
                    {"type": "text", "text": SCENE_PROMPT},
                ],
            }
        ],
    )

    text = "".join(b.text for b in response.content if b.type == "text")
    return _parse_scene_json(text)


def _parse_scene_json(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    try:
        parsed = json.loads(text)
        return {**_EMPTY_SCHEMA, **parsed}
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse VLM response as JSON: {text!r}")
        return {**_EMPTY_SCHEMA, "_raw": text}
