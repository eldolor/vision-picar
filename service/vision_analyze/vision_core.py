"""
vision_core.py

ECS-service copy of the image-analysis logic from brain/vision.py,
adapted to accept image bytes directly (a photo taken in the browser)
instead of a file path, and to call the model via Amazon Bedrock's
Converse API instead of the direct Anthropic API -- this service runs
in a private subnet with no internet egress, reaching Bedrock over a
VPC interface endpoint. Auth is via the ECS task role's IAM permissions
(bedrock:InvokeModel / bedrock:Converse), not an API key.

If you change the prompt or output schema in brain/vision.py, mirror the
change here -- these are meant to stay in sync but are packaged
separately for deployment simplicity (same pattern lambda/vision_analyze
used before it was decommissioned).

Model note: "claude-sonnet-5" (the model brain/vision.py and the old
Lambda deployment used via the direct Anthropic API) is not yet enabled
for this account on Bedrock -- verified via `aws bedrock-runtime converse`,
which returned AccessDeniedException for both the base model ID and its
inference profile. Using the cross-region inference profile for Claude
Sonnet 4.5 instead, confirmed working (including image input) via a real
`converse` call before wiring this in.
"""

import io
import json
import logging
import os

import boto3
import pillow_heif
from PIL import Image

pillow_heif.register_heif_opener()

logger = logging.getLogger()

MODEL_ID = os.environ.get("BEDROCK_MODEL_ID", "us.anthropic.claude-sonnet-4-5-20250929-v1:0")

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
        _client = boto3.client("bedrock-runtime")
    return _client


_BEDROCK_FORMAT_ALIASES = {"jpg": "jpeg"}


def _bedrock_image_format(media_type: str) -> str:
    # Bedrock's Converse API wants a short format token from a strict enum
    # (gif, jpeg, png, webp) -- not a MIME type, and NOT every real-world
    # MIME type maps directly onto it: some cameras/browsers report JPEGs
    # as "image/jpg" (non-standard but common), which Bedrock rejects with
    # a ValidationException since only "jpeg" is in its enum. Normalize
    # known aliases rather than passing the raw subtype straight through.
    fmt = media_type.split("/")[-1].lower()
    return _BEDROCK_FORMAT_ALIASES.get(fmt, fmt)


def _convert_to_jpeg(image_bytes: bytes) -> bytes:
    image = Image.open(io.BytesIO(image_bytes))
    buf = io.BytesIO()
    image.convert("RGB").save(buf, format="JPEG")
    return buf.getvalue()


def describe_image_bytes(image_bytes: bytes, media_type: str = "image/jpeg") -> dict:
    client = _get_client()

    fmt = _bedrock_image_format(media_type)
    if fmt not in ("gif", "jpeg", "png", "webp"):
        # Most commonly HEIC/HEIF -- the default photo format on iPhone,
        # which Bedrock's Converse API doesn't accept at all (unlike the
        # jpg/jpeg naming quirk above, this isn't a labeling issue -- the
        # actual bytes need transcoding). pillow-heif registers HEIC/HEIF
        # support into Pillow's Image.open() so this one code path handles
        # any input Pillow understands, not just HEIC specifically.
        image_bytes = _convert_to_jpeg(image_bytes)
        fmt = "jpeg"

    response = client.converse(
        modelId=MODEL_ID,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "image": {
                            "format": fmt,
                            "source": {"bytes": image_bytes},
                        }
                    },
                    {"text": SCENE_PROMPT},
                ],
            }
        ],
        inferenceConfig={"maxTokens": 500},
    )

    content_blocks = response["output"]["message"]["content"]
    text = "".join(b["text"] for b in content_blocks if "text" in b)
    return _parse_scene_json(text)


NAVIGATE_PROMPT_TEMPLATE = """You are the camera of a small indoor robot searching for a {target_object}.
Look at this image and decide the robot's single next move.
Consider:
1. Is the {target_object} visible in this image? If so, roughly which direction is it relative to the center of the frame?
2. Is there an obstacle directly ahead that would block moving forward?
3. Given the above, what is the single best next action to get closer to the {target_object} while not colliding with anything?

Respond with ONLY a JSON object, no other text, matching this schema:
{{
  "target_visible": true | false,
  "target_direction": "left" | "center" | "right" | "not_visible",
  "obstacle_ahead": true | false,
  "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
  "reasoning": "one short sentence explaining the choice"
}}"""

_NAVIGATE_EMPTY_SCHEMA = {
    "target_visible": False,
    "target_direction": "not_visible",
    "obstacle_ahead": False,
    "action": "STOP",
    "reasoning": "Unable to analyze image.",
}


def describe_image_bytes_navigate(image_bytes: bytes, target_object: str, media_type: str = "image/jpeg") -> dict:
    client = _get_client()

    fmt = _bedrock_image_format(media_type)
    if fmt not in ("gif", "jpeg", "png", "webp"):
        image_bytes = _convert_to_jpeg(image_bytes)
        fmt = "jpeg"

    response = client.converse(
        modelId=MODEL_ID,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "image": {
                            "format": fmt,
                            "source": {"bytes": image_bytes},
                        }
                    },
                    {"text": NAVIGATE_PROMPT_TEMPLATE.format(target_object=target_object)},
                ],
            }
        ],
        inferenceConfig={"maxTokens": 300},
    )

    content_blocks = response["output"]["message"]["content"]
    text = "".join(b["text"] for b in content_blocks if "text" in b)
    return _parse_navigate_json(text)


def _parse_navigate_json(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    try:
        parsed = json.loads(text)
        merged = {**_NAVIGATE_EMPTY_SCHEMA, **parsed}
        if merged["action"] not in ("FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP"):
            merged["action"] = "STOP"
        return merged
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse VLM navigate response as JSON: {text!r}")
        return {**_NAVIGATE_EMPTY_SCHEMA, "_raw": text}


GUIDANCE_PROMPT_TEMPLATE = """You are helping someone find a {target_object} using their phone camera.
Look at this photo and determine:
1. Is the {target_object} visible in this image?
2. If visible, where is it positioned horizontally in the frame?
3. How far away does it appear? Judge this by how much of the frame the
   object fills, not a guess at real-world distance:
   - "near": the object fills a large portion of the frame (roughly a
     third of the frame's width or height or more) -- close enough that a
     couple of steps would reach it.
   - "medium": the object is clearly visible and identifiable but still
     small-to-moderate in the frame -- several steps away.
   - "far": the object is visible but small/distant in the frame -- across
     the room or further.
   Be conservative about "near" -- only use it when the object is
   genuinely large/close in frame, not just clearly recognizable. Most
   newly-spotted objects across a room should be "medium" or "far".
4. What direction should the person move or turn to get closer to it?
5. If it is visible, clearly identifiable, and not significantly cut off
   by the edge of the frame, estimate a bounding box around it as
   fractions of the image width/height (0.0 = left/top edge, 1.0 =
   right/bottom edge). Only provide this when you are reasonably
   confident -- if you are unsure of its exact extent, return null
   instead of guessing.

Respond with ONLY a JSON object, no other text, matching this schema:
{{
  "target_visible": true | false,
  "position": "far_left" | "left" | "center" | "right" | "far_right" | "not_visible",
  "proximity": "near" | "medium" | "far" | "unknown",
  "guidance": "short human-readable instruction, e.g. 'Turn right and walk forward'",
  "bounding_box": {{"x_min": 0.0, "y_min": 0.0, "x_max": 1.0, "y_max": 1.0}} | null
}}"""

_GUIDANCE_EMPTY_SCHEMA = {
    "target_visible": False,
    "position": "not_visible",
    "proximity": "unknown",
    "guidance": "Unable to analyze image.",
    "bounding_box": None,
}

_BBOX_KEYS = ("x_min", "y_min", "x_max", "y_max")


def _validate_bounding_box(box) -> dict | None:
    if not isinstance(box, dict):
        return None
    try:
        values = {k: float(box[k]) for k in _BBOX_KEYS}
    except (KeyError, TypeError, ValueError):
        return None
    if not all(0.0 <= v <= 1.0 for v in values.values()):
        return None
    if values["x_min"] >= values["x_max"] or values["y_min"] >= values["y_max"]:
        return None
    return values

_GUIDANCE_POSITIONS = ("far_left", "left", "center", "right", "far_right", "not_visible")
_GUIDANCE_PROXIMITIES = ("near", "medium", "far", "unknown")


def describe_image_bytes_guidance(image_bytes: bytes, target_object: str, media_type: str = "image/jpeg") -> dict:
    client = _get_client()

    fmt = _bedrock_image_format(media_type)
    if fmt not in ("gif", "jpeg", "png", "webp"):
        image_bytes = _convert_to_jpeg(image_bytes)
        fmt = "jpeg"

    response = client.converse(
        modelId=MODEL_ID,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "image": {
                            "format": fmt,
                            "source": {"bytes": image_bytes},
                        }
                    },
                    {"text": GUIDANCE_PROMPT_TEMPLATE.format(target_object=target_object)},
                ],
            }
        ],
        inferenceConfig={"maxTokens": 300},
    )

    content_blocks = response["output"]["message"]["content"]
    text = "".join(b["text"] for b in content_blocks if "text" in b)
    return _parse_guidance_json(text)


def _parse_guidance_json(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    try:
        parsed = json.loads(text)
        merged = {**_GUIDANCE_EMPTY_SCHEMA, **parsed}
        if merged["position"] not in _GUIDANCE_POSITIONS:
            merged["position"] = "not_visible"
        if merged["proximity"] not in _GUIDANCE_PROXIMITIES:
            merged["proximity"] = "unknown"
        merged["bounding_box"] = _validate_bounding_box(merged["bounding_box"])
        return merged
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse VLM guidance response as JSON: {text!r}")
        return {**_GUIDANCE_EMPTY_SCHEMA, "_raw": text}


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
