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

Per-route models: /analyze feeds a photo a person takes to check a room
by hand (the Camera tab) and stays on the conservative Sonnet 4.5
default. /guidance is on Amazon Nova Lite, measured (real Bedrock calls,
real photo) at ~3x Sonnet's latency with matching accuracy for that task.

/navigate is on Claude Opus 4.5 since 2026-08-29, and that choice is the
one thing here backed by a real measurement on real rooms. All 22 frames
of the recorded walk red-backpack-20260829-195904 were replayed through
every invokable vision model on this account -- identical pixels,
identical prompt, only the model varying. Opus was the only candidate that
both made forward progress and stayed obstacle-aware, and the only one
that refused to call a red blanket a red backpack. Nova Lite scored a
perfect FORWARD rate by being blind: on a frame where the couch filled the
lower half it still answered FORWARD, obstacle_ahead=false.

A cautionary note this file earned the hard way: for months the line above
said Nova Lite while cloudformation/service.yaml's NavigateModelId
parameter passed Sonnet 4.5, and the env var wins. Every walk recorded
before 2026-08-29 was therefore produced by a model nobody involved
believed was running. Keep the code default and the template parameter in
step, and treat a walk's own recorded model_id (echoed on every /navigate
reply since the model picker shipped) as the only trustworthy answer to
"what produced this".

All routes stay independently overridable via env var without a code
change, for exactly this kind of per-route tuning; /navigate additionally
accepts a per-request model_id from the allow-list below.
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
ANALYZE_MODEL_ID = os.environ.get("BEDROCK_ANALYZE_MODEL_ID", MODEL_ID)
# Keep this in step with cloudformation/service.yaml's NavigateModelId
# parameter. They disagreed for a while -- this said Nova Lite, the template
# said Sonnet 4.5, and the env var silently won -- which meant a week of
# recorded walks were attributed to the wrong model. If you change one,
# change both.
NAVIGATE_MODEL_ID = os.environ.get(
    "BEDROCK_NAVIGATE_MODEL_ID", "us.anthropic.claude-opus-4-5-20251101-v1:0")
GUIDANCE_MODEL_ID = os.environ.get("BEDROCK_GUIDANCE_MODEL_ID", "amazon.nova-lite-v1:0")

# /navigate's model A/B set -- a request may pick one of these by id (app.py
# validates against this exact set before it ever reaches Bedrock, so an
# arbitrary string can't run up the bill or hit a model this account has no
# access to). Each was confirmed with a real `converse` call before being
# listed here, the same rule as NAVIGATE_MODEL_ID's default above --
# "claude-sonnet-5", the Opus 4.6/4.7/4.8 and 5 profiles, GPT-5.6 and
# Grok 4.6 are all in this account's Bedrock catalog
# (list-foundation-models) but return AccessDeniedException on an actual
# call, so they are deliberately absent despite looking available.
#
# The four here were chosen by replaying all 22 frames of the recorded walk
# red-backpack-20260829-195904 through every invokable vision model on the
# account -- identical pixels, identical prompt, only the model varying:
#
#   opus-4.5    the only model that both made forward progress (10/22) and
#               stayed obstacle-aware, and the only one that refused to call
#               a red blanket a red backpack. Best judgement available.
#   sonnet-4.5  what that walk actually ran on. FORWARD on 1/22 frames --
#               it is the over-cautious end of the range, and the model
#               whose behaviour the stall was first observed against, so it
#               stays in as the control.
#   qwen3-vl    the non-Anthropic axis, so a conclusion here is not just
#               "a bigger Claude". Good object ID, obstacle-blind.
#   nova-lite   the cheap floor. Answered FORWARD on 22/22 frames and drove
#               into a couch on a genuine close-obstacle frame -- kept
#               precisely so that a degenerate baseline is visible in the
#               comparison rather than assumed.
#
# Dropped after the same sweep: haiku-4.5 (sonnet's caution, none of opus's
# judgement), llama4-scout (as obstacle-blind as nova, no upside),
# pixtral-large (15 of 22 calls errored, ~42s median latency).
#
# Overridable as a whole via env var for the same reason the per-route
# defaults above are: this list will go stale as Bedrock's catalog changes,
# without a code change to fix it.
_DEFAULT_NAVIGATE_MODEL_CHOICES = {
    "us.anthropic.claude-opus-4-5-20251101-v1:0": "Claude Opus 4.5 (best judgement)",
    "us.anthropic.claude-sonnet-4-5-20250929-v1:0": "Claude Sonnet 4.5 (cautious)",
    "qwen.qwen3-vl-235b-a22b": "Qwen3-VL (non-Anthropic)",
    "amazon.nova-lite-v1:0": "Nova Lite (cheap baseline)",
}


def _load_navigate_model_choices() -> dict:
    configured = os.environ.get("BEDROCK_NAVIGATE_MODEL_CHOICES")
    if not configured:
        return dict(_DEFAULT_NAVIGATE_MODEL_CHOICES)
    return {model_id.strip(): model_id.strip() for model_id in configured.split(",") if model_id.strip()}


NAVIGATE_MODEL_CHOICES = _load_navigate_model_choices()

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
        modelId=ANALYZE_MODEL_ID,
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

Divide the image into three equal vertical thirds: "left", "center" and
"right". These thirds are the frame of reference for both questions 1 and 3,
so use them literally -- an object is in whichever third its centre falls in.

Consider:
1. Is the {target_object} visible in this image? If so, which third is it in?
2. Is there an obstacle directly ahead that would block moving forward?
3. Has the robot ARRIVED at the {target_object}? Arrived means it is directly
   in front of the robot and close enough to touch: it spans roughly the full
   width of the center third, or more. Judge this by how much of the frame it
   fills, not by guessing real-world distance. A {target_object} that is
   clearly visible but still across the room has NOT been reached.
4. What kind of room does this look like -- e.g. "kitchen", "hallway",
   "living room", "bedroom", "bathroom"? Use "unclear" if you can't tell.
5. Given the above, what is the single best next action to get closer to the
   {target_object} while not colliding with anything?{searched_rooms_note}

"action" is only about movement -- it never means "the search is over".
Report arrival in "target_reached" instead, so that a STOP caused by an
obstacle is never confused with a STOP caused by success.

Respond with ONLY a JSON object, no other text, matching this schema:
{{
  "target_visible": true | false,
  "target_direction": "left" | "center" | "right" | "not_visible",
  "target_reached": true | false,
  "obstacle_ahead": true | false,
  "room_guess": "short room-type label, or \\"unclear\\"",
  "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
  "reasoning": "one short sentence explaining the choice"
}}"""

# Appended into the prompt only when the caller has already searched at
# least one room (MissionMemory.searched_rooms, threaded through by
# control/mission_runner.py -- see brain/navigate.py). This is the room-
# level step memory named as the open half of phase S2b in
# AGENT-HARNESS.md section 12: a single photograph has no history of its
# own, so the only way to stop the policy re-searching a room is to tell it,
# in words, what it has already covered.
_SEARCHED_ROOMS_NOTE_TEMPLATE = (
    " You have already searched: {rooms}. If this looks like one of those "
    "rooms and the {target_object} is not visible here, prefer moving toward "
    "unexplored space over lingering in a room already searched."
)

# Note the default action is STOP, which is also a legitimate model answer
# for "blocked". That overloading is exactly why arrival gets its own
# `target_reached` field rather than being inferred from action == "STOP":
# a caller must never confuse a parse failure or an obstacle with success.
_NAVIGATE_EMPTY_SCHEMA = {
    "target_visible": False,
    "target_direction": "not_visible",
    "target_reached": False,
    "obstacle_ahead": False,
    "room_guess": "unclear",
    "action": "STOP",
    "reasoning": "Unable to analyze image.",
}


def describe_image_bytes_navigate(
    image_bytes: bytes,
    target_object: str,
    media_type: str = "image/jpeg",
    searched_rooms: list | None = None,
    model_id: str | None = None,
) -> dict:
    client = _get_client()
    model_id = model_id or NAVIGATE_MODEL_ID

    fmt = _bedrock_image_format(media_type)
    if fmt not in ("gif", "jpeg", "png", "webp"):
        image_bytes = _convert_to_jpeg(image_bytes)
        fmt = "jpeg"

    note = ""
    if searched_rooms:
        note = _SEARCHED_ROOMS_NOTE_TEMPLATE.format(
            rooms=", ".join(searched_rooms), target_object=target_object
        )
    prompt = NAVIGATE_PROMPT_TEMPLATE.format(target_object=target_object, searched_rooms_note=note)

    response = client.converse(
        modelId=model_id,
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
                    {"text": prompt},
                ],
            }
        ],
        inferenceConfig={"maxTokens": 300},
    )

    content_blocks = response["output"]["message"]["content"]
    text = "".join(b["text"] for b in content_blocks if "text" in b)
    decision = _parse_navigate_json(text)
    # Carried on the reply (not just logged) so a recorded walk -- and the
    # admin viewer reading it back later -- can tell which model produced
    # which decision, and at what token cost, without a side-channel.
    decision["model_id"] = model_id
    usage = response.get("usage") or {}
    decision["usage"] = {
        "input_tokens": usage.get("inputTokens"),
        "output_tokens": usage.get("outputTokens"),
    }
    return decision


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
        # Coerce rather than trust: this field terminates missions, and a
        # model returning the string "false" (or anything non-boolean) must
        # not read as truthy to a caller. Reached also implies visible --
        # the robot cannot have arrived at something it cannot see.
        merged["target_reached"] = merged["target_reached"] is True
        if merged["target_reached"] and not merged["target_visible"]:
            merged["target_reached"] = False
        # A non-string (or empty) room_guess is as good as "unclear" -- this
        # feeds straight into MissionMemory's room bookkeeping, which must
        # never mistake a garbage value for a real room label.
        room_guess = merged.get("room_guess")
        merged["room_guess"] = str(room_guess).strip() if isinstance(room_guess, str) and room_guess.strip() else "unclear"
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
        modelId=GUIDANCE_MODEL_ID,
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


DESCRIBE_PROMPT = """You are looking at a photo someone has just taken on their phone, usually of a room or part of one. Describe it back to that person.

Write for the person, not for a robot: they want to know what is in the picture. Do not mention obstacles, clearance, safe directions, or anything about navigating the space.

Respond with ONLY a JSON object, no other text, matching this schema:
{
  "summary": "one or two plain sentences describing the scene, as you would say it out loud",
  "room_type": "short label for the kind of space, e.g. \"kitchen\", \"home office\", \"back garden\"; use \"unclear\" if it is not obvious",
  "objects": [list of the notable things visible, each named specifically -- "red backpack", not "bag"]
}"""

_DESCRIBE_EMPTY_SCHEMA = {
    "summary": "",
    "room_type": "unclear",
    "objects": [],
}


def describe_image_bytes_person(image_bytes: bytes, media_type: str = "image/jpeg") -> dict:
    """Person-facing sibling of describe_image_bytes().

    Same Bedrock call, different prompt and schema. SCENE_PROMPT asks what a
    robot needs (obstacles ahead, free space, doorways, a safest direction);
    this asks what the person holding the phone needs. A separate function
    rather than a flag on describe_image_bytes(), matching how
    describe_image_bytes_navigate()/_guidance() were added -- the robot's
    schema is consumed elsewhere and must not shift underneath it.
    """
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
                    {"text": DESCRIBE_PROMPT},
                ],
            }
        ],
        inferenceConfig={"maxTokens": 500},
    )

    content_blocks = response["output"]["message"]["content"]
    text = "".join(b["text"] for b in content_blocks if "text" in b)
    return _parse_describe_json(text)


def _parse_describe_json(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    try:
        parsed = json.loads(text)
        merged = {**_DESCRIBE_EMPTY_SCHEMA, **parsed}
        if not isinstance(merged.get("objects"), list):
            merged["objects"] = []
        return merged
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse VLM describe response as JSON: {text!r}")
        return {**_DESCRIBE_EMPTY_SCHEMA, "_raw": text}


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
