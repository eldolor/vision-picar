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

import json
import logging
import os

import boto3

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


def _bedrock_image_format(media_type: str) -> str:
    # Bedrock's Converse API wants a short format token ("png", "jpeg"),
    # not a MIME type.
    return media_type.split("/")[-1].lower()


def describe_image_bytes(image_bytes: bytes, media_type: str = "image/jpeg") -> dict:
    client = _get_client()

    response = client.converse(
        modelId=MODEL_ID,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "image": {
                            "format": _bedrock_image_format(media_type),
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
