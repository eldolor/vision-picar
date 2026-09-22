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
# Three more were added on 2026-09-21, when access to them landed on this
# account: Claude Fable 5.1, Claude Opus 5 and GPT-6 Astra. The paragraph
# above still names the Opus 4.6/4.7/4.8-and-5 profiles and GPT-5.6 as
# catalog-visible but AccessDenied -- that was true when it was written and
# is no longer true of the 5-series. Each of the three was confirmed the
# same way every other entry here was, with a real Converse call carrying a
# real walk frame through describe_image_bytes_navigate() (frame 0010 of
# blue-bottle-20260907-142454): all three returned parseable navigate JSON
# inside the 300-token cap, at 2.7s / 4.9s / 5.9s.
#
# They are NOT ranked here, and the default below is deliberately unchanged.
# Nothing has replayed a walk through them yet, so any ordering claim would
# be the NavigateModelId mistake again -- promoted on a guess, then
# believed. control/walk_replay.py is the instrument; run it before moving
# the default.
#
# Fable 5.1 is reachable from this service's own region only because it is
# PINNED to another one -- see MODEL_REGIONS below, which carries the
# measurement. Nothing about that is visible from here, and nothing here
# should have to know it; the pin is the reason this entry can sit in the
# list beside the others rather than carrying a caveat.
#
# Overridable as a whole via env var for the same reason the per-route
# defaults above are: this list will go stale as Bedrock's catalog changes,
# without a code change to fix it.
_DEFAULT_NAVIGATE_MODEL_CHOICES = {
    "us.anthropic.claude-opus-4-5-20251101-v1:0": "Claude Opus 4.5 (best judgement)",
    "us.anthropic.claude-fable-5-1": "Claude Fable 5.1 (newest, unmeasured)",
    "us.anthropic.claude-opus-5": "Claude Opus 5 (newest Opus, unmeasured)",
    "us.openai.gpt-6-astra": "GPT-6 Astra (OpenAI, unmeasured)",
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

# A model that can only be invoked from a region other than the one this
# service runs in. Measured, not assumed: on 2026-09-21 Claude Fable 5.1
# answered a real navigate call from us-east-1 and was refused from BOTH
# us-east-2 (where this service is deployed) and us-west-2 with
# "data retention mode 'default' is not available for this model" -- a
# Bedrock ValidationException, on the us. and global. profiles alike. The
# two regions' inference-profile fan-out is identical and so is the
# account's get-use-case-for-model-access form, so this is AWS-side
# per-region enablement and there is nothing in this account to toggle.
#
# The fix is just an endpoint: bedrock-runtime is a regional API, and the
# execution role's Bedrock policy is Resource "*" in BOTH deployments of
# this service (serverless.yaml's VisionRole, which is what is live, and
# service.yaml's ECS task role), so a caller in us-east-2 may invoke
# us-east-1 directly with no IAM change. The cost is one cross-region hop
# on the models listed here and nothing at all on the rest -- which is why
# this is a per-model pin and not a service-wide region change. Everything
# unlisted keeps using the ambient region.
#
# Overridable, and EXPECTED to be emptied: the day Fable 5.1 is enabled in
# us-east-2 this pin buys a slower call and nothing else. Set
# BEDROCK_MODEL_REGIONS to "" to drop every pin, or to a comma-separated
# list of model_id=region pairs to replace them.
_DEFAULT_MODEL_REGIONS = {
    "us.anthropic.claude-fable-5-1": "us-east-1",
    "global.anthropic.claude-fable-5-1": "us-east-1",
}


def _load_model_regions() -> dict:
    configured = os.environ.get("BEDROCK_MODEL_REGIONS")
    if configured is None:
        return dict(_DEFAULT_MODEL_REGIONS)
    pins = {}
    for pair in configured.split(","):
        if not pair.strip():
            continue
        model_id, _, region = pair.partition("=")
        if region.strip():
            pins[model_id.strip()] = region.strip()
    return pins


MODEL_REGIONS = _load_model_regions()

# Keyed by region so a pinned model does not evict the ambient client on
# every other call -- the previous single global was rebuilt each time the
# region changed, which under alternating traffic meant a new boto3 client
# (and a new connection pool) per request.
_clients = {}


def _get_client(model_id: str | None = None):
    region = MODEL_REGIONS.get(model_id) if model_id else None
    if region not in _clients:
        _clients[region] = boto3.client("bedrock-runtime", region_name=region)
    return _clients[region]


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
    client = _get_client(ANALYZE_MODEL_ID)

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



# The prompt is a lever at least as strong as the model, and until now it was
# the only one that could not be varied without a redeploy. The stall that
# started this whole investigation was a WORDING problem: "is there an
# obstacle directly ahead" got read as "is there furniture anywhere in front
# of me", which stays true from every angle and every distance, so FORWARD
# could never come back once a room had furniture in it.
#
# Variants are named and served from here for the same reason models are:
# one allow-list, server side, echoed back on every reply so a recorded walk
# says which prompt produced it.
# The default, plus one extra question and one extra field: an ORDINAL
# proximity judgement. Built by string surgery on the default rather than
# written out, so the two cannot drift in the five questions they share.
#
# **Why ordinal and not centimetres.** A single monocular frame cannot give
# metric depth -- a real sofa and a doll's sofa are identical up to scale --
# so asking for a number invites a confident guess with no error bar. "How
# many moves of clearance" is a question about this image at this framing,
# which is the kind the model can actually answer.
#
# **Why it is a variant and not the default.** The 3x3 matrix in
# CLAUDE.md measured the default's wording; changing it would invalidate
# those numbers and, worse, change production's behaviour the moment this
# file is deployed there. Kept separate so the existing replay harness can
# A/B the two over the same frames, which is the tool this repo already has
# for exactly this question.
# **Measured 2026-08-31, on 80 frames from five recorded walks (two
# targets, 100% coverage), replayed through this variant. The field is
# informative but badly calibrated, and must not drive a veto as it
# stands:**
#
#   within_one_step 48/80 (60%)   a_few_steps 28/80   far 4/80 (5%)
#
# Sixty percent of frames from someone walking across a house say one
# forward move would hit something, and "far" is essentially never used.
# That is not what a walk looks like. It is the same over-reading already
# recorded for obstacle_ahead -- "is there an obstacle directly ahead"
# heard as "is there furniture anywhere in front of me" -- and an indoor
# room always has *something* within a step or two if walls and furniture
# at the periphery count.
#
# The skew is NOT the target being counted as an obstacle: on the 56
# frames where the target was not visible at all it holds at 55%. It
# agrees with obstacle_ahead on 85% of frames, so the two are consistent
# with each other and wrong together rather than independently noisy.
#
# **Consequence: wiring the veto to this would block roughly three of
# every five FORWARDs -- reproducing the never-FORWARD stall the 3x3
# matrix found for Sonnet.** brain/agent.py's veto stays off by default,
# and this is the evidence for that default.
#
# Not fixed by rewording, on this repo's own experience: the 3x3 matrix
# moved between degenerate modes rather than out of them. The next attempt
# should change the SHAPE of the question -- ask what is in the center
# third and in the path, not what is nearest anywhere in frame.
_DISTANCE_QUESTION = """
6. How close is the nearest thing the robot would collide with if it moved
   forward from here? Answer in ROBOT MOVES, not distance: "within_one_step"
   if a wall, furniture, door or person is close enough that one forward move
   would reach it; "a_few_steps" if there is clear floor for two or three
   moves before anything is in the way; "far" if the path ahead is open well
   beyond that. The {target_object} is the ONE thing that does not count --
   reaching it is the goal, not a collision, so judge the space around it.
"""

NAVIGATE_PROMPT_WITH_DISTANCE = (
    NAVIGATE_PROMPT_TEMPLATE
    .replace(
        '{searched_rooms_note}\n',
        "{searched_rooms_note}" + _DISTANCE_QUESTION,
    )
    .replace(
        '  "reasoning": "one short sentence explaining the choice"',
        '  "distance_estimate": "within_one_step" | "a_few_steps" | "far",\n'
        '  "reasoning": "one short sentence explaining the choice"',
    )
)

NAVIGATE_PROMPT_VARIANTS = {
    "default": NAVIGATE_PROMPT_TEMPLATE,
    "default-with-distance": NAVIGATE_PROMPT_WITH_DISTANCE,
}

# Differs from the default in questions 2 and 5 only. Question 2 asks about
# the NEXT STEP rather than about the room in general, and question 5 says
# outright that closing distance over open floor is progress -- the two
# changes aimed squarely at the observed failure. Kept as a variant rather
# than made the default because it has never been measured over a whole
# recorded walk; that is what the picker and replay are for.
NAVIGATE_PROMPT_VARIANTS["next-step-obstacle"] = """You are the camera of a small indoor robot searching for a {target_object}.
Look at this image and decide the robot's single next move.

Divide the image into three equal vertical thirds: "left", "center" and
"right". These thirds are the frame of reference for both questions 1 and 3,
so use them literally -- an object is in whichever third its centre falls in.

Consider:
1. Is the {target_object} visible in this image? If so, which third is it in?
2. Is there an obstacle close enough to block the robot's NEXT SINGLE STEP?
   The robot moves about 30cm per step. Answer true ONLY if something is
   within roughly one step -- that is, it fills the bottom portion of the
   frame and there is no clear floor between the camera and it. Furniture
   further away across open floor is NOT an obstacle for this step: if you
   can see clear floor immediately in front of the robot, answer false, even
   if there is furniture beyond that floor.
3. Has the robot ARRIVED at the {target_object}? Arrived means it is directly
   in front of the robot and close enough to touch: it spans roughly the full
   width of the center third, or more. Judge this by how much of the frame it
   fills, not by guessing real-world distance. A {target_object} that is
   clearly visible but still across the room has NOT been reached.
4. What kind of room does this look like -- e.g. "kitchen", "hallway",
   "living room", "bedroom", "bathroom"? Use "unclear" if you can't tell.
5. Given the above, what is the single best next action to get closer to the
   {target_object} while not colliding with anything? Prefer FORWARD whenever
   there is clear floor immediately ahead, even if the {target_object} is not
   perfectly centred and even if furniture is visible further away -- closing
   distance over open floor is progress. Turn only when something is within
   one step, or when the {target_object} is far enough to the side that
   forward motion would not close the gap.{searched_rooms_note}

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

# Both wordings above fail, in opposite directions: the default treats any
# furniture in the room as a reason not to move, and "next-step-obstacle"
# fixes that so thoroughly that the model stops seeing walls. This one keeps
# the next-step framing and adds back the one thing it threw away -- a
# surface filling the frame is a stop condition, whatever the robot is
# hunting for. Named for what it is trying to hold together.
#
# **MEASURED 2026-08-30, and it does not hold them together.** All 22 frames
# of red-backpack-20260829-195904, every model x every variant, full
# coverage (see CLAUDE.md's Stage 0 notes for the table). FORWARD rate:
#
#                        default   next-step-obstacle   next-step-and-walls
#   Claude Opus 4.5        0.591          0.318                0.455
#   Claude Sonnet 4.5      0.000          1.000                0.955
#   Qwen3-VL               0.773          1.000                1.000
#
# The wall clause buys one frame in 22 on Sonnet and nothing at all on Qwen:
# both still answer FORWARD to essentially everything, including the frames
# the collision check flags. It is a rounding error away from the failure it
# was written to correct, and should not be promoted to default. Keep it
# served -- a negative result is only durable while the thing that produced
# it can still be re-run -- but the next attempt should change the shape of
# the question, not its wording. Note what the table also says: `default` is
# the only column that avoids the always-FORWARD mode on all three models,
# and it is the current default for that reason.
NAVIGATE_PROMPT_VARIANTS["next-step-and-walls"] = """You are the camera of a small indoor robot searching for a {target_object}.
Look at this image and decide the robot's single next move.

Divide the image into three equal vertical thirds: "left", "center" and
"right". These thirds are the frame of reference for both questions 1 and 3,
so use them literally -- an object is in whichever third its centre falls in.

Consider:
1. Is the {target_object} visible in this image? If so, which third is it in?
2. Is there an obstacle close enough to block the robot's NEXT SINGLE STEP?
   The robot moves about 30cm per step. Two rules, and they matter equally:
   - Answer FALSE when there is clear floor immediately in front of the
     robot, even if there is furniture further away across that floor.
     Furniture on the far side of an open room does not block this step.
   - Answer TRUE when something is right in front of the camera -- in
     particular when a flat surface such as a wall, a door or the side of a
     sofa fills most of the frame and you cannot see floor between the camera
     and it. A featureless close-up IS an obstacle: it means the robot is
     already up against something.
3. Has the robot ARRIVED at the {target_object}? Arrived means it is directly
   in front of the robot and close enough to touch: it spans roughly the full
   width of the center third, or more. Judge this by how much of the frame it
   fills, not by guessing real-world distance. A {target_object} that is
   clearly visible but still across the room has NOT been reached.
4. What kind of room does this look like -- e.g. "kitchen", "hallway",
   "living room", "bedroom", "bathroom"? Use "unclear" if you can't tell.
5. Given the above, what is the single best next action to get closer to the
   {target_object} while not colliding with anything?
   - Prefer FORWARD when there is clear floor ahead, even if the
     {target_object} is off-centre or not visible at all -- closing distance
     over open floor is progress, and turning costs a step without gaining
     any.
   - Never choose FORWARD when question 2 is true. Being unable to see the
     {target_object} is not a reason to drive into a wall; turn instead, so
     the next frame shows somewhere the robot can actually go.{searched_rooms_note}

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

# The fourth attempt at the obstacle question, and the first that changes its
# SHAPE rather than its wording. The three before it -- the default,
# "next-step-obstacle" and "next-step-and-walls" -- all asked some version of
# "is there an obstacle ahead", and all three failed in one of two ways: a
# model that treats any furniture in the room as a reason never to move, or
# one that has stopped seeing walls. `default-with-distance` then asked how
# much clearance there is and skewed the same direction: 60% of frames from a
# walk across a house came back "within_one_step". CLAUDE.md's Stage 0 notes
# and NAVIGATE_PROMPT_WITH_DISTANCE above carry both measurements.
#
# The common failure is one of SCOPE, not of judgement. "Is there an obstacle
# ahead" is a question about the whole frame, and in an indoor room the honest
# answer is almost always yes -- there is always a wall somewhere in front of
# you. The model is not wrong; it is answering a question that cannot
# discriminate. So this variant asks about the one region the answer is
# actually about: the patch of ground the robot would drive over on its next
# step, which is the bottom half of the center third.
#
# Three changes, all serving that:
#
#   1. REGION. Only the bottom half of the center third counts, and the prompt
#      says outright that the side thirds and everything above the halfway
#      line are not in the path however close or large they look. The frame
#      already has thirds -- questions 1 and 3 use them -- so this reuses a
#      frame of reference the model has been given rather than inventing one.
#   2. DESCRIPTIVE, NOT EVALUATIVE. It asks what IS there, not whether that
#      counts as an obstacle. A cautious model can answer "yes, there is an
#      obstacle" truthfully about almost any room; it cannot answer
#      "open_floor" about a wall filling the patch. Naming the ground types
#      first is deliberate for the same reason -- "I see a rug" should resolve
#      towards open, not towards caution.
#   3. obstacle_ahead IS DERIVED FROM IT, not asked separately. The prompt
#      says to set it true when and only when the answer is "blocked", so it
#      is a restatement rather than a second opinion. This is what keeps the
#      variant drop-in comparable: walk_eval's FORWARD rate and collision
#      check read obstacle_ahead, so the new question can be scored in the
#      existing table with no change to the harness.
#
# **ONLY question 2 changes.** Questions 1, 3, 4 and 5 are byte-identical to
# the default, by construction -- this is built by string surgery on
# NAVIGATE_PROMPT_TEMPLATE, the same way NAVIGATE_PROMPT_WITH_DISTANCE is, so
# the two cannot drift. That is the whole experiment: "next-step-obstacle"
# moved question 2 AND question 5 at once and went degenerate, and there was
# no way to tell which half did it. If this variant still comes back
# always-FORWARD with question 5 untouched, the answer is that question 5 is
# the problem, and that is worth knowing.
#
# **MEASURED 2026-09-02, and it does not work. Do not promote it.** Eight
# recorded walks, 96 frames, two targets, replayed against `default` under
# two models -- every cell at coverage 1.0. Frame-weighted FORWARD rate:
#
#                        default   center-third-path
#   Claude Opus 4.5       0.323          0.365
#   Qwen3-VL              0.354          0.573
#
# The collision flag did not move at all: 6 of 8 walks for Opus and 7 of 8
# for Qwen, under both wordings. And the two models moved APART (0.03 apart
# under default, 0.21 apart here), with one Qwen cell tipping into a flat
# degenerate 1.0. Better scoping should converge two competent readers of
# the same patch of floor, not separate them.
#
# The per-frame field says why, and the answer is not what was expected:
#
#   Opus  blocked 66%  open_floor 33%     Qwen  open_floor 94%  blocked 5%
#   agreement on path_ahead 40% (kappa 0.075 -- chance is 35%)
#
# **The instruction was followed exactly.** obstacle_ahead restates
# path_ahead on 99% of Opus frames and 100% of Qwen's, which is what the
# prompt asked for. And the disagreement is perfectly NESTED, not scattered:
# every one of the 32 frames Opus called open_floor, Qwen also called
# open_floor; of the 63 Opus called blocked, Qwen called 58 of them open.
# There is no frame where the two contradict each other's ordering. They
# read the image the same way and cut "blocked" at wildly different
# thresholds. **A threshold has no wording.** That is why four attempts at
# rewording this question have now failed, and why a fifth should not be
# written: the next lever is an anchor or an example, or -- on hardware --
# the ultrasonic sensor, which measures the threshold instead of arguing
# about it.
#
# **READ THIS BEFORE TRUSTING ANY OF THE ABOVE.** While reading the frames
# that produced these numbers it turned out the corpus itself is invalid --
# both targets sit on raised furniture and every frame is shot from standing
# height, so the mission is not reachable by a floor robot and the view is
# not the robot's. See CLAUDE.md's Stage 0 notes. The numbers here are real
# and reproducible, but they measure the wrong task, and this variant has
# not had a fair test.
_CENTER_THIRD_QUESTION = """2. Look ONLY at the bottom half of the center third: the patch of ground the
   robot would drive over on its next single step. Everything else in this
   image -- both side thirds, and everything above the halfway line -- is NOT
   in the robot's path for this step, however close or large it looks. What
   is in that patch?
   - "open_floor": floor, rug, carpet or any other surface the robot could
     roll across. Choose this whenever that patch is ground, even when
     furniture, walls or people are visible elsewhere in the frame.
   - "blocked": a wall, door, item of furniture, step or person occupies
     that patch, so the robot cannot drive through it.
   - "unclear": the patch is too dark, too blurred or too close to identify.
   The {target_object} is the one thing that never counts as "blocked" --
   reaching it is the goal, not a collision, so judge the ground around it.
   Then set "obstacle_ahead" true when and ONLY when this answer is
   "blocked". It is a restatement of this question, not a second opinion
   about the room."""

NAVIGATE_PROMPT_CENTER_THIRD = (
    NAVIGATE_PROMPT_TEMPLATE
    .replace(
        "2. Is there an obstacle directly ahead that would block moving forward?",
        _CENTER_THIRD_QUESTION,
    )
    .replace(
        '  "reasoning": "one short sentence explaining the choice"',
        '  "path_ahead": "open_floor" | "blocked" | "unclear",\n'
        '  "reasoning": "one short sentence explaining the choice"',
    )
)

NAVIGATE_PROMPT_VARIANTS["center-third-path"] = NAVIGATE_PROMPT_CENTER_THIRD

# What is in the robot's path, as opposed to what is nearest anywhere in
# frame. "unclear" is the only value that must never be acted on -- same
# contract as DISTANCE_ESTIMATES above.
PATH_AHEAD_VALUES = ("open_floor", "blocked", "unclear")


# Phase M1 (PLAN-microduck-transplants.md). Every other variant on this page
# is another attempt to word the obstacle question well. This one deletes it.
#
# Microduck splits perception by what a sensor can actually answer -- "camera
# = direction, ToF = distance" -- and this project has measured the same
# thing the hard way. `obstacle_ahead` reads ~100% true on Opus and ~0% on
# Qwen over identical frames; `default-with-distance` says "within one step"
# on 60% of frames, and still 55% on frames with no target in them at all.
# Four wordings produced two never-FORWARD results and two always-FORWARD
# ones. A single monocular frame does not contain metric depth, so no wording
# recovers it.
#
# So: /navigate keeps WHAT and WHICH WAY, and a distance sensor owns HOW FAR.
# `robot/safety.py` already re-reads `get_distance()` before every FORWARD,
# which is the veto this variant hands the job back to.
#
# **Only question 2 is removed.** Questions 1, 3, 4 and 5 are byte-identical
# to the default, by construction (string surgery, pinned by a test), for the
# same reason "center-third-path" is: "next-step-obstacle" moved questions 2
# and 5 together and left no way to attribute the degenerate result. The
# remaining questions keep their original numbers -- 1, 3, 4, 5, with a gap.
# Renumbering them would edit four lines this experiment is trying to hold
# still, and the model is being asked to answer questions, not to audit an
# ordinal sequence.
#
# `obstacle_ahead` is absent from the reply, not false: the schema line is
# removed too, and describe_image_bytes_navigate() strips the field for any
# variant whose template does not ask for it. A caller must be able to tell
# "the model saw no obstacle" from "nobody asked" -- brain/navigate.py's
# to_scene() reports the second as free_space "unknown".
#
# **NOT MEASURED YET.** Replay it over the same frames as the 3x3 matrix
# before believing anything about it -- and read the coverage before the
# score. Note that control/walk_eval.py's collision check will flag such a
# replay, because ReplayRobot has no distance sensor to veto anything. That
# column is the specification for M10's real sensor, not a defect.
_OBSTACLE_QUESTION_LINE = (
    "2. Is there an obstacle directly ahead that would block moving forward?\n"
)
_OBSTACLE_SCHEMA_LINE = '  "obstacle_ahead": true | false,\n'

NAVIGATE_PROMPT_BEARING_ONLY = (
    NAVIGATE_PROMPT_TEMPLATE
    .replace(_OBSTACLE_QUESTION_LINE, "")
    .replace(_OBSTACLE_SCHEMA_LINE, "")
)

NAVIGATE_PROMPT_VARIANTS["bearing-only"] = NAVIGATE_PROMPT_BEARING_ONLY


def variant_asks_obstacle(variant: str) -> bool:
    """Does this prompt variant ask for `obstacle_ahead` at all?

    Derived from the template rather than kept as a second list, so a new
    variant that drops the field cannot forget to register itself here.
    """
    template = NAVIGATE_PROMPT_VARIANTS.get(variant, NAVIGATE_PROMPT_TEMPLATE)
    return '"obstacle_ahead"' in template


DEFAULT_PROMPT_VARIANT = os.environ.get("NAVIGATE_PROMPT_VARIANT", "default")


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
    # Always present, so a client never has to distinguish "the model said
    # nothing" from "this prompt variant does not ask". "unknown" is the
    # only value that must never be acted on.
    "distance_estimate": "unknown",
    # Same contract, for the center-third variant's question 2.
    "path_ahead": "unclear",
}

# Ordinal, deliberately. See NAVIGATE_PROMPT_WITH_DISTANCE for why there is
# no centimetre figure here and why there should not be one.
DISTANCE_ESTIMATES = ("within_one_step", "a_few_steps", "far", "unknown")


def describe_image_bytes_navigate(
    image_bytes: bytes,
    target_object: str,
    media_type: str = "image/jpeg",
    searched_rooms: list | None = None,
    model_id: str | None = None,
    prompt_variant: str | None = None,
) -> dict:
    model_id = model_id or NAVIGATE_MODEL_ID
    client = _get_client(model_id)

    fmt = _bedrock_image_format(media_type)
    if fmt not in ("gif", "jpeg", "png", "webp"):
        image_bytes = _convert_to_jpeg(image_bytes)
        fmt = "jpeg"

    note = ""
    if searched_rooms:
        note = _SEARCHED_ROOMS_NOTE_TEMPLATE.format(
            rooms=", ".join(searched_rooms), target_object=target_object
        )
    variant = prompt_variant or DEFAULT_PROMPT_VARIANT
    template = NAVIGATE_PROMPT_VARIANTS.get(variant, NAVIGATE_PROMPT_TEMPLATE)
    prompt = template.format(target_object=target_object, searched_rooms_note=note)

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
    # A field nobody asked for must not arrive as `false`. _parse_navigate_json
    # merges the empty schema over every reply, so obstacle_ahead would come
    # back False under a variant that never asked -- indistinguishable from a
    # model that looked and saw clear floor. Stripped here, keyed off the
    # template itself (see variant_asks_obstacle), so the absence is the
    # answer. brain/navigate.py maps it to free_space "unknown".
    if not variant_asks_obstacle(variant):
        decision.pop("obstacle_ahead", None)
    # Carried on the reply (not just logged) so a recorded walk -- and the
    # admin viewer reading it back later -- can tell which model produced
    # which decision, and at what token cost, without a side-channel.
    decision["model_id"] = model_id
    # Which wording produced this, carried for the same reason model_id is:
    # a recorded walk has to be able to say what made it.
    decision["prompt_variant"] = variant
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
        # Anything off the allow-list becomes "unknown" rather than passing
        # through. A caller may veto a move on this field, so an
        # unrecognised value has to fail towards "do not act on it" -- never
        # towards a made-up level of confidence.
        if merged.get("distance_estimate") not in DISTANCE_ESTIMATES:
            merged["distance_estimate"] = "unknown"
        # Same treatment, and for the same reason: an unrecognised value has
        # to fail towards "do not act on it". Note obstacle_ahead is NOT
        # recomputed from this -- the prompt asks the model to keep the two
        # in step, and quietly overwriting one with the other would hide
        # exactly the disagreement worth measuring.
        if merged.get("path_ahead") not in PATH_AHEAD_VALUES:
            merged["path_ahead"] = "unclear"
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
    client = _get_client(GUIDANCE_MODEL_ID)

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
    client = _get_client(MODEL_ID)

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
