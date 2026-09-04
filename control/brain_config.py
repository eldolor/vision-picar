"""
brain_config.py

Reads the `brain:` block out of config/robot.yaml.

Deliberately not robot/factory.py's load_config(): importing that module
would drag the robot runtime (and, through it, the simulator) into the
brain process. The brain is an HTTP client of the robot and nothing more
-- see PLAN-brain-relocation.md's definition of done, item 3. The two
processes share one config *file* without sharing a code path.
"""

import os
from pathlib import Path

import yaml

_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"

DEFAULTS = {
    # Where the robot runtime is. This single key is the entire difference
    # between "brain on the Pi" and "brain on the MacBook".
    "robot_url": "http://127.0.0.1:8000",
    # Cloud vision service. Unused until Phase S2b ports the vision policy
    # into Python; carried here so the brain has one place to look for it.
    "vision_url": "",
    # Which model answers /navigate for a mission that doesn't name one.
    # Empty means "no preference": the request omits model_id entirely and
    # the vision service applies its own default. This exists for the
    # headless case -- once B5 puts the brain on the Pi, a mission can start
    # with no twin in the loop to pick a model, and pinning one in config is
    # then the only way to say which. A POST /mission/start model_id
    # overrides it. Never guess a Bedrock model id here: the allow-list lives
    # in the vision service (GET /navigate/models) and is validated there.
    "navigate_model_id": "",
    # Which wording of the /navigate prompt a mission runs when it doesn't
    # name one. Same contract as navigate_model_id above, for the other lever:
    # empty means "no preference", the request omits prompt_variant, and the
    # vision service applies its own DEFAULT_PROMPT_VARIANT. Never guess a
    # name here either -- the allow-list lives in the vision service (GET
    # /navigate/models publishes it as `prompts`) and is validated there.
    "navigate_prompt_variant": "",
    "max_steps": 120,
    "min_distance_cm": 30.0,
    "request_timeout_s": 10.0,
    # B3.2 -- the AWS link failsafe.
    "vision_timeout_s": 20.0,
    "max_vision_failures": 3,
    # Replaying a recorded walk (control/walk_replay.py) is a different
    # question from a live mission's vision call, and it wants a different
    # deadline. A mission is impatient on purpose -- there is a robot
    # standing in a room with its motors live, which is the whole reason
    # B3.2's budget above is short. A replay has no robot, and it is the
    # burstiest caller in the project: several frames in flight at once
    # against one vision task, on the slowest model in the allow-list.
    # Sharing the mission's 20s cost one 22-frame comparison 21 of its
    # frames to read timeouts, and the scorer reported a number anyway.
    "replay_timeout_s": 60.0,
    # B3.3 -- the hung-loop failsafe, plus the loop's own pacing.
    "tick_timeout_s": 30.0,
    "tick_interval_s": 0.0,
    # Failsafe drills (control/drills.py) -- fault injection so B3.2 and
    # B3.3 can be demonstrated from the twin rather than only asserted in
    # tests. Their shortened deadlines exist so a demo takes seconds; the
    # guard being exercised is the same one.
    "allow_drills": True,
    # Recording a Robot-view walk to disk, for replay through the harness
    # (sim/replay_robot.py). Same reasoning as allow_drills: useful on a
    # LAN, not something an internet-reachable brain should accept.
    "allow_recording": True,
    "recording_dir": "recordings",
    # Which control/walk_store.py backend holds the walks: "local" (a
    # directory -- what a developer running the servers gets, and what the
    # EFS mount was) or "s3" (a bucket, which needs no VPC and is why the
    # deployed services can leave one). "local" stays the default so a
    # checkout with no AWS at all still records.
    "recording_backend": "local",
    "recording_bucket": "",
    "recording_prefix": "recordings",
    # When this brain can't record locally (allow_recording: false, e.g.
    # teleop-brain has no EFS mount), forward POST /recording/frame to a
    # peer brain that can, instead of just rejecting it. Empty means "just
    # reject with 403" -- the old, still-default behavior. See
    # PLAN-teleop-robot.md's "Recording proxy" section for why this is
    # safe to proxy when mission control is not: record_frame() touches no
    # robot/runner state at all, it is pure storage.
    "recording_proxy_url": "",
    "recording_proxy_secret": "",
    "recording_proxy_timeout_s": 10.0,
    "drill_vision_timeout_s": 2.0,
    "drill_tick_timeout_s": 3.0,
}


def load_brain_config(config_path=None) -> dict:
    path = Path(config_path) if config_path else _DEFAULT_CONFIG
    with open(path) as f:
        config = yaml.safe_load(f) or {}
    brain = config.get("brain") or {}
    unknown = set(brain) - set(DEFAULTS)
    if unknown:
        # Silently ignoring a typo'd key here would mean silently running
        # with the default -- e.g. a robot_url pointing at localhost when
        # the Pi is across the room.
        raise ValueError(f"Unknown keys in config brain block: {sorted(unknown)}")
    merged = {**DEFAULTS, **brain}

    # ROBOT_URL / VISION_URL override the yaml, the same way ALLOWED_ORIGINS
    # overrides robot/server.py's config -- a deployed image stays generic
    # (one ECR image, per-environment values from ECS task env vars) instead
    # of needing a config/robot.yaml baked per deployment target. Unset by
    # default, so local dev and the test suite are unaffected.
    if os.environ.get("ROBOT_URL"):
        merged["robot_url"] = os.environ["ROBOT_URL"]
    if os.environ.get("VISION_URL"):
        merged["vision_url"] = os.environ["VISION_URL"]
    if os.environ.get("NAVIGATE_MODEL_ID"):
        merged["navigate_model_id"] = os.environ["NAVIGATE_MODEL_ID"]
    if os.environ.get("NAVIGATE_PROMPT_VARIANT"):
        merged["navigate_prompt_variant"] = os.environ["NAVIGATE_PROMPT_VARIANT"]
    # Same idea, for a brain deployment whose robot has no EFS-backed
    # recordings volume mounted (cloudformation/teleop-brain.yaml) --
    # without this, a recorded frame would silently land on the container's
    # own ephemeral disk instead of the shared volume admin_server.py reads,
    # rather than failing loudly the way an unmounted volume should.
    if os.environ.get("ALLOW_RECORDING"):
        merged["allow_recording"] = os.environ["ALLOW_RECORDING"].lower() in ("1", "true", "yes")
    # Where to forward a recording frame this brain can't store itself.
    # RECORDING_PROXY_SECRET is that peer's own APP_SHARED_SECRET, not this
    # brain's -- same "each deployment gets its own generated secret"
    # pattern brain_server.py's ROBOT_SHARED_SECRET/VISION_SHARED_SECRET
    # already use for the robot and vision services.
    if os.environ.get("RECORDING_PROXY_URL"):
        merged["recording_proxy_url"] = os.environ["RECORDING_PROXY_URL"]
    if os.environ.get("RECORDING_PROXY_SECRET"):
        merged["recording_proxy_secret"] = os.environ["RECORDING_PROXY_SECRET"]
    # Same one-generic-image rule as ROBOT_URL above: the bucket name is a
    # per-deployment value, so it arrives as a task env var rather than
    # being baked into config/robot.yaml.
    if os.environ.get("RECORDING_BACKEND"):
        merged["recording_backend"] = os.environ["RECORDING_BACKEND"].strip().lower()
    if os.environ.get("RECORDING_BUCKET"):
        merged["recording_bucket"] = os.environ["RECORDING_BUCKET"]
    if os.environ.get("RECORDING_PREFIX"):
        merged["recording_prefix"] = os.environ["RECORDING_PREFIX"]

    return merged
