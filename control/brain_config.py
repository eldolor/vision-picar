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
    # Where WORLD state comes from -- `GET /world/pose` and `/world/map`
    # (`PLAN-mapping.md` N1). Empty means "the same place the robot is",
    # which is true today: `robot/server.py` serves both halves, so a brain
    # pointed at a robot needs no second URL.
    #
    # **It is a separate key because it is going to diverge.** The world's
    # routes are the ones `PLAN-ros-alignment.md` R5 moves behind the SLAM
    # bridge: at that point `slam_toolbox` answers the pose and the map from
    # its own container while the body still answers from the robot runtime,
    # and the only change on this side is this string. That is the test of
    # whether the ROS wall was drawn in the right place -- see
    # `control/remote_world.py`, which contains no hint that ROS exists.
    "world_url": "",
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
    # ---- policy: "tiered" (PLAN-onboard-perception.md 4.10, phase P2) ----
    # Which weights the local detector loads, and which CLIP encoder scores
    # the crops. Empty means brain/perceive.py's own defaults
    # (DEFAULT_DETECTOR, yoloe-11s-seg.pt since P23; DEFAULT_CLIP, RN50),
    # which is where the reasoning for each choice is written down --
    # 4.3.1 measured the detector on-chip and 4.9 chose the encoder. These
    # exist so a candidate model can be swapped WITHOUT editing code, which
    # is 4.7's promotion rule in its cheapest form: the name the twin shows
    # changes with it, and that is the experiment loop made watchable (6.3).
    "perception_detector": "",
    "perception_clip_model": "",
    # HISTORY: the CLIP margin used to be the gate (DEFAULT_MATCH_MARGIN,
    # 0.05, found ~2x too high on the 2026-09-07 corpus: true positives
    # banded at +0.016..+0.034). The gate is now the probability below;
    # `perception_match_margin` survives only as an override for sweeps.
    #
    # Exposed rather than lowered, deliberately. The corpus it was measured
    # on is the invalid standing-height one, and the negative column
    # overlaps -- so the number to ship is not yet known, and the way to
    # learn it is to vary this on a rig walk and read the margins off the
    # panel. Changing the default on that evidence would be the
    # NavigateModelId mistake again.
    # The gate: P(target | crop, texts). 0 means brain/perceive.py's
    # DEFAULT_MATCH_PROBABILITY (0.8), measured across two rig walks.
    "perception_match_probability": 0.0,
    # Override: threshold the raw CLIP margin instead. 0 means "use the
    # probability". Kept for sweeping the raw number on a new corpus -- it
    # is NOT comparable across target strings, which is why it stopped
    # being the gate.
    "perception_match_margin": 0.0,
    # Which of 4.2's crop paths a tiered mission takes: "auto" (the target's
    # COCO word decides -- 4.2's own rule), "label_gate" or "low_confidence".
    # The first valid rig walk measured "auto" losing 11 of 18 true positives
    # because YOLO relabels a close-up bottle as a `vase`; see
    # brain/perceive.py's CROP_PATHS note.
    "perception_crop_path": "",
    # 4.2's class-agnostic crop source, unioned with the detector's boxes.
    # Measured on the three-walk corpus: detector alone 86% recall, floor
    # mask alone 59%, BOTH 94% -- it is worse alone and better together.
    # Off by default because it is a third model per frame and 2.9's budget
    # says segmentation must not run at the detector's rate on the real part.
    "perception_floor_mask": False,
    # 6.1's hysteresis, in frames. 1 reproduces the naive trigger count
    # (2.8x); 2 is the measured 4.1x. Not tuning -- see brain/tiered.py.
    "tier_consecutive_frames": 2,
    # How many consecutive `absent` frames before asking the cloud whether
    # the target is even in this room. brain/tiered.py's own note says this
    # number is a guess and is the first thing to tune against a real walk.
    "tier_cold_search_after": 6,
    # Phase C: centimetres of new ground before the cloud looks again.
    # 0 disables, and is the shipped setting until Phase D measures one.
    "tier_cold_search_after_cm": 0.0,
    # Phase A: dispatch the deliberation call rather than blocking on it.
    "tier_async_cloud": True,
    "tier_hold_goal": True,
    "tier_steer_on_sight": True,
    # P25 / P7c item 2. Dead-reckon the bearing to an anchored sighting on
    # frames the detector misses, instead of falling back to a cloud goal
    # several seconds old. **OFF by default**: it changes what the robot
    # does on every blind frame and no walk has yet shown it helps -- the
    # measurement that the PROBLEM is real (median_command_run == 1 on three
    # of six rig walks) is not evidence that this is the cure. Turn it on,
    # walk it, and read walk_eval's median_command_run.
    "tier_hold_bearing": False,
    # Metres of travel after which an anchor is dropped. A monocular
    # sighting carries no range, so what is held is a DIRECTION -- exact
    # under rotation, wrong under translation -- which makes the bound a
    # distance rather than a timeout.
    "tier_hold_bearing_max_m": 1.0,
    # Where to ship one metrics row per mission. Empty disables the
    # shipper entirely, which is what keeps every test and every laptop
    # run from POSTing anywhere.
    "metrics_url": "",
    "metrics_secret": "",
    "tier_spin_guard_after": 8,
    # 2.4's staleness trigger: call out if nothing else has in this many
    # frames. 0 disables the floor. Without it a robot that can see its
    # target continuously stops deliberating entirely -- and arrival is the
    # cloud's call, so the mission never ends. See brain/tiered.py.
    "tier_stale_after": 8,
    # A hard cap on paid deliberation calls per mission, the same shape as
    # Robot view's 120-call cap. 0 means "no cap" -- max_steps still bounds
    # the mission, and under this policy most steps cost nothing, so the
    # step budget is a poor proxy for the bill.
    "tier_max_calls": 0,
    # 1.11a's corroboration bar -- **reported, never enforced.** The local
    # probability at or above which the on-board tier is taken to agree with
    # a sighting the cloud has already claimed. It gates nothing today: the
    # verdict is computed, counted and published so the next rig walks
    # measure the amendment live, and 1.11a asks for two searches on
    # out-of-vocabulary targets before any behaviour changes. Named here
    # rather than compiled in because 1.11a asks for exactly the treatment
    # DEFAULT_MATCH_PROBABILITY got.
    "tier_corroboration_bar": 0.5,
    "max_steps": 120,
    # End a mission `blocked` after this many FORWARDs in a row refused by
    # the safety layer (control/mission_runner.py's BLOCKED). 0 disables.
    # Five because a policy that is merely unlucky gets one or two refusals
    # and then turns; the first watched R1 run spent nineteen on one jamb.
    "stuck_after": 5,
    # The same number as config/robot.yaml's safety.min_distance_cm (20)
    # since S5; this default only applies when the yaml omits the key.
    "min_distance_cm": 20.0,
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
    # 3.46: where a mission's object inventory goes when it ends
    # (control/inventory_store.py). Always a local file under
    # `inventory_dir`; also s3://<inventory_bucket>/<inventory_prefix>/
    # <robot>/ when a bucket is set (decided by the user 2026-10-07). Empty
    # by default so a checkout with no AWS uploads nothing; the bucket name
    # is per deployment, so it arrives as INVENTORY_BUCKET.
    "inventory_dir": "recordings/inventory",
    "inventory_bucket": "",
    "inventory_prefix": "inventory",
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
    # 3.47: set-but-empty means "no bucket" (local only), not "unset":
    # the opt-out is what keeps a list of what is in the house off S3, so
    # it must win over a bucket in config/robot.yaml too.
    if "INVENTORY_BUCKET" in os.environ:
        merged["inventory_bucket"] = os.environ["INVENTORY_BUCKET"].strip()
    if os.environ.get("INVENTORY_DIR"):
        merged["inventory_dir"] = os.environ["INVENTORY_DIR"]
    # Same one-generic-image rule: where to ship metrics is a property of
    # the deployment, not of the config file baked into an artifact.
    if os.environ.get("METRICS_URL"):
        merged["metrics_url"] = os.environ["METRICS_URL"].strip()
    if os.environ.get("METRICS_SECRET"):
        merged["metrics_secret"] = os.environ["METRICS_SECRET"].strip()
    if os.environ.get("RECORDING_PREFIX"):
        merged["recording_prefix"] = os.environ["RECORDING_PREFIX"]

    return merged
