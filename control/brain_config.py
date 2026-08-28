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
    "max_steps": 120,
    "min_distance_cm": 30.0,
    "request_timeout_s": 10.0,
    # B3.2 -- the AWS link failsafe.
    "vision_timeout_s": 20.0,
    "max_vision_failures": 3,
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

    return merged
