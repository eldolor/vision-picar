"""
factory.py

Single place that decides which backend implements WorldInterface --
`robot/factory.py`'s sibling, deliberately the same shape.

The mirror is the point. Anyone who has read one of these files can read
the other, and the rule they enforce is identical: `brain/` and
`control/` call `get_world()` and never import a backend, which is what
keeps "swap in a real mapper" a config change rather than a rewrite.

Backends, as they arrive:

    none   -> NullWorld (world/interface.py). No mapper. TODAY'S DEFAULT,
              and the honest description of every deployment that exists
              -- nothing in this project can build a map yet.
    sim    -> sim/mock_world.py (N1/N3). The grid world's own walls as an
              occupancy grid, so the twin can draw a map with no hardware.
    ros    -> world/ros_world.py (N6). An HTTP client of the SLAM
              container. The only backend ROS is allowed to reach, and it
              reaches it over HTTP like everything else -- see
              PLAN-mapping.md's containment rule.
"""

import os
from pathlib import Path

import yaml

from world.interface import NullWorld, WorldInterface

_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def load_config(config_path: str | Path = _DEFAULT_CONFIG) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def get_world(config_path: str | Path = _DEFAULT_CONFIG) -> WorldInterface:
    config = load_config(config_path)
    # WORLD_MODE overrides the yaml, the same pattern ROBOT_MODE already
    # uses in robot/factory.py and for the same reason: one generic image,
    # per-environment mode from the task's env vars.
    mode = os.environ.get("WORLD_MODE") or (config.get("world") or {}).get("mode", "none")

    if mode == "none":
        return NullWorld()

    if mode == "sim":
        # N1/N3. MockWorld builds an occupancy grid by raycasting the same
        # walls sim/renderer.py draws, which is how the map becomes
        # watchable in the twin before any hardware exists.
        raise NotImplementedError(
            "sim/mock_world.py doesn't exist yet -- build it in PLAN-mapping.md "
            "phase N1, matching WorldInterface exactly."
        )

    if mode == "ros":
        # N6. Raises rather than falling back for the same reason
        # robot/factory.py's hardware branch does: a silent downgrade to
        # "no map" would look exactly like a mapper that had not converged,
        # which is the hardest possible thing to debug on hardware day.
        raise NotImplementedError(
            "world/ros_world.py doesn't exist yet -- build it in PLAN-mapping.md "
            "phase N6. It is an HTTP CLIENT of service/slam/; it must not "
            "import rclpy."
        )

    raise ValueError(f"Unknown world mode in config: {mode!r}")
