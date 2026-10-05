"""
factory.py

Single place that decides which backend implements WorldInterface --
`robot/factory.py`'s sibling, deliberately the same shape.

The mirror is the point. Anyone who has read one of these files can read
the other, and the rule they enforce is identical: `brain/` and
`control/` call `get_world()` and never import a backend, which is what
keeps "swap in a real mapper" a config change rather than a rewrite.

Backends, as they arrive:

    none   -> NullWorld (world/interface.py). No mapper. The CODE default
              when no `world:` block is given; config/robot.yaml ships
              `sim`, and a body with no grid (teleop, replay) wants this.
    sim    -> sim/mock_world.py (N1/N3). The grid world's own walls as an
              occupancy grid, so the twin can draw a map with no hardware.
    ros    -> world/ros_world.py (R5/R6, built; off by default --
              WORLD_MODE=ros). An HTTP client of the SLAM container. The only backend ROS is allowed to reach, and it
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


def get_world(
    config_path: str | Path = _DEFAULT_CONFIG,
    robot=None,
) -> WorldInterface:
    """Pick the world backend.

    `robot` is the already-built `RobotInterface`, passed through opaquely
    by the caller. Only the `sim` branch looks inside it, and the looking
    happens HERE rather than in `robot/server.py` on purpose: a factory is
    the one layer that is allowed to know what a backend is -- that is the
    whole job, and `robot/factory.py` imports `MockRobot` for the same
    reason. The server hands over whatever `get_robot()` gave it and stays
    ignorant, which is what CLAUDE.md section 2 requires of it.

    Why it needs the robot at all: in the sim the map and the body are two
    views of ONE `GridWorld`. A world model that built its own would be a
    correct map of a different house, with the robot's pose reported
    against a layout it is not standing in -- and it would look right,
    because both houses have the same walls.
    """
    config = load_config(config_path)
    # WORLD_MODE overrides the yaml, the same pattern ROBOT_MODE already
    # uses in robot/factory.py and for the same reason: one generic image,
    # per-environment mode from the task's env vars.
    mode = os.environ.get("WORLD_MODE") or (config.get("world") or {}).get("mode", "none")

    if mode == "none":
        return NullWorld()

    if mode == "sim":
        # N1. MockWorld discovers an occupancy grid by casting a
        # 360-degree ring over the same walls sim/renderer.py draws, which
        # is how the map becomes watchable in the twin before any hardware
        # exists.
        from sim.mock_world import MockWorld

        grid = getattr(robot, "world", None)
        if grid is None:
            # `mode: teleop` with `world.mode: sim` lands here, and it
            # should: a phone on a wheeled rig has no grid world to map.
            # Refusing by name beats returning a map of a house the robot
            # is not in, which would look entirely plausible.
            raise ValueError(
                "world mode 'sim' needs the grid-world robot, and this "
                "robot has no GridWorld. The world model and the body "
                "must share ONE grid. Either set `mode: sim`, or set "
                "WORLD_MODE=none (the honest setting for a teleop or "
                "hardware robot with no mapper -- both /world routes "
                "then answer `usable: false`)."
            )
        if callable(getattr(grid, "get_truth", None)):
            raise ValueError(
                "world mode 'sim' maps the house by casting over its grid, and "
                "this robot's house is in another process (SIM_BODY_URL, "
                "PLAN-ros-alignment.md 3.36). Use WORLD_MODE=ros, or unset "
                "SIM_BODY_URL to keep the body in this process.")
        return MockWorld(grid)

    if mode == "ros":
        # R5 (PLAN-ros-alignment.md 3.14; PLAN-mapping.md's N6): the map and
        # the pose from slam_toolbox, through the ROS container's bridge.
        # In the sim, MockWorld over the same grid supplies the TRUTH, so the
        # estimate can be measured against it; on hardware there is none,
        # and get_truth() says so.
        from world.ros_world import RosWorld

        grid = getattr(robot, "world", None)
        truth = None
        if grid is not None and callable(getattr(grid, "get_truth", None)):
            # 3.36: the house is in sim/body_server.py's process, which
            # serves the truth itself (sim/body_client.py's RemoteGrid).
            truth = grid
        elif grid is not None:
            from sim.mock_world import MockWorld
            truth = MockWorld(grid)
        url = (os.environ.get("ROS_BRIDGE_URL")
               or (config.get("world") or {}).get("bridge_url")
               or "http://127.0.0.1:8090")
        return RosWorld(url, secret=os.environ.get("APP_SHARED_SECRET", ""), truth=truth)

    raise ValueError(f"Unknown world mode in config: {mode!r}")
