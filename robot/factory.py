"""
factory.py

Single place that decides which backend implements RobotInterface.
brain/ should only ever call get_robot() -- never import sim.mock_robot
or a future robot.hardware_robot directly. This is what makes the
Phase 11 hardware swap-in a config change instead of a code change.
"""

import yaml
from pathlib import Path
from robot.interface import RobotInterface

_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def load_config(config_path: str | Path = _DEFAULT_CONFIG) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def get_robot(config_path: str | Path = _DEFAULT_CONFIG) -> RobotInterface:
    config = load_config(config_path)
    mode = config.get("mode", "sim")

    if mode == "sim":
        from sim.mock_robot import MockRobot
        from sim.maps.starter_house import build_starter_world

        world = build_starter_world()
        return MockRobot(world)

    if mode == "hardware":
        # Added in Phase 11. Until then this raises on purpose --
        # don't silently fall back to sim if hardware mode is requested.
        raise NotImplementedError(
            "robot.hardware_robot doesn't exist yet -- build it in Phase 11 "
            "and import it here, matching RobotInterface exactly."
        )

    raise ValueError(f"Unknown robot mode in config: {mode!r}")
