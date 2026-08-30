"""
factory.py

Single place that decides which backend implements RobotInterface.
brain/ should only ever call get_robot() -- never import sim.mock_robot
or a future robot.hardware_robot directly. This is what makes the
Phase 11 hardware swap-in a config change instead of a code change.
"""

import os
import yaml
from pathlib import Path
from robot.interface import RobotInterface

_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def load_config(config_path: str | Path = _DEFAULT_CONFIG) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def get_robot(config_path: str | Path = _DEFAULT_CONFIG) -> RobotInterface:
    config = load_config(config_path)
    # ROBOT_MODE overrides the yaml, the same pattern control/brain_config.py
    # already uses for ROBOT_URL/VISION_URL: a deployed image stays generic
    # (one ECR image, per-environment mode from the ECS task's env vars)
    # instead of needing a config/robot.yaml baked per deployment target --
    # see cloudformation/teleop-robot.yaml. Unset by default, so local dev
    # and the test suite are unaffected.
    mode = os.environ.get("ROBOT_MODE") or config.get("mode", "sim")

    if mode == "sim":
        from sim.mock_robot import MockRobot
        from sim.maps.starter_house import build_starter_world
        from sim.sensors import DistanceSensorModel

        world = build_starter_world()
        sim_config = config.get("sim", {})
        realtime = bool(sim_config.get("realtime", False))

        # Phase S5. Both defaults (enabled: false, and every value below
        # it) reproduce MockRobot's pre-S5 exact behavior exactly -- see
        # sim/sensors.py's own docstring for what turning this on changes
        # and why it's opt-in.
        noise_config = sim_config.get("sensor_noise", {}) or {}
        sensor = None
        if noise_config.get("enabled", False):
            sensor = DistanceSensorModel(
                min_range_cm=noise_config.get("min_range_cm", 2.0),
                max_range_cm=noise_config.get("max_range_cm", 400.0),
                noise_stddev_cm=noise_config.get("stddev_cm", 0.0),
                dropout_rate=noise_config.get("dropout_rate", 0.0),
                read_latency_s=noise_config.get("read_latency_s", 0.0),
            )

        return MockRobot(world, realtime=realtime, sensor=sensor)

    if mode == "teleop":
        from sim.teleop_robot import TeleopRobot, DEFAULT_STALL_TIMEOUT_S

        stall_timeout_s = config.get("teleop", {}).get(
            "stall_timeout_s", DEFAULT_STALL_TIMEOUT_S
        )
        return TeleopRobot(stall_timeout_s=stall_timeout_s)

    if mode == "hardware":
        # Added in Phase 11. Until then this raises on purpose --
        # don't silently fall back to sim if hardware mode is requested.
        raise NotImplementedError(
            "robot.hardware_robot doesn't exist yet -- build it in Phase 11 "
            "and import it here, matching RobotInterface exactly."
        )

    raise ValueError(f"Unknown robot mode in config: {mode!r}")
