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
    return _with_drive(_backend(config), config)


def _with_drive(robot: RobotInterface, config: dict) -> RobotInterface:
    """R4 (PLAN-ros-alignment.md 3.13): under `drive: ros` every verb is
    executed as velocities through the ROS container, which becomes the one
    writer to the wheels. `direct` (the default) leaves the backend as it is.
    ROBOT_DRIVE overrides the yaml, as ROBOT_MODE does."""
    drive = config.get("drive") or {}
    mode = os.environ.get("ROBOT_DRIVE") or drive.get("mode", "direct")
    if mode == "direct":
        return robot
    if mode == "ros":
        from robot.ros_drive import RosDriveRobot

        url = os.environ.get("ROS_BRIDGE_URL") or drive.get("bridge_url", "http://127.0.0.1:8090")
        return RosDriveRobot(robot, url, secret=os.environ.get("APP_SHARED_SECRET", ""))
    raise ValueError(f"Unknown drive mode in config: {mode!r} (direct | ros)")


def _sim_world():
    """The house SIM_MAP names (the starter house by default), with the
    people and pets SIM_MOVERS names, if any (PLAN-ros-alignment.md 3.30)."""
    from sim.maps import build_movers, build_world

    house = os.environ.get("SIM_MAP") or "starter_house"
    world = build_world(house)
    if os.environ.get("SIM_MOVERS"):
        for mover in build_movers(house, os.environ["SIM_MOVERS"]):
            world.add_mover(mover)
    return world


def _backend(config: dict) -> RobotInterface:
    # ROBOT_MODE overrides the yaml, the same pattern control/brain_config.py
    # already uses for ROBOT_URL/VISION_URL: a deployed image stays generic
    # (one ECR image, per-environment mode from the ECS task's env vars)
    # instead of needing a config/robot.yaml baked per deployment target --
    # see cloudformation/teleop-robot.yaml. Unset by default, so local dev
    # and the test suite are unaffected.
    mode = os.environ.get("ROBOT_MODE") or config.get("mode", "sim")

    if mode == "sim":
        from sim.mock_robot import MockRobot
        from sim.sensors import DistanceSensorModel

        # SIM_MAP picks the house (sim/maps/__init__.py); the starter house
        # is the default and what every existing test was measured on.
        world = _sim_world()
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

        # R5 (PLAN-ros-alignment.md 3.14): encoders that misreport, so
        # odometry drifts for SLAM to correct. Off by default.
        drift = sim_config.get("odom_drift", {}) or {}
        scale = ((float(drift.get("left_scale", 1.0)), float(drift.get("right_scale", 1.0)))
                 if drift.get("enabled", False) else (1.0, 1.0))
        # SIM_ODOM_DRIFT="left,right" overrides the yaml, as ROBOT_DRIVE does.
        if os.environ.get("SIM_ODOM_DRIFT"):
            left, right = os.environ["SIM_ODOM_DRIFT"].split(",")
            scale = (float(left), float(right))
        return MockRobot(world, realtime=realtime, sensor=sensor, encoder_scale=scale)

    if mode == "teleop":
        from sim.teleop_robot import TeleopRobot, DEFAULT_STALL_TIMEOUT_S

        stall_timeout_s = config.get("teleop", {}).get(
            "stall_timeout_s", DEFAULT_STALL_TIMEOUT_S
        )
        return TeleopRobot(stall_timeout_s=stall_timeout_s)

    if mode == "hardware":
        # R7 (PLAN-ros-alignment.md 3.16): the real motors, over the ESP32
        # driver board's serial line. ROBOT_SERIAL names the port (a udev
        # symlink on the car -- HARDWARE-BOM.md 4.2: never /dev/ttyUSB0 by
        # order). SIM_MOTOR_BOARD=fake runs the SAME backend against
        # sim/fake_esp32.py on a pseudo-terminal, turning a sim body's wheels
        # -- so the whole stack exercises the hardware code path without a
        # board. The camera and lidar are not the board's: until their
        # drivers exist, the sim body stands in for them in the fake variant,
        # and on the car they answer "unusable".
        from robot.hardware_robot import HardwareRobot

        if os.environ.get("SIM_MOTOR_BOARD") == "fake":
            from sim.fake_esp32 import FakeEsp32
            from sim.mock_robot import MockRobot

            body = MockRobot(_sim_world())
            board = FakeEsp32(body)
            robot = HardwareRobot(board.path, sensors=body)
            robot.fake_board = board          # kept alive with the robot
            return robot
        port = os.environ.get("ROBOT_SERIAL") or (config.get("hardware") or {}).get("serial_port")
        if not port:
            raise ValueError("mode: hardware needs ROBOT_SERIAL (or hardware.serial_port) "
                             "-- the ESP32 driver board's serial device")
        return HardwareRobot(port)

    raise ValueError(f"Unknown robot mode in config: {mode!r}")
