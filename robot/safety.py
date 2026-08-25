"""
safety.py

Local safety layer. Sits between the AI's chosen action and the robot
backend. Works identically against MockRobot and (later) the real
hardware backend, because it only ever calls RobotInterface methods.

Hierarchy (see build plan Phase 3 / Phase 7):

    Emergency stop
         v
    Ultrasonic / simulated-distance safety   <-- this module
         v
    Robot controller
         v
    AI decision

The AI proposes an action; this layer can veto it. It never trusts the
AI's own claims about distance -- it always re-checks via get_distance().
"""

import logging
from robot.interface import RobotInterface

logger = logging.getLogger("safety")

FORWARD_ACTIONS = {"FORWARD"}


class SafetyViolation(Exception):
    """Raised when an action is blocked by the safety layer."""


class SafetyController:
    def __init__(self, robot: RobotInterface, min_distance_cm: float = 20.0):
        self.robot = robot
        self.min_distance_cm = min_distance_cm

    def check_and_execute(self, action: str, **kwargs) -> dict:
        """
        Validates `action` against current sensor readings before
        executing it. Returns the backend's result dict. Raises
        SafetyViolation if the action is blocked -- callers (the agent
        loop) should catch this and treat it as an implicit STOP.
        """
        if action in FORWARD_ACTIONS:
            distance = self.robot.get_distance()
            if distance < self.min_distance_cm:
                self.robot.stop()
                msg = (
                    f"Blocked {action}: distance={distance}cm < "
                    f"min={self.min_distance_cm}cm"
                )
                logger.warning(msg)
                raise SafetyViolation(msg)

        return self._dispatch(action, **kwargs)

    def _dispatch(self, action: str, **kwargs) -> dict:
        dispatch_table = {
            "FORWARD": lambda: self.robot.drive_forward(
                kwargs.get("speed", 50), kwargs.get("duration", 0.5)
            ),
            "REVERSE": lambda: self.robot.reverse(
                kwargs.get("speed", 50), kwargs.get("duration", 0.5)
            ),
            "LEFT": lambda: self.robot.turn_left(kwargs.get("angle", 90)),
            "RIGHT": lambda: self.robot.turn_right(kwargs.get("angle", 90)),
            "STOP": lambda: self.robot.stop(),
            "LOOK_LEFT": lambda: self.robot.look_left(),
            "LOOK_RIGHT": lambda: self.robot.look_right(),
            "LOOK_CENTER": lambda: self.robot.look_center(),
        }
        if action not in dispatch_table:
            raise ValueError(f"Unknown action: {action!r}")
        return dispatch_table[action]()
