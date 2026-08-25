"""
mock_robot.py

Implements the exact function signatures that brain/ will call, backed by
GridWorld instead of GPIO/motors. This is the Phase 0 deliverable from the
build plan: the brain layer should never need to know whether it's talking
to this or to real hardware.

Speed/duration/angle are simulation-time-only concepts here -- they're
converted into discrete grid moves. That conversion is the only thing that
will differ from the real hardware backend; the function signatures and
return shapes stay identical.
"""

import logging
from sim.grid_world import GridWorld
from robot.interface import RobotInterface

logger = logging.getLogger("mock_robot")

# Simulation conversion constants (tune freely; only affects sim realism)
CELLS_PER_SECOND_AT_FULL_SPEED = 2.0  # at speed=100
DEGREES_PER_TURN = 90  # grid-world only supports 90-degree turns


class MockRobot(RobotInterface):
    """Drop-in backend for robot/ during the simulation-only phase."""

    def __init__(self, world: GridWorld):
        self.world = world

    # ---------- driving ----------

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        cells = self._speed_duration_to_cells(speed, duration)
        result = self.world.move(cells)
        self._settle(duration)
        return {"action": "drive_forward", "speed": speed, "duration": duration, **result}

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        cells = self._speed_duration_to_cells(speed, duration)
        result = self.world.move(-cells)
        self._settle(duration)
        return {"action": "reverse", "speed": speed, "duration": duration, **result}

    def turn_left(self, angle: int = 90) -> dict:
        steps = max(1, round(angle / DEGREES_PER_TURN))
        result = {}
        for _ in range(steps):
            result = self.world.turn_left()
        return {"action": "turn_left", "angle": angle, **result}

    def turn_right(self, angle: int = 90) -> dict:
        steps = max(1, round(angle / DEGREES_PER_TURN))
        result = {}
        for _ in range(steps):
            result = self.world.turn_right()
        return {"action": "turn_right", "angle": angle, **result}

    def stop(self) -> dict:
        self.world._record("STOP")
        return {"action": "stop"}

    # ---------- camera pan ----------

    def look_left(self) -> dict:
        return {"action": "look_left", **self.world.look_left()}

    def look_right(self) -> dict:
        return {"action": "look_right", **self.world.look_right()}

    def look_center(self) -> dict:
        return {"action": "look_center", **self.world.look_center()}

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        """Returns a structured scene description instead of pixels.
        See build plan Phase 0.5 -- grid-world 'camera frame' = text/struct,
        not an actual image."""
        return self.world.frame_description()

    def get_distance(self) -> float:
        """Distance in cm, matching the real ultrasonic sensor's units.
        Grid cells are treated as ~30cm for rough realism."""
        cells = self.world.distance_ahead()
        return round(cells * 30.0, 1)

    # ---------- internal ----------

    def _speed_duration_to_cells(self, speed: int, duration: float) -> int:
        speed = max(0, min(100, speed))
        cells = (speed / 100.0) * CELLS_PER_SECOND_AT_FULL_SPEED * duration
        return max(1, round(cells)) if speed > 0 and duration > 0 else 0

    def _settle(self, duration: float):
        # No real delay needed in tests; kept as a hook in case you want
        # simulated timing later (e.g. to test watchdog behavior).
        pass
