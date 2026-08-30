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

Two optional, opt-in behaviors, both Phase S4/S5 of
PLAN-sim-hardening.md and both off by default so the unit suite and every
existing demo stay exactly as fast and as exact as they always were:

- `realtime=True` makes `_settle()` actually sleep for a move's declared
  `duration` (config/robot.yaml's `sim.realtime`) -- see `_settle()`'s own
  docstring for why this matters and what it does not change.
- `sensor=` accepts a `sim.sensors.DistanceSensorModel` for noisy/lossy
  distance readings (config/robot.yaml's `sim.sensor_noise`) -- see that
  module's docstring for why `min_distance_cm` is otherwise never crossed
  at anything but exactly 0.0cm.
"""

import logging
import time
from typing import Optional

from sim.grid_world import GridWorld
from sim.sensors import DistanceSensorModel
from robot.interface import RobotInterface

logger = logging.getLogger("mock_robot")

# Simulation conversion constants (tune freely; only affects sim realism)
CELLS_PER_SECOND_AT_FULL_SPEED = 2.0  # at speed=100
DEGREES_PER_TURN = 90  # grid-world only supports 90-degree turns


class MockRobot(RobotInterface):
    """Drop-in backend for robot/ during the simulation-only phase."""

    def __init__(
        self,
        world: GridWorld,
        realtime: bool = False,
        sensor: Optional[DistanceSensorModel] = None,
    ):
        self.world = world
        self.realtime = realtime
        # None (the default) keeps get_distance()'s exact, noiseless,
        # always-30cm-multiple behavior every existing test depends on --
        # see sim/sensors.py's own docstring for why that is a real gap,
        # not just a simplification, and why closing it is opt-in.
        self.sensor = sensor

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
        Grid cells are treated as ~30cm for rough realism.

        With no sensor model configured (the default), this is exactly
        `cells * 30.0` -- unchanged from before Phase S5, and always an
        exact multiple of 30. With one configured, the reading goes
        through `DistanceSensorModel.read()` instead -- see that class's
        docstring for the noise/dropout/range/latency it adds."""
        cells = self.world.distance_ahead()
        if self.sensor is None:
            return round(cells * 30.0, 1)
        if self.realtime and self.sensor.read_latency_s:
            time.sleep(self.sensor.read_latency_s)
        return self.sensor.read(cells)

    # ---------- internal ----------

    def _speed_duration_to_cells(self, speed: int, duration: float) -> int:
        speed = max(0, min(100, speed))
        cells = (speed / 100.0) * CELLS_PER_SECOND_AT_FULL_SPEED * duration
        return max(1, round(cells)) if speed > 0 and duration > 0 else 0

    def _settle(self, duration: float):
        """No-op unless self.realtime is set (config/robot.yaml's
        sim.realtime, Phase S4). Before this, PLAN-sim-hardening.md
        section 3.1 was literally true: "the watchdog's async loop is
        never executed by any test" -- there was no way for an action to
        occupy any wall-clock time at all, sim or live. Enabling this
        makes a move actually take its declared duration, which is what
        lets robot/server.py's watchdog readout climb mid-move instead of
        only between moves, and what stops speed/duration from being
        decorative. Off by default so the unit suite (and every
        tests/demo_*.py script) stays exactly as fast as it always was --
        this is a live/manual-testing and integration-test knob, not
        something the automated suite should pay for by default."""
        if self.realtime and duration > 0:
            time.sleep(duration)
