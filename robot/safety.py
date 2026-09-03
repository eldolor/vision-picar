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
AI's own claims about distance -- it always re-checks the sensor itself.

Since phase M3 that re-check prefers `get_depth_grid()`'s path zones and
falls back to the scalar `get_distance()`; `path_clearance()` below is
the whole of that decision, and it is deliberately the only place in the
project that decides what "the path" means. Under `/navigate`'s
`bearing-only` wording this layer is the ONLY obstacle logic left on the
hardware path (M1), which is the argument for keeping it a number
against a threshold rather than anything cleverer.
"""

import logging
from typing import Optional, Tuple

from robot.interface import RobotInterface, ZONE_RANGE, ZONE_UNUSABLE

logger = logging.getLogger("safety")

# Only FORWARD triggers the pre-move distance re-check below. Correct today
# because every current backend either pivots in place (MockRobot's grid
# turns hit nothing) or doesn't move at all (ReplayRobot/TeleopRobot log-only
# acks) -- so a turn can never collide. It will be WRONG once
# robot/hardware_robot.py exists: a real PiCar-X's LEFT/RIGHT is Ackermann
# steering plus forward motion, an arc that consumes real space ahead, with
# no obstacle check at all as written. HARDWARE-READINESS.md section 5.2
# has the full reasoning. Nothing in simulation can ever surface the need
# for this, which is exactly why it's easy to forget -- flip this to
# {"FORWARD", "LEFT", "RIGHT"} as part of writing hardware_robot.py, not
# after the first collision.
FORWARD_ACTIONS = {"FORWARD"}

# How much of the depth grid's width counts as "the path the next move
# crosses" -- phase M3. The middle half of the columns, and the reason is
# geometry rather than taste.
#
# A PiCar-X is about 16.5cm wide. The middle half of the sim's 60-degree
# render is +/-15 degrees, which at one grid cell (30cm) ahead spans 16cm,
# and at the 20cm stop threshold spans 11cm. The middle half of a
# VL53L5CX's 45-degree field is +/-11.25 degrees: 12cm at one cell. Both
# bracket the robot's own width, which is the number that matters -- the
# veto should refuse what the chassis would hit and nothing else.
#
# The two failure modes either side of this are both real and both
# observable in the grid world. Take the WHOLE grid and the outermost rays
# of a 60-degree cone read the side walls of a 30cm corridor at ~18cm, so
# the robot is permanently vetoed in every corridor it is supposed to
# drive down. Take a single centre ray -- which is what `get_distance()`
# is -- and it looks straight through a doorway while the door frame is
# about to catch the chassis. Widening this fraction makes doorways
# unpassable; narrowing it turns the veto back into the one beam it
# already was.
#
# Revisited in M10 against the real sensor's actual field of view, which
# is the only thing that makes these angles more than arithmetic.
PATH_FRACTION = 0.5


def path_zone_indices(rows: int, cols: int) -> list:
    """Which zones of a `rows` x `cols` grid the next move crosses.

    The middle `PATH_FRACTION` of the columns, in every row. Rows are not
    narrowed here: the sim publishes `rows: 1` (`cast_ray()` has no
    elevation), and on a real sensor the rows that matter are decided by
    M8's floor rejection, which is geometry this layer does not have.
    Taking all rows until then is the conservative reading -- a floor
    return would veto a legal move rather than hide a real one.
    """
    span = max(1, round(cols * PATH_FRACTION))
    first = (cols - span) // 2
    return [r * cols + c for r in range(rows) for c in range(first, first + span)]


class SafetyViolation(Exception):
    """Raised when an action is blocked by the safety layer."""


class SafetyController:
    def __init__(self, robot: RobotInterface, min_distance_cm: float = 20.0):
        self.robot = robot
        self.min_distance_cm = min_distance_cm

    def path_clearance(self) -> Tuple[Optional[float], str]:
        """How much room the next forward move has, and where that came
        from -- phase M3.

        Returns `(centimetres, source)`. **`None` centimetres does not mean
        "unknown"**; it means the sensor reports nothing within range along
        the path, which is a fact about the room and never a veto. The
        cases it cannot answer at all resolve to the scalar instead, so
        there is no third meaning to get wrong.

        Three outcomes, in the order they are tried:

        1. **`depth_grid`** -- at least one path zone returned a real
           range. The clearance is the nearest of them. Side zones are
           informational and are not compared: in a 30cm corridor the
           outermost rays of a 60-degree cone read the walls beside the
           robot at ~18cm, and a veto that read those would refuse every
           move down every corridor.
        2. **`depth_grid_no_target`** -- every path zone answered, and
           none of them found anything within range. Clearance is `None`
           and the move proceeds.
        3. **`distance_sensor`** -- there is no grid, or every path zone
           was unusable. Falls back to `get_distance()`, which is exactly
           the pre-M3 veto: the scalar keeps its `0.0`-on-dropout collapse
           and so a fully blind grid still fails toward stop.

        **A failed zone never enters the comparison in either direction**,
        which is the rule this phase exists for. Reading `ZONE_UNUSABLE`
        as a distance would stop the robot constantly (`sim/sensors.py`
        would have handed it `0.0`); reading it as clear would drive
        through whatever the sensor could not see. It is dropped, and if
        that leaves nothing, case 3 answers.
        """
        # Duck-typed rather than an isinstance check: the safety layer is
        # handed test doubles and wrappers as well as real backends, and a
        # robot with no depth sensor at all is the normal case, not an
        # error.
        get_grid = getattr(self.robot, "get_depth_grid", None)
        if get_grid is not None:
            grid = get_grid()
            zones = grid.get("zones") or []
            indices = path_zone_indices(int(grid.get("rows", 1)), int(grid.get("cols", 0)))
            path = [zones[i] for i in indices if i < len(zones)]
            measured = [
                z["distance_cm"] for z in path
                if z.get("status") == ZONE_RANGE and z.get("distance_cm") is not None
            ]
            if measured:
                return min(measured), "depth_grid"
            if path and not all(z.get("status") == ZONE_UNUSABLE for z in path):
                return None, "depth_grid_no_target"

        return self.robot.get_distance(), "distance_sensor"

    def check_and_execute(self, action: str, **kwargs) -> dict:
        """
        Validates `action` against current sensor readings before
        executing it. Returns the backend's result dict. Raises
        SafetyViolation if the action is blocked -- callers (the agent
        loop) should catch this and treat it as an implicit STOP.
        """
        if action in FORWARD_ACTIONS:
            distance, source = self.path_clearance()
            if distance is not None and distance < self.min_distance_cm:
                self.robot.stop()
                msg = (
                    f"Blocked {action}: distance={distance}cm < "
                    f"min={self.min_distance_cm}cm ({source})"
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
