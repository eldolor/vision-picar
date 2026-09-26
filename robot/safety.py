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
import math
from typing import Optional, Tuple

from robot.interface import (
    DEPTH_COLS_DEFAULT,
    RobotInterface,
    ZONE_RANGE,
    ZONE_UNUSABLE,
)

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
# R2b: checked against the lidar's REAR beams, by the user's decision
# (PLAN-ros-alignment.md 3.10). Every way of backing up -- /action REVERSE
# and a negative /wheels velocity alike.
REVERSE_ACTIONS = {"REVERSE"}

# How far the lidar (at the robot's centre) sits from the REAR bumper. The
# scan measures from the centre, so this is subtracted before a rear range is
# compared with `min_distance_cm`. 15cm is half the sim's 30cm robot; the real
# deck-centre lidar is 11-14cm from either bumper -- measure it on the chassis
# (R8), and at R3 it becomes a transform rather than a constant.
LIDAR_TO_REAR_BUMPER_CM = 15.0

# What counts as "the path the next move crosses" -- phase M3, reworked by
# PLAN-onboard-perception.md section 5.1. The reason is geometry rather
# than taste, and the geometry is an ANGLE.
#
# The rule: a zone is in the path if its bearing is within
# +/-atan(half the chassis width / one move's travel) of straight ahead.
# That is the cone the chassis sweeps crossing the ground the next move
# covers, so the veto refuses what the robot would hit and nothing else.
#
# The two failure modes either side of it are both real and both
# observable in the grid world. Too WIDE and the outermost rays of a
# 60-degree cone read the side walls of a 30cm corridor at ~18cm, so the
# robot is permanently vetoed in every corridor it is supposed to drive
# down. Too NARROW and it degenerates to the single centre ray
# `get_distance()` already was, looking straight through a doorway while
# the frame is about to catch the chassis.
#
# **Why this is an angle and not a fraction of the columns.** It used to
# be `PATH_FRACTION = 0.5`, the middle half of the columns, which was a
# sound proxy while every sensor considered had a narrow forward field --
# 60 degrees in the sim, 45 on a VL53L5CX. On a 360-degree lidar the
# middle half of the columns is +/-90 degrees: the entire forward
# hemisphere, which is precisely the permanently-vetoed-in-every-corridor
# failure above. The grid carried `rows`/`cols` but never the angular span
# they cover, because with every sensor so far that span was implicit and
# similar. A 360-degree sensor makes the omission load-bearing, so
# `get_depth_grid()` now carries `fov_deg` and this selects on it.
#
# On the sim's 60-degree/8-column grid the two rules pick the same four
# zones, which is what makes this a change of input rather than of
# behaviour. On a 45-degree ToF the angular rule picks six columns where
# the fraction picked four -- the fraction's +/-11.25 degrees spans 12cm at
# one move's travel, which does NOT bracket a 16.5cm chassis. The old
# comment claimed it did. It was arithmetic that happened to land close
# enough on the one sensor it was checked against.
#
# CHASSIS_WIDTH_CM is the PiCar-X's, kept deliberately: the differential
# chassis chosen in PLAN-onboard-perception.md section 1.1 is 148mm wide,
# so this over-states the width and the cone errs wide, which is the safe
# direction. **Re-measure it on the real chassis** -- that is a hardware-day
# pre-flight item, not a guess to leave standing.
CHASSIS_WIDTH_CM = 16.5

# How far the range sensor sits BEHIND the leading edge of the chassis.
# Subtracted from every measured clearance, so `min_distance_cm` means
# what everyone reads it as: room between the *bumper* and the obstacle,
# not between the sensor and the obstacle.
#
# **0.0 is correct for every backend that exists today and will be wrong
# the day a lidar is fitted.** The PiCar-X's ultrasonic points forward from
# the front of the chassis, so sensor origin and bumper coincide and the
# term vanishes; the sim's rays are cast from a robot that is a point and
# has no bumper at all. A 360-degree lidar moves the origin to the middle
# of a ~228 x 148mm deck -- roughly 11-14cm back -- at which point a
# reading of "20cm" is 6-9cm of actual gap, inside the travel of the
# compliant bumper that is supposed to be the last resort.
#
# That is why this exists now rather than on hardware day: the number is
# currently zero, so nothing changes, but the *meaning* of the comparison
# at `check_and_execute()` is pinned before a sensor move can alter it
# silently. Measure it on the real chassis and set
# `safety.sensor_to_bumper_cm` -- a hardware-day pre-flight item alongside
# `CHASSIS_WIDTH_CM` above.
#
# One offset, applied to both the grid and the scalar, because today one
# sensor answers both. A robot carrying a deck-centre lidar AND a
# front-mounted ToF has two different mounts and needs the offset to ride
# on the grid the way `fov_deg` does (PLAN-onboard-perception.md 5.1 is
# the precedent for exactly that move). Don't infer a second copy here --
# publish it from the backend when that day comes.
SENSOR_TO_BUMPER_CM = 0.0

# How far ahead the cone is required to bracket the chassis: one move's
# travel, since that is the ground `path_zone_indices()` is asked about.
# One grid cell in the sim, and what the original arithmetic used.
PATH_REFERENCE_CM = 30.0
PATH_HALF_ANGLE_DEG = math.degrees(math.atan(CHASSIS_WIDTH_CM / 2 / PATH_REFERENCE_CM))

# The pre-5.1 rule, still used for a grid that does not declare `fov_deg`.
# Correct for the narrow forward sensors that are the only ones able to
# omit it (see `path_zone_indices()`), and wrong for a wide one -- which is
# why omitting it is warned about rather than silently accepted.
PATH_FRACTION = 0.5

# Above how many columns an undeclared field of view is treated as a
# mistake rather than as a narrow sensor. `DEPTH_COLS_DEFAULT` is what a
# VL53L5CX gives per row and what every backend here publishes today, so
# anything wider is a sensor this module has not been told the geometry of.
_WIDE_GRID_COLS = DEPTH_COLS_DEFAULT

# One warning per process, not per call: this is on the veto's hot path.
_warned_missing_fov = False


def _zone_bearings_deg(cols: int, fov_deg: float) -> list:
    """The bearing of each column's centre, left to right, relative to
    straight ahead.

    Matches how the zones are cast in the first place -- `MockRobot`'s
    `get_depth_grid()` and `sim/renderer.py`'s ray loop both walk
    `-fov/2 + fov * (i + 0.5) / cols`. Two copies of that convention is
    how the strip and the veto would start disagreeing about which
    direction a zone points.
    """
    return [-fov_deg / 2 + fov_deg * (i + 0.5) / cols for i in range(cols)]


def path_zone_indices(rows: int, cols: int, fov_deg=None) -> list:
    """Which zones of a `rows` x `cols` grid the next move crosses.

    `fov_deg` is the grid's own horizontal field of view, straight out of
    `get_depth_grid()`. Given one, the path is every column whose bearing
    is within `PATH_HALF_ANGLE_DEG` of ahead. Without one, it falls back to
    the middle `PATH_FRACTION` of the columns.

    **The fallback is for narrow sensors only, and says so out loud.** A
    backend that reports real ranges over a wide field and omits `fov_deg`
    gets the pre-5.1 behaviour, which on a 360-degree unit selects the
    whole forward hemisphere and vetoes everything. That is a silent
    fallback of exactly the kind M7 exists to remove, so it warns.

    Rows are not narrowed here: the sim publishes `rows: 1` (`cast_ray()`
    has no elevation), and on a real sensor the rows that matter are
    decided by M8's floor rejection, which is geometry this layer does not
    have. Taking all rows until then is the conservative reading -- a floor
    return would veto a legal move rather than hide a real one.
    """
    global _warned_missing_fov
    if cols <= 0:
        return []

    if fov_deg and fov_deg > 0:
        bearings = _zone_bearings_deg(cols, float(fov_deg))
        chosen = [c for c, b in enumerate(bearings) if abs(b) <= PATH_HALF_ANGLE_DEG]
        if not chosen:
            # Slices wider than the cone itself -- a coarse 360-degree grid.
            # Keep the column(s) pointing most nearly ahead rather than
            # returning nothing and falling through to the scalar, which is
            # the same `max(1, ...)` promise the fraction branch makes.
            nearest = min(abs(b) for b in bearings)
            chosen = [c for c, b in enumerate(bearings)
                      if abs(abs(b) - nearest) < 1e-9]
    else:
        if cols > _WIDE_GRID_COLS and not _warned_missing_fov:
            _warned_missing_fov = True
            logger.warning(
                "depth grid reports %d columns but no fov_deg -- falling back to "
                "the middle %.0f%% of columns, which is only correct for a narrow "
                "forward sensor. A wide sensor must declare fov_deg "
                "(PLAN-onboard-perception.md 5.1).", cols, PATH_FRACTION * 100)
        span = max(1, round(cols * PATH_FRACTION))
        first = (cols - span) // 2
        chosen = list(range(first, first + span))

    return [r * cols + c for r in range(rows) for c in chosen]


class SafetyViolation(Exception):
    """Raised when an action is blocked by the safety layer."""


class SafetyController:
    def __init__(self, robot: RobotInterface, min_distance_cm: float = 20.0,
                 sensor_to_bumper_cm: float = SENSOR_TO_BUMPER_CM):
        self.robot = robot
        self.min_distance_cm = min_distance_cm
        self.sensor_to_bumper_cm = sensor_to_bumper_cm

    def _to_bumper(self, distance_cm: float) -> float:
        """A sensor-frame range as clearance ahead of the bumper.

        Clamped at zero: a negative clearance means the obstacle is already
        inside the chassis outline, and `0.0` is the reading that always
        vetoes. Rounded because the subtraction otherwise turns the sim's
        exact multiples of 30 into binary noise in the log line.
        """
        return round(max(0.0, distance_cm - self.sensor_to_bumper_cm), 6)

    def path_clearance(self) -> Tuple[Optional[float], str]:
        """How much room the next forward move has, and where that came
        from -- phase M3.

        Returns `(centimetres, source)`. **`None` centimetres does not mean
        "unknown"**; it means the sensor reports nothing within range along
        the path, which is a fact about the room and never a veto. The
        cases it cannot answer at all resolve to the scalar instead, so
        there is no third meaning to get wrong.

        **Centimetres are measured from the bumper, not from the sensor**
        -- `SENSOR_TO_BUMPER_CM` is subtracted from every real range (see
        that constant for why, and why it is 0.0 today). `None` is left
        alone: there is no distance to a thing that is not there.

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
            indices = path_zone_indices(int(grid.get("rows", 1)),
                                        int(grid.get("cols", 0)),
                                        grid.get("fov_deg"))
            path = [zones[i] for i in indices if i < len(zones)]
            measured = [
                z["distance_cm"] for z in path
                if z.get("status") == ZONE_RANGE and z.get("distance_cm") is not None
            ]
            if measured:
                return self._to_bumper(min(measured)), "depth_grid"
            if path and not all(z.get("status") == ZONE_UNUSABLE for z in path):
                return None, "depth_grid_no_target"

        return self._to_bumper(self.robot.get_distance()), "distance_sensor"

    def rear_clearance(self) -> Tuple[Optional[float], str]:
        """How much room is behind the robot, from the lidar's rear beams --
        phase R2b, by the user's decision.

        The beams within the same half-angle as the forward path cone
        (`PATH_HALF_ANGLE_DEG`) either side of dead astern, nearest return,
        minus `LIDAR_TO_REAR_BUMPER_CM`. A beam with no return is clear. A
        backend with no usable scan answers `(None, "no_rear_sensor")` and
        the reverse proceeds -- today's behaviour, because refusing every
        reverse on a lidar-less backend would make it undrivable rather than
        safe. With a lidar fitted, this is what stands between a reverse
        and whatever is behind.
        """
        get_scan = getattr(self.robot, "get_scan", None)
        scan = get_scan() if get_scan is not None else None
        if not scan or not scan.get("usable"):
            return None, "no_rear_sensor"
        a0, inc = scan["angle_min_deg"], scan["angle_increment_deg"]
        rear = []
        for i, r in enumerate(scan["ranges_m"]):
            if r is None:
                continue
            angle = (a0 + i * inc + 180.0) % 360.0 - 180.0   # -180..180, 0 ahead
            if 180.0 - abs(angle) <= PATH_HALF_ANGLE_DEG:
                rear.append(r)
        if not rear:
            return None, "scan_rear_no_target"
        return max(0.0, round(min(rear) * 100.0 - LIDAR_TO_REAR_BUMPER_CM, 1)), "scan_rear"

    def vet_wheel_velocity(self, left_rad_s: float, right_rad_s: float):
        """Clamp a wheel-velocity command the way `check_and_execute()` vets
        a verb -- phase R2b.

        Decomposes the command into body velocity `v` and yaw rate `omega`
        with the chassis constants the robot publishes, zeroes `v` if it
        points into less than `min_distance_cm` of clearance (ahead from
        `path_clearance()`, behind from `rear_clearance()`), and recomposes.
        **Rotation is never clamped**: a differential chassis pivots within
        its own footprint, which is the property that lets a robot facing a
        wall turn away from it. Returns `(left, right, reason)`; `reason` is
        None when nothing was clamped.
        """
        wheels = self.robot.get_wheel_state()
        if not wheels.get("usable"):
            return left_rad_s, right_rad_s, None
        radius, track = wheels["wheel_radius_m"], wheels["track_width_m"]
        v = (left_rad_s + right_rad_s) / 2.0 * radius
        omega = (right_rad_s - left_rad_s) * radius / track
        reason = None
        if v > 0:
            clearance, source = self.path_clearance()
            if clearance is not None and clearance < self.min_distance_cm:
                v, reason = 0.0, f"forward clamped: {clearance}cm < {self.min_distance_cm}cm ({source})"
        elif v < 0:
            clearance, source = self.rear_clearance()
            if clearance is not None and clearance < self.min_distance_cm:
                v, reason = 0.0, f"reverse clamped: {clearance}cm < {self.min_distance_cm}cm ({source})"
        if reason is None:
            return left_rad_s, right_rad_s, None
        half = omega * track / 2.0
        return (v - half) / radius, (v + half) / radius, reason

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

        if action in REVERSE_ACTIONS:
            distance, source = self.rear_clearance()
            if distance is not None and distance < self.min_distance_cm:
                self.robot.stop()
                msg = (f"Blocked {action}: rear clearance={distance}cm < "
                       f"min={self.min_distance_cm}cm ({source})")
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
