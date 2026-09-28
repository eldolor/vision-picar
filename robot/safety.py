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

# ---- the chassis FOOTPRINT, and the corridor it sweeps (PLAN-ros-alignment 3.18) ----
#
# The URDF's body (picar_description's xacro): the chassis' OUTER length by
# its outer width, centred on the rotation centre. Since 3.20 that is the
# Waveshare UGV Rover's 253 x 231 mm (product page [V]; it was the 2WD
# build's 228 x 198) -- the shell, not the wheels, sets the width. The same
# rectangle nav2 plans with and collision_monitor checks.
# `tests/test_wall_linters.py` keeps these in step with the xacro.
#
# Why this exists: the path cone below (`PATH_HALF_ANGLE_DEG`) is sized for
# one 30cm move, so inside ~30cm it is NARROWER than the chassis. An
# obstacle off a front corner is outside it -- 3.17 watched a standing twist
# in the furnished home drive the chassis into the dining furniture while
# the cone read 40-80cm the whole way. The corridor check reads the
# 360-degree scan instead and asks the question the cone only approximates:
# is anything in the strip the chassis will sweep, within `min_distance_cm`
# of its leading edge? It runs in SERIES with the cone (whichever reads
# less decides), so nothing the cone caught before is released by it.
FOOTPRINT_LENGTH_M = 0.253
FOOTPRINT_WIDTH_M = 0.231
# Lateral room the corridor keeps beyond each side of the chassis. Returns
# BESIDE the body (not ahead of the leading edge) never stop a straight
# move -- a straight move cannot close on them, and treating them as
# blockers is R6's stop-polygon failure, frozen against a jamb. The margin
# is small because the starter house's doors are 30cm against a 23.1cm
# chassis (3.45cm a side; 5.1 on the old 19.8cm one); 3cm less the sim's
# 1.5cm ray-march over-read leaves 1.5cm that a converging wall can never
# close. Any larger and the corridor would refuse every starter-house door
# (`tests/chassis_fit.py` measures that edge at +3.5cm). The car's lidar is
# rated +/-3cm (RPLidar C1), so this is a hardware-day calibration item
# alongside `CHASSIS_WIDTH_CM` -- R8, with the chassis in hand.
FOOTPRINT_SIDE_MARGIN_CM = 3.0
# How far the safety layer asks the scan to look (a HINT: a real lidar
# ignores it, the sim stops casting there). The corridor only needs the
# leading edge plus `min_distance_cm` (~34cm to the corridor's far corner),
# the rear cone ~35cm; 60cm is headroom over both, and grows with
# `min_distance_cm` (see `_scan()`).
# Without the hint a sim scan casts 360 rays to 12m -- ~13ms in the
# furnished home, every control period, holding the GIL the wheel loop
# shares (3.17's second finding is about exactly that loop).
SAFETY_SCAN_RANGE_M = 0.6

# ---- pivots (PLAN-ros-alignment.md 3.19) ----
#
# A rectangle does not pivot within its own footprint: its corners sit
# 17.1cm from the rotation centre, its sides 11.55cm, so a turn sweeps a
# ring beyond the sides (15.1 and 9.9 on the old 2WD chassis). A turn is
# refused when the chassis, rotated by what it
# would turn before the next vet, would come within
# `PIVOT_MARGIN_CM` of a scan return AND that direction closes on it --
# turning AWAY is never refused, which is what lets a robot pinned against
# furniture free itself (R2b's "pivot away from a wall").
PIVOT_MARGIN_CM = 1.2
PIVOT_LOOKAHEAD_S = 0.05         # one wheel-loop period; scales with the turn rate
PIVOT_MIN_LOOKAHEAD_DEG = 1.0

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
# CHASSIS_WIDTH_CM is the PiCar-X's 16.5cm. It was kept on the belief that
# it over-states the differential chassis (148mm) and so errs wide -- but
# 148mm is the DECK. Across the wheels the old chassis was 19.8cm, and the
# UGV Rover is 23.1cm (`FOOTPRINT_WIDTH_M` above, the xacro, nav2's
# footprint), so this cone errs NARROW by ~6.6cm. The corridor check above
# is what covers the gap
# (3.18); this cone alone does not. **Re-measure it on the real chassis**
# -- a hardware-day pre-flight item, not a guess to leave standing.
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


def path_zone_indices(rows: int, cols: int, fov_deg=None, pan_deg=None) -> list:
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

    **`pan_deg` is where the grid points, relative to the BODY** (3.18 part
    2), published by a backend whose depth sensor pans with the camera --
    the sim's does. Zones are chosen by their bearing relative to the body,
    because the path is where the chassis goes, not where the camera looks.
    A grid panned off the path returns NO zones, and `path_clearance()` says
    so rather than reading a side wall as the way ahead: 3.17's "flaky"
    18.0 cm was exactly that, a camera left panned by a preempted mission.

    Rows are not narrowed here: the sim publishes `rows: 1` (`cast_ray()`
    has no elevation), and on a real sensor the rows that matter are
    decided by M8's floor rejection, which is geometry this layer does not
    have. Taking all rows until then is the conservative reading -- a floor
    return would veto a legal move rather than hide a real one.
    """
    global _warned_missing_fov
    if cols <= 0:
        return []

    pan = float(pan_deg or 0.0)
    if fov_deg and fov_deg > 0:
        bearings = [(b + pan + 180.0) % 360.0 - 180.0
                    for b in _zone_bearings_deg(cols, float(fov_deg))]
        chosen = [c for c, b in enumerate(bearings) if abs(b) <= PATH_HALF_ANGLE_DEG]
        if not chosen:
            # Slices wider than the cone itself -- a coarse 360-degree grid.
            # Keep the column(s) pointing most nearly ahead rather than
            # returning nothing and falling through to the scalar, which is
            # the same `max(1, ...)` promise the fraction branch makes --
            # but only a slice that actually COVERS ahead. A grid panned
            # off the path has none, and says so by returning nothing.
            nearest = min(abs(b) for b in bearings)
            if nearest <= float(fov_deg) / cols / 2 + 1e-9:
                chosen = [c for c, b in enumerate(bearings)
                          if abs(abs(b) - nearest) < 1e-9]
    elif abs(pan) > PATH_HALF_ANGLE_DEG:
        # No geometry, and pointed away: nothing in it is the path.
        chosen = []
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
        2b. **`depth_grid_facing_away`** -- the grid is panned off the path
           (3.18 part 2), so it has no opinion about the way ahead.
           Clearance is `None` here, and `forward_clearance()` refuses a
           FORWARD when nothing else -- the scan -- can see the path either.
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
                                        grid.get("fov_deg"), grid.get("pan_deg"))
            if not indices and zones and grid.get("pan_deg"):
                # Pointed off the path (3.18 part 2). NOT the scalar: on
                # every backend here it looks where the camera looks too.
                return None, "depth_grid_facing_away"
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

    def rear_clearance(self, scan: Optional[dict] = None) -> Tuple[Optional[float], str]:
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
        scan = self._scan() if scan is None else scan
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

    def _scan(self) -> Optional[dict]:
        """One scan for one decision, looking only as far as safety needs.
        None for a robot with no scan at all (a test double, a wrapper)."""
        get_scan = getattr(self.robot, "get_scan", None)
        if get_scan is None:
            return None
        return get_scan(max_range_m=max(SAFETY_SCAN_RANGE_M,
                                        FOOTPRINT_LENGTH_M / 2 + 1.5 * self.min_distance_cm / 100.0))

    def footprint_clearance(self, direction: int = 1, scan: Optional[dict] = None
                            ) -> Tuple[Optional[float], str]:
        """Room in the corridor the chassis sweeps -- 3.18.

        `direction` +1 is forward, -1 is astern. Every scan return is put in
        the body frame (the lidar at the rotation centre, as in the sim and
        as `base_link` is in the URDF); a return counts if it lies within
        half the chassis width plus `FOOTPRINT_SIDE_MARGIN_CM` of the
        centre-line AND beyond the leading edge in the direction of travel.
        The clearance is its distance past that edge -- how far the chassis
        can move before the leading edge reaches it, the same meaning the
        cone's number has. A return INSIDE the outline's width and within
        its length (the body is already on it) reads 0.0.

        `(None, "no_scan")` for a backend with no usable scan: the cone and
        the scalar still answer, exactly as before 3.18. `(None,
        "scan_footprint_no_target")` when the corridor is empty.
        """
        scan = self._scan() if scan is None else scan
        if not scan or not scan.get("usable"):
            return None, "no_scan"
        half_len = FOOTPRINT_LENGTH_M * 50.0
        half_wid = FOOTPRINT_WIDTH_M * 50.0
        a0, inc = scan["angle_min_deg"], scan["angle_increment_deg"]
        best = None
        for i, r in enumerate(scan["ranges_m"]):
            if r is None:
                continue
            a = math.radians(a0 + i * inc)
            along = direction * r * 100.0 * math.cos(a)
            lateral = abs(r * 100.0 * math.sin(a))
            if along > half_len and lateral <= half_wid + FOOTPRINT_SIDE_MARGIN_CM:
                room = along - half_len
            elif 0.0 < along <= half_len and lateral <= half_wid:
                room = 0.0
            else:
                continue
            best = room if best is None else min(best, room)
        if best is None:
            return None, "scan_footprint_no_target"
        return round(best, 1), "scan_footprint"

    def pivot_scale(self, omega_rad_s: float, scan: Optional[dict] = None
                    ) -> Tuple[float, Optional[str]]:
        """How much of a turn at `omega_rad_s` may go ahead -- 3.19.

        `(1.0, None)` when all of it may. Otherwise the largest fraction
        (to 1/64) whose look-ahead keeps every closing return outside
        `PIVOT_MARGIN_CM`, and the reason; `0.0` stops the turn. Scaling
        rather than zeroing lets a turn slow into the margin: zeroing it
        stopped robots a whole control period (~3 degrees at 1 rad/s)
        short of where they could safely have turned.
        """
        scan = self._scan() if scan is None else scan
        if self.pivot_blocked(omega_rad_s, scan) is None:
            return 1.0, None
        lo, hi = 0.0, 1.0
        for _ in range(6):
            mid = (lo + hi) / 2
            if self.pivot_blocked(omega_rad_s * mid, scan, exact=True) is None:
                lo = mid
            else:
                hi = mid
        return lo, self.pivot_blocked(omega_rad_s, scan)

    def pivot_blocked(self, omega_rad_s: float, scan: Optional[dict] = None,
                      exact: bool = False) -> Optional[str]:
        """Why a turn at `omega_rad_s` (CCW positive, REP-103) must be
        refused, or None -- 3.19. `exact` drops the minimum look-ahead,
        for `pivot_scale()`'s search over slower turns.

        Every scan return is put in the body frame (x ahead, y left) and its
        distance to the chassis rectangle compared now and after the turn
        the next vet could not see. Refused only if that distance ends under
        `PIVOT_MARGIN_CM` AND shrinks. No usable scan: never refused -- the
        pre-3.19 behaviour, and a rotation on a scan-less backend is still
        a rotation within the circle the forward checks already cleared.
        """
        if omega_rad_s == 0:
            return None
        scan = self._scan() if scan is None else scan
        if not scan or not scan.get("usable"):
            return None
        hl, hw = FOOTPRINT_LENGTH_M * 50.0, FOOTPRINT_WIDTH_M * 50.0
        d = omega_rad_s * PIVOT_LOOKAHEAD_S
        if not exact and abs(d) < math.radians(PIVOT_MIN_LOOKAHEAD_DEG):
            d = math.copysign(math.radians(PIVOT_MIN_LOOKAHEAD_DEG), d)
        c, s = math.cos(-d), math.sin(-d)     # the world turns the other way

        def dist(x, y):
            return math.hypot(max(abs(x) - hl, 0.0), max(abs(y) - hw, 0.0))

        a0, inc = scan["angle_min_deg"], scan["angle_increment_deg"]
        for i, r in enumerate(scan["ranges_m"]):
            if r is None:
                continue
            a = math.radians(a0 + i * inc)
            x, y = r * 100.0 * math.cos(a), -r * 100.0 * math.sin(a)   # scan is clockwise-positive
            now = dist(x, y)
            if now > PIVOT_MARGIN_CM + 20.0:
                continue
            after = dist(c * x - s * y, s * x + c * y)
            if after < PIVOT_MARGIN_CM and after < now - 1e-6:
                side = "left" if omega_rad_s > 0 else "right"
                return (f"turn {side} clamped: a corner would come within "
                        f"{after:.1f}cm < {PIVOT_MARGIN_CM}cm (scan_footprint)")
        return None

    @staticmethod
    def _nearer(*readings) -> Tuple[Optional[float], str]:
        """The reading that binds: the least centimetres among those that
        found something. All clear -> the first one's (clear) answer."""
        found = [r for r in readings if r[0] is not None]
        return min(found, key=lambda r: r[0]) if found else readings[0]

    def forward_clearance(self) -> Tuple[Optional[float], str]:
        """What a FORWARD is vetted against: the cone (`path_clearance()`)
        and the swept corridor (`footprint_clearance()`), in series.

        **If neither can see the path, the answer is 0.0** (3.18 part 2): a
        camera panned away and no scan means nothing observes the ground the
        move crosses, and unobserved must not read as clear -- M3's rule, one
        level up."""
        cone, corridor = self.path_clearance(), self.footprint_clearance(+1)
        if cone[1] == "depth_grid_facing_away" and corridor[1] == "no_scan":
            return 0.0, "path_not_observed"
        return self._nearer(cone, corridor)

    def reverse_clearance(self) -> Tuple[Optional[float], str]:
        """What a reverse is vetted against: the rear cone and the swept
        corridor astern, in series, off one scan."""
        scan = self._scan()
        return self._nearer(self.rear_clearance(scan), self.footprint_clearance(-1, scan))


    def vet_wheel_velocity(self, left_rad_s: float, right_rad_s: float):
        """Clamp a wheel-velocity command the way `check_and_execute()` vets
        a verb -- phase R2b.

        Decomposes the command into body velocity `v` and yaw rate `omega`
        with the chassis constants the robot publishes, zeroes `v` if it
        points into less than `min_distance_cm` of clearance (ahead from
        `forward_clearance()`, behind from `reverse_clearance()`), and
        recomposes. **Rotation is clamped only when it closes on something**
        (`pivot_blocked()`, 3.19): turning away is always allowed, which is
        the property that lets a robot facing a wall turn away from it. Returns `(left, right, reason)`; `reason` is
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
            clearance, source = self.forward_clearance()
            if clearance is not None and clearance < self.min_distance_cm:
                v, reason = 0.0, f"forward clamped: {clearance}cm < {self.min_distance_cm}cm ({source})"
        elif v < 0:
            clearance, source = self.reverse_clearance()
            if clearance is not None and clearance < self.min_distance_cm:
                v, reason = 0.0, f"reverse clamped: {clearance}cm < {self.min_distance_cm}cm ({source})"
        scale, pivot = self.pivot_scale(omega)
        if pivot:
            omega *= scale
            reason = pivot if reason is None else f"{reason}; {pivot}"
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
            distance, source = self.forward_clearance()
            if distance is not None and distance < self.min_distance_cm:
                self.robot.stop()
                msg = (
                    f"Blocked {action}: distance={distance}cm < "
                    f"min={self.min_distance_cm}cm ({source})"
                )
                logger.warning(msg)
                raise SafetyViolation(msg)

        if action in REVERSE_ACTIONS:
            distance, source = self.reverse_clearance()
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
