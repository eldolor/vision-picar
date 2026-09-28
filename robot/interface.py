"""
interface.py

The abstract contract brain/ is allowed to depend on. Both the simulation
backend (sim/mock_robot.py) and the eventual real hardware backend
(robot/hardware_robot.py, added in Phase 11) implement this exact set of
methods. brain/ never imports a backend directly -- only this interface,
via robot/factory.py.
"""

import math
import time
from abc import ABC, abstractmethod
from typing import Callable, Optional, Tuple

# What `get_distance()` returns on a backend that has no distance sensor at
# all -- a photograph has no depth in it, so ReplayRobot and TeleopRobot
# both answer with this. Far above any configured `min_distance_cm`, so
# robot/safety.py's veto never fires there: an honest "I cannot tell you"
# rather than a fabricated clearance that would make a walk look like it
# had exercised collision avoidance.
#
# Lives here, on the interface, because it is part of the contract rather
# than a property of any one backend -- and because it was previously
# defined twice, in sim/replay_robot.py and again in sim/teleop_robot.py
# with a comment asking the reader to keep the two in step by hand.
NO_SENSOR_CM = 999.0


# ---------- who is driving (phase M4) ----------
#
# Microduck lists authority priority between the physical controller, the
# app and the autonomous layer as something to *decide* rather than let
# emerge (`architecture` section 6). The decision, written out in
# `AGENT-HARNESS.md` and enforced by `robot/server.py`:
#
#     stop  >  a person  >  one autonomous driver at a time
#
# `stop` is not a driver -- it is everyone's right, always, and never
# claims or loses authority. The rest rank by ROLE, not by client: a
# person issuing one command at a time outranks any loop. (The in-browser
# "local brain" this order once ended with was deleted 2026-09-25; its
# rank below is dead.)
DRIVER_MANUAL = 30
DRIVER_AUTONOMOUS = 20
DRIVER_LOCAL = 10

DRIVER_PRIORITY = {
    "twin-dpad": DRIVER_MANUAL,
    "teleop-operator": DRIVER_MANUAL,
    "brain": DRIVER_AUTONOMOUS,
    "teleop": DRIVER_AUTONOMOUS,
    # R2b / R4: the ROS stack, through picar_sim_hardware or picar_hardware.
    # EQUAL to the brain -- they are two layers (mission picks goals, nav2
    # moves), not rivals -- and, like the brain, below a person. At the
    # autonomous rank the holder is EXCLUSIVE until it lapses, so the two
    # never interleave (robot/server.py's arbitrate()); industry practice as
    # twist_mux encodes it: e-stop > human > autonomy, one writer at a time.
    "ros": DRIVER_AUTONOMOUS,
    "twin-local-brain": DRIVER_LOCAL,
}

# A command that names no driver. **Ranked as manual, deliberately**, and
# the reasoning is worth keeping: the callers that do not name themselves
# are a person with curl, a script someone is running by hand, or a test.
# Ranking an unnamed caller LOW would mean a running mission ignores a
# human's direct command, which is precisely what the order above forbids.
# Ranking it high means a human at a terminal preempts the robot, which is
# what the order says should happen. When every client names itself this
# can be tightened; until then, err toward the person.
DRIVER_UNKNOWN = "unknown"


def driver_priority(name: str) -> int:
    return DRIVER_PRIORITY.get(name, DRIVER_MANUAL)


class Preempted(Exception):
    """A movement command was refused because a higher-priority driver
    holds the robot -- phase M4.

    Distinct from `SafetyViolation`, and the distinction is the point. A
    safety veto means *this move* is unsafe and the next one may be fine;
    a preemption means *this driver* is no longer in charge, and retrying
    is exactly the wrong response. `control/mission_runner.py` ends the
    mission `preempted` rather than counting it against any budget.

    Lives here rather than in `robot/safety.py` because `control/` may
    import `robot/interface.py` and little else (see CLAUDE.md section 6),
    and because "someone else is driving" is a fact about the robot rather
    than a judgement made by the safety layer.
    """



# ---------- the depth grid (phase M2, PLAN-microduck-transplants.md) ----------
#
# Microduck's split, which M1 measured the need for the hard way: the camera
# answers *what* and *which way*, a depth sensor answers *how far*. Four
# rewordings of `/navigate`'s obstacle question and one deletion of it
# established that a single monocular frame does not contain metric depth, so
# no wording recovers it. `get_depth_grid()` below is the seam the sensor
# arrives through -- built and exercised in the sim first (M2), consumed by
# `robot/safety.py` (M3), filled from a real VL53L5CX later (M10).
#
# A zone has THREE outcomes, not two, borrowed from `tof/src/lib.rs`:
#
#   ZONE_RANGE      a real measurement -- `distance_cm` is a number
#   ZONE_NO_TARGET  nothing within range -- `distance_cm` is None, and this
#                   is *information*: empty space is a fact about the room
#   ZONE_UNUSABLE   the zone could not be measured -- `distance_cm` is None,
#                   and this is the *absence* of a fact
#
# Collapsing the last two is what M3 argues against, and the argument is
# concrete rather than tidy: a veto that takes the nearest zone over a grid
# whose failed zones read `0.0` would stop constantly. A failed zone must
# never enter a range comparison in either direction. `sim/sensors.py`'s
# scalar `get_distance()` keeps its `0.0`-on-dropout fail-safe collapse --
# that is correct for a single number with no way to say "I could not tell",
# and it is the reason the tri-state lives here and not there.
ZONE_RANGE = "range"
ZONE_NO_TARGET = "no_target"
ZONE_UNUSABLE = "unusable"

# How many zones a backend with no depth sensor publishes. Eight because
# that is what a VL53L5CX gives per row and what the twin's strip is drawn
# for -- an all-grey strip of the same width as a live one, so "no sensor"
# and "sensor reading nothing" are visibly different states rather than one
# being an empty space on the page.
DEPTH_COLS_DEFAULT = 8


def unusable_grid(cols: int = DEPTH_COLS_DEFAULT) -> dict:
    """The honest answer from a backend that has no depth sensor.

    Same choice `NO_SENSOR_CM` makes for the scalar reading: say so, rather
    than fabricate a clearance. Every zone is `ZONE_UNUSABLE`, which no
    consumer may compare against a threshold -- so a photograph-driven
    backend cannot make a walk look as though it had exercised collision
    avoidance it never had.
    """
    # No `fov_deg`: a backend with no sensor has no field of view, and
    # inventing one would be the same fabrication `NO_SENSOR_CM` refuses.
    # Nothing reads it here anyway -- an all-unusable grid never reaches a
    # range comparison, so which zones are "the path" cannot matter.
    return {
        "rows": 1,
        "cols": cols,
        "zones": [{"status": ZONE_UNUSABLE, "distance_cm": None} for _ in range(cols)],
    }



def unusable_odometry() -> dict:
    """The honest answer from a backend that cannot measure its own motion.

    Same choice `unusable_grid()` and `NO_SENSOR_CM` make: say so rather
    than fabricate. `usable: False` and both quantities `None`, which no
    consumer may compare against a threshold -- so a backend driven by
    photographs cannot make a walk look as though it had measured
    distance travelled that it never did.

    **This matters more than it looks.** The only real-pixels backend this
    project has is `TeleopRobot` -- a phone on a wheeled rig -- and a phone
    has no encoders. So the walks that validate perception are exactly the
    walks that cannot report odometry, and a consumer that silently
    treated "no odometry" as "travelled 0cm" would freeze every
    distance-based rule on precisely those runs, invisibly.
    """
    return {"usable": False, "distance_m": None, "heading_deg": None}


def unusable_wheels() -> dict:
    """The honest answer from a backend with no wheel encoders -- phase R2.

    Same choice as `unusable_odometry()`: `usable: False` and no numbers, so
    a photograph-driven or phone-driven backend cannot look as though it had
    measured wheel motion it never had. `ros2_control` reads this at R4;
    zeros here would be a robot that had not moved, which is a claim.
    """
    return {"usable": False, "left": None, "right": None,
            "wheel_radius_m": None, "track_width_m": None, "counts_per_rev": None}


# ---- carrying out a verb (PLAN-ros-alignment.md 3.22) ----
#
# The MECHANICS of a verb, shared by the safety layer (which passes a
# `limit` that re-vets every period) and a backend's own unguarded verbs (no
# limit) -- one loop, so a guarded verb and a raw one are the same motion to
# the last bit whenever the way is clear. The decisions are not here: what
# the verb means is the backend's (`verb_plan()`), and what is safe is
# `robot/safety.py`'s.
VERB_PERIOD_S = 0.05          # one wheel-loop period (robot/server.py)
# The most a turn rotates between two checks: a check compares the chassis
# before and after a step, so a step so large a corner can pass THROUGH
# something and out the far side is not checked at all. The sim's verbs
# turn at ~400 deg/s, 20 degrees a period; five is the sim's own collision
# sub-step (`sim/grid_world.py` MAX_SUBSTEP_RAD).
VERB_TURN_STEP_DEG = 5.0
_VERB_DONE_M = 1e-7
_VERB_DONE_DEG = 1e-5

Limit = Callable[[float], Tuple[float, Optional[str]]]


def carry_out_verb(robot: "RobotInterface", plan: dict, limit: Optional[Limit] = None) -> dict:
    """Carry out `plan` (a backend's `verb_plan()`) as a standing wheel
    command, in periods, closed on the ENCODERS rather than the clock.

    Each period, in order: (1) if anyone has called `stop()` since the verb
    began (`/stop`, the watchdog), the verb is over and its command is never
    sent again; (2) how much is left, from the wheel positions; (3) how much
    of this period's share `limit(amount)` allows -- metres for a
    translation, degrees for a turn -- where 0 ends the verb "clamped";
    (4) command it, and let the time pass: the wall clock for a real board,
    `advance()` for the sim.

    Returns {"done": metres or degrees achieved, "ended": "complete" |
    "clamped" | "stopped" | "stalled" | "timeout", "reason"}.
    """
    straight = plan["kind"] == "straight"
    left, right, target = plan["left_rad_s"], plan["right_rad_s"], plan["target"]
    w0 = robot.get_wheel_state()
    radius, track = w0["wheel_radius_m"], w0["track_width_m"]
    v = (left + right) / 2.0 * radius
    rate = abs(v) if straight else math.degrees(abs((right - left) * radius / track))
    stops0 = getattr(robot, "stop_count", 0)

    def achieved() -> float:
        w = robot.get_wheel_state()
        dl = w["left"]["position_rad"] - w0["left"]["position_rad"]
        dr = w["right"]["position_rad"] - w0["right"]["position_rad"]
        if straight:
            return abs((dl + dr) / 2.0 * radius)
        return abs(math.degrees((dr - dl) * radius / track))

    ended, reason = "timeout", None
    periods = int(math.ceil(target / rate / VERB_PERIOD_S)) * 3 + 20 if rate else 0
    try:
        for _ in range(periods):
            if getattr(robot, "stop_count", 0) != stops0:
                ended, reason = "stopped", "stop() was called"
                break
            done = achieved()
            remaining = target - done
            if remaining <= (_VERB_DONE_M if straight else _VERB_DONE_DEG):
                ended = "complete"
                break
            amount = min(remaining, rate * VERB_PERIOD_S)
            if not straight:
                amount = min(amount, VERB_TURN_STEP_DEG)
            if limit is not None:
                allowed, why = limit(amount)
                if allowed <= (_VERB_DONE_M if straight else _VERB_DONE_DEG):
                    ended, reason = "clamped", why
                    break
                amount = min(amount, allowed)
            # Full speed for a shorter time, never a slower wheel: the same
            # motion, at the rate the motors are asked for.
            step = amount / rate
            robot.set_wheel_velocity(left, right)
            if plan.get("wall_clock"):
                time.sleep(step)
            else:
                robot.advance(step)
                if abs(achieved() - done) <= 1e-12:
                    ended, reason = "stalled", "the body did not move"
                    break
    finally:
        if ended != "stopped":
            robot.set_wheel_velocity(0.0, 0.0)
    return {"done": achieved(), "ended": ended, "reason": reason}


def unusable_scan() -> dict:
    """The honest answer from a backend with no lidar -- phase R2.

    `ranges_m: None` rather than an empty list or a list of Nones: "no
    sensor" and "a sensor that saw nothing within range" are different
    facts, and the second is a list of Nones (see `get_scan()`).
    """
    return {"usable": False, "angle_min_deg": None, "angle_increment_deg": None,
            "range_min_m": None, "range_max_m": None, "ranges_m": None}


class RobotInterface(ABC):
    @abstractmethod
    def drive_forward(self, speed: int, duration: float) -> dict: ...

    @abstractmethod
    def reverse(self, speed: int, duration: float) -> dict: ...

    @abstractmethod
    def turn_left(self, angle: int) -> dict: ...

    @abstractmethod
    def turn_right(self, angle: int) -> dict: ...

    @abstractmethod
    def stop(self) -> dict: ...

    @abstractmethod
    def look_left(self) -> dict: ...

    @abstractmethod
    def look_right(self) -> dict: ...

    @abstractmethod
    def look_center(self) -> dict: ...

    @abstractmethod
    def get_camera_frame(self) -> dict:
        """One camera frame. **Must carry pixels** -- phase S2.

            {"image_base64": str,            # the frame, base64
             "media_type": "image/jpeg",     # or whatever it really is
             "room": str,                    # MissionMemory reads this
             "metadata": {...},              # provenance; policies ignore it
             ...}                            # backend-specific extras

        The image keys are the contract. They exist because the whole
        design rests on this interface not changing when hardware lands
        (`robot/factory.py`), and a Pi camera returns bytes -- so a
        backend that answered with grid facts alone was
        `PLAN-sim-hardening.md` 2.1's blocker, guaranteeing the one
        abstraction brain/ depends on would have to change.

        Anything a backend adds beyond those keys is its own business and
        **no policy on the hardware path may read it.** `MockRobot` puts
        grid coordinates there for the rule-based agent (section 2.2); a
        camera cannot produce them, so a vision policy that reads them is
        cheating and will fail on a real robot.

        Raising is allowed and meaningful: `TeleopRobot` raises when its
        frames have gone stale, and `robot/server.py`'s `/frame` turns
        that into a 503 rather than pretending it has a picture.
        """

    @abstractmethod
    def get_distance(self) -> float: ...

    def get_depth_grid(self) -> dict:
        """Depth zones across the field of view -- phase M2.

            {"rows": int,          # vertical zones; 1 when there is no
                                   #   elevation to report
             "cols": int,          # horizontal zones, left to right
             "fov_deg": float,     # horizontal field of view the cols span
             "zones": [            # row-major, len() == rows * cols
                {"status": ZONE_RANGE | ZONE_NO_TARGET | ZONE_UNUSABLE,
                 "distance_cm": float | None},
                ...
             ]}

        **Not abstract, and deliberately so.** Most backends here have no
        depth sensor and never will -- a photograph has no depth in it, and
        a live phone walk has no ToF -- so the default is
        `unusable_grid()`: every zone unmeasurable, which is the same
        honest no-op `NO_SENSOR_CM` already gives for `get_distance()`. A
        backend that *does* have depth overrides this. That is what keeps
        `RobotInterface` from having to change again on the day a sensor is
        fitted (`robot/factory.py`'s promise), which is the whole reason
        this is on the interface rather than tucked inside `HardwareRobot`.

        **`fov_deg` is required of any backend reporting real ranges over a
        wide field**, and is what makes `cols` mean something. Columns are
        the centres of `cols` equal slices of it, so column `i` points at
        `-fov_deg/2 + fov_deg * (i + 0.5) / cols` -- the convention
        `sim/renderer.py` casts rays on and `robot/safety.py` selects the
        path on. Omitting it is tolerated only because every backend that
        predates it has a narrow forward field, where the fraction-of-columns
        rule it falls back to is a sound proxy for an angle; on a
        360-degree lidar that proxy selects the whole forward hemisphere
        and vetoes every corridor. `PLAN-onboard-perception.md` 5.1 is the
        full argument, and `robot/safety.py` warns when a wide grid arrives
        without it.

        **`rows`/`cols` travel in the data rather than being pinned by the
        contract**, borrowing Microduck's `Frame` shape, and that is what
        lets the simulator be honest. The grid world is two-dimensional and
        `sim/renderer.py:cast_ray()` has no elevation, so `MockRobot` can
        produce eight columns truthfully and cannot produce eight rows at
        all. It publishes `rows: 1`. A consumer can see the vertical
        dimension is absent instead of reading eight identical copies of
        one row and believing it has a matrix.

        **Nothing in `brain/` reads this yet.** M3 gives it its first
        consumer: `robot/safety.py` reduces the centre zones to one scalar
        and compares that to `min_distance_cm`, so the veto stays a number
        against a threshold and the safety layer's shape does not change.
        The scalar `get_distance()` remains the veto for any backend whose
        grid is all-unusable, so nothing regresses.
        """
        return unusable_grid()

    def get_odometry(self) -> dict:
        """How far this robot has travelled, and which way it now faces.

            {"usable": bool,             # False means the other two are None
             "distance_m": float|None,   # cumulative path length since start,
                                         #   monotonically non-decreasing
             "heading_deg": float|None}  # body heading, degrees, 0 = start

        **Not abstract, for the same reason `get_depth_grid()` is not.**
        Most backends here cannot measure motion -- a recorded walk is a
        list of photographs and a teleoperated phone has no encoders -- so
        the default is `unusable_odometry()`, and a backend that really
        has wheel encoders and an IMU overrides it.

        **`distance_m` is PATH LENGTH, not displacement.** A robot that
        drives a metre out and a metre back reports 2.0, not 0.0. The
        consumer is the tiered policy's cold-search interval
        (`brain/tiered.py`), which asks *"how much new ground has been
        covered since the cloud last looked"* -- and ground covered is
        path, not how far from home you ended up. Displacement would make
        a robot searching a small room never trigger.

        **Turning counts as no distance.** A pivot changes `heading_deg`
        and leaves `distance_m` alone, which is correct for a differential
        chassis (1.1) and is why the two are reported separately: a scan
        in place reveals new *view* without new *ground*, and a policy may
        want to treat those differently.

        The units are metres and degrees rather than the centimetres
        `get_distance()` uses, deliberately: that one is a proximity
        reading off a sensor with centimetre resolution, this one is an
        accumulating pose estimate that will be metres before a mission
        ends. Mixing them in one unit would guarantee an off-by-100
        somewhere.
        """
        return unusable_odometry()

    def get_wheel_state(self) -> dict:
        """Per-wheel position, velocity and encoder count -- phase R2.

            {"usable": bool,
             "left":  {"position_rad": float, "velocity_rad_s": float,
                       "counts": int} | None,
             "right": {...} | None,
             "wheel_radius_m": float | None,
             "track_width_m": float | None,
             "counts_per_rev": int | None}

        What `hardware_interface::SystemInterface.read()` returns at R4, which
        is why it is per WHEEL rather than a pose: `diff_drive_controller`
        owns the conversion to odometry, and a backend that did it itself
        would leave that conversion untested until hardware day (R0's
        argument). Positions accumulate from start; velocities are the
        standing command. The chassis constants travel with the reading so a
        consumer never pairs a count with the wrong wheel size.

        **Not abstract** -- `unusable_wheels()` for a backend with no
        encoders, the same pattern as `get_odometry()`.
        """
        return unusable_wheels()

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        """Set a STANDING wheel-velocity command -- phase R2b.

        What `hardware_interface::SystemInterface.write()` does at R4. It
        persists until changed, `stop()` zeroes it, and the robot server's
        watchdog stops it on silence. **Never call this without the safety
        layer** -- `robot/server.py`'s `POST /wheels` vets every command with
        `SafetyController.vet_wheel_velocity()` first.

        **Raises NotImplementedError by default**: a backend with no motors
        (a recorded walk, a phone) must refuse a velocity, not silently
        ignore one -- a controller that believes it is driving is worse off
        than one told it cannot.
        """
        raise NotImplementedError(f"{type(self).__name__} cannot take wheel velocities")

    def verb_plan(self, action: str, speed: int = 50, duration: float = 0.5,
                  angle: int = 90) -> Optional[dict]:
        """What a motion verb MEANS on this body, so the safety layer can
        carry it out itself, re-vetting every period -- 3.22.

        A dict: `kind` ("straight" or "turn"), the `left_rad_s` /
        `right_rad_s` the verb drives at, its `target` (metres, or degrees),
        and `wall_clock` (True if time passes on its own, False if the
        caller must `advance()` it, as in the sim).

        **None by default**, and None means "run my own verb, as before":
        right for a backend whose verbs are already guarded (`drive: ros`'s
        wrapper), are somebody else's (`RemoteRobot` -- the robot server
        guards them), or move nothing (a replay, a phone).
        """
        return None

    def verb_done(self, action: str, plan: dict, outcome: dict, **kwargs) -> dict:
        """The result of a verb the safety layer carried out from
        `verb_plan()`, in the same shape this backend's own verb returns.
        Only called on a backend that returned a plan."""
        raise NotImplementedError(f"{type(self).__name__} returned a verb plan but "
                                  "cannot report on it")

    def advance(self, dt: float) -> None:
        """Let `dt` seconds of a standing command elapse -- phase R2b.

        A no-op by default, because a real robot's wheels turn on their own
        clock. A SIMULATOR has no clock of its own, so `MockRobot` integrates
        here; the robot server calls this from its control loop, which is
        how a `POST /wheels` command becomes motion in the sim.
        """
        return None

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        """A 360-degree range scan -- phase R2, what the lidar gives.

            {"usable": bool,
             "angle_min_deg": float,        # first beam, degrees
             "angle_increment_deg": float,  # between beams
             "range_min_m": float, "range_max_m": float,
             "ranges_m": [float | None, ...]}  # one per beam

        **BODY state, which is why it is here and not on `WorldInterface`.**
        A scan is the robot's own reading -- how far everything is from ME --
        exactly as `get_depth_grid()` is; what it gets turned INTO (the map)
        is world state. `PLAN-ros-alignment.md` R2 first placed it at
        `/world/scan`; 3.6 records moving it for this reason.

        **Angles are relative to the robot's BODY heading**, clockwise and
        positive to the robot's right -- the convention every other angle in
        this project uses -- with 0 straight ahead. Body, not camera: the
        lidar sits on the deck and does not pan. A beam's entry is metres to
        the first return, or **None for "no return within range_max_m"** --
        information about empty space, and deliberately not the same thing
        as the whole scan being `usable: False`.

        **`max_range_m` is a HINT, never a filter a caller may rely on**
        (`PLAN-ros-alignment.md` 3.18). The safety layer passes it because it
        only needs the metre around the chassis; a backend MAY then report
        returns beyond it as None, and one that measures everything at once
        (a real lidar) simply ignores it. The sim honours it, because casting
        360 rays to 12m costs ~13ms of the wheel loop's 50ms period.

        **Not abstract** -- `unusable_scan()` for a backend with no lidar.
        """
        return unusable_scan()
