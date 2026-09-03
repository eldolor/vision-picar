"""
interface.py

The abstract contract brain/ is allowed to depend on. Both the simulation
backend (sim/mock_robot.py) and the eventual real hardware backend
(robot/hardware_robot.py, added in Phase 11) implement this exact set of
methods. brain/ never imports a backend directly -- only this interface,
via robot/factory.py.
"""

from abc import ABC, abstractmethod

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
#     stop  >  manual D-pad  >  remote mission  >  local brain
#
# `stop` is not a driver -- it is everyone's right, always, and never
# claims or loses authority. The rest rank by ROLE, not by client: a
# person issuing one command at a time outranks any loop, and a loop
# running on the robot's own network outranks one running in a browser
# tab.
DRIVER_MANUAL = 30
DRIVER_AUTONOMOUS = 20
DRIVER_LOCAL = 10

DRIVER_PRIORITY = {
    "twin-dpad": DRIVER_MANUAL,
    "teleop-operator": DRIVER_MANUAL,
    "brain": DRIVER_AUTONOMOUS,
    "teleop": DRIVER_AUTONOMOUS,
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
    return {
        "rows": 1,
        "cols": cols,
        "zones": [{"status": ZONE_UNUSABLE, "distance_cm": None} for _ in range(cols)],
    }



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
