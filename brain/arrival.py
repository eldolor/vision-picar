"""
brain/arrival.py

P7e's first half (`PLAN-ros-alignment.md` 3.11): **recognising arrival.**

Until this existed no tiered mission could end `found`. The local tier's
scene hard-codes `target_reached: False` -- rightly, since a detector alone
cannot say how far away something is -- so only a paid cloud call could end a
mission, and in the sim every mission that reached the backpack was labelled
`max_steps` or `blocked`. On a rig walk it was worse: P7e's basket, reached,
held at P = 0.998, and driven into until the step budget ran out.

The rule splits the question the way the project splits every question:
**the camera says WHAT and WHICH WAY, the range sensor says HOW FAR.** Arrival
is the target detected, centred, and the lidar reading within
`ARRIVAL_RADIUS_M` at its bearing -- on `ARRIVAL_FRAMES` consecutive frames,
so one bad frame can never end a mission. The detector's own distance
estimate is never read: in the sim that would be the simulator's geometry
leaking into the policy, and on the car a box's size is not a range.

It is the rule the car runs, not a sim stand-in. What the sim cannot test is
whether the real detector still recognises the target at 40 cm (the blue
bottle became a "vase" at close range, 4.10) -- that is a rig walk's
question, and this module needs no change to be asked it.

**It refuses to judge rather than guess** when the inputs cannot support it:
no local perception (the rule-based and cloud-only policies), no usable scan
(teleop, replay), or a panned camera -- where `bearing_deg` is no longer
body-relative (`brain/perceive.py`'s Perception note) and the beam read
would point somewhere else.
"""

import math
from typing import Optional

from brain.perceive import DETECTED
from robot.safety import LIDAR_X_M

# Measured, not chosen (3.11): over the 3.5/3.6b starts the scan at the
# target's bearing reads 0.465 m one move out and 0.165 m where the collar
# stops the robot. 0.40 m fires at the pose missions already stop at, and on
# continuous motion 40 cm out -- arrived, for a floor robot looking for an
# object on the floor.
ARRIVAL_RADIUS_M = 0.40
# The tier's steering band (brain/tiered.py STEER_BAND_DEG). Duplicated
# rather than imported so the agent does not import the tier; a test pins
# the two together.
ARRIVAL_CENTRE_DEG = 3.0
ARRIVAL_FRAMES = 2
# Beams either side of the bearing, read as a median (see _range_at). Five
# degrees at 40 cm is 3.5 cm of arc: any object worth finding covers it.
ARRIVAL_BEAM_HALF_DEG = 2
# A target's face is ONE surface: at arrival range its returns across the
# window agree to millimetres. Returns that spread by more than this, or
# that mix returns with beams that hit nothing, are an EDGE -- a door jamb
# beside a target seen through the doorway -- and are not judged (3.32:
# with guarded verbs a mission stopped 37 cm from a jamb and its median was
# the jamb, `found` 1.1 m short).
ARRIVAL_EDGE_M = 0.10

# Readout states.
ARRIVED = "arrived"
APPROACHING = "approaching"
NOT_JUDGED = "not_judged"


class ArrivalCheck:
    """Counts consecutive frames that satisfy the rule. One per mission."""

    def __init__(self, radius_m: float = ARRIVAL_RADIUS_M,
                 centre_deg: float = ARRIVAL_CENTRE_DEG,
                 frames: int = ARRIVAL_FRAMES):
        self.radius_m = radius_m
        self.centre_deg = centre_deg
        self.frames = frames
        self.streak = 0

    def observe(self, scene: dict, robot) -> dict:
        """Judge one frame. Returns the readout; `state` is ARRIVED only on
        the frame that completes the streak and every frame after it."""
        perception = scene.get("_perception")
        if not perception:
            return self._not_judged("no local perception on this policy")
        if perception.get("status") != DETECTED or perception.get("bearing_deg") is None:
            return self._miss("target not detected")
        if perception.get("pan_deg"):
            return self._not_judged("camera panned: bearing is not body-relative")
        bearing = float(perception["bearing_deg"])
        scan = robot.get_scan()
        if not scan.get("usable") or not scan.get("ranges_m"):
            return self._not_judged("no range sensor")
        window = self._window(scan, bearing)
        middle = sorted(window)[len(window) // 2]
        range_m = None if math.isinf(middle) else round(middle, 4)
        edge = self._is_edge(window)
        centred = abs(bearing) <= self.centre_deg
        close = range_m is not None and range_m <= self.radius_m
        readout = {"bearing_deg": round(bearing, 1),
                   "range_m": None if range_m is None else round(range_m, 3),
                   "radius_m": self.radius_m}
        if not (centred and close) or edge:
            self.streak = 0
            why = ("off centre" if not centred
                   else "no return at the bearing" if range_m is None
                   else "not yet within the radius" if not close
                   else "an edge at the bearing, not one surface")
            return {"state": APPROACHING, "streak": 0, "reason": why, **readout}
        self.streak += 1
        state = ARRIVED if self.streak >= self.frames else APPROACHING
        return {"state": state, "streak": self.streak,
                "reason": (f"target centred at {readout['range_m']} m, "
                           f"{self.streak} frame(s) running"), **readout}

    def _miss(self, why: str) -> dict:
        self.streak = 0
        return {"state": APPROACHING, "streak": 0, "reason": why,
                "bearing_deg": None, "range_m": None, "radius_m": self.radius_m}

    def _not_judged(self, why: str) -> dict:
        # Not a miss either: a policy that cannot be judged has no streak.
        self.streak = 0
        return {"state": NOT_JUDGED, "streak": 0, "reason": why,
                "bearing_deg": None, "range_m": None, "radius_m": self.radius_m}

    @staticmethod
    def _range_at(scan: dict, bearing_deg: float) -> Optional[float]:
        window = ArrivalCheck._window(scan, bearing_deg)
        middle = sorted(window)[len(window) // 2]
        return None if math.isinf(middle) else round(middle, 4)

    @staticmethod
    def _is_edge(window) -> bool:
        """3.32: returns that do not agree are an edge, not a face -- a
        spread over ARRIVAL_EDGE_M, or returns mixed with beams that hit
        nothing."""
        finite = [r for r in window if not math.isinf(r)]
        if not finite:
            return False
        if len(finite) < len(window):
            return True
        return max(finite) - min(finite) > ARRIVAL_EDGE_M

    @staticmethod
    def _window(scan: dict, bearing_deg: float) -> list:
        """The returns of the beams within ARRIVAL_BEAM_HALF_DEG of the
        bearing, read as a MEDIAN by `_range_at` -- a missing return counting
        as far. Scan angles are
        the bearing -- a missing return counting as far. Scan angles are
        body-frame, clockwise-positive, 0 ahead -- the same convention as a
        perception bearing.

        Median, not nearest, and criterion 2 is why: the first version took
        the minimum and declared `found` 95 cm from the backpack, stuck on
        a door jamb whose edge sat 2 degrees left of a dead-centre target
        (0.195 m against 0.81 m at the bearing). The nearest return in a
        window is whatever is nearest NEAR the target; the median is the
        target unless something covers most of the window -- in which case
        the camera could not be seeing past it either."""
        # In the BODY frame (3.27): the lidar sits LIDAR_X_M ahead of the
        # centre, so each beam is re-expressed as a bearing and range from
        # `base_link`, and the window takes the beam nearest each body
        # bearing. A beam with no return keeps its own angle. With the lidar
        # at the centre this is exactly the old beam-index lookup.
        ranges = scan["ranges_m"]
        start, step = scan["angle_min_deg"], scan["angle_increment_deg"]
        off = LIDAR_X_M
        body = []
        for i, r in enumerate(ranges):
            a = start + i * step
            if r is None:
                body.append((a, math.inf))
                continue
            x = r * math.cos(math.radians(a)) + off
            y = r * math.sin(math.radians(a))          # clockwise-positive, as the scan
            body.append((math.degrees(math.atan2(y, x)), math.hypot(x, y)))
        window = []
        for k in range(-ARRIVAL_BEAM_HALF_DEG, ARRIVAL_BEAM_HALF_DEG + 1):
            want = bearing_deg + k
            window.append(min(body, key=lambda b: abs((b[0] - want + 180.0) % 360.0 - 180.0))[1])
        return window


def arrived_scene(scene: dict, target: str, readout: dict) -> dict:
    """The scene rewritten as an arrival: STOP, target reached, and the
    target named in `important_objects` -- which is what `MissionMemory`
    reads to mark the mission found."""
    out = dict(scene)
    nav = dict(out.get("_navigate") or {})
    nav["target_reached"] = True
    nav["target_visible"] = True
    nav["reasoning"] = f"arrived -- {readout['reason']}; " + nav.get("reasoning", "")
    out["_navigate"] = nav
    out["important_objects"] = [target]
    out["safest_direction"] = "STOP"
    out.pop("turn_deg", None)
    return out
