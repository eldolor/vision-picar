"""
brain/arrival.py

P7e's first half (`PLAN-ros-alignment.md` 3.11): **recognising arrival.**

Until this existed no tiered mission could end `found`. The local tier's
scene hard-codes `target_reached: False` -- rightly, since a detector alone
cannot say how far away something is -- and under the shipped asynchronous
tier a landed cloud `target_reached` is never applied either (handoff 1c:
the cloud's distance is not measured, the lidar's is), so in the sim every
mission that reached the backpack was labelled
`max_steps` or `blocked`. On a rig walk it was worse: P7e's basket, reached,
held at P = 0.998, and driven into until the step budget ran out.

The rule splits the question the way the project splits every question:
**the camera says WHICH WAY, the range sensor says HOW FAR** -- and, since
2026-10-02 (handoff 1a), the cloud says WHAT: an arrival judged here ends
`found` only once `brain/agent.py` has had a cloud call on the arrival frame
confirm the target's identity (`brain/tiered.py` `confirm_arrival()`). Arrival
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
# 3.90 A: when frames carry capture stamps, the ARRIVAL_FRAMES run must also
# span this -- today's two frames at the brain's 0.25 s tick -- so a faster
# camera does not shorten the rule (R1c). Unstamped frames count as before.
ARRIVAL_SPAN_S = 0.25
# Stamps fall on the camera's frame grid (33 ms at 30 fps) and the brain's
# tick jitters, so two ticks 0.25 s apart read stamps 0.217-0.283 s apart: a
# run this much short of the span still covers it, or most runs would wait a
# third tick on the car (review).
SPAN_SLACK_S = 0.05


def run_spans(since: Optional[float], stamp: Optional[float],
              span_s: Optional[float]) -> bool:
    """3.90 A: does a run of frames stamped from `since` to `stamp` cover
    `span_s`? Unstamped, no span set, or a negative span (the wall clock
    stepped back): counted only, True. Shared by arrival and the tier's
    `candidate_sighting` so the two rules time alike."""
    if not span_s or since is None or stamp is None or stamp < since:
        return True
    return stamp - since >= span_s - SPAN_SLACK_S
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
# 3.80.O: under explore's search with a local tier, the local VLM's located
# yes (its box's bearing) is a second source of the arrival bearing. It lands
# about once a second, not every frame, so two of its answers count as two
# frames running only if they landed within this, and the frames between
# them neither count nor break the streak.
LOCAL_FRESH_S = 3.0
SOURCE_PERCEPTION = "perception"
SOURCE_LOCAL = "local"

# Readout states.
ARRIVED = "arrived"
APPROACHING = "approaching"
NOT_JUDGED = "not_judged"
# Arrived by this rule, but the cloud did not confirm the target's identity
# on the arrival frame (handoff 2026-10-02 1a) -- set by `brain/agent.py`,
# which holds the cloud; this module never calls out.
REFUSED = "refused"
# 3.80.C: arrived, and the local VLM could not decide the identity; asked
# again on the next arrival frame (or the cloud, after `tier_local_max_unable`).
# Neither found nor refused.
CHECKING = "checking"


class ArrivalCheck:
    """Counts consecutive frames that satisfy the rule. One per mission."""

    def __init__(self, radius_m: float = ARRIVAL_RADIUS_M,
                 centre_deg: float = ARRIVAL_CENTRE_DEG,
                 frames: int = ARRIVAL_FRAMES,
                 clock=None, local_fresh_s: float = LOCAL_FRESH_S,
                 span_s: Optional[float] = ARRIVAL_SPAN_S):
        self.radius_m = radius_m
        self.centre_deg = centre_deg
        self.frames = frames
        self.span_s = span_s
        self.streak = 0
        # 3.90 A: the capture stamp of the streak's first frame (None:
        # unstamped, so the streak is counted only).
        self._streak_since: Optional[float] = None
        # 3.80.O: a clock turns the local tier's located yes into a source of
        # the bearing; without one (every policy but explore's tiered search)
        # the rule reads perception alone, as before.
        self.clock = clock
        self.local_fresh_s = local_fresh_s
        self.source: Optional[str] = None   # the running streak's: local if any frame was
        self._local_at: Optional[float] = None   # when the streak last counted a frame
        self._last: Optional[dict] = None

    def _local_answer(self, scene: dict):
        """This frame's landed local answer: a body bearing for a located
        yes, "no" for a no, else None (nothing landed, or no clock). Read by
        brain/tiered.py's readers, imported late: this module does not
        import the tier at load (the agent must not)."""
        if self.clock is None:
            return None
        from brain.tiered import local_said_no, located_local_bearing
        bearing = located_local_bearing(scene)
        if bearing is not None:
            return bearing
        return "no" if local_said_no(scene) else None

    def refresh(self) -> None:
        """3.80.O review: an identity check just took its time (up to the
        step's deadline); the streak is as fresh as the end of it, or a
        slow (or hanging) cloud would restart every local-led arrival and
        never spend B3.2's budget."""
        if self.clock is not None and self.streak > 0:
            self._local_at = self.clock()

    def _holding(self) -> bool:
        return (self.clock is not None and self._local_at is not None and self.streak > 0
                and self.source == SOURCE_LOCAL and self._last is not None
                and self.clock() - self._local_at <= self.local_fresh_s)

    def observe(self, scene: dict, robot, stamp: Optional[float] = None) -> dict:
        """Judge one frame. Returns the readout; `state` is ARRIVED only on
        the frame that completes the streak and every frame after it.
        `stamp` is the frame's `captured_at`, when it has one (3.90 A)."""
        perception = scene.get("_perception")
        if not perception:
            return self._not_judged("no local perception on this policy")
        local = self._local_answer(scene)
        if perception.get("status") == DETECTED and perception.get("bearing_deg") is not None:
            source = SOURCE_PERCEPTION
        elif isinstance(local, float):
            source = SOURCE_LOCAL
        elif local is None and self._holding():
            # Between the local tier's answers: nothing new to judge.
            if perception.get("pan_deg"):
                return self._not_judged("camera panned: bearing is not body-relative")
            # The geometry is re-read at the held bearing (review): something
            # moving in, or the robot moving, ends the hold.
            scan = robot.get_scan()
            if not scan.get("usable") or not scan.get("ranges_m"):
                return self._not_judged("no range sensor")
            window = self._window(scan, self._last["bearing_deg"])
            middle = sorted(window)[len(window) // 2]
            if math.isinf(middle) or middle > self.radius_m or self._is_edge(window):
                return self._miss("the held arrival no longer holds at the lidar")
            return {**self._last, "streak": self.streak, "range_m": round(middle, 3),
                    "reason": "holding: waiting for the local tier's next answer"}
        else:
            return self._miss("the local tier does not see the target" if local == "no"
                              else "target not detected")
        if perception.get("pan_deg"):
            return self._not_judged("camera panned: bearing is not body-relative")
        # Perception's bearing is in the camera's frame, which is the body's
        # when unpanned; the local tier's is already a body bearing.
        bearing = float(perception["bearing_deg"]) if source == SOURCE_PERCEPTION else local
        if (source == SOURCE_LOCAL and self._local_at is not None
                and self.clock() - self._local_at > self.local_fresh_s):
            self.streak = 0             # the streak's last frame is too old to run on from
            self.source = None
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
                   "radius_m": self.radius_m, "source": source}
        if not (centred and close) or edge:
            self.streak = 0
            self.source = None
            self._local_at = None
            why = ("off centre" if not centred
                   else "no return at the bearing" if range_m is None
                   else "not yet within the radius" if not close
                   else "an edge at the bearing, not one surface")
            return {"state": APPROACHING, "streak": 0, "reason": why, **readout}
        self.streak += 1
        if self.streak == 1 or stamp is None:
            self._streak_since = stamp
        if self.clock is not None:
            self._local_at = self.clock()   # the streak's last counted frame (review)
        self.source = SOURCE_LOCAL if SOURCE_LOCAL in (source, self.source) else source
        readout["source"] = self.source
        spans = run_spans(self._streak_since, stamp, self.span_s)
        state = ARRIVED if self.streak >= self.frames and spans else APPROACHING
        self._last = {"state": state, "streak": self.streak,
                      "reason": (f"target centred at {readout['range_m']} m, "
                                 f"{self.streak} frame(s) running"), **readout}
        return dict(self._last)

    def _miss(self, why: str) -> dict:
        self.streak = 0
        self.source = None
        self._local_at = None
        return {"state": APPROACHING, "streak": 0, "reason": why,
                "bearing_deg": None, "range_m": None, "radius_m": self.radius_m}

    def _not_judged(self, why: str) -> dict:
        # Not a miss either: a policy that cannot be judged has no streak.
        self.streak = 0
        self.source = None
        self._local_at = None
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
