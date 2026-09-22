"""A sighting that survives the robot turning -- P7c item 2, P25's repair.

The problem this exists for was found by an operator watching a rig walk,
not by a test: the guidance shows FORWARD, then immediately LEFT, then
FORWARD again. Measured across six walks, **the command changes on every
frame** on half of them (`walk_eval.median_command_run == 1`), with up to
eleven immediate restorations.

The cause is that a bearing is egocentric and is re-derived from scratch
every frame. Turn twenty degrees and last frame's "slightly left" is wrong
-- so the policy needs a fresh detection simply to know where the target
went, and any frame the detector misses becomes a frame with no idea.

**The target is static.** The only thing moving is the robot, and the robot
measures its own motion far more reliably than a detector recognises
objects. So a sighting is anchored once, and the bearing to it is recomputed
from odometry -- re-detection corrects drift rather than supplying the
answer (1.8's *"the detector points, the lidar measures"*, with a clock).

## Two fidelities, because monocular range is not available

`sight()` takes an OPTIONAL range.

* **With a range** (a lidar at the detected bearing, 1.8) the sighting is a
  POINT. Bearing and distance both survive translation, so driving past the
  target updates both correctly.
* **Without one** it is a DIRECTION. A direction survives rotation exactly
  and translation not at all -- which is still most of the win, because
  rotation is what breaks a bearing between two frames a third of a second
  apart, and because a robot approaching a target head-on barely changes
  the bearing by moving.

Degrading to a direction is stated rather than hidden: `is_point` says which
one is held, and `bearing_from()` is honest in both. Faking a range would
make the point drift confidently in the wrong direction, which is worse than
not having one -- the same argument `unusable_odometry()` makes.

## What it does NOT do

It does not decide anything. It holds a sighting and answers questions about
it; whether to steer, commit or give up stays with the policy. And it never
invents odometry: `update()` on an unusable reading leaves the pose alone and
`bearing_from()` keeps answering from the last good one, because a backend
driven by photographs (`ReplayRobot`, `TeleopRobot`) has no encoders and must
not silently read as "travelled 0m".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


def _wrap(deg: float) -> float:
    """To (-180, 180]. Bearings are compared and averaged, and 359 vs 1
    is a one-degree difference that arithmetic reads as 358."""
    d = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


@dataclass
class Pose:
    """Dead-reckoned position in the odom frame, metres and degrees.

    The origin is wherever the robot was when tracking started, and `heading`
    is the body heading the odometry reports -- never the view heading, since
    a pan moves no wheels (`sim/mock_robot.py` makes the same distinction).

    **The heading CONVENTION is not assumed.** `MockRobot` starts at 90 and
    a left turn DECREASES it (90 -> 0 -> 270), which is compass-style, while
    the integration below uses `cos`/`sin` as if it were the mathematical
    convention. That mismatch mirrors the map and changes no ANSWER: the
    anchor, the motion and the bearing arithmetic all use the one convention
    consistently, so bearings and ranges come out right and only the sign of
    `y` is arbitrary. Nothing outside this module consumes `x`/`y`, which is
    what makes that acceptable rather than a latent bug -- and the reason it
    is written down is that the first demo of this class misread its own
    output for exactly this reason.
    """
    x: float = 0.0
    y: float = 0.0
    heading_deg: float = 0.0


class OdomTracker:
    """`get_odometry()` samples -> a dead-reckoned `Pose`.

    The interface reports **cumulative path length**, not displacement, so
    position has to be integrated: each sample contributes the distance
    travelled since the last one, along the heading it was travelling.

    Using the NEW heading for the segment just travelled is a deliberate
    approximation and is wrong in exactly one case -- a move that turns and
    drives in the same step. On a differential chassis (1.1) a pivot adds no
    distance and a drive changes no heading, so the two never mix; the error
    is zero there and small anywhere else. Recorded because the assumption
    stops holding the moment an Ackermann chassis or a continuous-motion
    controller (1.14) arrives.
    """

    def __init__(self) -> None:
        self.pose = Pose()
        self._last_distance: Optional[float] = None
        self.samples = 0
        self.dropped = 0

    def update(self, odom: dict) -> Optional[Pose]:
        """Fold one reading in. Returns the new pose, or None if unusable.

        An unusable reading is COUNTED and otherwise ignored -- it is not a
        zero-distance move. A walk made of photographs produces nothing but
        these, and treating them as "stood still" would let a policy believe
        a stale bearing indefinitely instead of knowing it is blind.
        """
        if not odom or not odom.get("usable"):
            self.dropped += 1
            return None
        distance = odom.get("distance_m")
        heading = odom.get("heading_deg")
        if distance is None or heading is None:
            self.dropped += 1
            return None

        self.samples += 1
        heading = float(heading)
        if self._last_distance is not None:
            step = float(distance) - self._last_distance
            # Path length is monotonically non-decreasing per the interface.
            # A decrease means the robot restarted or the backend changed
            # under us; re-anchor rather than integrate a negative step
            # backwards through a heading it never drove.
            if step < 0:
                step = 0.0
            rad = math.radians(heading)
            self.pose.x += step * math.cos(rad)
            self.pose.y += step * math.sin(rad)
        self._last_distance = float(distance)
        self.pose.heading_deg = heading
        return self.pose


@dataclass
class GoalPose:
    """One anchored sighting, and the bearing to it from anywhere since."""

    x: Optional[float] = None
    y: Optional[float] = None
    direction_deg: Optional[float] = None   # world-frame, when range is unknown
    range_m: Optional[float] = None
    sightings: int = 0
    _seen_at: Optional[Pose] = field(default=None, repr=False)

    @property
    def held(self) -> bool:
        return self.x is not None or self.direction_deg is not None

    @property
    def is_point(self) -> bool:
        """True when a range was supplied, so translation is handled too."""
        return self.x is not None

    def sight(self, pose: Pose, bearing_deg: float,
              range_m: Optional[float] = None) -> None:
        """Anchor a sighting seen `bearing_deg` off the robot's nose.

        A later sighting REPLACES the earlier one rather than averaging with
        it. Averaging assumes both are measurements of the same thing and the
        newer one is not better -- but a detector that has just seen the
        target at close range is strictly better informed than one that saw
        it across a room, and drift means the older anchor is the stale one.
        Correcting drift is the entire job (P7c).
        """
        world_deg = _wrap(pose.heading_deg + bearing_deg)
        self.sightings += 1
        self._seen_at = Pose(pose.x, pose.y, pose.heading_deg)
        self.direction_deg = world_deg
        if range_m is None:
            self.x = self.y = self.range_m = None
            return
        rad = math.radians(world_deg)
        self.x = pose.x + float(range_m) * math.cos(rad)
        self.y = pose.y + float(range_m) * math.sin(rad)
        self.range_m = float(range_m)

    def bearing_from(self, pose: Pose) -> Optional[float]:
        """Degrees to steer from `pose`: negative left, positive right, 0 ahead.

        With a point this is real geometry. With a direction only, it is the
        stored world direction minus the current heading -- exact under
        rotation, and unchanged by translation, which is the honest limit.
        """
        if self.x is not None and self.y is not None:
            world = math.degrees(math.atan2(self.y - pose.y, self.x - pose.x))
        elif self.direction_deg is not None:
            world = self.direction_deg
        else:
            return None
        return _wrap(world - pose.heading_deg)

    def distance_from(self, pose: Pose) -> Optional[float]:
        """Metres to the anchored point, or None when only a direction is held.

        None is not "far" and must not be compared against an arrival
        threshold. A policy that wants to stop on arrival needs a range
        source -- which on the real robot is the lidar at the bearing this
        returns, never this class inventing one.
        """
        if self.x is None or self.y is None:
            return None
        return math.hypot(self.x - pose.x, self.y - pose.y)

    def clear(self) -> None:
        self.x = self.y = self.direction_deg = self.range_m = None
        self._seen_at = None


__all__ = ["Pose", "OdomTracker", "GoalPose"]
