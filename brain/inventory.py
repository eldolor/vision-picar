"""
inventory.py -- every object a mission saw, and where (PLAN-ros-alignment.md
3.46).

A search drives through most of a house and looks at most of what is in
it, then keeps only the target. This keeps the rest: each detection in
each frame becomes an *observation*, placed on the map when the lidar can
say how far it is, and observations of one thing are fused into one
*landmark* with a belief that rises when it is seen again and falls when
the robot looks at its place and does not see it.

**Recorded and reported only.** No policy reads this; a test pins that the
search is identical with it on and off (3.46 criterion 1).

## Placement: the detector points, the lidar measures

A detection gives a class and a bearing. The range is the nearest scan
return within `RANGE_GATE_DEG` of that bearing (1.8's rule, used for the
target since R1) -- never a monocular guess and never the simulator's own
answer. The scan sees the nearest surface at the D500's height, which for
an object partly behind another is the one in front: that is an error the
car will make too, so it is measured rather than hidden. No return there,
or one beyond `MAX_RANGE_M`, leaves the observation bearing-only; it is
counted and never placed alone.

The pose is the WORLD's (`get_pose()`) at the moment the frame was taken,
not after the step's move -- `MissionMemory`'s sighting pose is read after
the action, which is fine for "which room" and wrong by a step for "where".

## Belief: why repetition only counts from a new viewpoint

Log-odds, the occupancy grid's rule applied to objects, reported as
`belief = sigmoid(score)`. A HIT adds `L_HIT` and a MISS adds `L_MISS`, but
only from a viewpoint `NEW_VIEW_M` / `NEW_VIEW_DEG` away from the last one
that counted for that landmark. Ten frames from one pose are one look: a
detector that errs on an image errs the same way on the next, so counting
frames would make a stationary robot certain of its own mistake. Time alone
never makes a new look.

A MISS is the landmark inside the camera's field, within `MISS_RANGE_M`,
not occluded (the scan at its bearing reaches at least its range minus
`OCCLUSION_M`), on a frame the detector ran on, with no observation joining
it. **An isolated miss is forgiven** (3.46 amendment 2): it is held as
pending and counts only when the next independent look also misses, and
then both count; a hit in between clears it. One miss between hits is most
often the detector dropping a box; an object that is really gone fails
every look. Detector confidence is NOT the increment: it is uncalibrated across
classes, the same reason the target gate thresholds `probability`.

## The class is a vote

Repetition shows the detector is consistent, not that it is right; a mug
called a vase every time would earn a confident vase. Each landmark counts
labels over its counted hits, reports the majority, and is `disputed` when
the runner-up holds `DISPUTE_SHARE` or more of the votes.

## Fusion

Single linkage: an observation joins the landmark of its own label that
has a member point within `MERGE_M` (so a sofa seen along its whole front
stays one landmark), and bridges any other same-label landmark it is also
that close to (3.46 sweep 1: without the bridge a bed seen from two ends
stayed two beds), else any landmark with a member within
`RELABEL_M` (a wrong label at the same place is a vote, not a new object),
else it starts a new one. The position is the mean of the member points.

## Where detections come from

`frame_detections()` reads what a frame carries: in the simulator, the
synthesised per-cell detections (1.12), grouped into one per visible
object as a detector's box would be. A real camera frame carries none until
3.46 C's on-board detector exists, so it returns None and the inventory
does nothing -- never "saw nothing". The cloud's `/navigate` answer names
no objects (only the target, at arrival), so it feeds nothing here.

## Not here

Persistence across missions (it needs map save/reload, open question 6),
re-projection after a loop closure (3.46 decides it on criterion 4's data),
and any use of the inventory to search.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from typing import Iterable, Optional

from robot.safety import LIDAR_X_M

# Every number below is a `[PLACEHOLDER]` set on the first half of 3.46's
# simulator sweep and judged on the second half.
L_HIT = 0.85            # one hit reads 0.70
L_MISS = -0.4
SCORE_CLAMP = 4.0
REPORT_BELIEF = 0.8     # two independent hits, no misses
NEW_VIEW_M = 0.5
NEW_VIEW_DEG = 30.0
MERGE_M = 0.5
RELABEL_M = 0.3
DISPUTE_SHARE = 1 / 3
RANGE_GATE_DEG = 2.0
MAX_RANGE_M = 4.0
MISS_RANGE_M = 3.0
MISS_FOV_MARGIN_DEG = 5.0
OCCLUSION_M = 0.3
DEFAULT_FOV_DEG = 60.0  # the sim camera's; a backend states its own


GROUP_DEG = 8.0         # same-label sim cells this close in bearing are one box


def frame_detections(frame: dict) -> Optional[list]:
    """`[{label, bearing_deg, confidence}]` off a frame, or None when the
    frame reports no detections at all (the detector did not run).

    The simulator reports one detection per visible CELL, so a sofa is a
    dozen. A detector reports one box per object, so same-label cells are
    chained by bearing (`GROUP_DEG`) and each chain becomes one detection
    at its mean bearing. Two chairs side by side at the same bearing merge,
    as overlapping boxes would after NMS.
    """
    raw = frame.get("detections") if isinstance(frame, dict) else None
    if raw is None:
        return None
    by_label: dict = {}
    for d in raw:
        if d.get("bearing_deg") is None or not d.get("label"):
            continue
        by_label.setdefault(str(d["label"]).strip().lower(), []).append(float(d["bearing_deg"]))
    out = []
    for label, bearings in sorted(by_label.items()):
        bearings.sort()
        group = [bearings[0]]
        for b in bearings[1:] + [None]:
            if b is not None and b - group[-1] <= GROUP_DEG:
                group.append(b)
                continue
            out.append({"label": label, "bearing_deg": sum(group) / len(group),
                        "confidence": 1.0})
            if b is not None:
                group = [b]
    return out


def _wrap(deg: float) -> float:
    d = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def _sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def scan_returns(scan: dict) -> list:
    """The scan as (bearing_deg, range_m) from the BODY CENTRE, the frame a
    camera bearing is in. The lidar sits `LIDAR_X_M` ahead of the centre, so
    each beam is moved back before it is compared with a camera bearing."""
    if not scan or not scan.get("usable"):
        return []
    a0 = float(scan.get("angle_min_deg", -180.0))
    step = float(scan.get("angle_increment_deg", 1.0))
    out = []
    for i, r in enumerate(scan.get("ranges_m") or []):
        if r is None:
            continue
        a = math.radians(a0 + i * step)
        fwd = LIDAR_X_M + r * math.cos(a)
        right = r * math.sin(a)
        out.append((math.degrees(math.atan2(right, fwd)), math.hypot(fwd, right)))
    return out


def range_at(returns: list, bearing_deg: float,
             gate_deg: float = RANGE_GATE_DEG) -> Optional[float]:
    """Nearest return within `gate_deg` of a body bearing, or None."""
    near = [r for b, r in returns if abs(_wrap(b - bearing_deg)) <= gate_deg]
    return min(near) if near else None


def _place(pose: dict, bearing_deg: float, range_m: float) -> tuple:
    """A body bearing and range from a map pose to a map point. Compass
    heading, clockwise; x east, y SOUTH (`WorldInterface.get_pose()`)."""
    h = math.radians(float(pose["heading_deg"]) + bearing_deg)
    return (float(pose["x_m"]) + range_m * math.sin(h),
            float(pose["y_m"]) - range_m * math.cos(h))


def _bearing_to(pose: dict, x: float, y: float) -> tuple:
    dx, dy = x - float(pose["x_m"]), y - float(pose["y_m"])
    compass = math.degrees(math.atan2(dx, -dy))
    return _wrap(compass - float(pose["heading_deg"])), math.hypot(dx, dy)


def _new_view(a: Optional[dict], b: dict) -> bool:
    if a is None:
        return True
    moved = math.hypot(float(b["x_m"]) - float(a["x_m"]), float(b["y_m"]) - float(a["y_m"]))
    turned = abs(_wrap(float(b["heading_deg"]) - float(a["heading_deg"])))
    return moved >= NEW_VIEW_M or turned >= NEW_VIEW_DEG


@dataclass
class Landmark:
    id: int
    room: str
    first_step: Optional[int]
    points: list = field(default_factory=list)       # (x, y, label)
    votes: Counter = field(default_factory=Counter)
    score: float = 0.0
    hits: int = 0
    misses: int = 0
    last_step: Optional[int] = None
    _hit_pose: Optional[dict] = None
    _miss_pose: Optional[dict] = None
    _pending_miss: bool = False

    @property
    def x(self) -> float:
        return sum(p[0] for p in self.points) / len(self.points)

    @property
    def y(self) -> float:
        return sum(p[1] for p in self.points) / len(self.points)

    @property
    def label(self) -> str:
        if self.votes:
            return self.votes.most_common(1)[0][0]
        return Counter(p[2] for p in self.points).most_common(1)[0][0]

    @property
    def belief(self) -> float:
        return _sigmoid(self.score)

    @property
    def disputed(self) -> bool:
        if len(self.votes) < 2:
            return False
        (_, first), (_, second) = self.votes.most_common(2)
        return second / (first + second) >= DISPUTE_SHARE

    def near(self, x: float, y: float, within: float) -> bool:
        return any(math.hypot(px - x, py - y) <= within for px, py, _ in self.points)

    def as_dict(self) -> dict:
        return {"id": self.id, "label": self.label, "x_m": round(self.x, 3),
                "y_m": round(self.y, 3), "belief": round(self.belief, 3),
                "reported": self.belief >= REPORT_BELIEF, "disputed": self.disputed,
                "hits": self.hits, "misses": self.misses, "votes": dict(self.votes),
                "room": self.room, "first_step": self.first_step,
                "last_step": self.last_step, "points": len(self.points)}


class Inventory:
    """One mission's landmarks. Feed it frames with `observe()`; read it with
    `report()`. Holds no reference to a robot or a world -- the caller hands
    in the pose and scan it took with the frame."""

    def __init__(self, map_id: Optional[str] = None):
        self.map_id = map_id
        self.landmarks: list = []
        self.frames = 0
        self.observations = 0
        self.unplaced = 0
        self.no_pose = 0
        self._next_id = 1

    # ---------- feeding ----------

    def observe(self, detections: Optional[Iterable[dict]], pose: dict, scan: dict,
                pan_deg: float = 0.0, fov_deg: float = DEFAULT_FOV_DEG,
                room: str = "unknown", step: Optional[int] = None) -> None:
        """One frame. `detections` is `[{label, bearing_deg}]` with bearings
        off the CAMERA axis, clockwise; None means the detector did not run
        on this frame (no misses can be inferred from it), [] means it ran
        and saw nothing."""
        if detections is None:
            return
        if not pose or not pose.get("usable"):
            self.no_pose += 1
            return
        if self.map_id is None:
            self.map_id = pose.get("map_id")
        elif pose.get("map_id") != self.map_id:
            # Coordinates from another map are meaningless here
            # (`get_pose()`'s map_id rule); never mix them.
            self.no_pose += 1
            return
        self.frames += 1
        returns = scan_returns(scan)
        joined = set()
        for d in detections:
            label = str(d.get("label", "")).strip().lower()
            if not label or d.get("bearing_deg") is None:
                continue
            self.observations += 1
            bearing = _wrap(pan_deg + float(d["bearing_deg"]))
            rng = range_at(returns, bearing)
            if rng is None or rng > MAX_RANGE_M:
                self.unplaced += 1
                continue
            x, y = _place(pose, bearing, rng)
            lm = self._match(label, x, y)
            if lm is None:
                lm = Landmark(id=self._next_id, room=room, first_step=step)
                self._next_id += 1
                self.landmarks.append(lm)
            lm.points.append((x, y, label))
            lm.last_step = step
            if self._absorb(lm, label, x, y) & joined:
                # 3.47: a landmark that already took this frame's look was
                # folded into `lm`, and `_absorb` carried its score and hits
                # over. Counting the look again would raise belief from one
                # frame twice.
                joined.add(lm.id)
            if lm.id not in joined and _new_view(lm._hit_pose, pose):
                lm.score = min(SCORE_CLAMP, lm.score + L_HIT)
                lm.hits += 1
                lm._pending_miss = False
                lm.votes[label] += 1
                lm._hit_pose = dict(pose)
            joined.add(lm.id)
        self._misses(pose, returns, pan_deg, fov_deg, joined)

    def _match(self, label: str, x: float, y: float) -> Optional[Landmark]:
        same = [lm for lm in self.landmarks if lm.label == label and lm.near(x, y, MERGE_M)]
        if same:
            return same[0]
        other = [lm for lm in self.landmarks if lm.near(x, y, RELABEL_M)]
        return other[0] if other else None

    def _absorb(self, keep: Landmark, label: str, x: float, y: float) -> set:
        """Complete the single linkage: a point within `MERGE_M` of other
        same-label landmarks bridges them into `keep`. The score is the
        larger one, never the sum -- two landmarks of one object may hold
        the same look, so a merge must not raise belief by itself. Returns
        the ids folded into `keep`."""
        absorbed = set()
        for other in [lm for lm in self.landmarks
                      if lm is not keep and lm.label == label and lm.near(x, y, MERGE_M)]:
            keep.points.extend(other.points)
            keep.votes.update(other.votes)
            keep.score = max(keep.score, other.score)
            keep.hits += other.hits
            keep.misses += other.misses
            if keep.first_step is None or (other.first_step is not None
                                           and other.first_step < keep.first_step):
                keep.first_step, keep.room = other.first_step, other.room
            self.landmarks.remove(other)
            absorbed.add(other.id)
        return absorbed

    def _misses(self, pose, returns, pan_deg, fov_deg, joined) -> None:
        half = fov_deg / 2 - MISS_FOV_MARGIN_DEG
        for lm in self.landmarks:
            if lm.id in joined:
                continue
            bearing, dist = _bearing_to(pose, lm.x, lm.y)
            if abs(_wrap(bearing - pan_deg)) > half or not 0.3 <= dist <= MISS_RANGE_M:
                continue
            rng = range_at(returns, bearing)
            if rng is not None and rng < dist - OCCLUSION_M:
                continue                      # something in front of it
            if _new_view(lm._miss_pose, pose):
                lm._miss_pose = dict(pose)
                if not lm._pending_miss:
                    lm._pending_miss = True        # forgiven unless the next look agrees
                    continue
                # Two looks in a row without it: both count.
                lm.score = max(-SCORE_CLAMP, lm.score + 2 * L_MISS)
                lm.misses += 2
                lm._pending_miss = False

    # ---------- reading ----------

    def report(self) -> dict:
        items = sorted((lm.as_dict() for lm in self.landmarks),
                       key=lambda d: (-d["belief"], d["label"]))
        return {
            "map_id": self.map_id,
            "reported": [d for d in items if d["reported"]],
            "candidates": [d for d in items if not d["reported"]],
            "counts": self.counts(),
            "params": {"l_hit": L_HIT, "l_miss": L_MISS, "report_belief": REPORT_BELIEF,
                       "merge_m": MERGE_M, "new_view_m": NEW_VIEW_M,
                       "new_view_deg": NEW_VIEW_DEG},
        }

    def counts(self) -> dict:
        return {"frames": self.frames, "observations": self.observations,
                "unplaced": self.unplaced, "no_pose": self.no_pose,
                "landmarks": len(self.landmarks),
                "reported": sum(lm.belief >= REPORT_BELIEF for lm in self.landmarks)}

    def summary(self) -> str:
        r = self.report()
        names = Counter(d["label"] for d in r["reported"])
        listed = ", ".join(f"{n} x{c}" if c > 1 else n for n, c in names.most_common())
        return (f"inventory: {len(r['reported'])} reported, {len(r['candidates'])} "
                f"candidates from {r['counts']['frames']} frames"
                + (f" -- {listed}" if listed else ""))
