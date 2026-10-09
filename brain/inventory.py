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
never makes a new look. Looks are kept by the frame that made them (3.60), so
a landmark never holds more hits than frames that saw it, however its parts
were merged.

A MISS is the landmark inside the camera's field, within `MISS_RANGE_M`,
not occluded (the scan at its bearing reaches at least its range minus
`OCCLUSION_M`), on a frame the detector ran on and the lidar returned
anything on (3.60), with no observation joining it and no detection of its
label within `GROUP_DEG` of its bearing left unplaced for want of any
return there (3.60; a return beyond `MAX_RANGE_M` spares nothing). **An isolated miss is
forgiven** (3.46 amendment 2): it is held as pending and counts only when
the next independent look also misses, and then both count; any sighting
in between clears it, from a new viewpoint or not (3.60). One miss between hits is most
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

from brain.goal_pose import _wrap
from robot.safety import scan_points_cm

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


def _sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def scan_returns(scan: dict) -> list:
    """The scan as (bearing_deg, range_m) from the BODY CENTRE, the frame a
    camera bearing is in. The lidar's offset is applied by
    `robot.safety.scan_points_cm`, the one place a scan is moved off the
    lidar (3.60: this held its own copy)."""
    if not scan or not scan.get("usable"):
        return []
    # The contract's fields, defaulted as this function always did, so a
    # short scan reads as no returns rather than raising.
    scan = {"angle_min_deg": float(scan.get("angle_min_deg", -180.0)),
            "angle_increment_deg": float(scan.get("angle_increment_deg", 1.0)),
            "ranges_m": scan.get("ranges_m") or []}
    # scan_points_cm is x ahead, y LEFT; a bearing is clockwise.
    return [(math.degrees(math.atan2(-y, x)), math.hypot(x, y) / 100.0)
            for x, y in scan_points_cm(scan)]


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
    score: float = 0.0
    last_step: Optional[int] = None
    # 3.60: looks are kept by the frame that made them, so a merge can union
    # them -- a frame two parts both counted is still one look.
    _looks: dict = field(default_factory=dict)        # frame -> the label it voted
    _missed: set = field(default_factory=set)         # frames that counted a miss
    _hit_pose: Optional[dict] = None
    _miss_pose: Optional[dict] = None
    _miss_frame: Optional[int] = None                 # when _miss_pose was taken
    _pending_miss: Optional[int] = None               # the frame of a forgiven miss

    @property
    def hits(self) -> int:
        return len(self._looks)

    @property
    def misses(self) -> int:
        return len(self._missed)

    @property
    def votes(self) -> Counter:
        return Counter(self._looks.values())

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
        joined = set()      # seen in this frame (no miss for these)
        unplaced = []       # (label, body bearing) seen but with no range
        for d in detections:
            label = str(d.get("label", "")).strip().lower()
            if not label or d.get("bearing_deg") is None:
                continue
            self.observations += 1
            bearing = _wrap(pan_deg + float(d["bearing_deg"]))
            rng = range_at(returns, bearing)
            if rng is None or rng > MAX_RANGE_M:
                self.unplaced += 1
                if rng is None:
                    # Seen, with nothing to range it by: spares a miss. A far
                    # return is not that -- the lidar saw past (3.60).
                    unplaced.append((label, bearing))
                continue
            x, y = _place(pose, bearing, rng)
            lm = self._match(label, x, y)
            if lm is None:
                # 3.55: never in the list without a point -- a reader
                # (summary()) once met one between these two lines.
                lm = Landmark(id=self._next_id, room=room, first_step=step,
                              points=[(x, y, label)])
                self._next_id += 1
                self.landmarks.append(lm)
            else:
                lm.points.append((x, y, label))
            lm.last_step = step
            self._count_look(lm, label, x, y, pose, joined)
            # 3.60: seen again, from anywhere, so the last miss was the
            # detector dropping it -- not only when this look counted.
            lm._pending_miss = None
            joined.add(lm.id)
        if returns:
            # 3.60: no scan, no miss. Without one nothing seen could be
            # placed (so nothing joined) and nothing can be found occluded.
            self._misses(pose, returns, pan_deg, fov_deg, joined, unplaced)

    def _count_look(self, lm: Landmark, label: str, x: float, y: float, pose: dict,
                    joined: set) -> None:
        """This frame's look at `lm`, after the point (x, y) was added to it;
        folds in any landmarks the point now bridges (`_absorb`).

        The merged object gets ONE look from this frame, in whichever order
        the merge happens (3.55). Each landmark taking part is due its
        score, plus one look if this pose is a new view for it and it was
        not already seen this frame; the merged score is the largest due.
        Hits and votes are a look per frame by construction (3.60): looks
        are keyed by frame, and a merge unions them. With nothing to merge
        this is the plain rule: a new viewpoint is one hit."""
        frame = self.frames
        others = [o for o in self.landmarks
                  if o is not lm and o.label == label and o.near(x, y, MERGE_M)]
        parts = [lm] + others
        fresh = [o for o in parts if o.id not in joined and _new_view(o._hit_pose, pose)]
        deserved = max(min(SCORE_CLAMP, o.score + L_HIT) if o in fresh else o.score
                       for o in parts)
        self._absorb(lm, others)
        lm.score = deserved
        if fresh and frame not in lm._looks:
            lm._looks[frame] = label
        if frame in lm._looks:
            lm._hit_pose = dict(pose)

    def _match(self, label: str, x: float, y: float) -> Optional[Landmark]:
        same = [lm for lm in self.landmarks if lm.label == label and lm.near(x, y, MERGE_M)]
        if same:
            return same[0]
        other = [lm for lm in self.landmarks if lm.near(x, y, RELABEL_M)]
        return other[0] if other else None

    def _absorb(self, keep: Landmark, others: list) -> None:
        """Complete the single linkage: `others` -- same-label landmarks a
        point within `MERGE_M` of them bridges to `keep` (found once, by
        `_count_look`) -- are folded into `keep`. The score is the larger
        one, never the sum -- two landmarks of one object may hold the same
        look, so a merge must not raise belief by itself. Looks and misses
        are unions by frame, and the viewpoints are whichever part's are
        newest (3.60: the absorbed part's were dropped)."""
        for other in others:
            keep.points.extend(other.points)
            if other._looks and max(other._looks) > max(keep._looks, default=0):
                keep._hit_pose = other._hit_pose
            for frame, voted in other._looks.items():
                keep._looks.setdefault(frame, voted)
            keep._missed |= other._missed
            keep._missed -= keep._looks.keys()   # a frame that saw a part did not miss it
            keep.score = max(keep.score, other.score)
            if other._miss_frame is not None and (keep._miss_frame is None
                                                  or other._miss_frame > keep._miss_frame):
                keep._miss_pose, keep._miss_frame = other._miss_pose, other._miss_frame
            if keep.first_step is None or (other.first_step is not None
                                           and other.first_step < keep.first_step):
                keep.first_step, keep.room = other.first_step, other.room
            self.landmarks.remove(other)

    def _misses(self, pose, returns, pan_deg, fov_deg, joined, unplaced=()) -> None:
        half = fov_deg / 2 - MISS_FOV_MARGIN_DEG
        for lm in self.landmarks:
            if lm.id in joined:
                continue
            bearing, dist = _bearing_to(pose, lm.x, lm.y)
            if abs(_wrap(bearing - pan_deg)) > half or not 0.3 <= dist <= MISS_RANGE_M:
                continue
            if any(label == lm.label and abs(_wrap(b - bearing)) <= GROUP_DEG
                   for label, b in unplaced):
                continue                      # seen there, just not ranged (3.60)
            rng = range_at(returns, bearing)
            if rng is not None and rng < dist - OCCLUSION_M:
                continue                      # something in front of it
            if _new_view(lm._miss_pose, pose):
                lm._miss_pose, lm._miss_frame = dict(pose), self.frames
                if lm._pending_miss is None:
                    lm._pending_miss = self.frames   # forgiven unless the next look agrees
                    continue
                # Two looks in a row without it: both count.
                lm.score = max(-SCORE_CLAMP, lm.score + 2 * L_MISS)
                lm._missed |= {lm._pending_miss, self.frames}
                lm._pending_miss = None

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
