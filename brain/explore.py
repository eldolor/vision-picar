"""
explore.py -- the `explore` mission policy (PLAN-ros-alignment.md 3.31): a
search that uses the map.

Each step reads SLAM's map and pose, and does exactly one of these:

* **the target is in sight** -- send nav2 a goal just short of it, along the
  bearing the camera saw it on, at the range the LIDAR reads there (never
  the detector's own distance, the same rule as `brain/arrival.py`);
* **no goal is live** -- pick the best reachable frontier
  (`brain/frontier.py`) and send nav2 there;
* **a goal is live** -- let it drive, and keep looking;
* **a goal ended** -- on arrival look left and right before moving on (the
  camera sees 60 degrees; the lidar sees 360, so the MAP fills in faster
  than the camera looks); on a failure, set the place aside on the retry
  rule;
* **the map is done but the camera is not** -- the lidar sees 360 degrees
  and the camera 60, so the map runs out of frontiers before the camera has
  looked at every floor; go and look at the largest patch of known floor
  the camera has not seen (`find_view_gaps`), and turn to face it;
* **a labelled room not yet entered** (3.70 E3) -- where the map carries
  room labels (a survey's, S2), go into each labelled room the robot's
  centre has not been inside yet;
* **nothing left** -- no reachable frontier, no unseen floor, no labelled
  room left to enter, and nothing waiting out a cooldown: the mission ends
  `exhausted` (3.70 E1; `searched` before it).

**No verb is ever sent while a goal is live.** Since 3.23 a goal is an
autonomous driver and the robot server refuses every other autonomous
`/action` while one is pending or active, which `MissionRunner` would read
as a preemption. The camera pans happen between goals.

The brain reaches nav2 through a `Navigator` it declares here, not through
`WorldInterface`, whose contract says a world model is never asked to drive.
`control/remote_navigator.py` implements it over `/world/goal`.
"""

import collections
import json
import logging
import math

import httpx
import time
from typing import Callable, Optional, Protocol

from brain.agent import MissionAgent, StepResult
from brain.arrival import ArrivalCheck
from brain.frontier import (RetryBook, camera_seen, find_frontiers, find_view_gaps,
                            known_near, reach, reachable)
from brain.goal_policy import LinearGoalPolicy
from brain.goal_policy import features as goal_features
from brain.memory import MissionMemory
from robot.interface import ZONE_UNUSABLE, Preempted, RobotInterface
from robot.safety import FOOTPRINT_LENGTH_M, SafetyViolation

logger = logging.getLogger("explore")

# nav2's goal states, as `/world/goal` reports them.
ACTIVE_STATES = ("pending", "active")
TERMINAL_STATES = ("succeeded", "aborted", "canceled", "rejected")
# A goal nav2 has been driving for this long is treated as failed: the 3.30
# standoff showed nav2 will wait on a person indefinitely.
GOAL_TIMEOUT_S = 120.0
# How far short of a sighted target to stop: inside the sim's 0.9 m
# "found" radius with the target ahead, outside the 0.40 m arrival radius
# so the last metres are judged, not driven blind.
APPROACH_STANDOFF_M = 0.5
# Where to stand, if not on the straight line: the reachable floor nearest
# the robot within this band of the target. The straight-line point lands
# between chair legs often enough that nav2 refused it in the furnished
# home's dining room; floor the chassis can reach is what nav2 can plan to.
APPROACH_NEAR_M, APPROACH_FAR_M = 0.35, 0.6
# A sighting this far from the last approach point is a new one.
APPROACH_MOVED_M = 0.5
# 3.69 B: a turn to face the target that the pivot guard refuses is tried
# again at half the angle, down to this, before the robot backs off.
FACE_MIN_TURN_DEG = 10
# Camera pans after a goal succeeds.
LOOK_AROUND = ("LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER")
# A goal that fails with the robot no further than this from where it was
# sent is the ROBOT's problem (wedged in furniture), not the place's.
STUCK_MOVED_M = 0.15
# Escapes tried before a search may conclude, while boxed in.
MAX_ESCAPES = 3
# An escape's turn sizes, each tried on the roomier side and then the other,
# until the pivot guard lets one through (3.43).
OPEN_ANGLES = (90, 45, 20, 10, 5)
# 3.44: a verb asked this slowly (0-100; 0.03 m/s = robot.safety.CREEP_M_S)
# needs only CREEP_MARGIN_CM (3 cm) of room, so a robot wedged at 19 cm can
# still creep out. The escape creeps only toward an end the full bar
# already refuses -- at most 17 cm to the 3 cm line, ~6 s, inside
# RemoteRobot's 10 s request timeout (a creep verb plans a whole cell).
CREEP_SPEED = 5
# "Boxed in": fewer reachable stopping places than this around the robot.
BOXED_CELLS = 40
# 3.70 E3: a room goal aims at least this far inside the room's label where
# it can -- nav2's xy goal tolerance (nav2.yaml, 0.10 m) -- so arriving within
# tolerance still puts the robot's centre in the room. In metres, not cells:
# the sim's map is 30 cm a cell and SLAM's 5 cm.
ROOM_INSET_M = 0.10
# The robot server's refusal when the mission's OWN goal still holds the
# robot (3.23) -- not a person taking over. A move refused for this waits.
OWN_GOAL_REFUSAL = "nav2 goal is active"
# 3.76 C5: just after the mission's own goal ends, `ros` still holds the
# robot until its authority lapses (watchdog_timeout_s after nav2's last
# wheel post). A move refused for that waits too.
OWN_GOAL_HOLDER = "ros is driving"
# A goal read back through the SLAM anchor (world/ros_world.py) comes back a
# little moved; within this it is still the mission's own (3.76).
OWN_GOAL_TOLERANCE_M = 0.05
# Transient goal-call failures in a row before the search gives up (3.76):
# about ten seconds of ticks.
BRIDGE_BLIPS_MAX = 40


def leave_clearance_m(min_distance_cm: float) -> float:
    """3.45: the room a frontier or view goal keeps from anything occupied on
    the map -- the chassis' half-length plus the safety layer's stopping
    distance. Stood there, at ANY heading, the move straight ahead and the
    one straight back each have the full bar, so the robot can always drive
    away. 3.31's 0.22 m (`frontier.GOAL_CLEARANCE_M`) was sized to clear
    nav2's 0.12 m inflation, not the 20 cm bar: on the furnished-home maps
    explore really had, only 36% of the goals it chose could be left at
    every heading on ground truth, and 0.30 m still only 52% (0.33 m rounds
    up to the map's next 5 cm cell, 0.35 m: 98%). `evaluations/slam-345/goal_sweep.py`.
    Approach points toward a seen target keep 0.22 m: they are judged by
    the arrival rule, at the lidar's 0.40 m, not left again."""
    return FOOTPRINT_LENGTH_M / 2 + min_distance_cm / 100.0


# 3.45: decisions kept for a run's record (`events`), newest last.
MAX_EVENTS = 5000


class Navigator(Protocol):
    """What the explore policy needs from a planner. House frame, metres."""

    def set_goal(self, x_m: float, y_m: float) -> dict:
        """{"accepted": bool, "reason"?: str}"""

    def get_goal(self) -> dict:
        """{"goal": {"state": str, "x_m": float, "y_m": float} | None, "plan": [...]}"""

    def cancel_goal(self) -> dict:
        """Cancel a live goal; harmless with none."""


class ExploreAgent(MissionAgent):
    """Frontier search over nav2. See the module docstring."""

    def __init__(
        self,
        robot: RobotInterface,
        memory: MissionMemory,
        navigator: Optional[Navigator] = None,
        clock: Callable[[], float] = time.monotonic,
        goal_timeout_s: float = GOAL_TIMEOUT_S,
        retry: Optional[RetryBook] = None,
        goal_policy: Optional[LinearGoalPolicy] = None,
        **kwargs,
    ):
        super().__init__(robot, memory, **kwargs)
        if navigator is None:
            raise ValueError("the explore policy needs a Navigator (nav2's goals)")
        self.navigator = navigator
        self.clock = clock
        self.goal_timeout_s = goal_timeout_s
        self.retry = retry or RetryBook()
        # 3.65 phase 2: None is the rule; a policy ranks the same candidates.
        self.goal_policy = goal_policy
        self.exhausted = False         # read by MissionRunner: ends `exhausted`
        # 3.70 E3: labelled rooms the robot's centre has been inside, by the
        # map's own room labels; and how often a room goal arrived without
        # that (twice and the room is given up, as a frontier is).
        self.rooms_entered: set = set()
        self._room_goals_reached: dict = {}
        # ...and how often one FAILED: a room nav2 keeps refusing is given up
        # after the retry limit, by name -- not attacked cell by cell.
        self._room_goals_failed: dict = {}
        # 3.70 E4/E5's report: every goal reached, as [x_m, y_m, kind].
        self.goals_reached: list = []
        self._last_map: Optional[dict] = None
        self._goal: Optional[dict] = None   # {"x_m","y_m","kind","sent_at"}
        self._last_sent: Optional[tuple] = None   # the last goal it sent, live or not (3.76)
        self._attempts = collections.deque(maxlen=4)   # goals it tried to send, answered or not
        self._blips = 0
        self._pending: list = []
        self._approach: Optional[tuple] = None
        # Where the target was last seen, kept after it leaves view: the den
        # run saw the backpack, lost the approach to an escape, and spent the
        # rest of its half hour in the garage.
        self._last_seen: Optional[tuple] = None
        # Floor the camera has had a clear look at, as house-frame cell
        # centres -- not map indices, because a SLAM map grows and its
        # origin moves.
        self.seen: set = set()
        self._looked: list = []        # view-gap centres already visited
        self._backed_off: list = []    # targets a refused face has backed off from (3.69)
        self._glimpse_backoffs: list = []   # where the robot stood when a glimpse did
        # Frontier goals already reached, with how often. A frontier still
        # there after the robot has stood at it TWICE cannot be cleared from
        # there -- the unknown under a table, behind a sofa -- and
        # nearest-first would send it straight back (the first house run made
        # 50 goals at 0.36 m apiece). Once is not enough to give up: a
        # doorway is still a frontier the first time it is reached, and
        # dropping it then cut a later run off from the den.
        self._reached: list = []
        self._view_goals: list = []    # where view goals have stood the robot
        # 3.31's furnished-home runs: wedged between a sofa and a coffee
        # table, every goal aborted, the places were set aside one by one,
        # and the search ended (`searched`, now `exhausted`) with the robot
        # simply stuck.
        self.escapes = 0
        self._wedged: list = []        # goals that failed without the robot moving
        self.seen_res: Optional[float] = None
        self.goals_sent = 0
        self.goals_failed = 0
        self.last_event = ""
        # 3.45: every goal, its end and every escape or pan, as structured
        # records -- logged at INFO as `explore {json}` and kept here. 3.44's
        # live runs could not say whether an escape ran, or what explore
        # chose before a stall, because nothing reached the brain's log.
        self.events: list = []

    def _event(self, name: str, pose: Optional[dict] = None, **fields) -> None:
        pose = pose if pose is not None else self._pose()
        rec = {"event": name, "t": round(self.clock(), 2), **fields}
        if pose.get("usable"):
            rec["pose"] = [round(pose["x_m"], 2), round(pose["y_m"], 2),
                           round(pose["heading_deg"], 1)]
        self.events.append(rec)
        del self.events[:-MAX_EVENTS]
        logger.info("explore %s", json.dumps(rec, default=str))

    # ---------- one step ----------

    def step(self) -> StepResult:
        frame = self.robot.get_camera_frame()
        scene = self._review_scene(self.vision_fn(frame), frame)
        action, executed, detail = self._choose(frame, scene)
        result = StepResult(step=len(self.history), frame=frame, scene=scene,
                            action=action, executed=executed, detail=detail)
        # A GOAL (or the final STOP) is a decision and spends the step
        # budget; waiting on nav2 and the camera pans after an arrival do
        # not -- the first house run spent 150 of its 200 steps on pans.
        if action in ("GOAL", "STOP"):
            self.history.append(result)
        pose = self._pose()
        if pose.get("usable"):
            frame["pose"] = {k: pose[k] for k in ("x_m", "y_m", "heading_deg", "map_id")}
            self._note_room(pose)
        self.memory.record_observation(result.step, frame, scene)
        room = frame.get("room", "unknown")
        self.memory.mark_room_searched(room)
        if action != "WAIT":
            self.memory.record_action(result.step, room, action, executed)
        return result

    def _choose(self, frame: dict, scene: dict):
        now = self.clock()
        if self.memory.is_complete():
            try:
                self._cancel()
            except Preempted:
                pass        # 3.76: found is found; a goal that is not ours is not ours to end
            return "STOP", True, "target found"
        self._note_seen()
        # 3.70 E3: the room it stands in counts as entered BEFORE it chooses,
        # or its first room goal would be the room it is already in.
        pose = self._pose()
        if pose.get("usable"):
            self._note_room(pose)

        state = self._goal_state()
        ended = None
        if self._goal is not None:
            if state in TERMINAL_STATES or state is None:
                self._goal_ended(state or "lost", now)
                ended = self.last_event
            elif state != "unreadable" and now - self._goal["sent_at"] > self.goal_timeout_s:
                # (3.76: not while the bridge is unreadable -- an outage is
                # not the place's failure.)
                # Record the failure first (it reads the goal), then cancel.
                self._goal_ended("timeout", now)
                try:
                    self.navigator.cancel_goal()
                except Exception:  # noqa: BLE001 -- best effort, as in _cancel()
                    logger.warning("goal cancel failed", exc_info=True)

        # 3.69 A: the target glimpsed on a pan cannot be approached (a
        # panned bearing is not a body bearing), so turn the BODY to where
        # it was and look again centred; the next frame decides.
        # Never over an escape still queued: a wedged robot backs out first.
        escaping = any(p == "REVERSE" or (isinstance(p, tuple) and p[0] in ("OPEN", "CREEP"))
                       for p in self._pending)
        glimpse = self._glimpse(frame) if self._goal is None and not escaping else None
        if glimpse is not None:
            self._pending = ["LOOK_CENTER", ("FACE", glimpse, 90, False, "glimpse")]
            self._event("glimpse", toward=[round(glimpse[0], 2), round(glimpse[1], 2)])

        # A look queued after an arrival goes first: no goal is live then.
        if self._pending and self._goal is None:
            return self._do_pending()

        sighting = self._sighting(frame)
        if sighting is not None:
            tx, ty = sighting
            self._last_seen = (tx, ty)
            moved = (self._approach is None
                     or math.hypot(tx - self._approach[0], ty - self._approach[1]) > APPROACH_MOVED_M)
            if moved and self.retry.available(tx, ty, now):
                return self._send_approach(tx, ty, now)

        if self._goal is not None:
            return "WAIT", True, f"nav2 {state}: {self._goal['kind']}"

        remembered = self._back_to_sighting(now)
        if remembered is not None:
            action, executed, detail = remembered
        else:
            action, executed, detail = self._next_frontier(now)
        if ended:
            detail = f"{ended}; {detail}"
        return action, executed, detail

    # ---------- goals ----------

    def _goal_state(self) -> Optional[str]:
        if self._goal is None:
            return None
        try:
            g = (self.navigator.get_goal() or {}).get("goal") or {}
        except Exception as exc:  # noqa: BLE001 -- judged by _bridge_blip
            self._bridge_blip(exc)
            return "unreadable"
        self._blips = 0
        if g.get("x_m") is not None and not self._is_own(g):
            # 3.76: a goal someone else set (a tap on the map) replaced the
            # mission's -- live, or already finished between polls. It used
            # to be taken for the mission's own: waited on, then recorded as
            # the mission's place reached or failed.
            raise Preempted(f"a nav2 goal set by someone else replaced the mission's "
                            f"({g.get('x_m')}, {g.get('y_m')})")
        return g.get("state")

    def _bridge_blip(self, exc: Exception) -> None:
        """3.76: a goal call that failed TRANSIENTLY -- the transport, a 5xx
        (the robot server's 503 `ros_unavailable`) -- is a wait, as a map
        outage is; one failed read used to end the whole search `failed`.
        Anything else (a 401, a bug), or more than BRIDGE_BLIPS_MAX in a
        row, still raises: a goal route that never answers is not a search."""
        status = getattr(getattr(exc, "response", None), "status_code", None)
        transient = (isinstance(exc, (httpx.TransportError, ConnectionError, TimeoutError))
                     or (status is not None and status >= 500))
        self._blips += 1
        if not transient or self._blips > BRIDGE_BLIPS_MAX:
            raise exc
        logger.warning("goal call failed (%d in a row), waiting: %s", self._blips, exc)

    def _is_own(self, g: dict, own: Optional[tuple] = None) -> bool:
        """Is this live goal the mission's own -- the one it holds, or any it
        recently tried to send? Read back through the SLAM anchor a goal
        comes back a little moved (`OWN_GOAL_TOLERANCE_M`)."""
        if g.get("x_m") is None or g.get("y_m") is None:
            return False
        mine = list(self._attempts)
        if own is not None:
            mine.append(own)
        if self._goal is not None:
            mine.append((self._goal["x_m"], self._goal["y_m"]))
        if self._last_sent is not None:
            mine.append(self._last_sent)
        return any(math.hypot(g["x_m"] - x, g["y_m"] - y) < OWN_GOAL_TOLERANCE_M for x, y in mine)

    def _send(self, x_m: float, y_m: float, kind: str, now: float, **info):
        # 3.76: remembered BEFORE the call. A send whose answer is lost (a
        # read timeout after the bridge took the goal) leaves that goal
        # driving; it is still the mission's own, never "someone else's".
        self._attempts.append((x_m, y_m))
        try:
            reply = self.navigator.set_goal(x_m, y_m) or {}
        except Preempted:
            raise
        except Exception as exc:  # noqa: BLE001 -- judged by _bridge_blip
            self._bridge_blip(exc)
            self.last_event = f"goal not sent: {exc}"
            self._event("goal_refused", kind=kind, goal=[round(x_m, 2), round(y_m, 2)],
                        reason=f"error: {exc}"[:120])
            return "WAIT", False, self.last_event
        if not reply.get("accepted"):
            # Refused before nav2 saw it -- most often authority still held by
            # this mission's own last verb (it lapses on silence). Not a
            # failure of the place: wait and ask again.
            self.last_event = f"goal refused: {reply.get('reason')}"
            self._event("goal_refused", kind=kind, goal=[round(x_m, 2), round(y_m, 2)],
                        reason=reply.get("reason"))
            return "WAIT", False, self.last_event
        pose = self._pose()
        self._blips = 0
        self._last_sent = (x_m, y_m)
        self._goal = {"x_m": x_m, "y_m": y_m, "kind": kind, "sent_at": now,
                      "from": (pose["x_m"], pose["y_m"]) if pose.get("usable") else None}
        self.goals_sent += 1
        self.last_event = f"goal sent ({kind}) to ({x_m:.2f}, {y_m:.2f})"
        self._event("goal_sent", kind=kind, goal=[round(x_m, 2), round(y_m, 2)], **info)
        return "GOAL", True, self.last_event

    def _send_approach(self, tx: float, ty: float, now: float):
        pose = self._pose()
        if not pose.get("usable"):
            return "WAIT", False, "no pose to approach from"
        d = math.hypot(tx - pose["x_m"], ty - pose["y_m"])
        if d <= APPROACH_STANDOFF_M + 0.05:
            # Already as close as the approach would bring it; the found
            # rule decides from here. 3.76: and a frontier or view goal still
            # live would drive it away from the target before arrival fires.
            self._cancel()
            self._approach = (tx, ty)
            return "WAIT", True, "at the target; looking"
        gx, gy = self._approach_point(pose, tx, ty, d)
        self._cancel()
        self._approach = (tx, ty)
        out = self._send(gx, gy, "approach", now)
        if out[0] == "GOAL":
            self._goal["target"] = (tx, ty)
        return out

    def _back_to_sighting(self, now: float):
        """Return to where the target was last seen, before any frontier --
        under the same retry rule as a place, so an unreachable sighting
        costs at most RETRY_LIMIT tries."""
        if self._last_seen is None:
            return None
        tx, ty = self._last_seen
        if self.retry.dropped(tx, ty):
            self._last_seen = None
            return None
        if not self.retry.available(tx, ty, now):
            return None
        pose = self._pose()
        # "Here" is anywhere an approach could have stood it (3.69 B): a
        # spot the approach left 0.6 m out, once more, sent the same goal
        # again and again -- reached at once, never facing it -- until the
        # step budget ran out.
        if pose.get("usable") and math.hypot(tx - pose["x_m"], ty - pose["y_m"]) \
                <= APPROACH_FAR_M + 0.1:
            # Here and not seeing it: look once, then let it go. A fresh
            # sighting brings it back.
            self._last_seen = None
            self._pending = [("FACE", (tx, ty), 90, False, True)]
            return self._do_pending()
        out = self._send_approach(tx, ty, now)
        if out[0] == "GOAL":
            out = (out[0], out[1], f"back to the last sighting; {out[2]}")
        return out

    def _approach_point(self, pose: dict, tx: float, ty: float, d: float) -> tuple:
        """Reachable floor within APPROACH_NEAR_M..APPROACH_FAR_M of the
        target, nearest the robot by path; the straight-line standoff point
        if the map offers none."""
        try:
            m = self.world.get_map()
        except Exception:  # noqa: BLE001
            m = {}
        steps, start, _need = reachable(m, pose["x_m"], pose["y_m"])
        if start is not None:
            res = m["resolution_m"]
            best = None
            for (cx, cy), n in steps.items():
                px = m["origin_x_m"] + (cx + 0.5) * res
                py = m["origin_y_m"] + (cy + 0.5) * res
                if APPROACH_NEAR_M <= math.hypot(px - tx, py - ty) <= APPROACH_FAR_M:
                    if best is None or n < best[0]:
                        best = (n, px, py)
            if best is not None:
                return best[1], best[2]
        k = (d - APPROACH_STANDOFF_M) / d
        return pose["x_m"] + (tx - pose["x_m"]) * k, pose["y_m"] + (ty - pose["y_m"]) * k

    def _known_at(self, x_m: float, y_m: float) -> int:
        """How much of the map is known around a place now -- recorded with
        a failure, so `RetryBook.available()` can try it again early once the
        map has grown there (3.76: never passed, so that rule never ran)."""
        m = self._last_map                # this tick's map (`_note_seen`): no second fetch
        if not m or not m.get("usable"):
            return 0
        return known_near(m, x_m, y_m, self.retry.radius_m)

    def _next_frontier(self, now: float):
        pose = self._pose()
        try:
            m = self.world.get_map()
        except Exception as exc:  # noqa: BLE001 -- a map outage is a wait, not an end
            return "WAIT", False, f"map unreachable: {exc}"
        if not pose.get("usable") or not m.get("usable"):
            return "WAIT", False, "no map yet"

        def known(x, y):
            return known_near(m, x, y, self.retry.radius_m)

        leave = leave_clearance_m(self.safety.min_distance_cm)
        # One reachability search per choice, shared by every step below (3.77).
        rch = reach(m, pose["x_m"], pose["y_m"], clearance_m=leave)
        frontiers = find_frontiers(m, pose["x_m"], pose["y_m"], clearance_m=leave,
                                   reach_map=rch)
        skipped = {"reached twice": 0, "set aside": 0}
        if self.goal_policy is not None:
            out = self._learned_goal(m, pose, frontiers, leave, known, now, skipped, rch)
            if out is not None:
                return out
        for rank, f in enumerate(frontiers if self.goal_policy is None else ()):
            if sum(math.hypot(f.goal[0] - rx, f.goal[1] - ry) < self.retry.radius_m
                   for rx, ry in self._reached) >= 2:
                skipped["reached twice"] += 1
                continue
            if self.retry.available(f.goal[0], f.goal[1], now, known):
                return self._send(f.goal[0], f.goal[1], "frontier", now, rank=rank,
                                  candidates=len(frontiers), path_m=round(f.distance_m, 2),
                                  size_m=round(f.size_m, 2), score=round(f.score, 2),
                                  skipped=dict(skipped))
            skipped["set aside"] += 1
        seen_cells = {self._to_cell(m, c) for c in self.seen}
        for g in find_view_gaps(m, pose["x_m"], pose["y_m"], seen_cells, clearance_m=leave,
                                reach_map=rch):
            if any(math.hypot(g.centre[0] - lx, g.centre[1] - ly) < self.retry.radius_m
                   for lx, ly in self._looked):
                continue
            # A patch's centre moves as the camera sees more of it, so also
            # never stand at the same place twice to look: the den run sent
            # twelve view goals to one spot.
            if any(math.hypot(g.goal[0] - vx, g.goal[1] - vy) < self.retry.radius_m
                   for vx, vy in self._view_goals):
                continue
            if self.retry.available(g.goal[0], g.goal[1], now, known):
                out = self._send(g.goal[0], g.goal[1], "view", now,
                                 look_at=[round(g.centre[0], 2), round(g.centre[1], 2)],
                                 area_m2=round(g.area_m2, 2), path_m=round(g.distance_m, 2))
                if out[0] == "GOAL":
                    self._goal["look_at"] = g.centre
                    self._goal["face"] = g.face
                    self._view_goals.append(g.goal)
                return out
        # "Boxed in" is counted at the clearance goals need (3.45 review):
        # counted at 0.22 m, a nook with room for nav2 but none to leave by
        # found no goal, yet read as roomy -- and ended the search (then
        # `searched`) without backing out.
        room_goal = self._room_goal(m, rch, now, known)
        if room_goal is not None:
            return room_goal
        if len(rch) < BOXED_CELLS and self.escapes < MAX_ESCAPES:
            # Nowhere reachable to stop -- from HERE. Boxed in, not done.
            self._queue_escape("boxed", reachable_cells=len(rch))
            return self._do_pending()
        if self.retry.cooling(now):
            return "WAIT", True, (f"no frontier ready; retrying in "
                                  f"{self.retry.next_ready_in(now):.0f} s")
        self.exhausted = True
        self.last_event = (f"no reachable frontier left ({len(frontiers)} on the map: "
                           f"{skipped['reached twice']} reached twice, "
                           f"{skipped['set aside']} set aside or dropped)")
        self._event("exhausted", frontiers=len(frontiers), skipped=skipped,
                    rooms_entered=sorted(self.rooms_entered))
        return "STOP", True, self.last_event

    def _view_allowed(self, g) -> bool:
        return not (any(math.hypot(g.centre[0] - lx, g.centre[1] - ly) < self.retry.radius_m
                        for lx, ly in self._looked)
                    or any(math.hypot(g.goal[0] - vx, g.goal[1] - vy) < self.retry.radius_m
                           for vx, vy in self._view_goals))

    def _learned_goal(self, m, pose, frontiers, leave, known, now, skipped, rch=None):
        """3.65 phase 2: pool every frontier and view gap the rule would
        send, and send the one the policy scores highest. None when the rules
        allow none -- the caller then decides boxed, cooling or exhausted,
        exactly as under the rule."""
        r = self.retry.radius_m
        seen_cells = {self._to_cell(m, c) for c in self.seen}
        pool, feats = [], []
        for rank, f in enumerate(frontiers):
            reached = sum(math.hypot(f.goal[0] - rx, f.goal[1] - ry) < r
                          for rx, ry in self._reached)
            if reached >= 2:
                skipped["reached twice"] += 1
                continue
            if not self.retry.available(f.goal[0], f.goal[1], now, known):
                skipped["set aside"] += 1
                continue
            pool.append(("frontier", f, rank))
            feats.append(goal_features(m, pose, f.goal, kind="frontier", path_m=f.distance_m,
                                       size_m=f.size_m, seen_cells=seen_cells,
                                       failures=self.retry.failures_near(*f.goal),
                                       reached=reached, goals_sent=self.goals_sent))
        for g in find_view_gaps(m, pose["x_m"], pose["y_m"], seen_cells, clearance_m=leave,
                                reach_map=rch):
            if not self._view_allowed(g) or not self.retry.available(g.goal[0], g.goal[1],
                                                                      now, known):
                continue
            pool.append(("view", g, None))
            feats.append(goal_features(m, pose, g.goal, kind="view", path_m=g.distance_m,
                                       area_m2=g.area_m2, seen_cells=seen_cells,
                                       failures=self.retry.failures_near(*g.goal),
                                       goals_sent=self.goals_sent))
        i = self.goal_policy.best(feats)
        if i is None:
            return None
        kind, c, rank = pool[i]
        score = round(self.goal_policy.score(feats[i]), 3)
        if kind == "frontier":
            return self._send(c.goal[0], c.goal[1], "frontier", now, rank=rank,
                              candidates=len(pool), path_m=round(c.distance_m, 2),
                              size_m=round(c.size_m, 2), score=round(c.score, 2),
                              skipped=dict(skipped), learned=score)
        out = self._send(c.goal[0], c.goal[1], "view", now,
                         look_at=[round(c.centre[0], 2), round(c.centre[1], 2)],
                         area_m2=round(c.area_m2, 2), path_m=round(c.distance_m, 2),
                         candidates=len(pool), learned=score)
        if out[0] == "GOAL":
            self._goal["look_at"] = c.centre
            self._goal["face"] = c.face
            self._view_goals.append(c.goal)
        return out

    # ---------- 3.70 E3: labelled rooms ----------

    @staticmethod
    def _labels(m: dict) -> Optional[dict]:
        """The map's room-label layer, if it has a usable one: `{"names":
        [...], "cells": [index into names, or -1, per map cell]}`."""
        labels = (m or {}).get("room_labels") or None
        if (not labels or not labels.get("names")
                or len(labels.get("cells") or ()) != m.get("width", 0) * m.get("height", 0)):
            return None
        return labels

    def _note_room(self, pose: dict) -> None:
        """Record the labelled room under the robot's centre, if any."""
        m = self._last_map
        labels = self._labels(m) if m else None
        if labels is None:
            return
        i, j = self._to_cell(m, (pose["x_m"], pose["y_m"]))
        if 0 <= i < m["width"] and 0 <= j < m["height"]:
            k = labels["cells"][j * m["width"] + i]
            if 0 <= k < len(labels["names"]):
                self.rooms_entered.add(labels["names"][k])

    def _room_goal(self, m: dict, rch, now: float, known):
        """A goal into the nearest labelled room not yet entered: the
        reachable cell of it the robot can reach soonest, preferring cells
        whose neighbours within ROOM_INSET_M carry the same label -- so
        nav2's arrival tolerance cannot leave the robot's centre in the
        doorway, outside the room. None when the map has no labels, or every
        labelled room is entered, given up (reached twice without entering,
        or failed `retry.limit` times), set aside, or out of reach."""
        labels = self._labels(m)
        if labels is None:
            return None
        steps = rch.steps
        w, h, res = m["width"], m["height"], m["resolution_m"]
        cells, names = labels["cells"], labels["names"]
        n = max(1, int(math.ceil(ROOM_INSET_M / res)))

        def inset(i, j, k):
            return all(0 <= i + di < w and 0 <= j + dj < h
                       and cells[(j + dj) * w + i + di] == k
                       for di in range(-n, n + 1) for dj in range(-n, n + 1))

        best = {}
        for (i, j), d in steps.items():
            k = cells[j * w + i]
            if not 0 <= k < len(names):
                continue
            name = names[k]
            if (name in self.rooms_entered or self._room_goals_reached.get(name, 0) >= 2
                    or self._room_goals_failed.get(name, 0) >= self.retry.limit):
                continue
            rank = (0 if inset(i, j, k) else 1, d)
            if name not in best or rank < best[name][0]:
                x = m["origin_x_m"] + (i + 0.5) * res
                y = m["origin_y_m"] + (j + 0.5) * res
                if self.retry.available(x, y, now, known):
                    best[name] = (rank, x, y)
        if not best:
            return None
        name, ((_, d), x, y) = min(best.items(), key=lambda kv: kv[1][0][1])
        out = self._send(x, y, "room", now, room=name, path_m=round(d * res, 2))
        if out[0] == "GOAL":
            self._goal["room"] = name
        return out

    def _goal_ended(self, state: str, now: float) -> None:
        g, self._goal = self._goal, None
        pose = self._pose()
        moved = (round(math.hypot(pose["x_m"] - g["from"][0], pose["y_m"] - g["from"][1]), 2)
                 if g.get("from") and pose.get("usable") else None)
        self._event("goal_ended", pose=pose, kind=g["kind"], state=state,
                    goal=[round(g["x_m"], 2), round(g["y_m"], 2)],
                    seconds=round(now - g["sent_at"], 1), moved_m=moved)
        if g.get("look_at"):
            # Looked for or not, a patch is visited once: an unseen corner
            # the camera cannot get a line to must not loop the search.
            self._looked.append(g["look_at"])
        if state == "succeeded":
            self.last_event = f"reached {g['kind']} goal"
            self.goals_reached.append([round(g["x_m"], 2), round(g["y_m"], 2), g["kind"]])
            if g.get("room"):
                if pose.get("usable"):
                    self._note_room(pose)
                if g["room"] not in self.rooms_entered:
                    self._room_goals_reached[g["room"]] = (
                        self._room_goals_reached.get(g["room"], 0) + 1)
            if g["kind"] == "frontier":
                self._reached.append((g["x_m"], g["y_m"]))
                self._pending = list(LOOK_AROUND)
            elif g["kind"] == "approach" and g.get("target"):
                # nav2 takes no heading: turn to face what we came for. And if
                # that still does not settle it, the next sighting may send
                # another approach from here.
                self._pending = [("FACE", g["target"], 90, False, True)]
                self._approach = None
            elif g.get("look_at"):
                self._pending = [("FACE", g.get("face") or g["look_at"])]
            return
        target = g.get("target") or (g["x_m"], g["y_m"])
        self.goals_failed += 1
        if g.get("from") and pose.get("usable") and math.hypot(
                pose["x_m"] - g["from"][0], pose["y_m"] - g["from"][1]) < STUCK_MOVED_M:
            # It never left: the robot is wedged, not the place unreachable.
            # Do not hold it against the place -- once. A place that wedges
            # it AGAIN is one the robot cannot leave for: 3.38's acceptance
            # run sent one living-room goal eleven times, each wedged against
            # an armchair, until the mission's time ran out.
            again = any(math.hypot(target[0] - wx, target[1] - wy) < self.retry.radius_m
                        for wx, wy in self._wedged)
            self._wedged.append(target)
            if g["kind"] == "approach":
                self._approach = None
            if again or g["kind"] == "approach":
                # (An approach wedging at all counts: a sighting that wedges
                # the robot every time must still run out of tries.)
                self.retry.fail(target[0], target[1], now, known=self._known_at)
                self._room_failed(g)
            self._queue_escape("wedged", goal=[round(g["x_m"], 2), round(g["y_m"], 2)],
                               again=again)
            self.last_event = f"{g['kind']} goal {state} without moving; backing out"
            logger.info(self.last_event)
            return
        n = self.retry.fail(target[0], target[1], now, known=self._known_at)
        self._room_failed(g)
        if g["kind"] == "approach":
            self._approach = None
        self.last_event = f"{g['kind']} goal {state}; set aside (failure {n})"
        logger.info(self.last_event)

    def _room_failed(self, g: dict) -> None:
        """A room goal's failure counts against the room only when it counts
        against the place: the first wedge is forgiven, as for any goal."""
        if g.get("room"):
            self._room_goals_failed[g["room"]] = self._room_goals_failed.get(g["room"], 0) + 1

    def _cancel(self) -> None:
        if self._goal is not None:
            # 3.76: only the mission's own -- DELETE is not arbitrated, and
            # the live goal may be a person's tap that replaced it.
            try:
                live = (self.navigator.get_goal() or {}).get("goal") or {}
            except Exception:  # noqa: BLE001 -- cancel blind, as before
                live = None
            if live and live.get("x_m") is not None and not self._is_own(live):
                self._goal = None
                raise Preempted(f"a nav2 goal set by someone else replaced the mission's "
                                f"({live.get('x_m')}, {live.get('y_m')})")
            try:
                self.navigator.cancel_goal()
            except Exception:  # noqa: BLE001 -- best effort; the goal times out anyway
                logger.warning("goal cancel failed", exc_info=True)
            self._goal = None

    def _queue_escape(self, why: str, **info) -> None:
        self.escapes += 1
        self._event("escape", why=why, n=self.escapes, **info)
        self._pending = ["REVERSE", ("CREEP",), ("OPEN",)]

    def _do_pending(self):
        item = self._pending.pop(0)
        if isinstance(item, tuple) and item[0] == "OPEN":
            # Turn towards whichever side the lidar finds more room: a
            # quarter first. 3.43: if the pivot guard refuses it, the next
            # ticks try the OTHER side, then 45 and 20 degrees each way --
            # 2026-10-06's wedged runs asked for the same refused side every
            # time while the other was free (escape_sweep: 59% freed).
            attempts = item[1] if len(item) > 1 else None
            if attempts is None:
                try:
                    scan = self.robot.get_scan()
                except Exception:  # noqa: BLE001
                    scan = {}
                side = "RIGHT"
                if scan.get("usable"):
                    start, inc = scan["angle_min_deg"], scan["angle_increment_deg"]
                    def room(lo, hi):
                        return sum(min(r if r is not None else 12.0, 3.0)
                                   for i, r in enumerate(scan["ranges_m"])
                                   if lo <= start + i * inc <= hi)
                    side = "LEFT" if room(-120, -30) > room(30, 120) else "RIGHT"
                other = "LEFT" if side == "RIGHT" else "RIGHT"
                attempts = [(s, a) for a in OPEN_ANGLES for s in (side, other)]
            turned = item[2] if len(item) > 2 else 0
            side, angle = attempts[0]
            out = self._verb(side, angle=angle)
            if out[0] == "WAIT":
                self._requeue(side, ("OPEN", attempts, turned))
            elif not out[1]:
                if len(attempts) > 1:
                    self._pending.insert(0, ("OPEN", attempts[1:], turned))
            elif self._still_wedged() and turned + angle < 360:
                # A turn went through but the robot still cannot drive
                # either way: keep turning the same way, smaller if need be,
                # before swinging back (alternating rocked it in place).
                other = "LEFT" if side == "RIGHT" else "RIGHT"
                self._pending.insert(0, ("OPEN", [(side, a) for a in OPEN_ANGLES]
                                         + [(other, a) for a in OPEN_ANGLES], turned + angle))
            return out
        if isinstance(item, tuple) and item[0] == "CREEP":
            # 3.44: still wedged after the reverse -- creep toward the
            # roomier end (the other if that is refused), which frees the
            # far end and changes what the corners can swing past.
            ends = item[1] if len(item) > 1 else self._creep_ends()
            if not ends:                 # the reverse freed it: on to the turn
                return self._do_pending() if self._pending else ("WAIT", True, "not wedged")
            seen = self._creep_readings()
            out = self._verb(ends[0], speed=CREEP_SPEED)
            # 3.74: how often a creep runs, and whether the depth camera's
            # floor band had something ahead just before -- the car's numbers
            # decide whether the band's blind range needs closing (a forward
            # creep can carry a sock inside it). A record, no behaviour.
            self._event("creep", way=ends[0], executed=bool(out[1]),
                        waited=out[0] == "WAIT", **seen)
            if out[0] == "WAIT":
                self._requeue(ends[0], ("CREEP", ends))
            elif not out[1] and len(ends) > 1:
                self._pending.insert(0, ("CREEP", ends[1:]))
            return out
        if isinstance(item, tuple) and item[0] == "FACE":
            pose = self._pose()
            if not pose.get("usable"):
                return "WAIT", False, "no pose to turn from"
            tx, ty = item[1]
            # ("FACE", point, cap, backed, kind): the cap a refusal set,
            # whether this turn comes after backing off, and what the point
            # is -- True: the TARGET's place; "glimpse": only a direction to
            # it (1 m out, not where it is); absent: an unseen patch.
            cap = item[2] if len(item) > 2 else 90
            backed = len(item) > 3 and item[3]
            kind = item[4] if len(item) > 4 else None
            want = math.degrees(math.atan2(tx - pose["x_m"], -(ty - pose["y_m"])))
            turn = (want - pose["heading_deg"] + 180.0) % 360.0 - 180.0
            if abs(turn) < 5:
                return "WAIT", True, "already facing it"
            if abs(turn) > cap:
                # A verb turns at most a quarter (or the cap a refusal set);
                # queue the rest.
                self._pending.insert(0, item)
                turn = math.copysign(cap, turn)
            action = "RIGHT" if turn > 0 else "LEFT"
            out = self._verb(action, angle=int(round(abs(turn))))
            if out[0] == "WAIT":
                self._requeue(action, item)      # retry the look itself, not a bare turn
            elif not out[1]:
                # No room to turn here (3.19's pivot guard). 3.69 B: the
                # target was often already inside the found radius, just not
                # in view -- so try smaller turns, then (toward the target
                # only, once per place) back off and try again.
                self._pending = [p for p in self._pending if p is not item]
                tried = min(cap, abs(turn))
                if tried > FACE_MIN_TURN_DEG:
                    self._pending.insert(0, ("FACE", (tx, ty), tried / 2, backed, kind))
                elif kind is None:
                    pass                          # an unseen patch: give up the look
                elif kind == "glimpse":
                    # Back off and turn once more -- once per place the robot
                    # stands; never a retry failure or a forgotten sighting:
                    # the point is made up, 1 m out.
                    here = (pose["x_m"], pose["y_m"])
                    again = any(math.hypot(here[0] - bx, here[1] - by) < APPROACH_MOVED_M
                                for bx, by in self._glimpse_backoffs)
                    if backed or again:
                        if "FORWARD" in self._pending:
                            self._pending.remove("FORWARD")
                    else:
                        self._glimpse_backoffs.append(here)
                        self._pending[0:0] = ["REVERSE", ("FACE", (tx, ty), 90, True, kind),
                                              "FORWARD"]
                elif backed or any(math.hypot(tx - bx, ty - by) < APPROACH_MOVED_M
                                   for bx, by in self._backed_off):
                    # The turn after backing off failed too: the forward
                    # queued to come back in facing it would go anywhere --
                    # and the sighting counts a failure, so going back to it
                    # runs out of tries (house 319 sat beside the backpack
                    # turning, refused, until the tick cap).
                    if "FORWARD" in self._pending:
                        self._pending.remove("FORWARD")
                    self.retry.fail(tx, ty, self.clock(), known=self._known_at)
                else:
                    # Back off, turn to it with the room that gives, and come
                    # back in facing it (the guarded verb stops at the bar):
                    # re-approaching would stand it where it cannot turn.
                    self._backed_off.append((tx, ty))
                    self._pending[0:0] = ["REVERSE", ("FACE", (tx, ty), 90, True, True),
                                          "FORWARD"]
                    # If that does not show it, let the sighting go (a fresh
                    # one brings it back) -- going back to it stood the robot
                    # where it could not turn, again and again, until the
                    # step budget ran out (house 36).
                    self._last_seen = None
            return out
        return self._verb(item)

    def _creep_readings(self) -> dict:
        """What the safety layer read just before a creep (3.74), off ONE
        depth grid: the forward veto and the floor band, each as (cm,
        source), and `floor_blind` -- how many band zones read unusable,
        which is what a sock inside the band's minimum range looks like
        (the band's own answer then reads clear). Nothing on failure -- a
        record must never stop the escape."""
        out = {}
        try:
            grid = self.robot.get_depth_grid()
        except Exception:  # noqa: BLE001
            return out
        for key, read in (("forward", "forward_clearance"), ("floor", "floor_clearance")):
            try:
                cm, source = getattr(self.safety, read)(grid)
                out[key] = [cm, source]
            except Exception:  # noqa: BLE001 -- a log line, not a decision
                pass
        band = (grid or {}).get("floor") or {}
        out["floor_blind"] = sum(z.get("status") == ZONE_UNUSABLE for z in band.get("zones") or [])
        return out

    def _creep_ends(self) -> list:
        """The straight moves the full bar refuses, roomier end first --
        empty when either end is open, or a clearance is unknown."""
        try:
            f, _ = self.safety.forward_clearance()
            b, _ = self.safety.reverse_clearance()
        except Exception:  # noqa: BLE001 -- unknown: do not creep on a guess
            return []
        lim = self.safety.min_distance_cm
        if f is None or b is None or f > lim or b > lim:
            return []
        return ["REVERSE", "FORWARD"] if b >= f else ["FORWARD", "REVERSE"]

    def _still_wedged(self) -> bool:
        """Forward AND reverse both refused by the safety layer's own
        clearances -- the robot cannot drive out either way (3.43)."""
        try:
            f, _ = self.safety.forward_clearance()
            b, _ = self.safety.reverse_clearance()
        except Exception:  # noqa: BLE001 -- unknown: do not keep turning on a guess
            return False
        lim = self.safety.min_distance_cm
        return f is not None and f <= lim and b is not None and b <= lim

    def _requeue(self, action: str, item) -> None:
        """After a move waited: put the composite step back, once."""
        if self._pending and self._pending[0] == action:
            self._pending.pop(0)
        if not (self._pending and self._pending[0] is item):
            self._pending.insert(0, item)

    def _to_cell(self, m: dict, centre: tuple) -> tuple:
        res = m["resolution_m"]
        return (int(math.floor((centre[0] - m["origin_x_m"]) / res)),
                int(math.floor((centre[1] - m["origin_y_m"]) / res)))

    def _note_seen(self) -> None:
        """Add what the camera can see right now to `seen`."""
        pose = self._pose()
        if not pose.get("usable"):
            return
        try:
            m = self.world.get_map()
            pan = (self.robot.get_depth_grid() or {}).get("pan_deg") or 0.0
        except Exception:  # noqa: BLE001 -- coverage is advisory
            return
        if not m.get("usable"):
            return
        self._last_map = m
        res = m["resolution_m"]
        self.seen_res = res
        for c in camera_seen(m, pose["x_m"], pose["y_m"], pose["heading_deg"] + pan):
            self.seen.add((round(m["origin_x_m"] + (c[0] + 0.5) * res, 3),
                           round(m["origin_y_m"] + (c[1] + 0.5) * res, 3)))

    def _verb(self, action: str, **kwargs):
        """A camera pan or an escape move, between goals. Any goal still live
        on the server is cancelled first: the agent can think a goal over
        (a reply that briefly showed none) while the robot server still
        holds it, and since 3.23 that refuses every autonomous move -- which
        ended one house run `preempted`. A refusal for the mission's OWN goal
        waits and tries again; one for anything else (a person at the D-pad)
        still ends the mission."""
        # 3.76: only the mission's OWN goal (the last one it sent). A goal
        # someone else set is theirs: cancelling it (DELETE is not
        # arbitrated) deleted a person's tap; it ends the mission instead.
        try:
            live = (self.navigator.get_goal() or {}).get("goal") or {}
        except Exception:  # noqa: BLE001 -- best effort; the refusal path covers it
            live = {}
        if live.get("state") in ("pending", "active"):
            # Before this mission sent any goal, a live one is a leftover (a
            # previous mission's that its end could not cancel): cancelled,
            # as before.
            if self._last_sent is not None and not self._is_own(live, self._last_sent):
                raise Preempted(f"a nav2 goal set by someone else is live "
                                f"({live.get('x_m')}, {live.get('y_m')})")
            try:
                self.navigator.cancel_goal()
            except Exception:  # noqa: BLE001 -- best effort; the refusal path covers it
                pass
        try:
            out = action, True, self.safety.check_and_execute(action, **kwargs)
        except SafetyViolation as exc:
            out = action, False, str(exc)
        except Preempted as exc:
            if OWN_GOAL_REFUSAL not in str(exc) and not str(exc).startswith(OWN_GOAL_HOLDER):
                raise
            self._pending.insert(0, action)
            out = "WAIT", False, f"{action} waits: the mission's own goal still holds the robot"
        self._event("verb", action=action, **kwargs, executed=out[1],
                    waited=out[0] == "WAIT", detail=str(out[2])[:160])
        return out

    # ---------- perception ----------

    def _glimpse(self, frame: dict) -> Optional[tuple]:
        """A point 1 m out along where a PANNED camera saw the target, in
        the house frame -- a direction to turn the body to, never a place
        to drive to (no range: the lidar does not look where the camera
        pans). None if the camera is centred or the target is not in view,
        or if this glimpse's turn is already queued."""
        target = self.memory.target_object
        hits = [d for d in frame.get("detections") or ()
                if target and d.get("label") == target and d.get("bearing_deg") is not None]
        if not hits or any(isinstance(p, tuple) and p[0] == "FACE" for p in self._pending):
            return None
        pan = (self.robot.get_depth_grid() or {}).get("pan_deg") or 0.0
        pose = self._pose()
        if abs(pan) <= 1.0 or not pose.get("usable"):
            return None
        a = math.radians(pose["heading_deg"] + pan + hits[0]["bearing_deg"])
        return pose["x_m"] + math.sin(a), pose["y_m"] - math.cos(a)

    def _sighting(self, frame: dict) -> Optional[tuple]:
        """The target's position in the house frame, if the camera sees it
        dead ahead enough to approach: its bearing from the detection, its
        range from the lidar at that bearing. Camera centred only -- a
        panned bearing is not a body bearing (R3)."""
        target = self.memory.target_object
        if not target:
            return None
        hits = [d for d in frame.get("detections") or ()
                if d.get("label") == target and d.get("bearing_deg") is not None]
        if not hits:
            return None
        depth = self.robot.get_depth_grid() or {}
        if abs(depth.get("pan_deg") or 0.0) > 1.0:
            return None
        bearing = hits[0]["bearing_deg"]
        try:
            scan = self.robot.get_scan()
        except Exception:  # noqa: BLE001 -- no scan, no range, no approach
            return None
        if not scan.get("usable"):
            return None
        window = ArrivalCheck._window(scan, bearing)
        if ArrivalCheck._is_edge(window):
            # 3.69 C: returns that disagree are two surfaces -- something in
            # front of the target and the wall behind it -- and the median
            # took the wall (house 125: 3.76 m for a backpack at 2.7). The
            # nearest return is short of the target, never past it, so the
            # approach only brings the robot closer to look again.
            # Within the approach band, though: if the median is there too,
            # most beams are on the near surface -- the target that close,
            # one beam slipping past it -- so take the median. If only the
            # nearest is, no range at all: it may be a jamb at arm's length
            # (read as "at the target", no approach sent) and the median the
            # wall behind. Another frame decides.
            band = APPROACH_FAR_M + 0.1
            finite = [x for x in window if not math.isinf(x)]
            near = min(finite) if finite else None
            middle = ArrivalCheck._range_at(scan, bearing)
            if near is None:
                return None
            if near > band:
                r = round(near, 4)
            elif middle is not None and middle <= band:
                r = middle
            else:
                return None
        else:
            r = ArrivalCheck._range_at(scan, bearing)
        pose = self._pose()
        if r is None or not pose.get("usable"):
            return None
        a = math.radians(pose["heading_deg"] + bearing)
        return pose["x_m"] + r * math.sin(a), pose["y_m"] - r * math.cos(a)
