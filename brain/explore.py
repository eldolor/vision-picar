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
* **nothing left** -- no reachable frontier, no unseen floor, and nothing
  waiting out a cooldown: the mission ends `searched`.

**No verb is ever sent while a goal is live.** Since 3.23 a goal is an
autonomous driver and the robot server refuses every other autonomous
`/action` while one is pending or active, which `MissionRunner` would read
as a preemption. The camera pans happen between goals.

The brain reaches nav2 through a `Navigator` it declares here, not through
`WorldInterface`, whose contract says a world model is never asked to drive.
`control/remote_navigator.py` implements it over `/world/goal`.
"""

import logging
import math
import time
from typing import Callable, Optional, Protocol

from brain.agent import MissionAgent, StepResult
from brain.arrival import ArrivalCheck
from brain.frontier import (RetryBook, camera_seen, find_frontiers, find_view_gaps,
                            known_near, reachable)
from brain.memory import MissionMemory
from robot.interface import RobotInterface
from robot.safety import SafetyViolation

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
# Camera pans after a goal succeeds.
LOOK_AROUND = ("LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER")


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
        **kwargs,
    ):
        super().__init__(robot, memory, **kwargs)
        if navigator is None:
            raise ValueError("the explore policy needs a Navigator (nav2's goals)")
        self.navigator = navigator
        self.clock = clock
        self.goal_timeout_s = goal_timeout_s
        self.retry = retry or RetryBook()
        self.searched = False          # read by MissionRunner: ends `searched`
        self._goal: Optional[dict] = None   # {"x_m","y_m","kind","sent_at"}
        self._pending: list = []
        self._approach: Optional[tuple] = None
        # Floor the camera has had a clear look at, as house-frame cell
        # centres -- not map indices, because a SLAM map grows and its
        # origin moves.
        self.seen: set = set()
        self._looked: list = []        # view-gap centres already visited
        # Frontier goals already reached, with how often. A frontier still
        # there after the robot has stood at it TWICE cannot be cleared from
        # there -- the unknown under a table, behind a sofa -- and
        # nearest-first would send it straight back (the first house run made
        # 50 goals at 0.36 m apiece). Once is not enough to give up: a
        # doorway is still a frontier the first time it is reached, and
        # dropping it then cut a later run off from the den.
        self._reached: list = []
        self._view_goals: list = []    # where view goals have stood the robot
        self.seen_res: Optional[float] = None
        self.goals_sent = 0
        self.goals_failed = 0
        self.last_event = ""

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
        self.memory.record_observation(result.step, frame, scene)
        room = frame.get("room", "unknown")
        self.memory.mark_room_searched(room)
        if action != "WAIT":
            self.memory.record_action(result.step, room, action, executed)
        return result

    def _choose(self, frame: dict, scene: dict):
        now = self.clock()
        if self.memory.is_complete():
            self._cancel()
            return "STOP", True, "target found"
        self._note_seen()

        state = self._goal_state()
        ended = None
        if self._goal is not None:
            if state in TERMINAL_STATES or state is None:
                self._goal_ended(state or "lost", now)
                ended = self.last_event
            elif now - self._goal["sent_at"] > self.goal_timeout_s:
                # Record the failure first (it reads the goal), then cancel.
                self._goal_ended("timeout", now)
                try:
                    self.navigator.cancel_goal()
                except Exception:  # noqa: BLE001 -- best effort, as in _cancel()
                    logger.warning("goal cancel failed", exc_info=True)

        # A look queued after an arrival goes first: no goal is live then.
        if self._pending and self._goal is None:
            return self._do_pending()

        sighting = self._sighting(frame)
        if sighting is not None:
            tx, ty = sighting
            moved = (self._approach is None
                     or math.hypot(tx - self._approach[0], ty - self._approach[1]) > APPROACH_MOVED_M)
            if moved and self.retry.available(tx, ty, now):
                return self._send_approach(tx, ty, now)

        if self._goal is not None:
            return "WAIT", True, f"nav2 {state}: {self._goal['kind']}"

        action, executed, detail = self._next_frontier(now)
        if ended:
            detail = f"{ended}; {detail}"
        return action, executed, detail

    # ---------- goals ----------

    def _goal_state(self) -> Optional[str]:
        if self._goal is None:
            return None
        g = (self.navigator.get_goal() or {}).get("goal") or {}
        return g.get("state")

    def _send(self, x_m: float, y_m: float, kind: str, now: float):
        reply = self.navigator.set_goal(x_m, y_m) or {}
        if not reply.get("accepted"):
            # Refused before nav2 saw it -- most often authority still held by
            # this mission's own last verb (it lapses on silence). Not a
            # failure of the place: wait and ask again.
            self.last_event = f"goal refused: {reply.get('reason')}"
            return "WAIT", False, self.last_event
        self._goal = {"x_m": x_m, "y_m": y_m, "kind": kind, "sent_at": now}
        self.goals_sent += 1
        self.last_event = f"goal sent ({kind}) to ({x_m:.2f}, {y_m:.2f})"
        return "GOAL", True, self.last_event

    def _send_approach(self, tx: float, ty: float, now: float):
        pose = self._pose()
        if not pose.get("usable"):
            return "WAIT", False, "no pose to approach from"
        d = math.hypot(tx - pose["x_m"], ty - pose["y_m"])
        if d <= APPROACH_STANDOFF_M + 0.05:
            # Already as close as the approach would bring it; the found
            # rule decides from here.
            self._approach = (tx, ty)
            return "WAIT", True, "at the target; looking"
        gx, gy = self._approach_point(pose, tx, ty, d)
        self._cancel()
        self._approach = (tx, ty)
        out = self._send(gx, gy, "approach", now)
        if out[0] == "GOAL":
            self._goal["target"] = (tx, ty)
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

        frontiers = find_frontiers(m, pose["x_m"], pose["y_m"])
        skipped = {"reached twice": 0, "set aside": 0}
        for f in frontiers:
            if sum(math.hypot(f.goal[0] - rx, f.goal[1] - ry) < self.retry.radius_m
                   for rx, ry in self._reached) >= 2:
                skipped["reached twice"] += 1
                continue
            if self.retry.available(f.goal[0], f.goal[1], now, known):
                return self._send(f.goal[0], f.goal[1], "frontier", now)
            skipped["set aside"] += 1
        seen_cells = {self._to_cell(m, c) for c in self.seen}
        for g in find_view_gaps(m, pose["x_m"], pose["y_m"], seen_cells):
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
                out = self._send(g.goal[0], g.goal[1], "view", now)
                if out[0] == "GOAL":
                    self._goal["look_at"] = g.centre
                    self._view_goals.append(g.goal)
                return out
        if self.retry.cooling(now):
            return "WAIT", True, (f"no frontier ready; retrying in "
                                  f"{self.retry.next_ready_in(now):.0f} s")
        self.searched = True
        self.last_event = (f"no reachable frontier left ({len(frontiers)} on the map: "
                           f"{skipped['reached twice']} reached twice, "
                           f"{skipped['set aside']} set aside or dropped)")
        return "STOP", True, self.last_event

    def _goal_ended(self, state: str, now: float) -> None:
        g, self._goal = self._goal, None
        if g.get("look_at"):
            # Looked for or not, a patch is visited once: an unseen corner
            # the camera cannot get a line to must not loop the search.
            self._looked.append(g["look_at"])
        if state == "succeeded":
            self.last_event = f"reached {g['kind']} goal"
            if g["kind"] == "frontier":
                self._reached.append((g["x_m"], g["y_m"]))
                self._pending = list(LOOK_AROUND)
            elif g["kind"] == "approach" and g.get("target"):
                # nav2 takes no heading: turn to face what we came for. And if
                # that still does not settle it, the next sighting may send
                # another approach from here.
                self._pending = [("FACE", g["target"])]
                self._approach = None
            elif g.get("look_at"):
                self._pending = [("FACE", g["look_at"])]
            return
        target = g.get("target") or (g["x_m"], g["y_m"])
        self.goals_failed += 1
        n = self.retry.fail(target[0], target[1], now)
        if g["kind"] == "approach":
            self._approach = None
        self.last_event = f"{g['kind']} goal {state}; set aside (failure {n})"
        logger.info(self.last_event)

    def _cancel(self) -> None:
        if self._goal is not None:
            try:
                self.navigator.cancel_goal()
            except Exception:  # noqa: BLE001 -- best effort; the goal times out anyway
                logger.warning("goal cancel failed", exc_info=True)
            self._goal = None

    def _do_pending(self):
        item = self._pending.pop(0)
        if isinstance(item, tuple) and item[0] == "FACE":
            pose = self._pose()
            if not pose.get("usable"):
                return "WAIT", False, "no pose to turn from"
            tx, ty = item[1]
            want = math.degrees(math.atan2(tx - pose["x_m"], -(ty - pose["y_m"])))
            turn = (want - pose["heading_deg"] + 180.0) % 360.0 - 180.0
            if abs(turn) < 5:
                return "WAIT", True, "already facing it"
            if abs(turn) > 90:
                # A verb turns at most a quarter; queue the rest.
                self._pending.insert(0, item)
                turn = math.copysign(90, turn)
            action = "RIGHT" if turn > 0 else "LEFT"
            try:
                return action, True, self.safety.check_and_execute(action, angle=int(round(abs(turn))))
            except SafetyViolation as exc:
                # No room to turn here (3.19's pivot guard). Give up on this
                # look rather than ask again every step.
                self._pending = [p for p in self._pending if p is not item]
                return action, False, str(exc)
        return self._verb(item)

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
        res = m["resolution_m"]
        self.seen_res = res
        for c in camera_seen(m, pose["x_m"], pose["y_m"], pose["heading_deg"] + pan):
            self.seen.add((round(m["origin_x_m"] + (c[0] + 0.5) * res, 3),
                           round(m["origin_y_m"] + (c[1] + 0.5) * res, 3)))

    def _verb(self, action: str):
        try:
            return action, True, self.safety.check_and_execute(action)
        except SafetyViolation as exc:
            return action, False, str(exc)

    # ---------- perception ----------

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
        r = ArrivalCheck._range_at(scan, bearing)
        pose = self._pose()
        if r is None or not pose.get("usable"):
            return None
        a = math.radians(pose["heading_deg"] + bearing)
        return pose["x_m"] + r * math.sin(a), pose["y_m"] - r * math.cos(a)
