"""
world/ros_world.py

Phase R5 (`PLAN-ros-alignment.md` 3.14) -- the world, as `slam_toolbox`
sees it. The file `PLAN-mapping.md` named for N6: an HTTP client of the ROS
container's bridge, and a `WorldInterface` like any other, so the robot
server, the brain and the twin read a SLAM map through the same routes they
read `MockWorld`'s.

**It converts on its side of the wall** and nowhere else. The bridge answers
in ROS's own terms -- x forward, y LEFT, yaw counter-clockwise, an
OccupancyGrid of 0-100 and -1 -- and this module turns that into the
project's x-east / y-SOUTH / clockwise-compass convention and its tri-state
cells (`world/interface.py`). Nothing here imports ROS.

**Aligning SLAM's frame to the house (sim only).** SLAM's map frame begins
wherever the robot stood when the container started. On hardware that frame
IS the map, and `map_id` says which one. In the sim the estimate must be
subtracted from the truth, so this lays SLAM's frame on the house using the
truth AT THE SESSION'S START (the bridge records it at odometry zero): the
"align the first pose" of trajectory evaluation. After that the truth is
never read to produce a pose -- the anchor is a frame choice, not a
correction. A new session (the container restarted: a new map) re-anchors.
"""

import logging
import math
import threading
import time
from typing import NamedTuple, Optional, Tuple

import httpx
import numpy as np

from world.bridge_faults import classify

from world.interface import (
    CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN, WorldInterface, unusable_map,
    unusable_pose, unusable_truth)

logger = logging.getLogger("ros_world")

# OccupancyGrid probabilities are 0-100. slam_toolbox writes 100 and 0 for
# the confident cases; these cut the middle the way nav2's map server does.
OCCUPIED_AT = 65
FREE_BELOW = 25
# A pose whose transform is older than this is not a pose (3.76 H1-13): the
# bridge answers tf2's LATEST transform, and with slam_toolbox or
# ros2_control dead that is frozen. Normal ages are under 0.1 s (TF at
# 20 Hz and slam_toolbox's 0.05 s publish period).
POSE_STALE_S = 1.0
# nav2 publishes the global costmap at 1 Hz (nav2.yaml); one this old is
# from a nav2 that stopped (3.83). Read by the bridge's clock (`age_s`).
COSTMAP_STALE_S = 5.0
# How often an unanchored session may ask the truth again (3.76 round 7).
TRUTH_RETRY_S = 1.0


class Anchor(NamedTuple):
    """SLAM's frame laid on the house: ours_world = R(alpha) . ours_slam + t,
    heading + alpha. Installed once per session and never changed (3.76
    round 5). `epoch` is added to map_version; it stays 0 while a session's
    anchor cannot change, and is kept so a future re-anchor has to count."""
    session: Optional[str]
    alpha: float
    t: Tuple[float, float]
    anchored_at: Optional[str]
    epoch: int = 0


def _stale(pose: dict) -> bool:
    age = pose.get("age_s")             # absent from a bridge that predates it
    return age is not None and age > POSE_STALE_S


def _ros_to_ours(x: float, y: float, yaw: float):
    """ROS (x fwd, y left, yaw CCW) -> ours in the SAME frame (x, y flipped
    to point the other way, compass heading clockwise with 0 along -y)."""
    return x, -y, (90.0 - math.degrees(yaw)) % 360.0


def _ours_to_ros(x: float, y: float):
    """The inverse of `_ros_to_ours()` for a position. Both directions of
    the flip live here, next to each other, so they cannot disagree."""
    return x, -y


def _body(r: httpx.Response):
    """The reply's JSON object, or None."""
    try:
        body = r.json()
    except ValueError:
        return None
    return body if isinstance(body, dict) else None


def _whole_grid(g: dict) -> bool:
    """A grid record that can be placed: its geometry present and its data
    exactly width x height. A short or malformed one is no grid, never a
    500 or a grid read out of bounds."""
    try:
        w, h, version, data = g["width"], g["height"], g["version"], g["data"]
        geometry = [g[k] for k in ("resolution_m", "origin_x_m", "origin_y_m",
                                   "origin_yaw_rad")]
    except (KeyError, TypeError):
        return False
    whole = lambda v: isinstance(v, int) and not isinstance(v, bool)   # noqa: E731
    real = lambda v: (isinstance(v, (int, float)) and not isinstance(v, bool)   # noqa: E731
                      and math.isfinite(v))
    return (whole(w) and whole(h) and whole(version) and w > 0 and h > 0
            and all(real(v) for v in geometry) and geometry[0] > 0
            and isinstance(data, list) and len(data) == w * h)


def _cells_in_range(data: list) -> bool:
    """Every cell an integer in -1..100, OccupancyGrid's range for a map and
    for nav2's published costmap alike. Checked before placing, so a bad
    cell is no grid and a bug in the placement still raises."""
    try:
        a = np.asarray(data)
    except (TypeError, ValueError):
        return False
    return (a.dtype.kind in "iu" and a.ndim == 1
            and bool(a.size == 0 or (a.min() >= -1 and a.max() <= 100)))


def _json_object(r: httpx.Response) -> dict:
    """A bridge reply as a dict, or an httpx.HTTPError -- the one exception
    every caller already catches. Anything but a 2xx raises, naming the
    route and keeping the bridge's own `error` text (its 503 "nav2 is not
    up"); a body that is not a JSON object (a proxy's HTML page) is a
    fault, never an answer (3.76 H, round 10's review)."""
    body = _body(r)
    where = f"{r.request.method} {r.request.url.path}"
    if not r.is_success:
        why = body.get("error") if body else None
        raise httpx.HTTPStatusError(
            f"the ROS bridge answered {r.status_code} to {where}" + (f": {why}" if why else ""),
            request=r.request, response=r)
    if body is None:
        raise httpx.DecodingError(f"the ROS bridge's reply to {where} is not a JSON object",
                                  request=r.request)
    return body


class RosWorld(WorldInterface):
    def __init__(self, bridge_url: str, secret: str = "", truth: Optional[WorldInterface] = None,
                 timeout_s: float = 2.0, client: Optional[httpx.Client] = None):
        headers = {"x-app-secret": secret} if secret else {}
        self._http = client or httpx.Client(base_url=bridge_url.rstrip("/"), headers=headers,
                                            timeout=timeout_s)
        self._truth = truth
        # The anchor, replaced whole under _lock and never mutated, so a
        # reader on FastAPI's threadpool converts with one consistent anchor
        # (3.76 H1-5). One per session, installed once (round 5).
        self._lock = threading.Lock()
        self._anchor = Anchor(None, 0.0, (0.0, 0.0), None, 0)
        # The bridge's `at` on the reply that installed the anchor (3.76
        # R8): a late reply from an older session must not re-install it.
        self._anchor_at: Optional[float] = None
        self._map_cache = None
        self._costmap_cache = None
        # The last first-contact truth read, (session, monotonic time), so an
        # unanchored session asks the truth at most every TRUTH_RETRY_S
        # rather than on every pose read on the motion path (round 7), and
        # the session whose truth failure has been logged (round 8).
        self._truth_tried: dict = {}     # session -> monotonic time of its last failed read
        self._truth_reading: set = set()     # sessions with a truth read in flight
        self._truth_warned: set = set()          # {(session, kind)}

    @property
    def _session(self) -> Optional[str]:
        return self._anchor.session

    @property
    def anchored_at(self) -> Optional[str]:
        return self._anchor.anchored_at

    # ---------- the anchor ----------

    def _apply(self, x: float, y: float, heading: float, anchor: Optional[Anchor] = None):
        anchor = anchor or self._anchor
        alpha, t = anchor.alpha, anchor.t
        c, s = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
        return (c * x - s * y + t[0], s * x + c * y + t[1], (heading + alpha) % 360.0)

    def _fetch_pose(self):
        r = self._http.get("/slam/pose")
        return _json_object(r)

    def _ensure_session(self, reply: dict) -> Optional[Anchor]:
        """The anchor for this reply's session -- installed once, on the
        session's first usable reply, and never changed -- or None if there
        is nothing to anchor on yet.

        The anchor is the truth AT THE SESSION'S START -- odometry zero,
        where SLAM's map frame begins -- as recorded by the bridge. It must
        not be the truth at first contact: a consumer that first asks after
        the robot has driven would fold everything SLAM got wrong so far into
        the frame, and read 0.0 cm. (The twin did exactly that on the first
        phone-size check, 2026-09-26.) At odometry zero SLAM's pose is the
        origin with yaw 0, which is compass 90 in SLAM's own frame.

        So while the bridge says its start-truth thread is still working
        (`start_truth_pending`), nothing is installed and the answer is None
        (3.76 round 5): the pose, the map and goals are unusable for those
        few seconds rather than anchored at first contact and moved later.
        Every way of moving an installed anchor raced something -- a goal
        being converted, a point chosen on the old map, a failed post -- and
        a moved anchor moves every position a consumer holds. A bridge that
        has given up (or predates the field) anchors at first contact, once.
        """
        current = self._anchor
        session, start, at = reply.get("session"), reply.get("start_truth"), reply.get("at")
        if session == current.session:
            return current
        if reply.get("map") is None:
            return None
        if start is None and reply.get("start_truth_pending") and self._truth is not None:
            return None                      # the start truth is coming: wait for it
        # Computed outside the lock: the first-contact truth is an HTTP call
        # in the 3.36 body process. Published only if no one beat us to it.
        new = self._anchor_for(session, reply, start)
        if new is None:
            return None
        with self._lock:
            if (session != self._anchor.session and at is not None
                    and self._anchor_at is not None and at < self._anchor_at):
                return None                  # a late reply from an older session (R8)
            if self._anchor is current:
                self._anchor = new
                if at is not None:
                    self._anchor_at = at
                if any(k[0] == session for k in self._truth_warned):
                    logger.info("SLAM session %s anchored (%s) after its truth "
                                "failures", session, new.anchored_at)
                # Only the current session's warnings matter: no growth
                # across container restarts.
                self._truth_warned = {k for k in self._truth_warned if k[0] == session}
                self._truth_tried = {}
            elif self._anchor.session != session:
                return None                  # a newer session landed meanwhile
            return self._anchor

    def _warn_truth_once(self, session, kind: str, why: str) -> None:
        """Once per (session, kind) -- an exception after an unusable answer
        is still logged (round 10)."""
        with self._lock:
            if (session, kind) in self._truth_warned:
                return
            self._truth_warned.add((session, kind))
        logger.warning("SLAM session %s: %s; unanchored, asking again every %.0f s",
                       session, why, TRUTH_RETRY_S)

    def _anchor_for(self, session, reply: dict, start) -> Optional[Anchor]:
        if start is not None and self._truth is not None:
            alpha = (start["heading_deg"] - 90.0 + 180.0) % 360.0 - 180.0
            return Anchor(session, alpha, (start["x_m"], start["y_m"]), "session_start")
        if self._truth is not None:
            # No start truth, and none coming (the bridge gave up, or
            # predates it): first contact, and say so, because the error it
            # produces can be too small.
            est = reply.get("map")
            if est is None:
                return None
            x, y, h = _ros_to_ours(est["x_m"], est["y_m"], est["yaw_rad"])
            with self._lock:                 # round 9; single-flight since round 10's review
                if time.monotonic() - self._truth_tried.get(session, -math.inf) < TRUTH_RETRY_S:
                    return None              # failed moments ago: unanchored for now
                if session in self._truth_reading:
                    return None              # another reader is asking right now
                self._truth_reading.add(session)
            failed = "the truth answered nothing"
            truth = None
            try:
                truth = self._truth.get_truth()
            except Exception as e:  # noqa: BLE001 -- the body process down (RemoteGrid raises)
                failed = f"the truth could not be read ({e})"
            finally:
                # One critical section: this session's flag cleared (never
                # another session's) and a FAILED read stamped when it ended,
                # so no reader slips between the two (round 10's review). A
                # healthy read is never followed by a dropout.
                with self._lock:
                    self._truth_reading.discard(session)
                    if truth is None or not truth.get("usable"):
                        self._truth_tried = {session: time.monotonic()}   # only the current session: no growth
            if truth is None:
                self._warn_truth_once(session, "error", failed)
                return None                  # unanchored (round 7)
            if not truth.get("usable"):
                self._warn_truth_once(session, "unusable", "the truth answered usable: false")
            if truth.get("usable"):
                alpha = (truth["heading_deg"] - h + 180.0) % 360.0 - 180.0
                ax, ay, _ = self._apply(x, y, h, Anchor(session, alpha, (0.0, 0.0), None))
                return Anchor(session, alpha, (truth["x_m"] - ax, truth["y_m"] - ay),
                              "first_contact")
            # A truth that answered unusable this once (round 6): install
            # nothing and ask again after TRUTH_RETRY_S. An identity anchor here would
            # hold for the whole session, since an anchor never moves.
            return None
        return Anchor(session, 0.0, (0.0, 0.0), None)      # no truth at all: hardware

    @property
    def map_id(self) -> Optional[str]:
        return f"slam-{self._session}" if self._session else None

    # ---------- WorldInterface ----------

    def get_pose(self) -> dict:
        try:
            reply = self._fetch_pose()
        except httpx.HTTPError:
            return unusable_pose()
        anchor = self._ensure_session(reply)
        est = reply.get("map")
        if anchor is None or est is None or _stale(est):
            return unusable_pose()
        x, y, h = self._apply(*_ros_to_ours(est["x_m"], est["y_m"], est["yaw_rad"]), anchor)
        return {"usable": True, "map_id": f"slam-{anchor.session}", "x_m": x, "y_m": y,
                "heading_deg": round(h, 4)}

    def get_odom_pose(self) -> dict:
        """diff_drive_controller's dead reckoning, in the same anchored frame
        -- what the robot would believe with no SLAM. Not part of the
        interface: it exists so R5 can show what SLAM corrects."""
        try:
            reply = self._fetch_pose()
        except httpx.HTTPError:
            return unusable_pose()
        anchor = self._ensure_session(reply)
        o = reply.get("odom")
        if anchor is None or o is None or _stale(o):
            return unusable_pose()
        # odom and map coincide at SLAM's start, so the same anchor applies.
        x, y, h = self._apply(*_ros_to_ours(o["x_m"], o["y_m"], o["yaw_rad"]), anchor)
        return {"usable": True, "map_id": f"slam-{anchor.session}", "x_m": x, "y_m": y,
                "heading_deg": round(h, 4)}

    def get_truth(self) -> dict:
        return self._truth.get_truth() if self._truth else unusable_truth()

    def get_map(self) -> dict:
        return self._placed("/slam/map", "_map_cache", self._resample)

    def _placed(self, path: str, cache_attr: str, place, max_age_s=None) -> dict:
        """A bridge grid (`/slam/map`, `/nav/costmap`) placed on the house:
        fetched, checked, and placed by `place(record, anchor)` once per
        (session, version, anchor). `usable: false` when there is none to
        place -- another session's, malformed, older than `max_age_s` by the
        bridge's own clock, or a bridge that cannot answer."""
        try:
            if self._session is None:
                self._ensure_session(self._fetch_pose())
            g = _json_object(self._http.get(path))
        except httpx.HTTPError as e:
            # No grid, and whether waiting can bring one (3.82): explore ends
            # its mission on `retryable: false` instead of waiting it out.
            return {**unusable_map(), **classify(e)}
        anchor = self._anchor
        if g.get("session") != anchor.session or not _whole_grid(g):
            return unusable_map()
        age = g.get("age_s")
        if max_age_s is not None and isinstance(age, (int, float)) and not age <= max_age_s:
            return {**unusable_map(), "stale": True, "age_s": age}
        key = (g["session"], g["version"], anchor)
        cache = getattr(self, cache_attr)
        if cache and cache[0] == key:
            return cache[1]
        if not _cells_in_range(g["data"]):
            return unusable_map()
        out = place(g, anchor)
        setattr(self, cache_attr, (key, out))
        return out

    def _resample(self, m: dict, anchor=None) -> dict:
        """SLAM's grid, re-expressed in the house frame at the same
        resolution. Nearest-neighbour: every output cell asks which SLAM
        cell its centre falls in (`_sample_index()`). Exact for the
        rotations the anchor makes at a cardinal start, and honest (never
        invents a state) otherwise."""
        anchor = anchor or self._anchor
        geo, idx = self._sample_index(m, anchor)
        data = np.asarray(m["data"], dtype=np.int16)
        v = data[np.maximum(idx, 0)]
        cells = np.where(v < 0, CELL_UNKNOWN,
                         np.where(v >= OCCUPIED_AT, CELL_OCCUPIED,
                                  np.where(v < FREE_BELOW, CELL_FREE, CELL_UNKNOWN)))
        cells = np.where(idx >= 0, cells, CELL_UNKNOWN)
        return {"usable": True, "map_id": f"slam-{anchor.session}",
                "map_version": m["version"] + anchor.epoch, **geo,
                "cells": cells.tolist()}

    # ---------- 3.83: nav2's global costmap (not part of WorldInterface) ----------
    #
    # For the twin's costmap layer. A costmap is nav2's, so like goals it
    # lives on this backend alone and the robot server reaches it by duck
    # typing. It is in SLAM's map frame, so it is placed with the map's own
    # anchor and resampled cell for cell the way the map is.

    def get_costmap(self) -> dict:
        """nav2's global costmap in the house frame: `usable`, `map_id`,
        `costmap_version`, the grid geometry, and `costs` -- one per cell,
        row-major, as nav2 publishes them (-1 unknown, 0 free, 1-98
        inflation, 99 inscribed, 100 lethal). `usable: false` when there is
        none to place: before nav2's first publish, from another session,
        when the bridge cannot answer (with 3.82's `retryable`), or when
        nav2 has not published for `COSTMAP_STALE_S` (`stale: true`): a
        stopped nav2's last costmap is not today's."""
        return self._placed("/nav/costmap", "_costmap_cache", self._place_costmap,
                            max_age_s=COSTMAP_STALE_S)

    def _place_costmap(self, c: dict, anchor) -> dict:
        geo, idx = self._sample_index(c, anchor)
        data = np.asarray(c["data"], dtype=np.int16)
        costs = np.where(idx >= 0, data[np.maximum(idx, 0)], -1)
        return {"usable": True, "map_id": f"slam-{anchor.session}",
                "costmap_version": c["version"] + anchor.epoch, **geo,
                "costs": costs.tolist()}

    def _sample_index(self, m: dict, anchor):
        """The one placement of a SLAM-frame grid on the house: the
        house-frame grid's geometry and, for each of its cells, the index of
        the SLAM-frame cell its centre falls in (-1 outside). The map and
        the costmap both read through it, so the two layers cannot disagree
        by a cell (inflation drawn inside a wall). Vectorised in 3.83 with
        the old per-cell loop's arithmetic in the same order: 0 differences
        against the frozen copy in tests/ros_resample_reference.py."""
        res, w, h = m["resolution_m"], m["width"], m["height"]
        ox, oy, oyaw = m["origin_x_m"], m["origin_y_m"], m["origin_yaw_rad"]
        alpha, t = anchor.alpha, anchor.t
        cyaw, syaw = math.cos(oyaw), math.sin(oyaw)

        def corner(i, j):
            gx, gy = (i + 0.5) * res, (j + 0.5) * res
            rx, ry = ox + cyaw * gx - syaw * gy, oy + syaw * gx + cyaw * gy
            x, y, _ = _ros_to_ours(rx, ry, 0.0)
            wx, wy, _ = self._apply(x, y, 0.0, anchor)
            return wx, wy

        corners = [corner(i, j) for i in (-0.5, w - 0.5) for j in (-0.5, h - 0.5)]
        min_x = min(c[0] for c in corners)
        min_y = min(c[1] for c in corners)
        out_w = int(math.ceil((max(c[0] for c in corners) - min_x) / res))
        out_h = int(math.ceil((max(c[1] for c in corners) - min_y) / res))
        ca, sa = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
        rows, cols = np.mgrid[0:out_h, 0:out_w].astype(np.float64)
        wx, wy = min_x + (cols + 0.5) * res, min_y + (rows + 0.5) * res
        dx, dy = wx - t[0], wy - t[1]
        x, y = ca * dx + sa * dy, -sa * dx + ca * dy
        rx, ry = _ours_to_ros(x, y)
        rx, ry = rx - ox, ry - oy
        gx, gy = cyaw * rx + syaw * ry, -syaw * rx + cyaw * ry
        i, j = np.floor(gx / res).astype(np.int64), np.floor(gy / res).astype(np.int64)
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        idx = np.where(inside, j * w + i, -1).ravel()
        geo = {"resolution_m": res, "width": out_w, "height": out_h,
               "origin_x_m": min_x, "origin_y_m": min_y}
        return geo, idx

    # ---------- R6: goals (not part of WorldInterface) ----------
    #
    # A goal is a request to nav2, so it lives on this backend alone and the
    # robot server reaches it by duck typing. Positions cross the wall in
    # the house frame and are converted here, exactly as poses are.

    def _unapply(self, x: float, y: float, anchor=None):
        """House frame -> ours-in-SLAM's-frame: the anchor, inverted."""
        anchor = anchor or self._anchor
        alpha, t = anchor.alpha, anchor.t
        c, s = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
        dx, dy = x - t[0], y - t[1]
        return c * dx + s * dy, -s * dx + c * dy

    def set_goal(self, x_m: float, y_m: float) -> dict:
        # The bridge's CURRENT session, every time (3.76 H1-12): after a
        # container restart a goal converted with the old session's anchor
        # was sent into the new map, wherever that began.
        # A bridge that cannot answer the pose cannot be trusted with the goal
        # either: raise (the robot server answers `ros_unavailable`), unless
        # there was never a session, which is said by name.
        try:
            reply = self._fetch_pose()
        except httpx.HTTPError as e:
            # 3.82: a fault no wait mends (a 401: the secrets differ) is
            # raised even before a session -- read as "no session yet", it
            # was waited on for the whole mission.
            if self._session is not None or not classify(e)["retryable"]:
                raise
            reply = None
        # A session's anchor never changes (round 5), so the point the brain
        # chose on this anchor's map is converted on it; a new session
        # anchors first, and one still waiting for its start truth refuses.
        anchor = self._ensure_session(reply) if reply is not None else None
        if anchor is None:
            return {"accepted": False, "reason": "no SLAM session yet"}
        rx, ry = _ours_to_ros(*self._unapply(x_m, y_m, anchor))
        # The session it was converted for (round 8): a bridge that restarted
        # since the pose read answers 409 rather than take it into a new map.
        r = self._http.post("/goal", json={"x_m": rx, "y_m": ry, "yaw_rad": 0.0,
                                           "session": anchor.session})
        if r.status_code == 409:
            body = _body(r)
            now_on = body.get("session") if body else None   # round 10
            return {"accepted": False, "reason": "no SLAM session yet",
                    "detail": (f"the SLAM session changed between the pose read and the goal "
                               f"(converted for {anchor.session}, the bridge is on {now_on})")}
        # Any other error status raises with the bridge's own text (the robot
        # server answers it `ros_unavailable`, naming the cause: a 401, a 503
        # "nav2 is not up"),
        # and a 200 that is not a JSON object is never a goal nav2 took.
        return {"accepted": True, **_json_object(r)}

    def cancel_goal(self) -> dict:
        # Raises on an error status (a 401, a 5xx) rather than returning its
        # body as an answer: the stop's goal-ending loop retries on a raise,
        # and a refused cancel must not read as one that happened.
        r = self._http.delete("/goal")
        return _json_object(r)

    def get_goal(self) -> dict:
        r = self._http.get("/goal")
        reply = _json_object(r)
        anchor = self._anchor
        if reply.get("goal") and reply.get("session") != anchor.session:
            # A goal in a session not yet anchored (3.76 H1-12): adopt it,
            # or goal_in_progress(), the stop's goal-ending loop and the
            # brain's cancel_own() all miss a live goal.
            try:
                adopted = self._ensure_session(self._fetch_pose())
            except httpx.HTTPError:
                adopted = None
            if adopted is None:
                # Cannot be placed in the house yet (R7): its state still
                # counts -- arbitration, the stop's hold -- so it is answered
                # without coordinates rather than as no goal at all.
                g = {k: v for k, v in reply["goal"].items()
                     if k not in ("x_m", "y_m", "yaw_rad")}
                return {"goal": dict(g, unanchored=True), "plan": []}
            anchor = adopted
        if reply.get("session") != anchor.session or not reply.get("goal"):
            return {"goal": None, "plan": []}
        g = dict(reply["goal"])
        gx, gy, _ = self._apply(*_ros_to_ours(g["x_m"], g["y_m"], 0.0), anchor)
        g["x_m"], g["y_m"] = gx, gy
        g.pop("yaw_rad", None)
        plan = []
        for px, py in reply.get("plan") or []:
            hx, hy, _ = self._apply(*_ros_to_ours(px, py, 0.0), anchor)
            plan.append([round(hx, 4), round(hy, 4)])
        return {"goal": g, "plan": plan}
