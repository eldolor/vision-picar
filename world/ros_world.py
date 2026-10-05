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

import math
from typing import Optional

import httpx

from world.interface import (
    CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN, WorldInterface, unusable_map,
    unusable_pose, unusable_truth)

# OccupancyGrid probabilities are 0-100. slam_toolbox writes 100 and 0 for
# the confident cases; these cut the middle the way nav2's map server does.
OCCUPIED_AT = 65
FREE_BELOW = 25


def _ros_to_ours(x: float, y: float, yaw: float):
    """ROS (x fwd, y left, yaw CCW) -> ours in the SAME frame (x, y flipped
    to point the other way, compass heading clockwise with 0 along -y)."""
    return x, -y, (90.0 - math.degrees(yaw)) % 360.0


def _ours_to_ros(x: float, y: float):
    """The inverse of `_ros_to_ours()` for a position. Both directions of
    the flip live here, next to each other, so they cannot disagree."""
    return x, -y


class RosWorld(WorldInterface):
    def __init__(self, bridge_url: str, secret: str = "", truth: Optional[WorldInterface] = None,
                 timeout_s: float = 2.0, client: Optional[httpx.Client] = None):
        headers = {"x-app-secret": secret} if secret else {}
        self._http = client or httpx.Client(base_url=bridge_url.rstrip("/"), headers=headers,
                                            timeout=timeout_s)
        self._truth = truth
        self._session: Optional[str] = None
        # Anchor: ours_world = R(alpha) . ours_slam + (tx, ty); heading + alpha.
        self._alpha = 0.0
        self._t = (0.0, 0.0)
        self._map_cache = None
        self.anchored_at: Optional[str] = None

    # ---------- the anchor ----------

    def _apply(self, x: float, y: float, heading: float):
        c, s = math.cos(math.radians(self._alpha)), math.sin(math.radians(self._alpha))
        return (c * x - s * y + self._t[0], s * x + c * y + self._t[1],
                (heading + self._alpha) % 360.0)

    def _fetch_pose(self):
        r = self._http.get("/slam/pose")
        r.raise_for_status()
        return r.json()

    def _ensure_session(self, reply: dict) -> bool:
        """Re-anchor on a new session. False if there is nothing to anchor on.

        The anchor is the truth AT THE SESSION'S START -- odometry zero,
        where SLAM's map frame begins -- as recorded by the bridge. It must
        not be the truth at first contact: a consumer that first asks after
        the robot has driven would fold everything SLAM got wrong so far into
        the frame, and read 0.0 cm. (The twin did exactly that on the first
        phone-size check, 2026-09-26.) At odometry zero SLAM's pose is the
        origin with yaw 0, which is compass 90 in SLAM's own frame.
        """
        if reply.get("session") == self._session:
            return True
        if reply.get("map") is None:
            return False
        self._session = reply["session"]
        self._map_cache = None
        start = reply.get("start_truth")
        if start is None and self._truth is not None:
            # A bridge that predates start_truth: fall back to first contact,
            # and say so, because the error it produces can be too small.
            est = reply["map"]
            x, y, h = _ros_to_ours(est["x_m"], est["y_m"], est["yaw_rad"])
            truth = self._truth.get_truth()
            if truth.get("usable"):
                self._alpha = (truth["heading_deg"] - h + 180.0) % 360.0 - 180.0
                self._t = (0.0, 0.0)
                ax, ay, _ = self._apply(x, y, h)
                self._t = (truth["x_m"] - ax, truth["y_m"] - ay)
                self.anchored_at = "first_contact"
                return True
        if start is not None and self._truth is not None:
            self._alpha = (start["heading_deg"] - 90.0 + 180.0) % 360.0 - 180.0
            self._t = (start["x_m"], start["y_m"])
            self.anchored_at = "session_start"
        else:
            self._alpha, self._t = 0.0, (0.0, 0.0)
            self.anchored_at = None
        return True

    @property
    def map_id(self) -> Optional[str]:
        return f"slam-{self._session}" if self._session else None

    # ---------- WorldInterface ----------

    def get_pose(self) -> dict:
        try:
            reply = self._fetch_pose()
        except httpx.HTTPError:
            return unusable_pose()
        if not self._ensure_session(reply) or reply.get("map") is None:
            return unusable_pose()
        est = reply["map"]
        x, y, h = self._apply(*_ros_to_ours(est["x_m"], est["y_m"], est["yaw_rad"]))
        return {"usable": True, "map_id": self.map_id, "x_m": x, "y_m": y,
                "heading_deg": round(h, 4)}

    def get_odom_pose(self) -> dict:
        """diff_drive_controller's dead reckoning, in the same anchored frame
        -- what the robot would believe with no SLAM. Not part of the
        interface: it exists so R5 can show what SLAM corrects."""
        try:
            reply = self._fetch_pose()
        except httpx.HTTPError:
            return unusable_pose()
        if not self._ensure_session(reply) or reply.get("odom") is None:
            return unusable_pose()
        # odom and map coincide at SLAM's start, so the same anchor applies.
        o = reply["odom"]
        x, y, h = self._apply(*_ros_to_ours(o["x_m"], o["y_m"], o["yaw_rad"]))
        return {"usable": True, "map_id": self.map_id, "x_m": x, "y_m": y,
                "heading_deg": round(h, 4)}

    def get_truth(self) -> dict:
        return self._truth.get_truth() if self._truth else unusable_truth()

    def get_map(self) -> dict:
        try:
            if self._session is None:
                self._ensure_session(self._fetch_pose())
            r = self._http.get("/slam/map")
            r.raise_for_status()
            m = r.json()
        except httpx.HTTPError:
            return unusable_map()
        if m.get("session") != self._session or not m.get("data"):
            return unusable_map()
        key = (m["session"], m["version"], self._alpha, self._t)
        if self._map_cache and self._map_cache[0] == key:
            return self._map_cache[1]
        out = self._resample(m)
        self._map_cache = (key, out)
        return out

    def _resample(self, m: dict) -> dict:
        """SLAM's grid, re-expressed in the house frame at the same
        resolution. Nearest-neighbour: every output cell asks which SLAM
        cell its centre falls in. Exact for the rotations the anchor makes
        at a cardinal start, and honest (never invents a state) otherwise."""
        res, w, h = m["resolution_m"], m["width"], m["height"]
        ox, oy, oyaw = m["origin_x_m"], m["origin_y_m"], m["origin_yaw_rad"]
        data = m["data"]
        cyaw, syaw = math.cos(oyaw), math.sin(oyaw)

        def slam_cell_to_ours_world(i, j):
            # centre of grid cell (i col, j row) in ROS map frame
            gx, gy = (i + 0.5) * res, (j + 0.5) * res
            rx, ry = ox + cyaw * gx - syaw * gy, oy + syaw * gx + cyaw * gy
            x, y, _ = _ros_to_ours(rx, ry, 0.0)
            wx, wy, _ = self._apply(x, y, 0.0)
            return wx, wy

        corners = [slam_cell_to_ours_world(i, j) for i in (-0.5, w - 0.5) for j in (-0.5, h - 0.5)]
        min_x = min(c[0] for c in corners)
        min_y = min(c[1] for c in corners)
        out_w = int(math.ceil((max(c[0] for c in corners) - min_x) / res))
        out_h = int(math.ceil((max(c[1] for c in corners) - min_y) / res))
        ca, sa = math.cos(math.radians(self._alpha)), math.sin(math.radians(self._alpha))
        cells = [CELL_UNKNOWN] * (out_w * out_h)
        for row in range(out_h):
            for col in range(out_w):
                wx, wy = min_x + (col + 0.5) * res, min_y + (row + 0.5) * res
                # undo the anchor, then ours -> ROS (flip y), then the origin
                dx, dy = wx - self._t[0], wy - self._t[1]
                x, y = ca * dx + sa * dy, -sa * dx + ca * dy
                rx, ry = _ours_to_ros(x, y)
                rx, ry = rx - ox, ry - oy
                gx, gy = cyaw * rx + syaw * ry, -syaw * rx + cyaw * ry
                i, j = int(math.floor(gx / res)), int(math.floor(gy / res))
                if 0 <= i < w and 0 <= j < h:
                    v = data[j * w + i]
                    cells[row * out_w + col] = (
                        CELL_UNKNOWN if v < 0 else CELL_OCCUPIED if v >= OCCUPIED_AT
                        else CELL_FREE if v < FREE_BELOW else CELL_UNKNOWN)
        return {"usable": True, "map_id": self.map_id, "map_version": m["version"],
                "resolution_m": res, "width": out_w, "height": out_h,
                "origin_x_m": min_x, "origin_y_m": min_y, "cells": cells}


    # ---------- R6: goals (not part of WorldInterface) ----------
    #
    # A goal is a request to nav2, so it lives on this backend alone and the
    # robot server reaches it by duck typing. Positions cross the wall in
    # the house frame and are converted here, exactly as poses are.

    def _unapply(self, x: float, y: float):
        """House frame -> ours-in-SLAM's-frame: the anchor, inverted."""
        c, s = math.cos(math.radians(self._alpha)), math.sin(math.radians(self._alpha))
        dx, dy = x - self._t[0], y - self._t[1]
        return c * dx + s * dy, -s * dx + c * dy

    def set_goal(self, x_m: float, y_m: float) -> dict:
        if self._session is None:
            self.get_pose()                  # anchor first
        if self._session is None:
            return {"accepted": False, "reason": "no SLAM session yet"}
        rx, ry = _ours_to_ros(*self._unapply(x_m, y_m))
        r = self._http.post("/goal", json={"x_m": rx, "y_m": ry, "yaw_rad": 0.0})
        return {"accepted": r.status_code == 200, **r.json()}

    def cancel_goal(self) -> dict:
        # Raises on an error status (a 401, a 5xx) rather than returning its
        # body as an answer: the stop's goal-ending loop retries on a raise,
        # and a refused cancel must not read as one that happened.
        r = self._http.delete("/goal")
        r.raise_for_status()
        return r.json()

    def get_goal(self) -> dict:
        r = self._http.get("/goal")
        r.raise_for_status()
        reply = r.json()
        if reply.get("session") != self._session or not reply.get("goal"):
            return {"goal": None, "plan": []}
        g = dict(reply["goal"])
        gx, gy, _ = self._apply(*_ros_to_ours(g["x_m"], g["y_m"], 0.0))
        g["x_m"], g["y_m"] = gx, gy
        g.pop("yaw_rad", None)
        plan = []
        for px, py in reply.get("plan") or []:
            hx, hy, _ = self._apply(*_ros_to_ours(px, py, 0.0))
            plan.append([round(hx, 4), round(hy, 4)])
        return {"goal": g, "plan": plan}
