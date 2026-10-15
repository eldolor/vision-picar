"""
tests/ros_resample_reference.py

`RosWorld._resample()` as it was before 3.83 vectorised it, FROZEN: the
per-cell loop that placed every SLAM map on the house up to 2026-10-15.
tests/test_costmap.py checks the numpy placement against it cell for cell
(the map's states, and the source cell the costmap reads). Do not edit it to
match a change in world/ros_world.py; a difference is the finding.
"""

import math

from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN
from world.ros_world import FREE_BELOW, OCCUPIED_AT, _ours_to_ros, _ros_to_ours


def reference_resample(self, m: dict, anchor) -> dict:
    """SLAM's grid, re-expressed in the house frame at the same
    resolution. Nearest-neighbour: every output cell asks which SLAM
    cell its centre falls in. Exact for the rotations the anchor makes
    at a cardinal start, and honest (never invents a state) otherwise."""
    res, w, h = m["resolution_m"], m["width"], m["height"]
    ox, oy, oyaw = m["origin_x_m"], m["origin_y_m"], m["origin_yaw_rad"]
    data = m["data"]
    anchor = anchor or self._anchor
    session, alpha, t = anchor.session, anchor.alpha, anchor.t
    cyaw, syaw = math.cos(oyaw), math.sin(oyaw)

    def slam_cell_to_ours_world(i, j):
        # centre of grid cell (i col, j row) in ROS map frame
        gx, gy = (i + 0.5) * res, (j + 0.5) * res
        rx, ry = ox + cyaw * gx - syaw * gy, oy + syaw * gx + cyaw * gy
        x, y, _ = _ros_to_ours(rx, ry, 0.0)
        wx, wy, _ = self._apply(x, y, 0.0, anchor)
        return wx, wy

    corners = [slam_cell_to_ours_world(i, j) for i in (-0.5, w - 0.5) for j in (-0.5, h - 0.5)]
    min_x = min(c[0] for c in corners)
    min_y = min(c[1] for c in corners)
    out_w = int(math.ceil((max(c[0] for c in corners) - min_x) / res))
    out_h = int(math.ceil((max(c[1] for c in corners) - min_y) / res))
    ca, sa = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    cells = [CELL_UNKNOWN] * (out_w * out_h)
    for row in range(out_h):
        for col in range(out_w):
            wx, wy = min_x + (col + 0.5) * res, min_y + (row + 0.5) * res
            # undo the anchor, then ours -> ROS (flip y), then the origin
            dx, dy = wx - t[0], wy - t[1]
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
    return {"usable": True, "map_id": f"slam-{session}",
            "map_version": m["version"] + anchor.epoch,
            "resolution_m": res, "width": out_w, "height": out_h,
            "origin_x_m": min_x, "origin_y_m": min_y, "cells": cells}
