"""relocalise -- does a reloaded map fit where the robot is? (3.50.D3)

A reload assumes the robot stands on the dock: localisation starts at the
dock's pose, (0, 0, 0) in the saved map's frame. slam_toolbox searches only
+-0.25 m and +-20 deg around that (slam.yaml), so a robot put down elsewhere
is not pulled back to where it really is -- it is placed on the dock, and its
scans then disagree with the map. Two tests, both on scans at rest:

* agreement: the share of scan returns that land within AGREE_TOL_M of an
  occupied cell of the reloaded /map, the median of SCANS scans;
* at the dock: the localised pose within DOCK_TOL_M and DOCK_TOL_DEG of it
  (a matcher that did move the robot found it somewhere else).

Either failing means a fresh map instead. The numbers were written in the
plan before any measurement (3.50 "3.50.D3 criteria"). Kept out of
bridge.py, which imports rclpy, so it is tested on a laptop.
"""

import math
from statistics import median
from typing import Optional, Sequence

AGREE_TOL_M = 0.10
AGREE_MIN = 0.60
SCANS = 5
DOCK_TOL_M = 0.5
DOCK_TOL_DEG = 15.0
# From the localisation node's start to a verdict: a node that has given no
# pose by then has not matched (criterion 8: a fall-back within 10 s).
VERDICT_TIMEOUT_S = 8.0
# The same cut nav2's map server uses for "occupied" (world/ros_world.py).
OCCUPIED_AT = 65


def occupied_cells(grid: dict) -> set:
    """(col, row) of every occupied cell of a bridge map record."""
    w = grid["width"]
    return {(i % w, i // w) for i, v in enumerate(grid["data"]) if v >= OCCUPIED_AT}


def scan_agreement(scan: dict, laser_pose: Sequence[float], grid: dict,
                   occupied: Optional[set] = None, tol_m: float = AGREE_TOL_M) -> Optional[float]:
    """The share of `scan`'s returns within `tol_m` of an occupied cell, with
    the laser at `laser_pose` (x, y, yaw) in the map's frame -- or None if the
    scan has no return to judge. `scan` is the bridge's own record (ROS
    convention: counter-clockwise, None for no return); `grid` its map record."""
    occupied = occupied_cells(grid) if occupied is None else occupied
    res = grid["resolution_m"]
    oyaw = grid.get("origin_yaw_rad") or 0.0
    oc, os_ = math.cos(-oyaw), math.sin(-oyaw)
    reach = int(math.ceil(tol_m / res - 1e-9))
    x0, y0, yaw = laser_pose
    hits = total = 0
    for i, r in enumerate(scan["ranges_m"]):
        if r is None or not math.isfinite(r) or r <= 0:
            continue
        a = yaw + scan["angle_min_rad"] + i * scan["angle_increment_rad"]
        dx = x0 + r * math.cos(a) - grid["origin_x_m"]
        dy = y0 + r * math.sin(a) - grid["origin_y_m"]
        gx, gy = oc * dx - os_ * dy, os_ * dx + oc * dy        # into the grid's axes
        col, row = int(math.floor(gx / res)), int(math.floor(gy / res))
        total += 1
        if any((col + c, row + rr) in occupied
               and math.hypot((col + c + 0.5) * res - gx, (row + rr + 0.5) * res - gy)
               <= tol_m + res / math.sqrt(2)
               for c in range(-reach, reach + 1) for rr in range(-reach, reach + 1)):
            hits += 1
    return hits / total if total else None


def judge(shares: Sequence[Optional[float]], pose: Optional[Sequence[float]]) -> dict:
    """The verdict on a reload from the agreement of SCANS scans and the
    localised pose (x, y, yaw) in the map's frame. `ok` False means a fresh
    map; `reason` says which test failed."""
    known = [s for s in shares if s is not None]
    share = median(known) if known else None
    out = {"agreement": None if share is None else round(share, 3),
           "dock_error_m": None, "dock_error_deg": None}
    if pose is None:
        return {**out, "ok": False, "reason": "no localised pose"}
    dist = math.hypot(pose[0], pose[1])
    turn = abs(math.degrees(math.atan2(math.sin(pose[2]), math.cos(pose[2]))))
    out.update(dock_error_m=round(dist, 3), dock_error_deg=round(turn, 2))
    if share is None:
        return {**out, "ok": False, "reason": "no scan returns to judge"}
    if share < AGREE_MIN:
        return {**out, "ok": False,
                "reason": f"scans agree with the saved map {share:.0%} (< {AGREE_MIN:.0%})"}
    if dist > DOCK_TOL_M or turn > DOCK_TOL_DEG:
        return {**out, "ok": False,
                "reason": f"localised {dist:.2f} m, {turn:.0f} deg from the dock"}
    return {**out, "ok": True, "reason": "matched"}
