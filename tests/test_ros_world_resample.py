"""3.87: `RosWorld._resample` in numpy gives EXACTLY the old loop's cells.

The old per-cell Python loop (frozen below as the reference) took 141 ms a
map version on the Jetson inside the robot server's threadpool; beside one
more busy thread it starved the event loop the wheel loop runs on (3.87).
"""
import math
import random

import pytest

from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN
from world.ros_world import (FREE_BELOW, OCCUPIED_AT, Anchor, RosWorld, _ours_to_ros,
                             _ros_to_ours)


def reference(m, anchor):
    """`_resample` as it was before 3.87 (dev c17c15c), verbatim."""
    res, w, h = m["resolution_m"], m["width"], m["height"]
    ox, oy, oyaw = m["origin_x_m"], m["origin_y_m"], m["origin_yaw_rad"]
    data = m["data"]
    session, alpha, t = anchor.session, anchor.alpha, anchor.t
    cyaw, syaw = math.cos(oyaw), math.sin(oyaw)

    def apply(x, y):
        c, s = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
        return c * x - s * y + t[0], s * x + c * y + t[1]

    def slam_cell_to_ours_world(i, j):
        gx, gy = (i + 0.5) * res, (j + 0.5) * res
        rx, ry = ox + cyaw * gx - syaw * gy, oy + syaw * gx + cyaw * gy
        x, y, _ = _ros_to_ours(rx, ry, 0.0)
        return apply(x, y)

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
    return {"usable": True, "map_id": f"slam-{session}", "map_version": m["version"] + anchor.epoch,
            "resolution_m": res, "width": out_w, "height": out_h,
            "origin_x_m": min_x, "origin_y_m": min_y, "cells": cells}


def slam_map(rng, w, h, yaw):
    return {"session": "s1", "version": rng.randint(0, 500), "resolution_m": 0.05000000074505806,
            "width": w, "height": h, "origin_x_m": rng.uniform(-9, 2),
            "origin_y_m": rng.uniform(-9, 2), "origin_yaw_rad": yaw,
            "data": [rng.choice((-1, -1, 0, 0, 0, 10, 24, 25, 40, 64, 65, 90, 100))
                     for _ in range(w * h)]}


CASES = [(0.0, 0.0), (90.0, 0.0), (180.0, 0.0), (270.0, 0.0), (-90.0, 0.0),
         (37.5, 0.0), (0.0, 0.3), (123.4, -1.1), (90.0, math.pi / 2)]


@pytest.mark.parametrize("alpha,yaw", CASES)
def test_resample_matches_the_loop_exactly(alpha, yaw):
    rng = random.Random(f"{alpha}/{yaw}")
    m = slam_map(rng, rng.randint(20, 70), rng.randint(20, 70), yaw)
    a = Anchor(session="s1", alpha=alpha, t=(rng.uniform(-3, 6), rng.uniform(-3, 6)),
               anchored_at=None, epoch=rng.choice((0, 2)))
    got = RosWorld.__new__(RosWorld)._resample(m, a)
    want = reference(m, a)
    assert got == want
    assert all(type(v) is int for v in got["cells"])


def test_a_house_sized_map_matches_too():
    rng = random.Random(387)
    m = slam_map(rng, 253, 263, 0.0)          # the board's home_first_floor map, 2026-10-16
    a = Anchor(session="s1", alpha=90.0, t=(3.2, 4.1), anchored_at=None)
    assert RosWorld.__new__(RosWorld)._resample(m, a) == reference(m, a)
