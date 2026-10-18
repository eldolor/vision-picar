"""keepout -- the mask behind POST /nav/keepout, as plain Python (3.68).

The robot server decides where nav2 must not plan (robot/wedge_watch.py);
this only draws those discs into an OccupancyGrid for nav2's stock
KeepoutFilter: 100 inside a disc (lethal to the robot's centre), -1
(unknown) elsewhere, which the filter skips -- it writes any other value
over an unknown master cell, so a 0 made unmapped space free (3.76 K5).
Kept out of bridge.py, which imports rclpy, so it is tested on a
laptop (tests/test_wedge_watch.py).
"""

import math

RES_M = 0.05
MARGIN_M = 0.10
UNKNOWN = -1      # nav_msgs/OccupancyGrid's unknown: the filter leaves the cell as it was


def parse(spots, now: float) -> list:
    """The route's body -> [{x_m, y_m, r_m, until}], in the map frame.
    Raises ValueError on a malformed spot rather than drawing a guess."""
    out = []
    for s in spots:
        x, y, r, ttl = (float(s[k]) for k in ("x_m", "y_m", "r_m", "ttl_s"))
        if not all(math.isfinite(v) for v in (x, y, r, ttl)) or r <= 0 or r > 1.0 or ttl <= 0:
            raise ValueError(f"bad keepout spot {s!r}")
        out.append({"x_m": x, "y_m": y, "r_m": r, "until": now + ttl})
    return out


def live(spots: list, now: float) -> list:
    return [s for s in spots if s["until"] > now]


def mask(spots: list, res: float = RES_M) -> dict:
    """One grid that covers every disc (a single unknown cell when there
    are none, so a cleared set still replaces the last mask). Row-major from
    the origin, +x then +y, as nav_msgs/OccupancyGrid."""
    if not spots:
        return {"origin_x_m": 0.0, "origin_y_m": 0.0, "resolution_m": res,
                "width": 1, "height": 1, "data": [UNKNOWN]}
    x0 = min(s["x_m"] - s["r_m"] for s in spots) - MARGIN_M
    y0 = min(s["y_m"] - s["r_m"] for s in spots) - MARGIN_M
    x1 = max(s["x_m"] + s["r_m"] for s in spots) + MARGIN_M
    y1 = max(s["y_m"] + s["r_m"] for s in spots) + MARGIN_M
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    data = [UNKNOWN] * (w * h)
    for s in spots:
        r2 = s["r_m"] ** 2
        i0, i1 = int((s["x_m"] - s["r_m"] - x0) / res), int((s["x_m"] + s["r_m"] - x0) / res)
        j0, j1 = int((s["y_m"] - s["r_m"] - y0) / res), int((s["y_m"] + s["r_m"] - y0) / res)
        for j in range(max(j0, 0), min(j1 + 1, h)):
            cy = y0 + (j + 0.5) * res
            for i in range(max(i0, 0), min(i1 + 1, w)):
                cx = x0 + (i + 0.5) * res
                if (cx - s["x_m"]) ** 2 + (cy - s["y_m"]) ** 2 <= r2:
                    data[j * w + i] = 100
    return {"origin_x_m": x0, "origin_y_m": y0, "resolution_m": res,
            "width": w, "height": h, "data": data}
