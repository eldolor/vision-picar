"""
brain/mission_report.py

PLAN-ros-alignment.md 3.70 E4/E5: the "where it looked" half of a mission's
report -- how much of the house the robot's own map holds.

The brain cannot know the house's true floor, so coverage here is an
ESTIMATE on the map alone: known free floor over known free floor plus the
unknown cells the map has walled in. Unknown cells that reach the map's edge
through other unknown cells are outside the house as far as the map can
tell, and are not counted. What it gets wrong, it gets wrong both ways: an
unknown patch under a sofa counts as floor still to see, and a room behind a
doorway the map has not closed off counts as outside. Its error against
truth (`tests/demo_explore.coverage()`) is measured, not assumed
(`tests/test_mission_ends.py`).
"""

from collections import deque

from world.interface import CELL_FREE, CELL_UNKNOWN


def map_coverage(m: dict) -> dict:
    """`{"coverage": float|None, "mapped_free_m2": float|None}` for one
    `WorldInterface.get_map()` answer. None for an unusable map."""
    if not m or not m.get("usable"):
        return {"coverage": None, "mapped_free_m2": None}
    w, h, res, cells = m["width"], m["height"], m["resolution_m"], m["cells"]
    free = sum(1 for c in cells if c == CELL_FREE)
    # Unknown cells connected to the map's border through unknown cells.
    outside = bytearray(w * h)
    q = deque()
    for i in range(w):
        for j in (0, h - 1):
            k = j * w + i
            if cells[k] == CELL_UNKNOWN and not outside[k]:
                outside[k] = 1
                q.append(k)
    for j in range(h):
        for i in (0, w - 1):
            k = j * w + i
            if cells[k] == CELL_UNKNOWN and not outside[k]:
                outside[k] = 1
                q.append(k)
    while q:
        k = q.popleft()
        i, j = k % w, k // w
        for n in ((k - 1) if i > 0 else -1, (k + 1) if i < w - 1 else -1,
                  (k - w) if j > 0 else -1, (k + w) if j < h - 1 else -1):
            if n >= 0 and not outside[n] and cells[n] == CELL_UNKNOWN:
                outside[n] = 1
                q.append(n)
    # Each walled-in unknown patch counts as floor still to see only if it
    # borders known floor (a frontier): one walled in by obstacles alone is
    # the inside of a sofa or a cupboard, which no robot can reach.
    seen_k = bytearray(w * h)
    enclosed = 0
    for start in range(w * h):
        if cells[start] != CELL_UNKNOWN or outside[start] or seen_k[start]:
            continue
        seen_k[start] = 1
        patch, frontier, q = 0, False, deque([start])
        while q:
            k = q.popleft()
            patch += 1
            i, j = k % w, k // w
            for n in ((k - 1) if i > 0 else -1, (k + 1) if i < w - 1 else -1,
                      (k - w) if j > 0 else -1, (k + w) if j < h - 1 else -1):
                if n < 0:
                    continue
                if cells[n] == CELL_FREE:
                    frontier = True
                elif cells[n] == CELL_UNKNOWN and not seen_k[n]:
                    seen_k[n] = 1
                    q.append(n)
        if frontier:
            enclosed += patch
    total = free + enclosed
    return {"coverage": round(free / total, 4) if total else None,
            "mapped_free_m2": round(free * res * res, 2)}
