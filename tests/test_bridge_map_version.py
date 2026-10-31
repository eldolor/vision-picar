"""
tests/test_bridge_map_version.py

3.124: the ROS bridge's map_version moves only when the grid changes.
slam_toolbox republishes an unchanged /map every second; 3.123's ETag is
keyed on the version, so a repeat must keep it. The rule is plain Python
(picar_bridge/mapversion.py), pinned here without ROS.
"""

import inspect
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "service/slam/src/picar_bridge"))
from picar_bridge import mapversion  # noqa: E402

BRIDGE = ROOT / "service/slam/src/picar_bridge/picar_bridge/bridge.py"


def grid(**over):
    m = {"session": "s1", "resolution_m": 0.05, "width": 3, "height": 2,
         "origin_x_m": -1.0, "origin_y_m": -0.5, "origin_yaw_rad": 0.0,
         "data": [-1, 0, 100, 0, 0, -1], "at": 1.0}
    m.update(over)
    return m


def test_an_identical_grid_keeps_the_version():
    held = {**grid(), "version": 7}
    assert mapversion.next_version(held, 7, grid(at=2.0)) == 7


@pytest.mark.parametrize("change", [
    {"data": [-1, 0, 100, 0, 100, -1]},
    {"width": 2, "height": 3},
    {"resolution_m": 0.1},
    {"origin_x_m": -1.05},
    {"origin_y_m": -0.45},
    {"origin_yaw_rad": 1e-12},
    {"session": "s2"},
])
def test_any_change_to_the_grid_bumps_it(change):
    held = {**grid(), "version": 7}
    assert mapversion.next_version(held, 7, grid(**change)) == 8


def test_no_held_map_bumps_even_an_identical_grid():
    # The bridge clears last_map at every reload, fresh start and re-map.
    assert mapversion.next_version(None, 7, grid()) == 8


def test_versions_never_go_down_over_a_stream():
    stream = [grid(), grid(), grid(data=[0] * 6), grid(data=[0] * 6), None, grid(data=[0] * 6),
              grid(width=6, height=1, data=[0] * 6)]
    held, version, seen = None, 0, []
    for m in stream:
        if m is None:            # a reload
            held = None
            continue
        version = mapversion.next_version(held, version, m)
        held = {**m, "version": version}
        seen.append(version)
    assert seen == [1, 1, 2, 2, 3, 4]


def test_the_bridge_uses_the_rule():
    src = BRIDGE.read_text()
    body = src[src.index("def _on_map(self, msg):"):src.index("# ---------- R6: goals")]
    assert "mapversion.next_version(self.last_map, self.map_version" in body
    assert not re.search(r"self\.map_version\s*\+=", src)


def test_mapversion_needs_no_rclpy():
    src = inspect.getsource(mapversion)
    assert not re.search(r"^\s*(import|from)\s+rclpy", src, re.M)
