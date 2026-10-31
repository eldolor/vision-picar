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
from tests.test_review_376h1 import _Msg, bridge_mod  # noqa: E402,F401 -- the stubbed bridge

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


def _map_msg(cells, width=None, sec=100):
    return _Msg(header=_Msg(stamp=_Msg(sec=sec, nanosec=0)),
                info=_Msg(resolution=0.05, width=width or len(cells), height=1,
                          origin=_Msg(position=_Msg(x=0.0, y=0.0),
                                      orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
                data=list(cells))


def test_the_bridge_keeps_the_version_of_a_repeated_map(bridge_mod):
    """Through `_on_map` itself (review: a source grep passes code that
    stores the new map before comparing)."""
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    seen = []
    for cells in ([0, -1, 100], [0, -1, 100], [0, 0, 100], [0, 0, 100]):
        b._on_map(_map_msg(cells))
        seen.append(b.last_map["version"])
    assert seen == [1, 1, 2, 2] and b.map_version == 2
    first = b.last_map
    b._on_map(_map_msg([0, 0, 100]))
    assert b.last_map is not first and b.last_map["data"] is first["data"]   # replaced whole
    assert b.last_map["data"] == [0, 0, 100]
    with b.lock:
        b.last_map = None                                   # as a reload or re-map does
    b._on_map(_map_msg([0, 0, 100]))
    assert b.last_map["version"] == 3


def test_the_bridge_never_rewinds_the_version(bridge_mod):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    b._on_map(_map_msg([0]))
    b._on_map(_map_msg([0, 0], width=2))
    b._on_map(_map_msg([0]))
    assert [b.map_version, b.last_map["version"]] == [3, 3]
    assert not re.search(r"self\.map_version\s*\+=", BRIDGE.read_text())


def test_mapversion_needs_no_rclpy():
    src = inspect.getsource(mapversion)
    assert not re.search(r"^\s*(import|from)\s+rclpy", src, re.M)
