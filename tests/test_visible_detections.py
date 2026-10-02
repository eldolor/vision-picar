"""
tests/test_visible_detections.py

`PLAN-ros-alignment.md` 3.32 criterion 4: the sim's synthetic detections
respect occlusion the way a detector's box does -- reported if any part of
the object's face is visible, at the bearing of the visible part; not
reported when wholly hidden. The picture is unchanged (the golden image in
tests/test_renderer.py is the guard for that).
"""

import math

from sim import renderer
from sim.grid_world import GridWorld

# A wall block at (3, 2) between the robot at (1.5, 2.5), facing +x, and
# the cells behind it. Grid frame: y DOWN, so a larger y is a clockwise
# (positive) bearing, the project's convention.
LAYOUT = [
    "##########",
    "#........#",
    "#..#.....#",
    "#........#",
    "#........#",
    "##########",
]
PX, PY, VIEW = 1.5, 2.5, 0.0


def _centre_bearing(cell):
    return math.atan2(cell[1] + 0.5 - PY, cell[0] + 0.5 - PX) - VIEW


def test_wholly_behind_the_wall_is_not_seen():
    assert renderer.visible_bearing(LAYOUT, PX, PY, VIEW, (6, 2)) is None


def test_half_behind_the_wall_is_seen_at_the_bearing_of_the_half_that_shows():
    cell = (5, 3)              # its upper part hides behind the block, its lower part shows
    b = renderer.visible_bearing(LAYOUT, PX, PY, VIEW, cell)
    assert b is not None, "part of it is visible"
    assert b > _centre_bearing(cell) + math.radians(0.5), (
        math.degrees(b), math.degrees(_centre_bearing(cell)))


def test_in_the_open_the_bearing_is_the_centre():
    """No wall in the way and the whole face inside the field of view (an
    object straddling the frame's edge is clipped to the part in frame, as a
    box would be)."""
    open_layout = [row.replace("#..#", "#...") if i == 2 else row
                   for i, row in enumerate(LAYOUT)]
    cell = (6, 3)
    b = renderer.visible_bearing(open_layout, PX, PY, VIEW, cell)
    assert abs(b - _centre_bearing(cell)) < math.radians(1.0)


def test_frames_report_the_partly_visible_object_and_not_the_hidden_one():
    world = GridWorld(layout=LAYOUT, rooms={}, objects={(5, 3): "red backpack",
                                                         (6, 2): "blue bottle"},
                      robot_x=1, robot_y=2)
    world.x, world.y, world.theta = PX, PY, VIEW
    labels = {d["label"] for d in world.frame_description()["detections"]}
    assert labels == {"red backpack"}, labels
