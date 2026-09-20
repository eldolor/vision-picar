"""
tests/test_world_contract.py

Phase N1 (PLAN-mapping.md) -- pin WorldInterface's return-shape contract
as a backend-agnostic suite, exactly as `tests/test_robot_contract.py`
does for `RobotInterface`. Written BEFORE a backend can answer anything,
which is the point: the contract is defined against an honest "I cannot
tell you" first, and a real mapper arrives later and has something to
conform to.

The suite also pins the BODY/WORLD SPLIT itself (see the last section),
because that split is the reason this file exists and it is the thing
most likely to erode quietly. A `get_pose()` that reappears on
`RobotInterface` because it felt like body state would not break a single
other test.

Run with: pytest tests/test_world_contract.py -v
"""

import inspect

import pytest

from robot.interface import RobotInterface
from world.interface import (
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_STATES,
    CELL_UNKNOWN,
    NullWorld,
    WorldInterface,
    unusable_map,
    unusable_pose,
)

# Every WorldInterface backend that exists. One today; the suite is
# parameterized from the start so that adding MockWorld (N1) and RosWorld
# (N6) is a line here rather than a new file -- which is what kept the
# four RobotInterface backends honest with each other.
WORLD_BACKENDS = [
    pytest.param(NullWorld, id="NullWorld"),
]


@pytest.fixture(params=WORLD_BACKENDS)
def world(request):
    return request.param()


# ---------- get_pose() ----------


def test_get_pose_answers_the_same_shape_everywhere(world):
    pose = world.get_pose()
    assert set(pose) == {"usable", "map_id", "x_m", "y_m", "heading_deg"}
    assert isinstance(pose["usable"], bool)


def test_an_unusable_pose_carries_no_numbers(world):
    """The whole point of the honest default. A backend that cannot
    localise must not hand a planner coordinates it can plot."""
    pose = world.get_pose()
    if pose["usable"]:
        return
    assert pose["x_m"] is None
    assert pose["y_m"] is None
    assert pose["heading_deg"] is None
    assert pose["map_id"] is None


def test_a_usable_pose_names_its_map(world):
    """Coordinates without a map_id are two numbers that mean nothing,
    and a stale map's coordinates plot somewhere plausible and wrong."""
    pose = world.get_pose()
    if not pose["usable"]:
        return
    assert isinstance(pose["map_id"], str) and pose["map_id"]
    for key in ("x_m", "y_m", "heading_deg"):
        assert isinstance(pose[key], (int, float))


def test_unusable_pose_is_not_the_origin():
    """(0, 0) is a valid pose and would be indistinguishable from a real
    robot at its map origin. None cannot be plotted by accident."""
    pose = unusable_pose()
    assert pose["usable"] is False
    assert pose["x_m"] is None and pose["y_m"] is None


# ---------- get_map() ----------


def test_get_map_answers_the_same_shape_everywhere(world):
    grid = world.get_map()
    assert set(grid) == {
        "usable",
        "map_id",
        "map_version",
        "resolution_m",
        "width",
        "height",
        "origin_x_m",
        "origin_y_m",
        "cells",
    }
    assert isinstance(grid["usable"], bool)
    assert isinstance(grid["map_version"], int)
    assert isinstance(grid["width"], int)
    assert isinstance(grid["height"], int)
    assert isinstance(grid["cells"], list)


def test_the_map_declares_its_own_shape(world):
    """Row-major, and a consumer reads the shape off the answer rather
    than assuming one -- the same rule get_depth_grid() follows."""
    grid = world.get_map()
    assert len(grid["cells"]) == grid["width"] * grid["height"]


def test_every_cell_is_one_of_the_three_states(world):
    grid = world.get_map()
    for cell in grid["cells"]:
        assert cell in CELL_STATES


def test_an_unusable_map_has_no_extent(world):
    """Unlike the depth strip, there is no honest number of cells to
    hatch for a house nobody has seen -- so an empty map is a label in
    the UI, not a drawing."""
    grid = world.get_map()
    if grid["usable"]:
        return
    assert grid["width"] == 0 and grid["height"] == 0
    assert grid["cells"] == []
    assert grid["map_id"] is None
    assert grid["resolution_m"] is None


def test_a_usable_map_names_itself_and_its_scale(world):
    grid = world.get_map()
    if not grid["usable"]:
        return
    assert isinstance(grid["map_id"], str) and grid["map_id"]
    assert isinstance(grid["resolution_m"], (int, float))
    assert grid["resolution_m"] > 0
    assert grid["width"] > 0 and grid["height"] > 0


def test_unknown_is_distinct_from_free():
    """M3's argument, one abstraction up: a planner that reads unmapped
    as empty floor routes through a wall it has not seen yet. The three
    states must be three different values."""
    assert len({CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED}) == 3


def test_the_unusable_helpers_match_the_interface_default():
    """A backend that overrides nothing and one that returns the helper
    explicitly must be indistinguishable."""
    assert NullWorld().get_pose() == unusable_pose()
    assert NullWorld().get_map() == unusable_map()


# ---------- the body/world split ----------
#
# The reason this interface exists at all. These two tests are the only
# thing standing between the split and a plausible-looking shortcut.

WORLD_STATE_METHODS = ("get_pose", "get_map")

BODY_STATE_METHODS = (
    "drive_forward",
    "reverse",
    "turn_left",
    "turn_right",
    "stop",
    "look_left",
    "look_right",
    "look_center",
    "get_camera_frame",
    "get_distance",
    "get_depth_grid",
    "get_odometry",
)


@pytest.mark.parametrize("name", WORLD_STATE_METHODS)
def test_world_state_does_not_live_on_the_robot_interface(name):
    """Egocentric is body, allocentric is world. A pose is expressed in a
    MAP's frame and is meaningless without a map_id, which is the tell:
    it is not a reading the body can take about itself.

    If this fails, someone moved get_pose() back onto RobotInterface
    because 'where am I' felt like body state. It reads that way and it
    is not -- see world/interface.py's opening note. Odometry is what the
    body says about itself; pose is what the world says about the body.
    """
    assert not hasattr(RobotInterface, name)


@pytest.mark.parametrize("name", BODY_STATE_METHODS)
def test_body_state_does_not_live_on_the_world_interface(name):
    """The mirror. A world model is asked what is true about the house;
    it is never asked to drive, and it owns no sensor."""
    assert not hasattr(WorldInterface, name)


def test_the_world_interface_has_no_abstract_methods():
    """Both methods carry an honest default, for the same reason
    get_depth_grid() does: a new backend should be silent about the map,
    never broken by it. A robot with no mapper is a valid robot."""
    assert not getattr(WorldInterface, "__abstractmethods__", None)


def test_every_world_method_is_documented():
    """These docstrings ARE the contract -- there is no other statement of
    what a pose or a map means in this project."""
    for name in WORLD_STATE_METHODS:
        assert inspect.getdoc(getattr(WorldInterface, name))
