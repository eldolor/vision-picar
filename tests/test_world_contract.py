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
from sim.maps.starter_house import build_starter_world
from sim.mock_world import MockWorld
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
    pytest.param(lambda: MockWorld(build_starter_world()), id="MockWorld"),
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

WORLD_STATE_METHODS = ("get_pose", "get_map", "get_truth")

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
    # R2. A scan is the robot's own reading -- how far everything is from
    # it -- so it is body state even though a map is built from it; that is
    # why it is NOT at /world/scan (PLAN-ros-alignment.md 3.6).
    "get_wheel_state",
    "get_scan",
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


# ---------- MockWorld: the map is DISCOVERED (N1) ----------
#
# These are the only tests here that are allowed to know which backend
# they are talking to, because they are about the simulator's job rather
# than the contract's. Everything above must keep passing for every
# backend; these pin the one behaviour that makes the sim worth having.


def test_the_map_starts_unknown_and_fills_in_as_the_robot_drives():
    """The whole reason MockWorld observes rather than copying the layout.
    A map handed over whole would draw a complete house the instant a
    mission started, which is a false picture of what a mapper does."""
    grid = build_starter_world()
    world = MockWorld(grid)

    assert all(c == CELL_UNKNOWN for c in world._cells)

    first = world.get_map()["cells"]
    seen_at_start = sum(1 for c in first if c != CELL_UNKNOWN)
    assert 0 < seen_at_start < len(first), "a ring should see some of the house, not all"

    # Drive somewhere the first ring could not reach through a wall.
    grid.robot_x, grid.robot_y = 5, 5
    later = world.get_map()["cells"]
    assert sum(1 for c in later if c != CELL_UNKNOWN) > seen_at_start


def test_what_is_behind_a_wall_stays_unknown():
    """A ray stops at the first wall. This is the entire difference
    between a map and a copy of the layout, so it gets its own test."""
    grid = build_starter_world()
    world = MockWorld(grid)
    cells = world.get_map()["cells"]
    assert any(c == CELL_UNKNOWN for c in cells)


def test_the_cell_the_robot_stands_in_is_floor():
    """It drove there, so it is floor by construction -- and the ring
    alone can miss it, because every ray leaves it immediately."""
    grid = build_starter_world()
    world = MockWorld(grid)
    grid_map = world.get_map()
    idx = grid.robot_y * grid_map["width"] + grid.robot_x
    assert grid_map["cells"][idx] == CELL_FREE


def test_map_version_does_not_move_when_nothing_new_is_seen():
    """map_version exists so a client can skip re-downloading 10^5 cells.
    A version that bumped on every read would be useless for that."""
    world = MockWorld(build_starter_world())
    world.get_map()
    settled = world.get_map()["map_version"]
    assert world.get_map()["map_version"] == settled
    assert world.observe() == 0


def test_the_pose_tracks_the_body():
    """The map and the body are two views of ONE GridWorld -- a world
    model with its own copy would report the robot's pose against a
    layout it is not standing in, and look entirely plausible."""
    grid = build_starter_world()
    world = MockWorld(grid)
    before = world.get_pose()
    grid.robot_x += 1
    after = world.get_pose()
    assert after["x_m"] == pytest.approx(before["x_m"] + 0.30)
    assert after["y_m"] == before["y_m"]


def test_the_pose_reports_the_body_heading_not_the_view():
    """A camera pan moves where the robot is LOOKING, not where it is. A
    map that swung 90 degrees on look-left would describe the camera."""
    grid = build_starter_world()
    world = MockWorld(grid)
    before = world.get_pose()["heading_deg"]
    grid.look_left()
    assert world.get_pose()["heading_deg"] == before


def test_the_pose_is_a_compass_bearing_like_every_other_angle_here():
    """Same convention as get_odometry(): clockwise from north, positive
    to the robot's right. Two headings in two rotational senses is the
    harder bug, because it looks right at 0 and 180."""
    from sim.grid_world import Heading

    grid = build_starter_world()
    world = MockWorld(grid)
    for heading, expected in [
        (Heading.N, 0.0),
        (Heading.E, 90.0),
        (Heading.S, 180.0),
        (Heading.W, 270.0),
    ]:
        grid.heading = heading
        assert world.get_pose()["heading_deg"] == expected


def test_the_world_and_the_body_agree_about_cell_size():
    """Two copies of 30cm is how a map and a distance reading start
    describing different houses."""
    from sim.mock_robot import DEFAULT_CELL_CM
    from sim.mock_world import CELL_M

    assert CELL_M * 100 == DEFAULT_CELL_CM


# ---------- RemoteWorld: the same contract, over a socket (N1) ----------


def test_remote_world_answers_the_contract(world_over_asgi):
    """The shipped config maps the sim, so a RemoteWorld against a real
    robot/server.py app gets a real pose and a real map -- over HTTP,
    through the same routes the twin uses."""
    pose = world_over_asgi.get_pose()
    assert set(pose) == {"usable", "map_id", "x_m", "y_m", "heading_deg"}
    assert pose["usable"] is True
    assert isinstance(pose["map_id"], str)

    grid = world_over_asgi.get_map()
    assert grid["usable"] is True
    assert len(grid["cells"]) == grid["width"] * grid["height"]
    assert all(c in CELL_STATES for c in grid["cells"])


def test_a_server_with_no_world_routes_reports_no_map(monkeypatch):
    """A 404 is 'this server predates the route', which is a real state --
    the stacks are redeployed one at a time. It must read as 'no mapper',
    never as a transport failure, and never as a mapper that CRASHED:
    'I have no map' is something 1.5's bootstrap rule can work with and
    'the mapper died' is not."""
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from control.remote_world import RemoteWorld

    bare = FastAPI()  # no /world/* routes at all
    client = TestClient(bare, base_url="http://old.test")
    world = RemoteWorld("http://old.test", client=client)

    assert world.get_pose() == unusable_pose()
    assert world.get_map() == unusable_map()
    client.close()


def test_a_broken_mapper_is_not_reported_as_an_absent_one():
    """Every status but 404 raises. The two states must never collapse:
    one is a fact the planner plans around, the other is an outage."""
    from fastapi import FastAPI, HTTPException
    from fastapi.testclient import TestClient

    from control.remote_world import RemoteWorld, WorldTransportError

    app = FastAPI()

    @app.get("/world/pose")
    def boom():
        raise HTTPException(status_code=500, detail="mapper died")

    client = TestClient(app, base_url="http://broken.test", raise_server_exceptions=False)
    world = RemoteWorld("http://broken.test", client=client)
    with pytest.raises(WorldTransportError):
        world.get_pose()
    client.close()
