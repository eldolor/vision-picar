"""
tests/test_movers.py

PLAN-ros-alignment.md 3.30 -- things that move, in the simulator. Criterion
1 (a move is seen at once) and criterion 4's "off means off" live here,
plus the invariants the design rests on: movers walk deterministically, keep
out of the robot's way, and replace `objects` rather than mutate it.
Criterion 2's ground-truth sweep is `tests/test_mover_safety.py`.
"""

import math
import time

import pytest
from fastapi.testclient import TestClient

from sim import renderer
from sim.grid_world import Heading
from sim.maps import build_movers, build_world
from sim.mock_robot import DEFAULT_CELL_M, MockRobot
from sim.mock_world import MockWorld
from sim.movers import MOVER_KEEPOUT_M, Mover, point_to_cell_cells
from world.interface import CELL_FREE, CELL_OCCUPIED


def _scaled(x=None, y=None, theta=None):
    world = build_world("scaled_house")
    if x is not None:
        world.x, world.y = x, y
    if theta is not None:
        world.theta = theta
    return world


def _ahead_m(robot):
    """The scan's beam dead ahead, in metres."""
    scan = robot.get_scan()
    i = round((0.0 - scan["angle_min_deg"]) / scan["angle_increment_deg"])
    return scan["ranges_m"][i]


# ---------- criterion 1: a move is seen at once ----------


def test_a_moved_sofa_is_in_the_scan_at_once_and_not_where_it_was():
    world = _scaled(5.5, 4.5, math.pi)          # living room, facing west
    robot = MockRobot(world, render=False)
    before = _ahead_m(robot)
    world.move_object((2, 2), (3, 4))           # the sofa, into the line ahead
    after = _ahead_m(robot)
    # The beam now returns off the sofa's east face (x = 4), not the far wall.
    from robot.safety import LIDAR_X_M
    lidar_x = world.x - LIDAR_X_M / DEFAULT_CELL_M
    assert after == pytest.approx((lidar_x - 4.0) * DEFAULT_CELL_M,
                                  abs=renderer.FPV_STEP * DEFAULT_CELL_M)
    assert after < before
    assert (3, 4) in world.solid_cells and (2, 2) not in world.solid_cells


def test_collision_follows_the_move():
    world = _scaled(5.5, 4.5, math.pi)
    world.move_object((2, 2), (3, 4))
    # Toward the new cell: blocked short of it.
    got = world.translate(2.0)
    assert got < 2.0 - 1e-6
    assert world.x - 4.0 >= 0.4                 # the front stopped at the sofa's face
    # Through the old cell: clear now.
    world.x, world.y, world.theta = 2.5, 4.5, -math.pi / 2
    assert world.translate(2.0) == pytest.approx(2.0)
    assert world.robot_y == 2


def test_the_discovered_map_redraws_both_cells_on_the_next_observation():
    world = _scaled(5.5, 4.5, math.pi)
    seen = MockWorld(world)
    seen.observe()
    idx = lambda c: c[1] * world.width + c[0]
    assert seen._cells[idx((2, 2))] == CELL_OCCUPIED
    assert seen._cells[idx((3, 4))] == CELL_FREE
    world.move_object((2, 2), (3, 4))
    seen.observe()
    assert seen._cells[idx((3, 4))] == CELL_OCCUPIED
    assert seen._cells[idx((2, 2))] == CELL_FREE


@pytest.mark.parametrize("src,dst,why", [
    ((9, 9), (3, 4), "no object"),
    ((2, 2), (0, 0), "not floor"),
    ((2, 2), (24, 2), "already holds"),
    ((2, 2), (6, 4), "turning circle"),
])
def test_impossible_moves_are_refused_and_change_nothing(src, dst, why):
    world = _scaled()
    before = dict(world.objects)
    with pytest.raises(ValueError, match=why):
        world.move_object(src, dst)
    assert world.objects == before


# ---------- movers ----------


def _crossing_world(**pose):
    world = _scaled(**pose)
    for m in build_movers("scaled_house", "hallway_crossing"):
        world.add_mover(m)
    return world, world.movers[0]


def test_a_mover_walks_its_path_one_hop_per_period_deterministically():
    world, person = _crossing_world()
    assert person.cell == (12, 4) and world.objects[(12, 4)] == "person"
    world.advance_time(0.5)
    assert person.cell == (12, 4)
    cells = []
    for _ in range(7):
        world.advance_time(1.0)
        cells.append(person.cell)
    assert cells == [(13, 4), (14, 4), (15, 4), (14, 4), (13, 4), (12, 4), (13, 4)]
    assert set(world.objects.values()) >= {"person"}
    assert list(world.objects.values()).count("person") == 1


def test_a_mover_never_hops_inside_the_keep_out_and_waits_instead():
    # Robot standing in the hallway just below the person's row.
    world, person = _crossing_world(x=14.5, y=5.6, theta=-math.pi / 2)
    keepout = math.hypot(*_half_cells()) + MOVER_KEEPOUT_M / DEFAULT_CELL_M
    for _ in range(20):
        world.advance_time(1.0)
        assert point_to_cell_cells(world.x, world.y, person.cell) >= keepout - 1e-9
    assert person.waits > 0
    assert (14, 4) not in world.objects


def _half_cells():
    from sim.grid_world import FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH
    return FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH


def test_a_mover_beside_the_robot_steps_past_instead_of_freezing():
    """3.30's second live run: the robot 27 cm from the person's next cell,
    the person waiting on the robot and nav2 waiting on the person. A step
    that keeps the distance, or opens it, is allowed; one that closes it
    never is."""
    world, person = _crossing_world()
    world.x, world.y, world.theta = 12.9, 5.9, -math.pi / 2   # the robot came up to it
    start = point_to_cell_cells(world.x, world.y, person.cell)
    assert start < math.hypot(*_half_cells()) + MOVER_KEEPOUT_M / DEFAULT_CELL_M
    seen = [person.cell]
    for _ in range(6):
        world.advance_time(1.0)
        seen.append(person.cell)
        assert point_to_cell_cells(world.x, world.y, person.cell) >= min(
            start, math.hypot(*_half_cells()) + MOVER_KEEPOUT_M / DEFAULT_CELL_M) - 1e-9
    assert len(set(seen)) > 1


def test_a_hop_replaces_objects_rather_than_mutating_it():
    """The robot server reads the scan on its threadpool while the wheel loop
    steps the world; a dict resized under an iterator raises."""
    world, _person = _crossing_world()
    held = world.objects
    snapshot = dict(held)
    world.advance_time(1.0)
    assert world.objects is not held
    assert held == snapshot


def test_the_clock_follows_the_robot_and_idle_time():
    world, person = _crossing_world()
    robot = MockRobot(world, render=False)
    robot.drive_wheels(1.0, 1.0, 0.6)
    assert world.sim_time == pytest.approx(0.6)
    robot.pass_time(0.5)
    assert world.sim_time == pytest.approx(1.1)
    assert person.cell == (13, 4)


@pytest.mark.parametrize("path,why", [
    ([(12, 4), (14, 4)], "not 4-adjacent"),
    ([(12, 4)], "at least two"),
])
def test_bad_paths_are_refused(path, why):
    with pytest.raises(ValueError, match=why):
        Mover("cat", path)


def test_a_path_through_a_wall_or_an_object_is_refused():
    world = _scaled()
    with pytest.raises(ValueError, match="not floor"):
        world.add_mover(Mover("cat", [(10, 1), (11, 1)]))
    with pytest.raises(ValueError, match="already holds"):
        world.add_mover(Mover("cat", [(2, 2), (3, 2)]))


def test_a_mover_cannot_start_inside_the_keep_out():
    world = _scaled(12.5, 5.6, -math.pi / 2)
    with pytest.raises(ValueError, match="keep-out"):
        world.add_mover(Mover("cat", [(12, 4), (13, 4)]))
    assert world.movers == [] and (12, 4) not in world.objects


def test_an_unknown_scenario_names_the_known_ones():
    with pytest.raises(ValueError, match="hallway_crossing"):
        build_movers("scaled_house", "stampede")
    with pytest.raises(ValueError, match="has none"):
        build_movers("starter_house", "hallway_crossing")


# ---------- criterion 4: off means off ----------


def test_without_movers_the_world_is_untouched_by_time():
    world = _scaled()
    robot = MockRobot(world, render=False)
    held = world.objects
    for _ in range(5):
        robot.drive_forward()
        robot.turn_left(45)
    assert world.objects is held
    assert world.movers == []


# ---------- the routes, and the server's idle clock ----------


def _client(monkeypatch, **env):
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    from robot.server import create_app
    return TestClient(create_app())


def test_routes_list_and_move_objects(monkeypatch):
    client = _client(monkeypatch, ROBOT_MODE="sim", SIM_MAP="scaled_house",
                     SIM_MOVERS="hallway_crossing")
    listed = client.get("/sim/objects").json()
    by_cell = {(o["x"], o["y"]): o for o in listed["objects"]}
    assert by_cell[(12, 4)] == {"x": 12, "y": 4, "name": "person", "mover": True}
    assert by_cell[(2, 2)]["mover"] is False
    moved = client.post("/sim/objects/move", json={"src": [2, 2], "dst": [3, 2]})
    assert moved.status_code == 200
    cells = {(o["x"], o["y"]) for o in moved.json()["objects"]}
    assert (3, 2) in cells and (2, 2) not in cells
    refused = client.post("/sim/objects/move", json={"src": [12, 4], "dst": [12, 5]})
    assert refused.status_code == 409 and "mover" in refused.json()["detail"]


def test_routes_are_sim_only(monkeypatch):
    client = _client(monkeypatch, ROBOT_MODE="teleop", WORLD_MODE="none")
    assert client.get("/sim/objects").status_code == 501
    assert client.post("/sim/objects/move", json={"src": [1, 1], "dst": [2, 2]}).status_code == 501


def test_a_person_keeps_walking_while_the_robot_waits(monkeypatch):
    """The wheel loop passes idle time to the sim body, so movers move with
    the robot parked -- live, on the server's own clock."""
    client = _client(monkeypatch, ROBOT_MODE="sim", SIM_MAP="scaled_house",
                     SIM_MOVERS="hallway_crossing")
    with client:
        time.sleep(1.4)
        listed = client.get("/sim/objects").json()
    assert listed["sim_time_s"] >= 1.0
    person = [o for o in listed["objects"] if o["mover"]][0]
    assert (person["x"], person["y"]) != (12, 4)
