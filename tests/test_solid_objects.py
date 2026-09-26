"""
tests/test_solid_objects.py

`PLAN-ros-alignment.md` 3.9, decided by the user: "objects must be treated
as solid to emulate the real world." Solid to everything that SENSES or
MOVES; billboards to the camera. One test per acceptance criterion.
"""

import math

from sim import renderer
from sim.grid_world import Heading
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import DEFAULT_CELL_M, MockRobot
from sim.mock_world import MockWorld
from world.interface import CELL_OCCUPIED
from tests.test_bearing_turns import CLEAR_STARTS, GOAL, STEPS, TARGET, _quiet_cloud
from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.mission_runner import MissionRunner
from tests.conftest import mock_world_for

BACKPACK = (10, 7)


def _facing_backpack_from(x, y):
    grid = build_starter_world()
    grid.x, grid.y = x, y
    grid.heading = Heading.E
    return grid, MockRobot(grid, render=False)


def test_criterion_1_the_robot_never_enters_an_objects_cell():
    """Every clear start, ticked through a real mission; checked at every
    step, not only at the end -- driving THROUGH an object is the bug."""
    for (x, y, off) in CLEAR_STARTS:
        grid = build_starter_world()
        grid.x, grid.y = x, y
        grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
        robot = MockRobot(grid, render=False)
        tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                            steer_on_sight=True, hold_goal=True)
        runner = MissionRunner(robot, target_object=TARGET, max_steps=STEPS,
                               policy="tiered", vision_fn=tier,
                               world=mock_world_for(robot))
        runner.start()
        while runner.tick():
            assert (grid.robot_x, grid.robot_y) not in grid.solid_cells, (
                f"from {(x, y, off)} the robot drove into an object at "
                f"{(grid.robot_x, grid.robot_y)}")


def test_criterion_2_the_sensors_see_the_backpack_not_the_wall_behind_it():
    grid, robot = _facing_backpack_from(8.5, 7.5)
    face_cells = BACKPACK[0] - 8.5  # centre to the backpack's near face
    assert robot.get_scan()["ranges_m"][180] == \
        __import__("pytest").approx(face_cells * DEFAULT_CELL_M, abs=0.02)
    # The scalar speaks in whole free cells ahead of the robot's own: one.
    assert robot.get_distance() == 30.0
    path = [z for i, z in enumerate(robot.get_depth_grid()["zones"]) if i in (3, 4)]
    assert all(z["distance_cm"] is not None and z["distance_cm"] < 35 for z in path)


def test_criterion_3_a_seen_object_is_occupied_on_the_map():
    grid, robot = _facing_backpack_from(8.5, 7.5)
    world = MockWorld(grid)
    m = world.get_map()
    assert m["cells"][BACKPACK[1] * m["width"] + BACKPACK[0]] == CELL_OCCUPIED


def test_criterion_4_the_camera_still_draws_objects_as_objects():
    """The picture does not change: the camera's wall profile passes no
    solid set, so the backpack is a billboard, not a grey wall column. The
    golden image in tests/test_renderer.py is the byte-level guard; this
    pins the mechanism."""
    grid, _ = _facing_backpack_from(8.5, 7.5)
    walls_only = renderer.cast_ray(grid.layout, grid.x, grid.y, 0.0)
    with_objects = renderer.cast_ray(grid.layout, grid.x, grid.y, 0.0,
                                     solid=grid.solid_cells)
    assert walls_only > with_objects, "the render must see past the object"
    names = [o["name"] for o in renderer._visible_objects(
        grid.layout, grid.objects, grid.x, grid.y, 0.0)]
    assert TARGET in names, "an object must not occlude itself"


def test_the_robot_does_not_start_inside_an_object():
    """The starter house started the robot on the sofa's cell -- harmless
    while objects were floor labels, impossible once they are solid."""
    grid = build_starter_world()
    assert (grid.robot_x, grid.robot_y) not in grid.solid_cells
