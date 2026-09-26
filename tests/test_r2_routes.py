"""
tests/test_r2_routes.py

Phase R2's read-only half (`PLAN-ros-alignment.md` 3.6): wheel state, the
lidar scan and ground truth -- the three readings R4's ROS nodes and R5's
error readout are built on. Each numbered block is one of 3.6's acceptance
criteria; the shape-on-every-backend criterion lives with the rest of the
contract in `tests/test_robot_contract.py` and `tests/test_world_contract.py`.
"""

import math

import httpx
import pytest

from control.remote_robot import RemoteRobot
from control.remote_world import RemoteWorld
from sim import renderer
from sim.grid_world import Heading
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import DEFAULT_CELL_M, MockRobot
from sim.mock_world import MockWorld
from world.interface import NullWorld


def _robot_at(x, y, heading):
    grid = build_starter_world()
    grid.x, grid.y = x, y
    grid.heading = heading
    return MockRobot(grid, render=False)


# ---------- criterion 2: scan geometry ----------


def test_every_beam_is_the_renderers_own_ray():
    """The scan is cast on the same geometry the picture and the map are
    built from, so the three cannot disagree about where a wall is."""
    robot = _robot_at(5.5, 7.5, Heading.E)
    scan = robot.get_scan()
    for i, r in enumerate(scan["ranges_m"]):
        angle = robot.world.theta + math.radians(scan["angle_min_deg"] + i * scan["angle_increment_deg"])
        # Objects are solid to sensing (3.9), so the lidar's ray is the one
        # that stops at them too.
        cells = renderer.cast_ray(robot.world.layout, robot.world.x, robot.world.y,
                                  angle, solid=robot.world.solid_cells)
        if cells >= renderer.FPV_MAX_DIST:
            assert r is None
        else:
            assert r == pytest.approx(cells * DEFAULT_CELL_M, abs=renderer.FPV_STEP * DEFAULT_CELL_M)


def test_a_beam_through_the_kitchen_door_reaches_past_it_to_the_backpack():
    """From the hallway row facing the door, the beam dead ahead goes
    through the doorway -- not stopping at it, which would draw the door as
    a wall on every map -- and returns off the backpack's face, since
    objects are solid (3.9). Before that it passed through the backpack to
    the kitchen wall behind it (1.95 m)."""
    robot = _robot_at(5.5, 7.5, Heading.E)
    scan = robot.get_scan()
    ahead = scan["ranges_m"][180]  # angle_min -180, 1 degree per beam
    door_m = (8 - 5.5) * DEFAULT_CELL_M
    backpack_face_m = (10 - 5.5) * DEFAULT_CELL_M
    assert ahead > door_m + DEFAULT_CELL_M
    assert ahead == pytest.approx(backpack_face_m, abs=0.05)


def test_the_scan_is_in_the_body_frame_and_ignores_the_camera_pan():
    """The lidar is on the deck and does not pan. A scan that swung with
    look-left would put every wall on the map 90 degrees out."""
    robot = _robot_at(5.5, 7.5, Heading.E)
    before = robot.get_scan()["ranges_m"]
    robot.look_left()
    assert robot.get_scan()["ranges_m"] == before
    robot.turn_left(90)
    assert robot.get_scan()["ranges_m"] != before, "a real turn must rotate it"


# ---------- criterion 3: truth ----------


def test_in_the_sim_truth_is_the_pose_today():
    """They part at R5, when the pose comes from slam_toolbox; the identity
    now is what makes the later difference mean something."""
    grid = build_starter_world()
    world = MockWorld(grid)
    MockRobot(grid, render=False).turn_left(30)
    pose, truth = world.get_pose(), world.get_truth()
    assert truth["usable"] is True and truth["source"] == "sim"
    for k in ("x_m", "y_m", "heading_deg"):
        assert truth[k] == pose[k]


def test_anything_but_a_simulator_has_no_truth():
    truth = NullWorld().get_truth()
    assert truth["usable"] is False
    assert truth["x_m"] is None and truth["heading_deg"] is None


# ---------- criterion 4: over the wire ----------


def test_wheels_scan_and_truth_survive_the_socket(robot_and_world_over_asgi):
    """The same actions, in process and over HTTP, give the same three
    readings -- the property R4's plugin depends on, since it will only
    ever see them over the wire."""
    remote, remote_world = robot_and_world_over_asgi
    local = MockRobot(build_starter_world())
    local_world = MockWorld(local.world)
    for bot in (remote, local):
        bot.drive_forward(50, 0.5)
        bot.turn_left(30)
    assert remote.get_wheel_state() == local.get_wheel_state()
    # The server stamps a scan when it TAKES it (R5, `stamp_unix`) -- when,
    # not what, so it is set aside here and checked for separately.
    over_the_wire = remote.get_scan()
    assert isinstance(over_the_wire.pop("stamp_unix"), float)
    assert over_the_wire == local.get_scan()
    assert remote_world.get_truth() == local_world.get_truth()


@pytest.mark.parametrize("path, call", [
    ("/wheels", lambda c: RemoteRobot("http://robot.test", client=c).get_wheel_state()),
    ("/scan", lambda c: RemoteRobot("http://robot.test", client=c).get_scan()),
    ("/world/truth", lambda c: RemoteWorld("http://robot.test", client=c).get_truth()),
])
def test_a_server_predating_r2_reads_as_unusable_not_as_an_error(path, call):
    """The stacks redeploy one at a time, so an older server is a real
    state -- and must read as 'no such sensor', never as a transport error
    that would spend a mission's failure budget."""
    def handler(request):
        if request.url.path == path:
            return httpx.Response(404, json={"detail": "Not Found"})
        return httpx.Response(200, json={})

    assert call(httpx.Client(transport=httpx.MockTransport(handler)))["usable"] is False


def test_a_broken_server_is_not_reported_as_an_absent_sensor():
    """Every non-404 failure raises: a broken lidar is not a missing one."""
    from control.remote_robot import RobotTransportError

    client = httpx.Client(transport=httpx.MockTransport(
        lambda r: httpx.Response(500, json={"detail": "lidar crashed"})))
    with pytest.raises(RobotTransportError):
        RemoteRobot("http://robot.test", client=client).get_scan()
