"""
tests/test_ros_world.py

Phase R5 (`PLAN-ros-alignment.md` 3.14) -- `world/ros_world.py`, the
`WorldInterface` over slam_toolbox, against a FAKE bridge in-process.

What is pinned is the conversion at the wall, because it is the one place a
map can come out correct and mirrored (`world/interface.py`: "the kind of
thing that survives a long way before anyone notices"): ROS's x-forward,
y-LEFT, counter-clockwise frame into the project's x-east, y-SOUTH,
clockwise-compass one, plus the sim-only anchor to the house.
"""

import math

import httpx
import pytest

from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN
from world.ros_world import RosWorld


class FakeBridge:
    """Answers /slam/pose and /slam/map as picar_bridge does: ROS frame,
    ROS convention, a session id."""

    def __init__(self, session="s1", start_truth=None):
        self.session = session
        self.start_truth = start_truth
        self.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        self.odom_pose = dict(self.map_pose)
        self.version = 1
        # 10 x 10 cells at 0.1 m, origin (-0.5, -0.5): the robot at the
        # centre. One occupied cell 0.3 m straight AHEAD (ROS +x) and one
        # 0.3 m to the LEFT (ROS +y); everything else free.
        self.grid = [0] * 100
        self.grid[5 * 10 + 8] = 100      # row j=5 (y ~ 0.05), col i=8 (x ~ 0.35)
        self.grid[8 * 10 + 5] = 100      # row j=8 (y ~ 0.35), col i=5 (x ~ 0.05)
        self.grid[0] = -1

    def handler(self, request):
        if request.url.path == "/slam/pose":
            return httpx.Response(200, json={"session": self.session, "map": self.map_pose,
                                             "odom": self.odom_pose,
                                             "start_truth": self.start_truth})
        if request.url.path == "/slam/map":
            return httpx.Response(200, json={
                "session": self.session, "version": self.version, "resolution_m": 0.1,
                "width": 10, "height": 10, "origin_x_m": -0.5, "origin_y_m": -0.5,
                "origin_yaw_rad": 0.0, "data": self.grid})
        return httpx.Response(404)


class FakeTruth:
    def __init__(self, x, y, heading):
        self.pose = {"usable": True, "source": "sim", "x_m": x, "y_m": y, "heading_deg": heading}

    def get_truth(self):
        return dict(self.pose)


def ros_world(bridge=None, truth=None):
    # The real bridge records the truth at the session's start; a fake with
    # a truth does too, so the tests exercise the anchor that ships.
    bridge = bridge or FakeBridge(
        start_truth={k: truth.pose[k] for k in ("x_m", "y_m", "heading_deg")} if truth else None)
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client), bridge


def test_the_first_pose_is_anchored_to_the_truth():
    world, _ = ros_world(truth=FakeTruth(0.75, 0.75, 90.0))
    pose = world.get_pose()
    assert pose["usable"] and pose["map_id"] == "slam-s1"
    assert pose["x_m"] == pytest.approx(0.75) and pose["y_m"] == pytest.approx(0.75)
    assert pose["heading_deg"] == pytest.approx(90.0)


@pytest.mark.parametrize("start_heading", [0.0, 90.0, 180.0, 270.0, 37.0])
def test_driving_forward_in_ros_is_driving_along_the_true_heading(start_heading):
    world, bridge = ros_world(truth=FakeTruth(1.0, 1.0, start_heading))
    world.get_pose()                                   # anchor
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}   # 0.5 m ahead in ROS
    pose = world.get_pose()
    h = math.radians(start_heading)
    # compass: heading 0 is -y (north), 90 is +x (east)
    assert pose["x_m"] == pytest.approx(1.0 + 0.5 * math.sin(h), abs=1e-9)
    assert pose["y_m"] == pytest.approx(1.0 - 0.5 * math.cos(h), abs=1e-9)


def test_a_left_turn_in_ros_is_a_counter_clockwise_compass_change():
    world, bridge = ros_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": math.radians(30)}   # 30 deg LEFT
    assert world.get_pose()["heading_deg"] == pytest.approx(60.0)


def test_the_map_is_not_mirrored():
    """Facing east at (1, 1): ROS's 'ahead' obstacle must land EAST of the
    robot and ROS's 'left' obstacle NORTH of it (smaller y_m)."""
    world, _ = ros_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    m = world.get_map()
    assert m["usable"] and len(m["cells"]) == m["width"] * m["height"]
    occupied = []
    for row in range(m["height"]):
        for col in range(m["width"]):
            if m["cells"][row * m["width"] + col] == CELL_OCCUPIED:
                occupied.append((m["origin_x_m"] + (col + 0.5) * m["resolution_m"],
                                 m["origin_y_m"] + (row + 0.5) * m["resolution_m"]))
    assert len(occupied) == 2, occupied
    east = max(occupied, key=lambda p: p[0])
    north = min(occupied, key=lambda p: p[1])
    assert east[0] == pytest.approx(1.35, abs=0.06) and east[1] == pytest.approx(0.95, abs=0.06)
    assert north[0] == pytest.approx(1.05, abs=0.06) and north[1] == pytest.approx(0.65, abs=0.06)
    assert set(m["cells"]) <= {CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN}


def test_a_restarted_container_is_a_new_map_and_a_new_anchor():
    world, bridge = ros_world(truth=FakeTruth(1.0, 1.0, 90.0))
    first = world.get_pose()["map_id"]
    bridge.session = "s2"
    assert world.get_pose()["map_id"] == "slam-s2" != first


def test_on_hardware_there_is_no_anchor_and_no_truth():
    world, bridge = ros_world(truth=None)
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    pose = world.get_pose()
    assert pose["x_m"] == pytest.approx(0.5) and pose["y_m"] == pytest.approx(0.0)
    assert pose["heading_deg"] == pytest.approx(90.0), "ROS +x is compass 90 in SLAM's own frame"
    assert world.get_truth()["usable"] is False


def test_odometry_is_reported_in_the_same_frame_so_the_two_can_be_compared():
    world, bridge = ros_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.odom_pose = {"x_m": 0.4, "y_m": 0.0, "yaw_rad": 0.0}
    o = world.get_odom_pose()
    assert o["x_m"] == pytest.approx(1.4) and o["y_m"] == pytest.approx(1.0)


def test_a_sideways_displacement_in_ros_lands_on_the_correct_side():
    """ROS +y is LEFT. Facing east, left is north -- smaller y_m. The test
    the forward-only cases above could not make: a y-flip deleted from the
    pose conversion passed all of them."""
    world, bridge = ros_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.5, "yaw_rad": 0.0}
    pose = world.get_pose()
    assert pose["x_m"] == pytest.approx(1.0) and pose["y_m"] == pytest.approx(0.5)


def test_the_anchor_is_the_sessions_start_not_the_first_question():
    """The defect the first phone-size check showed: the twin opened after
    eight moves, anchored on that moment, and read 'SLAM error 0.0 cm'.
    SLAM believes it drove 0.5 m ahead; the robot truly went 0.52 m. The
    pose must say 0.5 -- the estimate -- however late the first question."""
    truth = FakeTruth(1.0, 1.0, 90.0)
    world, bridge = ros_world(truth=truth)
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}     # SLAM's belief
    truth.pose.update(x_m=1.52)                                      # where it really is
    pose = world.get_pose()                                          # the FIRST question
    assert pose["x_m"] == pytest.approx(1.5), "anchored on the late question, not the start"
    assert world.anchored_at == "session_start"
