"""
tests/test_bridge_convert.py

picar_bridge's conversions at the wall, on a laptop (2026-09-27).

Until today the bridge's scan reorder and quaternion maths were exercised
only through the live container (tests/test_ros_chain_live.py's
beam-for-beam check). They are the two places a lidar picture comes out
MIRRORED -- ours is clockwise, ROS's counter-clockwise -- so they are now
plain Python (service/slam/src/picar_bridge/picar_bridge/convert.py) and
pinned here without ROS. Importing it needs no rclpy: that is checked too.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import convert  # noqa: E402


def _scan(values_by_bearing, n=360, start=-180.0, inc=1.0, default=5.0):
    """Our convention: beam i at start + i*inc, CLOCKWISE-positive."""
    ours = [default] * n
    for bearing, v in values_by_bearing.items():
        ours[int(round((bearing - start) / inc)) % n] = v
    return ours


def _ros_at(ranges, ros_deg, start=-180.0, inc=1.0):
    return ranges[int(round((ros_deg - start) / inc)) % len(ranges)]


def test_something_to_the_right_is_at_minus_ninety_in_ros():
    """Our +90 is clockwise = RIGHT. ROS's right is -90 (CCW-positive)."""
    ranges = convert.ros_ranges(_scan({90.0: 1.0}), -180.0, 1.0)
    assert _ros_at(ranges, -90.0) == 1.0
    assert _ros_at(ranges, 90.0) == 5.0, "mirrored: the obstacle appeared on the LEFT"


@pytest.mark.parametrize("bearing", [0.0, 45.0, -30.0, 179.0, -179.0])
def test_every_direction_is_negated_not_shifted(bearing):
    ranges = convert.ros_ranges(_scan({bearing: 0.7}), -180.0, 1.0)
    assert _ros_at(ranges, -bearing) == 0.7


def test_straight_ahead_and_straight_behind_do_not_move():
    ranges = convert.ros_ranges(_scan({0.0: 0.4, -180.0: 0.9}), -180.0, 1.0)
    assert _ros_at(ranges, 0.0) == 0.4 and _ros_at(ranges, -180.0) == 0.9


def test_no_return_is_infinity_in_ros():
    ranges = convert.ros_ranges(_scan({10.0: None}), -180.0, 1.0)
    assert math.isinf(_ros_at(ranges, -10.0)) and _ros_at(ranges, -10.0) > 0


def test_a_coarser_scan_converts_the_same_way():
    ranges = convert.ros_ranges(_scan({90.0: 1.5}, n=180, inc=2.0), -180.0, 2.0)
    assert _ros_at(ranges, -90.0, inc=2.0) == 1.5


def test_a_real_sim_scan_facing_a_wall_reads_the_wall_ahead_in_ros():
    from sim.maps.starter_house import build_starter_world
    from sim.mock_robot import MockRobot
    robot = MockRobot(build_starter_world(), render=False)
    s = robot.get_scan()
    ranges = convert.ros_ranges(s["ranges_m"], s["angle_min_deg"], s["angle_increment_deg"])
    for bearing in (0.0, 60.0, -120.0):
        ours = s["ranges_m"][int(round((bearing - s["angle_min_deg"]) / s["angle_increment_deg"]))]
        ros = _ros_at(ranges, -bearing, s["angle_min_deg"], s["angle_increment_deg"])
        assert (math.isinf(ros) and ours is None) or ros == ours


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, math.pi - 0.01])
def test_yaw_from_a_pure_z_rotation(yaw):
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    y, p, r = convert.yaw_pitch_roll(*q)
    assert y == pytest.approx(yaw) and p == pytest.approx(0.0) and r == pytest.approx(0.0)


def test_pitch_from_a_pure_y_rotation():
    pitch = 0.2618                                    # the camera's 15 degrees down
    y, p, r = convert.yaw_pitch_roll(0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    assert p == pytest.approx(pitch) and y == pytest.approx(0.0)


def test_the_module_imports_no_ros():
    assert "rclpy" not in sys.modules


# ---------- 3.36: the anchor from truth and odometry at one instant ----------

def _house_pose_after(start, odom):
    """The CONSUMER's map -- world/ros_world.py's own anchor and conversion,
    not a copy -- from an odometry pose to the house, for a session whose
    start truth is `start`."""
    from world.ros_world import RosWorld, _ros_to_ours
    w = RosWorld("http://unused", truth=object())     # a sim: it anchors on start_truth
    w._ensure_session({"session": "s", "map": {}, "start_truth": start})
    x, y, h = _ros_to_ours(odom["x_m"], odom["y_m"], odom["yaw_rad"])
    hx, hy, hh = w._apply(x, y, h)
    return {"x_m": hx, "y_m": hy, "heading_deg": hh}


def test_the_anchor_is_recovered_after_the_robot_has_moved():
    """Whatever the robot did since odometry zero, truth and odometry at one
    instant give back the house pose of odometry zero exactly."""
    from picar_bridge.convert import odometry_zero_in_house
    for start in ({"x_m": 1.95, "y_m": 1.35, "heading_deg": 90.0},
                  {"x_m": 2.05, "y_m": 0.93, "heading_deg": 72.8},
                  {"x_m": -3.0, "y_m": 7.5, "heading_deg": 301.0}):
        for odom in ({"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                     {"x_m": 0.42, "y_m": -0.31, "yaw_rad": 0.33},
                     {"x_m": -1.7, "y_m": 2.2, "yaw_rad": -2.9}):
            truth = _house_pose_after(start, odom)
            got = odometry_zero_in_house(truth, odom)
            assert math.isclose(got["x_m"], start["x_m"], abs_tol=1e-9), (start, odom, got)
            assert math.isclose(got["y_m"], start["y_m"], abs_tol=1e-9), (start, odom, got)
            dh = (got["heading_deg"] - start["heading_deg"] + 180) % 360 - 180
            assert abs(dh) < 1e-9, (start, odom, got)


def test_at_odometry_zero_the_anchor_is_the_truth():
    from picar_bridge.convert import odometry_zero_in_house
    truth = {"x_m": 1.95, "y_m": 1.35, "heading_deg": 90.0}
    assert odometry_zero_in_house(truth, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}) == truth
