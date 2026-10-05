"""convert -- the bridge's conversions at the wall, as plain Python.

Kept out of bridge.py (which imports rclpy) so they are unit-tested on a
laptop (tests/test_bridge_convert.py) rather than only through the live
container. These are the two places a picture can come out mirrored:

  * the project's bearings are CLOCKWISE-positive and ROS's are
    counter-clockwise (REP-103);
  * a beam with no return is None on our side and +inf on ROS's (REP-117).
"""

import math


def ros_ranges(ours, start_deg, inc_deg):
    """Our clockwise scan -> the ranges of a ROS LaserScan with the SAME
    angle_min / angle_increment, counter-clockwise.

    ROS beam j sits at start + j*inc, CCW-positive; the same direction in
    our clockwise convention is its negation, which is our beam
    ((-(start + j*inc)) - start) / inc, modulo the beam count.
    """
    n = len(ours)
    out = []
    for j in range(n):
        ours_deg = -(start_deg + j * inc_deg)
        i = int(round((ours_deg - start_deg) / inc_deg)) % n
        r = ours[i]
        out.append(math.inf if r is None else float(r))
    return out


def yaw_pitch_roll(x, y, z, w):
    """A unit quaternion's ZYX Euler angles, radians (yaw CCW-positive)."""
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


def odometry_zero_in_house(truth, odom):
    """Where the robot stood, in the house's frame, when its odometry read
    zero -- from the truth and the odometry taken at the SAME instant
    (PLAN-ros-alignment.md 3.36). This is the anchor world/ros_world.py lays
    SLAM's frame on: `{"x_m", "y_m", "heading_deg"}`, house frame (x east,
    y south, compass heading clockwise from north), with odometry zero
    facing compass 90 in its own frame.

    It used to be the truth read at session start on the assumption the
    robot had not moved; when the container restarts while a person drives
    the fallback (3.24 G3), or the first reads come late on a slow board,
    it had, and every house-frame goal was converted ~20 deg off. Composing
    with the odometry at the same moment needs no such assumption.

    `truth`: {"x_m", "y_m", "heading_deg"}; `odom`: ROS odom frame
    {"x_m", "y_m", "yaw_rad"} (x forward, y left, yaw CCW)."""
    ox, oy = odom["x_m"], -odom["y_m"]                       # ROS -> ours, same frame
    oh = (90.0 - math.degrees(odom["yaw_rad"])) % 360.0
    alpha = (truth["heading_deg"] - oh + 180.0) % 360.0 - 180.0
    c, s = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    return {"x_m": truth["x_m"] - (c * ox - s * oy),
            "y_m": truth["y_m"] - (s * ox + c * oy),
            "heading_deg": (90.0 + alpha) % 360.0}
