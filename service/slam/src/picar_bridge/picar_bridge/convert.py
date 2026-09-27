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
