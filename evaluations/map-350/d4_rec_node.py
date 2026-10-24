"""Inside the ROS container (3.50.D4, amendment 4's data): record every /map,
each /scan with map -> laser at its own stamp, and the wheel odometry, for
SECS seconds, to OUT (a pickle). Run by d4_record.py with `docker exec`."""
import math
import pickle
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

SECS = float(sys.argv[1])
OUT = sys.argv[2]


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self):
        super().__init__("d4_rec")
        self.maps, self.scans, self.pending, self.odom = [], [], [], []
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 50)

    def on_map(self, m):
        i = m.info
        self.maps.append({"t": time.time(), "w": i.width, "h": i.height, "res": i.resolution,
                          "ox": i.origin.position.x, "oy": i.origin.position.y,
                          "data": bytes(bytearray((v & 0xff) for v in m.data))})

    def on_odom(self, o):
        self.odom.append((time.time(), o.twist.twist.linear.x, o.twist.twist.angular.z))

    def on_scan(self, s):
        self.pending.append({"angle_min_rad": s.angle_min, "angle_increment_rad": s.angle_increment,
                             "ranges_m": [None if not math.isfinite(v) else v for v in s.ranges],
                             "range_min_m": s.range_min, "range_max_m": s.range_max,
                             "stamp_ns": s.header.stamp.sec * 1_000_000_000 + s.header.stamp.nanosec,
                             "at": time.time()})
        keep = []
        for p in self.pending:
            try:
                tr = self.buf.lookup_transform("map", "laser", Time(nanoseconds=p["stamp_ns"]))
            except Exception:  # noqa: BLE001 -- not in yet
                if time.time() - p["at"] < 0.5:
                    keep.append(p)
                continue
            t = tr.transform.translation
            p["laser"] = (t.x, t.y, yaw(tr.transform.rotation))
            self.scans.append(p)
        self.pending = keep


rclpy.init()
n = Rec()
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(OUT, "wb") as f:
    pickle.dump({"maps": n.maps, "scans": n.scans, "odom": n.odom}, f)
print("maps", len(n.maps), "scans", len(n.scans), "odom", len(n.odom))
