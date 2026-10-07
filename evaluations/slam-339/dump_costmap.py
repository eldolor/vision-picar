import json, sys, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node("costmap_dump")
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m): got["m"] = m
n.create_subscription(OccupancyGrid, sys.argv[1], cb, qos)
while "m" not in got: rclpy.spin_once(n, timeout_sec=1.0)
m = got["m"]; i = m.info
json.dump({"res": i.resolution, "w": i.width, "h": i.height, "ox": i.origin.position.x,
           "oy": i.origin.position.y, "data": list(m.data)}, open(sys.argv[2], "w"))
print("dumped", sys.argv[1], i.width, i.height, i.resolution)
