"""picar_bridge -- the HTTP side of the ROS wall (PLAN-ros-alignment.md R4).

`robot/server.py` may not import rclpy (tests/test_ros_containment.py), and
the D-pad and the brain speak verbs. So verbs become twists OUTSIDE ROS
(`robot/ros_drive.py`, closed on the wheel encoders) and cross into ROS here:

    POST /cmd_vel {driver, linear_m_s, angular_rad_s}
        -> geometry_msgs/Twist on that driver's twist_mux input

and the other direction:

    GET /odom            diff_drive_controller's /odom, latest
    GET /scan            the LaserScan this node last published
    GET /tf?target=&source=   a tf2 lookup, for R3's measurements
    POST /pan {angle_rad}     R3's test hook: hold the pan joint at an angle
    GET /health

It also REPUBLISHES the robot's scan (GET <robot>/scan) as
sensor_msgs/LaserScan on /scan -- the R4 table's `sim_scan_node`, folded in
here because it is the same HTTP client -- and publishes the pan joint so
robot_state_publisher can place camera_link.

Two conversions happen here and nowhere else, because this is the wall:
the project's bearings are CLOCKWISE-positive and ROS's are
counter-clockwise (REP-103), and a beam with no return is None on our side
and +inf on ROS's (REP-117).
"""

import json
import math
import os
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

# Driver (robot/interface.py DRIVER_PRIORITY names) -> twist_mux input.
# Priorities live in picar_bringup/config/twist_mux.yaml.
DRIVER_TOPICS = {
    "twin-dpad": "cmd_vel/teleop",
    "brain": "cmd_vel/brain",
    "ros": "cmd_vel/nav",
}
SCAN_HZ = 10.0
PAN_HZ = 20.0


def _yaw_pitch_roll(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


class Bridge(Node):
    def __init__(self):
        super().__init__("picar_bridge")
        self.robot_url = os.environ.get("ROBOT_URL", "http://host.docker.internal:8000")
        self.secret = os.environ.get("APP_SHARED_SECRET", "")
        self.lock = threading.Lock()
        self.pubs = {d: self.create_publisher(Twist, t, 10) for d, t in DRIVER_TOPICS.items()}
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self._on_odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom = None
        self.last_scan = None
        self.scan_count = 0
        self.twists = {d: 0 for d in DRIVER_TOPICS}
        self.pan_rad = 0.0
        self.create_timer(1.0 / SCAN_HZ, self._poll_scan)
        self.create_timer(1.0 / PAN_HZ, self._publish_pan)

    # ---------- ROS side ----------

    def _on_odom(self, msg):
        yaw, _, _ = _yaw_pitch_roll(msg.pose.pose.orientation)
        with self.lock:
            self.last_odom = {
                "x_m": msg.pose.pose.position.x, "y_m": msg.pose.pose.position.y,
                "yaw_rad": yaw, "linear_m_s": msg.twist.twist.linear.x,
                "angular_rad_s": msg.twist.twist.angular.z, "at": time.time()}

    def _robot_get(self, path):
        req = urllib.request.Request(self.robot_url + path)
        if self.secret:
            req.add_header("x-app-secret", self.secret)
        with urllib.request.urlopen(req, timeout=0.5) as r:
            return json.loads(r.read())

    def _poll_scan(self):
        try:
            s = self._robot_get("/scan")
        except Exception as e:  # noqa: BLE001 -- keep polling; say why once in a while
            self.get_logger().warn(f"scan poll failed: {e}", throttle_duration_sec=5.0)
            return
        if not s.get("usable") or not s.get("ranges_m"):
            return
        ours = s["ranges_m"]
        n = len(ours)
        inc_deg = s["angle_increment_deg"]
        # Ours: beam i at start + i*inc, CLOCKWISE. ROS: CCW. Walking ROS's
        # beams from -180 CCW is walking ours from +180 clockwise-backwards,
        # i.e. ours[(-j) mod n] when ours starts at -180 in 1-degree steps.
        start = s["angle_min_deg"]
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = math.radians(start)
        msg.angle_increment = math.radians(inc_deg)
        msg.angle_max = msg.angle_min + msg.angle_increment * (n - 1)
        msg.range_min = float(s["range_min_m"])
        msg.range_max = float(s["range_max_m"])
        msg.scan_time = 1.0 / SCAN_HZ
        ranges = []
        for j in range(n):
            ros_deg = start + j * inc_deg          # CCW-positive angle of beam j
            ours_deg = -ros_deg                    # the same direction, our way
            i = int(round((ours_deg - start) / inc_deg)) % n
            r = ours[i]
            ranges.append(math.inf if r is None else float(r))
        msg.ranges = ranges
        self.scan_pub.publish(msg)
        with self.lock:
            self.last_scan = {"angle_min_rad": msg.angle_min,
                              "angle_increment_rad": msg.angle_increment,
                              "ranges_m": [None if math.isinf(r) else r for r in ranges],
                              "at": time.time()}
            self.scan_count += 1

    def _publish_pan(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["pan_joint"]
        with self.lock:
            msg.position = [self.pan_rad]
        self.joint_pub.publish(msg)

    # ---------- HTTP side ----------

    def cmd_vel(self, body):
        driver = body.get("driver", "")
        if driver not in self.pubs:
            return 400, {"error": f"unknown driver {driver!r}", "drivers": sorted(self.pubs)}
        t = Twist()
        t.linear.x = float(body.get("linear_m_s", 0.0))
        t.angular.z = float(body.get("angular_rad_s", 0.0))
        self.pubs[driver].publish(t)
        with self.lock:
            self.twists[driver] += 1
        return 200, {"published": DRIVER_TOPICS[driver]}

    def tf(self, target, source):
        try:
            tr = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception as e:  # noqa: BLE001
            return 404, {"error": str(e)}
        t, q = tr.transform.translation, tr.transform.rotation
        yaw, pitch, roll = _yaw_pitch_roll(q)
        return 200, {"translation_m": [t.x, t.y, t.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                     "yaw_rad": yaw, "pitch_rad": pitch, "roll_rad": roll}

    def health(self):
        with self.lock:
            return 200, {"ok": True, "robot_url": self.robot_url,
                         "odom_age_s": None if not self.last_odom else time.time() - self.last_odom["at"],
                         "scan_age_s": None if not self.last_scan else time.time() - self.last_scan["at"],
                         "scans_published": self.scan_count, "twists": dict(self.twists),
                         "pan_rad": self.pan_rad}


def _handler(bridge):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _authorised(self):
            if bridge.secret and self.headers.get("x-app-secret") != bridge.secret:
                self._send(401, {"error": "bad or missing x-app-secret"})
                return False
            return True

        def do_GET(self):  # noqa: N802
            url = urlparse(self.path)
            if url.path == "/health":
                return self._send(*bridge.health())
            if not self._authorised():
                return None
            if url.path == "/odom":
                with bridge.lock:
                    return self._send(200, bridge.last_odom or {"usable": False})
            if url.path == "/scan":
                with bridge.lock:
                    return self._send(200, bridge.last_scan or {"usable": False})
            if url.path == "/tf":
                q = parse_qs(url.query)
                return self._send(*bridge.tf(q.get("target", ["base_link"])[0],
                                             q.get("source", ["laser"])[0]))
            return self._send(404, {"error": "no such route"})

        def do_POST(self):  # noqa: N802
            if not self._authorised():
                return None
            n = int(self.headers.get("content-length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            path = urlparse(self.path).path
            if path == "/cmd_vel":
                return self._send(*bridge.cmd_vel(body))
            if path == "/pan":
                with bridge.lock:
                    bridge.pan_rad = float(body.get("angle_rad", 0.0))
                return self._send(200, {"pan_rad": bridge.pan_rad})
            return self._send(404, {"error": "no such route"})

        def log_message(self, *args):  # the twist stream is 20 Hz; do not log it
            pass

    return Handler


def main():
    rclpy.init()
    bridge = Bridge()
    port = int(os.environ.get("BRIDGE_PORT", "8090"))
    server = ThreadingHTTPServer(("0.0.0.0", port), _handler(bridge))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    bridge.get_logger().info(f"bridge on :{port}, robot at {bridge.robot_url}")
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    try:
        executor.spin()
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
