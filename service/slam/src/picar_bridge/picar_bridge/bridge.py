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
    GET /slam/pose       map -> base_footprint (SLAM) and odom -> base_footprint
    GET /slam/map        slam_toolbox's OccupancyGrid, as ROS publishes it
    POST /goal {x_m, y_m, yaw_rad}   R6: a nav2 NavigateToPose goal, map frame
    GET /goal            its state and the planned path (map frame)
    DELETE /goal         cancel it
    GET /nav/stats, POST /nav/stats/reset   R6's criteria 5 and 6: angular
                         reversals in what REACHED the wheels, and how often
                         collision_monitor stopped or slowed what it was given

(R5) Both /slam routes answer in ROS's OWN frame and convention, tagged with
this bridge's `session` -- a restarted container is a new map, and the
consumer (world/ros_world.py) must be able to tell. Converting to the
project's x-east / y-south / clockwise convention is the consumer's job, on
its side of the wall.
    POST /pan {angle_rad}     R3's test hook: hold the pan joint at an angle
    GET /health

And it shows ROS tools the BRAIN (2026-09-27, brain_view.py): it polls the
brain's GET /mission/status at BRAIN_HZ and publishes /brain/status,
/diagnostics and /brain/markers, so a bag, rviz or Foxglove sees the
mission beside the scans. Outbound only -- no route, and the brain still
knows nothing of ROS. BRAIN_URL="" turns it off.

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
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from picar_bridge import brain_view, convert
from picar_bridge.keepalive import KeepAliveClient

# Driver (robot/interface.py DRIVER_PRIORITY names) -> twist_mux input.
# Priorities live in picar_bringup/config/twist_mux.yaml.
DRIVER_TOPICS = {
    "twin-dpad": "cmd_vel/teleop",
    "brain": "cmd_vel/brain",
    "ros": "cmd_vel/nav",
}
SCAN_HZ = 10.0
PAN_HZ = 20.0
# The brain ticks at ~4 Hz; a status twice a second is enough to follow it
# and cheap enough to leave on.
BRAIN_HZ = 2.0


def _yaw_pitch_roll(q):
    return convert.yaw_pitch_roll(q.x, q.y, q.z, q.w)


class Bridge(Node):
    def __init__(self):
        super().__init__("picar_bridge")
        self.robot_url = os.environ.get("ROBOT_URL", "http://host.docker.internal:8000")
        self.secret = os.environ.get("APP_SHARED_SECRET", "")
        # Kept-open connections (keepalive.py, PLAN-ros-alignment.md 3.24 G1):
        # a new connection per poll through Docker Desktop's port-forwarding
        # stalled past the timeout ~1 time in 10, and each stall was a scan
        # never published.
        auth = {"x-app-secret": self.secret} if self.secret else {}
        self.robot_http = KeepAliveClient(self.robot_url, auth, timeout=0.5)
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
        self.session = uuid.uuid4().hex[:8]
        # R5, evaluation only: where the SIMULATOR says the robot stood when
        # this session began -- i.e. at odometry zero, which is where SLAM's
        # map frame starts. Read once from the robot's sim-only /world/truth
        # (usable: false on hardware, and then nothing is recorded), handed
        # to world/ros_world.py so it can lay SLAM's frame on the house
        # without reading the truth at any later moment. NEVER published
        # into ROS: nothing on this side may navigate by it.
        self.start_truth = None
        self.start_truth_tried = False
        self.last_map = None
        self.map_version = 0
        # slam_toolbox publishes /map latched (transient local).
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # ---- R6 ----
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.goal = None               # {"x_m","y_m","yaw_rad","state","since"}
        self.plan = []
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel_mux", self._on_mux, 10)
        self.create_subscription(Twist, "/diff_drive_controller/cmd_vel_unstamped",
                                 self._on_applied, 10)
        self._reset_stats()
        self.create_timer(1.0 / SCAN_HZ, self._poll_scan)
        self.create_timer(1.0 / PAN_HZ, self._publish_pan)
        # ---- the brain, for ROS tools ----
        self.brain_url = os.environ.get("BRAIN_URL", "http://host.docker.internal:8001/brain").rstrip("/")
        self.brain_http = KeepAliveClient(self.brain_url, auth, timeout=1.0) if self.brain_url else None
        self.brain_status_pub = self.create_publisher(String, "brain/status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.brain_marker_pub = self.create_publisher(MarkerArray, "brain/markers", 10)
        self.brain_at = None
        self.brain_error = None
        self.brain_polls = 0
        if self.brain_url:
            # A thread, not a ROS timer: an HTTP call that times out must not
            # hold an executor thread the nav2 action client needs.
            threading.Thread(target=self._brain_loop, daemon=True).start()

    # ---------- the brain, shown to ROS tools ----------

    def _brain_loop(self):
        while rclpy.ok():
            started = time.time()
            self._poll_brain()
            time.sleep(max(0.0, 1.0 / BRAIN_HZ - (time.time() - started)))

    def _poll_brain(self):
        status, error = None, None
        try:
            status = self.brain_http.get_json("/mission/status")
        except Exception as e:  # unreachable is a STATE here, not a crash
            error = str(e)[:200]
        with self.lock:
            self.brain_polls += 1
            self.brain_error = error
            if status is not None:
                self.brain_at = time.time()
            pan = self.pan_rad
        stamp = self.get_clock().now().to_msg()
        if status is not None:
            self.brain_status_pub.publish(String(data=json.dumps(status)))
        d = brain_view.diagnostic(status, error)
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        arr.status = [DiagnosticStatus(
            level=bytes([d["level"]]), name=d["name"], message=d["message"],
            hardware_id=d["hardware_id"],
            values=[KeyValue(key=k, value=v) for k, v in d["values"]])]
        self.diag_pub.publish(arr)
        self.brain_marker_pub.publish(self._markers(brain_view.markers(status, pan), stamp))

    @staticmethod
    def _markers(specs, stamp):
        out = MarkerArray()
        for spec in specs:
            m = Marker()
            m.header.frame_id = "base_footprint"
            m.header.stamp = stamp
            m.ns = "brain"
            m.id = spec["id"]
            if spec["type"] == "delete":
                m.action = Marker.DELETE
                out.markers.append(m)
                continue
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = spec["colour"]
            if spec["type"] == "text":
                m.type = Marker.TEXT_VIEW_FACING
                m.text = spec["text"]
                m.pose.position.x, m.pose.position.y, m.pose.position.z = spec["position"]
                m.pose.orientation.w = 1.0
                m.scale.z = 0.08
            else:
                m.type = Marker.ARROW
                yaw, length = spec["yaw_rad"], spec["length_m"]
                m.points = [Point(x=0.0, y=0.0, z=0.05),
                            Point(x=length * math.cos(yaw), y=length * math.sin(yaw), z=0.05)]
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = 0.02, 0.05, 0.06
            out.markers.append(m)
        return out

    # ---------- ROS side ----------

    def _on_odom(self, msg):
        yaw, _, _ = _yaw_pitch_roll(msg.pose.pose.orientation)
        with self.lock:
            xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self._last_odom_xy is not None:
                self.stats["metres"] += math.dist(xy, self._last_odom_xy)
            self._last_odom_xy = xy
            self.last_odom = {
                "x_m": msg.pose.pose.position.x, "y_m": msg.pose.pose.position.y,
                "yaw_rad": yaw, "linear_m_s": msg.twist.twist.linear.x,
                "angular_rad_s": msg.twist.twist.angular.z, "at": time.time()}

    def _on_map(self, msg):
        info = msg.info
        yaw, _, _ = _yaw_pitch_roll(info.origin.orientation)
        with self.lock:
            self.map_version += 1
            self.last_map = {
                "session": self.session, "version": self.map_version,
                "resolution_m": info.resolution, "width": info.width, "height": info.height,
                "origin_x_m": info.origin.position.x, "origin_y_m": info.origin.position.y,
                "origin_yaw_rad": yaw, "data": list(msg.data), "at": time.time()}

    # ---------- R6: goals ----------

    def _on_plan(self, msg):
        with self.lock:
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def send_goal(self, body):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return 503, {"error": "nav2 is not up (navigate_to_pose has no server)"}
        self.cancel_goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(body["x_m"])
        ps.pose.position.y = float(body["y_m"])
        yaw = float(body.get("yaw_rad", 0.0))
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        with self.lock:
            self.plan = []
            self.goal = {"x_m": ps.pose.position.x, "y_m": ps.pose.position.y, "yaw_rad": yaw,
                         "state": "pending", "since": time.time()}
            record = self.goal
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, record))
        return 200, {"state": "pending"}

    def _on_goal_response(self, future, record):
        handle = future.result()
        with self.lock:
            if not handle.accepted:
                record["state"] = "rejected"
                return
            if record is not self.goal:          # superseded while pending
                handle.cancel_goal_async()
                return
            self.goal_handle = handle
            record["state"] = "active"
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, record))

    def _on_result(self, future, record):
        status = future.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}.get(status, f"status_{status}")
        with self.lock:
            record["state"] = name
            record["ended"] = time.time()
            if self.goal is record:
                self.goal_handle = None

    def cancel_goal(self, why="cancelled"):
        with self.lock:
            handle, self.goal_handle = self.goal_handle, None
            if self.goal and self.goal["state"] in ("pending", "active"):
                self.goal["cancel_reason"] = why
        if handle is not None:
            handle.cancel_goal_async()
            return True
        return False

    def goal_state(self):
        with self.lock:
            return 200, {"session": self.session, "goal": dict(self.goal) if self.goal else None,
                         "plan": list(self.plan)}

    # ---------- R6: what reached the wheels ----------

    def _reset_stats(self):
        self.stats = {"applied": 0, "reversals": 0, "metres": 0.0,
                      "monitor_stops": 0, "monitor_slows": 0, "since": time.time()}
        self._last_w_sign = 0
        self._last_mux = None
        self._last_odom_xy = None

    def _on_mux(self, msg):
        with self.lock:
            self._last_mux = (msg.linear.x, msg.angular.z, time.time())

    def _on_applied(self, msg):
        v, w = msg.linear.x, msg.angular.z
        with self.lock:
            st = self.stats
            st["applied"] += 1
            if abs(w) > 0.1:
                sign = 1 if w > 0 else -1
                if self._last_w_sign and sign != self._last_w_sign:
                    st["reversals"] += 1
                self._last_w_sign = sign
            mux = self._last_mux
            if mux and time.time() - mux[2] < 0.2:
                asked = math.hypot(mux[0], mux[1])
                given = math.hypot(v, w)
                if asked > 1e-3 and given < 1e-3:
                    st["monitor_stops"] += 1
                elif asked > 1e-3 and given < 0.9 * asked:
                    st["monitor_slows"] += 1

    def _pose_in(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(frame, "base_footprint", Time())
        except Exception:  # noqa: BLE001
            return None
        t = tr.transform.translation
        yaw, _, _ = _yaw_pitch_roll(tr.transform.rotation)
        return {"x_m": t.x, "y_m": t.y, "yaw_rad": yaw}

    def slam_pose(self):
        return 200, {"session": self.session, "start_truth": self.start_truth,
                     "map": self._pose_in("map"),
                     "odom": self._pose_in("odom"), "at": time.time()}

    def _robot_get(self, path):
        return self.robot_http.get_json(path)

    def _record_start_truth(self):
        """First time the robot answers: the robot has not moved yet (the
        controllers came up with this container), so its truth now is its
        truth at odometry zero."""
        self.start_truth_tried = True
        try:
            t = self._robot_get("/world/truth")
        except Exception:  # noqa: BLE001 -- a pre-R2 robot, or hardware
            return
        if t.get("usable"):
            self.start_truth = {k: t[k] for k in ("x_m", "y_m", "heading_deg")}

    def _poll_scan(self):
        try:
            s = self._robot_get("/scan")
            if not self.start_truth_tried:
                self._record_start_truth()
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
        # Stamped with the NEWEST time the transform tree already covers
        # (odom -> base_footprint), so a scan can never arrive ahead of TF.
        # History, both measured in R6 (PLAN-ros-alignment.md 3.15):
        #   * capture stamps (R5, the robot server's `stamp_unix`) are often a
        #     few ms AHEAD of the newest odometry transform, so each costmap's
        #     tf2 MessageFilter held the scan waiting -- and on Humble that
        #     wait path deadlocked the costmaps' TF listeners within seconds.
        #     Their robot pose froze at the start, the planner planned from
        #     there, and the controller declared every goal "reached";
        #   * arrival stamps made it rarer, not impossible: one scaled-house
        #     run in two still froze.
        # The newest odom stamp is at most one 20 Hz period old -- the age a
        # real lidar driver's scans usually carry -- and it is transformable
        # by construction.
        try:
            tr = self.tf_buffer.lookup_transform("odom", "base_footprint", Time())
        except Exception:  # noqa: BLE001 -- no odometry yet: nothing to stamp against
            return
        msg.header.stamp = tr.header.stamp
        msg.header.frame_id = "laser"
        msg.angle_min = math.radians(start)
        msg.angle_increment = math.radians(inc_deg)
        msg.angle_max = msg.angle_min + msg.angle_increment * (n - 1)
        msg.range_min = float(s["range_min_m"])
        msg.range_max = float(s["range_max_m"])
        msg.scan_time = 1.0 / SCAN_HZ
        ranges = convert.ros_ranges(ours, start, inc_deg)
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
        # A person outranks the plan (3.15): M4's preemption, inside ROS.
        if driver == "twin-dpad" and (t.linear.x or t.angular.z):
            self.cancel_goal("preempted by twin-dpad")
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
                         "pan_rad": self.pan_rad, "session": self.session,
                         "map_version": self.map_version,
                         "brain_url": self.brain_url,
                         "brain_polls": self.brain_polls,
                         "brain_age_s": None if not self.brain_at else time.time() - self.brain_at,
                         "brain_error": self.brain_error}


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
            if url.path == "/slam/pose":
                return self._send(*bridge.slam_pose())
            if url.path == "/slam/map":
                with bridge.lock:
                    return self._send(200, bridge.last_map or {"session": bridge.session,
                                                               "version": 0, "usable": False})
            if url.path == "/goal":
                return self._send(*bridge.goal_state())
            if url.path == "/nav/stats":
                with bridge.lock:
                    return self._send(200, dict(bridge.stats))
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
            if path == "/goal":
                return self._send(*bridge.send_goal(body))
            if path == "/nav/stats/reset":
                with bridge.lock:
                    bridge._reset_stats()
                return self._send(200, {"reset": True})
            if path == "/pan":
                with bridge.lock:
                    bridge.pan_rad = float(body.get("angle_rad", 0.0))
                return self._send(200, {"pan_rad": bridge.pan_rad})
            return self._send(404, {"error": "no such route"})

        def do_DELETE(self):  # noqa: N802
            if not self._authorised():
                return None
            if urlparse(self.path).path == "/goal":
                return self._send(200, {"cancelled": bridge.cancel_goal()})
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
