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
    POST /nav/keepout {spots: [{x_m, y_m, r_m, ttl_s}]}   3.68: the whole set
                         of discs nav2 must not plan through (map frame),
                         replacing the last; published as a latched mask for
                         nav2's KeepoutFilter. The robot server decides them
                         (robot/wedge_watch.py); this only draws them and
                         drops each when its ttl_s runs out.
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
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from picar_bridge import brain_view, convert, keepout, mapstore, relocalise, slam_process
from picar_bridge.keepalive import KeepAliveClient

# Driver (robot/interface.py DRIVER_PRIORITY names) -> twist_mux input.
# Priorities live in picar_bringup/config/twist_mux.yaml.
DRIVER_TOPICS = {
    "twin-dpad": "cmd_vel/teleop",
    "brain": "cmd_vel/brain",
    "ros": "cmd_vel/nav",
}
SCAN_HZ = 10.0
# How long the start-truth thread tries (3.36): the controllers' first
# odometry, then one answer from the robot. Generous for a slow board; the
# anchor no longer assumes the robot is at rest, so waiting costs nothing.
START_TRUTH_WINDOW_S = 30.0
PAN_HZ = 20.0
# The brain ticks at ~4 Hz; a status twice a second is enough to follow it
# and cheap enough to leave on.
BRAIN_HZ = 2.0
# How long POST /goal waits for nav2's action server before answering 503
# (3.76 H1-8): well inside world/ros_world.py's 2 s client timeout, or the
# caller gives up first and reads a timeout instead of "nav2 is not up".
NAV2_WAIT_S = 1.0
# A goal nav2 has not accepted or rejected in this long never will be (its
# navigator died after wait_for_server): it ends `aborted` (3.76 H1-2). While
# pending, the robot server refuses every other autonomous driver and a
# stop waits on it, so "pending" must not be forever. nav2 answers in ms.
GOAL_RESPONSE_TIMEOUT_S = 5.0
# The start truth (3.76 H1-4, round 2): the truth is sampled somewhere inside
# the request, so the odometry is carried by its own velocities to the
# request's midpoint, and only a request this quick is used -- the error left
# is at most speed x half of it (1 rad/s: under 3 deg; at rest: none).
START_TRUTH_MAX_RTT_S = 0.1
# Older odometry than this is not extrapolated (the controllers stalled).
START_TRUTH_MAX_ODOM_AGE_S = 0.5
# With no answer within START_TRUTH_MAX_RTT_S this long after the first, the
# quickest seen is used (3.76 Q2: a loaded Jetson at container start may
# never meet the bar); /world/truth is asked at most every START_TRUTH_POLL_S.
START_TRUTH_FALLBACK_S = 3.0
START_TRUTH_POLL_S = 0.2
# The cancel schedule (3.76 round 10): ONE rule for every goal the bridge is
# cancelling -- the live goal's and any superseded one's -- whoever asks (a
# stop, the D-pad, a superseding goal, or the bridge's own timer): a cancel
# is due once the last send to that goal is CANCEL_RESEND_S old during the
# goal's first CANCEL_SLOWER_AFTER_S of cancelling, then CANCEL_SLOWEST_S
# old. 0.2 s is under the stop loop's 0.25 s poll, so every poll re-sends
# while it matters; a hung navigator gets about 360 cancels an hour per goal
# after 30 s (a known limit). The timer's CANCEL_TIMER_S tick is only how
# often it looks.
CANCEL_RESEND_S = 0.2
CANCEL_SLOWER_AFTER_S = 30.0
CANCEL_SLOWEST_S = 10.0
CANCEL_TIMER_S = 0.5
# A cancelled goal with no result this long after its first cancel, while
# nav2's navigator is gone and nothing has driven on cmd_vel/nav for as long,
# ends `aborted` (3.76 R6): otherwise it read `active` for the session, the
# stop held `ros` at zero and every autonomous driver was refused for good.
CANCEL_RESULT_TIMEOUT_S = 10.0
# 3.50.D: where saved maps live -- a Docker volume in the sim, the NVMe on
# the car. A container without it saves nothing, as before 3.50 (every
# other session's stack mounts none).
MAP_DIR = os.environ.get("PICAR_MAP_DIR", "/maps")
# slam_toolbox writes the graph in well under a second for a house
# (3.50.D1: 0.4 s for 7 MB); a save stuck this long is abandoned.
SAVE_TIMEOUT_S = 30.0
# 3.50.D3: with a saved map in the store, slam_toolbox starts localising on
# it from the dock. PICAR_MAP_RELOAD=0 keeps a fresh map every start (a run
# that must map from empty while a store is mounted).
MAP_RELOAD = os.environ.get("PICAR_MAP_RELOAD", "1") != "0"
# A stopped node's last map -> odom is stamped up to slam.yaml's
# transform_timeout (0.2 s) ahead: a fresh node's frame starts after it.
SLAM_TF_LEAD_S = 0.25


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
        # True while the start-truth thread is still working (3.76 round 5):
        # world/ros_world.py anchors nothing for a session until it ends.
        self.start_truth_pending = True
        threading.Thread(target=self._record_start_truth, daemon=True,
                         name="start-truth").start()
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
        # Every goal handle the bridge has asked to cancel and nav2 has not
        # yet answered with a result: id(handle) -> {handle, record, since,
        # last} (3.76 Q3). Re-sent by a timer until the result arrives.
        self.cancelling = {}
        self._nav_ready = True           # refreshed by _resend_cancels, outside the lock
        self.goal = None               # {"x_m","y_m","yaw_rad","state","since"}
        self.plan = []
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel_mux", self._on_mux, 10)
        # What nav2's controller and behaviours command (R6 R6's remap):
        # the last non-zero one, for R6's "nothing is driving" check.
        self.last_nav_twist_at = None
        self.create_subscription(Twist, "cmd_vel/nav", self._on_nav_twist, 10)
        self.create_subscription(Twist, "/diff_drive_controller/cmd_vel_unstamped",
                                 self._on_applied, 10)
        self._reset_stats()
        # ---- 3.68: keepout spots, for nav2's KeepoutFilter ----
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.keepout_spots = []
        self.keepout_info_pub = self.create_publisher(CostmapFilterInfo, "/costmap_filter_info", latched)
        self.keepout_mask_pub = self.create_publisher(OccupancyGrid, "/keepout_mask", latched)
        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 0                    # keepout
        info.filter_mask_topic = "/keepout_mask"
        info.base, info.multiplier = 0.0, 1.0
        self.keepout_info_pub.publish(info)
        self._publish_keepout([])
        self.create_timer(1.0, self._expire_keepout)
        # Its own callback group (PLAN-ros-alignment.md 3.36): the poll is a
        # BLOCKING HTTP read, and in the node's default (mutually exclusive)
        # group it starved every subscription -- on the Jetson a full scan
        # takes about one timer period, the timer is then always due, and
        # rclpy serves due timers before subscriptions: /odom went unread for
        # minutes, nav2's odom TF went stale, every goal stalled. Here the
        # poll holds one executor thread and the rest keep running.
        self.create_timer(1.0 / SCAN_HZ, self._poll_scan,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / PAN_HZ, self._publish_pan)
        self.create_timer(CANCEL_TIMER_S, self._resend_cancels)
        # ---- the brain, for ROS tools ----
        self.brain_url = os.environ.get("BRAIN_URL", "http://host.docker.internal:8001/brain").rstrip("/")
        self.brain_http = KeepAliveClient(self.brain_url, auth, timeout=1.0) if self.brain_url else None
        self.brain_status_pub = self.create_publisher(String, "brain/status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.brain_marker_pub = self.create_publisher(MarkerArray, "brain/markers", 10)
        self.brain_at = None
        self.brain_error = None
        self.brain_polls = 0
        # ---- 3.50.D: the saved map ----
        # One map_id for the map this session builds; saved versions of it
        # carry it (a reload, D3, will adopt the saved map's instead).
        self.map_id = mapstore.new_map_id()
        self.map_store = mapstore.MapStore(MAP_DIR) if os.path.isdir(MAP_DIR) else None
        self.map_saves = {"saved": 0, "failed": 0, "skipped": 0, "last_version": None,
                          "last_at": None, "last_error": None}
        self._saving = False             # one save at a time; under self.lock
        self._save_again = None          # a save asked for during one: its reason
        self._mission_running = None     # the brain's last `running`, for the end edge
        self._mission_elapsed = None     # and its budget.elapsed_s: a new mission restarts it
        self.serialize_cli = self.create_client(
            SerializePoseGraph, "slam_toolbox/serialize_map",
            callback_group=ReentrantCallbackGroup())
        # ---- 3.50.D3: slam_toolbox, started by start_slam() ----
        # `starting` until slam_toolbox runs; `checking` while a reloaded map
        # is judged; `ready` after. Until `ready` no map pose is published and
        # no goal is taken: the frame may yet be swapped for a fresh one.
        self.slam = None
        self.slam_state = "starting"
        self.slam_started_at = None      # wall time of the current node's start
        self.reload = {"result": "none"}
        # The saved start truth of a matched reload: its map frame is session
        # 1's, so /world/error lays it on the house from there.
        self.reload_start_truth = None
        self.reload_matched = False
        if self.map_store is None:
            self.get_logger().warning(f"no {MAP_DIR}: maps are not saved this session")
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
            ended = False
            if status is not None:
                self.brain_at = time.time()
                running = bool(status.get("running"))
                elapsed = (status.get("budget") or {}).get("elapsed_s")
                # A mission ended: `running` fell, or (3.50.D2 review) a new
                # mission began inside one poll -- running throughout, with
                # its elapsed time back near zero. A mission shorter than one
                # poll is never seen at all (a known limit).
                restarted = (running and self._mission_running is True
                             and isinstance(elapsed, (int, float))
                             and isinstance(self._mission_elapsed, (int, float))
                             and elapsed < self._mission_elapsed)
                ended = (self._mission_running is True and not running) or restarted
                self._mission_running = running
                self._mission_elapsed = elapsed
            pan = self.pan_rad
        if ended:
            # 3.50: a map is saved when a mission ends -- no new route (the
            # bridge's cap), and the brain names the moment the map is worth
            # keeping. Off the poll thread: a save must not stall /diagnostics.
            self.save_map_async("mission_end")
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
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            # 3.50.D3: a map the previous slam_toolbox published (a reload
            # that fell back) is not this frame's.
            if self.slam_started_at is not None and stamp < self.slam_started_at:
                return
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
        # A goal converted for another session (the container restarted
        # between the caller's pose read and this post) is refused before
        # anything is cancelled (3.76 round 8). A caller that names none is
        # taken as before.
        if body.get("session") not in (None, self.session):
            return 409, {"error": "the SLAM session changed", "session": self.session}
        with self.lock:
            state, why = self.slam_state, self.reload.get("error")
        if state != "ready":
            # 3.50.D3: nav2 is held while slam_toolbox starts or a reload is
            # judged -- a bad match swaps the whole frame.
            return 503, {"error": f"SLAM is not ready ({state})" + (f": {why}" if why else ""),
                         "slam": state}
        if not self.nav.wait_for_server(timeout_sec=NAV2_WAIT_S):
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
            # The superseded goal was sent its cancel above, and nav2's
            # navigator preempts it for this one anyway; its handle must not
            # stand in for the new goal's.
            self.goal_handle = None
            self.goal = {"x_m": ps.pose.position.x, "y_m": ps.pose.position.y, "yaw_rad": yaw,
                         "state": "pending", "since": time.time(),
                         # internal, monotonic, never published (round 8)
                         "_t0": time.monotonic()}
            record = self.goal
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, record))
        return 200, {"state": "pending"}

    def _on_goal_response(self, future, record):
        handle = future.result()
        send = None
        with self.lock:
            if not handle.accepted:
                if record["state"] == "pending":
                    record["state"] = "rejected"
                return
            if record is not self.goal:
                # Superseded while pending: nav2 preempts it for the new one,
                # and it is cancelled until its result says so (3.76 Q3).
                record.setdefault("cancel_reason", "superseded")
                send = self._ask_cancel(handle, record)
            else:
                if record["state"] != "pending":
                    # Given up on (H1-2) and accepted after all (3.76 R2): it is
                    # live again, reads `active`, and is cancelled until it ends.
                    record["reason"] = "nav2 accepted it after the deadline; cancelled"
                    record.pop("ended", None)
                    record.setdefault("cancel_reason", "accepted after the deadline")
                self.goal_handle = handle
                record["state"] = "active"
                if record.get("cancel_reason"):  # cancelled while pending (spec review 3, V2)
                    # It reads `active` until nav2's result says it ended
                    # (3.76 H1-1): a lost cancel must stay visible.
                    send = self._ask_cancel(handle, record)
        if send is not None:
            send.cancel_goal_async()             # outside bridge.lock (round 8)
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, record, handle))

    def _on_result(self, future, record, handle=None):
        status = future.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}.get(status, f"status_{status}")
        with self.lock:
            record["state"] = name
            record["ended"] = time.time()
            if handle is not None:
                self.cancelling.pop(id(handle), None)
            if self.goal is record:
                self.goal_handle = None

    def _ask_cancel(self, handle, record):
        """Under self.lock. Note that the bridge wants `handle` ended (an
        entry in `cancelling` until nav2's result, 3.76 Q3), and return it if
        a cancel is due now (else None). The CALLER sends, outside the lock
        (round 8): a future with no executor runs its callbacks inline, and
        rclpy's client lock taken under bridge.lock could invert against a
        callback that takes bridge.lock."""
        now = time.monotonic()
        return handle if self._take_due(self._register(handle, record, now), now) else None

    def _register(self, handle, record, now):
        """Under self.lock. The `cancelling` entry for `handle`, made once."""
        entry = self.cancelling.get(id(handle))
        if entry is None:
            entry = self.cancelling[id(handle)] = {
                "handle": handle, "record": record, "since": now, "last": -math.inf}
            record.setdefault("_cancel_sent_at", now)         # internal, monotonic
        return entry

    @staticmethod
    def _take_due(entry, now):
        """Under self.lock. The one schedule (round 10, constant block): is a
        cancel due for `entry`? If so, mark it sent."""
        interval = (CANCEL_RESEND_S if now - entry["since"] < CANCEL_SLOWER_AFTER_S
                    else CANCEL_SLOWEST_S)
        if now - entry["last"] < interval:
            return False
        entry["last"] = now
        return True

    def _due_handles(self, now):
        """Under self.lock. Every cancelling goal due now, dropping any R6
        gives up on."""
        due = []
        for key, entry in list(self.cancelling.items()):
            if self._navigator_silent(entry["since"], now):
                del self.cancelling[key]             # R6: nothing left to stop
                continue
            if self._take_due(entry, now):
                due.append(entry["handle"])
        return due

    def _navigator_silent(self, since, now):
        """Under self.lock. When to give up on a cancel nav2 never answered
        (3.76 R6): the navigator is GONE and nothing has driven on
        cmd_vel/nav for CANCEL_RESULT_TIMEOUT_S since. Never while the
        navigator is up (round 5 reverted F2's 30 s clause): nav2's recovery
        tree can be silent for longer than that -- a Wait, a refused Spin or
        BackUp -- and then resume the very goal the stop was ending, with no
        handle left to cancel it. A hung but advertised navigator therefore
        holds the goal live: a fail-safe lockout until a new goal or a
        restart (the ROS spec's known limits)."""
        return (now - since > CANCEL_RESULT_TIMEOUT_S
                and (self.last_nav_twist_at is None
                     or now - self.last_nav_twist_at > CANCEL_RESULT_TIMEOUT_S)
                and not self._nav_ready)

    def _resend_cancels(self):
        """The bridge's own re-send of every cancel not yet answered by a
        result (3.76 Q3), looking every CANCEL_TIMER_S; sent outside the lock."""
        # rclpy is asked OUTSIDE bridge.lock (round 10's review): the
        # answer is cached for _navigator_silent(), which runs under it on
        # every Stop and D-pad cancel. Starts True -- assume the navigator
        # is up, the fail-safe side.
        # Asked only while something is being cancelled; a query that raises
        # reads as True -- the navigator up, the fail-safe side (no give-up).
        with self.lock:
            busy = bool(self.cancelling) or bool(
                self.goal and self.goal["state"] == "active" and self.goal.get("_cancel_sent_at"))
        if busy:
            try:
                ready = bool(self.nav.server_is_ready())
            except Exception as e:  # noqa: BLE001
                ready = True
                self.get_logger().warn(f"navigator readiness query failed ({e}); "
                                       "read as up, so no cancelled goal is given up",
                                       throttle_duration_sec=30.0)
        else:
            ready = True
        with self.lock:
            self._nav_ready = ready
            due = self._due_handles(time.monotonic())
        for handle in due:
            handle.cancel_goal_async()

    def _expire_pending(self):
        """Under self.lock. Bounded outcomes for a goal nav2 stopped
        answering. Pending with no answer (H1-2): `aborted`; if the answer
        comes after all, _on_goal_response() makes it live and cancels it.
        Active, cancelled, and no result (3.76 R6): `aborted` only once
        nav2's navigator is gone AND nothing has driven on cmd_vel/nav since
        the cancel -- so the stop's hold is lifted only over silent wheels."""
        g, now = self.goal, time.monotonic()
        if not g:
            return
        if g["state"] == "pending" and now - g["_t0"] > GOAL_RESPONSE_TIMEOUT_S:
            g["state"] = "aborted"
            g["ended"] = time.time()
            g["reason"] = f"nav2 did not answer the goal within {GOAL_RESPONSE_TIMEOUT_S:.0f} s"
        elif (g["state"] == "active" and g.get("_cancel_sent_at") is not None
              and self._navigator_silent(g["_cancel_sent_at"], now)):
            g["state"] = "aborted"
            g["ended"] = time.time()
            g["reason"] = ("cancelled, no result from nav2, and nothing has driven "
                           f"on cmd_vel/nav for {CANCEL_RESULT_TIMEOUT_S:.0f} s")
            if self.goal_handle is not None:
                self.cancelling.pop(id(self.goal_handle), None)
            self.goal_handle = None

    def _superseded_live(self):
        """Under self.lock. Goals asked to cancel, other than the current
        one, with no result yet."""
        return sum(1 for e in self.cancelling.values() if e["record"] is not self.goal)

    def cancel_goal(self, why="cancelled"):
        """Ask nav2 to cancel the live goal AND every superseded goal still
        cancelling, each on the one schedule (round 10), until nav2 reports
        it ended (3.76 H1-1):
        the handle is kept until _on_result(), because a cancel can be lost
        or refused. The bridge also re-sends on its own timer (Q3). True
        while anything the bridge asked to end is still live."""
        with self.lock:
            self._expire_pending()
            live = bool(self.goal and self.goal["state"] in ("pending", "active"))
            if live:
                # The first reason stands (R9): a D-pad stream must not
                # overwrite the stop that ended the goal.
                self.goal.setdefault("cancel_reason", why)
            now = time.monotonic()
            if self.goal_handle is not None:
                self._register(self.goal_handle, self.goal, now)
            due = self._due_handles(now)         # the live goal and every superseded one
            superseded = self._superseded_live()
        for handle in due:
            handle.cancel_goal_async()           # outside bridge.lock (round 8)
        # A pending goal has no handle yet; the cancel_reason set above makes
        # _on_goal_response() cancel it the moment nav2 accepts it.
        return live or superseded > 0

    def goal_state(self):
        with self.lock:
            self._expire_pending()
            g = ({k: v for k, v in self.goal.items() if not k.startswith("_")}
                 if self.goal else None)          # internal fields stay internal (round 8)
            superseded = self._superseded_live()
            if g is not None and superseded and g["state"] not in ("pending", "active"):
                # A superseded goal nav2 has not confirmed ended may still be
                # driving (3.76 Q3): read as live, so goal_in_progress()
                # refuses other autonomous drivers and a stop keeps its hold.
                g["state"] = "active"
                g["reason"] = f"{superseded} superseded goal(s) still being cancelled"
            return 200, {"session": self.session, "goal": g, "plan": list(self.plan),
                         "cancelling": len(self.cancelling)}

    # ---------- 3.68: keepout spots ----------

    def set_keepout(self, body):
        # 3.83 review: as POST /goal, spots converted for another session
        # (this bridge restarted since the robot server's pose read) are
        # refused, not drawn into this map.
        if body.get("session") not in (None, self.session):
            return 409, {"error": "the SLAM session changed", "session": self.session}
        try:
            spots = keepout.parse(body.get("spots") or [], time.time())
        except (KeyError, TypeError, ValueError) as e:
            return 400, {"error": str(e)}
        # Published under the lock, as is each expiry: a set and an expiry
        # publishing in the other order would latch a stale mask.
        with self.lock:
            self.keepout_spots = spots
            self._publish_keepout(spots)
        return 200, {"spots": len(spots)}

    def _expire_keepout(self):
        with self.lock:
            kept = keepout.live(self.keepout_spots, time.time())
            if len(kept) != len(self.keepout_spots):
                self.keepout_spots = kept
                self._publish_keepout(kept)

    def _publish_keepout(self, spots):
        m = keepout.mask(spots)
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = m["resolution_m"]
        msg.info.width, msg.info.height = m["width"], m["height"]
        msg.info.origin.position.x = m["origin_x_m"]
        msg.info.origin.position.y = m["origin_y_m"]
        msg.info.origin.orientation.w = 1.0
        msg.data = m["data"]
        self.keepout_mask_pub.publish(msg)

    # ---------- R6: what reached the wheels ----------

    def _reset_stats(self):
        self.stats = {"applied": 0, "reversals": 0, "metres": 0.0,
                      "monitor_stops": 0, "monitor_slows": 0, "since": time.time()}
        self._last_w_sign = 0
        self._last_mux = None
        self._last_odom_xy = None

    def _on_nav_twist(self, msg):
        if msg.linear.x or msg.angular.z:
            with self.lock:
                self.last_nav_twist_at = time.monotonic()

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
        # Time() is the LATEST transform, however old (3.76 H1-13): with
        # slam_toolbox or ros2_control dead the chain freezes and this would
        # answer the last pose for ever. `age_s` lets the consumer tell.
        stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return {"x_m": t.x, "y_m": t.y, "yaw_rad": yaw, "age_s": max(0.0, age)}

    def slam_pose(self):
        with self.lock:                      # one consistent pair (round 5)
            ready = self.slam_state == "ready"
            if self.reload_matched and self.reload_start_truth is not None:
                start, pending = self.reload_start_truth, not ready
            else:
                start, pending = self.start_truth, self.start_truth_pending or not ready
        # 3.50.D3: no map pose, and the start truth pending, until the frame
        # is decided -- world/ros_world.py then anchors nothing and reads the
        # pose and the map unusable, rather than anchor a frame about to go.
        return 200, {"session": self.session, "start_truth": start,
                     "start_truth_pending": pending,
                     "map": self._pose_in("map") if ready else None,
                     "odom": self._pose_in("odom"), "at": time.time()}

    def _robot_get(self, path):
        return self.robot_http.get_json(path)

    def _record_start_truth(self):
        try:
            self._find_start_truth()
        finally:
            with self.lock:
                self.start_truth_pending = False

    def _find_start_truth(self):
        """Where the robot stood in the house at odometry zero (R5), for
        world/ros_world.py to lay SLAM's frame on. Its own thread (3.36).

        Composed from the truth and the odometry read at ONE instant
        (`convert.odometry_zero_in_house`), never assumed from a robot at
        rest: on the Jetson it was read seconds after start -- once after the
        chain suite had begun driving, and after every container restart in
        the fallback test, while a person drove -- and each time it anchored
        SLAM's frame ~20 deg off for the session."""
        deadline = time.time() + START_TRUTH_WINDOW_S
        quickest = None                  # (rtt, truth, odometry at the midpoint), 3.76 Q2
        first_answer = None
        while rclpy.ok() and time.time() < deadline:
            with self.lock:
                have_odom = self.last_odom is not None
            if not have_odom:                        # the controllers are not up yet
                time.sleep(0.05)
                continue
            asked = time.time()
            try:
                t = self._robot_get("/world/truth")
            except Exception:  # noqa: BLE001 -- busy: try again
                time.sleep(START_TRUTH_POLL_S)
                continue
            answered = time.time()
            if not t.get("usable"):
                return                               # answered: hardware, nothing to record
            # The truth was sampled somewhere inside the request (3.76 H1-4,
            # R4): carry the newest odometry by its own velocities to the
            # request's midpoint. The same odometry read before and after
            # proves nothing mid-turn.
            with self.lock:
                odom = dict(self.last_odom)
            mid, rtt = (asked + answered) / 2, answered - asked
            if mid - odom["at"] <= START_TRUTH_MAX_ODOM_AGE_S:
                # The fallback's clock starts at the first USABLE pair (3.76
                # F4), not at the first answer paired with stale odometry.
                first_answer = first_answer or answered
                pair = (rtt, t, convert.odometry_at(odom, mid))
                if rtt <= START_TRUTH_MAX_RTT_S:
                    with self.lock:
                        self.start_truth = convert.odometry_zero_in_house(t, pair[2])
                    return
                if quickest is None or rtt < quickest[0]:
                    quickest = pair
            if quickest is not None and answered - first_answer >= START_TRUTH_FALLBACK_S:
                # A loaded board may never answer within the bar (3.76 Q2):
                # the quickest answer seen, extrapolated like any other.
                with self.lock:
                    self.start_truth = convert.odometry_zero_in_house(quickest[1], quickest[2])
                self.get_logger().warn(
                    f"start truth from a {quickest[0] * 1000:.0f} ms request (none within "
                    f"{START_TRUTH_MAX_RTT_S * 1000:.0f} ms in {START_TRUTH_FALLBACK_S:.0f} s); "
                    "the anchor may be off by up to the robot's speed times half that")
                return
            time.sleep(max(0.0, START_TRUTH_POLL_S - (time.time() - asked)))
        if quickest is not None:
            # The window closed before the fallback's own 3 s (3.76 round 5):
            # the quickest pair seen beats none.
            with self.lock:
                self.start_truth = convert.odometry_zero_in_house(quickest[1], quickest[2])
            self.get_logger().warn(
                f"start truth from a {quickest[0] * 1000:.0f} ms request as the "
                f"{START_TRUTH_WINDOW_S:.0f} s window closed")
            return
        self.get_logger().error("no start truth recorded in the first "
                                f"{START_TRUTH_WINDOW_S:.0f} s; house-frame goals and "
                                "/world/error are unanchored this session")

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
        # Stamped with when the scan was TAKEN (the robot server's
        # `stamp_unix`), so SLAM pairs it with the odometry of that moment.
        # PLAN-ros-alignment.md 3.40: stamped instead with the newest odom
        # transform (below, now only the fallback), a scan was filed on
        # average 21 ms -- up to one 50 ms wheel-loop step -- before it was
        # taken; turning at 1 rad/s that is up to 2.9 deg of heading per scan,
        # and the furnished home's map bent 2.5-6 deg across the open rooms.
        # History (R6, 3.15): capture stamps run a few ms AHEAD of the newest
        # odom transform, so tf2 MessageFilters hold the scan until odometry
        # catches up -- and on Humble that wait path once deadlocked the
        # costmaps' TF listeners. The image builds the upstream fix (tf2
        # 0.25.24), and 3.40's criterion 4 re-checks R6's goals for it.
        try:
            tr = self.tf_buffer.lookup_transform("odom", "base_footprint", Time())
        except Exception:  # noqa: BLE001 -- no odometry yet: nothing to stamp against
            return
        msg.header.stamp = tr.header.stamp
        cap = s.get("stamp_unix")
        if isinstance(cap, (int, float)):
            # Two clocks (the robot server's and this container's): use the
            # capture time only while they agree to within half a second, or
            # every scan would be filed in the future and dropped.
            if abs(cap - self.get_clock().now().nanoseconds * 1e-9) < 0.5:
                msg.header.stamp = Time(nanoseconds=int(cap * 1e9)).to_msg()
            else:
                self.get_logger().warn("scan capture time is >0.5 s off this clock; "
                                       "stamping with the newest odometry instead",
                                       throttle_duration_sec=10.0)
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

    # ---------- 3.50.D3: starting slam_toolbox, and the reload ----------

    def start_slam(self) -> None:
        """Start slam_toolbox: localising on the store's current map from
        the dock if there is one, else mapping from empty. A reload that does
        not fit (relocalise.judge) is replaced by a fresh map. Its own thread
        (main): the check waits on scans. A failure here leaves no SLAM and
        nav2 held, so it is recorded where /health shows it (Thermos)."""
        try:
            self._start_slam()
        except Exception as e:  # noqa: BLE001 -- a state, shown, not a dead thread
            with self.lock:
                self.slam_state = "failed"
                self.reload = {**self.reload, "result": "failed",
                               "error": f"{type(e).__name__}: {e}"[:300]}
            self.get_logger().error(f"slam_toolbox not started: {e}")
            # No half-started node left publishing map -> odom (review round 2).
            try:
                if self.slam is not None:
                    self.slam.stop()
            except Exception:  # noqa: BLE001
                pass

    def _start_slam(self) -> None:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        self.slam = slam_process.SlamProcess(
            os.path.join(get_package_prefix("slam_toolbox"), "lib", "slam_toolbox"),
            os.path.join(get_package_share_directory("picar_bringup"), "config", "slam.yaml"))
        saved = self.map_store.current() if (self.map_store is not None and MAP_RELOAD) else None
        if saved is None:
            self._start_fresh({"result": "none" if MAP_RELOAD else "off"})
            return
        d = saved.meta.get("dock_pose")
        if not (isinstance(d, dict) and all(
                isinstance(d.get(k), (int, float)) and not isinstance(d.get(k), bool)
                and math.isfinite(d[k]) for k in ("x", "y", "theta"))):
            self.get_logger().warning(f"saved map {saved.map_id} v{saved.version}: dock_pose "
                                      f"{d!r} unreadable; localising at the origin")
            d = mapstore.DOCK_POSE
        dock = (float(d["x"]), float(d["y"]), float(d["theta"]))
        t0 = time.monotonic()
        with self.lock:
            self.slam_state = "checking"
            self.slam_started_at = time.time()
            self.last_map = None
            self.map_id = saved.map_id
            self.reload = {"result": "checking", "map_id": saved.map_id, "version": saved.version}
        try:
            self.slam.start("localisation", saved.prefix, dock)
            verdict = self.check_reload(t0 + relocalise.VERDICT_TIMEOUT_S, dock)
        except Exception as e:  # noqa: BLE001 -- review round 3: a reload that cannot
            # be judged is a reload that does not fit: a fresh map, not no SLAM.
            verdict = {**relocalise.judge([], None), "ok": False,
                       "reason": f"the reload raised {type(e).__name__}: {e}"[:300]}
        record = {"map_id": saved.map_id, "version": saved.version,
                  "took_s": round(time.monotonic() - t0, 2), **verdict}
        if not verdict["ok"]:
            self.get_logger().warning(f"saved map {saved.map_id} v{saved.version} does not fit "
                                      f"({verdict['reason']}): starting a fresh map")
            self._start_fresh({**record, "result": "fell_back"})
            return
        with self.lock:
            self.reload = {**record, "result": "matched"}
            self.reload_start_truth = saved.meta.get("start_truth")
            self.reload_matched = True
            self.slam_state = "ready"
        self.get_logger().info(f"reloaded map {saved.map_id} v{saved.version} at the dock "
                               f"({verdict['agreement']:.0%} of returns on the map)")

    def _start_fresh(self, record: dict) -> None:
        with self.lock:
            self.slam_state = "starting"
            self.reload_matched = False
            self.reload_start_truth = None
            # A fallen-back reload's map_id is not this map's: a new one
            # before the swap, so no save can file a fresh graph under it.
            self.map_id = mapstore.new_map_id()
            self.last_map = None
        self.slam.start("mapping")
        with self.lock:
            # Stamped once the old node has EXITED (start() waits for it), plus
            # its transform's lead (Thermos): nothing it published counts.
            self.slam_started_at = time.time() + SLAM_TF_LEAD_S
            self.last_map = None
            self.reload = record
        # Ready once THIS node places the robot: a stopped node's map -> odom
        # lingers in the TF buffer for a moment, in the frame just dropped.
        while rclpy.ok() and self._fresh_pose() is None:
            if not self.slam.alive():
                raise RuntimeError(f"{slam_process.EXECUTABLES['mapping']} exited before "
                                   "its first pose")
            time.sleep(0.1)
        with self.lock:
            self.slam_state = "ready"

    def _fresh_pose(self):
        """The map pose, if the transform under it was published by the
        slam_toolbox started last."""
        here = self._pose_in("map")
        with self.lock:
            since = self.slam_started_at
        if here is None or since is None or here["age_s"] > 1.0 \
                or time.time() - here["age_s"] < since:
            return None
        return here

    def check_reload(self, deadline: float, dock=(0.0, 0.0, 0.0)) -> dict:
        """Judge the reloaded map on SCANS scans at rest: their agreement
        with its /map, and the localised pose's distance from the dock."""
        shares, pose, seen = [], None, None
        occupied = None
        while time.monotonic() < deadline and len(shares) < relocalise.SCANS:
            with self.lock:
                m, scan = self.last_map, self.last_scan
            here = self._fresh_pose()
            laser = self._laser_in_map()
            if m is None or scan is None or here is None or laser is None \
                    or scan["at"] == seen:
                time.sleep(0.05)
                continue
            if occupied is None or occupied[0] is not m:
                occupied = (m, relocalise.occupied_cells(m))
            seen = scan["at"]
            pose = (here["x_m"], here["y_m"], here["yaw_rad"])
            shares.append(relocalise.scan_agreement(scan, laser, m, occupied[1]))
        if len(shares) < relocalise.SCANS:
            return {**relocalise.judge([], None), "ok": False,
                    "reason": f"no localised pose and {relocalise.SCANS} scans within "
                              f"{relocalise.VERDICT_TIMEOUT_S:.0f} s"}
        return relocalise.judge(shares, pose, dock)

    def _laser_in_map(self):
        try:
            tr = self.tf_buffer.lookup_transform("map", "laser", Time())
        except Exception:  # noqa: BLE001
            return None
        t = tr.transform.translation
        yaw, _, _ = _yaw_pitch_roll(tr.transform.rotation)
        return (t.x, t.y, yaw)

    # ---------- 3.50.D: saving the map ----------

    def save_map_async(self, why: str) -> bool:
        """Start a save, or -- with one already running -- queue ONE more
        to follow it (3.50.D2 review): a mission that ends during a save
        has a map worth keeping too."""
        with self.lock:
            if self.map_store is None:
                return False
            if self._saving:
                self._save_again = why
                return True
            self._saving = True
        try:
            threading.Thread(target=self._save_loop, args=(why,), daemon=True,
                             name="map-save").start()
        except Exception:
            with self.lock:
                self._saving = False
            raise
        return True

    def _save_loop(self, why: str) -> None:
        try:
            while True:
                self._save_map(why)
                # One critical section for "anything queued?" and the release:
                # a request between the two would otherwise be lost.
                with self.lock:
                    why, self._save_again = self._save_again, None
                    if why is None:
                        self._saving = False
                        return
        except BaseException:
            with self.lock:
                self._saving = False
            raise

    def _save_map(self, why: str) -> None:
        """slam_toolbox serialises into a staging directory; the store makes
        it the next version only if both files arrived. Any failure leaves
        the previous version current."""
        pending, error, meta = None, None, None
        with self.lock:
            localising = self.slam is not None and self.slam.mode == "localisation"
            state, map_id = self.slam_state, self.map_id
        if localising or state != "ready":
            # 3.50.D3: localisation cannot serialise, and it leaves the saved
            # map as it was; before `ready` (or after a failed start) there is
            # no map of this frame worth keeping -- nothing to save, and not a
            # failure (review round 3: an empty graph must not become current).
            with self.lock:
                self.map_saves["skipped"] += 1
            self.get_logger().info(f"map {map_id} not saved ({why}): "
                                   + ("localising on it, unchanged" if localising else f"SLAM {state}"))
            return
        try:
            pending = self.map_store.begin(map_id)
            if not self.serialize_cli.wait_for_service(timeout_sec=2.0):
                raise mapstore.SaveFailed("slam_toolbox's serialize_map is not up")
            req = SerializePoseGraph.Request()
            req.filename = pending.prefix
            done = threading.Event()
            future = self.serialize_cli.call_async(req)
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(SAVE_TIMEOUT_S):
                future.cancel()
                # slam_toolbox may still be writing there: leave the staging
                # for the next commit's prune rather than delete it under it.
                self.map_store.abandon(pending, remove=False)
                pending = None
                raise mapstore.SaveFailed(f"serialize_map gave no answer in {SAVE_TIMEOUT_S:.0f} s")
            result = future.result()
            if result is None or result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                raise mapstore.SaveFailed(f"serialize_map failed ({getattr(result, 'result', None)})")
            with self.lock:
                start = self.start_truth
            meta = self.map_store.commit(pending, session=self.session, start_truth=start)
        except Exception as e:  # noqa: BLE001 -- a failed save is a state, not a crash
            error = f"{type(e).__name__}: {e}"[:300]
            if pending is not None:
                self.map_store.abandon(pending)
        with self.lock:
            if error is None:
                self.map_saves.update(saved=self.map_saves["saved"] + 1,
                                      last_version=meta["version"], last_at=time.time(),
                                      last_error=None)
            else:
                self.map_saves.update(failed=self.map_saves["failed"] + 1, last_error=error)
        if error is None:
            self.get_logger().info(f"map {map_id} v{meta['version']} saved ({why})")
        else:
            self.get_logger().error(f"map {map_id} not saved ({why}): {error}")

    def health(self):
        with self.lock:
            # 3.50.D3: no slam_toolbox (the start failed) is not ok.
            return 200, {"ok": self.slam_state != "failed", "robot_url": self.robot_url,
                         "odom_age_s": None if not self.last_odom else time.time() - self.last_odom["at"],
                         "scan_age_s": None if not self.last_scan else time.time() - self.last_scan["at"],
                         "scans_published": self.scan_count, "twists": dict(self.twists),
                         "pan_rad": self.pan_rad, "session": self.session,
                         "map_version": self.map_version,
                         "brain_url": self.brain_url,
                         "brain_polls": self.brain_polls,
                         "brain_age_s": None if not self.brain_at else time.time() - self.brain_at,
                         "brain_error": self.brain_error,
                         "map": {"map_id": self.map_id, "saving": self._saving,
                                 "store": self.map_store is not None, **self.map_saves},
                         # 3.50.D3: which slam_toolbox runs, and the reload's verdict
                         "slam": {"state": self.slam_state,
                                  "mode": self.slam.mode if self.slam else None,
                                  "starts": self.slam.starts if self.slam else 0,
                                  "reload": dict(self.reload)}}


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
            # Each of these is copied under the lock and serialised and
            # written OUTSIDE it (3.76 H1-6): every callback, the scan poll
            # and /cmd_vel take that lock, and a slow reader held them all.
            # last_odom/last_scan/last_map are replaced whole, never mutated.
            if url.path == "/odom":
                with bridge.lock:
                    reply = bridge.last_odom or {"usable": False}
                return self._send(200, reply)
            if url.path == "/scan":
                with bridge.lock:
                    reply = bridge.last_scan or {"usable": False}
                return self._send(200, reply)
            if url.path == "/slam/pose":
                return self._send(*bridge.slam_pose())
            if url.path == "/slam/map":
                with bridge.lock:
                    # 3.50.D3: no map until the frame is decided, as /slam/pose
                    reply = (bridge.last_map if bridge.slam_state == "ready" else None) \
                        or {"session": bridge.session, "version": 0, "usable": False}
                return self._send(200, reply)
            if url.path == "/goal":
                return self._send(*bridge.goal_state())
            if url.path == "/nav/stats":
                with bridge.lock:
                    reply = dict(bridge.stats)
                return self._send(200, reply)
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
            if path == "/nav/keepout":
                return self._send(*bridge.set_keepout(body))
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
    # 3.50.D3: slam_toolbox is the bridge's to start (slam_process.py); the
    # reload's check reads scans and TF, so the executor must be spinning.
    threading.Thread(target=bridge.start_slam, daemon=True, name="slam-start").start()
    try:
        executor.spin()
    finally:
        if bridge.slam is not None:
            bridge.slam.stop()
        server.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
