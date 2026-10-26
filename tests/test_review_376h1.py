"""
tests/test_review_376h1.py

PLAN-ros-alignment.md 3.76 batch H1 -- the ROS bridge
(`service/slam/src/picar_bridge/picar_bridge/bridge.py`) and its consumer
`world/ros_world.py`, reviewed whole-file. One test per confirmed finding,
each red on `dev` for the reason its docstring gives.

bridge.py imports rclpy, which is never installed on a laptop. So it is
loaded here against STUB modules placed in `sys.modules` for the length of
one test (and removed again: tests/test_bridge_convert.py asserts rclpy is
not loaded). The stubs carry only what the bridge touches; what is tested is
the bridge's own logic -- its goal bookkeeping, its locking, its anchor
pairing -- not ROS.
"""

import importlib
import sys
import threading
import time
import types
from pathlib import Path

import httpx
import pytest

from tests.test_ros_goals import GoalBridge
from tests.test_ros_world import FakeBridge, FakeTruth
from world.ros_world import RosWorld

BRIDGE_PKG = Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"


# ---------- the stub ROS the bridge is loaded against ----------

class _Msg:
    """Any ROS message: attributes spring into being on first read, so
    `ps.pose.position.x = 1.0` works as it does on a real message."""

    def __init__(self, *args, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = _Msg()
        setattr(self, name, value)
        return value


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_kw):
        self.lines.append(msg)

    info = warn = warning = error = debug = _log


class _Clock:
    def now(self):
        ns = int(time.time() * 1e9)
        return types.SimpleNamespace(nanoseconds=ns,
                                     to_msg=lambda: _Msg(sec=ns // 10**9, nanosec=ns % 10**9))


class _Node:
    def __init__(self, name):
        self._logger = _Logger()

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **kw):
        return None

    def create_timer(self, *a, **kw):
        return None

    def create_client(self, *a, **kw):        # 3.50.D2: slam_toolbox's serialize_map
        return None

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()


class _Time:
    def __init__(self, nanoseconds=0, **_kw):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return _Msg(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


@pytest.fixture
def bridge_mod(monkeypatch):
    """picar_bridge.bridge, imported against stubs; everything removed after."""
    ok = {"value": False}          # rclpy.ok(): False at construction, so no thread loops
    mods = {}

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        mods[name] = m
        return m

    mod("rclpy", ok=lambda: ok["value"], init=lambda: None, shutdown=lambda: None)
    mod("rclpy.action", ActionClient=lambda *a, **kw: None)
    mod("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=_Msg,
        ReentrantCallbackGroup=_Msg)
    mod("rclpy.executors", MultiThreadedExecutor=_Msg)
    mod("rclpy.node", Node=_Node)
    mod("rclpy.qos", DurabilityPolicy=_Msg(), QoSProfile=_Msg, ReliabilityPolicy=_Msg())
    mod("rclpy.time", Time=_Time)
    mod("action_msgs")
    mod("action_msgs.msg", GoalStatus=types.SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
    mod("diagnostic_msgs")
    mod("diagnostic_msgs.msg", DiagnosticArray=_Msg, DiagnosticStatus=_Msg, KeyValue=_Msg)
    mod("geometry_msgs")
    mod("geometry_msgs.msg", Point=_Msg, PoseStamped=_Msg, Twist=_Msg)
    mod("nav2_msgs")
    mod("nav2_msgs.action", NavigateToPose=types.SimpleNamespace(Goal=_Msg))
    mod("nav2_msgs.msg", CostmapFilterInfo=_Msg)          # 3.83: the keepout filter
    mod("nav_msgs")
    mod("nav_msgs.msg", OccupancyGrid=_Msg, Odometry=_Msg, Path=_Msg)
    mod("slam_toolbox")
    mod("slam_toolbox.srv", SerializePoseGraph=_Msg)        # 3.50.D2: the map save
    mod("sensor_msgs")
    mod("sensor_msgs.msg", JointState=_Msg, LaserScan=_Msg)
    mod("std_msgs")
    mod("std_msgs.msg", String=_Msg)
    mod("std_srvs")
    mod("std_srvs.srv", Trigger=types.SimpleNamespace(Request=_Msg))   # 3.105: nav2's is_active
    mod("tf2_ros", Buffer=_Msg, TransformListener=lambda *a, **kw: None)
    mod("visualization_msgs")
    mod("visualization_msgs.msg", Marker=_Msg, MarkerArray=_Msg)
    for name, m in mods.items():
        monkeypatch.setitem(sys.modules, name, m)
    monkeypatch.syspath_prepend(str(BRIDGE_PKG))
    monkeypatch.setenv("BRAIN_URL", "")
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)   # the routes under test, not a 401
    for name in ("picar_bridge.bridge",):
        monkeypatch.delitem(sys.modules, name, raising=False)
    b = importlib.import_module("picar_bridge.bridge")
    b._test_ok = ok
    yield b
    sys.modules.pop("picar_bridge.bridge", None)


# ---------- a fake nav2 action client ----------

class _Future:
    def __init__(self):
        self._cbs, self._value, self._done = [], None, False

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self._done:
            cb(self)

    def result(self):
        return self._value

    def done(self):
        return self._done

    def complete(self, value):
        self._value, self._done = value, True
        for cb in list(self._cbs):
            cb(self)


class _Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancels = 0
        self.cancel_futures = []
        self.result_future = _Future()

    def cancel_goal_async(self):
        self.cancels += 1
        f = _Future()                          # nav2's answer: never comes, or lost
        self.cancel_futures.append(f)
        return f

    def get_result_async(self):
        return self.result_future


class _Nav:
    def __init__(self):
        self.futures = []
        self.waited = []
        self.ready = True

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec):
        self.waited.append(timeout_sec)
        return True

    def send_goal_async(self, goal):
        f = _Future()
        self.futures.append(f)
        return f


def _bridge(bridge_mod):
    b = bridge_mod.Bridge()
    b.nav = _Nav()
    b.slam_state = "ready"             # 3.50.D3: slam_toolbox up, no reload being judged
    with b.lock:
        b._set_nav2(True, None)          # 3.105: nav2's nodes active
    return b


def _later(b, seconds=6.0):
    """As if `seconds` had passed since each handle's last cancel (R9's limit)."""
    for entry in getattr(b, "cancelling", {}).values():
        entry["last"] -= seconds
    if b.goal and "cancel_last_at" in b.goal:          # round-2 code
        b.goal["cancel_last_at"] -= seconds


def _accepted_goal(b):
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    handle = _Handle()
    b.nav.futures[-1].complete(handle)
    assert b.goal_state()[1]["goal"]["state"] == "active"
    return handle


# ---------- H1-1: a cancel that nav2 never acknowledges ----------

def test_h1_1_a_cancel_is_resent_until_nav2_ends_the_goal(bridge_mod):
    """The robot server's stop loop re-sends DELETE /goal every 0.25 s until
    the goal is neither pending nor active (spec review 3, V2) -- because a
    cancel can be lost or refused. The bridge dropped its handle on the FIRST
    cancel, so every re-send found no handle and answered `cancelled: false`
    while nav2 kept driving. Red on dev: 1 cancel reached nav2, and the
    second DELETE answered False."""
    b = _bridge(bridge_mod)
    handle = _accepted_goal(b)
    assert b.cancel_goal() is True
    _later(b)
    assert b.cancel_goal() is True, "the re-sent cancel must reach nav2 again"
    assert handle.cancels == 2
    handle.result_future.complete(types.SimpleNamespace(status=5))   # CANCELED
    assert b.goal_state()[1]["goal"]["state"] == "canceled"
    assert b.cancel_goal() is False and handle.cancels == 2, "nothing left to cancel"


def test_h1_1_a_new_goal_does_not_keep_the_old_handle(bridge_mod):
    """Kept handles must belong to the live goal: once a newer goal is sent,
    a cancel must not be aimed at the superseded one's handle alone."""
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})          # supersedes; old is cancelled once
    assert old.cancels == 1
    new = _Handle()
    b.nav.futures[-1].complete(new)
    b.cancel_goal()
    assert new.cancels == 1 and old.cancels == 1
    old.result_future.complete(types.SimpleNamespace(status=5))   # the old one's late result
    assert b.goal_state()[1]["goal"]["state"] == "active", "the old result must not end the new goal"
    _later(b)
    assert b.cancel_goal() is True and new.cancels == 2


def test_h1_1_a_goal_cancelled_while_pending_reads_ended_only_when_nav2_says_so(bridge_mod):
    """The same defect for a goal cancelled before nav2 accepted it: on
    acceptance it was sent one cancel and marked `canceled` at once, so a
    lost cancel left nav2 driving while every reader saw it ended. Red on
    dev: `canceled` before any result, and the re-sent cancel answered
    False with 1 cancel sent."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert b.cancel_goal() is True                     # still pending
    handle = _Handle()
    b.nav.futures[-1].complete(handle)                 # nav2 accepts it now
    assert handle.cancels == 1
    assert b.goal_state()[1]["goal"]["state"] == "active", "ended before nav2 said so"
    _later(b)
    assert b.cancel_goal() is True and handle.cancels == 2
    handle.result_future.complete(types.SimpleNamespace(status=5))
    assert b.goal_state()[1]["goal"]["state"] == "canceled"


# ---------- H1-2: a goal nav2 never answers ----------

def test_h1_2_a_goal_nav2_never_answers_does_not_stay_pending(bridge_mod):
    """`goal_in_progress()` in robot/server.py refuses every other
    autonomous driver while the goal reads pending, and the stop loop waits
    for it to leave pending. If nav2 never answers send_goal_async (its
    navigator died after wait_for_server), both waited forever. Red on dev:
    still `pending` long after the deadline."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    b.goal["_t0"] -= bridge_mod.GOAL_RESPONSE_TIMEOUT_S + 1.0
    g = b.goal_state()[1]["goal"]
    assert g["state"] not in ("pending", "active"), g
    assert b.cancel_goal() is False


def test_r2_a_goal_accepted_after_the_deadline_is_live_and_cancelled_until_it_ends(bridge_mod):
    """3.76 R2: nav2 accepting after the 5 s deadline got one fire-and-forget
    cancel and the handle was dropped; the goal still read `aborted`. A lost
    cancel then left nav2 driving a goal nothing tracked -- no lockout, no
    stop hold, no handle for DELETE. Red before the fix: still `aborted`,
    and the re-sent DELETE answered False."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    b.goal["_t0"] -= bridge_mod.GOAL_RESPONSE_TIMEOUT_S + 1.0
    assert b.goal_state()[1]["goal"]["state"] == "aborted"
    late = _Handle()
    b.nav.futures[-1].complete(late)
    assert late.cancels == 1
    assert b.goal_state()[1]["goal"]["state"] == "active"
    _later(b)
    assert b.cancel_goal() is True and late.cancels == 2
    late.result_future.complete(types.SimpleNamespace(status=5))
    assert b.goal_state()[1]["goal"]["state"] == "canceled"


def test_r6_a_cancelled_goal_whose_navigator_died_ends_once_nothing_drives(bridge_mod):
    """3.76 R6: a cancelled goal with no result read `active` for the whole
    session if bt_navigator died: the stop held `ros` at zero and every
    autonomous driver was refused for good. It now ends `aborted` -- but
    only with the navigator gone AND no twist on cmd_vel/nav, so the hold is
    lifted only over silent wheels. Red before the fix: `active` forever."""
    b = _bridge(bridge_mod)
    handle = _accepted_goal(b)
    b.cancel_goal()
    late = bridge_mod.CANCEL_RESULT_TIMEOUT_S + 1.0
    b.goal["_cancel_sent_at"] -= late
    assert b.goal_state()[1]["goal"]["state"] == "active", "the navigator is still up"
    b.nav.ready = False
    b._resend_cancels()             # the timer reads readiness outside the lock (round 10 review)
    b.last_nav_twist_at = time.monotonic()            # something still drives
    assert b.goal_state()[1]["goal"]["state"] == "active"
    b.last_nav_twist_at = time.monotonic() - late
    g = b.goal_state()[1]["goal"]
    assert g["state"] == "aborted" and "nothing has driven" in g["reason"]
    assert b.cancel_goal() is False


def test_round5_a_live_navigator_keeps_the_hold_however_long_it_is_silent(bridge_mod):
    """3.76 round 5 (both reviewers, safety): F2 gave up an unanswered
    cancel after 30 s of silence with nav2 still up, lifting the stop's
    hold -- but nav2's recovery tree can be silent that long (a Wait, a
    refused Spin or BackUp) and then resume the goal the stop was ending,
    with no handle left to cancel. Only a GONE navigator is given up. Red on
    4c8ca07: the entry was given up after 31 s with the navigator alive."""
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})                # old superseded, its cancel unanswered
    new = _Handle()
    b.nav.futures[-1].complete(new)
    new.result_future.complete(types.SimpleNamespace(status=4))
    for entry in b.cancelling.values():
        entry["since"] -= 300.0                          # five minutes unanswered
    b.last_nav_twist_at = time.monotonic() - 300.0       # and silent all that time
    b._resend_cancels()
    reply = b.goal_state()[1]
    assert reply["cancelling"] == 1 and reply["goal"]["state"] == "active", \
        "gave up a cancel while nav2's navigator was still up"
    b.nav.ready = False                                  # the navigator is gone: now it may go
    b._resend_cancels()
    assert b.goal_state()[1]["cancelling"] == 0


def test_r9_a_dpad_stream_sends_one_cancel_and_keeps_the_first_reason(bridge_mod):
    """3.76 R9: every non-zero D-pad twist cancels the goal; with the handle
    kept, each sent nav2 a cancel (10-20 a second) and overwrote the reason.
    Red before the fix: 10 cancels, reason `preempted by twin-dpad`."""
    b = _bridge(bridge_mod)
    handle = _accepted_goal(b)
    b.cancel_goal("stopped")
    for _ in range(9):
        assert b.cancel_goal("preempted by twin-dpad") is True
    assert handle.cancels == 1
    assert b.goal["cancel_reason"] == "stopped"
    _later(b)
    b.cancel_goal("preempted by twin-dpad")
    assert handle.cancels == 2


def test_h1_2_a_goal_answered_in_time_is_untouched(bridge_mod):
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert b.goal_state()[1]["goal"]["state"] == "pending"


def test_q3_a_superseded_goal_is_cancelled_until_its_result_and_reads_live(bridge_mod):
    """3.76 Q3: a superseded goal got one cancel. If it was lost and the new
    goal ended, /goal read the new goal's end -- no lockout, the stop loop
    settled -- and nothing re-sent while nav2 drove the old goal. Now the
    bridge keeps every handle it asked to cancel and re-sends on a timer,
    and /goal reads live while one is unanswered. Red on round 2: read
    `succeeded` with the old goal unconfirmed."""
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})                # old: one cancel, lost
    new = _Handle()
    b.nav.futures[-1].complete(new)
    new.result_future.complete(types.SimpleNamespace(status=4))   # the new goal succeeds
    reply = b.goal_state()[1]
    assert reply["goal"]["state"] == "active", "an unconfirmed superseded goal read as ended"
    assert reply["cancelling"] == 1
    assert b.cancel_goal() is True
    _later(b)
    b._resend_cancels()
    assert old.cancels == 2, "the bridge did not re-send the lost cancel itself"
    old.result_future.complete(types.SimpleNamespace(status=5))
    reply = b.goal_state()[1]
    assert reply["goal"]["state"] == "succeeded" and reply["cancelling"] == 0
    b._resend_cancels()
    assert old.cancels == 2


def test_q3_a_goal_superseded_while_pending_is_cancelled_until_its_result(bridge_mod):
    """...and the same for one superseded before nav2 accepted it, whose
    late accept got one fire-and-forget cancel. Red on round 2: 1 cancel,
    and no result was ever awaited."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    first = b.nav.futures[-1]
    b.send_goal({"x_m": 2.0, "y_m": 0.0})
    late = _Handle()
    first.complete(late)                                 # accepted after it was superseded
    assert late.cancels == 1
    _later(b)
    b._resend_cancels()
    assert late.cancels == 2
    late.result_future.complete(types.SimpleNamespace(status=5))
    assert b.goal_state()[1]["cancelling"] == 0


# ---------- H1-8: the 503 must arrive before the caller gives up ----------

def test_h1_8_nav2_down_is_answered_before_the_callers_timeout(bridge_mod):
    """RosWorld's client gives up after 2.0 s; the bridge waited 2.0 s for
    nav2 before answering 503 'nav2 is not up', so the caller saw a timeout
    (`ros_unavailable: ... timed out`) instead. Red on dev: 2.0 >= 2.0."""
    import inspect
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    client_timeout = inspect.signature(RosWorld.__init__).parameters["timeout_s"].default
    assert b.nav.waited and max(b.nav.waited) <= client_timeout / 2


# ---------- H1-4: the start truth paired with odometry of the same instant ----------

def _truth_for(start_heading, yaw_deg):
    """A pure rotation in place at the house's (2, 1): the heading the truth
    reports after odometry turned `yaw_deg` counter-clockwise."""
    return {"usable": True, "x_m": 2.0, "y_m": 1.0,
            "heading_deg": (start_heading - yaw_deg) % 360.0}


def _record(bridge_mod, b, robot_get):
    b._robot_get = robot_get
    bridge_mod._test_ok["value"] = True
    try:
        t0 = time.time()
        b._record_start_truth()
        return time.time() - t0
    finally:
        bridge_mod._test_ok["value"] = False


def _heading_error(st):
    return (st["heading_deg"] - 90.0 + 180.0) % 360.0 - 180.0


def test_h1_4_a_turn_during_the_truth_request_does_not_skew_the_anchor(bridge_mod):
    """The truth is sampled while the request is in flight; the odometry was
    read after the reply. A robot turning during that read baked the turn
    into the session's anchor. Red on dev: 20 deg off (110 instead of 90)."""
    import math
    b = _bridge(bridge_mod)
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "linear_m_s": 0.0,
                   "angular_rad_s": 0.0, "at": time.time()}
    calls = []

    def robot_get(path):
        truth = _truth_for(90.0, math.degrees(b.last_odom["yaw_rad"]))
        if not calls:
            time.sleep(0.15)                 # a slow request ...
            b.last_odom = {**b.last_odom, "yaw_rad": math.radians(20.0), "at": time.time()}
        calls.append(path)                   # ... during which it turned 20 deg
        return truth

    _record(bridge_mod, b, robot_get)
    st = b.start_truth
    assert st is not None and abs(_heading_error(st)) < 0.5, st
    assert st["x_m"] == pytest.approx(2.0) and st["y_m"] == pytest.approx(1.0)


def test_r4_r5_mid_turn_with_no_new_odometry_the_anchor_still_holds(bridge_mod):
    """3.76 R4: the round-1 fix compared odometry before and after the
    request; with no new odometry message in between (20 Hz, a 10 ms request)
    both were the SAME message and read as no motion mid-turn. R5: and while
    the robot kept moving nothing was recorded for the whole 30 s window.
    The odometry is now carried by its own velocities to the request's
    midpoint. Red before the fix: ~4.6 deg off (an 80 ms-old message at
    1 rad/s) -- or, with the bracket, no anchor until the window closed."""
    import math
    b = _bridge(bridge_mod)
    w = 1.0                                              # turning at 1 rad/s, CCW
    msg_at = time.time() - 0.08                          # the newest message, 80 ms old
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "linear_m_s": 0.0,
                   "angular_rad_s": w, "at": msg_at}

    def robot_get(path):
        yaw_now = w * (time.time() - msg_at)            # the truth, sampled now
        return _truth_for(90.0, math.degrees(yaw_now))

    took = _record(bridge_mod, b, robot_get)
    st = b.start_truth
    assert st is not None and abs(_heading_error(st)) < 0.5, st
    assert took < 1.0, f"a moving robot waited {took:.1f} s for its anchor"


def test_q2_a_slow_board_still_gets_a_start_truth_and_is_not_polled_hard(bridge_mod, monkeypatch):
    """3.76 Q2: with every answer slower than START_TRUTH_MAX_RTT_S (a loaded
    Jetson at container start) no start truth was ever recorded, and
    /world/truth was asked ~20 times a second for the whole window. Now the
    quickest answer is used after START_TRUTH_FALLBACK_S, polling at most
    every START_TRUTH_POLL_S. Red on round 2: none recorded in a 5 s window."""
    monkeypatch.setattr(bridge_mod, "START_TRUTH_WINDOW_S", 5.0)
    b = _bridge(bridge_mod)
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "linear_m_s": 0.0,
                   "angular_rad_s": 0.0, "at": time.time() + 60}       # fresh throughout
    calls = []

    def slow(path):
        calls.append(time.time())
        time.sleep(0.12)
        return _truth_for(90.0, 0.0)

    took = _record(bridge_mod, b, slow)
    assert b.start_truth is not None, "no start truth from a board slower than the bar"
    assert abs(_heading_error(b.start_truth)) < 0.01
    assert took < 4.5
    rate = (len(calls) - 1) / (calls[-1] - calls[0])
    assert rate <= 1.0 / bridge_mod.START_TRUTH_POLL_S + 0.5, f"{rate:.1f} requests/s"


def test_f4_the_fallback_clock_starts_at_the_first_usable_pair(bridge_mod, monkeypatch):
    """3.76 F4: the 3 s fallback started at the first answer of any kind,
    even one paired with odometry too old to use, so the first usable
    (slow) pair after 3 s was taken at once. Red on 52c00bf: the 300 ms
    pair was used, not the quickest (~120 ms)."""
    monkeypatch.setattr(bridge_mod, "START_TRUTH_WINDOW_S", 12.0)
    b = _bridge(bridge_mod)
    t0 = time.time()
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "linear_m_s": 0.0,
                   "angular_rad_s": 0.0, "at": t0 - 10.0}             # stale: unusable
    state = {"fresh": False}

    def answers(path):
        if not state["fresh"] and time.time() - t0 > 3.2:
            state["fresh"] = True
            b.last_odom = {**b.last_odom, "at": time.time() + 60}     # the controllers wake
            time.sleep(0.3)                                           # the first usable: slow
        else:
            time.sleep(0.12)
        return _truth_for(90.0, 0.0)

    _record(bridge_mod, b, answers)
    assert b.start_truth is not None
    warned = [line for line in b.get_logger().lines if "start truth from a" in line]
    assert warned, b.get_logger().lines
    rtt_ms = int(warned[-1].split("start truth from a ")[1].split(" ms")[0])
    assert rtt_ms < 200, f"took the first usable pair ({rtt_ms} ms), not the quickest"


# ---------- H1-6: no network write under the bridge's lock ----------

def _handler(bridge_mod, b, path):
    Handler = bridge_mod._handler(b)
    h = Handler.__new__(Handler)
    h.path, h.headers, h.requestline = path, {}, f"GET {path} HTTP/1.1"
    h.request_version, h.command = "HTTP/1.1", "GET"
    return h


@pytest.mark.parametrize("path", ["/slam/map", "/odom", "/scan", "/nav/stats"])
def test_h1_6_a_slow_reader_does_not_hold_the_bridge_lock(bridge_mod, path):
    """The reply was serialised and WRITTEN to the socket inside
    `bridge.lock` -- the lock every subscription callback, the scan poll and
    /cmd_vel take. A client that reads slowly (or a Docker proxy that stalls)
    held all of them. Red on dev: the lock was not free while the reply was
    being written."""
    b = _bridge(bridge_mod)
    b.last_map = {"session": b.session, "version": 1, "data": [0] * 100}
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "at": time.time()}
    b.last_scan = {"ranges_m": [1.0], "at": time.time()}
    writing, release = threading.Event(), threading.Event()

    class SlowSocket:
        def write(self, data):
            writing.set()
            release.wait(5.0)

        def flush(self):
            pass

    h = _handler(bridge_mod, b, path)
    h.wfile = SlowSocket()
    t = threading.Thread(target=h.do_GET, daemon=True)
    t.start()
    try:
        assert writing.wait(2.0)
        got = b.lock.acquire(timeout=0.5)
        if got:
            b.lock.release()
        assert got, f"GET {path} held bridge.lock while writing to the client"
    finally:
        release.set()
        t.join(2.0)


# ---------- H1-13: a frozen transform is not a live pose ----------

def test_h1_13_the_bridge_reports_how_old_each_pose_is(bridge_mod):
    """lookup_transform(Time()) returns the LATEST transform, however old:
    with slam_toolbox or ros2_control dead the chain freezes and /slam/pose
    kept answering it, stamped with the time of the request. Red on dev: no
    age on the pose (KeyError)."""
    b = _bridge(bridge_mod)
    old = time.time() - 10.0
    tr = _Msg()
    tr.header.stamp = _Msg(sec=int(old), nanosec=int((old % 1) * 1e9))
    tr.transform.translation = _Msg(x=1.0, y=0.0, z=0.0)
    tr.transform.rotation = _Msg(x=0.0, y=0.0, z=0.0, w=1.0)
    b.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **kw: tr)
    reply = b.slam_pose()[1]
    assert reply["map"]["age_s"] == pytest.approx(10.0, abs=0.5)
    assert reply["odom"]["age_s"] == pytest.approx(10.0, abs=0.5)


def test_h1_13_ros_world_reads_a_stale_pose_as_unusable():
    """Red on dev: the 10 s old pose read usable."""
    bridge = FakeBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    world = RosWorld("http://bridge", truth=FakeTruth(1.0, 1.0, 90.0), client=client)
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "age_s": 0.05}
    assert world.get_pose()["usable"] is True
    bridge.map_pose = {**bridge.map_pose, "age_s": 10.0}
    assert world.get_pose()["usable"] is False
    bridge.odom_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "age_s": 10.0}
    assert world.get_odom_pose()["usable"] is False
    # A bridge from before the age was reported still reads as it did.
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    assert world.get_pose()["usable"] is True


# ---------- H1-3: re-anchor when the start truth arrives ----------

def _world(bridge, truth):
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client)


# ---------- H1-5: one consistent anchor per reader ----------

def test_h1_5_a_concurrent_reader_never_gets_a_half_set_anchor():
    """`_ensure_session` published the new session BEFORE it set the anchor;
    in the first-contact path the truth read between them is an HTTP call
    (sim/body_client.py). A second request on FastAPI's threadpool matched
    the session and converted with the old anchor. Red on dev: the second
    reader got (0.5, 0) -- SLAM's own frame -- labelled usable."""
    entered, release = threading.Event(), threading.Event()

    class SlowTruth(FakeTruth):
        def get_truth(self):
            entered.set()
            release.wait(5.0)
            return super().get_truth()

    bridge = FakeBridge(start_truth=None)
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, SlowTruth(3.0, 2.0, 90.0))
    results = {}
    a = threading.Thread(target=lambda: results.update(a=world.get_pose()), daemon=True)
    a.start()
    assert entered.wait(2.0)
    b = threading.Thread(target=lambda: results.update(b=world.get_pose()), daemon=True)
    b.start()
    b.join(0.3)
    release.set()
    a.join(2.0)
    b.join(2.0)
    p = results["a"]
    assert p["usable"] and (p["x_m"], p["y_m"]) == pytest.approx((3.0, 2.0)), p
    # B, racing A's truth read: unusable for now (round 7 throttles the
    # truth retry), or anchored -- never SLAM's own frame marked usable.
    p = results["b"]
    assert not p["usable"] or (p["x_m"], p["y_m"]) == pytest.approx((3.0, 2.0)), p


# ---------- H1-12: goals across a container restart ----------

def _goal_world():
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = GoalBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world = _world(bridge, truth)
    assert world.get_pose()["usable"]
    return world, bridge


def test_h1_12_a_goal_after_a_restart_uses_the_new_sessions_anchor():
    """set_goal() checked the session only when it had none. After the
    container restarted (a new map, begun wherever the robot then stood), a
    goal was converted with the OLD session's anchor and sent into the new
    map. Red on dev: nav2 was sent x 3.0 for a goal 1 m ahead (x 1.0)."""
    world, bridge = _goal_world()
    bridge.session = "s2"
    bridge.start_truth = {"x_m": 3.0, "y_m": 1.0, "heading_deg": 90.0}
    assert world.set_goal(4.0, 1.0)["accepted"]
    assert bridge.goal["x_m"] == pytest.approx(1.0) and bridge.goal["y_m"] == pytest.approx(0.0)
    assert world.get_goal()["goal"]["state"] == "active"


def test_h1_12_a_live_goal_in_a_newer_session_is_not_hidden():
    """get_goal() answered `goal: None` for any session but the one last
    anchored, so goal_in_progress(), the stop loop and the brain's
    cancel_own() missed a live goal until some other call re-anchored.
    Red on dev: None."""
    world, bridge = _goal_world()
    bridge.session = "s2"
    bridge.start_truth = {"x_m": 3.0, "y_m": 1.0, "heading_deg": 90.0}
    bridge.goal = {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0, "state": "active"}
    bridge.goal_session = "s2"
    g = world.get_goal()["goal"]
    assert g is not None and g["state"] == "active"
    assert (g["x_m"], g["y_m"]) == pytest.approx((4.0, 1.0))


def test_h1_12_a_goal_is_refused_while_the_new_session_has_no_map():
    world, bridge = _goal_world()
    bridge.session = "s2"
    bridge.map_pose = None
    reply = world.set_goal(4.0, 1.0)
    assert reply["accepted"] is False and bridge.goal is None


# ---------- round 2: the anchor under live goals and late replies ----------

def test_r7_a_live_goal_that_cannot_be_anchored_still_reports_its_state():
    """3.76 R7: a live goal in a new session whose pose could not be read
    (no map yet, or a 5xx) read `goal: None` -- goal_in_progress() let
    another driver in and the stop loop lifted the `ros` hold. Red before
    the fix: None."""
    world, bridge = _goal_world()
    bridge.session = "s2"
    bridge.map_pose = None                              # the new session has no map yet
    bridge.goal = {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0, "state": "active"}
    bridge.goal_session = "s2"
    g = world.get_goal()["goal"]
    assert g is not None and g["state"] == "active" and g.get("unanchored") is True
    assert "x_m" not in g and "y_m" not in g, "no house position it does not have"


def test_r7_consumers_cope_with_an_unanchored_goal():
    """Every reader of get_goal(): the robot server's goal_in_progress()
    reads the state; the brain's cancel_own() reads x_m with a default."""
    import math as _m
    g = {"state": "active", "unanchored": True}
    assert g.get("state") in ("pending", "active")
    assert _m.hypot(g.get("x_m", _m.inf) - 1.0, g.get("y_m", _m.inf) - 1.0) == _m.inf


def test_r8_a_late_reply_from_an_older_session_does_not_re_install_it():
    """3.76 R8: two readers straddling a restart -- one installs the new
    session, the other's late reply from the old one re-installed the old
    anchor. Red before the fix: map_id flapped back to slam-s1."""
    world, _ = _goal_world()
    newer = {"session": "s2", "map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
             "start_truth": {"x_m": 3.0, "y_m": 1.0, "heading_deg": 90.0}, "at": 2000.0}
    older = {"session": "s1", "map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
             "start_truth": {"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}, "at": 1500.0}
    assert world._ensure_session(newer)[0] == "s2"
    assert world._ensure_session(older) is None
    assert world.map_id == "slam-s2"


# ---------- Q4: explore and a goal it cannot place ----------

# ---------- round 4 ----------

# ---------- round 5: one anchor per session ----------

class _PendingBridge(FakeBridge):
    """FakeBridge that says whether its start-truth thread is still working."""
    pending = True

    goal_reply = None                  # (status, text) for POST /goal, if set

    def handler(self, request):
        if request.url.path == "/goal" and request.method == "POST" and self.goal_reply:
            return httpx.Response(self.goal_reply[0], text=self.goal_reply[1])
        r = super().handler(request)
        if request.url.path == "/slam/pose":
            body = r.json()
            body["start_truth_pending"] = self.pending
            return httpx.Response(200, json=body)
        return r


def test_round5_nothing_is_anchored_while_the_start_truth_is_coming():
    """3.76 round 5: a session asked before its start truth was recorded
    was anchored at first contact and moved later -- every move raced
    something. Now the pose, the map and goals wait. Red on 4c8ca07: a
    usable first-contact pose."""
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = _PendingBridge(start_truth=None)
    world = _world(bridge, truth)
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    truth.pose.update(x_m=1.52)
    assert world.get_pose()["usable"] is False
    assert world.get_map()["usable"] is False
    assert world.anchored_at is None and world.map_id is None
    bridge.start_truth = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}
    bridge.pending = False
    pose = world.get_pose()
    assert world.anchored_at == "session_start" and pose["x_m"] == pytest.approx(1.5)


def test_round5_a_goal_is_refused_while_the_start_truth_is_coming():
    from tests.test_ros_goals import GoalBridge as _GB

    class B(_GB):
        def handler(self, request):
            r = super().handler(request)
            if request.url.path == "/slam/pose":
                return httpx.Response(200, json={**r.json(), "start_truth_pending": True})
            return r
    bridge = B(start_truth=None)
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    reply = world.set_goal(2.0, 1.0)
    assert reply["accepted"] is False and bridge.goal is None


def test_round5_an_installed_anchor_never_moves():
    """No start truth and none coming (a bridge that gave up): first
    contact, once. A start truth appearing later does not move it. Red on
    4c8ca07: moved to session_start."""
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, truth)
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    truth.pose.update(x_m=2.0)
    first = world.get_pose()
    assert world.anchored_at == "first_contact"
    v0 = world.get_map()["map_version"]
    bridge.start_truth = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}
    again = world.get_pose()
    assert world.anchored_at == "first_contact"
    assert (again["x_m"], again["y_m"]) == pytest.approx((first["x_m"], first["y_m"]))
    assert world.get_map()["map_version"] == v0


def test_round5_the_bridge_says_when_its_start_truth_thread_is_done(bridge_mod):
    """Red on 4c8ca07: no `start_truth_pending` on /slam/pose."""
    b = _bridge(bridge_mod)
    b.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **kw: (_ for _ in ()).throw(
        RuntimeError("no tf")))
    b.start_truth_pending = True
    assert b.slam_pose()[1]["start_truth_pending"] is True
    b._record_start_truth()                            # rclpy.ok() is False: gives up at once
    assert b.slam_pose()[1]["start_truth_pending"] is False


def test_round5_a_window_closing_before_the_fallback_still_records_the_quickest(bridge_mod, monkeypatch):
    """CR: a 30 s window that closed before START_TRUTH_FALLBACK_S had run
    recorded nothing although slow pairs had been seen. Red on 4c8ca07:
    no start truth."""
    monkeypatch.setattr(bridge_mod, "START_TRUTH_WINDOW_S", 1.0)
    b = _bridge(bridge_mod)
    b.last_odom = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "linear_m_s": 0.0,
                   "angular_rad_s": 0.0, "at": time.time() + 60}

    def slow(path):
        time.sleep(0.12)
        return _truth_for(90.0, 0.0)

    _record(bridge_mod, b, slow)
    assert b.start_truth is not None


# ---------- round 5: explore never DELETEs an unplaced goal, never charges it ----------

def _explore_with_a_goal():
    from tests.test_review_376e import _explore
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    assert agent._goal is not None
    return agent, nav


# ---------- round 6 ----------

def test_round6_one_unusable_truth_read_does_not_install_an_identity_anchor():
    """CR: with a truth but no start truth coming, one unusable truth read
    at first contact installed an identity anchor -- for good, since an
    anchor never moves. Red on 156f097: a usable pose in SLAM's own frame."""
    truth = FakeTruth(1.0, 1.0, 90.0)
    truth.pose["usable"] = False
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, truth)
    assert world.get_pose()["usable"] is False and world.map_id is None
    truth.pose["usable"] = True
    world._truth_tried = {}         # as if TRUTH_RETRY_S had passed
    pose = world.get_pose()
    assert world.anchored_at == "first_contact"
    assert (pose["x_m"], pose["y_m"]) == pytest.approx((1.0, 1.0))


# ---------- round 7 ----------

def test_round7_an_unplaced_goal_is_treated_as_not_the_missions_own():
    """3.76 round 7: an unplaced goal (RosWorld's `unanchored`, no x_m) is
    never the mission's own. The mission's goals go through
    RosWorld.set_goal, which refuses unless the bridge's CURRENT session is
    anchored, and a session's anchor is installed once and never changes
    (round 5); so get_goal places every goal of an anchored session, and an
    unanchored goal belongs to a session the mission never sent to (its own
    died with the old container). It is treated as batch E's E9 treats any
    goal that is not the mission's own: Preempted, never cancelled. Red on
    5f174e6: read as `unplaced` and waited on."""
    from robot.interface import Preempted
    agent, nav = _explore_with_a_goal()
    nav.goal = {"state": "active", "unanchored": True}
    nav.calls.clear()
    with pytest.raises(Preempted):
        agent.step()
    assert ("cancel",) not in nav.calls


def test_round7_cancel_does_not_delete_an_unplaced_goal():
    """`_cancel` meets it as someone else's (E9): Preempted, no DELETE.
    Red on 5f174e6: ended `lost` without raising."""
    from robot.interface import Preempted
    agent, nav = _explore_with_a_goal()
    nav.goal = {"state": "active", "unanchored": True}
    nav.calls.clear()
    with pytest.raises(Preempted):
        agent._cancel()
    assert ("cancel",) not in nav.calls


def test_round7_a_verb_after_the_first_goal_meets_it_as_someone_elses():
    """Red on 5f174e6: the verb waited."""
    from robot.interface import Preempted
    from tests.test_review_376e import _explore
    agent, nav = _explore()
    agent._last_sent = (1.0, 1.0)
    nav.goal = {"state": "active", "unanchored": True}
    agent._pending = ["LOOK_LEFT"]
    with pytest.raises(Preempted):
        agent._do_pending()
    assert nav.goal["state"] == "active"


def test_round7_is_own_is_false_without_a_position():
    from tests.test_review_376e import _explore
    agent, _ = _explore()
    agent._last_sent = (1.0, 1.0)
    assert agent._is_own({"state": "active", "unanchored": True}) is False
    assert agent._is_own({"state": "active"}, (1.0, 1.0)) is False


def test_round7_a_truth_that_raises_leaves_the_session_unanchored():
    """CR: RemoteGrid raises when the body process is down; it escaped
    _ensure_session, so get_pose -- and get_goal, on the robot server's
    arbitration path -- raised. Red on 5f174e6: RuntimeError."""
    class Down(FakeTruth):
        def get_truth(self):
            raise RuntimeError("body process down")
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, Down(1.0, 1.0, 90.0))
    assert world.get_pose()["usable"] is False and world.map_id is None


def test_round7_an_unanchored_session_asks_the_truth_at_most_once_a_second():
    """CR: every pose read on the motion path paid a truth round trip while
    unanchored. Red on 5f174e6: 5 reads, 5 truth calls."""
    calls = []

    class Counting(FakeTruth):
        def get_truth(self):
            calls.append(1)
            return {"usable": False}
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, Counting(1.0, 1.0, 90.0))
    for _ in range(5):
        world.get_pose()
    assert len(calls) == 1


# ---------- round 8 ----------

def test_round8_cancels_are_sent_outside_the_bridge_lock(bridge_mod):
    """CR: a future with no executor runs done callbacks inline, so
    rclpy's client lock taken under bridge.lock could deadlock against a
    callback that takes bridge.lock. Red on 4986042: sent with it held."""
    b = _bridge(bridge_mod)
    held = []

    class Probe(_Handle):
        def cancel_goal_async(self):
            free = b.lock.acquire(blocking=False)
            if free:
                b.lock.release()
            held.append(not free)
            return super().cancel_goal_async()

    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    b.cancel_goal()                                      # cancelled while pending
    b.nav.futures[-1].complete(Probe())                  # _on_goal_response sends
    _later(b)
    b.cancel_goal()                                      # an explicit re-send
    _later(b)
    b._resend_cancels()                                  # the timer's
    assert len(held) == 3 and not any(held), held


def test_round8_goal_state_publishes_no_internal_fields(bridge_mod):
    """CR: /goal published monotonic fields (`cancel_sent_at`, `t0`) that
    no client can read against its clock. Red on 4986042: present."""
    b = _bridge(bridge_mod)
    _accepted_goal(b)
    b.cancel_goal()
    g = b.goal_state()[1]["goal"]
    assert not [k for k in g if k.startswith("_") or k in ("t0", "cancel_sent_at")], g


def test_round8_the_bridge_refuses_a_goal_converted_for_another_session(bridge_mod):
    """CR: set_goal read S1's pose and its POST landed on S2 (a restart in
    between): the goal went into the new map, unplaced, and explore took
    it for someone else's. Red on 4986042: accepted, the live goal
    cancelled."""
    b = _bridge(bridge_mod)
    live = _accepted_goal(b)
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0, "session": "an-older-one"})
    assert code == 409 and body["session"] == b.session
    assert live.cancels == 0 and b.goal_state()[1]["goal"]["state"] == "active"
    assert b.send_goal({"x_m": 1.0, "y_m": 0.0, "session": b.session})[0] == 200
    assert b.send_goal({"x_m": 1.0, "y_m": 0.0})[0] == 200          # an old caller


def test_round8_ros_world_turns_a_session_change_into_a_wait():
    """...and RosWorld names the session it converted for, and turns the
    409 into "no SLAM session yet", which explore already waits on. Red on
    4986042: the goal was accepted into the new session."""
    class Restarting(GoalBridge):
        def handler(self, request):
            if request.url.path == "/goal" and request.method == "POST":
                import json
                body = json.loads(request.content)
                if body.get("session") not in (None, self.session):
                    return httpx.Response(409, json={"error": "the SLAM session changed"})
            r = super().handler(request)
            if request.url.path == "/slam/pose" and getattr(self, "restart_after_pose", False):
                self.session, self.restart_after_pose = "s2", False   # restarts just after
            return r
    bridge = Restarting(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.restart_after_pose = True
    reply = world.set_goal(2.0, 1.0)
    assert reply["accepted"] is False and reply["reason"] == "no SLAM session yet"
    assert bridge.goal is None


def test_round8_a_truth_failure_is_logged_once_per_session(caplog):
    """CR: the exception was swallowed silently. Red on 4986042: no log."""
    class Down(FakeTruth):
        def get_truth(self):
            raise RuntimeError("body process down")
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, Down(1.0, 1.0, 90.0))
    with caplog.at_level("WARNING"):
        for _ in range(3):
            world._truth_tried = {}
            world.get_pose()
    hits = [r for r in caplog.records if "body process down" in r.getMessage()]
    assert len(hits) == 1, [r.getMessage() for r in caplog.records]


# ---------- round 9 ----------

def test_round9_a_truth_that_keeps_reading_unusable_is_logged_once(caplog):
    """CR: only an exception was logged; a truth that kept answering
    usable: false left the session unanchored in silence. Red on 406320c:
    no log."""
    truth = FakeTruth(1.0, 1.0, 90.0)
    truth.pose["usable"] = False
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, truth)
    with caplog.at_level("WARNING"):
        for _ in range(3):
            world._truth_tried = {}
            world.get_pose()
    hits = [r for r in caplog.records if "usable: false" in r.getMessage()]
    assert len(hits) == 1, [r.getMessage() for r in caplog.records]


# ---------- round 10 ----------

def test_round10_one_schedule_for_every_goal_and_every_asker(bridge_mod):
    """3.76 rounds 8-10: one schedule -- due every 0.2 s for an entry's
    first 30 s, then every 10 s -- for a stop, the D-pad and the timer
    alike; the cancel's future is not consulted. (Replaces round 8's age
    bands and round 9's explicit flag; a stop on a 6 s-old entry goes at
    once, which round 8 held to 1 s.)"""
    b = _bridge(bridge_mod)
    handle = _accepted_goal(b)
    b.cancel_goal()
    entry = next(iter(b.cancelling.values()))
    assert handle.cancels == 1
    entry["last"] = time.monotonic() + 60.0    # pinned: not due, however slow the machine
    b.cancel_goal()
    b._resend_cancels()
    assert handle.cancels == 1, "re-sent within 0.2 s"
    entry["since"] -= 6.0
    entry["last"] = time.monotonic() - 0.3
    b.cancel_goal()                                      # a stop at age 6 s
    assert handle.cancels == 2
    entry["last"] = time.monotonic() - 0.3
    b._resend_cancels()                                  # the timer, the same rule
    assert handle.cancels == 3
    entry["since"] -= 30.0                               # past 30 s: every 10 s
    entry["last"] = time.monotonic() - 5.0
    b.cancel_goal()
    b._resend_cancels()
    assert handle.cancels == 3
    entry["last"] -= 6.0
    b.cancel_goal()
    assert handle.cancels == 4


def test_round10_a_stop_reaches_a_superseded_goal_aged_6_s_at_once(bridge_mod):
    """CR: cancel_goal() (a stop, the D-pad) asked only for the live goal's
    handle; a superseded goal still cancelling waited for the timer. Now
    one schedule for every entry, whoever asks. Red on cca386b: not sent."""
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})                # old superseded: one cancel
    b.nav.futures[-1].complete(_Handle())
    entry = b.cancelling[id(old)]
    entry["since"] -= 6.0
    entry["last"] = time.monotonic() - 0.3
    b.cancel_goal()
    assert old.cancels == 2


def test_round10_a_409_whose_body_is_not_an_object_does_not_raise():
    """CR, TB: `r.json().get` on a JSON list raised. Red on cca386b:
    AttributeError."""
    class B(GoalBridge):
        def handler(self, request):
            if request.url.path == "/goal" and request.method == "POST":
                return httpx.Response(409, json=["the SLAM session changed"])
            return super().handler(request)
    bridge = B(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    reply = world.set_goal(2.0, 1.0)
    assert reply["accepted"] is False and reply["reason"] == "no SLAM session yet"


def test_round10_each_kind_of_truth_failure_is_logged_and_the_recovery_too(caplog):
    """CR: once a session had warned (unusable), a later exception went
    unlogged, and nothing said when it finally anchored. Red on cca386b:
    one warning, no info line."""
    state = {"mode": "unusable"}

    class Flaky(FakeTruth):
        def get_truth(self):
            if state["mode"] == "raise":
                raise RuntimeError("body process down")
            if state["mode"] == "unusable":
                return {"usable": False}
            return super().get_truth()
    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    world = _world(bridge, Flaky(1.0, 1.0, 90.0))
    with caplog.at_level("INFO", logger="ros_world"):
        for mode in ("unusable", "raise", "ok"):
            state["mode"] = mode
            world._truth_tried = {}
            world.get_pose()
    msgs = [r.getMessage() for r in caplog.records if r.name == "ros_world"]
    assert any("usable: false" in m for m in msgs) and any("body process down" in m for m in msgs), msgs
    assert any("anchored" in m for m in msgs), msgs


def test_round10_a_healthy_truth_read_is_not_followed_by_a_dropout():
    """CR: the throttle was stamped before every read, so a second reader
    racing a healthy first-contact read got an unusable pose for 1 s. Only a
    failed read stamps it now. Red on cca386b: the racing reader unusable."""
    entered, release = threading.Event(), threading.Event()

    class SlowTruth(FakeTruth):
        def get_truth(self):
            entered.set()
            release.wait(5.0)
            return super().get_truth()

    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, SlowTruth(3.0, 2.0, 90.0))
    results = {}
    a = threading.Thread(target=lambda: results.update(a=world.get_pose()), daemon=True)
    a.start()
    assert entered.wait(2.0)
    b = threading.Thread(target=lambda: results.update(b=world.get_pose()), daemon=True)
    b.start()
    time.sleep(0.2)
    release.set()
    a.join(2.0)
    b.join(2.0)
    # Round 10's review: single-flight -- the racing reader is unusable only
    # while A's read is in flight, and a read straight after it is usable
    # (no TRUTH_RETRY_S dropout after a healthy read).
    assert not results["b"]["usable"]
    assert results["a"]["usable"]
    now = world.get_pose()
    assert now["usable"], now
    assert (now["x_m"], now["y_m"]) == pytest.approx((3.0, 2.0))


def test_round10_review_one_truth_read_at_a_time():
    """Round 10's review (CR): with the throttle stamped only on failure,
    every pose reader during a slow first-contact read made its own truth
    call. Now one read is in flight per session; the rest answer unusable."""
    calls, entered, release = [], threading.Event(), threading.Event()

    class SlowTruth(FakeTruth):
        def get_truth(self):
            calls.append(1)
            entered.set()
            release.wait(5.0)
            return super().get_truth()

    bridge = _PendingBridge(start_truth=None)
    bridge.pending = False
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, SlowTruth(3.0, 2.0, 90.0))
    a = threading.Thread(target=world.get_pose, daemon=True)
    a.start()
    assert entered.wait(2.0)
    for _ in range(5):
        assert not world.get_pose()["usable"]
    release.set()
    a.join(2.0)
    assert len(calls) == 1


def test_round10_review_a_500_with_a_text_body_is_an_outage_not_a_crash():
    """`**r.json()` raised JSONDecodeError (not httpx.HTTPError) on a
    bridge's plain-text 500, which the robot server's handler misses."""
    import httpx
    bridge = _PendingBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    bridge.pending = False
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    assert world.get_pose()["usable"]
    bridge.goal_reply = (500, "Internal Server Error")
    with pytest.raises(httpx.HTTPStatusError):
        world.set_goal(2.0, 2.0)


def test_round10_review_a_garbled_200_is_a_fault_not_an_accepted_goal():
    """A proxy's HTML page answered 200 read as `accepted: True`; and a 503
    'nav2 is not up' lost its text. One parser: errors raise with the
    bridge's own words, a non-object body raises (an httpx.HTTPError)."""
    import httpx
    bridge = _PendingBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    bridge.pending = False
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    assert world.get_pose()["usable"]
    bridge.goal_reply = (200, "<html>tunnel page</html>")
    with pytest.raises(httpx.HTTPError):
        world.set_goal(2.0, 2.0)
    bridge.goal_reply = (503, '{"error": "nav2 is not up (navigate_to_pose has no server)"}')
    with pytest.raises(httpx.HTTPStatusError, match="nav2 is not up"):
        world.set_goal(2.0, 2.0)


def test_round10_review_the_navigator_is_only_asked_while_cancelling(bridge_mod):
    b = _bridge(bridge_mod)
    asked = []
    b.nav.server_is_ready = lambda: asked.append(1) or True
    b._resend_cancels()
    assert asked == []


def test_last_review_a_redirect_is_not_an_answer():
    """`>= 400` let a proxy's 307 with a JSON object read as success: a
    goal nav2 never got reported accepted. Anything but 2xx raises."""
    import httpx
    bridge = _PendingBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    bridge.pending = False
    bridge.map_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    world = _world(bridge, FakeTruth(1.0, 1.0, 90.0))
    assert world.get_pose()["usable"]
    bridge.goal_reply = (307, '{"state": "pending"}')
    with pytest.raises(httpx.HTTPStatusError, match="POST /goal"):
        world.set_goal(2.0, 2.0)


def test_last_review_an_ended_goal_does_not_keep_the_navigator_asked(bridge_mod):
    b = _bridge(bridge_mod)
    _accepted_goal(b)
    b.cancel_goal()
    b.cancelling.clear()
    b.goal["state"] = "canceled"              # ended; `_cancel_sent_at` stays on the record
    asked = []
    b.nav.server_is_ready = lambda: asked.append(1) or True
    b._resend_cancels()
    assert asked == []
