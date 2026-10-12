"""
tests/test_review_376b.py

`PLAN-ros-alignment.md` 3.76, batch B: the bugs the whole-file review of
the body drivers (`robot/hardware_robot.py`, `robot/lidar_ld19.py`,
`robot/ros_drive.py`) confirmed. Each test was written before its fix and
seen to fail on `dev` for the reason it names.
"""

import json
import threading
import time

import httpx
import pytest

import robot.lidar_ld19 as ld19
from robot.hardware_robot import HardwareRobot
from robot.lidar_ld19 import Assembler, Ld19Scanner, Packet, encode
from robot.ros_drive import RosDriveRobot
from sim.fake_esp32 import FakeEsp32
from sim.fake_lidar import Emitter
from sim.maps.scaled_house import build_scaled_world
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot


def _wait(cond, timeout=2.0, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


@pytest.fixture
def car():
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    assert _wait(lambda: robot.get_wheel_state()["usable"]), "no first frame"
    yield robot, board
    robot.close()
    board.close()


# ---------- B1: the board's last command is always the host's ----------


def test_a_stop_during_a_resend_is_the_last_word_on_the_wire(car):
    """`set_wheel_velocity()` set `_cmd` under the lock and wrote the line
    after releasing it, and `advance()` read `_cmd` with no lock at all. A
    `/stop` (which never takes the server's motion lock) landing between
    them left `_cmd` at zero and the BOARD driving: the wheel loop then read
    a stopped robot and never vetted or re-sent again."""
    robot, _ = car
    sent, release, in_send = [], threading.Event(), threading.Event()
    real_send = robot._send

    def send(obj):
        if obj.get("T") == 1 and (obj["L"] or obj["R"]) and threading.current_thread().name == "resend":
            in_send.set()
            release.wait(2.0)                          # the wheel loop's write, delayed
        sent.append(obj)
        return real_send(obj)

    robot._send = send
    robot.set_wheel_velocity(3.0, 3.0)
    resend = threading.Thread(target=robot.advance, args=(0.05,), name="resend")
    resend.start()
    assert in_send.wait(2.0)
    stopper = threading.Thread(target=robot.stop)
    stopper.start()
    time.sleep(0.1)                                    # the stop lands mid-resend
    release.set()
    resend.join()
    stopper.join()
    last = [o for o in sent if o.get("T") == 1][-1]
    w = robot.get_wheel_state()
    assert (last["L"], last["R"]) == (0.0, 0.0), f"the board was left driving: {last}"
    assert (w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]) == (0.0, 0.0)


# ---------- B2: one bad frame does not end feedback for good ----------


def test_a_malformed_feedback_line_does_not_kill_the_reader(car):
    """Only `json.loads`'s ValueError was caught: a valid but non-object
    line (`[]`), or a T:1001 frame missing a speed, raised in the reader
    thread and ended it. Every move was then refused `no_feedback` until a
    restart, with no `link_error` to say why."""
    robot, board = car
    board._write([])
    board._write({"T": 1001})
    board._write({"T": 1001, "L": None, "R": 0.0})
    frames = robot.frames
    assert _wait(lambda: robot.frames > frames + 3), "feedback stopped after a bad line"
    assert robot.get_wheel_state()["usable"]


def test_a_non_finite_wheel_speed_is_dropped_not_integrated(car):
    """json.loads takes NaN: one such frame made the integrated travel NaN
    for good, and every wheel-state read raised from then on."""
    robot, board = car
    board._write({"T": 1001, "L": float("nan"), "R": 0.0, "odl": 0, "odr": 0})   # json writes NaN
    assert _wait(lambda: robot.frames_dropped >= 1), "the NaN frame was taken"
    frames = robot.frames
    assert _wait(lambda: robot.frames > frames + 3)
    w = robot.get_wheel_state()
    assert w["usable"] and w["left"]["position_rad"] == w["left"]["position_rad"]   # not NaN
    assert "non-finite" in robot.feedback_status()["last_drop"]


# ---------- B3: a hung-up serial line ends its reader ----------


def test_the_lidar_reader_ends_on_a_hung_up_line(monkeypatch):
    """After a USB unplug a hung-up tty is always readable and reads b"".
    The reader `continue`d: a busy loop at 100% CPU beside the wheel loop,
    and `link_error` never said why the scan went stale."""
    lidar = ld19.Ld19Lidar.__new__(ld19.Ld19Lidar)
    ld19.Ld19Scanner.__init__(lidar)
    lidar._fd, lidar._running, lidar.link_error, lidar.port = -1, True, None, "/dev/null"
    reads = {"n": 0}

    def read(fd, n):
        reads["n"] += 1
        if reads["n"] > 1000:
            lidar._running = False                     # end the spin the test is about
        return b""

    monkeypatch.setattr(ld19.select, "select", lambda r, w, x, t=None: (r, [], []))
    monkeypatch.setattr(ld19.os, "read", read)
    lidar._reader()
    assert reads["n"] == 1, f"{reads['n']} empty reads -- the reader spun on a hung-up line"
    assert lidar.link_error


def test_the_board_reader_ends_on_a_hung_up_line(car, monkeypatch):
    robot, _ = car
    robot._running = False
    robot._reader_thread.join(2.0)
    import robot.hardware_robot as hw
    reads = {"n": 0}

    def read(fd, n):
        reads["n"] += 1
        if reads["n"] > 1000:
            robot._running = False
        return b""

    monkeypatch.setattr(hw.select, "select", lambda r, w, x, t=None: (r, [], []))
    monkeypatch.setattr(hw.os, "read", read)
    robot._running = True
    robot._reader()
    assert reads["n"] == 1, f"{reads['n']} empty reads -- the reader spun on a hung-up line"
    assert robot.link_error


def test_a_hung_up_board_line_zeroes_the_standing_command(car):
    """`_zero_if_stale()` runs only on the reader thread; a reader that ends
    must zero a standing command itself, or nothing does until the watchdog
    or the board's heartbeat."""
    robot, _ = car
    robot.set_wheel_velocity(3.0, 3.0)
    robot._link_lost("the serial line hung up (end of file)")
    assert robot._cmd == (0.0, 0.0)


# ---------- B4: packets read together are stamped apart ----------


class _Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_a_revolution_read_in_one_late_chunk_is_aged_as_old_as_it_is():
    """Every packet after the first in one read was stamped with the read's
    time, so a revolution delivered whole in one late chunk (the reader held off
    the GIL) read as just taken: `sensor_age_s()` near zero, and the safety
    layer subtracted no travel for a scan that was 0.1 s old."""
    clock = _Clock()
    s = Ld19Scanner(clock=clock)
    em = Emitter(lambda a, t: 2000, t0=clock.t)
    for _ in range(80):                                # ~2 revolutions, read on time
        clock.t += 12 / 4500.0
        s.feed(b"".join(em.due(clock.t)), clock.t)
    clock.t += 0.25                                    # the reader stalls 250 ms: a whole revolution
    s.feed(b"".join(em.due(clock.t)), clock.t)         # ...then reads it all at once
    assert s.sensor_age_s() >= 0.05, f"a 0.1 s revolution read as {s.sensor_age_s():.3f} s old"


def test_packets_after_a_lidar_pause_are_stamped_by_the_lidars_clock():
    """The other half of B4: when the LIDAR paused (not the reader), the
    packets that then arrive together were measured just now. Spread evenly
    across the gap, they would read older than they are; the lidar's own
    `timestamp_ms` places them. (A guard on B4's fix, not a `dev` bug:
    `dev` stamped every packet at the read's time.)"""
    clock = _Clock()
    s = Ld19Scanner(clock=clock)
    em = Emitter(lambda a, t: 2000, t0=clock.t)
    for _ in range(80):
        clock.t += 12 / 4500.0
        s.feed(b"".join(em.due(clock.t)), clock.t)
    gap = 0.2
    clock.t += gap
    em = Emitter(lambda a, t: 2000, t0=clock.t - 0.01)   # it resumes 10 ms before the read
    burst = b"".join(em.due(clock.t))
    s.feed(burst, clock.t)
    assert s._t_prev == clock.t
    newest_points = [p for p in s._assembler._points][-12:]
    assert min(t for _, _, t in newest_points) >= clock.t - 0.02, \
        "packets measured 10 ms ago were stamped across the 200 ms pause"


# ---------- B5: a lidar whose angle stops advancing cannot fill memory ----------


def test_a_revolution_that_never_wraps_is_bounded():
    """Points were appended until the angle wrapped. A lidar still sending
    packets with its angle stuck (the case `_fresh()` names) grew one list
    by ~4,500 points a second until the Jetson ran out of memory."""
    a = Assembler()
    stuck = [(10.0, 1000, 0.0)] * 12
    for _ in range(5000):
        a.add(stuck)
    assert len(a._points) <= 4 * 450, len(a._points)


# ---------- B6: a zero-degree turn moves nothing ----------


def test_a_zero_degree_turn_sends_no_motion(car):
    """`verb_plan()` answers None for angle 0, so the safety layer
    dispatched `turn_left(0)`, whose `_run()` sent a full-speed pivot
    command -- never vetted -- before zeroing it."""
    robot, _ = car
    sent, real_send = [], robot._send
    robot._send = lambda obj: sent.append(obj) or real_send(obj)
    robot.turn_left(0)
    robot.turn_right(0)
    assert not [o for o in sent if o.get("T") == 1 and (o["L"] or o["R"])], sent


# ---------- B7: the board clock map follows a slow board ----------


def test_the_frame_age_does_not_grow_with_uptime_on_a_slow_board():
    """The host-minus-board offset only ever took a new minimum. A board
    crystal running slow makes that offset grow, so the frame's age grew
    with uptime: at 50 ppm it passed `EXTRAPOLATE_MAX_S` (0.1 s) in about
    35 minutes, and every moving position read was extrapolated by a full
    0.1 s of speed."""
    hw = HardwareRobot.__new__(HardwareRobot)
    hw._board_s, hw._last_ms, hw._clock_offset = 0.0, None, None
    t0, latency = 1000.0, 0.004
    hw._track_board_clock(0, t0 + latency, first=True)
    hw._last_ms = 0
    for k in range(1, 2 * 3600 * 20):                  # two hours of 20 Hz frames
        host_t = k * 0.05
        ms = int(host_t * (1 - 50e-6) * 1000)
        hw._track_board_clock(ms, t0 + host_t + latency)
    now = t0 + host_t + latency
    age = now - (hw._board_s + hw._clock_offset)
    assert age < 0.02, f"a fresh frame reads {age:.3f} s old after two hours"


# ---------- B8: a ROS verb sends nothing after a stop ----------


def test_a_ros_verb_posts_no_motion_after_a_stop():
    """`_run()` checked for a stop only at the top of its loop: a stop
    landing while it read the encoders was followed by one more non-zero
    twist, which could reach ROS after the stop's zeros."""
    inner = MockRobot(build_starter_world(), render=False)
    robot = RosDriveRobot(inner, "http://bridge")
    posts, stopped = [], {"at": None}

    def bridge(request):
        if request.url.path == "/cmd_vel":
            posts.append(json.loads(request.content))
        return httpx.Response(200, json={})

    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge))
    real_state, calls = inner.get_wheel_state, {"n": 0}

    def state():
        calls["n"] += 1
        if calls["n"] == 4 and stopped["at"] is None:
            robot.stop()                               # lands inside progress()
            stopped["at"] = len(posts)
        return real_state()

    inner.get_wheel_state = state
    robot.turn_right(90)
    time.sleep(0.2)                                    # the stop's zeros finish
    assert stopped["at"] is not None, "the stop never landed -- the test proves nothing"
    after = [p for p in posts[stopped["at"]:] if p["linear_m_s"] or p["angular_rad_s"]]
    assert not after, f"motion posted after the stop: {after}"


def test_an_overflowing_speed_is_dropped_too(car):
    """The rest of B2 (the fixes' own review): float() of a 400-digit
    integer raises OverflowError, outside the first catch list."""
    robot, board = car
    board._write({"T": 1001, "L": int("9" * 400), "R": 0.0})   # json writes the digits
    assert _wait(lambda: robot.frames_dropped >= 1)
    frames = robot.frames
    assert _wait(lambda: robot.frames > frames + 3), "feedback stopped after the line"
