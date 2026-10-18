"""
tests/test_review_376m2.py

`PLAN-ros-alignment.md` 3.76, step M2: the bugs the review of the simulated
bodies and maps (`sim/maps/`, `sim/body_*`, `sim/sensor_server.py`,
`sim/fake_esp32.py`, `sim/fake_lidar.py`, `sim/replay_robot.py`,
`sim/teleop_robot.py`, whole files) confirmed. Each test was written before
its fix and seen to fail on `dev` for the reason it names.
"""

import threading
import time

import httpx

from sim.body_client import SimBodyClient


def _client(handler):
    """A SimBodyClient with no servers behind it and no poller: the bundle is
    fetched on the calling thread, as in the module's own test mode."""
    c = object.__new__(SimBodyClient)
    transport = httpx.MockTransport(handler)
    c._poll_http = c._sensors = httpx.Client(base_url="http://sensors", transport=transport)
    c._hint, c._bundle, c._bundle_at = None, None, 0.0
    c._lock = threading.Lock()
    c._wake = threading.Event()
    c._fresh = threading.Condition(c._lock)
    c._thread = None
    return c


def _scan(range_m):
    return {"usable": True, "ranges_m": [range_m], "angle_min_deg": 0.0,
            "angle_increment_deg": 1.0, "range_max_m": range_m}


# ---------- M2.1: one shared range, swapped by every other caller ----------


def test_an_inventory_read_does_not_take_the_safety_bundle():
    """The bundle served one range and any other hinted call replaced it: an
    inventory read (4.5 m) dropped the safety vet's bundle -- the vet then
    waited, inside the motion lock, and moves read `no_scan` -- and was most
    often answered with the safety range's short scan (Thermos: 20 of 20)."""
    seen = []

    def handler(request):
        hint = float(request.url.params.get("max_range_m", "inf"))
        seen.append((request.url.path, hint))
        body = {"scan": _scan(hint), "depth": {}, "distance_cm": 50.0}
        return httpx.Response(200, json=body if request.url.path == "/safety" else _scan(hint))

    c = _client(handler)
    assert c.get_scan(max_range_m=0.9)["range_max_m"] == 0.9      # the vet's
    far = c.get_scan(max_range_m=4.5)                              # the inventory's
    assert far["range_max_m"] == 4.5, far
    assert c._hint == 0.9 and c._bundle is not None, "the safety bundle was dropped"
    assert ("/scan", 4.5) in seen


# ---------- M2.2: a reading's age ran from its reply, not its capture ----------


def test_a_slow_reply_is_aged_from_when_it_was_asked_for():
    """`_bundle_at` was stamped after the reply: a /safety answer that took
    0.4 s read as 0 s old, and `SafetyController._aged` took no travel off
    it (Thermos: ~24 cm at 0.6 m/s, past the 20 cm bar)."""
    def handler(request):
        time.sleep(0.2)
        return httpx.Response(200, json={"scan": _scan(0.9), "depth": {}, "distance_cm": 50.0})

    c = _client(handler)
    c.get_scan(max_range_m=0.9)
    assert c.sensor_age_s() >= 0.19, c.sensor_age_s()


# ---------- M2.3: the default house's rooms disagreed with its layout ----------


import pytest  # noqa: E402

from sim.maps import build_world  # noqa: E402


@pytest.mark.parametrize("house", ["starter_house", "scaled_house", "home_first_floor",
                                   "complex_house"])
def test_room_cells_are_floor_and_belong_to_one_room(house):
    world = build_world(house)
    seen = {}
    for room, cells in world.rooms.items():
        for x, y in cells:
            assert world.layout[y][x] != "#", f"{room} claims wall cell {(x, y)}"
            assert (x, y) not in seen, f"{(x, y)} is in {seen.get((x, y))} and {room}"
            seen[(x, y)] = room


def test_every_free_cell_of_the_default_house_has_a_room():
    """`starter_house` (the default `sim_map`) put six kitchen cells -- the
    target's among them -- in the hallway too (the hallway won), wall cells
    in the hallway, and 19 free cells in no room: a mission to the kitchen
    reached it only on its back row, and rooms entered were wrong."""
    world = build_world("starter_house")
    owned = set().union(*world.rooms.values())
    free = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row) if c != "#"}
    assert free - owned == set(), sorted(free - owned)
    assert world.room_at(10, 7) == "kitchen"


# ---------- M2.4: teleop's frame number read after its lock ----------


def test_an_overlapping_push_returns_its_own_frame_number():
    """`push_frame` logged and returned `self._seq` after the lock: a second
    push landing between them made both return its number, and the twin
    paired a decision with the wrong frame (`lastTeleopSeq`)."""
    from sim.teleop_robot import TeleopRobot
    robot = TeleopRobot()
    real = robot._record
    overlapped = []
    once = []

    def record(event):
        if not once:
            once.append(True)
            overlapped.append(robot.push_frame("b"))     # lands between the two
        real(event)

    robot._record = record
    first = robot.push_frame("a")
    assert first == {"seq": 1}, first
    assert overlapped == [{"seq": 2}]


def test_the_teleop_log_is_bounded():
    from sim.teleop_robot import TeleopRobot
    robot = TeleopRobot()
    for _ in range(3000):
        robot.push_frame("x")
    assert len(robot.log) <= 2000


# ---------- M2.5: replay order past frame 9999 ----------


def test_a_replay_plays_frames_in_their_numbers_order(tmp_path):
    """Frames sorted by name: `frame-10000` before `frame-1001` (the recorder
    pads to four digits), so a long walk played out of order."""
    from sim.replay_robot import ReplayRobot
    for n in (999, 1001, 9999, 10000, 10001):
        (tmp_path / f"frame-{n:04d}.jpg").write_bytes(b"x")
    names = [p.name for p in ReplayRobot(tmp_path).frames]
    assert names == ["frame-0999.jpg", "frame-1001.jpg", "frame-9999.jpg",
                     "frame-10000.jpg", "frame-10001.jpg"], names


# ---------- M2.6: one bad line ended the fake board's command reader ----------


@pytest.mark.parametrize("bad", [b"[1]", b'{"T":11,"L":"x","R":1}', b'{"T":900,"main":7}'])
def test_the_fake_board_keeps_reading_after_a_bad_line(bad):
    """`_handle` had no guard: non-object JSON, a string where `int()` wants a
    number, an unknown main type -- any raise killed `_reader` for good, and
    the last setpoint ran to the heartbeat. The firmware never stops parsing."""
    import os
    from sim.fake_esp32 import FakeEsp32
    from sim.maps.scaled_house import build_scaled_world
    from sim.mock_robot import MockRobot
    board = FakeEsp32(MockRobot(build_scaled_world(), render=False))
    fd = os.open(board.path, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, bad + b"\n" + b'{"T":1,"L":0.3,"R":0.3}\n')
        end = time.monotonic() + 2.0
        while [round(v, 3) for v in board.setpoint] != [0.3, 0.3] and time.monotonic() < end:
            time.sleep(0.01)
        assert [round(v, 3) for v in board.setpoint] == [0.3, 0.3], board.setpoint
    finally:
        os.close(fd)
        board.close()


def test_the_fake_board_takes_no_boolean_speed():
    """ArduinoJson's `is<float>()` refuses `true`; `isinstance(True, int)` did
    not, so the fake was kinder than the firmware."""
    import os
    from sim.fake_esp32 import FakeEsp32
    from sim.maps.scaled_house import build_scaled_world
    from sim.mock_robot import MockRobot
    board = FakeEsp32(MockRobot(build_scaled_world(), render=False))
    fd = os.open(board.path, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b'{"T":1,"L":true,"R":true}\n')
        time.sleep(0.3)
        assert [round(v, 3) for v in board.setpoint] != [1.0, 1.0], board.setpoint
    finally:
        os.close(fd)
        board.close()
