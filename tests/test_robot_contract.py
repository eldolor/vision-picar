"""
tests/test_robot_contract.py

Phase S1 (PLAN-sim-hardening.md) -- pin RobotInterface's return-shape
contract as an explicit, backend-agnostic suite every backend must pass,
not just MockRobot. Before this file, `robot/interface.py` was an ABC with
signatures but no documented return-shape contract, no units, no error
semantics -- and there was nothing to check `robot/hardware_robot.py`
against but "does it look right" once Phase 11 lands. There wasn't even
anything checking that RemoteRobot, ReplayRobot and TeleopRobot -- three
backends added after the interface was first written -- actually agree
with each other and with MockRobot.

Every test below is phrased in backend-neutral terms: no grid coordinates,
no cell counts, no room names beyond "is it a non-empty string". A test
that only passes against one backend has found a real contract violation,
not a backend-specific quirk -- that's the point of running each one once
per backend via the `robot` fixture below, rather than writing MockRobot's
own tests four times with the noun changed.

**The depth grid is pinned here as of phase M2** -- the same
extension, one sensor later. `get_depth_grid()` is the only method on
`RobotInterface` with a default implementation, which makes it the only
one a backend can get wrong by saying nothing: three of the four backends
below inherit the all-unusable answer and are supposed to, and the fourth
must not. The tests therefore pin the *shape* for every backend and the
tri-state's meaning for all of them, rather than pinning a distance any
one backend happens to produce.

**Pixels are pinned here as of phase S2.** They were not before: MockRobot
returned grid facts and no image, so asserting pixels would have been
either false for it or too loose to mean anything, and this file said to
revisit when S2 landed. It has -- `sim/renderer.py` gives the grid world a
real raycast camera -- so all four backends now agree on a decodable image
plus a `room` key, and that is what's pinned below. This is the extension
S1 was designed for rather than a rewrite of it.

Note `PIXEL` is a genuine (tiny) JPEG rather than the four-byte stub it
used to be. A contract that says "decodable image" has to be tested with
something actually decodable, or it only pins the base64.

Run with: pytest tests/test_robot_contract.py -v
"""

import base64
import io

import pytest
from PIL import Image

from robot.interface import (
    RobotInterface,
    ZONE_NO_TARGET,
    ZONE_RANGE,
    ZONE_UNUSABLE,
)
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot
from sim.replay_robot import ReplayRobot
from sim.teleop_robot import TeleopRobot


def _tiny_jpeg() -> str:
    """A real 8x8 JPEG, base64. Built rather than pasted so it stays
    obvious what it is and cannot rot into an unreadable blob."""
    buf = io.BytesIO()
    Image.new("RGB", (8, 8), (90, 100, 120)).save(buf, format="JPEG")
    return base64.b64encode(buf.getvalue()).decode()


PIXEL = _tiny_jpeg()

MOVEMENT_METHODS = ("drive_forward", "reverse")
TURN_METHODS = ("turn_left", "turn_right")
PAN_METHODS = ("look_left", "look_right", "look_center")

# The four backends that exist, plus the one wrapper every mission's robot
# is actually driven through. `_HaltGate` implements RobotInterface and
# delegates method by method, which makes it the shape most likely to fall
# behind the interface silently: it did exactly that when get_depth_grid()
# landed, inheriting the all-unusable default while wrapping a MockRobot
# that had a working sensor. Nothing else in the suite would have noticed,
# because a gate that forgets a *sensing* method still passes every test
# about the methods it does guard.
BACKENDS = ["mock", "replay", "teleop", "remote", "halt_gate"]


def _write_walk(tmp_path, count=3):
    for i in range(count):
        (tmp_path / f"frame-{i:03d}.jpg").write_bytes(base64.b64decode(PIXEL))
    return tmp_path


@pytest.fixture(params=BACKENDS)
def robot(request, tmp_path):
    """One RobotInterface instance per backend this project ships today.
    Parametrizing the fixture (rather than the test functions) is what
    makes every test in this file run once per backend automatically."""
    kind = request.param

    if kind == "mock":
        yield MockRobot(build_starter_world())

    elif kind == "replay":
        yield ReplayRobot(_write_walk(tmp_path))

    elif kind == "teleop":
        bot = TeleopRobot(stall_timeout_s=30.0)
        bot.push_frame(PIXEL, "image/jpeg")
        yield bot

    elif kind == "remote":
        # Reuses tests/conftest.py's own fixture (an in-process
        # RemoteRobot over robot/server.py's ASGI app, mode: sim under the
        # hood) rather than re-implementing the ASGI-mounting dance here.
        yield request.getfixturevalue("robot_over_asgi")

    elif kind == "halt_gate":
        # is_running=True: the gate open, which is its state for all but
        # the last instant of a mission. Its refusal behaviour when closed
        # is tests/test_failsafes.py's subject, not this file's.
        from control.mission_runner import _HaltGate

        yield _HaltGate(MockRobot(build_starter_world()), lambda: True)

    else:  # pragma: no cover -- guards a typo in BACKENDS above
        raise ValueError(f"unknown backend: {kind!r}")


# ---------- the interface itself ----------


def test_every_backend_is_a_robot_interface(robot):
    assert isinstance(robot, RobotInterface)


# ---------- driving, turning, panning: dict out, the right action name ----------


def test_movement_methods_return_a_dict_named_after_themselves(robot):
    for method in MOVEMENT_METHODS:
        result = getattr(robot, method)()
        assert isinstance(result, dict), f"{method}() must return a dict"
        assert result.get("action") == method, f"{method}()'s own result must self-identify"


def test_turn_methods_return_a_dict_named_after_themselves(robot):
    for method in TURN_METHODS:
        result = getattr(robot, method)()
        assert isinstance(result, dict), f"{method}() must return a dict"
        assert result.get("action") == method


def test_pan_methods_return_a_dict_named_after_themselves(robot):
    for method in PAN_METHODS:
        result = getattr(robot, method)()
        assert isinstance(result, dict), f"{method}() must return a dict"
        assert result.get("action") == method


def test_drive_and_reverse_accept_speed_and_duration(robot):
    """Matches robot/interface.py's declared signature -- speed: int,
    duration: float -- regardless of whether a given backend does anything
    with them (TeleopRobot logs and acks; MockRobot converts to cells)."""
    assert isinstance(robot.drive_forward(speed=30, duration=0.2), dict)
    assert isinstance(robot.reverse(speed=30, duration=0.2), dict)


def test_turns_accept_an_angle(robot):
    assert isinstance(robot.turn_left(angle=45), dict)
    assert isinstance(robot.turn_right(angle=45), dict)


def test_stop_is_idempotent(robot):
    """Calling stop() when already stopped must never raise -- it's the
    one command every failsafe (B3.1-B3.3) depends on being safe to issue
    at any time, including twice in a row."""
    first = robot.stop()
    second = robot.stop()
    assert isinstance(first, dict) and first.get("action") == "stop"
    assert isinstance(second, dict) and second.get("action") == "stop"


# ---------- sensing ----------


def test_get_distance_is_a_non_negative_number_of_centimeters(robot):
    distance = robot.get_distance()
    assert isinstance(distance, (int, float))
    assert distance >= 0, "a negative distance has no physical meaning"


def test_get_camera_frame_is_a_dict_with_a_room_key(robot):
    """The one thing every backend agrees on today, pixels or not: a dict,
    with a room label MissionMemory can key off of (a real one, or the
    honest 'unknown' a camera-only backend reports -- see
    sim/replay_robot.py and sim/teleop_robot.py's own docstrings)."""
    frame = robot.get_camera_frame()
    assert isinstance(frame, dict)
    assert isinstance(frame.get("room"), str) and frame["room"], (
        "room must be a non-empty string, even when its value is 'unknown'"
    )


def test_get_camera_frame_carries_a_decodable_image(robot):
    """Phase S2's contract, and the reason `RobotInterface` does not have
    to change when hardware lands: every backend answers with pixels, so
    a Pi camera is a fourth implementation of a shape that already exists
    rather than a fifth shape (PLAN-sim-hardening.md 2.1)."""
    frame = robot.get_camera_frame()

    data = frame.get("image_base64")
    assert isinstance(data, str) and data, "every backend must supply image_base64"

    raw = base64.b64decode(data, validate=True)
    img = Image.open(io.BytesIO(raw))
    img.verify()  # raises if the bytes are not a real image
    assert img.width > 0 and img.height > 0


def test_get_camera_frame_declares_its_media_type(robot):
    """A vision call has to name the media type it is sending, so guessing
    it here would just move the guess downstream."""
    media_type = robot.get_camera_frame().get("media_type")
    assert isinstance(media_type, str)
    assert media_type.startswith("image/"), media_type


# ---------- the depth grid (phase M2) ----------

VALID_ZONE_STATUSES = {ZONE_RANGE, ZONE_NO_TARGET, ZONE_UNUSABLE}


def test_get_depth_grid_is_a_row_major_grid_that_declares_its_own_shape(robot):
    """`rows`/`cols` travel in the data rather than being fixed by the
    contract, which is what lets a backend say the vertical dimension is
    absent (MockRobot publishes `rows: 1`, because `cast_ray()` has no
    elevation) instead of faking eight identical rows."""
    grid = robot.get_depth_grid()
    assert isinstance(grid, dict)

    rows, cols = grid.get("rows"), grid.get("cols")
    assert isinstance(rows, int) and rows >= 1, "rows must be a positive int"
    assert isinstance(cols, int) and cols >= 1, "cols must be a positive int"

    zones = grid.get("zones")
    assert isinstance(zones, list)
    assert len(zones) == rows * cols, (
        f"declared {rows}x{cols} but sent {len(zones)} zones -- a consumer "
        "indexing row-major would read past the end or silently drop a row"
    )


def test_every_zone_is_one_of_the_three_outcomes(robot):
    """Two would not be enough, and that is the whole point of M3: "nothing
    is there" is information about the room, "I could not tell" is the
    absence of information, and a veto that treats the second as the first
    (or as 0.0cm) either drives into things or never moves."""
    for zone in robot.get_depth_grid()["zones"]:
        assert isinstance(zone, dict)
        assert zone.get("status") in VALID_ZONE_STATUSES, zone


def test_only_a_measured_zone_carries_a_number(robot):
    """The contract that makes the tri-state load-bearing: a consumer can
    filter on `status` and never has to decide what `distance_cm` means on
    a zone that failed. A sentinel here -- 0.0, or a max range -- is what
    `sim/sensors.py`'s docstring can defend for a lone scalar and what M3
    argues is wrong for a grid."""
    for zone in robot.get_depth_grid()["zones"]:
        if zone["status"] == ZONE_RANGE:
            assert isinstance(zone["distance_cm"], (int, float))
            assert zone["distance_cm"] >= 0, "a negative distance has no meaning"
        else:
            assert zone["distance_cm"] is None, (
                f"{zone['status']} carries no distance -- reporting one is how "
                "an unmeasurable zone gets compared against a threshold"
            )


def test_a_sensorless_backend_says_so_rather_than_guessing(robot):
    """`NO_SENSOR_CM`'s rule, one sensor later, stated as one biconditional
    so neither half can drift: a backend whose scalar reports no sensor must
    publish no measured zone, and a backend that publishes a measured zone
    must not report no sensor. A photograph has no depth in it and a live
    phone walk has no ToF -- those backends answer all-unusable, never a
    fabricated clearance that would make a recorded walk look as though it
    had exercised collision avoidance it never had.

    The failure this is written against is a wrapper: `_HaltGate` and
    `RecordingRobot` both delegate every other method and would have
    inherited `RobotInterface`'s all-unusable default while wrapping a
    robot that had a sensor -- a silent lie in the one direction a safety
    consumer must not be lied to."""
    from robot.interface import NO_SENSOR_CM

    grid = robot.get_depth_grid()
    measured = [z for z in grid["zones"] if z["status"] == ZONE_RANGE]
    sensorless = robot.get_distance() == NO_SENSOR_CM

    if sensorless:
        assert not measured, (
            "the scalar reports no distance sensor while the grid publishes "
            f"{len(measured)} measured zones -- which sensor is real?"
        )
        assert all(z["status"] == ZONE_UNUSABLE for z in grid["zones"]), (
            "a backend with no depth sensor must answer all-unusable: "
            "'nothing is there' is a claim about the room it cannot make"
        )
    else:
        assert measured or all(
            z["status"] == ZONE_NO_TARGET for z in grid["zones"]
        ), (
            "a backend with a working scalar published no usable zone at all"
        )
