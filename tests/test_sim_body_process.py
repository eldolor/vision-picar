"""PLAN-ros-alignment.md 3.36 -- the simulated body as its own programs.

Under `SIM_MOTOR_BOARD=fake` the robot server used to run the simulator in
its own process, and on the Jetson that process could not keep its 20 Hz
wheel loop on time (3.33's G4). Now `sim/body_server.py` (physics, the fake
board, the truth) and `sim/sensor_server.py` (the sensors, several worker
processes, reading the body's state from shared memory) own all of it, and
the robot server does what it does on the car: opens the board's serial
line and reads its sensors from outside itself.

These start the two programs for real (tests/conftest.py's SimPrograms).
"""

import json
import math
import os
import subprocess
import sys
import time

import httpx
import pytest

from tests.conftest import REPO_ROOT

SIM_MODULES = ("sim.fake_esp32", "sim.grid_world", "sim.renderer", "sim.mock_robot")


@pytest.fixture
def programs(sim_programs):
    return sim_programs()


@pytest.fixture
def remote_robot(programs, monkeypatch):
    """The factory's robot over the split simulator."""
    programs.apply(monkeypatch)
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    from robot.factory import get_robot
    bot = get_robot()
    deadline = time.monotonic() + 3
    while bot.frames == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    yield bot
    bot.close()
    bot.sensors.close()


def _truth(programs):
    return httpx.get(f"{programs.body_url}/truth").json()


def _vet_range():
    from robot.safety import FOOTPRINT_LENGTH_M, SAFETY_SCAN_RANGE_M
    return max(SAFETY_SCAN_RANGE_M, FOOTPRINT_LENGTH_M / 2 + 1.5 * 20.0 / 100.0)


# ---------- criterion 1: the robot server runs no simulation ----------

def test_the_robot_server_process_imports_no_simulator(programs):
    """Checked in a subprocess, like the brain's own isolation test, so this
    file's imports cannot mask it."""
    probe = ("import robot.server, sys; "
             "print(sorted(m for m in sys.modules if m.startswith('sim.')))")
    env = {k: v for k, v in os.environ.items() if k != "APP_SHARED_SECRET"}
    env.update({**programs.env(), "WORLD_MODE": "none", "ROBOT_DRIVE": "direct"})
    out = subprocess.run([sys.executable, "-c", probe], cwd=REPO_ROOT, env=env,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr[-2000:]
    loaded = eval(out.stdout.strip().splitlines()[-1])
    assert not [m for m in loaded if m in SIM_MODULES], loaded


# ---------- criterion 2: same body, same answers ----------

def test_an_unhinted_scan_is_the_sensor_programs_own(remote_robot, programs):
    mine = remote_robot.get_scan()
    theirs = httpx.get(f"{programs.sensor_urls[-1]}/scan").json()
    assert mine["usable"] and len(mine["ranges_m"]) == 360
    assert mine["ranges_m"] == theirs["ranges_m"]


def test_a_forward_verb_moves_the_simulated_body(remote_robot, programs):
    from robot.interface import carry_out_verb
    before = _truth(programs)
    result = carry_out_verb(remote_robot, remote_robot.verb_plan("FORWARD"))
    after = _truth(programs)
    moved = math.dist((before["x_m"], before["y_m"]), (after["x_m"], after["y_m"]))
    assert 0.27 <= moved <= 0.33, (moved, result)


def test_the_sensors_follow_the_body_as_it_moves(remote_robot, programs):
    """After a turn, the safety bundle is cast from the new pose: it equals
    a direct read from the sensor program once both see the settled body."""
    from robot.interface import carry_out_verb
    hint = 12.0
    before = remote_robot.get_scan(max_range_m=hint)["ranges_m"]
    carry_out_verb(remote_robot, remote_robot.verb_plan("LEFT", angle=90))
    time.sleep(0.3)
    bundle = remote_robot.get_scan(max_range_m=hint)
    direct = httpx.get(f"{programs.sensor_urls[-1]}/scan", params={"max_range_m": hint}).json()
    assert bundle["usable"] and bundle["ranges_m"] == direct["ranges_m"]
    assert bundle["ranges_m"] != before


def test_the_sim_extras_answer_through_the_process_boundary(remote_robot):
    grid = remote_robot.world
    assert grid.map_name == "starter_house"
    objs = grid.describe_objects()
    assert objs["objects"] and "sim_time_s" in objs
    assert remote_robot.get_truth()["usable"]
    assert remote_robot.get_camera_frame()["image_base64"]
    assert any(z["status"] != "unusable" for z in remote_robot.get_depth_grid()["zones"])
    assert remote_robot.look_left()["pan"] != 0
    remote_robot.look_center()


def test_moving_furniture_reaches_physics_and_the_sensors(remote_robot, programs):
    """A move physics accepts is seen by the sensor workers (their replica
    rebuilds its objects from the shared state); a refused one comes back as
    the in-process grid's ValueError."""
    grid = remote_robot.world
    objects = grid.describe_objects()["objects"]
    taken = {(o["x"], o["y"]) for o in objects}
    moved = None
    for o in objects:
        if o["mover"]:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            dst = (o["x"] + dx, o["y"] + dy)
            if dst in taken:
                continue
            try:
                grid.move_object([o["x"], o["y"]], list(dst))
            except ValueError:
                continue
            moved = (o, dst)
            break
        if moved:
            break
    assert moved, "no object in the starter house could move to a free neighbour"
    o, dst = moved
    names = {(x["x"], x["y"]): x["name"] for x in grid.describe_objects()["objects"]}
    assert names.get(dst) == o["name"] and (o["x"], o["y"]) not in names
    with pytest.raises(ValueError):
        grid.move_object([o["x"], o["y"]], list(dst))


def test_what_crosses_the_boundary_is_exactly_what_the_body_measured():
    """At the 3.18 sweep's own near-obstacle starts in every house and
    heading, the scan and depth grid come back from the server's encoder
    equal to what the body measured -- so the vet, fed either, decides the
    same."""
    from fastapi.responses import JSONResponse
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.footprint_sweep import starts

    checked = 0
    for house in ("starter_house", "scaled_house", "home_first_floor"):
        for x, y in starts(house, 10, seed=36):
            for heading_deg in range(0, 360, 45):
                world = build_world(house)
                world.x, world.y, world.theta = x, y, math.radians(heading_deg)
                body = MockRobot(world, render=False)
                for reading in (body.get_scan(), body.get_scan(max_range_m=1.0),
                                body.get_depth_grid()):
                    assert json.loads(JSONResponse(content=reading).body) == reading
                checked += 1
    assert checked == 3 * 10 * 8


# ---------- criteria 3 and 8: dying programs stop the wheels ----------

def _robot_on(programs):
    from robot.hardware_robot import HardwareRobot
    from sim.body_client import SimBodyClient
    client = SimBodyClient(programs.body_url, programs.sensors_url)
    bot = HardwareRobot(client.board_path, sensors=client, track_scrub=1.0)
    deadline = time.monotonic() + 3
    while bot.frames == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    return bot, client


def test_the_physics_program_dying_stops_the_wheels(programs):
    """The board dies with physics, its feedback goes stale, and 3.34's rule
    zeroes the standing command within its bar."""
    bot, client = _robot_on(programs)
    try:
        bot.set_wheel_velocity(1.0, 1.0)
        time.sleep(0.2)
        programs.kill_body()
        killed = time.monotonic()
        while time.monotonic() - killed < 2.0 and bot._cmd != (0.0, 0.0):
            time.sleep(0.01)
        waited = time.monotonic() - killed
        assert bot._cmd == (0.0, 0.0), bot._cmd
        assert waited <= 0.35 + 0.1, waited
    finally:
        bot.close()
        client.close()


def test_the_sensor_program_dying_blinds_the_vet_in_time(programs):
    """Criterion 8: no fresh bundle within SENSOR_STALE_S and the scan,
    depth and distance read unusable -- the fail-safe path, where forward is
    vetoed -- well inside the 0.5 s silence bar."""
    from robot.safety import SafetyController
    from sim.body_client import POLL_S, SENSOR_STALE_S
    bot, client = _robot_on(programs)
    try:
        safety = SafetyController(bot, 20.0)
        hint = _vet_range()
        assert client.get_scan(max_range_m=hint)["usable"]
        programs.kill_sensors()
        killed = time.monotonic()
        while time.monotonic() - killed < 2.0 and client.get_scan(max_range_m=hint)["usable"]:
            time.sleep(0.005)
        waited = time.monotonic() - killed
        assert waited <= SENSOR_STALE_S + POLL_S + 0.1, waited
        assert all(z["status"] == "unusable" for z in client.get_depth_grid()["zones"])
        assert client.get_distance() == 0.0
        left, right, reason = safety.vet_wheel_velocity(3.0, 3.0)
        assert (left, right) == (0.0, 0.0) and reason, (left, right, reason)
    finally:
        bot.close()
        client.close()


# ---------- criterion 9: no motion_lock holder blocks on a socket ----------

def test_the_vet_never_touches_the_network(programs):
    """Once the bundle is primed, every read the safety layer makes -- the
    hinted scan, the depth grid, the distance -- is served from memory.
    Proven by making the network raise and vetting anyway."""
    from robot.safety import SafetyController
    bot, client = _robot_on(programs)
    try:
        safety = SafetyController(bot, 20.0)
        safety.vet_wheel_velocity(1.0, 1.0)                    # primes the range
        time.sleep(0.1)

        def no_network(*a, **k):
            raise AssertionError("the vet made a network call")

        for c in (client._sensors, client._body):
            c.get = c.post = no_network
        for _ in range(20):
            for left, right in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)):
                safety.vet_wheel_velocity(left, right)
            time.sleep(0.01)
    finally:
        bot.close()
        client._running = False
        client._poll_http.close()


# ---------- criterion 7: the sensor load is on more than one core ----------

def test_the_safety_stream_and_the_heavy_reads_are_different_programs(remote_robot):
    """The poller's bundles come from the first sensor program and ROS's
    full scans and frames from the last -- different processes, so the two
    loads cannot queue on one GIL."""
    client = remote_robot.sensors
    assert len(client.sensor_urls) >= 2
    poll_pid = client._poll_http.get("/health").json()["pid"]
    heavy_pid = client._sensors.get("/health").json()["pid"]
    assert poll_pid != heavy_pid
    assert remote_robot.get_scan()["usable"]                 # served by the heavy one
    assert client.polls > 0                                  # and the stream is live


# ---------- criterion 10: no fake board in the robot server's process ----------

def test_the_factory_refuses_an_in_process_fake_board(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    monkeypatch.setenv("SIM_MOTOR_BOARD", "fake")
    monkeypatch.delenv("SIM_BODY_URL", raising=False)
    monkeypatch.delenv("SIM_SENSORS_URL", raising=False)
    from robot.factory import get_robot
    with pytest.raises(ValueError, match="separate programs"):
        get_robot()
