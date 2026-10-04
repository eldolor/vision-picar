"""PLAN-ros-alignment.md 3.36 -- the simulated body in its own process.

Under `SIM_MOTOR_BOARD=fake` the robot server used to run the simulator
(the GridWorld body, its ray-cast sensors, the fake board's loop) in its
own process, and on the Jetson that process could not keep its 20 Hz wheel
loop on time (3.33's G4). With `SIM_BODY_URL` set, `sim/body_server.py`
owns all of it, and the robot server does what it does on the car: opens
the board's serial line and reads its sensors from outside itself.

These run a real body server subprocess; the robot server half is in
process (or in a subprocess, where its imports are the subject).
"""

import os
import subprocess
import sys
import time

import httpx
import pytest

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

SIM_MODULES = ("sim.fake_esp32", "sim.grid_world", "sim.renderer", "sim.mock_robot")


def _start_body(env_extra=None):
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_SHARED_SECRET", "SIM_BODY_URL", "SIM_MOVERS")}
    env.update({"SIM_MAP": "starter_house", **(env_extra or {})})
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "sim.body_server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT, env=env)
    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"body server exited early with code {proc.returncode}")
        try:
            if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                return proc, url
        except httpx.HTTPError:
            time.sleep(0.1)
    proc.kill()
    pytest.fail("body server was not healthy in time")


def _stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture
def body():
    proc, url = _start_body()
    yield url
    _stop(proc)


@pytest.fixture
def remote_robot(body, monkeypatch):
    """The factory's robot with the body out of process: HardwareRobot over
    the body server's pty, sensors over its HTTP."""
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    monkeypatch.setenv("SIM_MOTOR_BOARD", "fake")
    monkeypatch.setenv("SIM_BODY_URL", body)
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    from robot.factory import get_robot
    bot = get_robot()
    deadline = time.monotonic() + 3
    while bot.frames == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    yield bot
    bot.close()


# ---------- criterion 1: the robot server runs no simulation ----------

def test_the_robot_server_process_imports_no_simulator(body):
    """Checked in a subprocess, like the brain's own isolation test, so this
    file's imports cannot mask it."""
    probe = ("import robot.server, sys; "
             "print(sorted(m for m in sys.modules if m.startswith('sim.')))")
    env = {k: v for k, v in os.environ.items() if k != "APP_SHARED_SECRET"}
    env.update({"ROBOT_MODE": "hardware", "SIM_MOTOR_BOARD": "fake",
                "SIM_BODY_URL": body, "WORLD_MODE": "none", "ROBOT_DRIVE": "direct"})
    out = subprocess.run([sys.executable, "-c", probe], cwd=REPO_ROOT, env=env,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr[-2000:]
    loaded = eval(out.stdout.strip().splitlines()[-1])
    assert not [m for m in loaded if m in SIM_MODULES], loaded


# ---------- criterion 2: same body, same answers ----------

def test_the_scan_is_the_body_servers_own(remote_robot, body):
    mine = remote_robot.get_scan()
    theirs = httpx.get(f"{body}/scan").json()
    assert mine["usable"] and mine["ranges_m"] == theirs["ranges_m"]


def test_a_forward_verb_moves_the_simulated_body(remote_robot, body):
    before = httpx.get(f"{body}/truth").json()
    from robot.interface import carry_out_verb
    result = carry_out_verb(remote_robot, remote_robot.verb_plan("FORWARD"))
    after = httpx.get(f"{body}/truth").json()
    moved = ((after["x_m"] - before["x_m"]) ** 2 + (after["y_m"] - before["y_m"]) ** 2) ** 0.5
    assert 0.27 <= moved <= 0.33, (moved, result)


def test_the_sim_extras_answer_through_the_process_boundary(remote_robot, body):
    grid = remote_robot.world
    assert grid.map_name == "starter_house"
    objs = grid.describe_objects()
    assert objs["objects"] and "sim_time_s" in objs
    assert remote_robot.get_truth()["usable"]
    assert remote_robot.get_camera_frame()["image_base64"]
    assert any(z["status"] != "unusable" for z in remote_robot.get_depth_grid()["zones"])


def test_moving_furniture_reaches_the_body(remote_robot):
    """A move the body accepts is seen through the boundary; one it refuses
    comes back as the same ValueError the in-process grid raises."""
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
                continue                      # a wall, or the turning circle
            moved = (o, dst)
            break
        if moved:
            break
    assert moved, "no object in the starter house could move to a free neighbour"
    o, dst = moved
    names = {(x["x"], x["y"]): x["name"] for x in grid.describe_objects()["objects"]}
    assert names.get(dst) == o["name"] and (o["x"], o["y"]) not in names
    with pytest.raises(ValueError):
        grid.move_object([o["x"], o["y"]], list(dst))      # nothing there now


def test_an_unreachable_body_fails_safe(remote_robot, body, monkeypatch):
    """A sensor read that cannot reach the body answers 'unusable' (and 0.0
    for the scalar, which always vetoes) -- never raises into the wheel loop,
    where a raised tick would leave a standing command undriven by any vet."""
    remote_robot.sensors._http.close()
    import httpx as _h
    remote_robot.sensors._http = _h.Client(base_url="http://127.0.0.1:9", timeout=0.2)
    assert remote_robot.get_scan()["usable"] is False
    assert any(z["status"] != "unusable" for z in remote_robot.get_depth_grid()["zones"]) is False
    assert remote_robot.get_distance() == 0.0


# ---------- criterion 3: no new latency in the stop ----------

def test_the_body_process_dying_stops_the_wheels():
    """The board dies with the body process, its feedback goes stale, and
    3.34's rule zeroes the standing command -- within the same 0.35 s bar."""
    proc, url = _start_body()
    try:
        from robot.hardware_robot import HardwareRobot
        from sim.body_client import SimBodyClient
        client = SimBodyClient(url)
        bot = HardwareRobot(client.board_path, sensors=client, track_scrub=1.0)
        deadline = time.monotonic() + 3
        while bot.frames == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        bot.set_wheel_velocity(1.0, 1.0)
        time.sleep(0.2)
        proc.kill()
        killed = time.monotonic()
        while time.monotonic() - killed < 2.0:
            if not bot.get_wheel_state().get("usable") and bot._cmd == (0.0, 0.0):
                break
            time.sleep(0.01)
        waited = time.monotonic() - killed
        assert bot._cmd == (0.0, 0.0), bot._cmd
        assert waited <= 0.35 + 0.1, waited     # stale_after 0.25 + one check
        bot.close()
    finally:
        _stop(proc)


def test_what_crosses_the_boundary_is_exactly_what_the_body_measured():
    """Criterion 2's safety half. The sweep (3.18) places an in-process body
    thousands of times, so it cannot run across a process; what DOES cross
    is each reading's JSON. At the sweep's own near-obstacle starts in every
    house, every heading, the scan and the depth grid come back from the
    server's encoder equal to what the body measured -- so the vet, fed
    either, decides the same. The live stopping tests in G4 cover the wire."""
    import math
    from fastapi.responses import JSONResponse
    import json
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
