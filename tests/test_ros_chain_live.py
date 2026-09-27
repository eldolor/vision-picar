"""
tests/test_ros_chain_live.py

Phase R4 (`PLAN-ros-alignment.md` 3.13) -- the ROS chain, measured live.

    /action -> robot/ros_drive.py -> bridge -> twist_mux -> diff_drive_controller
            -> picar_sim_hardware -> POST /wheels -> the sim

One test per acceptance criterion, against a real stack: the robot server in
`drive: ros` (`ROBOT_DRIVE=ros bash service/tunnel/restart.sh`), the brain
beside it, and `service/slam/`'s container (`ros2 launch picar_bringup
picar.launch.py`, bridge on :8090). Everything SKIPS when that stack is not
up, the way the Playwright tests skip without a browser -- and the always-run
half of R4 is `tests/test_ros_drive.py`, the same verb executor against a
fake chain in-process.

Positions come from `/world/truth` (sim-only, read by the TEST), so the
tests do not depend on where a previous one left the robot.
"""

import math
import os
import subprocess
import time

import httpx
import pytest

ROBOT = os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000")
BRAIN = os.environ.get("PICAR_BRAIN_URL", "http://127.0.0.1:8001/brain")
BRIDGE = os.environ.get("PICAR_BRIDGE_URL", "http://127.0.0.1:8090")
CONTAINER = os.environ.get("PICAR_ROS_CONTAINER", "picar-ros")


def _client(url):
    secret = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    return httpx.Client(base_url=url, headers={"x-app-secret": secret} if secret else {},
                        timeout=30)


@pytest.fixture(scope="module")
def stack():
    try:
        robot, bridge = _client(ROBOT), _client(BRIDGE)
        drive = robot.get("/health").json().get("drive") or {}
        bridge.get("/health").raise_for_status()
    except Exception:  # noqa: BLE001
        pytest.skip("no live R4 stack (robot server + ROS container) -- see this module's docstring")
    if drive.get("mode") != "ros":
        pytest.skip("the robot server is not in drive: ros (ROBOT_DRIVE=ros)")
    return robot, bridge


def _truth(robot):
    return robot.get("/world/truth").json()


def _act(robot, action, driver="twin-dpad", **kw):
    return robot.post("/action", json={"action": action, **kw}, headers={"x-driver": driver}).json()


def _face(robot, heading_deg):
    """Turn to a compass heading (0 = north, clockwise), through the chain."""
    err = (heading_deg - _truth(robot)["heading_deg"] + 180) % 360 - 180
    if abs(err) > 1:
        _act(robot, "RIGHT" if err > 0 else "LEFT", angle=int(round(abs(err))))
    time.sleep(1.2)  # let the D-pad's authority lapse for whoever drives next


def _clearance(robot):
    return robot.get("/depth").json()["path"]["clearance_cm"]


# ---------- 1: the chain moves the robot, and /odom tells the truth ----------

def test_a_twist_reaches_the_wheels_intact_and_odom_agrees_with_truth(stack):
    robot, bridge = stack
    _face(robot, 90)                    # east: a clear run from the start room
    t0, o0 = _truth(robot), bridge.get("/odom").json()
    seen = []
    end = time.time() + 1.0
    while time.time() < end:
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.1, "angular_rad_s": 0})
        seen.append(robot.get("/wheels").json()["left"]["velocity_rad_s"])
        time.sleep(0.05)
    bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0, "angular_rad_s": 0})
    time.sleep(0.5)
    t1, o1 = _truth(robot), bridge.get("/odom").json()
    moving = [v for v in seen if v]
    assert moving and all(abs(v - 0.1 / 0.0325) < 0.01 * 0.1 / 0.0325 for v in moving), moving
    truth_m = math.dist((t0["x_m"], t0["y_m"]), (t1["x_m"], t1["y_m"]))
    odom_m = math.dist((o0["x_m"], o0["y_m"]), (o1["x_m"], o1["y_m"]))
    assert truth_m > 0.05
    assert abs(truth_m - odom_m) < 0.02, (truth_m, odom_m)


# ---------- 2 + 3: verbs through the chain, and only through it ----------

@pytest.mark.parametrize("action,kw,metres,degrees", [
    ("FORWARD", {}, 0.30, 0.0),
    ("LEFT", {"angle": 45}, 0.0, -45.0),
    ("RIGHT", {"angle": 90}, 0.0, 90.0),
    ("LEFT", {"angle": 45}, 0.0, -45.0),
    ("REVERSE", {}, 0.30, 0.0),
])
def test_a_verb_through_ros_means_what_it_meant(stack, action, kw, metres, degrees):
    robot, _ = stack
    before = robot.get("/health").json()["drive"]["verbs_through_ros"]
    t0 = _truth(robot)
    reply = _act(robot, action, **kw)
    t1 = _truth(robot)
    assert reply["executed"] is True, reply
    assert reply["result"]["via"] == "ros", "the verb must go through the chain"
    assert robot.get("/health").json()["drive"]["verbs_through_ros"] == before + 1
    moved = math.dist((t0["x_m"], t0["y_m"]), (t1["x_m"], t1["y_m"]))
    turned = (t1["heading_deg"] - t0["heading_deg"] + 180) % 360 - 180
    assert abs(moved - metres) <= 0.02, moved
    assert abs(turned - degrees) <= 2.0, turned


def test_only_ros_may_write_the_wheels(stack):
    robot, _ = stack
    reply = robot.post("/wheels", json={"left_rad_s": 1.0, "right_rad_s": 1.0},
                       headers={"x-driver": "twin-dpad"}).json()
    assert reply["executed"] is False and reply["reason"] == "not_the_actuator"


# ---------- 4: authority, both layers ----------

def test_teleop_outranks_brain_inside_ros(stack):
    robot, bridge = stack
    t0 = _truth(robot)
    end = time.time() + 1.0
    while time.time() < end:
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.1, "angular_rad_s": 0})
        bridge.post("/cmd_vel", json={"driver": "twin-dpad", "linear_m_s": 0, "angular_rad_s": 1.0})
        time.sleep(0.05)
    for d in ("brain", "twin-dpad"):
        bridge.post("/cmd_vel", json={"driver": d, "linear_m_s": 0, "angular_rad_s": 0})
    time.sleep(0.5)
    t1 = _truth(robot)
    assert math.dist((t0["x_m"], t0["y_m"]), (t1["x_m"], t1["y_m"])) < 0.01, "brain's forward leaked"
    assert abs((t1["heading_deg"] - t0["heading_deg"] + 180) % 360 - 180) > 20, "teleop's turn lost"


def test_a_dpad_tap_still_preempts_a_mission_under_drive_ros(stack):
    robot, _ = stack
    brain = _client(BRAIN)
    try:
        brain.get("/health").raise_for_status()
    except Exception:  # noqa: BLE001
        pytest.skip("no brain server")
    _face(robot, 90)                    # east, toward the doorway: open floor
    started = brain.post("/mission/start", json={"policy": "frontier", "target_object": "red backpack",
                                                  "max_steps": 40})
    assert started.status_code == 200, started.text
    deadline = time.time() + 15         # the first move, through ROS, then grab it
    while time.time() < deadline and brain.get("/mission/status").json()["step"] < 1:
        time.sleep(0.1)
    _act(robot, "LEFT", angle=15)
    deadline = time.time() + 15
    while time.time() < deadline:
        status = brain.get("/mission/status").json()
        if status["outcome"] != "running":
            break
        time.sleep(0.3)
    assert status["outcome"] == "preempted", status["outcome"]
    assert any("twin-dpad" in line for line in status["log_tail"]), status["log_tail"][-3:]
    health = robot.get("/health").json()
    assert health["driver"] == "twin-dpad"
    time.sleep(health["watchdog_timeout_s"] + 0.3)
    health = robot.get("/health").json()
    assert health["driver"] == "twin-dpad" and health["authority_holder"] is None, \
        "authority must lapse on silence"


# ---------- 5: the safety vet still stands in the path ----------

def test_a_standing_twist_into_a_wall_stops_short(stack):
    robot, bridge = stack
    _face(robot, 0)                     # north: the start room's wall
    start = _clearance(robot)
    end = time.time() + 6.0
    while time.time() < end and (_clearance(robot) or 999) > 5:
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.1, "angular_rad_s": 0})
        time.sleep(0.05)
    bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0, "angular_rad_s": 0})
    time.sleep(0.4)
    final = _clearance(robot)
    assert final < start, "it must actually have driven"
    assert final >= 19.4, f"drove to {final} cm through ROS"
    _act(robot, "REVERSE")              # leave room for the next test


# ---------- 6: silence stops it, in ROS and without ROS ----------

def _time_to_stop(robot, cut):
    """Run a standing twist, call `cut()`, and time until the wheels read zero."""
    t_cut = cut()
    while time.time() - t_cut < 3.0:
        if robot.get("/wheels").json()["left"]["velocity_rad_s"] == 0:
            return time.time() - t_cut
        time.sleep(0.01)
    return math.inf


def test_silence_inside_ros_stops_the_wheels_within_half_a_second(stack):
    robot, bridge = stack
    _face(robot, 180)                   # south, room to roll
    for _ in range(10):
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0, "angular_rad_s": 0.8})
        time.sleep(0.05)
    waited = _time_to_stop(robot, time.time)
    assert waited <= 0.5, waited


def test_killing_the_container_stops_the_wheels_on_the_robots_own_watchdog(stack):
    robot, bridge = stack
    timeout = robot.get("/health").json()["watchdog_timeout_s"]
    for _ in range(10):
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0, "angular_rad_s": 0.8})
        time.sleep(0.05)

    def kill():
        subprocess.run(["docker", "kill", "-s", "KILL", CONTAINER], check=True, capture_output=True)
        return time.time()
    try:
        waited = _time_to_stop(robot, kill)
    finally:
        subprocess.run(["docker", "start", CONTAINER], capture_output=True)
        # Wait for the chain to be BACK, not a fixed time: the next test
        # measures the scan rate, and a fixed 10 s once read 2 Hz of warm-up.
        deadline = time.time() + 60
        while time.time() < deadline:
            try:
                h = bridge.get("/health").json()
                if h.get("scan_age_s") is not None and h["scan_age_s"] < 0.5:
                    break
            except Exception:  # noqa: BLE001 -- still starting
                pass
            time.sleep(0.5)
        time.sleep(2)
    # Killing freezes the LAST command as a standing one; only the robot
    # server's watchdog can end it. One watchdog period of slack.
    assert waited <= timeout + 0.35, (waited, timeout)


# ---------- 7: the scan crosses the wall intact ----------

def test_the_scan_crosses_beam_for_beam_at_five_hertz(stack):
    robot, bridge = stack
    before = bridge.get("/health").json()["scans_published"]
    time.sleep(2.0)
    rate = (bridge.get("/health").json()["scans_published"] - before) / 2.0
    assert rate >= 5.0, rate
    ours = robot.get("/scan").json()
    theirs = bridge.get("/scan").json()
    n = len(ours["ranges_m"])
    for j, r in enumerate(theirs["ranges_m"]):
        # ROS beam j is CCW angle -180 + j; ours is clockwise from -180.
        i = (-j) % n
        mine = ours["ranges_m"][i]
        assert (r is None) == (mine is None), (j, r, mine)
        if r is not None:
            assert abs(r - mine) <= 0.001, (j, r, mine)
