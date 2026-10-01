"""
tests/test_urdf.py

Phase R3 (`PLAN-ros-alignment.md` 3.12) -- the robot's URDF and TF tree.

Two kinds of test, on purpose:

* **Always run:** the chassis constants are the SAME number in the URDF, in
  `diff_drive_controller`'s config and in `sim/mock_robot.py`. Three copies of
  a wheel radius is how a sim and a controller come to disagree about a
  wheel; this is the test that stops it. Pure text -- no ROS on the laptop.
* **Run against a live container** (`service/slam/`, `ros2 launch
  picar_bringup picar.launch.py`, bridge on :8090): what
  `robot_state_publisher` actually publishes, looked up with tf2, against a
  pure-Python evaluation of the same URDF. They SKIP when no bridge answers,
  the way the Playwright tests skip without a browser.

Python never imports ROS here (`tests/test_ros_containment.py`): it reads
XML and speaks HTTP to the bridge.
"""

import json
import math
import os
import re
import shutil
import subprocess
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from sim.mock_robot import TRACK_WIDTH_M, WHEEL_RADIUS_M

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "service/slam/src/picar_description/urdf/picar.urdf.xacro"
CONTROLLERS = ROOT / "service/slam/src/picar_bringup/config/controllers.yaml"
BRIDGE = os.environ.get("PICAR_BRIDGE_URL", "http://127.0.0.1:8090")
IMAGE = os.environ.get("PICAR_ROS_IMAGE", "vision-picar-ros")
PAN_DEG = (-90, -45, 0, 45, 90)


def _xacro_property(name):
    m = re.search(rf'<xacro:property name="{name}"\s+value="([^"]+)"', XACRO.read_text())
    return float(m.group(1))


def _yaml_number(key):
    m = re.search(rf"^\s*{key}:\s*([0-9.]+)\s*$", CONTROLLERS.read_text(), re.M)
    return float(m.group(1))


# ---------- criterion 2: one chassis, three files ----------

def test_wheel_radius_is_one_number():
    assert _xacro_property("wheel_radius") == WHEEL_RADIUS_M
    assert _yaml_number("wheel_radius") == WHEEL_RADIUS_M


def test_wheel_separation_is_one_number():
    assert _xacro_property("wheel_separation") == TRACK_WIDTH_M
    assert _yaml_number("wheel_separation") == TRACK_WIDTH_M


def test_the_xacro_is_well_formed_xml():
    """A `--` inside an XML comment is not well-formed, and nothing offline
    noticed: the file is read by regex above, so the first thing to parse it
    was xacro inside the container, which then refused to launch at all
    (3.21, a comment reading "skid steer -- modelled as ...")."""
    ET.fromstring(XACRO.read_text())


# ---------- the live half ----------

def _bridge(path, body=None):
    secret = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    req = urllib.request.Request(BRIDGE + path, data=None if body is None else json.dumps(body).encode(),
                                 method="GET" if body is None else "POST")
    req.add_header("content-type", "application/json")
    if secret:
        req.add_header("x-app-secret", secret)
    with urllib.request.urlopen(req, timeout=2) as r:
        return json.loads(r.read())


@pytest.fixture(scope="module")
def live():
    try:
        _bridge("/health")
    except Exception:  # noqa: BLE001
        pytest.skip(f"no ROS bridge at {BRIDGE} -- start service/slam's container to run R3's TF checks")
    try:
        # /health is open; the routes below are not. A bridge that answers
        # but will not let us in is a bridge this run cannot measure.
        _bridge("/tf?target=base_link&source=laser")
    except urllib.error.HTTPError as e:
        if e.code == 401:
            pytest.skip("the ROS bridge wants a secret -- set APP_SHARED_SECRET (or LOCAL_SECRET)")
        raise
    if not shutil.which("docker"):
        pytest.skip("docker is needed to expand the xacro")
    urdf = subprocess.run(
        ["docker", "run", "--rm", IMAGE, "bash", "-c",
         "xacro /ws/install/picar_description/share/picar_description/urdf/picar.urdf.xacro"],
        capture_output=True, text=True, check=True).stdout
    yield ET.fromstring(urdf)
    _bridge("/pan", {"angle_rad": 0.0})


@pytest.fixture(scope="module")
def urdf():
    """The REPO's xacro, expanded in the ROS image -- not the copy installed
    in it, which is only as new as the last image build. No bridge needed:
    criterion 4 is geometry."""
    if not shutil.which("docker"):
        pytest.skip("docker is needed to expand the xacro")
    src = XACRO.parent.parent
    run = subprocess.run(
        ["docker", "run", "--rm", "-v", f"{src}:/pd:ro", IMAGE, "bash", "-c",
         "source /opt/ros/humble/setup.bash && xacro /pd/urdf/picar.urdf.xacro"],
        capture_output=True, text=True)
    if run.returncode != 0:
        pytest.skip(f"could not expand the xacro in {IMAGE}: {run.stderr[-300:]}")
    return ET.fromstring(run.stdout)


def _rot(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_rot(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def fk(urdf, source, target="base_link", joints=None):
    """4x4 pose of `source` in `target` by walking the URDF's joints -- what
    robot_state_publisher does, in twenty lines of numpy."""
    joints = joints or {}
    by_child = {j.find("child").get("link"): j for j in urdf.findall("joint")}
    T = np.eye(4)
    link = source
    while link != target:
        j = by_child[link]
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        step = np.eye(4)
        step[:3, :3] = _rot(rpy)
        step[:3, 3] = xyz
        if j.get("type") in ("revolute", "continuous"):
            m = np.eye(4)
            m[:3, :3] = _axis_rot([float(v) for v in j.find("axis").get("xyz").split()],
                                  joints.get(j.get("name"), 0.0))
            step = step @ m
        T = step @ T
        link = j.find("parent").get("link")
    return T


def _ypr(R):
    yaw = math.atan2(R[1, 0], R[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    return yaw, pitch, roll


def _published(source, pan_rad):
    _bridge("/pan", {"angle_rad": pan_rad})
    # robot_state_publisher republishes at its own rate; wait for the new angle.
    import time
    deadline = time.time() + 3
    while time.time() < deadline:
        tf = _bridge(f"/tf?target=base_link&source={source}")
        if source != "camera_link" or abs(tf["yaw_rad"] - pan_rad) < 0.01:
            return tf
        time.sleep(0.1)
    return tf


# ---------- criterion 3: TF is what the URDF says ----------

@pytest.mark.parametrize("pan_deg", PAN_DEG)
@pytest.mark.parametrize("frame", ["laser", "camera_link", "front_bumper"])
def test_published_tf_matches_the_urdf(live, frame, pan_deg):
    pan = math.radians(pan_deg)
    tf = _published(frame, pan)
    T = fk(live, frame, joints={"pan_joint": pan})
    assert np.allclose(tf["translation_m"], T[:3, 3], atol=0.001), (tf, T[:3, 3])
    for got, want in zip((tf["yaw_rad"], tf["pitch_rad"], tf["roll_rad"]), _ypr(T[:3, :3])):
        assert abs(math.degrees(got - want)) < 0.1


# ---------- criterion 4: how wrong is the hand shortcut ----------

def shortcut_error_deg(urdf, pan_deg, target_body_deg, range_m, frame="camera_link"):
    """The project's shortcut composes a panned bearing as pan + in-frame
    azimuth, the azimuth measured at the LENS (`camera_link`); the truth is
    the bearing from base_link. Both in the project's clockwise-positive
    convention. (3.12 measured at `pan_link`, which was the lens's x/y while
    the lens sat on the pan axis; 3.27's CAD puts it 4.8 cm ahead of it.)"""
    pan = -math.radians(pan_deg)                       # ROS joint: CCW-positive
    T = fk(urdf, frame, joints={"pan_joint": pan})
    b = -math.radians(target_body_deg)                 # CCW angle in base_link
    p_body = np.array([range_m * math.cos(b), range_m * math.sin(b), T[2, 3], 1.0])
    # Azimuth in the lens's own horizontal plane: the pan yaw only, so the
    # camera's downward pitch does not leak into a horizontal bearing.
    yaw = math.atan2(T[1, 0], T[0, 0])
    dx, dy = p_body[0] - T[0, 3], p_body[1] - T[1, 3]
    in_frame_cw = -math.degrees(math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                                           math.cos(yaw) * dx + math.sin(yaw) * dy))
    shortcut = pan_deg + in_frame_cw
    return abs((shortcut - target_body_deg + 180) % 360 - 180)


def test_the_hand_shortcut_is_inside_the_steering_band_beyond_a_metre(urdf):
    """R3 criterion 4. FAILED on 3.12's placeholder (pan axis 8 cm ahead:
    4.59 deg at 1 m); PASSES on the Rover's CAD (3.27: axis 0.9 cm behind,
    lens 4.8 cm ahead of it): worst 1.89 deg at 1 m, 5.01 at 0.4 m."""
    worst = {}
    for pan_deg in range(-90, 91, 15):
        for off in range(-30, 31, 5):                  # targets within the view
            for range_m in (0.4, 0.7, 1.0, 1.5, 2.0, 3.0):
                e = shortcut_error_deg(urdf, pan_deg, pan_deg + off, range_m)
                worst[range_m] = max(worst.get(range_m, 0.0), e)
    print("\nshortcut error, worst over pan and target bearing:",
          {r: round(e, 2) for r, e in sorted(worst.items())})
    assert all(e < 3.0 for r, e in worst.items() if r >= 1.0), worst


def test_a_centred_target_is_safe_to_read_with_the_shortcut(urdf):
    """What the tier and brain/arrival.py actually rely on: camera centred,
    target inside the 3-degree steering band. Measured worst 0.75 degree at
    0.4 m on 3.12's placeholder, 0.32 on the CAD (3.27) -- well inside the
    band, so the final approach is sound."""
    worst = max(shortcut_error_deg(urdf, 0, off, r)
                for off in range(-3, 4) for r in (0.4, 0.7, 1.0, 2.0, 3.0))
    assert worst < 1.0, worst
