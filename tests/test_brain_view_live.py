"""
tests/test_brain_view_live.py

Live half of tests/test_brain_view.py: with the ROS container up, the
brain's mission is visible ON ROS TOPICS -- which is what makes it visible
to rosbag, rviz and Foxglove. It reads the brain's own /mission/status and
checks the topics inside the container say the same thing.

It deliberately does NOT start a mission. The first version did, and the
mission drove the robot somewhere tests/test_ros_chain_live.py's geometry
does not expect -- two of that suite's tests failed after it in a full run.
A RUNNING mission was read off the topics live once on 2026-09-27
(PLAN-ros-alignment.md 3.17), and the running case is pinned offline in
tests/test_brain_view.py against a real MissionRunner's status.

Skips without the container, Docker, or a secret -- same rule as the other
live suites.
"""

import json
import os
import shutil
import subprocess
import time

import httpx
import pytest

BRAIN = os.environ.get("PICAR_BRAIN_URL", "http://127.0.0.1:8001/brain")
BRIDGE = os.environ.get("PICAR_BRIDGE_URL", "http://127.0.0.1:8090")
CONTAINER = os.environ.get("PICAR_ROS_CONTAINER", "picar-ros")


def _secret():
    return os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET") or ""


def _in_ros(cmd, timeout=20):
    out = subprocess.run(["docker", "exec", CONTAINER, "/entrypoint.sh", "bash", "-c", cmd],
                         capture_output=True, text=True, timeout=timeout)
    return out.stdout


@pytest.fixture(scope="module")
def brain():
    if not shutil.which("docker") or not _secret():
        pytest.skip("needs docker and the local secret")
    try:
        health = httpx.get(BRIDGE + "/health", timeout=2).json()
        client = httpx.Client(base_url=BRAIN, headers={"x-app-secret": _secret()}, timeout=10)
        if client.get("/mission/status").status_code != 200:
            pytest.skip("brain not answering")
    except Exception:  # noqa: BLE001
        pytest.skip("no ROS container / brain")
    if not health.get("brain_url"):
        pytest.skip("this bridge predates the brain feed, or BRAIN_URL is off")
    # A freshly started container has not polled yet; wait for one answer.
    deadline = time.time() + 10
    while time.time() < deadline and httpx.get(BRIDGE + "/health", timeout=2).json().get("brain_age_s") is None:
        time.sleep(0.5)
    time.sleep(1.0)
    return client


def test_the_bridge_is_polling_the_brain(brain):
    h = httpx.get(BRIDGE + "/health", timeout=2).json()
    assert h["brain_error"] is None and h["brain_age_s"] is not None and h["brain_age_s"] < 2.0


def _topic_status():
    raw = _in_ros("timeout 8 ros2 topic echo --once --field data /brain/status")
    # `--field data` prints the string YAML-quoted; the JSON is inside.
    body = raw.strip().strip("'").replace("''", "'")
    return json.JSONDecoder().raw_decode(body[body.index("{"):])[0]


def test_brain_status_on_ros_is_the_brains_own_status(brain):
    ours = brain.get("/mission/status").json()
    theirs = _topic_status()
    for key in ("outcome", "policy", "mission", "step", "max_steps", "running"):
        assert theirs.get(key) == ours.get(key), (key, theirs.get(key), ours.get(key))


def _brain_diagnostics(tries=10):
    """The first `/diagnostics` message that carries the bridge's entry.

    Not simply the first message: the controller manager publishes on
    `/diagnostics` too ("loop time"), and `echo --once` returns whichever
    publisher speaks first -- which failed this test intermittently in the
    combined live run (3.24, G1). Bounded, so a bridge that never publishes
    still fails."""
    out = ""
    for _ in range(tries):
        out = _in_ros("timeout 8 ros2 topic echo --once /diagnostics")
        if "brain: mission" in out:
            return out
    return out


def test_diagnostics_names_the_brains_state(brain):
    ours = brain.get("/mission/status").json()
    out = _brain_diagnostics()
    assert "brain: mission" in out
    expect = "running, step" if ours.get("running") else (ours.get("outcome") or "idle")
    assert expect in out, (expect, out[:400])


def test_the_caption_is_a_marker_in_base_footprint(brain):
    out = _in_ros("timeout 8 ros2 topic echo --once /brain/markers")
    assert "frame_id: base_footprint" in out and "text: " in out


def test_foxglove_offers_no_way_to_publish():
    """The Foxglove window is read-only by configuration: the node must be
    running with only the connectionGraph capability."""
    if not shutil.which("docker"):
        pytest.skip("needs docker")
    try:
        caps = _in_ros("timeout 8 ros2 param get /foxglove_bridge capabilities")
    except Exception:  # noqa: BLE001
        pytest.skip("no container")
    if not caps.strip():
        pytest.skip("no foxglove_bridge in this container")
    assert "connectionGraph" in caps
    for cap in ("clientPublish", "services", "parameters", "parametersSubscribe"):
        assert cap not in caps, caps
