"""
tests/test_http_rate_live.py

Does HTTP hold the 20 Hz control loop? Measured 2026-09-27 and pinned.

picar_sim_hardware talks to robot/server.py over HTTP at controllers.yaml's
20 Hz: a GET and a POST per cycle on one kept-alive libcurl connection.
tests/http_rate_bench.py does the same from inside the container, over the
same Docker hop. The criteria, written before this test was run:

  1. the plugin really runs at 20 Hz (19-21 POST /wheels a second);
  2. no request fails;
  3. no control cycle takes longer than 125 ms -- HALF of
     diff_drive_controller's cmd_vel_timeout (0.25 s), the silence that
     would actually stop the wheels. A late cycle is harmless; a gap near
     the timeout is a stop the robot did not ask for.

What the measurement found (PLAN-ros-alignment.md 3.17): HTTP itself is not
the limit -- a bare FastAPI app over the same hop held 200 Hz with p99
1.6-4 ms. The robot server's tail (p99 ~20-34 ms, max ~64 ms) is the
SIMULATOR's work in the same Python process (a 360-beam scan is ~13 ms of
ray casting, a camera frame ~35 ms), which the car will not have.

Skips without the container and a secret.
"""

import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import httpx
import pytest

ROBOT = os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000")
CONTAINER = os.environ.get("PICAR_ROS_CONTAINER", "picar-ros")
BENCH = Path(__file__).resolve().parent / "http_rate_bench.py"


def _secret():
    return os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET") or ""


@pytest.fixture(scope="module")
def stack():
    if not shutil.which("docker") or not _secret():
        pytest.skip("needs docker and the local secret")
    try:
        health = httpx.get(ROBOT + "/health", timeout=2).json()
    except Exception:  # noqa: BLE001
        pytest.skip("no robot server")
    if (health.get("drive") or {}).get("mode") != "ros":
        pytest.skip("the robot server is not driving through ROS (ROBOT_DRIVE=ros)")
    ip = subprocess.run(["docker", "exec", CONTAINER, "getent", "ahostsv4", "host.docker.internal"],
                        capture_output=True, text=True).stdout.split()
    if not ip:
        pytest.skip("no container")
    subprocess.run(["docker", "cp", str(BENCH), f"{CONTAINER}:/tmp/http_rate_bench.py"], check=True)
    return ip[0]


def test_the_plugin_really_runs_at_twenty_hertz(stack):
    def posts():
        return httpx.get(ROBOT + "/health", timeout=2).json()["drive"]["wheel_posts_from_ros"]
    a = posts()
    time.sleep(5.0)
    rate = (posts() - a) / 5.0
    assert 19.0 <= rate <= 21.0, rate


def test_http_holds_the_twenty_hertz_loop(stack):
    out = subprocess.run(
        ["docker", "exec", "-e", f"HOST={stack}", "-e", f"APP_SHARED_SECRET={_secret()}",
         CONTAINER, "python3", "/tmp/http_rate_bench.py", "20", "15"],
        capture_output=True, text=True, timeout=60).stdout
    r = json.loads(out.strip().splitlines()[-1])
    assert r["errors"] == 0, r
    assert r["cycle_max_ms"] < 125.0, r
