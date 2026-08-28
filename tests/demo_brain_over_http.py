"""
demo_brain_over_http.py

Stage 2 demo: the same mission, run three ways -- in-process, through
RemoteRobot over a real socket, and through the brain service driving
that same socket. All three should agree on every action.

That agreement is the whole claim of PLAN-brain-relocation.md: where the
brain runs is a base URL, not an architecture. This script is the manual
version of tests/test_remote_robot.py and tests/test_brain_server.py --
run it when you want to watch it happen rather than assert it.

Run with: python -m tests.demo_brain_over_http
"""

import socket
import subprocess
import sys
import time
from pathlib import Path

import httpx
from fastapi.testclient import TestClient

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from control.brain_server import create_app
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot

REPO_ROOT = Path(__file__).resolve().parent.parent
TARGET = "red backpack"
BUDGET = 150


def fresh_robot():
    return MockRobot(build_starter_world())


def free_port():
    """Not :8000 -- this demo is most useful while a robot server and brain
    are already running, and colliding with them is a confusing failure."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_robot_server(port=None):
    port = port or free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "robot.server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT,
    )
    url = f"http://127.0.0.1:{port}"
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                return proc, url
        except httpx.HTTPError:
            time.sleep(0.1)
    proc.terminate()
    raise RuntimeError("robot server never came up")


def run_in_process():
    memory = MissionMemory(mission=f"Find the {TARGET}.", target_object=TARGET)
    agent = ObjectSearchAgent(fresh_robot(), memory, min_distance_cm=30)
    report = agent.run_mission(max_steps=BUDGET)
    return [r.action for r in agent.history], report["steps_taken"], report["found"]


def run_over_http(robot_url):
    with RemoteRobot(robot_url) as robot:
        runner = MissionRunner(robot, target_object=TARGET, max_steps=BUDGET)
        runner.start()
        while runner.tick():
            pass
        status = runner.status()
    return [r.action for r in runner.agent.history], status["step"], status["found"]


def run_through_the_brain_service(robot_url):
    app = create_app(robot_factory=lambda: RemoteRobot(robot_url))
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": TARGET, "max_steps": BUDGET})
        while True:
            status = client.get("/mission/status").json()
            if not status["running"]:
                break
            time.sleep(0.05)
    return status


def main():
    print("=== the same mission, three ways ===\n")

    local_actions, local_steps, local_found = run_in_process()
    print(f"in-process             : {local_steps} steps, found={local_found}")

    proc, url = start_robot_server()
    try:
        remote_actions, remote_steps, remote_found = run_over_http(url)
        print(f"RemoteRobot over :8000 : {remote_steps} steps, found={remote_found}")

        proc.terminate()
        proc.wait(timeout=10)
        proc, url = start_robot_server()  # fresh world for the third run

        status = run_through_the_brain_service(url)
        print(f"brain service          : {status['step']} steps, found={status['found']}")
    finally:
        proc.terminate()
        proc.wait(timeout=10)

    print()
    print(f"identical action sequence in-process vs HTTP: {local_actions == remote_actions}")
    print(f"same step count through the brain service   : {status['step'] == local_steps}")
    print(f"\nlast reasoning: {status['last_reasoning']}")
    print("log tail:")
    for line in status["log_tail"][-5:]:
        print(f"  {line}")


if __name__ == "__main__":
    main()
