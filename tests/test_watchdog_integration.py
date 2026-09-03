"""
tests/test_watchdog_integration.py

Phase S4 (PLAN-sim-hardening.md) -- the watchdog's real async polling
loop, proven against a live server with a real event loop and real
wall-clock time. `tests/test_server.py` already covers
`watchdog_should_stop()`'s pure decision logic; before this file, that
plan's section 3.1 finding -- "the watchdog's async loop is never executed
by any test" -- was literally true, because nothing gave an action any
wall-clock duration to be silent *during*.

Uses a live `uvicorn robot.server:app` subprocess (not the ASGI
in-process transport `tests/conftest.py`'s `robot_over_asgi` fixture uses
elsewhere -- that transport doesn't run the app's background asyncio
tasks the way a real process does) pointed, via the ROBOT_CONFIG_PATH env
var (see robot/server.py's own docstring), at a temp config with a short
watchdog_timeout_s and sim.realtime: true -- see sim/mock_robot.py's
_settle() -- so this doesn't need to wait a full production-sized second
per assertion.

Run with: pytest tests/test_watchdog_integration.py -v
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import httpx
import pytest
import yaml

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

WATCHDOG_TIMEOUT_S = 0.3


def _write_temp_config(tmp_path: Path) -> Path:
    base = yaml.safe_load((REPO_ROOT / "config" / "robot.yaml").read_text())
    base["safety"]["watchdog_timeout_s"] = WATCHDOG_TIMEOUT_S
    base.setdefault("sim", {})["realtime"] = True
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(base))
    return path


@pytest.fixture
def live_server(tmp_path):
    """A live uvicorn robot.server:app, pointed at a temp config with a
    short watchdog_timeout_s and sim.realtime: true. Yields its base URL."""
    config_path = _write_temp_config(tmp_path)
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    env = {**os.environ, "ROBOT_CONFIG_PATH": str(config_path)}
    proc = subprocess.Popen(
        [
            sys.executable, "-m", "uvicorn", "robot.server:app",
            "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning",
        ],
        cwd=REPO_ROOT, env=env,
    )
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f"robot server exited early with code {proc.returncode}")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail(f"robot server was not healthy within {SERVER_START_TIMEOUT_S}s")
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def test_the_config_override_actually_took(live_server):
    """Sanity check on the fixture itself, before trusting the timing
    assertions below: the live server really is running the temp config,
    not silently falling back to the repo's real one."""
    health = httpx.get(f"{live_server}/health").json()
    assert health["watchdog_timeout_s"] == WATCHDOG_TIMEOUT_S


def test_the_watchdog_measures_real_silence(live_server):
    """The thing PLAN-sim-hardening.md 3.1 said no test exercised: real
    wall-clock time passing with no command arriving, observed through a
    real event loop on a real process."""
    httpx.post(f"{live_server}/action", json={"action": "LOOK_CENTER"})

    time.sleep(WATCHDOG_TIMEOUT_S + 0.3)

    health = httpx.get(f"{live_server}/health").json()
    assert health["seconds_since_last_command"] > WATCHDOG_TIMEOUT_S


def test_commands_faster_than_the_timeout_keep_it_quiet(live_server):
    for _ in range(6):
        httpx.post(f"{live_server}/action", json={"action": "LOOK_CENTER"})
        time.sleep(WATCHDOG_TIMEOUT_S / 4)

    health = httpx.get(f"{live_server}/health").json()
    assert health["seconds_since_last_command"] < WATCHDOG_TIMEOUT_S


def test_a_move_actually_occupies_its_duration_under_realtime(live_server):
    """sim.realtime: true (Phase S4) is what makes the watchdog readout
    worth watching at all in the twin -- a move now takes real time, not
    just the gap between moves."""
    duration = 0.4
    start = time.monotonic()
    resp = httpx.post(
        f"{live_server}/action",
        json={"action": "FORWARD", "duration": duration},
        timeout=5.0,
    )
    elapsed = time.monotonic() - start

    assert resp.status_code == 200
    assert resp.json()["executed"] is True
    assert elapsed >= duration * 0.8


# ---------- the loop's own liveness (phase M5) ----------


def test_the_watchdog_loop_reports_that_it_is_still_running(live_server):
    """Phase M5's second verdict input, and the reason it needs a live
    server rather than a stub.

    `seconds_since_last_command` measures the CLIENT's silence; this
    measures whether the guard that acts on that silence is itself alive.
    Nothing noticed before if the asyncio task died: the server kept
    answering every request, `/health` kept reporting a growing silence,
    and the thing that was supposed to stop the motors was gone.

    Stop updating `watchdog_polled_at` inside the loop and this goes red --
    which is exactly what happened when it was checked: the stubbed tests
    in `tests/test_health.py` all passed against a server that had stopped
    polling, because a stub cannot tell a seeded value from a live one.
    """
    interval = httpx.get(f"{live_server}/health").json()["watchdog_poll_interval_s"]
    assert interval > 0

    # Longer than several poll intervals, so a value that is merely seeded
    # at start-up has had time to go stale.
    time.sleep(max(0.5, interval * 10))

    age = httpx.get(f"{live_server}/health").json()["seconds_since_watchdog_poll"]
    assert age <= interval * 5, (
        f"the watchdog loop last polled {age}s ago (it wakes every {interval}s) -- "
        "either the loop is not running or it is no longer recording that it is"
    )


def test_the_health_command_agrees_with_a_live_server(live_server):
    """`python -m control.health`'s robot half, against a real process
    rather than a stub -- the one thing the stubbed suite cannot check is
    that the fields it reads are the fields the server actually sends."""
    from control import health

    report = health.check_robot(live_server)
    assert report["status"] == health.OK, report["problems"]
    assert report["identity"]["git_revision"]
    assert report["description"]["watchdog_poll_age_s"] is not None
