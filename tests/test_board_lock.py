"""3.93 criteria 1-5: the board lock, with the board stood in for by a local
shell (PICAR_BOARD_SSH="sh -c") and the lock in a temporary directory. The
holder is the same code that runs on the board. On SLOW_FILES: real waits
for the silence limit and the maximum (~20 s)."""

import fcntl
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "jetson" / "board_lock.py"
CHECK = ROOT / "tools" / "jetson" / "board_lock_check.sh"

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="fcntl")


@pytest.fixture
def env(tmp_path):
    e = {**os.environ, "PICAR_BOARD_SSH": "sh -c", "PICAR_BOARD_LOCK_DIR": str(tmp_path),
         "PICAR_BOARD_PYTHON": sys.executable, "PICAR_BOARD_WHO": "test-session",
         "PICAR_BOARD_STATE": str(tmp_path / "hold.json")}
    e.pop("PICAR_BOARD_TOKEN", None)
    yield e
    subprocess.run([sys.executable, TOOL, "release"], env=e, capture_output=True, timeout=30)


def tool(env, *args, timeout=30):
    return subprocess.run([sys.executable, TOOL, *args], env=env, capture_output=True,
                          text=True, timeout=timeout)


def owner(env) -> str:
    try:
        return (Path(env["PICAR_BOARD_LOCK_DIR"]) / ".picar-board.owner").read_text().strip()
    except OSError:
        return ""


def lock_free(env) -> bool:
    with open(Path(env["PICAR_BOARD_LOCK_DIR"]) / ".picar-board.lock", "a") as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(f, fcntl.LOCK_UN)
        return True


def start_run(env, *args, cmd=("sleep", "60")):
    """A `run` in the background; returns once the board side says it holds."""
    p = subprocess.Popen([sys.executable, TOOL, "run", *args, "--", *cmd], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    end = time.monotonic() + 15
    while time.monotonic() < end:
        if "token " in owner(env) and not lock_free(env):
            return p
        time.sleep(0.1)
    p.kill()
    raise AssertionError("the run never held the lock")


def wait_free(env, within: float) -> float:
    t0 = time.monotonic()
    while time.monotonic() - t0 < within:
        if lock_free(env):
            return time.monotonic() - t0
        time.sleep(0.1)
    raise AssertionError(f"lock still held after {within} s")


def stop(p):
    if p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            p.kill()


def test_criterion_1_exclusion(env):
    a = start_run(env, "--why", "first")
    try:
        line = owner(env)
        assert line.startswith("test-session (") and ": first, since " in line
        b = tool(env, "run", "--", "true")
        assert b.returncode == 75 and line in b.stderr
        h = tool(env, "hold", "--why", "second")
        assert h.returncode == 75 and line in h.stderr
        assert not lock_free(env)               # a by-hand flock -n fails too
        s = tool(env, "status")
        assert s.returncode == 75 and line in s.stdout
    finally:
        stop(a)
    wait_free(env, 5)
    assert tool(env, "status").stdout.strip() == "FREE"
    assert owner(env) == ""                     # the holder clears its own line


def test_run_passes_the_token_and_exit_code(env, tmp_path):
    out = tmp_path / "token.txt"
    r = tool(env, "run", "--", "sh", "-c", f'echo "$PICAR_BOARD_TOKEN" > {out}; exit 3')
    assert r.returncode == 3
    assert len(out.read_text().strip()) == 16
    assert lock_free(env)


def test_criterion_1_waits_for_a_busy_board(env):
    a = start_run(env, cmd=("sleep", "2"))
    t0 = time.monotonic()
    b = tool(env, "run", "--wait", "20", "--", "true")
    assert b.returncode == 0 and "waiting" in b.stderr
    assert time.monotonic() - t0 < 15
    stop(a)


def test_criterion_2_release_on_sigkill(env):
    a = start_run(env)
    os.kill(a.pid, signal.SIGKILL)
    a.wait()
    assert wait_free(env, 5) < 5


def test_criterion_3_release_on_silence(env):
    e = {**env, "PICAR_BOARD_HEARTBEAT_S": "0", "PICAR_BOARD_SILENCE_S": "3"}
    a = start_run(e)
    try:
        assert wait_free(e, 3 + 5) >= 2.5      # not before the limit
        assert a.poll() is None                 # stdin still open: it was the silence
    finally:
        stop(a)


def test_heartbeat_keeps_a_hold_past_the_silence_limit(env):
    e = {**env, "PICAR_BOARD_HEARTBEAT_S": "0.5", "PICAR_BOARD_SILENCE_S": "2"}
    a = start_run(e)
    try:
        time.sleep(4)
        assert not lock_free(e)
    finally:
        stop(a)


def test_criterion_4_release_at_the_maximum(env):
    a = start_run(env, "--max", "2")
    try:
        assert wait_free(env, 7) < 7
    finally:
        stop(a)


def test_hold_env_release(env):
    h = tool(env, "hold", "--why", "multi-step")
    assert h.returncode == 0, h.stderr
    assert not lock_free(env) and ": multi-step, since " in owner(env)
    e = tool(env, "env")
    token = e.stdout.strip().removeprefix("export PICAR_BOARD_TOKEN=")
    assert e.returncode == 0 and owner(env).endswith(f"token {token}")
    assert tool(env, "hold").returncode == 0     # already held by this worktree: no second holder
    assert tool(env, "release").returncode == 0
    wait_free(env, 5)
    assert tool(env, "env").returncode == 1


def check(env, token=None):
    e = {**env}
    if token is not None:
        e["PICAR_BOARD_TOKEN"] = token
    return subprocess.run(["bash", "-c", f'source "{CHECK}"; echo ran'], env=e,
                          capture_output=True, text=True, timeout=30)


def test_criterion_5_the_check(env):
    free = check(env)
    assert free.returncode == 0 and "ran" in free.stdout and "free" in free.stderr
    assert tool(env, "hold").returncode == 0
    token = tool(env, "env").stdout.strip().split("=", 1)[1]
    assert check(env, token).returncode == 0 and "ran" in check(env, token).stdout
    for bad in (None, "0123456789abcdef"):
        r = check(env, bad)
        assert r.returncode == 75 and "ran" not in r.stdout and "test-session" in r.stderr


def test_criterion_5_a_hold_taken_by_hand(env):
    """The other session's way: flock on the file, a line with no token."""
    d = Path(env["PICAR_BOARD_LOCK_DIR"])
    (d / ".picar-board.owner").write_text("session_X (Mac): by hand, since now\n")
    hand = subprocess.Popen([sys.executable, "-c",
                             "import fcntl,sys,time;f=open(sys.argv[1],'a');"
                             "fcntl.flock(f,fcntl.LOCK_EX);print('x',flush=True);time.sleep(60)",
                             str(d / ".picar-board.lock")], stdout=subprocess.PIPE, text=True)
    try:
        hand.stdout.readline()
        r = check(env, "anything")
        assert r.returncode == 75 and "by hand" in r.stderr
        b = tool(env, "run", "--", "true")
        assert b.returncode == 75 and "session_X (Mac): by hand" in b.stderr
    finally:
        hand.kill()
