"""3.93 criteria 1-5: the board lock, with the board stood in for by a local
shell (PICAR_BOARD_SSH="sh -c") and the lock in a temporary directory. The
holder is the same code that runs on the board. On SLOW_FILES: real waits
for the silence limit and the maximum (~70 s)."""

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
    # start_run's own sleeps (a unique length: no other process's), left by
    # the tests that SIGKILL `run`.
    subprocess.run(["pkill", "-f", "^sleep 63.71$"], capture_output=True)


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


def start_run(env, *args, cmd=("sleep", "63.71")):
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


# -- the review fixes (code-review + Thermos, 2026-10-22) --------------------

def test_unreachable_board_is_not_busy(env):
    e = {**env, "PICAR_BOARD_SSH": "false"}
    r = tool(e, "run", "--", "true")
    assert r.returncode == 1 and "unreachable" in r.stderr
    h = tool(e, "hold")
    assert h.returncode == 1 and "unreachable" in h.stderr
    assert tool(e, "status").returncode == 1


def test_sigterm_reaches_the_command_and_the_hold_outlasts_it(env, tmp_path):
    """SIGTERM to `run`: the command gets it, and the board stays held
    through the command's teardown (here 2 s), released only after."""
    pidfile, hits = tmp_path / "child.pid", tmp_path / "hits"
    cmd = ("sh", "-c", f'echo $$ > {pidfile}; trap "echo TERM >> {hits}; sleep 2; exit 0" TERM; '
                       'while :; do sleep 0.1; done')
    a = start_run(env, cmd=cmd)
    end = time.monotonic() + 5
    while not (pidfile.exists() and pidfile.read_text().strip()) and time.monotonic() < end:
        time.sleep(0.05)
    child = int(pidfile.read_text())
    a.send_signal(signal.SIGTERM)
    time.sleep(1.0)
    assert not lock_free(env)                   # held through the teardown
    a.wait(timeout=20)
    assert hits.read_text().count("TERM") == 1
    wait_free(env, 5)
    with pytest.raises(OSError):
        os.kill(child, 0)


def test_run_never_leaves_its_group(env):
    """No setpgid: a supervisor's kill of its group still reaches `run` and
    the command (round 4)."""
    sup = subprocess.Popen([sys.executable, "-c",
                            "import os,subprocess,sys;"
                            "print(os.getpgrp(), flush=True);"
                            f"subprocess.call([sys.executable, {str(TOOL)!r}, 'run', '--', "
                            "sys.executable, '-c', 'import os,time;print(os.getpgrp(),flush=True);time.sleep(30)'])"],
                           env=env, stdout=subprocess.PIPE, text=True, start_new_session=True)
    group = int(sup.stdout.readline())
    assert int(sup.stdout.readline()) == group  # the command is in the supervisor's group
    os.killpg(group, signal.SIGKILL)
    sup.wait(timeout=10)
    wait_free(env, 5)


def test_run_warns_when_its_hold_ends_under_it(env):
    a = subprocess.run([sys.executable, TOOL, "run", "--max", "1.5", "--", "sleep", "5"],
                       env=env, capture_output=True, text=True, timeout=30)
    assert a.returncode == 0
    assert "WARNING: the board lock was lost (max)" in a.stderr


def hand_hold(env, line="session_X (Mac): by hand, since now"):
    d = Path(env["PICAR_BOARD_LOCK_DIR"])
    (d / ".picar-board.owner").write_text(line + "\n")
    p = subprocess.Popen([sys.executable, "-c",
                          "import fcntl,sys,time;f=open(sys.argv[1],'a');"
                          "fcntl.flock(f,fcntl.LOCK_EX);print('x',flush=True);time.sleep(120)",
                          str(d / ".picar-board.lock")], stdout=subprocess.PIPE, text=True)
    p.stdout.readline()
    return p


def test_release_stops_a_hold_still_waiting(env):
    hand = hand_hold(env)
    h = subprocess.Popen([sys.executable, TOOL, "hold", "--wait", "60"], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        end = time.monotonic() + 10
        while not (Path(env["PICAR_BOARD_STATE"]).exists()) and time.monotonic() < end:
            time.sleep(0.1)
        assert tool(env, "release").returncode == 0
        h.kill()
        hand.kill()
        hand.wait()
        time.sleep(3)
        assert lock_free(env)                   # the keeper did not take it later
    finally:
        h.kill()
        hand.kill()


def test_hold_that_gives_up_leaves_no_keeper(env):
    hand = hand_hold(env)
    h = subprocess.Popen([sys.executable, TOOL, "hold", "--wait", "60"], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        time.sleep(2)
        h.send_signal(signal.SIGINT)            # Ctrl-C on `hold` while it waits
        h.wait(timeout=30)
        hand.kill()
        hand.wait()
        time.sleep(3)
        assert lock_free(env)
    finally:
        h.kill()
        hand.kill()


def test_a_reused_pid_is_not_a_hold(env):
    other = subprocess.Popen(["sleep", "60"])
    try:
        Path(env["PICAR_BOARD_STATE"]).write_text(
            f'{{"pid": {other.pid}, "token": "abc", "line": "x"}}')
        assert tool(env, "env").returncode == 1
        assert tool(env, "release").returncode == 0
        assert other.poll() is None             # not signalled
    finally:
        other.kill()


def bench(env, token=None, seconds=0.0):
    e = {**env}
    e.pop("PICAR_BOARD_TOKEN", None)
    if token:
        e["PICAR_BOARD_TOKEN"] = token
    return subprocess.Popen(["bash", "-c", f'source "{CHECK}"; echo ran; sleep {seconds}'],
                            env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def test_a_bench_on_a_free_board_holds_it_while_it_runs(env):
    b = bench(env, seconds=4)
    assert b.stdout.readline().strip() == "ran"
    r = tool(env, "run", "--", "true")
    assert r.returncode == 75 and "bench on the board" in r.stderr
    b.wait(timeout=20)
    wait_free(env, 5)


def test_a_bench_keeps_the_board_after_its_sessions_hold_ends(env):
    assert tool(env, "hold").returncode == 0
    token = tool(env, "env").stdout.strip().split("=", 1)[1]
    b = bench(env, token=token, seconds=6)
    assert b.stdout.readline().strip() == "ran"
    assert tool(env, "release").returncode == 0
    time.sleep(2.5)
    assert not lock_free(env)                   # taken over by the bench
    assert "kept after its session's hold ended" in owner(env)
    b.wait(timeout=20)
    wait_free(env, 5)


def test_an_expired_token_is_refused(env):
    hand = hand_hold(env, "s (Mac): x, since 2026-01-01T00:00:00+00:00, "
                          "until 2026-01-01T01:00:00+00:00, token deadbeefdeadbeef")
    try:
        r = check(env, "deadbeefdeadbeef")
        assert r.returncode == 75
    finally:
        hand.kill()


def test_a_broken_check_is_not_busy(env, tmp_path):
    r = check({**env, "PICAR_BOARD_LOCK_DIR": str(tmp_path / "missing")})
    assert r.returncode == 1 and "ran" not in r.stdout


# -- review round 2 ------------------------------------------------------------

def test_a_bench_keeps_the_board_against_a_waiting_session(env):
    """The takeover blocks on the lock, so a session already waiting with
    --wait does not slip in when the bench's own session lets go."""
    assert tool(env, "hold").returncode == 0
    token = tool(env, "env").stdout.strip().split("=", 1)[1]
    b = bench(env, token=token, seconds=6)
    assert b.stdout.readline().strip() == "ran"
    waiter = subprocess.Popen([sys.executable, TOOL, "run", "--wait", "30", "--", "true"],
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(1.0)
    t_release = time.monotonic()
    assert tool(env, "release").returncode == 0
    waiter.wait(timeout=40)
    assert waiter.returncode == 0
    b.wait(timeout=20)
    # The waiter got the board only once the bench had ended.
    assert time.monotonic() - t_release >= 4


def test_a_process_the_bench_leaves_behind_does_not_hold_the_board(env):
    b = subprocess.Popen(["bash", "-c", f'source "{CHECK}"; sleep 62.41 & echo $!'],
                         env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    orphan = int(b.stdout.readline().strip())
    try:
        b.wait(timeout=20)
        wait_free(env, 3)
    finally:
        os.kill(orphan, signal.SIGKILL)


def test_ctrl_c_reaches_the_command_once_and_the_hold_outlasts_it(env, tmp_path):
    """SIGINT to the whole foreground group (a Ctrl-C): the command gets it
    once, from run, and the board stays held through its teardown."""
    hits = tmp_path / "hits"
    cmd = ("sh", "-c", f'trap "echo INT >> {hits}; sleep 2; exit 0" INT; '
                       'while :; do sleep 0.1; done')
    p = subprocess.Popen([sys.executable, TOOL, "run", "--", *cmd], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                         start_new_session=True)
    end = time.monotonic() + 15
    while not ("token " in owner(env) and not lock_free(env)) and time.monotonic() < end:
        time.sleep(0.1)
    time.sleep(0.5)
    os.killpg(p.pid, signal.SIGINT)
    time.sleep(1.0)
    assert not lock_free(env)                   # held through the teardown
    p.wait(timeout=20)
    assert hits.read_text().count("INT") == 1
    wait_free(env, 5)


def test_a_command_that_cannot_start_releases_the_hold(env):
    r = tool(env, "run", "--", "/no/such/bench")
    assert r.returncode == 127 and "released" in r.stderr
    wait_free(env, 5)


# -- review round 4: the round-3 fixes pinned -------------------------------

def test_the_bench_owner_line_is_written_before_the_check_returns(env):
    owner_path = Path(env["PICAR_BOARD_LOCK_DIR"]) / ".picar-board.owner"
    r = subprocess.run(["bash", "-c", f'source "{CHECK}"; cat "{owner_path}"'], env=env,
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0 and "bench on the board" in r.stdout


def test_release_reports_a_keeper_that_does_not_stop(env, tmp_path):
    state = env["PICAR_BOARD_STATE"]
    stuck = subprocess.Popen([sys.executable, "-c",
                              "import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);"
                              "time.sleep(60)", "board_lock.py", "_keep", "--state", state])
    try:
        time.sleep(0.5)
        Path(state).write_text(f'{{"pid": {stuck.pid}, "token": "abc", "line": "x"}}')
        r = tool({**env, "PICAR_BOARD_STOP_S": "1"}, "release")
        assert r.returncode == 1 and "did not stop" in r.stderr
        assert Path(state).exists()             # kept, so a later release can find it
    finally:
        stuck.kill()


def test_a_hold_that_ends_at_once_says_so(env):
    h = tool({**env, "PICAR_BOARD_HEARTBEAT_S": "0", "PICAR_BOARD_SILENCE_S": "0.2"}, "hold")
    assert h.returncode in (0, 1)
    time.sleep(2)
    assert tool(env, "env").returncode == 1     # never a token for a hold that is gone
    wait_free(env, 5)


def test_hold_giving_up_returns_promptly(env):
    hand = hand_hold(env)
    try:
        t0 = time.monotonic()
        h = subprocess.Popen([sys.executable, TOOL, "hold", "--wait", "60"], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        time.sleep(2)
        h.send_signal(signal.SIGINT)
        h.wait(timeout=30)
        assert time.monotonic() - t0 < 10       # its own keeper reaped, not 20 s on a zombie
    finally:
        hand.kill()


def test_an_inherited_sig_ign_stays_ignored_for_the_command(env, tmp_path):
    """`nohup`: SIGHUP ignored when `run` starts stays ignored in the command."""
    out = tmp_path / "sighup"
    probe = ("import signal;open(%r,'w').write(str(signal.getsignal(signal.SIGHUP) "
             "is signal.SIG_IGN))" % str(out))
    r = subprocess.run(["nohup", sys.executable, TOOL, "run", "--", sys.executable, "-c", probe],
                       env=env, capture_output=True, text=True, timeout=60,
                       cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert out.read_text() == "True"
