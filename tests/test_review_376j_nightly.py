"""
tests/test_review_376j_nightly.py

`PLAN-ros-alignment.md` 3.76, batch J: the bugs the whole-file review of
`tools/nightly_suite.sh` confirmed (J1-J3), and its fixes' review rounds.
Each runs the real script against a scratch origin, with a stub venv python
standing in for pytest and a stub `aws`. On `SLOW_FILES`: process and
signal timing, ~35 s. Each test was written before its fix and seen to fail
on `dev`, except the ones marked "a guard".
"""

import os
import shutil
import subprocess as sp
import sys
import time

import pytest

from tests.test_plan_claims import TOOLS, _git

NIGHTLY = os.path.join(TOOLS, "nightly_suite.sh")


STUB = """#!/bin/sh
# A stand-in for the venv's python: `-m pytest` records the commit it ran on
# (and hangs while $FAKE_HANG exists); anything else is the real python.
if [ "$1" = "-m" ] && [ "$2" = "pytest" ]; then
  git rev-parse --short HEAD > "$FAKE_OUT"
  [ -n "${FAKE_SELF_KILL:-}" ] && kill -9 $$
  if [ -n "${FAKE_HANG:-}" ] && [ -f "$FAKE_HANG" ]; then
    printf '....'                                # pytest -q: a progress line, unfinished
    if [ -n "${FAKE_SLOW_INT:-}" ]; then         # a teardown that never finishes
      trap 'echo INT >> "$FAKE_OUT.sig"' INT
    else                                         # a teardown that takes a moment
      trap 'echo INT >> "$FAKE_OUT.sig"; sleep 1; exit 130' INT
    fi
    sleep 600 &
    echo $! > "$FAKE_OUT.pid"
    while :; do wait; done                       # wait again after a trap that returns
  fi
  echo "1 passed in 0.01s"
  [ -n "${FAKE_CHATTER:-}" ] && echo "INFO: a server a test left running"
  exit 0
fi
exec "$REAL_PY" "$@"
"""


@pytest.fixture
def nightly(tmp_path, monkeypatch):
    """A main checkout with an origin holding dev, the script under test
    committed in it, a stub venv python, and a stub `aws` that records each
    email. Returns (run, push, paths)."""
    for k in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE"):
        monkeypatch.delenv(k, raising=False)
    bare, main, home = tmp_path / "origin.git", tmp_path / "main", tmp_path / "home"
    sp.run(["git", "init", "-q", "--bare", "-b", "dev", str(bare)], check=True)
    sp.run(["git", "clone", "-q", str(bare), str(main)], check=True, capture_output=True)
    (main / "tools").mkdir()
    shutil.copy(NIGHTLY, main / "tools" / "nightly_suite.sh")
    (main / "tests").mkdir()
    (main / "tests" / "f.txt").write_text("one\n")
    (main / ".gitignore").write_text(".venv/\n")
    _git(main, "checkout", "-q", "-b", "dev")
    _git(main, "add", "-A")
    _git(main, "commit", "-q", "-m", "c1")
    _git(main, "push", "-q", "origin", "dev")
    (main / ".venv" / "bin").mkdir(parents=True)
    stub = main / ".venv" / "bin" / "python"
    stub.write_text(STUB)
    stub.chmod(0o755)
    out = tmp_path / "ran-on"
    bindir, mails = tmp_path / "bin", tmp_path / "mails"
    bindir.mkdir()
    (bindir / "aws").write_text(f'#!/bin/sh\necho "$*" >> {mails}\n')
    (bindir / "aws").chmod(0o755)
    (bindir / "git").write_text(                 # FAKE_GIT_FAIL: every refresh path fails
        '#!/bin/sh\nif [ -n "${FAKE_GIT_FAIL:-}" ]; then case " $* " in\n'
        '  *" checkout "*|*" worktree "*) echo "fatal: refused" >&2; exit 128 ;; esac; fi\n'
        f'exec {shutil.which("git")} "$@"\n')
    (bindir / "git").chmod(0o755)
    base = {k: v for k, v in os.environ.items()          # round 12: a nightly run's own
            if not k.startswith("NIGHTLY_")}             # settings are not this test's
    env = {**base, "HOME": str(home), "NIGHTLY_EMAIL": "", "FAKE_OUT": str(out),
           "PATH": f"{bindir}:{os.environ['PATH']}",
           "REAL_PY": sys.executable, "FAKE_HANG": str(tmp_path / "hang"),
           "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
           "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}

    latest = home / "Library" / "Logs" / "vision-picar" / "nightly-latest.txt"

    def run(timeout=60, **extra):
        r = sp.run(["sh", str(main / "tools" / "nightly_suite.sh")], capture_output=True,
                   text=True, env={**env, **extra}, timeout=timeout)
        return r, (latest.read_text() if latest.exists() else ""), \
            (out.read_text().strip() if out.exists() else "")

    def start(shell=("sh",), **extra):
        if shell != ("sh",):                     # the script runs again under its lock
            wrap = tmp_path / "shell"
            wrap.write_text(f'#!/bin/sh\nexec {" ".join(shell)} "$@"\n')
            wrap.chmod(0o755)
            extra["NIGHTLY_SH"] = str(wrap)
        return sp.Popen([*shell, str(main / "tools" / "nightly_suite.sh")], stdout=sp.PIPE,
                        stderr=sp.PIPE, text=True, env={**env, **extra}, start_new_session=True)

    def push(text):
        (main / "tests" / "f.txt").write_text(text)
        _git(main, "commit", "-q", "-am", "next")
        _git(main, "push", "-q", "origin", "dev")
        return sp.run(["git", "rev-parse", "--short", "HEAD"], cwd=main, capture_output=True,
                      text=True).stdout.strip()

    wt = main / ".git" / "nightly-worktree"
    return run, push, {"wt": wt, "hang": tmp_path / "hang", "out": out, "start": start,
                       "latest": latest, "mails": mails, "main": main}


def _gone(pid, wait_s=5.0):
    """The process has exited (a zombie waiting for init counts): a SIGKILL
    lands, and an orphan is reaped, a moment after `killpg` returns. Asks
    the kernel, then `ps` for a zombie: macOS, where the nightly runs, has
    no /proc (round 7: a missing /proc read as "gone")."""
    deadline = time.monotonic() + wait_s
    while True:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        state = sp.run(["ps", "-o", "stat=", "-p", str(pid)], capture_output=True,
                       text=True).stdout.strip()
        if not state or state.startswith("Z"):
            return True
        if time.monotonic() > deadline:
            return False
        time.sleep(0.05)


def test_j1_a_dirty_nightly_worktree_still_runs_the_latest_dev(nightly):
    """`checkout --detach origin/dev && reset --hard` skipped the reset when
    the checkout refused over a modified tracked file, and the night reported
    PASS under the previous night's commit."""
    run, push, p = nightly
    r, latest, ran = run()
    assert r.returncode == 0 and "PASS" in latest
    (p["wt"] / "tests" / "f.txt").write_text("left behind by a test\n")
    (p["wt"] / "tests" / "stray.txt").write_text("untracked\n")
    new = push("two\n")
    r, latest, ran = run()
    assert ran == new, (ran, new, r.stdout, r.stderr)
    assert latest.split()[1] == new and "PASS" in latest
    assert not (p["wt"] / "tests" / "stray.txt").exists()


def test_j2_a_hung_suite_ends_as_a_failure(nightly):
    """No time limit: a hung test held the job for good (launchd starts no
    second copy), and no report or email ever came."""
    run, push, p = nightly
    p["hang"].write_text("")
    t0 = time.monotonic()
    r, latest, ran = run(timeout=60, NIGHTLY_TIMEOUT_S="2")
    assert time.monotonic() - t0 < 30
    assert r.returncode != 0 and "FAIL" in latest and "timed out" in latest, latest
    assert _gone(int(open(f"{p['out']}.pid").read()))
    assert open(f"{p['out']}.sig").read() == "INT\n"          # its teardown ran first


@pytest.mark.parametrize("shell", [("sh",), ("bash", "--posix")])
def test_j2_stopping_the_job_takes_the_suite_with_it(nightly, shell):
    """Round 1: pytest runs in its own process group (so a time limit can kill
    all of it); a stop (TERM to the job's whole group, so the driver gets it
    twice, round 2) must not leave it running, must let its teardown run, and
    must report the stop rather than a traceback."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"](shell, NIGHTLY_EMAIL="me@example.com")
    pidfile = f"{p['out']}.pid"
    deadline = time.monotonic() + 20
    while not os.path.exists(pidfile) and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(0.2)
    os.killpg(proc.pid, 15)
    proc.communicate(timeout=30)
    assert _gone(int(open(pidfile).read()))
    latest = p["latest"].read_text()
    assert "STOPPED" in latest and "nightly: stopped by signal" in latest, latest
    assert open(f"{p['out']}.sig").read() == "INT\n"
    assert not p["mails"].exists()                  # round 3: a stop someone made is not mailed


@pytest.mark.parametrize("delay", [0.0, 0.05, 0.1, 0.2, 0.4, 0.8])
def test_j2_a_stop_at_any_moment_leaves_nothing_running(nightly, delay):
    """Round 3: a stop that landed while pytest was starting (before the
    driver could kill it), or before the sh's trap was set, left the run
    going with no report, or crashed the driver."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"]()
    time.sleep(delay)
    os.killpg(proc.pid, 15)
    proc.communicate(timeout=30)
    time.sleep(0.3)                                 # a stub that was starting has written its pid
    pidfile = f"{p['out']}.pid"
    if os.path.exists(pidfile):
        assert _gone(int(open(pidfile).read()))
    logs = p["latest"].parent
    assert not any("Traceback" in f.read_text() for f in logs.glob("nightly-*.log"))
    if p["latest"].exists():
        assert "STOPPED" in p["latest"].read_text() or "PASS" not in p["latest"].read_text()


def test_j2_a_failure_a_late_stop_lands_on_is_still_a_failure(nightly):
    """Round 4: STOPPED followed the sh's trap, not what ended the run; a
    hung night stopped during its grace read STOPPED and was never mailed."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"](NIGHTLY_EMAIL="me@example.com", NIGHTLY_TIMEOUT_S="1")
    pidfile = f"{p['out']}.sig"
    deadline = time.monotonic() + 20                # the limit has passed: its SIGINT landed
    while not os.path.exists(pidfile) and time.monotonic() < deadline:
        time.sleep(0.02)
    os.killpg(proc.pid, 15)
    proc.communicate(timeout=30)
    latest = p["latest"].read_text()
    assert "FAIL" in latest and "timed out" in latest, latest
    assert "send-email" in p["mails"].read_text()


def test_j2_a_stop_in_the_time_limits_grace_ends_within_ten_seconds(nightly):
    """Round 5: a stop during the time limit's 60 s grace was only recorded,
    and the driver waited out the minute, past launchd's 20 s."""
    run, push, p = nightly
    p["hang"].write_text("")
    (p["main"] / "slow-int").write_text("")
    proc = p["start"](NIGHTLY_TIMEOUT_S="1", FAKE_SLOW_INT=str(p["main"] / "slow-int"))
    sig = f"{p['out']}.sig"
    deadline = time.monotonic() + 20
    while not os.path.exists(sig) and time.monotonic() < deadline:
        time.sleep(0.02)
    t0 = time.monotonic()
    os.killpg(proc.pid, 15)
    proc.communicate(timeout=90)
    assert time.monotonic() - t0 < 15
    assert _gone(int(open(f"{p['out']}.pid").read()))


def test_j3_a_directory_that_is_not_a_worktree_is_taken_back(nightly):
    """Round 5: a directory left where the worktree goes, not registered,
    failed every night ('must be run in a work tree')."""
    run, push, p = nightly
    p["wt"].mkdir(parents=True)
    (p["wt"] / "leftover").write_text("x")
    r, latest, ran = run()
    assert "PASS" in latest, (latest, r.stderr)


def test_j2_pytest_killed_by_a_signal_says_so(nightly):
    """Round 5: a pytest that died of a signal (OOM) left the log's last line
    as the summary."""
    run, push, p = nightly
    r, latest, ran = run(FAKE_SELF_KILL="1")
    assert "FAIL" in latest and "pytest killed by signal 9" in latest, latest


def test_j2_the_summary_is_pytests_not_a_leftover_line(nightly):
    """Round 8: a line a leftover server wrote after pytest's summary became
    the night's summary."""
    run, push, p = nightly
    r, latest, ran = run(FAKE_CHATTER="1")
    assert latest.rstrip().endswith("1 passed in 0.01s"), latest


def test_j1_one_run_at_a_time(nightly):
    """Round 10: a run started by hand during the nightly one rewrote the
    shared worktree under its pytest. Round 11: a kernel lock, so nothing is
    left to go stale; a refused run says so in the report."""
    import fcntl
    run, push, p = nightly
    run()
    with open(p["main"] / ".git" / "nightly.lock", "a") as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        r, latest, ran = run()
        assert r.returncode != 0 and latest.split()[1:3] == ["-", "SKIPPED"], latest
    new = push("nine\n")                         # released with its holder
    r, latest, ran = run()
    assert ran == new and "PASS" in latest


def test_j1_the_lock_is_not_handed_to_the_suite(nightly):
    """Round 12: NIGHTLY_LOCKED reached pytest, so the nightly's own run of
    this file skipped the lock it tests."""
    run, push, p = nightly
    stub = p["main"] / ".venv" / "bin" / "python"
    stub.write_text(stub.read_text().replace(
        'git rev-parse --short HEAD > "$FAKE_OUT"',
        'git rev-parse --short HEAD > "$FAKE_OUT"; env | grep "^NIGHTLY_LOCKED" > "$FAKE_OUT.env"'))
    run()
    assert open(f"{p['out']}.env").read() == ""


def test_j1_a_missing_venv_is_still_reported(nightly):
    """Round 12: with no venv python the lock's exec failed before any report."""
    run, push, p = nightly
    (p["main"] / ".venv" / "bin" / "python").unlink()
    r, latest, ran = run()
    assert latest.split()[2] == "FAIL", latest


def test_j1_a_failed_fetch_names_no_commit(nightly):
    """Round 3: a failed fetch reported yesterday's commit as the one that failed."""
    run, push, p = nightly
    run()
    _git(p["main"], "remote", "set-url", "origin", "/nonexistent/origin.git")
    r, latest, ran = run(NIGHTLY_EMAIL="me@example.com")
    assert latest.split()[1:3] == ["-", "FAIL"] and "fetch failed" in latest, latest
    assert "send-email" in p["mails"].read_text()


def test_j1_a_refresh_that_fails_is_reported_and_emailed(nightly):
    """Round 1: the refresh's failure is the night's FAIL, and like any FAIL
    it is emailed (the first fix exited before the email)."""
    run, push, p = nightly
    run()
    push("four\n")
    r, latest, ran = run(NIGHTLY_EMAIL="me@example.com", FAKE_GIT_FAIL="1")
    assert r.returncode != 0 and "FAIL" in latest and "could not refresh" in latest, latest
    assert latest.split()[1] == "-"                 # round 4: no commit was tested
    assert "send-email" in p["mails"].read_text()


def _admin(p):
    return sp.run(["git", "rev-parse", "--git-dir"], cwd=p["wt"], capture_output=True,
                  text=True).stdout.strip()


def test_j1_a_stale_lock_heals(nightly):
    """Round 6: a refresh that fails is followed by a fresh worktree; a stale
    index.lock no longer fails every night until someone looks."""
    run, push, p = nightly
    run()
    open(os.path.join(p["wt"], _admin(p), "index.lock"), "w").close()
    new = push("six\n")
    r, latest, ran = run()
    assert ran == new and "PASS" in latest, latest


def test_j3_a_worktree_of_a_moved_repo_heals(nightly):
    """Round 6: a `.git` file pointing at the repo's old place (the checkout
    moved) failed every night."""
    run, push, p = nightly
    run()
    (p["wt"] / ".git").write_text("gitdir: /nonexistent/old/.git/worktrees/nightly-worktree\n")
    new = push("seven\n")
    r, latest, ran = run()
    assert ran == new and "PASS" in latest, latest


def test_j3_a_locked_registration_is_taken_back(nightly):
    """Round 6: a locked worktree whose directory is gone needs `add -f -f`."""
    run, push, p = nightly
    run()
    _git(p["main"], "worktree", "lock", str(p["wt"]))
    shutil.rmtree(p["wt"])
    new = push("eight\n")
    r, latest, ran = run()
    assert ran == new and "PASS" in latest, latest


def test_j1_guard_a_branch_checked_out_in_the_worktree_is_not_moved(nightly):
    """A guard (round 1): `reset --hard` would have moved a branch someone
    switched to in the nightly worktree; `checkout -f --detach` leaves it."""
    run, push, p = nightly
    run()
    _git(p["wt"], "switch", "-q", "-c", "look")
    (p["wt"] / "tests" / "f.txt").write_text("mine\n")
    _git(p["wt"], "commit", "-q", "-am", "mine")
    mine = sp.run(["git", "rev-parse", "look"], cwd=p["wt"], capture_output=True,
                  text=True).stdout.strip()
    new = push("five\n")
    r, latest, ran = run()
    assert ran == new
    assert sp.run(["git", "rev-parse", "look"], cwd=p["main"], capture_output=True,
                  text=True).stdout.strip() == mine


def test_j3_a_deleted_nightly_worktree_is_rebuilt(nightly):
    """A worktree directory deleted by hand stays registered; `worktree add`
    then failed every night and the report named no cause."""
    run, push, p = nightly
    run()
    shutil.rmtree(p["wt"])
    new = push("three\n")
    r, latest, ran = run()
    assert ran == new and "PASS" in latest, (latest, r.stdout, r.stderr)
