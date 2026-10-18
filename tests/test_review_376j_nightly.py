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
  [ -n "${FAKE_CHATTER_TIMED:-}" ] && echo "INFO: request handled in 0.03s"
  [ -n "${FAKE_SUBTESTS:-}" ] && echo "3 failed, 2 passed, 1 subtests passed in 0.04s" \
    && echo "INFO: a server a test left running" && exit 1
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
        # FAKE_PIPE: what a pipe does in the script's own sh (git inherits it)
        '[ -n "${FAKE_PIPE:-}" ] && (yes | head -n 1 > /dev/null) 2>> "$FAKE_PIPE"\n'
        # FAKE_FETCH_HANG: a fetch that stalls until it is killed
        '[ -n "${FAKE_FETCH_HANG:-}" ] && case " $* " in *" fetch "*) sleep 600 ;; esac\n'
        # FAKE_DAEMON: a daemon git leaves running (fsmonitor, maintenance)
        '[ -n "${FAKE_DAEMON:-}" ] && (sleep 30 > /dev/null 2>&1 & echo $! >> "$FAKE_DAEMON")\n'
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
        return r, (latest.read_text() if latest.is_file() else ""), \
            (out.read_text().strip() if out.exists() else "")

    def start(shell=("sh",), inner=True, **extra):
        if shell != ("sh",) and inner:           # the script runs again under its lock
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


def _until_running(p, wait_s=20.0):
    """Until the sh's own RUNNING line is written: past the lock and its trap
    (the wrapper's earlier line says "locked")."""
    deadline = time.monotonic() + wait_s
    while "RUNNING  started" not in (p["latest"].read_text() if p["latest"].exists() else ""):
        if time.monotonic() > deadline:            # round 19: never go on silently
            pytest.fail(f"no RUNNING line in {wait_s} s")
        time.sleep(0.05)


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
        latest = p["latest"].read_text()
        # Round 14: "or no PASS" let a stop read as FAIL. Once the night's
        # line is written, a stop is STOPPED (RUNNING only if it never got there).
        assert "PASS" not in latest and "FAIL" not in latest, latest


def test_j2_a_failure_a_late_stop_lands_on_is_still_a_failure(nightly):
    """Round 4: STOPPED followed the sh's trap, not what ended the run; a
    hung night stopped during its grace read STOPPED and was never mailed."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"](NIGHTLY_EMAIL="me@example.com", NIGHTLY_TIMEOUT_S="1")
    sig = f"{p['out']}.sig"
    deadline = time.monotonic() + 20                # the limit has passed: its SIGINT landed
    while not os.path.exists(sig) and time.monotonic() < deadline:
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
        # Round 14: added, never replacing the holder's verdict.
        lines = latest.splitlines()
        assert r.returncode != 0 and lines[-1].split()[1:3] == ["-", "SKIPPED"], latest
        assert "PASS" in lines[0], latest
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


# ---------- round 14 ----------


def test_r14_a_daemon_a_run_leaves_does_not_hold_the_lock(nightly, tmp_path):
    """The lock fd was handed through exec to the sh and every child: a
    daemon git starts (fsmonitor, maintenance) held it, and every later night
    read SKIPPED, unmailed."""
    import signal
    run, push, p = nightly
    pids = tmp_path / "daemons"
    try:
        r, latest, ran = run(FAKE_DAEMON=str(pids))
        assert "PASS" in latest, latest
        r, latest, ran = run()
        assert "PASS" in latest and "SKIPPED" not in latest, latest
    finally:                                     # round 15: no daemon outlives the test
        for pid in (pids.read_text().split() if pids.exists() else []):
            try:
                os.kill(int(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_r14_a_pipe_in_the_script_behaves(nightly, tmp_path):
    """An exec from python left SIGPIPE ignored in the sh, which no sh can
    reset: `grep ... | head -20` failed with "Broken pipe"."""
    run, push, p = nightly
    errs = tmp_path / "pipe-errors"
    errs.write_text("")
    run(FAKE_PIPE=str(errs))
    assert errs.read_text() == "", errs.read_text()


def test_r14_a_lock_error_that_is_not_held_still_runs(nightly):
    """Any flock error read as "held", and a lock file that could not be
    opened crashed the wrapper with a traceback and no report."""
    run, push, p = nightly
    (p["main"] / ".git" / "nightly.lock").mkdir()
    r, latest, ran = run()
    assert "PASS" in latest, (latest, r.stderr)
    assert "Traceback" not in r.stderr


def test_r14_a_stop_during_the_fetch_is_stopped_not_yesterdays_pass(nightly):
    """The trap was set after the fetch, and the report written only at the
    end: a stop in the fetch left yesterday's verdict standing."""
    run, push, p = nightly
    run()                                           # yesterday's PASS
    proc = p["start"](FAKE_FETCH_HANG="1", NIGHTLY_EMAIL="me@example.com")
    _until_running(p)
    os.killpg(proc.pid, 15)
    proc.communicate(timeout=30)
    latest = p["latest"].read_text()
    assert "PASS" not in latest and "STOPPED" in latest, latest
    assert not p["mails"].exists() or "send-email" not in p["mails"].read_text()


def test_r14_the_summary_is_pytests_not_a_servers_timed_line(nightly):
    """Round 8's filter took any line ending "in <n>s"."""
    run, push, p = nightly
    r, latest, ran = run(FAKE_CHATTER_TIMED="1")
    assert latest.rstrip().endswith("1 passed in 0.01s"), latest


# ---------- round 15 ----------


def test_r15_a_stop_to_the_job_alone_ends_a_stalled_fetch(nightly):
    """launchd TERMs the job's main process only. The wrapper passed it to
    the sh, which runs a trap only after its foreground command: a stalled
    fetch (120 s) outlasted launchd's 20 s, and the night read RUNNING."""
    run, push, p = nightly
    proc = p["start"](FAKE_FETCH_HANG="1", NIGHTLY_EMAIL="me@example.com")
    _until_running(p)
    t0 = time.monotonic()
    os.kill(proc.pid, 15)                           # the wrapper alone, as launchd does
    try:
        proc.communicate(timeout=15)
    finally:                                        # round 16: the fake fetch's sleep too
        try:
            os.killpg(proc.pid, 9)
        except ProcessLookupError:
            pass
    assert time.monotonic() - t0 < 15
    latest = p["latest"].read_text()
    assert latest.split()[2] == "STOPPED" and "before the suite ran" in latest, latest
    assert not p["mails"].exists() or "send-email" not in p["mails"].read_text()


def test_r15_pytest_9s_subtests_summary_is_the_summary(nightly):
    """pytest 9 adds "N subtests passed" to its summary; the anchored match
    missed it and took a later line."""
    run, push, p = nightly
    r, latest, ran = run(FAKE_SUBTESTS="1")
    assert "1 subtests passed in 0.04s" in latest, latest


def test_r15_an_unlocked_run_says_so_in_the_report(nightly):
    """A lock error other than "held" ran unlocked with a word only in
    launchd.log; every later night could run two at once unnoticed."""
    run, push, p = nightly
    (p["main"] / ".git" / "nightly.lock").mkdir()
    r, latest, ran = run()
    assert "PASS" in latest and "(unlocked: " in latest, latest


# ---------- round 16 ----------


def test_r16_a_refused_run_under_a_stale_line_is_the_line(nightly):
    """Appending SKIPPED kept an old night's PASS on top when the holder was
    no nightly run (a daemon from before round 14): it read PASS for good."""
    import fcntl
    run, push, p = nightly
    run()
    p["latest"].write_text("2026-01-01 abc PASS  1 passed in 0.01s\n")    # an old night
    with open(p["main"] / ".git" / "nightly.lock", "a") as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        r, latest, ran = run()
    assert latest.splitlines()[0].split()[1:3] == ["-", "SKIPPED"], latest


def test_r16_a_sh_killed_before_its_trap_still_leaves_a_report(nightly, tmp_path):
    """A stop forwarded before the sh set its trap killed it by default: no
    RUNNING, no STOPPED, yesterday's verdict standing."""
    run, push, p = nightly
    run()                                           # yesterday's PASS
    killer = tmp_path / "sh-dies"
    killer.write_text('#!/bin/sh\nkill -TERM $PPID\nsleep 5\n')   # a stop, forwarded before any trap
    killer.chmod(0o755)
    r, latest, ran = run(NIGHTLY_SH=str(killer))
    assert "STOPPED" in latest and "stopped by signal 15" in latest, latest




# ---------- round 17 ----------


def test_r17_a_refused_run_survives_the_holders_verdict(nightly):
    """The SKIPPED a refused run adds under tonight's line was erased when the
    holder wrote its verdict over the whole file."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"](NIGHTLY_TIMEOUT_S="3")
    _until_running(p)
    r, latest, ran = run()                          # refused while the first runs
    assert "SKIPPED" in latest, latest
    proc.communicate(timeout=60)
    lines = p["latest"].read_text().splitlines()
    assert "FAIL" in lines[0] and any(" SKIPPED " in l for l in lines[1:]), lines


def test_r17_the_wrapper_never_writes_over_a_verdict(nightly, tmp_path):
    """The wrapper's line replaced a verdict the sh had already written when
    the sh then died of a signal it does not trap (SIGQUIT, an OOM kill)."""
    run, push, p = nightly
    late = tmp_path / "sh-writes-then-dies"
    late.write_text('#!/bin/sh\nNIGHTLY_LOCKED=wrapper:$$ /bin/sh "$@"\nkill -KILL $$\n')   # the run, then death
    late.chmod(0o755)
    r, latest, ran = run(NIGHTLY_SH=str(late))
    assert latest.split()[2] == "PASS", latest


def test_r17_a_stale_exported_handshake_never_skips_the_lock(nightly):
    """NIGHTLY_LOCKED left exported in a shell skipped the wrapper, the lock
    and the "(unlocked" note; NIGHTLY_UNLOCKED put a false note on the line."""
    import fcntl
    run, push, p = nightly
    run()
    with open(p["main"] / ".git" / "nightly.lock", "a") as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        r, latest, ran = run(NIGHTLY_LOCKED="1")
        assert "SKIPPED" in latest, latest
    r, latest, ran = run(NIGHTLY_UNLOCKED="stale")
    assert "unlocked" not in latest, latest


# ---------- round 18 ----------


def _shell(tmp_path, name, body):
    """A NIGHTLY_SH: `body`, then (unless it ends the run itself) the real sh."""
    f = tmp_path / name
    f.write_text("#!/bin/sh\n" + body)
    f.chmod(0o755)
    return str(f)


def test_r18_tonights_skipped_lines_survive_the_scripts_own_lines(nightly, tmp_path):
    """The sh's RUNNING line truncated the file the wrapper wrote, and with it
    a SKIPPED a refused run had just added."""
    run, push, p = nightly
    sh = _shell(tmp_path, "skip-then-run",
                f'echo "$(date +%Y-%m-%d) - SKIPPED  refused" >> "{p["latest"]}"\nexec /bin/sh "$@"\n')
    r, latest, ran = run(NIGHTLY_SH=sh)
    lines = latest.splitlines()
    assert "PASS" in lines[0] and any(" SKIPPED " in l for l in lines[1:]), lines


def test_r18_the_wrappers_stopped_line_keeps_tonights_skipped(nightly, tmp_path):
    """The wrapper's STOPPED fallback rewrote the file without them."""
    run, push, p = nightly
    sh = _shell(tmp_path, "skip-then-die",
                f'echo "$(date +%Y-%m-%d) - SKIPPED  refused" >> "{p["latest"]}"\nkill -TERM $PPID\nsleep 5\n')
    r, latest, ran = run(NIGHTLY_SH=sh)
    lines = latest.splitlines()
    assert "STOPPED" in lines[0] and any(" SKIPPED " in l for l in lines[1:]), lines


def test_r18_an_unlocked_run_says_so_while_it_runs(nightly, tmp_path):
    """The wrapper wrote "RUNNING  locked" though it held no lock."""
    run, push, p = nightly
    (p["main"] / ".git" / "nightly.lock").mkdir()
    seen = tmp_path / "first-line"
    sh = _shell(tmp_path, "peek", f'head -n 1 "{p["latest"]}" > "{seen}"\nexec /bin/sh "$@"\n')
    run(NIGHTLY_SH=sh)
    assert "RUNNING  unlocked" in seen.read_text(), seen.read_text()


# ---------- round 19 ----------


def test_r19_the_unlocked_note_stays_on_the_running_line(nightly):
    """The sh's own RUNNING line replaced the wrapper's "unlocked" at once,
    so the note was gone for the whole run."""
    run, push, p = nightly
    (p["main"] / ".git" / "nightly.lock").mkdir()
    proc = p["start"](FAKE_FETCH_HANG="1")
    try:
        _until_running(p)
        assert "(unlocked: " in p["latest"].read_text(), p["latest"].read_text()
    finally:
        try:
            os.killpg(proc.pid, 9)
        except ProcessLookupError:
            pass
        proc.communicate(timeout=30)


def test_r19_a_skipped_first_line_survives_the_next_run(nightly):
    """The wrapper kept only lines after the first, so tonight's SKIPPED on
    line 1 (a holder that was no nightly run) vanished with the next run."""
    import datetime
    run, push, p = nightly
    run()
    day = datetime.date.today().isoformat()
    p["latest"].write_text(f"{day} - SKIPPED  another nightly run holds x\n")
    r, latest, ran = run()
    lines = latest.splitlines()
    assert "PASS" in lines[0] and any(" SKIPPED " in l for l in lines[1:]), lines


def test_r19_nohup_keeps_the_run_going(nightly, tmp_path):
    """The wrapper replaced nohup's ignored HUP with its forwarder, and the
    sh's trap read the HUP as a stop."""
    run, push, p = nightly
    p["hang"].write_text("")
    proc = p["start"](("nohup", "sh"), inner=False, NIGHTLY_TIMEOUT_S="4")   # as typed
    deadline = time.monotonic() + 20                # the suite is running (git resets
    while not p["out"].exists() and time.monotonic() < deadline:   # HUP in its own children)
        time.sleep(0.05)
    os.killpg(proc.pid, 1)                          # the terminal closes
    proc.communicate(timeout=60)
    latest = p["latest"].read_text()
    assert "timed out" in latest and "STOPPED" not in latest, latest


# ---------- round 20 ----------


@pytest.mark.skipif(not os.path.exists("/dev/full"), reason="needs /dev/full (Linux)")
def test_r20_a_write_that_fails_never_empties_the_report(nightly, tmp_path):
    """The sh's report moved its temp file into place even when the write had
    failed (a full disk): the report became an empty file -- here, /dev/full
    itself. The sh's first write (RUNNING) fails; the temp file is dropped,
    so its verdict, later, lands."""
    run, push, p = nightly
    run()
    full = f'{p["latest"]}.new.$$'                  # the sh's own temp name, after the exec
    sh = _shell(tmp_path, "disk-full", f'ln -s /dev/full "{full}"\nexec /bin/sh "$@"\n')
    r, latest, ran = run(NIGHTLY_SH=sh)
    assert "could not write the report" in r.stderr, r.stderr
    assert not p["latest"].is_symlink(), "a failed write was moved into place"
    assert latest.split()[2] == "PASS", latest
    for f in p["latest"].parent.glob("nightly-latest.txt.new.*"):
        f.unlink()


def test_r20_a_script_that_exits_before_its_report_is_a_fail(nightly, tmp_path):
    """Only a death by signal was reported: a sh that exited non-zero before
    its own line left "RUNNING  locked" for good, unmailed."""
    run, push, p = nightly
    r, latest, ran = run(NIGHTLY_SH=_shell(tmp_path, "exits", "exit 2\n"))
    assert latest.split()[2] == "FAIL" and "exited 2 without a report" in latest, latest


def test_r20_a_refused_run_that_cannot_write_says_so(nightly):
    """The refused run's write had no guard: a traceback, not a line."""
    import fcntl, shutil
    run, push, p = nightly
    run()
    p["latest"].unlink()
    p["latest"].mkdir()                             # the report cannot be written
    try:
        with open(p["main"] / ".git" / "nightly.lock", "a") as held:
            fcntl.flock(held, fcntl.LOCK_EX)
            r, latest, ran = run()
    finally:
        shutil.rmtree(p["latest"])
    assert r.returncode == 1 and "Traceback" not in r.stderr and "could not write" in r.stderr, r.stderr


# ---------- round 21 ----------


def test_r21_the_wrappers_own_fail_is_mailed(nightly, tmp_path):
    """A FAIL only the wrapper saw (the sh exited before its report) was
    reported but never emailed: a broken nightly failed in silence."""
    run, push, p = nightly
    r, latest, ran = run(NIGHTLY_SH=_shell(tmp_path, "exits", "exit 2\n"),
                         NIGHTLY_EMAIL="me@example.com")
    assert "FAIL" in latest and "send-email" in p["mails"].read_text(), latest


def test_r21_a_verdict_that_could_not_be_written_never_reads_running(nightly, tmp_path):
    """The wrapper's fallback took only its own RUNNING line: a sh whose own
    RUNNING landed but whose verdict write failed left the night RUNNING."""
    run, push, p = nightly
    # the sh's own RUNNING line, then an exit before any verdict
    sh = _shell(tmp_path, "running-then-exits",
                f'echo "$(date +%Y-%m-%d) - RUNNING  started 03:00" > "{p["latest"]}"\nexit 1\n')
    r, latest, ran = run(NIGHTLY_SH=sh)
    assert latest.split()[2] == "FAIL", latest


# ---------- round 22 ----------


def _running_then(p, tmp_path, name, end):
    """A NIGHTLY_SH that writes the sh's own RUNNING line, then ends as `end`."""
    return _shell(tmp_path, name,
                  f'echo "$(date +%Y-%m-%d) - RUNNING  started 03:00" > "{p["latest"]}"\n{end}\n')


def test_r22_a_stop_whose_report_failed_is_stopped_and_not_mailed(nightly, tmp_path):
    """A stop's exit (124) with no verdict landed read as a FAIL, and was
    mailed. Round 23: the wrapper decides by the stop it saw."""
    run, push, p = nightly
    sh = _running_then(p, tmp_path, "stop", "trap '' TERM\nkill -TERM $PPID\nsleep 1\nexit 124")
    r, latest, ran = run(NIGHTLY_SH=sh, NIGHTLY_EMAIL="me@example.com")
    assert latest.split()[2] == "STOPPED", latest
    assert not p["mails"].exists() or "send-email" not in p["mails"].read_text()


def test_r22_an_oom_killed_script_is_a_mailed_fail(nightly, tmp_path):
    """Any untrapped signal read as STOPPED and went unmailed; only a stop's is."""
    run, push, p = nightly
    r, latest, ran = run(NIGHTLY_SH=_running_then(p, tmp_path, "oom", "kill -KILL $$"),
                         NIGHTLY_EMAIL="me@example.com")
    assert latest.split()[2] == "FAIL" and "signal 9" in latest, latest
    assert "send-email" in p["mails"].read_text()


def test_r22_a_pass_whose_report_failed_is_a_pass(nightly, tmp_path):
    run, push, p = nightly
    r, latest, ran = run(NIGHTLY_SH=_running_then(p, tmp_path, "pass", "exit 0"))
    assert latest.split()[2] == "PASS", latest


# ---------- round 23 ----------


def test_r23_pytests_internal_error_is_a_fail_not_a_stop(nightly, tmp_path):
    """Exit 3 meant STOPPED to the wrapper; pytest exits 3 on an internal
    error, which then read STOPPED, unmailed."""
    run, push, p = nightly
    r, latest, ran = run(NIGHTLY_SH=_running_then(p, tmp_path, "internal", "exit 3"),
                         NIGHTLY_EMAIL="me@example.com")
    assert latest.split()[2] == "FAIL", latest
    assert "send-email" in p["mails"].read_text()


# ---------- round 24 ----------


def test_r24_a_stop_during_the_wrappers_mail_ends_it_soon(nightly, tmp_path):
    """The mail's 60 s was fixed when it started: a stop during the send was
    recorded but waited the full minute, past launchd's 20 s."""
    run, push, p = nightly
    bindir = tmp_path / "bin"
    (bindir / "aws").write_text(f'#!/bin/sh\necho "$*" >> {p["mails"]}\nexec sleep 120\n')
    sh = _shell(tmp_path, "exits", "exit 2\n")
    proc = p["start"](NIGHTLY_SH=sh, NIGHTLY_EMAIL="me@example.com")
    deadline = time.monotonic() + 20
    while not (p["mails"].exists() and "send-email" in p["mails"].read_text()) \
            and time.monotonic() < deadline:
        time.sleep(0.05)
    t0 = time.monotonic()
    os.kill(proc.pid, 15)                           # launchd's stop, to the wrapper
    proc.communicate(timeout=60)
    assert time.monotonic() - t0 < 12, time.monotonic() - t0
    assert "email failed" in p["latest"].read_text()

