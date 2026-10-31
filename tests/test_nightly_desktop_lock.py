"""
tests/test_nightly_desktop_lock.py

3.129: the nightly's full suite holds the desktop lock (`tools/desktop_lock.py`)
exclusive. It runs the real script against 3.76 J's scratch origin (its
`nightly` fixture), with the lock tool committed there and a lock directory
of the test's own.
"""

import os
import shutil
import subprocess as sp
import sys
import time

import pytest

from tests.test_plan_claims import TOOLS, _git
from tests.test_review_376j_nightly import _until_running, nightly  # noqa: F401

LOCK_TOOL = os.path.join(TOOLS, "desktop_lock.py")


@pytest.fixture
def locked(nightly, tmp_path, monkeypatch):  # noqa: F811
    """The scratch origin's dev with tools/desktop_lock.py, and a stub
    pytest that also records the lock token it was given."""
    run, push, p = nightly
    main = p["main"]
    shutil.copy(LOCK_TOOL, main / "tools" / "desktop_lock.py")
    _git(main, "add", "-A")
    _git(main, "commit", "-q", "-m", "lock tool")
    _git(main, "push", "-q", "origin", "dev")
    stub = main / ".venv" / "bin" / "python"
    stub.write_text(stub.read_text().replace(
        'git rev-parse --short HEAD > "$FAKE_OUT"',
        'git rev-parse --short HEAD > "$FAKE_OUT"\n'
        '  echo "${PICAR_DESKTOP_LOCK:-none}" > "$FAKE_OUT.token"'))
    d = tmp_path / "lockdir"
    d.mkdir()
    monkeypatch.delenv("PICAR_DESKTOP_LOCK", raising=False)  # the suite's own hold
    lock_env = {"PICAR_DESKTOP_LOCK_DIR": str(d)}
    return run, p, lock_env, d


def holder(lock_env, go):
    """Another session's shared job, holding until the file `go` exists."""
    env = {k: v for k, v in os.environ.items() if k != "PICAR_DESKTOP_LOCK"}
    return sp.Popen([sys.executable, LOCK_TOOL, "shared", "--why", "a session's training", "--",
                     sys.executable, "-c",
                     f"import os, time\nwhile not os.path.exists({str(go)!r}): time.sleep(0.05)"],
                    env={**env, **lock_env})


def status(lock_env):
    env = {k: v for k, v in os.environ.items() if k != "PICAR_DESKTOP_LOCK"}
    return sp.run([sys.executable, LOCK_TOOL, "status"], env={**env, **lock_env},
                  capture_output=True, text=True, timeout=20).stdout


def until(pred, timeout=20.0):
    end = time.monotonic() + timeout
    while not pred():
        if time.monotonic() > end:
            raise AssertionError("timed out")
        time.sleep(0.05)


def test_a_free_desktop_runs_the_suite_under_the_exclusive_lock(locked):
    run, p, lock_env, d = locked
    r, latest, ran = run(**lock_env)
    assert r.returncode == 0 and " PASS " in latest.splitlines()[0], latest
    token = (p["out"].parent / (p["out"].name + ".token")).read_text().strip()
    assert token.startswith("exclusive:"), token      # live tests' locks pass through
    assert "unlocked" not in latest
    assert status(lock_env).strip() == "FREE"          # let go at the end
    assert not os.listdir(d / "heavy-run.d")


def test_a_busy_desktop_skips_the_night_without_a_fail_or_an_email(locked, tmp_path):
    run, p, lock_env, d = locked
    h = holder(lock_env, tmp_path / "go")
    until(lambda: status(lock_env).startswith("SHARED"))
    try:
        r, latest, ran = run(NIGHTLY_LOCK_WAIT_S="1", NIGHTLY_LOCK_RETRY_S="0.2",
                             NIGHTLY_EMAIL="me@example.com", **lock_env)
    finally:
        (tmp_path / "go").write_text("")
        h.wait(timeout=20)
    first = latest.splitlines()[0]
    assert " SKIPPED  nightly: desktop busy for 1 s: " in first and "training" in first, first
    assert r.returncode == 126 and ran == ""            # pytest never started
    assert not p["mails"].exists(), "a skipped night was emailed"


def test_the_night_waits_for_a_holder_that_finishes(locked, tmp_path):
    run, p, lock_env, d = locked
    h = holder(lock_env, tmp_path / "go")
    until(lambda: status(lock_env).startswith("SHARED"))
    proc = p["start"](NIGHTLY_LOCK_WAIT_S="60", NIGHTLY_LOCK_RETRY_S="0.2", **lock_env)
    _until_running(p)
    time.sleep(1.0)
    assert not p["out"].exists(), "the suite ran beside a holder"
    (tmp_path / "go").write_text("")
    h.wait(timeout=20)
    proc.communicate(timeout=60)
    assert proc.returncode == 0 and " PASS " in p["latest"].read_text().splitlines()[0]


def test_a_stop_during_the_wait_reads_stopped(locked, tmp_path):
    run, p, lock_env, d = locked
    h = holder(lock_env, tmp_path / "go")
    until(lambda: status(lock_env).startswith("SHARED"))
    try:
        proc = p["start"](NIGHTLY_LOCK_WAIT_S="600", NIGHTLY_LOCK_RETRY_S="30", **lock_env)
        _until_running(p)
        time.sleep(1.0)                                 # inside the 30 s retry sleep
        t = time.monotonic()
        os.killpg(proc.pid, 15)
        proc.communicate(timeout=30)
        assert time.monotonic() - t < 10, "the stop waited for the retry"
    finally:
        (tmp_path / "go").write_text("")
        h.wait(timeout=20)
    first = p["latest"].read_text().splitlines()[0]
    assert " STOPPED " in first and not p["out"].exists(), first


def test_a_commit_without_the_tool_runs_unlocked(nightly, tmp_path, monkeypatch):  # noqa: F811
    """A guard: a commit before 3.128 has no tools/desktop_lock.py."""
    run, push, p = nightly
    monkeypatch.delenv("PICAR_DESKTOP_LOCK", raising=False)
    r, latest, ran = run(PICAR_DESKTOP_LOCK_DIR=str(tmp_path))
    assert r.returncode == 0 and " PASS " in latest.splitlines()[0]
    log = next((p["latest"].parent).glob("nightly-2*.log")).read_text()
    assert "desktop lock: none in this commit" in log
