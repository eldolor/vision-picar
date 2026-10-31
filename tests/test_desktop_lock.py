"""tools/desktop_lock.py -- 3.128's shared/exclusive desktop lock, on real
flock(2) between real processes, in a temporary lock directory."""

import os
import shutil
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import desktop_lock as dl  # noqa: E402

TOOL = os.path.join(ROOT, "tools", "desktop_lock.py")
pytestmark = pytest.mark.skipif(not hasattr(os, "fork"), reason="flock needs POSIX")

WORKER = """
import os, sys, time
log, name, go = sys.argv[1:4]
def w(s):
    fd = os.open(log, os.O_WRONLY | os.O_APPEND | os.O_CREAT)
    os.write(fd, (s + "\\n").encode())
    os.close(fd)
w(name + " start")
while go != "-" and not os.path.exists(go):
    time.sleep(0.02)
w(name + " end")
"""


@pytest.fixture
def box(tmp_path, monkeypatch):
    """A lock directory of its own, and an environment that holds nothing:
    the suite itself runs under `ready`'s shared hold."""
    d = tmp_path / "git"
    d.mkdir()
    monkeypatch.setenv("PICAR_DESKTOP_LOCK_DIR", str(d))
    monkeypatch.delenv(dl.ENV, raising=False)
    monkeypatch.delenv("PICAR_HEAVY_LOCK", raising=False)
    (tmp_path / "worker.py").write_text(WORKER)
    return tmp_path


def env():
    return dict(os.environ)


def start(box, mode, name, go="-", extra=()):
    """`desktop_lock.py <mode> -- worker`, logging `name start/end`; the
    worker waits for the file `go` unless it is "-"."""
    gate = str(box / go) if go != "-" else "-"
    return subprocess.Popen([sys.executable, TOOL, mode, *extra, "--why", name, "--",
                             sys.executable, str(box / "worker.py"), str(box / "log"),
                             name, gate], env=env(), stderr=subprocess.PIPE, text=True)


def log(box):
    p = box / "log"
    return p.read_text().split("\n")[:-1] if p.exists() else []


def until(pred, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    raise AssertionError("timed out")


def release(box, go):
    (box / go).write_text("")


def finish(*procs):
    for p in procs:
        assert p.wait(timeout=20) == 0, p.stderr.read()


def test_shared_holders_run_together_and_exclusive_runs_alone(box):
    s1, s2 = start(box, "shared", "S1", "g1"), start(box, "shared", "S2", "g2")
    until(lambda: {"S1 start", "S2 start"} <= set(log(box)))
    e = start(box, "exclusive", "E", "gE")
    time.sleep(0.5)
    assert "E start" not in log(box), "exclusive ran beside shared holders"
    release(box, "g1")
    release(box, "g2")
    until(lambda: "E start" in log(box))
    s3 = start(box, "shared", "S3")
    time.sleep(0.5)
    assert "S3 start" not in log(box), "shared ran beside an exclusive holder"
    release(box, "gE")
    finish(s1, s2, e, s3)
    L = log(box)
    assert L.index("E start") > max(L.index("S1 end"), L.index("S2 end"))
    assert L.index("S3 start") > L.index("E end")


def test_a_waiting_exclusive_is_not_starved_by_later_shared_jobs(box):
    s1 = start(box, "shared", "S1", "g1")
    until(lambda: "S1 start" in log(box))
    e = start(box, "exclusive", "E")
    until(lambda: dl.gate_waiting(str(box / "git")))
    s2 = start(box, "shared", "S2")
    time.sleep(0.5)
    assert "S2 start" not in log(box), "a later shared job passed the queued exclusive"
    release(box, "g1")
    finish(s1, e, s2)
    L = log(box)
    assert L.index("E start") < L.index("E end") < L.index("S2 start")


OLD_STYLE = """
import fcntl, os, sys, time
path, log, go = sys.argv[1:4]
f = open(path, "a+")
fcntl.flock(f, fcntl.LOCK_EX)
fd = os.open(log, os.O_WRONLY | os.O_APPEND | os.O_CREAT)
os.write(fd, b"OLD start\\n")
while go != "-" and not os.path.exists(go):
    time.sleep(0.02)
os.write(fd, b"OLD end\\n")
"""


def old_style(box, go="-"):
    return subprocess.Popen([sys.executable, "-c", OLD_STYLE, str(box / "git" / dl.LOCK),
                             str(box / "log"), str(box / go) if go != "-" else "-"],
                            env=env(), stderr=subprocess.PIPE, text=True)


def test_a_holder_that_predates_the_tool_still_excludes_and_is_excluded(box):
    old = old_style(box, "gO")
    until(lambda: "OLD start" in log(box))
    s, e = start(box, "shared", "S"), start(box, "exclusive", "E")
    time.sleep(0.5)
    assert not {"S start", "E start"} & set(log(box))
    release(box, "gO")
    finish(old, s, e)
    (box / "log").unlink()
    s = start(box, "shared", "S", "gS")
    until(lambda: "S start" in log(box))
    old = old_style(box)
    time.sleep(0.5)
    assert "OLD start" not in log(box), "an old-style exclusive ran beside a shared holder"
    release(box, "gS")
    finish(s, old)
    L = log(box)
    assert L.index("OLD start") > L.index("S end")


def nested(box, outer, inner):
    return subprocess.run([sys.executable, TOOL, outer, "--", sys.executable, TOOL, inner,
                           "--", sys.executable, "-c", "pass"],
                          env=env(), capture_output=True, text=True, timeout=20)


def test_requests_under_a_holder_pass_through_or_fail_at_once(box):
    assert nested(box, "exclusive", "exclusive").returncode == 0
    assert nested(box, "exclusive", "shared").returncode == 0
    assert nested(box, "shared", "shared").returncode == 0
    r = nested(box, "shared", "exclusive")
    assert r.returncode == 1 and "under a shared one" in r.stderr


def test_a_token_whose_holder_is_gone_or_holds_nothing_is_not_trusted(box):
    """A PICAR_DESKTOP_LOCK that outlived its hold (a daemon or tmux started
    under it) must not let a later job past a live holder."""
    e = start(box, "exclusive", "E", "gE")
    until(lambda: "E start" in log(box))
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()
    for token in (f"exclusive:{dead.pid}", f"exclusive:{os.getpid()}", "exclusive"):
        r = subprocess.run([sys.executable, TOOL, "shared", "--no-wait", "--", "true"],
                           env={**env(), dl.ENV: token}, capture_output=True, text=True,
                           timeout=20)
        assert r.returncode == dl.BUSY, (token, r.stderr)
    release(box, "gE")
    finish(e)


@pytest.mark.skipif(not shutil.which("flock") or not os.path.exists("/proc/locks"),
                    reason="flock(1) and /proc/locks are Linux's")
def test_a_shell_flock_around_the_tool_is_seen_not_deadlocked(box):
    lock = str(box / "git" / dl.LOCK)
    open(lock, "a").close()
    inner = f"{sys.executable} {TOOL} shared -- {sys.executable} -c pass"
    # flock(1) as the parent, then a descriptor the shell holds.
    for cmd in ([shutil.which("flock"), lock, "sh", "-c", inner],
                ["bash", "-c", f"exec 9>>{lock}; flock 9; {inner}"]):
        r = subprocess.run(cmd, env=env(), capture_output=True, text=True, timeout=20)
        assert r.returncode == 0, r.stderr


def test_no_wait_says_busy_and_status_names_the_holders(box):
    e = start(box, "exclusive", "the-live-run", "gE")
    until(lambda: "the-live-run start" in log(box))
    r = subprocess.run([sys.executable, TOOL, "shared", "--no-wait", "--", "true"],
                       env=env(), capture_output=True, text=True, timeout=20)
    assert r.returncode == dl.BUSY and "the-live-run" in r.stderr
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()
    (box / "git" / dl.NOTES / f"{dead.pid}.1").write_text("shared pid gone\n")
    out = subprocess.run([sys.executable, TOOL, "status"], env=env(),
                         capture_output=True, text=True, timeout=20).stdout
    assert out.startswith("EXCLUSIVE") and "exclusive pid" in out and "the-live-run" in out
    assert "gone" not in out and not (box / "git" / dl.NOTES / f"{dead.pid}.1").exists()
    release(box, "gE")
    finish(e)
    assert subprocess.run([sys.executable, TOOL, "status"], env=env(), capture_output=True,
                          text=True, timeout=20).stdout.strip() == "FREE"


HEAVY = """
import os, sys
sys.path.insert(0, sys.argv[1])
import tests.demo_explore as de
de.heavy_lock()
de.heavy_lock()                 # a second call in one process is a no-op
print(os.environ.get("PICAR_DESKTOP_LOCK"))
"""


def test_heavy_lock_is_the_exclusive_hold_and_keeps_its_line(box):
    s = start(box, "shared", "S", "gS")
    until(lambda: "S start" in log(box))
    p = subprocess.Popen([sys.executable, "-c", HEAVY, ROOT], cwd=ROOT, env=env(),
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    until(lambda: dl.gate_waiting(str(box / "git")))
    release(box, "gS")
    out, err = p.communicate(timeout=60)
    assert p.returncode == 0, err
    assert out.strip() == f"exclusive:{p.pid}" and dl.WAIT_LINE in err and "shared pid" in err
    finish(s)
    # PICAR_HEAVY_LOCK=0 skips it, even beside an exclusive holder.
    e = start(box, "exclusive", "E", "gE")
    until(lambda: "E start" in log(box))
    r = subprocess.run([sys.executable, "-c", HEAVY, ROOT], cwd=ROOT, timeout=60,
                       env={**env(), "PICAR_HEAVY_LOCK": "0"}, capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout.strip() == "None"
    release(box, "gE")
    finish(e)
