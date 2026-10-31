"""
tools/desktop_lock.py

3.128: who may load this machine, and how. One file, `.git/heavy-run.lock`
(rule 7's lock, the same file `tests/demo_explore.heavy_lock()` always
took), now held two ways with `flock(2)`:

* exclusive -- a run whose numbers depend on timing: live runs, `ready
  --full`, anything with a wall-clock bar. It waits for every holder, then
  has the machine alone.
* shared -- other heavy work: the fast tier (`ready`, the push gate),
  training, evaluations, docker builds. Any number together; none while an
  exclusive holder runs.

Linux `flock` lets new shared holders pass a waiting exclusive one, so every
acquirer first takes `.git/heavy-run.gate` exclusively, then the lock, then
lets the gate go; an exclusive acquirer holds the gate while it waits, and
shared ones queue behind it.

A holder's children see its mode in PICAR_DESKTOP_LOCK. Under an exclusive
holder any request passes through (`ready --full`'s live tests call
`stack()`); under a shared one a shared request passes and an exclusive one
raises `Nested` at once. On Linux a request also passes through when an
ANCESTOR process holds the file (a bash `flock ... ready`), read from
/proc/locks, so no one deadlocks on their own parent. A plain exclusive
`flock` on the file by code that predates this (unrebased worktrees) still
excludes and is excluded.

    python tools/desktop_lock.py status
    python tools/desktop_lock.py shared --why "3.125 sweep" -- <command...>
    python tools/desktop_lock.py exclusive --why "3.126 live run" -- <command...>

`--no-wait` exits 75 when the lock is busy. The lock is held by this
wrapper, not inherited by the command (daemons the command starts would
keep it, as the nightly's once did), so a `kill -9` of the wrapper frees it
while the command runs on. PICAR_HEAVY_LOCK=0 skips it, as before.
PICAR_DESKTOP_LOCK_DIR moves the files (tests).
"""

import argparse
import fcntl
import os
import signal
import subprocess
import sys
import time
from contextlib import contextmanager

BUSY = 75
ENV = "PICAR_DESKTOP_LOCK"
MODES = ("shared", "exclusive")
LOCK, GATE, NOTES = "heavy-run.lock", "heavy-run.gate", "heavy-run.d"
# Evaluations grep stderr for this line (3.79, 3.88, 3.96 harnesses).
WAIT_LINE = "waiting for the heavy-run lock"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Busy(Exception):
    """The lock is held and the caller asked not to wait."""


class Nested(Exception):
    """A shared holder's child asked for the exclusive lock."""


def lock_dir() -> str:
    d = os.environ.get("PICAR_DESKTOP_LOCK_DIR")
    if d:
        return d
    # Without the hook's GIT_DIR and friends, which may be relative to
    # another cwd; outside a checkout, /tmp, as heavy_lock() always did.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"], capture_output=True,
                            text=True, cwd=ROOT, env=env).stdout.strip() or "/tmp"
    return os.path.abspath(os.path.join(ROOT, common))


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _ancestors() -> set:
    out, pid = set(), os.getppid()
    while pid > 1 and pid not in out:
        out.add(pid)
        try:
            # The command name is in parentheses and may hold spaces.
            pid = int(open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def _rows(path: str):
    """(pid, "exclusive"|"shared") for each granted flock on `path`, or
    None without /proc/locks (macOS). A waiter's row starts "->" and is
    skipped by the field check."""
    try:
        st = os.stat(path)
        rows = open("/proc/locks").read().splitlines()
    except OSError:
        return None
    out = []
    for row in rows:
        f = row.split()
        if len(f) < 6 or f[1] != "FLOCK":
            continue
        try:
            pid = int(f[4])
            major, minor, ino = f[5].split(":")
            if (int(ino), int(major, 16), int(minor, 16)) != (
                    st.st_ino, os.major(st.st_dev), os.minor(st.st_dev)):
                continue
        except ValueError:
            continue
        out.append((pid, "exclusive" if f[3] == "WRITE" else "shared"))
    return out


def ancestor_mode(path: str):
    """"exclusive" or "shared" when a parent process holds `path` with
    flock, else None (and always None without /proc/locks)."""
    rows = _rows(path)
    if not rows:
        return None
    st = os.stat(path)
    mine = _ancestors()
    # A descriptor inherited from the shell (`exec 9>lock; flock 9`): the
    # row names the flock(1) process, long gone, but the lock is on our fd.
    try:
        fds = os.listdir("/proc/self/fd")
    except OSError:
        fds = []
    for fd in fds:
        try:
            s = os.stat(f"/proc/self/fd/{fd}")
        except OSError:
            continue
        if (s.st_ino, s.st_dev) == (st.st_ino, st.st_dev):
            inherited_fd = True
            break
    else:
        inherited_fd = False
    held = None
    for pid, m in rows:
        # With such a descriptor, a row whose taker is gone is ours too;
        # a live stranger's row never is.
        if pid in mine or (inherited_fd and not _alive(pid)):
            held = "exclusive" if m == "exclusive" else held or "shared"
    return held


def _token(mode: str) -> str:
    """PICAR_DESKTOP_LOCK's value for children: the mode and the holder's
    pid, so a value that outlived its hold (a daemon, a tmux started under
    it) is not trusted."""
    return f"{mode}:{os.getpid()}"


def env_mode(path: str):
    """The mode PICAR_DESKTOP_LOCK names, if its holder still holds `path`
    (alive, and on Linux holding a flock on it), else None."""
    mode, _, pid = os.environ.get(ENV, "").partition(":")
    if mode not in MODES or not pid.isdigit() or not _alive(int(pid)):
        return None
    rows = _rows(path)
    if rows is not None and not any(p == int(pid) and (m == mode or m == "exclusive")
                                    for p, m in rows):
        return None
    return mode


def inherited(mode: str, path: str):
    """The mode an ancestor already holds that covers `mode`, else None.
    Raises Nested when only a shared hold covers an exclusive request."""
    held = env_mode(path)
    if held != "exclusive":
        held = ancestor_mode(path) or held
    if held == "exclusive" or (held == "shared" and mode == "shared"):
        return held
    if held == "shared":
        raise Nested("an exclusive desktop lock was asked for under a shared one "
                     f"({ENV}=shared or a parent's flock): the parent must take "
                     "exclusive, or run this outside it")
    return None


def holders(d: str) -> list:
    """Live holders' lines; a dead pid's line is removed."""
    out = []
    nd = os.path.join(d, NOTES)
    for name in sorted(os.listdir(nd)) if os.path.isdir(nd) else []:
        p = os.path.join(nd, name)
        try:
            pid = int(name.split(".")[0])
        except ValueError:
            continue
        if not _alive(pid):
            try:
                os.unlink(p)
            except OSError:
                pass
            continue
        try:
            out.append(open(p).read().strip())
        except OSError:
            pass
    return out


def _describe(d: str) -> str:
    lines = holders(d)
    if lines:
        return "; ".join(lines)
    try:                       # a holder that predates 3.128 writes only here
        return open(os.path.join(d, LOCK)).read().strip() or "held"
    except OSError:
        return "held"


def _take(fd: int, op: int, d: str, wait: bool, out) -> None:
    try:
        fcntl.flock(fd, op | fcntl.LOCK_NB)
        return
    except BlockingIOError:
        if not wait:
            raise Busy(_describe(d))
    print(f"{WAIT_LINE} ({os.path.join(d, LOCK)}): {_describe(d)}", file=out, flush=True)
    fcntl.flock(fd, op)


class Hold:
    """One hold, released by `release()` or when the process exits."""

    def __init__(self, mode, fd, note, path):
        self.mode, self._fd, self._note, self._path = mode, fd, note, path

    def release(self) -> None:
        if self._fd is None:
            return
        try:
            os.unlink(self._note)
        except OSError:
            pass
        if self.mode == "exclusive":
            os.ftruncate(self._fd, 0)
        fcntl.flock(self._fd, fcntl.LOCK_UN)
        os.close(self._fd)
        self._fd = None


_seq = 0


def acquire(mode: str, why: str = "", wait: bool = True, out=None):
    """A Hold, or None when an ancestor's hold already covers `mode` (or
    PICAR_HEAVY_LOCK=0). Raises Busy (wait=False) or Nested."""
    global _seq
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}, not {mode!r}")
    out = out or sys.stderr
    if os.environ.get("PICAR_HEAVY_LOCK") == "0":
        return None
    d = lock_dir()
    os.makedirs(os.path.join(d, NOTES), exist_ok=True)
    path = os.path.join(d, LOCK)
    open(path, "a").close()
    if inherited(mode, path):
        return None
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        gate = os.open(os.path.join(d, GATE), os.O_CREAT | os.O_RDWR, 0o644)
        try:
            if gate_waiting(d) and wait:
                print(f"{WAIT_LINE} ({path}): an exclusive job is queued ahead; "
                      f"holding now: {_describe(d)}", file=out, flush=True)
            _take(gate, fcntl.LOCK_EX, d, wait, out)
            _take(fd, fcntl.LOCK_EX if mode == "exclusive" else fcntl.LOCK_SH, d, wait, out)
        finally:
            os.close(gate)              # closing lets the gate go
        _seq += 1
        line = (f"{mode} pid {os.getpid()} {why or ' '.join(sys.argv)[:200]} "
                f"since {time.ctime()}")
        note = os.path.join(d, NOTES, f"{os.getpid()}.{_seq}")
        h = Hold(mode, fd, note, path)
        with open(note, "w") as f:
            f.write(line + "\n")
        if mode == "exclusive":         # for readers that predate 3.128
            os.ftruncate(fd, 0)
            os.pwrite(fd, (line + "\n").encode(), 0)
        return h
    except BaseException:
        try:
            os.unlink(note)
        except (NameError, OSError):
            pass
        os.close(fd)                    # closing lets the lock go
        raise


@contextmanager
def held(mode: str, why: str = "", wait: bool = True):
    """Hold the lock for a block; yields PICAR_DESKTOP_LOCK's value for
    children: this hold's token, the inherited value, or None when nothing
    is held (PICAR_HEAVY_LOCK=0)."""
    h = acquire(mode, why, wait)
    try:
        yield _token(mode) if h is not None else os.environ.get(ENV)
    finally:
        if h is not None:
            h.release()


_LIFE = []


def hold_for_life(mode: str, why: str = "") -> None:
    """Hold until this process exits, and tell its children (heavy_lock())."""
    if _LIFE and (_LIFE[0] == "exclusive" or mode == "shared"):
        return
    h = acquire(mode, why)
    _LIFE[:] = [mode, h]
    if h is not None:
        os.environ[ENV] = _token(mode)


def state(d: str) -> str:
    """FREE, SHARED or EXCLUSIVE, probed without waiting."""
    path = os.path.join(d, LOCK)
    if not os.path.exists(path):
        return "FREE"
    fd = os.open(path, os.O_RDONLY)
    try:
        for op, name in ((fcntl.LOCK_EX, "FREE"), (fcntl.LOCK_SH, "SHARED")):
            try:
                fcntl.flock(fd, op | fcntl.LOCK_NB)
                fcntl.flock(fd, fcntl.LOCK_UN)
                return name
            except BlockingIOError:
                continue
        return "EXCLUSIVE"
    finally:
        os.close(fd)


def gate_waiting(d: str) -> bool:
    """Whether an exclusive acquirer is queued (it holds the gate)."""
    path = os.path.join(d, GATE)
    if not os.path.exists(path):
        return False
    fd = os.open(path, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    except BlockingIOError:
        return True
    finally:
        os.close(fd)


def cmd_status() -> int:
    d = lock_dir()
    s = state(d)
    print(s)
    if s != "FREE":
        lines = holders(d)
        for line in lines or [_describe(d)]:
            print(f"  {line}")
    if gate_waiting(d):
        print("  an exclusive job is queued; new shared jobs wait behind it")
    return 0


def cmd_run(mode: str, why: str, cmd: list, wait: bool) -> int:
    try:
        h = acquire(mode, why, wait)
    except Busy as e:
        print(f"desktop lock busy: {e}", file=sys.stderr)
        return BUSY
    except Nested as e:
        print(f"desktop lock: {e}", file=sys.stderr)
        return 1
    env = dict(os.environ)
    if h is not None:
        env[ENV] = _token(mode)
    try:
        p = subprocess.Popen(cmd, env=env)
        signal.signal(signal.SIGTERM, lambda *_: p.send_signal(signal.SIGTERM))
        while True:
            try:
                return p.wait()
            except KeyboardInterrupt:   # the child got the terminal's too
                continue
    finally:
        if h is not None:
            h.release()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    for m in MODES:
        p = sub.add_parser(m)
        p.add_argument("--why", default="")
        p.add_argument("--no-wait", action="store_true")
        p.add_argument("command", nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    if a.cmd == "status":
        return cmd_status()
    cmd = a.command[1:] if a.command[:1] == ["--"] else a.command
    if not cmd:
        ap.error("give a command after --")
    return cmd_run(a.cmd, a.why, cmd, not a.no_wait)


if __name__ == "__main__":
    sys.exit(main())
