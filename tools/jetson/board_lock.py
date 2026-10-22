"""
tools/jetson/board_lock.py

3.93: one session on the Jetson at a time. The lock lives ON THE BOARD
(`~/.picar-board.lock`, `flock(2)`), so every session that reaches the board
over ssh -- laptop, second Mac, cloud -- sees the same one, and a hold taken
by hand (`flock ~/.picar-board.lock -c ...` plus a line in
`~/.picar-board.owner`) excludes the tool's and the other way round.

A small holder runs on the board and keeps the lock while its ssh stdin is
open. It lets go when stdin closes (the local process ended or was killed),
when no heartbeat byte has come for the silence limit (a dropped network),
or at the hold's declared maximum -- so a dead session cannot keep the
board. A board-side bench (`board_lock_check.sh`) holds the lock itself for
as long as it runs, so a bench outliving its session still keeps the board.

    python tools/jetson/board_lock.py status
    python tools/jetson/board_lock.py run --why "3.92 VE run" -- <command...>
    python tools/jetson/board_lock.py hold --why "3.81 bench" [--max 7200]
    eval "$(python tools/jetson/board_lock.py env)"    # PICAR_BOARD_TOKEN
    python tools/jetson/board_lock.py release

Exit codes: 75 when another session holds the board (its owner's line is
printed; `--wait S` waits instead), 1 when the board could not be reached.
The command under `run`, and anything that reads `env` after `hold`, gets
PICAR_BOARD_TOKEN; pass it to board-side scripts
(`ssh picar-jetson "PICAR_BOARD_TOKEN=$PICAR_BOARD_TOKEN bash ..."`), whose
`board_lock_check.sh` refuses to run under another session's hold.

Env, for tests: PICAR_BOARD_SSH (default ssh to picar-jetson),
PICAR_BOARD_LOCK_DIR (default `~` on the board), PICAR_BOARD_PYTHON
(python3), PICAR_BOARD_HEARTBEAT_S (20; 0 sends none, so a hold ends at the
silence limit), PICAR_BOARD_SILENCE_S (90), PICAR_BOARD_WHO,
PICAR_BOARD_STATE (the hold's local state file; default in the git dir of
the checkout this file is in), PICAR_BOARD_STOP_S (20: how long `release`
waits for a keeper).
"""

import argparse
import datetime
import json
import os
import platform
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

BUSY = 75
UNREACHABLE = 1
SSH = ("ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 "
       "-o ServerAliveCountMax=4 picar-jetson")
# A probe (`status`, the bench check) takes the lock for microseconds: an
# acquirer retries this long before it calls the board busy.
GRACE_S = 1.5

# Runs on the board (python3 -c). argv: lock, owner, base line, token, wait_s,
# max_s, silence_s. The line's `until` is written here, on the board's clock,
# because the bench check compares it with the board's clock.
HOLDER = r"""
import datetime, fcntl, os, select, sys, time
lock, owner = (os.path.expanduser(a) for a in sys.argv[1:3])
base, token = sys.argv[3], sys.argv[4]
wait_s, max_s, silence_s = (float(a) for a in sys.argv[5:8])
fd = sys.stdin.fileno()
f = open(lock, "a")

def who():
    try:
        with open(owner) as o:
            return o.read().strip()
    except OSError:
        return ""

end, said = time.monotonic() + max(wait_s, %(grace)r), False
while True:
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        break
    except BlockingIOError:
        if time.monotonic() >= end:
            print("BUSY " + (who() or "held, no owner line"), flush=True)
            sys.exit(75)
        if wait_s > %(grace)r and not said:
            print("WAITING " + (who() or "held, no owner line"), flush=True)
            said = True
        r, _, _ = select.select([fd], [], [], 0.2)
        if r and not os.read(fd, 4096):
            sys.exit(0)                       # given up while waiting
until = datetime.datetime.fromtimestamp(time.time() + max_s).astimezone()
line = f"{base}, until {until.isoformat(timespec='seconds')}, token {token}"
tmp = owner + ".tmp"
with open(tmp, "w") as o:
    o.write(line + "\n")
os.replace(tmp, owner)
print("HELD " + line, flush=True)
deadline = time.monotonic() + max_s
last, why = time.monotonic(), "max"
while True:
    now = time.monotonic()
    if now >= deadline:
        why = "max"
        break
    if now - last >= silence_s:
        why = "silence"
        break
    r, _, _ = select.select([fd], [], [], max(0.0, min(deadline - now, last + silence_s - now)))
    if r:
        if not os.read(fd, 4096):
            why = "closed"
            break
        last = time.monotonic()
if who() == line:
    open(owner, "w").close()
try:
    print("RELEASED " + why, flush=True)
except Exception:
    pass
""" % {"grace": GRACE_S}

# Runs on the board: FREE, or HELD and the owner's line.
STATUS = r"""
import fcntl, os, sys
lock, owner = (os.path.expanduser(a) for a in sys.argv[1:3])
f = open(lock, "a")
try:
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    print("FREE")
except BlockingIOError:
    try:
        with open(owner) as o:
            line = o.read().strip()
    except OSError:
        line = ""
    print("HELD " + (line or "no owner line"))
"""


def paths():
    d = os.environ.get("PICAR_BOARD_LOCK_DIR", "~")
    return f"{d}/.picar-board.lock", f"{d}/.picar-board.owner"


def remote(code: str, *args) -> list:
    py = os.environ.get("PICAR_BOARD_PYTHON", "python3")
    return shlex.split(os.environ.get("PICAR_BOARD_SSH", SSH)) + [shlex.join([py, "-c", code, *map(str, args)])]


def iso(t: float) -> str:
    return datetime.datetime.fromtimestamp(t).astimezone().isoformat(timespec="seconds")


def who() -> str:
    if os.environ.get("PICAR_BOARD_WHO"):
        return os.environ["PICAR_BOARD_WHO"]
    try:
        branch = subprocess.run(["git", "-C", os.path.dirname(os.path.abspath(__file__)),
                                 "rev-parse", "--abbrev-ref", "HEAD"], capture_output=True,
                                text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        branch = ""
    return f"{branch or 'session'} pid {os.getpid()}"


def owner_line(why: str) -> str:
    """The other session's by-hand format. The holder adds `until` (on the
    board's clock) and the token, which `board_lock_check.sh` reads back."""
    host = "Mac" if platform.system() == "Darwin" else socket.gethostname().split(".")[0]
    return f"{who()} ({host}): {why}, since {iso(time.time())}".replace("\n", " ")


class Hold:
    """One holder on the board, kept while this object's process lives."""

    def __init__(self, why: str, wait_s: float, max_s: float):
        self.token = secrets.token_hex(8)
        self.line = owner_line(why)       # the board's full line once HELD
        lock, owner = paths()
        silence = float(os.environ.get("PICAR_BOARD_SILENCE_S", 90))
        # Its own session: a Ctrl-C, a closed terminal or `timeout` signalling
        # the foreground group must not end the hold before the command.
        self.proc = subprocess.Popen(
            remote(HOLDER, lock, owner, self.line, self.token, wait_s, max_s, silence),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, start_new_session=True)
        self._reader = None
        self.busy_line = ""
        self.unreachable = ""
        self.released_why = None
        self._beat = None

    def acquire(self) -> bool:
        """Blocks until HELD (True), or BUSY / a holder that never answered
        (False; `busy_line` or `unreachable` says which)."""
        for raw in self.proc.stdout:
            msg = raw.rstrip("\n")
            if msg.startswith("HELD "):
                self.line = msg[5:]
                self._start_heartbeat()
                self._reader = threading.Thread(target=self._read_rest, daemon=True)
                self._reader.start()
                return True
            if msg.startswith("WAITING "):
                print(f"board busy, waiting: {msg[8:]}", file=sys.stderr, flush=True)
            elif msg.startswith("BUSY "):
                self.busy_line = msg[5:]
                break
        self.proc.wait()
        if not self.busy_line:
            self.unreachable = f"the board's holder exited {self.proc.returncode} (ssh failed?)"
        return False

    def _read_rest(self):
        for raw in self.proc.stdout:
            if raw.startswith("RELEASED "):
                self.released_why = raw.split(" ", 1)[1].strip()

    def _start_heartbeat(self):
        every = float(os.environ.get("PICAR_BOARD_HEARTBEAT_S", 20))
        if every <= 0:
            return
        stop = threading.Event()

        def beat():
            while not stop.wait(every):
                try:
                    self.proc.stdin.write(".")
                    self.proc.stdin.flush()
                except (BrokenPipeError, ValueError, OSError):
                    return
        self._beat = stop
        threading.Thread(target=beat, daemon=True).start()

    def release(self, wait: bool = True):
        """Close the holder's stdin (it lets go at EOF). `wait=False` is for a
        signal handler: no blocking wait there, the local ssh is killed."""
        if self._beat:
            self._beat.set()
        try:
            self.proc.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass
        if not wait:
            try:
                self.proc.kill()
            except OSError:
                pass
            return
        try:
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()

    def alive(self) -> bool:
        return self.proc.poll() is None

    def lost(self) -> str:
        if self._reader is not None:
            self._reader.join(timeout=2)   # its RELEASED line names the cause
        return f"the board lock was lost ({self.released_why or 'the holder ended'})"


def state_path() -> str:
    """Per checkout: the git dir of the worktree this file is in, whatever the
    cwd -- one session's `release` never finds another's hold."""
    if os.environ.get("PICAR_BOARD_STATE"):
        return os.environ["PICAR_BOARD_STATE"]
    try:
        gd = subprocess.run(["git", "-C", os.path.dirname(os.path.abspath(__file__)),
                             "rev-parse", "--absolute-git-dir"], capture_output=True,
                            text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        gd = ""
    base = gd or os.path.expanduser("~/.cache")
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, "picar-board-hold.json")


def read_state():
    try:
        with open(state_path()) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def write_state(d: dict) -> None:
    path = state_path()
    with open(path + ".tmp", "w") as f:
        json.dump(d, f)
    os.replace(path + ".tmp", path)


def is_keeper(pid) -> bool:
    """A live `_keep` of this tool for THIS state file -- not any process that
    reused the pid of one that died without removing its state (a reboot,
    SIGKILL), nor another worktree's keeper."""
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        cmd = subprocess.run(["ps", "-ww", "-o", "command=", "-p", str(pid)], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return "board_lock.py" in cmd and "_keep" in cmd and f"--state {state_path()}" in cmd


def stop_keeper(pid: int) -> bool:
    """True once no keeper of ours runs under `pid`."""
    if not is_keeper(pid):
        return True
    os.kill(pid, signal.SIGTERM)
    end = time.monotonic() + float(os.environ.get("PICAR_BOARD_STOP_S", 20))
    while time.monotonic() < end:
        try:
            if os.waitpid(pid, os.WNOHANG)[0] == pid:
                return True               # our own child (hold's give-up path): reaped
        except ChildProcessError:
            pass                          # not ours: kill(0) below
        try:
            os.kill(pid, 0)
        except OSError:
            return True
        time.sleep(0.2)
    return not is_keeper(pid)


def cmd_status(_):
    lock, owner = paths()
    try:
        r = subprocess.run(remote(STATUS, lock, owner), capture_output=True, text=True, timeout=30)
        out = r.stdout.strip() or f"status failed: {r.stderr.strip()}"
    except subprocess.TimeoutExpired:
        out = "status failed: the board did not answer in 30 s"
    print(out)
    return 0 if out == "FREE" else BUSY if out.startswith("HELD") else UNREACHABLE


def cmd_run(a):
    if not a.cmd:
        print("run: give a command after --", file=sys.stderr)
        return 2
    h = Hold(a.why, a.wait, a.max)
    try:
        held = h.acquire()
    except KeyboardInterrupt:
        h.release()
        return 130
    if not held:
        if h.unreachable:
            print(f"board unreachable: {h.unreachable}", file=sys.stderr)
            return UNREACHABLE
        print(f"board busy: {h.busy_line}", file=sys.stderr)
        return BUSY
    print(f"board lock held: {h.line}", file=sys.stderr, flush=True)
    # `run` never ends while its command runs, and never moves groups: the
    # command sits where any child would, so the terminal, pipelines and
    # supervisors treat it as usual. A Ctrl-C or a closed terminal reaches
    # the command from the tty already, so `run` only keeps waiting; a
    # SIGTERM to `run` (`kill`, `timeout`) goes on to the command, every
    # time, and one that lands before the command started is delivered
    # once it has. The holder is in a session of its own, so the hold
    # outlasts the command's teardown. Known limits (the spec): a SIGINT
    # sent to `run` alone is not forwarded; grandchildren are reached only
    # through the command; a board-side bench keeps the board itself.
    state = {"child": None, "pending": []}

    def on_term(signum, _frame):
        child = state["child"]
        if child is None:
            state["pending"].append(signum)
        elif child.poll() is None:
            child.send_signal(signum)

    old = {signal.SIGTERM: signal.signal(signal.SIGTERM, on_term),
           signal.SIGINT: signal.signal(signal.SIGINT, lambda *_: None),
           signal.SIGHUP: signal.signal(signal.SIGHUP, lambda *_: None)}
    try:
        child = subprocess.Popen(a.cmd, env={**os.environ, "PICAR_BOARD_TOKEN": h.token})
    except OSError as e:
        for s_, f_ in old.items():
            signal.signal(s_, f_)
        h.release()
        print(f"run: {e}; board lock released", file=sys.stderr)
        return 127
    state["child"] = child
    for signum in state["pending"]:
        child.send_signal(signum)

    warned = False
    try:
        while True:
            try:
                rc = child.wait(timeout=1.0)
                break
            except subprocess.TimeoutExpired:
                pass
            if not warned and not h.alive():
                # At --max or the silence limit. A board-side bench keeps the
                # board itself (board_lock_check.sh); anything else is now
                # unprotected, so say so loudly rather than kill it mid-run.
                print(f"WARNING: {h.lost()} while the command runs; another "
                      "session may now take the board", file=sys.stderr, flush=True)
                warned = True
    finally:
        for s_, f_ in old.items():
            signal.signal(s_, f_)
        h.release()
        print("board lock released", file=sys.stderr)
    return rc


def cmd_keep(a):
    """The detached child behind `hold`: holds until SIGTERM or the holder ends."""
    h = Hold(a.why, a.wait, a.max)
    stop = threading.Event()

    def on_term(*_):
        stop.set()
        h.release(wait=False)      # also ends an acquire still waiting
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    write_state({"pid": os.getpid(), "waiting": True})
    if not h.acquire():
        if not stop.is_set():
            write_state({"pid": os.getpid(), "busy": h.busy_line,
                         "unreachable": h.unreachable})
        return BUSY if h.busy_line else UNREACHABLE
    if stop.is_set():
        h.release()
        cur = read_state()
        if cur and cur.get("pid") == os.getpid():
            os.remove(state_path())
        return 0
    write_state({"pid": os.getpid(), "token": h.token, "line": h.line})
    while not stop.wait(1.0) and h.alive():
        pass
    h.release()
    cur = read_state()
    if cur and cur.get("pid") == os.getpid():
        if stop.is_set():
            os.remove(state_path())
        else:
            # The hold ended on its own (max, silence): say so to `env` / `hold`.
            write_state({"pid": os.getpid(), "lost": h.lost()})
    return 0


def cmd_hold(a):
    cur = read_state()
    if cur and cur.get("token") and is_keeper(cur.get("pid")):
        print(f"this worktree already holds the board: {cur['line']}", file=sys.stderr)
        return 0
    if cur and is_keeper(cur.get("pid")):
        # A keeper still waiting from an earlier hold: never two of them.
        if not stop_keeper(cur["pid"]):
            print(f"hold: an earlier keeper (pid {cur['pid']}) did not stop", file=sys.stderr)
            return UNREACHABLE
    path = state_path()
    if os.path.exists(path):
        os.remove(path)
    args = [sys.executable, os.path.abspath(__file__), "_keep", "--state", path,
            "--why", a.why, "--wait", str(a.wait), "--max", str(a.max)]
    log = open(path + ".log", "w")
    keeper = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                              start_new_session=True,
                              env={**os.environ, "PICAR_BOARD_STATE": path})
    end = time.monotonic() + a.wait + 60
    try:
        while time.monotonic() < end:
            cur = read_state()
            if cur and cur.get("token") and keeper.poll() is None:
                print(f"board lock held: {cur['line']}", file=sys.stderr)
                return 0
            if cur and ("busy" in cur or "unreachable" in cur):
                os.remove(path)
                if cur.get("busy"):
                    print(f"board busy: {cur['busy']}", file=sys.stderr)
                    return BUSY
                print(f"board unreachable: {cur['unreachable']}", file=sys.stderr)
                return UNREACHABLE
            if keeper.poll() is not None:
                cur = read_state()          # it may have written busy just now
                if cur and ("busy" in cur or "unreachable" in cur):
                    continue
                if cur and cur.get("lost"):
                    os.remove(path)
                    print(f"hold: taken, then {cur['lost']} at once", file=sys.stderr)
                    return UNREACHABLE
                print(f"hold: the keeper exited {keeper.returncode} (see {path}.log)",
                      file=sys.stderr)
                return UNREACHABLE
            time.sleep(0.2)
        print(f"hold: no answer from the board (see {path}.log)", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    # Given up: the keeper must not take the board later behind our back.
    if not stop_keeper(keeper.pid):
        print(f"hold: the keeper (pid {keeper.pid}) did not stop; state kept for `release`",
              file=sys.stderr)
        return UNREACHABLE
    if os.path.exists(path):
        os.remove(path)
    return UNREACHABLE


def cmd_release(_):
    cur = read_state()
    if not cur or not cur.get("pid"):
        print("this worktree holds no board lock", file=sys.stderr)
        return 0
    if cur.get("lost"):
        print(f"note: {cur['lost']} before this release", file=sys.stderr)
    if not stop_keeper(cur["pid"]):
        print(f"release: the keeper (pid {cur['pid']}) did not stop; state kept", file=sys.stderr)
        return 1
    if os.path.exists(state_path()):
        os.remove(state_path())
    print("board lock released", file=sys.stderr)
    return 0


def cmd_env(_):
    cur = read_state()
    if cur and cur.get("lost"):
        print(f"this worktree's hold is gone: {cur['lost']}", file=sys.stderr)
        return 1
    if not cur or not cur.get("token") or not is_keeper(cur.get("pid")):
        print("this worktree holds no board lock (take one with `hold`)", file=sys.stderr)
        return 1
    print(f"export PICAR_BOARD_TOKEN={cur['token']}")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    sub = p.add_subparsers(dest="verb", required=True)
    sub.add_parser("status")
    for name in ("run", "hold", "_keep"):
        s = sub.add_parser(name)
        s.add_argument("--why", default="bench")
        s.add_argument("--wait", type=float, default=0.0, help="seconds to wait for a busy board")
        s.add_argument("--max", type=float, default=4 * 3600.0, help="the hold ends by itself after this")
        if name == "run":
            s.add_argument("cmd", nargs=argparse.REMAINDER)
        if name == "_keep":
            s.add_argument("--state", required=True)
    sub.add_parser("release")
    sub.add_parser("env")
    a = p.parse_args(argv)
    if a.verb == "run" and a.cmd[:1] == ["--"]:
        a.cmd = a.cmd[1:]
    return {"status": cmd_status, "run": cmd_run, "hold": cmd_hold, "_keep": cmd_keep,
            "release": cmd_release, "env": cmd_env}[a.verb](a)


if __name__ == "__main__":
    sys.exit(main())
