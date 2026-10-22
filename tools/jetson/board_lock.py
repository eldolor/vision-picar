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
board.

    python tools/jetson/board_lock.py status
    python tools/jetson/board_lock.py run --why "3.92 VE run" -- <command...>
    python tools/jetson/board_lock.py hold --why "3.81 bench" [--max 7200]
    eval "$(python tools/jetson/board_lock.py env)"    # PICAR_BOARD_TOKEN
    python tools/jetson/board_lock.py release

`run` and `hold` exit 75 when the board is busy (printing its owner's line),
or wait up to `--wait` seconds. The command under `run`, and anything that
reads `env` after `hold`, gets PICAR_BOARD_TOKEN; pass it to board-side
scripts (`ssh picar-jetson "PICAR_BOARD_TOKEN=$PICAR_BOARD_TOKEN bash ..."`),
whose `board_lock_check.sh` refuses to run under another session's hold.

Env (tests): PICAR_BOARD_SSH (default ssh to picar-jetson), PICAR_BOARD_LOCK_DIR
(default `~` on the board), PICAR_BOARD_PYTHON (python3), PICAR_BOARD_HEARTBEAT_S
(20; 0 sends none), PICAR_BOARD_SILENCE_S (90), PICAR_BOARD_WHO, PICAR_BOARD_STATE
(the hold's local state file; default in this worktree's git dir).
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
SSH = "ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=4 picar-jetson"

# Runs on the board (python3 -c). argv: lock, owner, line, wait_s, max_s, silence_s.
HOLDER = r"""
import fcntl, os, select, sys, time
lock, owner = (os.path.expanduser(a) for a in sys.argv[1:3])
line = sys.argv[3]
wait_s, max_s, silence_s = (float(a) for a in sys.argv[4:7])
fd = sys.stdin.fileno()
f = open(lock, "a")

def who():
    try:
        with open(owner) as o:
            return o.read().strip()
    except OSError:
        return ""

end, said = time.monotonic() + wait_s, False
while True:
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        break
    except BlockingIOError:
        if time.monotonic() >= end:
            print("BUSY " + (who() or "held, no owner line"), flush=True)
            sys.exit(75)
        if not said:
            print("WAITING " + (who() or "held, no owner line"), flush=True)
            said = True
        r, _, _ = select.select([fd], [], [], 2.0)
        if r and not os.read(fd, 4096):
            sys.exit(0)                       # given up while waiting
tmp = owner + ".tmp"
with open(tmp, "w") as o:
    o.write(line + "\n")
os.replace(tmp, owner)
print("HELD", flush=True)
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
"""

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
        branch = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"], capture_output=True,
                                text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        branch = ""
    return f"{branch or 'session'} pid {os.getpid()}"


def owner_line(why: str, max_s: float, token: str) -> str:
    """The other session's by-hand format, with the end and the token after it."""
    host = "Mac" if platform.system() == "Darwin" else socket.gethostname().split(".")[0]
    now = time.time()
    return (f"{who()} ({host}): {why}, since {iso(now)}, until {iso(now + max_s)}, "
            f"token {token}").replace("\n", " ")


class Hold:
    """One holder on the board, kept while this object's process lives."""

    def __init__(self, why: str, wait_s: float, max_s: float):
        self.token = secrets.token_hex(8)
        self.line = owner_line(why, max_s, self.token)
        lock, owner = paths()
        silence = float(os.environ.get("PICAR_BOARD_SILENCE_S", 90))
        self.proc = subprocess.Popen(remote(HOLDER, lock, owner, self.line, wait_s, max_s, silence),
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.state = None
        self.busy_line = ""
        self._beat = None

    def acquire(self) -> bool:
        """Blocks until HELD (True) or BUSY / a holder that died (False)."""
        for raw in self.proc.stdout:
            msg = raw.rstrip("\n")
            if msg == "HELD":
                self.state = "held"
                self._start_heartbeat()
                return True
            if msg.startswith("WAITING "):
                print(f"board busy, waiting: {msg[8:]}", file=sys.stderr, flush=True)
            elif msg.startswith("BUSY "):
                self.busy_line = msg[5:]
                break
        self.proc.wait()
        if not self.busy_line:
            self.busy_line = f"holder exited {self.proc.returncode} (ssh failed?)"
        return False

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

    def release(self):
        if self._beat:
            self._beat.set()
        try:
            self.proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()

    def alive(self) -> bool:
        return self.proc.poll() is None


def state_path() -> str:
    """Per worktree (its own git dir), so one session never releases another's hold."""
    if os.environ.get("PICAR_BOARD_STATE"):
        return os.environ["PICAR_BOARD_STATE"]
    try:
        gd = subprocess.run(["git", "rev-parse", "--absolute-git-dir"], capture_output=True,
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


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def cmd_status(_):
    lock, owner = paths()
    r = subprocess.run(remote(STATUS, lock, owner), capture_output=True, text=True, timeout=30)
    out = r.stdout.strip() or f"status failed: {r.stderr.strip()}"
    print(out)
    return 0 if out == "FREE" else BUSY if out.startswith("HELD") else 1


def cmd_run(a):
    if not a.cmd:
        print("run: give a command after --", file=sys.stderr)
        return 2
    h = Hold(a.why, a.wait, a.max)
    if not h.acquire():
        print(f"board busy: {h.busy_line}", file=sys.stderr)
        return BUSY
    print(f"board lock held: {h.line}", file=sys.stderr, flush=True)
    try:
        return subprocess.call(a.cmd, env={**os.environ, "PICAR_BOARD_TOKEN": h.token})
    finally:
        h.release()
        print("board lock released", file=sys.stderr)


def cmd_keep(a):
    """The detached child behind `hold`: holds until SIGTERM or the holder ends."""
    h = Hold(a.why, a.wait, a.max)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    path = a.state
    if not h.acquire():
        with open(path + ".tmp", "w") as f:
            json.dump({"busy": h.busy_line}, f)
        os.replace(path + ".tmp", path)
        return BUSY
    with open(path + ".tmp", "w") as f:
        json.dump({"pid": os.getpid(), "token": h.token, "line": h.line}, f)
    os.replace(path + ".tmp", path)
    while not stop.wait(1.0) and h.alive():
        pass
    h.release()
    try:
        cur = read_state()
        if cur and cur.get("pid") == os.getpid():
            os.remove(path)
    except OSError:
        pass
    return 0


def cmd_hold(a):
    path = state_path()
    cur = read_state()
    if cur and cur.get("pid") and pid_alive(cur["pid"]):
        print(f"this worktree already holds the board: {cur['line']}", file=sys.stderr)
        return 0
    if os.path.exists(path):
        os.remove(path)
    args = [sys.executable, os.path.abspath(__file__), "_keep", "--why", a.why,
            "--wait", str(a.wait), "--max", str(a.max), "--state", path]
    log = open(path + ".log", "w")
    subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    end = time.monotonic() + a.wait + 60
    while time.monotonic() < end:
        cur = read_state()
        if cur:
            if "busy" in cur:
                os.remove(path)
                print(f"board busy: {cur['busy']}", file=sys.stderr)
                return BUSY
            print(f"board lock held: {cur['line']}", file=sys.stderr)
            return 0
        time.sleep(0.2)
    print(f"hold: no answer from the board (see {path}.log)", file=sys.stderr)
    return 1


def cmd_release(_):
    cur = read_state()
    if not cur or not cur.get("pid"):
        print("this worktree holds no board lock", file=sys.stderr)
        return 0
    if pid_alive(cur["pid"]):
        os.kill(cur["pid"], signal.SIGTERM)
        end = time.monotonic() + 20
        while pid_alive(cur["pid"]) and time.monotonic() < end:
            time.sleep(0.2)
    if os.path.exists(state_path()):
        os.remove(state_path())
    print("board lock released", file=sys.stderr)
    return 0


def cmd_env(_):
    cur = read_state()
    if not cur or not cur.get("token") or not pid_alive(cur.get("pid", -1)):
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
