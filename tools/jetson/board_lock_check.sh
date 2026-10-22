# 3.93: sourced by board-side benches. The bench holds the board's lock
# (tools/jetson/board_lock.py) for as long as it runs, through a small
# detached keeper that watches the bench's pid ($$) -- not through an
# inherited fd, so a process the bench leaves behind never holds the board:
#   * the lock free: the keeper takes it now;
#   * held under PICAR_BOARD_TOKEN (the caller's own `run` / `hold`, its
#     `until` not passed on the board's clock): the bench goes on, and the
#     keeper blocks on the lock so the kernel hands it over the moment that
#     hold ends (the session died, the network went silent, --max) -- a
#     bench that outlives its session still keeps the board;
#   * held by anyone else: exit 75.
# A failure of the check itself exits 1 (or python's own code), never 75.
#
#   source "$(dirname "${BASH_SOURCE[0]}")/board_lock_check.sh"

python3 - "${PICAR_BOARD_LOCK_DIR:-$HOME}" "${PICAR_BOARD_TOKEN:-}" "$$" "$0" <<'PY'
import datetime, fcntl, os, re, sys, threading, time
d, token, bench, script = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
lock_path, owner = os.path.join(d, ".picar-board.lock"), os.path.join(d, ".picar-board.owner")
try:
    f = open(lock_path, "a")
except OSError as e:
    print(f"board lock check failed: {e}", file=sys.stderr)
    sys.exit(1)


def owner_line():
    try:
        with open(owner) as o:
            return o.read().strip()
    except OSError:
        return ""


def write_owner(line):
    with open(owner + ".tmp", "w") as o:
        o.write(line + "\n")
    os.replace(owner + ".tmp", owner)


def take_now():
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except BlockingIOError:
        return False


def alive(pid):
    """Running, not a zombie its parent has yet to reap."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        with open(f"/proc/{pid}/stat") as st:          # the board
            return st.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        pass
    import subprocess                                  # no /proc (the Mac's tests)
    r = subprocess.run(["ps", "-o", "stat=", "-p", str(pid)], capture_output=True, text=True)
    return r.returncode == 0 and not r.stdout.strip().startswith("Z")


def keep(takeover):
    """Fork a detached keeper that holds the lock while the bench lives."""
    if os.fork():
        return
    os.setsid()
    null = os.open(os.devnull, os.O_RDWR)
    for fd in (0, 1, 2):
        os.dup2(null, fd)
    got = threading.Event()
    if takeover:
        def wait():
            fcntl.flock(f, fcntl.LOCK_EX)     # blocking: handed over at once
            got.set()
        threading.Thread(target=wait, daemon=True).start()
    else:
        got.set()
    line = None
    while alive(bench):
        if got.wait(0.5) and line is None:
            now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
            host = os.uname().nodename.split(".")[0]
            line = (f"{os.path.basename(script)} pid {bench} ({host}): bench on the board"
                    + (", kept after its session's hold ended" if takeover else "")
                    + f", since {now}")
            write_owner(line)
    if line is not None and owner_line() == line:
        open(owner, "w").close()
    os._exit(0)


if take_now():
    keep(False)
    print("board lock taken for this bench (it was free)", file=sys.stderr)
    sys.exit(0)
m = re.search(r"until (\S+), token (\S+)$", owner_line())
if token and m and m.group(2) == token:
    try:   # `until` is written by the holder on the board's own clock
        live = datetime.datetime.fromisoformat(m.group(1)).timestamp() > time.time()
    except ValueError:
        live = False
    if live:
        keep(True)
        sys.exit(0)
end = time.monotonic() + 1.5          # a status probe holds it for microseconds
while time.monotonic() < end:
    if take_now():
        keep(False)
        print("board lock taken for this bench (it was free)", file=sys.stderr)
        sys.exit(0)
    time.sleep(0.1)
print(f"the board is held by another session: {owner_line() or 'no owner line'}", file=sys.stderr)
print("wait for it (tools/jetson/board_lock.py status), then take the lock", file=sys.stderr)
sys.exit(75)
PY
_pbl_rc=$?
[ "$_pbl_rc" -eq 0 ] || exit "$_pbl_rc"
unset _pbl_rc
