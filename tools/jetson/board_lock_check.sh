# 3.93: sourced by board-side benches. The bench holds the board's lock
# (tools/jetson/board_lock.py) for as long as it runs:
#   * the lock free: the bench takes it on fd 9 and keeps it until it exits
#     (and so do any processes it leaves running, which inherit fd 9);
#   * held under PICAR_BOARD_TOKEN (the caller's own `run` / `hold`): the
#     bench goes on, and a watcher takes the lock over the moment that hold
#     ends (the session died, the network went silent, --max), so a bench
#     that outlives its session still keeps the board;
#   * held by anyone else, or under a token whose `until` has passed: exit 75.
# A failure of the check itself exits 1, never 75 ("busy").
#
#   source "$(dirname "${BASH_SOURCE[0]}")/board_lock_check.sh"

_pbl_dir="${PICAR_BOARD_LOCK_DIR:-$HOME}"
if ! exec 9>>"$_pbl_dir/.picar-board.lock"; then
    echo "board lock check failed: cannot open $_pbl_dir/.picar-board.lock" >&2
    exit 1
fi
python3 - "$_pbl_dir" "${PICAR_BOARD_TOKEN:-}" "$$" "$0" <<'PY'
import datetime, fcntl, os, re, sys, time
d, token, pid, script = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
owner = os.path.join(d, ".picar-board.owner")


def take():
    try:
        fcntl.flock(9, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except BlockingIOError:
        return False


def mine_line():
    host = os.uname().nodename.split(".")[0]
    now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    return f"{os.path.basename(script)} pid {pid} ({host}): bench on the board, since {now}"


def write_owner(line):
    with open(owner + ".tmp", "w") as o:
        o.write(line + "\n")
    os.replace(owner + ".tmp", owner)


end = time.monotonic() + 1.5          # a status probe holds it for microseconds
while time.monotonic() < end:
    if take():
        write_owner(mine_line())
        print("board lock taken for this bench (it was free)", file=sys.stderr)
        sys.exit(0)
    time.sleep(0.1)
try:
    with open(owner) as o:
        line = o.read().strip()
except OSError:
    line = ""
m = re.search(r"until (\S+), token (\S+)$", line)
if token and m and m.group(2) == token:
    try:
        expired = datetime.datetime.fromisoformat(m.group(1)).timestamp() < time.time()
    except ValueError:
        expired = True
    if not expired:
        sys.exit(10)                  # ours: the shell starts the watcher
print(f"the board is held by another session: {line or 'no owner line'}", file=sys.stderr)
print("wait for it (tools/jetson/board_lock.py status), then take the lock", file=sys.stderr)
sys.exit(75)
PY
_pbl_rc=$?
if [ "$_pbl_rc" -eq 10 ]; then
    # The caller's hold: take the lock over on fd 9 when it ends, for as
    # long as this bench (the watcher's parent) lives.
    python3 - "$_pbl_dir" "$$" "$0" <<'PY' &
import datetime, fcntl, os, sys, time
d, parent, script = sys.argv[1], int(sys.argv[2]), sys.argv[3]
while os.getppid() == parent:
    try:
        fcntl.flock(9, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        time.sleep(1.0)
        continue
    now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    owner = os.path.join(d, ".picar-board.owner")
    with open(owner + ".tmp", "w") as o:
        o.write(f"{os.path.basename(script)} pid {parent}: bench on the board, kept after "
                f"its session's hold ended, since {now}\n")
    os.replace(owner + ".tmp", owner)
    break
PY
    _pbl_rc=0
fi
[ "$_pbl_rc" -eq 0 ] || exit "$_pbl_rc"
unset _pbl_dir _pbl_rc
