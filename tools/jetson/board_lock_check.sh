# 3.93: sourced by board-side benches -- refuse to run under another
# session's hold on the board (tools/jetson/board_lock.py). Goes on when the
# lock is free (with a note to take it) or when PICAR_BOARD_TOKEN is the
# holder's; otherwise exits 75. A hold taken by hand (`flock` on
# ~/.picar-board.lock with a line in ~/.picar-board.owner) has no token, so
# it always refuses.
#
#   source "$(dirname "${BASH_SOURCE[0]}")/board_lock_check.sh"

python3 - "${PICAR_BOARD_LOCK_DIR:-$HOME}" "${PICAR_BOARD_TOKEN:-}" <<'PY' || exit 75
import fcntl, os, re, sys
d, token = sys.argv[1], sys.argv[2]
f = open(os.path.join(d, ".picar-board.lock"), "a")
try:
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    try:
        with open(os.path.join(d, ".picar-board.owner")) as o:
            line = o.read().strip()
    except OSError:
        line = ""
    m = re.search(r"token (\S+)$", line)
    if token and m and m.group(1) == token:
        sys.exit(0)
    print(f"the board is held by another session: {line or 'no owner line'}", file=sys.stderr)
    print("wait for it (tools/jetson/board_lock.py status), then take the lock", file=sys.stderr)
    sys.exit(75)
print("note: the board lock is free; take it first (tools/jetson/board_lock.py run / hold)",
      file=sys.stderr)
PY
