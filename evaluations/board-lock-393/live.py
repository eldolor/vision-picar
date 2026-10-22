"""
evaluations/board-lock-393/live.py

3.93 criterion 6, on the real board: `run` takes and releases the board's
lock; `status` from a second process names the holder; the board-side check
refuses a wrong token and passes the holder's; a SIGKILLed `run` frees the
board within 5 s. Changes nothing on the board but the lock and owner files.

    .venv/bin/python evaluations/board-lock-393/live.py
"""

import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOL = [sys.executable, str(ROOT / "tools" / "jetson" / "board_lock.py")]
CHECK = (ROOT / "tools" / "jetson" / "board_lock_check.sh").read_text()


def status():
    return subprocess.run(TOOL + ["status"], capture_output=True, text=True, timeout=60).stdout.strip()


def board_check(token):
    """The check as a bench would run it, on the board (sent inline: the
    board's checkout may predate it)."""
    script = f"PICAR_BOARD_TOKEN={shlex.quote(token)} bash -c {shlex.quote(CHECK + '\necho ran')}"
    r = subprocess.run(["ssh", "picar-jetson", script], capture_output=True, text=True, timeout=60)
    return r.returncode, r.stdout.strip(), r.stderr.strip()


def main():
    out = {}
    if os.environ.get("PICAR_BOARD_TOKEN") is None:
        assert status() == "FREE", "the board must be free to start"
        # Re-run self under the lock: the inner half sees the token.
        rc = subprocess.call(TOOL + ["run", "--why", "3.93 criterion 6", "--", sys.executable, __file__])
        print("after run:", status())
        out["released"] = status() == "FREE"
        t0 = time.monotonic()
        p = subprocess.Popen(TOOL + ["run", "--why", "3.93 kill test", "--", "sleep", "120"])
        end = time.monotonic() + 30
        while not status().startswith("HELD") and time.monotonic() < end:
            time.sleep(0.5)
        os.kill(p.pid, signal.SIGKILL)
        p.wait()
        t0 = time.monotonic()
        while status() != "FREE" and time.monotonic() - t0 < 30:
            time.sleep(0.25)
        freed = time.monotonic() - t0
        print(f"SIGKILL -> FREE in {freed:.1f} s (bar 5 s)")
        print("criterion 6:", "met" if rc == 0 and out["released"] and freed <= 5 else "FAILED")
        return
    s = status()
    print("status from a second process:", s)
    assert s.startswith("HELD ") and "3.93 criterion 6" in s
    rc, so, se = board_check("0000000000000000")
    print("wrong token:", rc, se.splitlines()[0] if se else "")
    assert rc == 75 and "ran" not in so
    rc, so, se = board_check(os.environ["PICAR_BOARD_TOKEN"])
    print("holder's token:", rc, so, se[-300:] if rc else "")
    assert rc == 0 and so == "ran"


if __name__ == "__main__":
    main()
