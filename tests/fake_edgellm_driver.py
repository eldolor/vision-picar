"""A stand-in for tools/jetson/edgellm_identity.cpp, speaking its protocol,
for tests/test_local_vlm.py. The target in the request picks what it does:

    yes / no   a merged `[{` or `[]` first, then filler until cancelled
    slow       filler every 0.2 s until cancelled
    prefill    1 s before the first token (a cancel is not seen until then)
    noparse    three filler tokens, then the end id
    error      the request is refused (`E 0 0 0`)
    die        exits as soon as it reads the request
    wedge      one filler token, then ignores everything

Env: FAKE_NEVER_READY=1 never prints READY; FAKE_ANSWER=yes|no answers so
for every target (a live mission's real target names none of the modes).
"""

import json
import math
import os
import select
import sys
import time

FILLER = 9
_buf = b""


def readline(timeout=None):
    """One stdin line, or None if none arrives in `timeout`; "" at EOF."""
    global _buf
    end = None if timeout is None else time.monotonic() + timeout
    while b"\n" not in _buf:
        wait = None if end is None else max(0.0, end - time.monotonic())
        r, _, _ = select.select([0], [], [], wait)
        if not r:
            return None
        chunk = os.read(0, 4096)
        if not chunk:
            return ""
        _buf += chunk
    line, _buf = _buf.split(b"\n", 1)
    return line.decode()


def out(s):
    sys.stdout.write(s + "\n")
    sys.stdout.flush()


def lp(p):
    return f"{math.log(p):.6f}"


def stream(first, gap):
    """Emit `first` (a token line) then filler until a C or 64 tokens."""
    if first:
        out(first)
    for _ in range(64):
        line = readline(gap)
        if line == "C":
            return 3
        out(f"T {FILLER} 1 {FILLER}:-0.01")
    return 2


def main():
    if os.environ.get("FAKE_NEVER_READY"):
        while readline() not in ("", None):
            pass
        return
    out("READY")
    while True:
        path = readline()
        if path == "":
            return
        if path in (None, "C") or not path:
            continue
        doc = json.loads(open(path).read())
        content = doc["requests"][0]["messages"][0]["content"]
        assert os.path.exists(content[0]["image"]), "image file missing"
        text = content[1]["text"]
        if os.environ.get("FAKE_ANSWER"):
            text = os.environ["FAKE_ANSWER"]
        t0 = time.monotonic()
        if "die" in text:
            sys.exit(1)
        if "wedge" in text:
            out(f"T {FILLER} 1 {FILLER}:-0.01")
            while True:
                time.sleep(1)
        if "error" in text:
            out("E 0 0 0")
            continue
        if "noparse" in text:
            for _ in range(3):
                out(f"T {FILLER} 1 {FILLER}:-0.01")
            out(f"E 1 {(time.monotonic() - t0) * 1000:.3f} 1")
            continue
        if "prefill" in text:
            time.sleep(1.0)
            reason = stream(f"T {FILLER} 1 {FILLER}:-0.01", 0.02)
        elif "slow" in text:
            reason = stream(None, 0.2)
        elif "yes" in text:
            reason = stream(f"T 6 2 6:{lp(0.9)} 7:{lp(0.1)}", 0.02)
        else:
            reason = stream(f"T 7 2 7:{lp(0.95)} 6:{lp(0.05)}", 0.02)
        out(f"E {reason} {(time.monotonic() - t0) * 1000:.3f} 1")


if __name__ == "__main__":
    main()
