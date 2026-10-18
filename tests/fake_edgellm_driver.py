"""A stand-in for tools/jetson/edgellm_identity.cpp, speaking its protocol,
for tests/test_local_vlm.py. The target in the request picks what it does:

    yes / no   a merged `[{` or `[]` first; a yes then its box (3.80.D),
               `[{"bbox_2d": [282, 40, 362, 160]}]`, and the end; a no
               filler until cancelled
    boxslow    a yes whose box comes 0.2 s a token (a deadline after the
               decision)
    slow       filler every 0.2 s until cancelled
    prefill    1 s before the first token (a cancel is not seen until then)
    noparse    three filler tokens, then the end id
    error      the request is refused (`E 0 0 0`)
    die        exits as soon as it reads the request
    wedge      one filler token, then ignores everything

3.80.F, the direction prompt (it says "left, right or forward"): a target
naming `goleft`, `goright` or `gofwd` answers so (ids 20-24); `gounsure`
puts most of the first token elsewhere; `gospace` sends a whitespace token
first; `noparse` ends after three filler tokens; `goslow` sends filler every 0.2 s until cancelled; anything else
answers `forward`. FAKE_DIRECTION=left|right|forward
answers so for every target.

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


# 3.80.D: the box after a merged `[{` (ids 10-16 in the tests' vocabulary).
BOX_TAIL = (10, 11, 12, 13, 12, 14, 12, 15, 16)


def box(gap):
    """A yes, then its box a token every `gap` s, then the end (1, as EOS).
    3 if cancelled on the way."""
    out(f"T 6 2 6:{lp(0.9)} 7:{lp(0.1)}")
    for tid in BOX_TAIL:
        if readline(gap) == "C":
            return 3
        out(f"T {tid} 1 {tid}:-0.01")
    return 1


# 3.80.F: the direction words, in the tests' vocabulary.
LEFT, RIGHT, FORWARD, FOR, CAP_LEFT, SPACE, MOVE, COLON = 20, 22, 23, 24, 21, 25, 26, 27


def direction(text):
    """The first word of a direction reply, then filler until cancelled."""
    forced = os.environ.get("FAKE_DIRECTION")
    pick = ({"left": LEFT, "right": RIGHT, "forward": FORWARD}[forced] if forced
            else LEFT if "goleft" in text else RIGHT if "goright" in text else FORWARD)
    if "goslow" in text:
        stream(None, 0.2)     # whitespace-free filler, never a word, until cancelled
        return
    if "noparse" in text:
        for _ in range(3):
            out(f"T {FILLER} 1 {FILLER}:-0.01")
        return
    # The asked-for lead-in, "Move:" (and a space, for `gospace`).
    out(f"T {MOVE} 1 {MOVE}:-0.01")
    out(f"T {COLON} 1 {COLON}:-0.01")
    if "gospace" in text:
        out(f"T {SPACE} 1 {SPACE}:-0.01")
    if "gounsure" in text:
        first = f"T {LEFT} 3 {FILLER}:{lp(0.7)} {LEFT}:{lp(0.2)} {RIGHT}:{lp(0.1)}"
    elif pick == LEFT:
        # `left` and ` Left` both count for left; `for` for forward.
        first = f"T {LEFT} 4 {LEFT}:{lp(0.6)} {CAP_LEFT}:{lp(0.2)} {FOR}:{lp(0.15)} {RIGHT}:{lp(0.05)}"
    else:
        first = f"T {pick} 2 {pick}:{lp(0.8)} {LEFT}:{lp(0.2)}"
    stream(first, 0.02)


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
        t0 = time.monotonic()
        if "left, right or forward" in text:
            direction(text)
            out(f"E 3 {(time.monotonic() - t0) * 1000:.3f} 1")
            continue
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
        elif "boxslow" in text:
            reason = box(0.2)
        elif "slow" in text:
            reason = stream(None, 0.2)
        elif "yes" in text:
            reason = box(0.02)
        else:
            reason = stream(f"T 7 2 7:{lp(0.95)} 6:{lp(0.05)}", 0.02)
        out(f"E {reason} {(time.monotonic() - t0) * 1000:.3f} 1")


if __name__ == "__main__":
    main()
