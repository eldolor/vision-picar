"""
service/local_vlm/driver.py

3.80.B (`docs/plans/ros-alignment/3.80-tiered-identity.md`): one resident
TensorRT Edge-LLM runtime -- 3.78's `edgellm_identity` -- asked the arrival
question exactly as 3.78 measured it.

The driver's protocol (`tools/jetson/edgellm_identity.cpp`) has one request
in flight at a time, and its cancel check reads stdin, so nothing but `C`
may be written while a request runs. This class keeps that rule: a request
file is written only after the previous request's end line, and the whole
exchange is under one lock.
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from tools.jetson.bench_identity_llama import decision_reached, p_localised
from tools.jetson.bench_identity_trt import (content_entry, parse_end_line,
                                             parse_token_line, request_doc)

logger = logging.getLogger("local_vlm")

# After a cancel, how long the driver may take to end the request before it
# is taken for wedged. A cancel lands at the next streamed token (~25 ms on
# the board); a whole 64-token reply is ~1.5 s.
CANCEL_GRACE_S = 3.0
# Less time than this left once the runtime is free: answer `timeout`
# without sending. The driver's cancel check polls the file descriptor, so a
# `C` written right behind the request path can land in its stdin buffer
# unseen, and the request would then run to its end.
MIN_SEND_S = 0.1
_EOF = object()
# 3.80.D: the first JSON object in a reply, for its `bbox_2d`.
_OBJECT = re.compile(r"\{[^{}]*\}")

# 3.80.F: the search question -- the cloud's "single best next action to get
# closer to the target" (vision_core's NAVIGATE_PROMPT_TEMPLATE, question 5),
# asked for one word. The word comes after "Move:" because VE's reduced
# vocabulary (3.78) keeps only the space-led forms of two of the three words
# (" left", " forward"; bare "left", "Left", "forward" were cut, bare "right"
# was not): a reply that OPENED with the word could only say right (F4,
# 2026-10-16).
DIRECTION_PROMPT = ("You are the camera of a small indoor robot searching a house for "
                    "a {target}. Which single move brings the robot closer to finding "
                    "the {target}: turn left, turn right, or go forward? Go forward only "
                    "if the way ahead is open. Reply with one line in the form "
                    "\"Move: <word>\", where <word> is one of left, right or forward.")
DIRECTIONS = ("left", "right", "forward")
DIRECTION_MAX_TOKENS = 8
# Below this much of the first token's probability on the three words, the
# reply did not answer the question (`unparsed`, unable).
DIRECTION_MIN_MASS = 0.5
_PUNCT = " \t\n.,:;!\"'*`"


def direction_doc(image: Path, target: str) -> dict:
    """The search question as an Edge-LLM request file: the identity
    request's sampling (greedy, top-20 logprobs), its own prompt, and a few
    tokens at most -- the answer is the first."""
    doc = request_doc(image, target)
    doc["max_generate_length"] = DIRECTION_MAX_TOKENS
    doc["requests"][0]["messages"][0]["content"][1]["text"] = DIRECTION_PROMPT.format(
        target=target)
    return doc


def _option(token: str) -> Optional[str]:
    """The direction a token starts, or None: any non-empty prefix of one
    of the three words, any case (`L`, `for`, ` Forward`)."""
    t = token.strip(_PUNCT).lower()
    if not t:
        return None
    hits = [d for d in DIRECTIONS if d.startswith(t)]
    return hits[0] if len(hits) == 1 else None


def direction_of(step: dict) -> Optional[dict]:
    """The answer read off the first generated token that is one of the
    words (or a prefix of one): `{"direction", "p", "mass", "probs"}`.
    `probs` is each word's probability among that step's top alternatives,
    `mass` their sum, `p` the winner's share of it. None for any other
    token (`Move`, `:`, whitespace): read the next one."""
    if _option(step["token"]) is None:
        return None
    probs = dict.fromkeys(DIRECTIONS, 0.0)
    alts = step.get("top_logprobs") or [{"token": step["token"], "logprob": step["logprob"]}]
    for a in alts:
        d = _option(a["token"])
        if d is not None:
            probs[d] += math.exp(a["logprob"])
    mass = sum(probs.values())
    best = max(DIRECTIONS, key=lambda d: probs[d])
    return {"direction": best if mass > 0 else None,
            "p": round(probs[best] / mass, 4) if mass > 0 else None,
            "mass": round(mass, 4), "probs": {d: round(v, 4) for d, v in probs.items()}}


def parse_box(text: str) -> Optional[list]:
    """`[x1, y1, x2, y2]` from the first object in `text` that carries a
    well-formed `bbox_2d`, or None. Only the generated text is read: the
    prompt's own example box is never in it (perceive_lab's lesson)."""
    for m in _OBJECT.finditer(text):
        try:
            box = [float(v) for v in json.loads(m.group(0))["bbox_2d"]]
        except (ValueError, KeyError, TypeError):
            continue
        if len(box) == 4 and box[2] > box[0] and box[3] > box[1]:
            return box
    return None


class DriverGone(RuntimeError):
    """The driver process is not running (killed, crashed, or wedged)."""


class EdgeDriver:
    def __init__(self, cmd: list, env: dict, decode: Callable[[int], str],
                 workdir: Optional[str] = None, stderr=None):
        self.decode = decode
        self.workdir = Path(workdir or ("/dev/shm" if os.path.isdir("/dev/shm")
                                        else tempfile.gettempdir()))
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=stderr, text=True, bufsize=1, env=env)
        self.lines: "queue.Queue" = queue.Queue()
        self.lock = threading.Lock()
        self.ready = False
        self.served = 0
        self._n = 0
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.put(line.strip())
        self.lines.put(_EOF)

    @property
    def alive(self) -> bool:
        return self.proc.poll() is None

    def _next(self, timeout: float):
        """The driver's next stdout line, None on timeout; DriverGone at EOF."""
        try:
            line = self.lines.get(timeout=max(0.0, timeout))
        except queue.Empty:
            return None
        if line is _EOF:
            self.lines.put(_EOF)  # sticky: every later reader sees the death too
            raise DriverGone(f"driver exited ({self.proc.poll()})")
        return line

    def wait_ready(self, timeout: float) -> None:
        end = time.monotonic() + timeout
        while True:
            line = self._next(end - time.monotonic())
            if line is None:
                self.proc.kill()  # a half-loaded runtime must not linger
                raise DriverGone(f"driver not READY in {timeout:.0f} s")
            if line == "READY":
                self.ready = True
                return

    def _send(self, text: str) -> None:
        try:
            self.proc.stdin.write(text + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as e:
            raise DriverGone(f"driver stdin closed: {e}") from e

    def ask(self, jpeg: bytes, target: str, deadline_s: float, box: bool = False,
            question: str = "identity") -> dict:
        """One arrival question. Returns `{"status", "score", "parsed",
        "ms", "driver_ms", "box"}`; status is `answered`, `timeout`,
        `unparsed`, `busy` or `error`. Raises DriverGone when the driver is
        not there.

        3.80.D: with `box`, a reply that decides yes is read on until its box
        closes (or the deadline: the score, with `box` None), so a sighting
        says where; a no still ends at the deciding token. The score is the
        one read at the decision either way.

        It returns at the deciding token or at the deadline, whichever comes
        first. The driver ends the request on its own after the cancel, and
        that tail is drained in the background with the runtime still held,
        so the next request waits for it (and is `busy` if it cannot).

        3.80.F: `question="direction"` asks the search question instead
        (`DIRECTION_PROMPT`) and returns at the first token that is one of
        the words: status `answered` with `direction`, `p`, `mass` and
        `probs`, or `unparsed` when under DIRECTION_MIN_MASS of that step's
        probability is on the three words, or when the reply ends without
        one. `score` is None for it.

        `driver_ms` is the driver's own time, known only when the request
        ended before the reply returned (`error`, or an end id before any
        decision); otherwise None."""
        t0 = time.monotonic()
        end = t0 + deadline_s
        if not self.alive:
            raise DriverGone(f"driver exited ({self.proc.poll()})")
        if not self.lock.acquire(timeout=max(0.0, deadline_s)):
            return {"status": "busy", "score": None, "parsed": False,
                    "ms": (time.monotonic() - t0) * 1000, "driver_ms": None}
        files: list = []
        handed_off = False
        try:
            if end - time.monotonic() < MIN_SEND_S:
                return {"status": "timeout", "score": None, "parsed": False,
                        "ms": (time.monotonic() - t0) * 1000, "driver_ms": None}
            self._n += 1
            img = self.workdir / f"local-vlm-{os.getpid()}-{self._n}.jpg"
            req = self.workdir / f"local-vlm-{os.getpid()}-{self._n}.json"
            files = [img, req]
            img.write_bytes(jpeg)
            req.write_text(json.dumps(direction_doc(img, target) if question == "direction"
                                      else request_doc(img, target)))
            # A line left by an earlier exchange is not this request's: the
            # lock means none can be in flight now.
            while True:
                try:
                    stale = self.lines.get_nowait()
                except queue.Empty:
                    break
                if stale is _EOF:
                    self.lines.put(_EOF)
                    raise DriverGone(f"driver exited ({self.proc.poll()})")
            self._send(str(req))
            result, ended = (self._stream_direction(t0, end) if question == "direction"
                             else self._stream(t0, end, box))
            if not ended:
                handed_off = True
                threading.Thread(target=self._drain, args=(files,), daemon=True).start()
            return result
        finally:
            if not handed_off:
                self._release(files)

    def _release(self, files) -> None:
        for f in files:
            f.unlink(missing_ok=True)
        self.lock.release()

    def _drain(self, files) -> None:
        """After a cancel: wait for the request's end line, then free the
        runtime. A driver that does not end it in CANCEL_GRACE_S is wedged
        and is killed; the service's watcher then reports it gone."""
        try:
            until = time.monotonic() + CANCEL_GRACE_S
            while True:
                line = self._next(until - time.monotonic())
                if line is None:
                    logger.error("driver did not end a cancelled request; killing it")
                    self.proc.kill()
                    return
                if line.startswith("E "):
                    self.served += 1
                    return
        except DriverGone:
            return
        finally:
            self._release(files)

    def _stream(self, t0: float, end: float, want_box: bool = False) -> tuple:
        """`(result, ended)`: `ended` False when a cancel was sent and the
        request's end line is still to come (`_drain` reads it)."""
        content = []
        decided = None   # the reply at the deciding token, once reached (box mode)

        def reply(ms, driver_ms=None, found=None):
            score, parsed, _ = decided or p_localised(content)
            return {"status": "answered" if parsed else "unparsed",
                    "score": score if parsed else None, "parsed": parsed,
                    "ms": ms, "driver_ms": driver_ms, "box": found,
                    "decision_ms": decision_ms}

        decision_ms = None
        while True:
            line = self._next(end - time.monotonic())
            if line is None:
                self._send("C")
                ms = (time.monotonic() - t0) * 1000
                if decided is not None:
                    # Past the decision, short of the box: the score stands.
                    return reply(ms), False
                return ({"status": "timeout", "score": None, "parsed": False,
                         "ms": ms, "driver_ms": None, "box": None}, False)
            if line.startswith("T "):
                chosen, alts = parse_token_line(line)
                content.append(content_entry(self.decode, chosen, alts))
                if decided is None and decision_reached(content):
                    decision_ms = (time.monotonic() - t0) * 1000
                    decided = p_localised(content)
                    text = "".join(c["token"] for c in content)
                    if not (want_box and decided[1] and "{" in text):
                        self._send("C")
                        return reply(decision_ms), False
                if decided is not None:
                    found = parse_box("".join(c["token"] for c in content))
                    if found is not None:
                        # The driver ends the request at its end id anyway; a
                        # C that arrives after that is skipped by its loop.
                        self._send("C")
                        return reply((time.monotonic() - t0) * 1000, found=found), False
            elif line.startswith("E "):
                reason, driver_ms, ok = parse_end_line(line)
                self.served += 1
                ms = (time.monotonic() - t0) * 1000
                if not ok and not content:
                    return ({"status": "error", "score": None, "parsed": False,
                             "ms": ms, "driver_ms": driver_ms, "box": None}, True)
                return reply(ms, driver_ms), True
            else:
                logger.warning("driver line not understood: %r", line[:80])

    def _stream_direction(self, t0: float, end: float) -> tuple:
        """`(result, ended)` for the search question: the answer at the
        first word, the rest cancelled."""
        def unable(status, ms, driver_ms=None, read=None):
            return {"status": status, "score": None, "parsed": False, "direction": None,
                    "p": None, "mass": read["mass"] if read else None,
                    "probs": read["probs"] if read else None, "ms": ms,
                    "driver_ms": driver_ms, "box": None, "decision_ms": None}

        while True:
            line = self._next(end - time.monotonic())
            if line is None:
                self._send("C")
                return unable("timeout", (time.monotonic() - t0) * 1000), False
            if line.startswith("T "):
                chosen, alts = parse_token_line(line)
                read = direction_of(content_entry(self.decode, chosen, alts))
                if read is None:
                    continue    # the lead-in ("Move:"): the word is still to come
                self._send("C")
                ms = (time.monotonic() - t0) * 1000
                if read["mass"] < DIRECTION_MIN_MASS:
                    return unable("unparsed", ms, read=read), False
                return {"status": "answered", "score": None, "parsed": True, **read,
                        "ms": ms, "driver_ms": None, "box": None, "decision_ms": ms}, False
            elif line.startswith("E "):
                reason, driver_ms, ok = parse_end_line(line)
                self.served += 1
                ms = (time.monotonic() - t0) * 1000
                return unable("unparsed" if ok else "error", ms, driver_ms), True
            else:
                logger.warning("driver line not understood: %r", line[:80])

    def close(self) -> None:
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)  # inside the unit's TimeoutStopSec (20 s)
        except Exception:  # noqa: BLE001
            self.proc.kill()
