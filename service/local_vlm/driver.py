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
import os
import queue
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

    def ask(self, jpeg: bytes, target: str, deadline_s: float) -> dict:
        """One arrival question. Returns `{"status", "score", "parsed",
        "ms", "driver_ms"}`; status is `answered`, `timeout`, `unparsed`,
        `busy` or `error`. Raises DriverGone when the driver is not there.

        It returns at the deciding token or at the deadline, whichever comes
        first. The driver ends the request on its own after the cancel, and
        that tail is drained in the background with the runtime still held,
        so the next request waits for it (and is `busy` if it cannot).
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
            req.write_text(json.dumps(request_doc(img, target)))
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
            result, ended = self._stream(t0, end)
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

    def _stream(self, t0: float, end: float) -> tuple:
        """`(result, ended)`: `ended` False when a cancel was sent and the
        request's end line is still to come (`_drain` reads it)."""
        content = []
        while True:
            line = self._next(end - time.monotonic())
            if line is None:
                self._send("C")
                return ({"status": "timeout", "score": None, "parsed": False,
                         "ms": (time.monotonic() - t0) * 1000, "driver_ms": None}, False)
            if line.startswith("T "):
                chosen, alts = parse_token_line(line)
                content.append(content_entry(self.decode, chosen, alts))
                if decision_reached(content):
                    ms = (time.monotonic() - t0) * 1000
                    self._send("C")
                    score, parsed, _ = p_localised(content)
                    return ({"status": "answered" if parsed else "unparsed",
                             "score": score if parsed else None, "parsed": parsed,
                             "ms": ms, "driver_ms": None}, False)
            elif line.startswith("E "):
                reason, driver_ms, ok = parse_end_line(line)
                self.served += 1
                ms = (time.monotonic() - t0) * 1000
                if not ok and not content:
                    return ({"status": "error", "score": None, "parsed": False,
                             "ms": ms, "driver_ms": driver_ms}, True)
                score, parsed, _ = p_localised(content)
                return ({"status": "answered" if parsed else "unparsed",
                         "score": score if parsed else None, "parsed": parsed,
                         "ms": ms, "driver_ms": driver_ms}, True)
            else:
                logger.warning("driver line not understood: %r", line[:80])

    def close(self) -> None:
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)  # inside the unit's TimeoutStopSec (20 s)
        except Exception:  # noqa: BLE001
            self.proc.kill()
