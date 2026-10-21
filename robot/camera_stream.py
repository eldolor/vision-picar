"""
robot/camera_stream.py

3.90 A: **the car's camera as a stream.** A reader thread keeps only the
newest frame; `get_camera_frame()` returns it at once. Opening the device for
each photo would add its start-up time (~100 ms on a USB camera) to every
read, and the pictures would be whatever the auto-exposure had settled on
after a cold open.

Three rules, each a criterion in `docs/plans/ros-alignment/3.90-camera-stream.md`:

* **Newest wins.** Nothing queues: a reader that falls behind skips frames,
  so a read is never older than the source's own latency.
* **No stale picture.** A source that stops delivering makes
  `get_camera_frame()` raise `CameraStale` once the newest frame is older
  than `stale_s` -- `robot/server.py`'s `/frame` turns that into a 503,
  exactly as it does for `sim/teleop_robot.py`'s `TeleopStall`. An old
  frame served as current is the one wrong answer.
* **It reopens by itself.** A read error or an unplugged device closes the
  source; the reader opens it again every `reopen_s` until it answers, with
  no restart of the robot server.

The reader runs on its own thread, never on the robot server's event loop,
whose wheel loop and watchdog must not wait on a camera.

Every frame carries `captured_at` (wall-clock seconds on this machine, when
the reader got it). The brain times its counted rules by it (criterion 4):
only differences between two of this camera's stamps are ever taken.

**Encoding is lazy.** A source yields either JPEG bytes (a UVC camera's own
MJPEG, passed through untouched) or a picture that still needs encoding; the
latter is encoded only when read, once per frame, so a 30 fps camera read at
5 fps pays for 5 encodes, not 30.
"""

import base64
import logging
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# A frame older than this is not served (criterion 2: 503 within 1.0 s).
DEFAULT_STALE_S = 0.5
# Between attempts to open a source that is not answering (criterion 2:
# delivering again within 3 s of a replug).
DEFAULT_REOPEN_S = 0.5


class CameraStale(RuntimeError):
    """No frame newer than `stale_s`. Raised, never answered with an old one."""


class CameraStream:
    """The newest frame of a source, read on a thread of its own.

    `open_source()` returns an object with `read()` -> one frame (bytes for
    a JPEG, anything else for a picture `encode()` turns into JPEG bytes) or
    None for "nothing this time", and `close()`. It may raise; so may
    `read()`. Either closes the source and the reader opens it again."""

    def __init__(self, open_source: Callable[[], object],
                 encode: Optional[Callable[[object], bytes]] = None,
                 stale_s: float = DEFAULT_STALE_S, reopen_s: float = DEFAULT_REOPEN_S,
                 clock: Callable[[], float] = time.time, name: str = "camera"):
        self._open_source = open_source
        self._encode = encode
        self.stale_s = stale_s
        self.reopen_s = reopen_s
        self._clock = clock
        self.name = name
        self._lock = threading.Lock()
        self._raw = None              # the newest frame as the source gave it
        self._captured_at: Optional[float] = None
        self._seq = 0
        self._jpeg_seq = 0            # which seq `_jpeg` encodes
        self._jpeg: Optional[bytes] = None
        self._opens = 0
        self._failures = 0
        self._last_error: Optional[str] = None
        self._running = True
        self._source = None
        self._thread = threading.Thread(target=self._run, name=f"{name}-reader", daemon=True)
        self._thread.start()

    # ---------- the reader ----------

    def _run(self) -> None:
        while self._running:
            if self._source is None:
                try:
                    self._source = self._open_source()
                    self._opens += 1
                except Exception as e:  # noqa: BLE001 -- a camera may fail any way
                    self._note_failure(f"open: {e}")
                    self._sleep(self.reopen_s)
                    continue
            try:
                frame = self._source.read()
            except Exception as e:  # noqa: BLE001
                self._note_failure(f"read: {e}")
                self._close_source()
                self._sleep(self.reopen_s)
                continue
            if frame is None:
                # Nothing this time. A source that says so for good is caught
                # by the stale check on the read side, not here.
                self._sleep(0.005)
                continue
            now = self._clock()
            with self._lock:
                self._raw = frame
                self._captured_at = now
                self._seq += 1
        self._close_source()

    def _note_failure(self, why: str) -> None:
        with self._lock:
            self._failures += 1
            self._last_error = why
        logger.warning("%s: %s", self.name, why)

    def _close_source(self) -> None:
        source, self._source = self._source, None
        if source is not None:
            try:
                source.close()
            except Exception:  # noqa: BLE001 -- already failing; closing is best-effort
                pass

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while self._running and time.monotonic() < end:
            time.sleep(min(0.05, seconds))

    # ---------- the read side ----------

    def get_camera_frame(self) -> dict:
        with self._lock:
            raw, captured_at, seq = self._raw, self._captured_at, self._seq
            if captured_at is None:
                raise CameraStale(f"{self.name}: no frame yet ({self._last_error or 'opening'})")
            age = self._clock() - captured_at
            if age > self.stale_s:
                raise CameraStale(f"{self.name}: no new frame in {age:.1f}s "
                                  f"(limit {self.stale_s}s): {self._last_error or 'no error'}")
            jpeg = self._jpeg if self._jpeg_seq == seq else None
        if jpeg is None:
            # Outside the lock: an encode must not hold up the reader.
            jpeg = raw if isinstance(raw, (bytes, bytearray)) else self._encode(raw)
            with self._lock:
                if self._seq == seq:
                    self._jpeg, self._jpeg_seq = bytes(jpeg), seq
        return {
            "image_base64": base64.b64encode(jpeg).decode("ascii"),
            "media_type": "image/jpeg",
            "room": "unknown",
            "captured_at": captured_at,
            "metadata": {"source": self.name, "seq": seq},
        }

    def status(self) -> dict:
        """For `/health` -- description, never a verdict (control/health.py)."""
        with self._lock:
            age = None if self._captured_at is None else round(self._clock() - self._captured_at, 3)
            return {"frames": self._seq, "age_s": age, "opens": self._opens,
                    "failures": self._failures, "last_error": self._last_error,
                    # The reader's OS thread id, so a bench can read its own
                    # CPU time from /proc (3.90 criterion 3).
                    "reader_tid": self._thread.native_id}

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)


class OpenCvSource:
    """A V4L2 camera through OpenCV. Asks the camera for MJPEG and, where the
    backend allows it, takes the camera's own JPEG bytes without decoding
    them (`CAP_PROP_CONVERT_RGB` 0); otherwise the decoded picture is
    encoded lazily by `encode_jpeg`. cv2 is imported here only: the suite
    and the simulator never need it."""

    def __init__(self, device, width: int = 1280, height: int = 720, fps: int = 30):
        import cv2

        self._cv2 = cv2
        dev = int(device) if str(device).isdigit() else device
        self._cap = cv2.VideoCapture(dev, cv2.CAP_V4L2) if hasattr(cv2, "CAP_V4L2") \
            else cv2.VideoCapture(dev)
        if not self._cap.isOpened():
            raise OSError(f"cannot open camera {device!r}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._passthrough = bool(self._cap.set(cv2.CAP_PROP_CONVERT_RGB, 0))

    def read(self):
        ok, frame = self._cap.read()
        if not ok:
            raise OSError("camera read failed")
        if self._passthrough and frame.ndim == 2 and frame.shape[0] == 1:
            return frame.tobytes()      # the camera's own JPEG
        if self._passthrough and frame.ndim == 1:
            return frame.tobytes()
        return frame

    def close(self) -> None:
        self._cap.release()


def encode_jpeg(picture, quality: int = 85) -> bytes:
    import cv2

    ok, buf = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("JPEG encode failed")
    return buf.tobytes()
