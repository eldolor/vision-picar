"""
control/stream_recorder.py

3.90 E: **a walk recorded by the brain from the phone's stream.** Under
"Drive via brain" with "Record this walk" on, the phone pushes each frame
once (`POST /teleop/frame`); this recorder drains every frame the robot
server kept (`GET /teleop/frames?after=<seq>`) and files it in the brain's
walk store. Before it, the phone uploaded every frame a second time to
`/recording/frame`, and only the frames that happened to be captured on a
tick were saved.

**Every frame received, not only those perception took.** Decisions take
the newest frame, so at 4 fps some frames are never perceived. Each row says
which: `perceived: true` carries the decision taken on that frame (joined by
the robot's own seq, `teleop_seq` / `last_frame_seq`, as
`control/perception_eval.py` already joins); `perceived: false` carries the
pixels alone. Offline replays score every frame; `control/walk_eval.py`
scores only the perceived ones.

**Off the decision path.** The recorder is a thread polling the robot every
`poll_s`; it never touches the mission's loop. A frame is filed once the
mission has decided past it, or after `settle_s`, so a slow tick does not
mark a frame unperceived that is about to be perceived.

**Nothing is lost in silence.** Frames that fell out of the robot's buffer
before the recorder read them, and frames past the walk's cap, are counted
in meta.json (`frames_lost`, `frames_over_cap`). A robot server that
restarted (its `boot_id` changed) starts the cursor over.
"""

import base64
import io
import logging
import threading
import time
from typing import Callable, Optional

from control.recording_routes import (MAX_STREAM_FRAMES, WalkFull, finish_walk_meta,
                                      store_frame)

logger = logging.getLogger(__name__)

POLL_S = 0.25
# How long a fetched frame may wait for the mission to decide past it before
# it is filed as not perceived. Two ticks of a slow tiered step.
SETTLE_S = 2.0


def _jpeg_size(data: bytes):
    try:
        from PIL import Image

        with Image.open(io.BytesIO(data)) as img:
            return {"width": img.width, "height": img.height}
    except Exception:  # noqa: BLE001 -- a size is a nicety, never a failure
        return None


class StreamRecorder:
    def __init__(self, store, walk: str, fetch_frames: Callable[[int], dict], runner,
                 meta: Optional[dict] = None, poll_s: float = POLL_S,
                 settle_s: float = SETTLE_S, max_frames: int = MAX_STREAM_FRAMES,
                 clock: Callable[[], float] = time.monotonic):
        self.store = store
        self.walk = walk
        self._fetch = fetch_frames
        self._runner = runner
        self._meta = dict(meta or {})
        self.poll_s = poll_s
        self.settle_s = settle_s
        self.max_frames = max_frames
        self._clock = clock
        self._after = 0
        self._boot_id = None
        self._pending: list = []        # (fetched_at, frame), oldest first
        self._capture = None
        self.saved = 0
        self.perceived = 0
        self.lost = 0
        self.over_cap = 0
        self.failed = 0
        self.finished = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="stream-recorder", daemon=True)

    def start(self, after: int = 0) -> "StreamRecorder":
        """`after`: the robot's latest seq at the mission's start, so frames
        pushed before it are not filed under this walk."""
        self._after = after
        self._thread.start()
        return self

    def stop(self) -> None:
        """Drain what the robot still holds, then finish. Idempotent."""
        self._stop.set()

    def status(self) -> dict:
        return {"walk": self.walk, "saved": self.saved, "perceived": self.perceived,
                "lost": self.lost, "over_cap": self.over_cap, "failed": self.failed,
                "finished": self.finished.is_set()}

    # ---------- the thread ----------

    def _running(self) -> bool:
        return self._runner.is_running() and not self._stop.is_set()

    def _run(self) -> None:
        try:
            while True:
                running = self._running()
                self._poll()
                self._file(final=not running)
                if not running:
                    break
                self._stop.wait(self.poll_s)
        except Exception:  # noqa: BLE001 -- a recorder must never take the brain down
            logger.exception("stream recorder for %s stopped", self.walk)
        finally:
            self._finish()

    def _poll(self) -> None:
        try:
            got = self._fetch(self._after)
        except Exception as e:  # noqa: BLE001 -- a missed poll is retried next time
            logger.warning("stream recorder: fetch failed: %s", e)
            return
        boot = got.get("boot_id")
        if self._boot_id is not None and boot != self._boot_id:
            # The robot server restarted: its seqs began again (review, A).
            logger.warning("stream recorder: the robot restarted; cursor reset")
            self._after = 0
            got = self._fetch(0)
        self._boot_id = got.get("boot_id")
        self.lost += int(got.get("lost") or 0)
        now = self._clock()
        for frame in got.get("frames") or []:
            seq = frame["metadata"]["seq"]
            if seq > self._after:
                self._pending.append((now, frame))
                self._after = seq

    def _file(self, final: bool) -> None:
        decided_past = self._runner.last_frame_seq
        now = self._clock()
        while self._pending:
            fetched_at, frame = self._pending[0]
            seq = frame["metadata"]["seq"]
            settled = (final or now - fetched_at >= self.settle_s
                       or (decided_past is not None and decided_past >= seq))
            if not settled:
                break
            self._pending.pop(0)
            self._save(frame)

    def _save(self, frame: dict) -> None:
        seq = frame["metadata"]["seq"]
        decision = self._runner.decision_for(seq)
        try:
            image = base64.b64decode(frame["image_base64"])
            if self._capture is None:
                self._capture = _jpeg_size(image)
            store_frame(self.store, self.walk, self.saved, image,
                        frame.get("media_type", "image/jpeg"),
                        {"navigate": decision, "teleop_seq": seq,
                         "perceived": decision is not None,
                         "captured_at": frame.get("captured_at")},
                        max_frames=self.max_frames, count=self.saved)
        except WalkFull:
            self.over_cap += 1
            return
        except Exception as e:  # noqa: BLE001 -- counted; the walk goes on
            self.failed += 1
            if self.failed == 1:
                logger.warning("stream recorder: a frame was not saved: %s", e)
            return
        self.saved += 1
        self.perceived += decision is not None

    def _finish(self) -> None:
        try:
            if self.saved:
                finish_walk_meta(
                    self.store, self.walk, source="stream",
                    frames_perceived=self.perceived, frames_lost=self.lost,
                    frames_over_cap=self.over_cap or None,
                    frames_failed=self.failed or None, capture=self._capture,
                    **{k: v for k, v in self._meta.items() if v is not None})
        except Exception:  # noqa: BLE001
            logger.exception("stream recorder: finishing %s failed", self.walk)
        finally:
            self.finished.set()
