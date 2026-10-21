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
import collections
import logging
import threading
import time
from typing import Callable, Optional

from control.recording_routes import (MAX_STREAM_FRAMES, WalkFull, finish_walk_meta,
                                      store_frame)

logger = logging.getLogger(__name__)

POLL_S = 0.25
# How long a fetched frame may wait for the mission to decide on it, or past
# it, before it is filed as not perceived: the brain's hung-tick deadline
# (B3.3's `tick_timeout_s`, 30 s), so no tick still running can be outwaited
# (review: a vision tick's 2-3 s cloud call outlasted a 2 s wait, and an
# arrival confirmation may take 8). Normally a frame settles one tick later.
SETTLE_S = 31.0
# More frames than this waiting (a mission stalled with the phone pushing)
# are filed oldest first, unperceived, to bound memory.
# Bounded by bytes, not frames (review: the right count depends on the
# phone's rate and the tick deadline): 48 MB of base64 is ~240 frames of
# 1280 px, 60 s at 4 fps or 30 s at 8; past it the oldest are filed,
# unperceived (at 8 fps a tick near the 30 s deadline loses its frames).
MAX_PENDING_BYTES = 48 * 1024 * 1024
# walk.jsonl rows appended together: an append on S3 rewrites the file.
ROW_BATCH = 25


def _jpeg_size(data: bytes):
    """Width and height from a JPEG's start-of-frame marker -- no imaging
    library, as the walks Lambda packages control/ without one. None for
    anything it cannot read: a size is a nicety, never a failure."""
    i = 2
    while i + 9 < len(data) and data[i] == 0xFF:
        marker, length = data[i + 1], int.from_bytes(data[i + 2:i + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2):
            return {"height": int.from_bytes(data[i + 5:i + 7], "big"),
                    "width": int.from_bytes(data[i + 7:i + 9], "big")}
        i += 2 + length
    return None


class StreamRecorder:
    def __init__(self, store, walk: str, fetch_frames: Callable[[int], dict], runner,
                 meta: Optional[dict] = None, poll_s: float = POLL_S,
                 settle_s: float = SETTLE_S, max_frames: int = MAX_STREAM_FRAMES,
                 clock: Callable[[], float] = time.monotonic):
        self.store = store
        self.walk = walk
        self._fetch = fetch_frames
        self.runner = runner            # whose mission this walk records
        self._meta = dict(meta or {})
        self.poll_s = poll_s
        self.settle_s = settle_s
        self.max_frames = max_frames
        self._clock = clock
        self._after = 0
        self._boot_id = None
        self._pending = collections.deque()   # (fetched_at, frame), oldest first
        self._pending_bytes = 0
        self._decided_past_warned = False
        self._rows: list = []           # (walk.jsonl line, perceived) not yet appended
        self._capture = None
        self.saved = 0
        self.rows_failed = 0            # images stored whose rows did not append
        self.lookup_failed = 0          # frames whose decision could not be read
        self.dropped = 0                # fetched, never filed: the recorder ended early
        self.written = 0                # images stored, whatever became of their rows
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
        # `written`: images stored so far; `saved`: rows appended (behind it
        # by up to a batch until the walk ends); `usable`: images with a row
        # stored or still to append. `failed` counts images never stored,
        # `rows_failed` images stored without a row.
        written, rows_failed = self.written, self.rows_failed   # one snapshot
        return {"walk": self.walk, "written": written, "saved": self.saved,
                "usable": written - rows_failed, "rows_failed": rows_failed,
                "lookup_failed": self.lookup_failed,
                "perceived": self.perceived,
                "lost": self.lost, "over_cap": self.over_cap, "failed": self.failed,
                "finished": self.finished.is_set()}

    # ---------- the thread ----------

    def _running(self) -> bool:
        return self.runner.is_running() and not self._stop.is_set()

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
            try:
                got = self._fetch(0)
            except Exception as e:  # noqa: BLE001 -- retried next poll (review)
                logger.warning("stream recorder: fetch after a restart failed: %s", e)
                return
        self._boot_id = got.get("boot_id")
        self.lost += int(got.get("lost") or 0)
        now = self._clock()
        for frame in got.get("frames") or []:
            seq = frame["metadata"]["seq"]
            if seq > self._after:
                self._pending.append((now, frame))
                self._pending_bytes += len(frame.get("image_base64") or "")
                self._after = seq

    def _file(self, final: bool) -> None:
        now = self._clock()
        while self._pending:
            fetched_at, frame = self._pending[0]
            meta = frame["metadata"]
            settled = (final or now - fetched_at >= self.settle_s
                       or self._pending_bytes > MAX_PENDING_BYTES
                       or self._decided_past(meta))
            if not settled:
                break
            self._pending.popleft()
            self._pending_bytes -= len(frame.get("image_base64") or "")
            self._save(frame)
        if self._rows and (final or len(self._rows) >= ROW_BATCH):
            self._flush_rows()

    def _decided_past(self, meta: dict) -> bool:
        try:
            return self.runner.decided_past(meta["seq"], meta.get("boot_id"))
        except Exception:  # noqa: BLE001 -- settles on time instead (review)
            if not self._decided_past_warned:
                self._decided_past_warned = True
                logger.exception("stream recorder: decided_past failed")
            return False

    def _flush_rows(self) -> None:
        """Append the held rows, once. A failed append is never retried: on
        S3 an append that raised may still have landed, and a retry would
        file the rows twice (review: a wrong walk is worse than a short
        one). Its rows are counted failed at once, so status says so while
        the walk runs. A brain killed between flushes leaves up to
        ROW_BATCH - 1 images without rows."""
        rows, self._rows = self._rows, []
        try:
            self.store.append_text(self.walk, "walk.jsonl", "".join(line for line, _ in rows))
        except Exception as e:  # noqa: BLE001
            self.rows_failed += len(rows)
            logger.warning("stream recorder: %d walk.jsonl rows not saved: %s", len(rows), e)
            return
        self.saved += len(rows)
        self.perceived += sum(1 for _, perceived in rows if perceived)

    def _save(self, frame: dict) -> None:
        seq = frame["metadata"]["seq"]
        try:
            decision = self.runner.decision_for(seq, frame["metadata"].get("boot_id"))
        except Exception:  # noqa: BLE001 -- one bad lookup costs one frame's decision (review)
            self.lookup_failed += 1
            if self.lookup_failed == 1:
                logger.exception("stream recorder: no decision for frame %s", seq)
            decision = None
        try:
            image = base64.b64decode(frame["image_base64"])
            if self._capture is None:
                self._capture = _jpeg_size(image)
            # Numbered by images written, which never goes back (review: a
            # count lowered by a failed batch made later frames overwrite
            # stored ones under the same name).
            held = []
            store_frame(self.store, self.walk, self.written, image,
                        frame.get("media_type", "image/jpeg"),
                        {"navigate": decision, "teleop_seq": seq,
                         "perceived": decision is not None,
                         "captured_at": frame.get("captured_at")},
                        max_frames=self.max_frames, count=self.written, rows=held)
        except WalkFull:
            self.over_cap += 1
            return
        except Exception as e:  # noqa: BLE001 -- counted; the walk goes on
            self.failed += 1
            if self.failed == 1:
                logger.warning("stream recorder: a frame was not saved: %s", e)
            return
        self.written += 1
        self._rows.extend((line, decision is not None) for line in held)

    def _finish(self) -> None:
        try:
            # Frames fetched but never filed (the loop ended on an error) are
            # counted, never silently gone (review).
            self.dropped = len(self._pending)
            self._pending.clear()
            if self._rows:
                self._flush_rows()
            if self.written:
                # Whenever an image was stored: a walk whose rows failed is
                # marked finished with its failures, never left looking
                # in progress (review).
                finish_walk_meta(
                    self.store, self.walk, source="stream",
                    frames_perceived=self.perceived, frames_lost=self.lost,
                    frames_over_cap=self.over_cap or None,
                    frames_failed=self.failed or None,
                    frames_without_rows=self.rows_failed or None,
                    frames_lookup_failed=self.lookup_failed or None,
                    frames_dropped=self.dropped or None, capture=self._capture,
                    **{k: v for k, v in self._meta.items() if v is not None})
        except Exception:  # noqa: BLE001
            logger.exception("stream recorder: finishing %s failed", self.walk)
        finally:
            self.finished.set()
