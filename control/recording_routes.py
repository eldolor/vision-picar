"""The two routes that WRITE a recorded walk, mountable by either server.

`POST /recording/frame` and `POST /recording/finish` began life in
`control/brain_server.py`, and lived there for a good reason: the brain was
the process holding the EFS mount, so it was the only one that could store
anything. That reason is gone twice over.

First, the walks moved to S3 (`control/walk_store.py`), so "who can write a
walk" is now a question about credentials rather than about which container
has a volume attached. Second, the brain is going back to the Pi -- which
is where `PLAN-brain-relocation.md` always said it belonged -- and a robot
on someone's floor should not be carrying AWS credentials so that it can
put an object in a bucket.

So the write path follows the storage instead of the brain. These routes
mount into whichever process owns the bucket: today the recordings Lambda
(`service/lambda/walks_handler.py`) in the cloud, and `brain_server` itself
when you are running the whole thing locally against a directory.

## Why this is safe to move and mission control is not

`PLAN-teleop-robot.md`'s "Recording proxy" section already argued exactly
this for the proxy case, and the argument is unchanged: **`record_frame()`
touches no robot state, no runner state and no mission state. It is pure
storage.** Nothing about it needs to be co-located with the thing driving
the car, which is why it was the only route ever allowed to be proxied
between two brains. `/mission/*` is the opposite and must never move: it
holds the live mission, and a second process believing it owns one is how
two brains end up driving the same robot.

## The proxy hook

`brain_server` can be configured to forward these to a peer that owns
storage (`recording_proxy_url` -- teleop-brain does this today). That
behaviour is brain-specific and is injected here as `proxy`, rather than
being reimplemented: a mounting process that owns its own storage passes
nothing and the branch is simply absent.
"""

from __future__ import annotations

import base64
import binascii
import json
import re
import time
from typing import Callable, Optional

from fastapi import Depends, HTTPException
from pydantic import BaseModel

from control.walk_store import WalkStoreError

# Kept here, with the routes that enforce them, rather than in either
# server: these ARE the naming contract for a walk, and
# control/admin_server.py validates against a copy of the same pattern
# because it may not import this module's process-level dependencies.
WALK_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
MAX_FRAME_BYTES = 4 * 1024 * 1024
MAX_FRAMES_PER_WALK = 500
FRAME_SUFFIX = {"image/jpeg": ".jpg", "image/png": ".png", "image/webp": ".webp"}


class RecordFrameRequest(BaseModel):
    walk: str
    seq: int
    image_base64: str
    media_type: str = "image/jpeg"
    # The /navigate answer this frame got live, if there was one.
    navigate: Optional[dict] = None
    # The robot's own id for this frame under teleop (sim/teleop_robot.py
    # stamps one on every push and echoes it back).
    #
    # It exists so a recorded walk can say WHICH decision saw these pixels.
    # Under "Drive via brain" the twin pushes frames on one timer, polls
    # /mission/status on another, and the mission ticks on a third -- median
    # 2.5 pushed frames per step, range 1-10 -- so the status stored beside a
    # frame routinely belongs to an earlier one. Pair this with the status's
    # `last_frame_seq` and the alignment is exact instead of coincidental.
    # Null outside teleop, where there is nothing to align.
    teleop_seq: Optional[int] = None


class FinishWalkRequest(BaseModel):
    """Sent once by the twin when a Robot-view recording stops.

    Frames arrive one at a time and nothing in the stream says which one is
    last, so without this a walk is only "finished" in the sense that no
    more frames happened to arrive. The marker it writes (meta.json) is what
    lets control/admin_server.py tell a completed walk from one still in
    progress, and it is where the walk-level model_id and target live --
    both facts the twin knows and the frames do not carry on their own.

    Best-effort by design: a closed tab or a dead battery never sends it,
    which is why admin also scores lazily on first read.
    """
    walk: str
    model_id: Optional[str] = None
    target_object: Optional[str] = None
    # The pixel size the frames were CAPTURED at, which is a property of the
    # corpus and not of any one frame.
    #
    # Added 2026-09-08, because the whole Stage 0 corpus turned out to be
    # 640x480 and nobody noticed for a month: the twin asked getUserMedia for
    # a camera with no resolution constraint, the browser returned its
    # default, and every finding about small distant targets in
    # PLAN-onboard-perception.md 4.10/4.11 was measured on VGA. A walk that
    # records its own capture size cannot hide that from the next reader --
    # the same argument Stage 0 already makes for writing the rig height into
    # the note, applied to the one number the browser knows and a person
    # cannot see.
    capture_width: Optional[int] = None
    capture_height: Optional[int] = None


def mount_recording_routes(
    app,
    store,
    *,
    prefix: str = "",
    require_secret: Optional[Callable] = None,
    allow_recording: Callable[[], bool] = lambda: True,
    proxy: Optional[Callable] = None,
) -> None:
    """Add the two write routes to `app`, backed by `store`.

    `allow_recording` is a callable rather than a bool so a process can
    decide per request without this module reading anyone's config, and
    `proxy` is the brain's forward-to-a-peer behaviour (see the module
    docstring); a process that owns its storage passes neither.
    """
    deps = [Depends(require_secret)] if require_secret else []

    def _refuse():
        raise HTTPException(status_code=403, detail="Recording is disabled on this brain.")

    @app.post(prefix + "/recording/frame", dependencies=deps)
    async def record_frame(req: RecordFrameRequest):
        """Save one walk frame. Off by default on any process that should
        not be accepting writes from whoever can reach it."""
        if not allow_recording():
            if proxy:
                return await proxy("/recording/frame", req)
            _refuse()
        if not WALK_NAME.match(req.walk):
            raise HTTPException(
                status_code=400,
                detail="walk must be 1-64 chars of letters, digits, dot, dash or underscore.",
            )
        if not 0 <= req.seq <= 9999:
            raise HTTPException(status_code=400, detail="seq out of range.")
        # Checked before decoding -- base64 is 4/3 the size of what it
        # carries, so this bounds the allocation rather than discovering the
        # size after paying for it.
        if len(req.image_base64) > MAX_FRAME_BYTES * 4 // 3 + 4:
            raise HTTPException(status_code=413, detail="Frame too large.")
        try:
            image = base64.b64decode(req.image_base64, validate=True)
        except (binascii.Error, ValueError):
            raise HTTPException(status_code=400, detail="image_base64 is not valid base64.")

        # The name is already sanitised; walk_store refuses a separator or
        # a `..` again on the way in, on either backend.
        try:
            store.create_walk(req.walk)
        except WalkStoreError:
            raise HTTPException(status_code=400, detail="Bad walk name.")

        # "starts with frame-", not "isn't walk.jsonl": control/admin_server.py
        # can leave a tags.json sidecar beside these, which a suffix-exclusion
        # check would miscount as a frame.
        existing = store.list_names(req.walk, prefix="frame-")
        if len(existing) >= MAX_FRAMES_PER_WALK:
            raise HTTPException(
                status_code=409,
                detail=f"This walk already has {MAX_FRAMES_PER_WALK} frames.",
            )

        suffix = FRAME_SUFFIX.get(req.media_type, ".jpg")
        name = f"frame-{req.seq:04d}{suffix}"
        store.write_bytes(req.walk, name, image)
        store.append_text(req.walk, "walk.jsonl", json.dumps({
            "seq": req.seq, "file": name, "media_type": req.media_type,
            "navigate": req.navigate,
            "teleop_seq": req.teleop_seq,
        }) + "\n")

        return {"saved": name, "walk": req.walk, "frames": len(existing) + 1,
                "dir": store.location(req.walk)}

    @app.post(prefix + "/recording/finish", dependencies=deps)
    async def finish_walk(req: FinishWalkRequest):
        """Mark a recorded walk complete and record what produced it.

        Only ever writes meta.json beside the frames -- it deliberately does
        not score anything. Scoring lives in control/admin_server.py, which
        is the process that owns reviewing recordings and the one that has
        Bedrock permissions; this route's job ends when the frames are
        safely stored.
        """
        if not allow_recording():
            if proxy:
                return await proxy("/recording/finish", req)
            _refuse()
        if not WALK_NAME.match(req.walk):
            raise HTTPException(status_code=400, detail="Bad walk name.")

        try:
            if not store.walk_exists(req.walk):
                raise HTTPException(status_code=404, detail="No such walk.")
        except WalkStoreError:
            raise HTTPException(status_code=400, detail="Bad walk name.")

        meta = store.read_json(req.walk, "meta.json") or {}
        meta["finished_at"] = time.time()
        meta["frames"] = len(store.list_names(req.walk, prefix="frame-"))
        if req.model_id:
            meta["model_id"] = req.model_id
        if req.target_object:
            meta["target_object"] = req.target_object
        if req.capture_width and req.capture_height:
            meta["capture"] = {"width": req.capture_width,
                               "height": req.capture_height}
        store.write_json(req.walk, "meta.json", meta)
        return {"walk": req.walk, "meta": meta}
