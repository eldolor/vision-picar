"""
admin_server.py

Viewer/deleter for Robot-view "Record this walk" data (recording_dir on the
shared EFS volume, cloudformation/recordings.yaml) -- deliberately its own
process, not a route inside control/brain_server.py.

**Why separate from the brain.** The brain (control/brain_server.py) is the
part of this system with a real destination other than AWS: PLAN-brain-
relocation.md's whole point is that it moves to the Pi once the PiCar-X
exists. Reviewing and pruning recorded walks has no reason to move with it
-- it is an ops/data-management concern, not the autonomy loop, and staying
independent means it keeps working (and keeps its own uptime) regardless of
whether the brain is currently on Fargate, a MacBook, or the Pi, as long as
this process can still reach the recordings volume. Bundling the two would
have meant this page going down every time the mission server restarts, and
would have tied a "where do I review my data" question to "where does the
robot's brain happen to live today" -- two questions with different
answers.

This module is intentionally thin: inventory, JPEG bytes, delete, and the
walk scorecard, over the same recording_dir control/brain_server.py's
POST /recording/frame already writes to (that route stays in brain_server.py
-- it is the live capture path, tightly coupled to an active Robot-view
session, unlike browsing history after the fact). No mission logic, no
RemoteRobot, no vision policy -- this process still never talks to the
robot. It does now call the vision service, but only for replay -- see
below.

**On replay calling the vision service.** POST .../replay re-asks a recorded
walk's frames under a different model, which means calling the deployed
/navigate. That is a deliberate exception to the paragraph above, argued in
control/walk_replay.py's docstring: a replay's whole question is "what would
the SERVICE say about these pixels", including its prompt and parsing, so
going straight to Bedrock would answer a different question and would stop
tracking the real route the moment either changed. Only this one route does
it, and its failure is confined to itself.

**On the scorecard calling Bedrock.** This process previously made no AWS
call but the EFS mount itself, and the judge tier (control/walk_eval.py)
changes that. It does NOT weaken the separation above, for one specific
reason: Bedrock is a managed endpoint reached over the VPC's existing
bedrock-runtime interface endpoint, not a peer service in this project.
The rule this file has always kept is that reviewing recordings must not
depend on where the brain or the robot happens to be running today --
and an evaluator that reads frames off the volume it already mounts and
calls Bedrock directly keeps exactly that property. Routing the judge
through service/vision_analyze instead would have broken it: admin would
gain a hard dependency on another service's uptime, and would have to ship
frames it already holds on local disk over HTTP to have them read back.

    GET    /admin                                  the viewer/deleter page
    GET    /admin.js                               its client script
    GET    /recording/walks                        list saved walks (label, model, score)
    GET    /recording/walks/{walk}                  one walk's frames + navigate log
    GET    /recording/walks/{walk}/frames/{file}    one frame's image bytes
    GET    /recording/walks/{walk}/download         the whole walk as a .zip
    PUT    /recording/walks/{walk}/tag              set/clear this walk's label
    PUT    /recording/walks/{walk}/meta             set walk-level metadata (model_id)
    POST   /recording/walks/{walk}/evaluate         (re)score a walk
    GET    /recording/walks/{walk}/evaluation       the scorecard, computed lazily
    POST   /recording/walks/{walk}/replay           re-ask its frames under another model
    GET    /recording/walks/{walk}/replays          every stored replay of it
    DELETE /recording/walks/{walk}                  delete a whole walk
    DELETE /recording/walks/{walk}/frames/{file}    delete one frame
    GET    /stats                                   walk/frame/byte totals
    GET    /health
"""

import asyncio
import io
import json
import logging
import os
import re
import time
import zipfile
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, RedirectResponse, Response
from pydantic import BaseModel

from control.brain_config import load_brain_config
from control.metrics_routes import register_metrics_routes
from control.recording_routes import mount_recording_routes
from control import walk_eval, walk_replay
from control.walk_store import METRICS_PREFIX, WalkStoreError, walk_store_from_config

logger = logging.getLogger("admin_server")

# The judge tier's model. Opus 4.5 by default: on the 2026-08-29 frames it
# was the only candidate that both made forward progress and stayed
# obstacle-aware, and the only one that refused to call a red blanket a red
# backpack -- i.e. the best available judgement, which is the one thing a
# judge needs. Overridable without a code change, like every other model ID
# in this project.
JUDGE_MODEL_ID = os.environ.get(
    "BEDROCK_JUDGE_MODEL_ID", "us.anthropic.claude-opus-4-5-20251101-v1:0")
JUDGE_SAMPLE_FRAMES = int(os.environ.get("JUDGE_SAMPLE_FRAMES", "8"))
# The collision check looks at FORWARD frames only, from the end backwards --
# see control/walk_eval.check_collisions for why that is where they hide.
COLLISION_MAX_CHECKS = int(os.environ.get("COLLISION_MAX_CHECKS", "12"))
# Off by default so this service keeps its old behaviour (and needs no
# Bedrock permissions) unless the deployment opts in.
JUDGE_ENABLED = os.environ.get("WALK_JUDGE_ENABLED", "").lower() in ("1", "true", "yes")

# How many times one /navigate call is tried before a frame is given up on.
NAVIGATE_ATTEMPTS = 4

# How much of a walk a replay has to actually get back before its score is
# allowed to mean anything. Replay is the only controlled comparison this
# project has, so a replay that quietly lost most of its frames is worse
# than no replay at all: three prompt variants of one 22-frame walk all
# came back scored 33, on 3, 9 and 2 surviving frames respectively, which
# reads as "the wording made no difference" and was really "the vision
# service timed out". Below this a replay is still stored -- the console
# shows the error count, and knowing a comparison failed is the point --
# but left unscored, and /recording/summary already skips a null score.
REPLAY_MIN_COVERAGE = float(os.environ.get("REPLAY_MIN_COVERAGE", "0.8"))


def _bedrock_client():
    """Imported lazily and built per call site rather than at module import:
    boto3 is only needed when the judge tier is switched on, and this module
    must stay importable (and testable) without it."""
    import boto3
    from botocore.config import Config

    return boto3.client(
        "bedrock-runtime",
        config=Config(retries={"max_attempts": 3, "mode": "adaptive"}, read_timeout=90),
    )


def post_navigate(vision_url: str, timeout_s: float, image_bytes: bytes,
                  target_object: str, model_id, prompt_variant=None):
    """One call to the DEPLOYED /navigate -- see control/walk_replay.py's
    docstring for why replay goes through the service rather than straight to
    Bedrock. Module level, like _bedrock_client above, so a test can replace
    it without a live service."""
    import base64

    import httpx

    body = {
        "image_base64": base64.b64encode(image_bytes).decode(),
        "media_type": "image/jpeg",
        "target_object": target_object,
    }
    if model_id:
        body["model_id"] = model_id
    if prompt_variant:
        body["prompt_variant"] = prompt_variant
    headers = {"Content-Type": "application/json"}
    secret = os.environ.get("VISION_SHARED_SECRET") or os.environ.get("APP_SHARED_SECRET")
    if secret:
        headers["x-app-secret"] = secret
    # Retry the transient ones. A replay fires every frame of a walk at the
    # vision service at once, which is a burstier pattern than anything else
    # in this project produces, and Bedrock throttles it -- raising the
    # worker count to beat the load balancer's 60s timeout turned 0 errors
    # into 10 of 22. Backoff fixes that without trading throughput for it.
    import random
    import time as _time

    last = ""
    for attempt in range(NAVIGATE_ATTEMPTS):
        try:
            with httpx.Client(timeout=timeout_s) as client:
                resp = client.post(vision_url.rstrip("/") + "/navigate",
                                   json=body, headers=headers)
        except httpx.TransportError as e:
            # A read timeout is by far the commonest way a replay loses a
            # frame, and it arrives as an EXCEPTION rather than a status
            # code -- so it used to fall straight out of this function and
            # the backoff written just above for exactly this burst never
            # ran on the failure that dominates it. One 22-frame walk lost
            # 21 frames to timeouts and was scored anyway, as though the
            # model had answered; see the coverage guard in _replay().
            last = f"{type(e).__name__}: {e}"[:160]
        else:
            if resp.status_code == 200:
                return resp.json()
            last = f"HTTP {resp.status_code}: {resp.text[:160]}"
            # 4xx other than 429 is our own bad request -- retrying cannot help.
            if resp.status_code < 500 and resp.status_code != 429:
                break
        if attempt < NAVIGATE_ATTEMPTS - 1:
            _time.sleep((0.6 * 2 ** attempt) + random.uniform(0, 0.4))
    raise RuntimeError(last)


_ADMIN_HTML = Path(__file__).resolve().parent / "admin.html"
_ADMIN_JS = Path(__file__).resolve().parent / "admin.js"

# Same patterns control/brain_server.py's record_frame() validates against
# (it owns the write path and therefore the naming contract) -- duplicated
# rather than imported, for the same reason robot/server.py's require_secret
# is duplicated elsewhere in this project: importing brain_server would drag
# in RemoteRobot, MissionRunner and the vision policy, which this process
# has no business touching.
WALK_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
FRAME_FILENAME = re.compile(r"^frame-\d{4}\.(jpg|png|webp)$")
FRAME_CONTENT_TYPE = {".jpg": "image/jpeg", ".png": "image/png", ".webp": "image/webp"}

# A walk's curation label, e.g. for judging what's worth keeping for future
# RL training data -- stored as one small sidecar file per walk rather than
# a database, matching this module's "no infrastructure beyond a directory"
# scope. Fixed set rather than free text: this is a workflow state (keep it
# reviewable/filterable), not a notes field.
TAG_FILE_NAME = "tags.json"
VALID_LABELS = {"good", "bad", "training-ready"}

# The machine scorecard (control/walk_eval.py), kept in its own sidecar and
# deliberately NOT written into tags.json. The label above is the operator's
# own curation vocabulary and a workflow state; this is an opinion produced
# by a scorer whose thresholds are calibrated against six walks. Merging
# them would let the machine overwrite a human judgement and would make
# "who decided this walk was bad" unanswerable. They are shown side by side
# in the console instead.
EVAL_FILE_NAME = "eval.json"

# One stored replay per model, so re-asking the same walk under the same
# model is free and a walk accumulates a comparison table rather than
# overwriting one. Slugged because a model id contains ":" and "." and has
# to survive being a filename.
REPLAY_PREFIX = "replay-"


def _replay_file(model_id: str, prompt_variant=None) -> str:
    """One file per (model, prompt) pair -- the two axes are independent and a
    walk should be able to hold a grid of both, not one result per model."""
    key = model_id if not prompt_variant or prompt_variant == "default" \
        else f"{model_id}__{prompt_variant}"
    return REPLAY_PREFIX + re.sub(r"[^A-Za-z0-9._-]", "_", key) + ".json"

# Walk-level facts the frames themselves don't carry -- written by
# brain_server's POST /recording/finish, and backfillable for the walks
# recorded before that route existed (see PUT /recording/walks/{walk}/meta).
META_FILE_NAME = "meta.json"


class TagRequest(BaseModel):
    label: Optional[str] = None  # None/omitted clears the label


class ReplayRequest(BaseModel):
    """Which model to re-ask a recorded walk's frames under. Omitted means
    the vision service's own default."""
    model_id: Optional[str] = None
    # The other axis. Varying the wording over a recorded walk is the
    # experiment this project most needs and could never run before.
    prompt_variant: Optional[str] = None


class MetaRequest(BaseModel):
    """Backfill for walks recorded before the model was captured per frame.

    Every walk recorded up to 2026-08-29 was produced by whatever
    BEDROCK_NAVIGATE_MODEL_ID the deployed service happened to carry, which
    was NOT the model the docs named -- so their model has to be supplied
    from outside rather than inferred from the log."""
    model_id: Optional[str] = None
    target_object: Optional[str] = None
    note: Optional[str] = None


def _read_label(store, walk: str) -> Optional[str]:
    tags = store.read_json(walk, TAG_FILE_NAME)
    return tags.get("label") if tags else None


def _walk_entries(store, walk: str) -> list:
    try:
        text = store.read_text(walk, "walk.jsonl")
    except FileNotFoundError:
        return []
    entries = []
    for line in text.splitlines():
        if line.strip():
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return sorted(entries, key=lambda e: e.get("seq", 0))


def _walk_model_id(store, walk: str, entries: list) -> Optional[str]:
    """Per-frame model_id (present since the model picker shipped) wins;
    meta.json is the fallback for walks recorded before that."""
    for e in entries:
        mid = (e.get("navigate") or {}).get("model_id")
        if mid:
            return mid
    meta = store.read_json(walk, META_FILE_NAME) or {}
    return meta.get("model_id")


# newWalkName() in web-twin builds "<target>-<model tag>-YYYYMMDD-HHMMSS".
WALK_TIMESTAMP = re.compile(r"(\d{8})-(\d{6})$")


def _walk_recorded_at(walk_name: str, store, walk: str) -> Optional[float]:
    """When this walk was recorded, as an epoch seconds float.

    Parsed from the name rather than taken from the directory's mtime: the
    console writes eval.json and tags.json into the walk directory, so mtime
    moves every time a walk is scored or labelled, and a list sorted by it
    would reshuffle itself as you used it.

    This exists because the console's "Newest first" used to sort on the
    walk NAME, which meant time order only while every name was
    "<target>-<timestamp>". Once the model tag was added to the name, that
    sort silently became "group by model" -- and a run of walks on one model
    buried the walks recorded either side of it on another.
    """
    m = WALK_TIMESTAMP.search(walk_name)
    if m:
        import datetime
        try:
            return datetime.datetime.strptime(
                m.group(1) + m.group(2), "%Y%m%d%H%M%S").timestamp()
        except ValueError:
            pass
    meta = store.read_json(walk, META_FILE_NAME) or {}
    return meta.get("finished_at")


def _walk_target(store, walk_name: str) -> str:
    """The target object, for the judge's prompt. meta.json if recorded;
    otherwise recovered from the walk name, which newWalkName() builds as
    "<target>-<model tag>-<timestamp>"."""
    meta = store.read_json(walk_name, META_FILE_NAME) or {}
    if meta.get("target_object"):
        return meta["target_object"]
    stem = re.sub(r"-\d{8}-\d{6}$", "", walk_name)
    # Strip the prompt tag before the model tag -- newWalkName() appends them
    # in that order, so they come off in reverse. Getting this wrong makes the
    # target "blue bottle next step and walls", which then goes into the
    # judge's and the collision check's prompts.
    stem = re.sub(r"-(default|next-step[a-z0-9-]*)$", "", stem)
    stem = re.sub(r"-(nova|claude|haiku|sonnet|opus|qwen|llama|pixtral)[a-z0-9-]*$", "", stem)
    return stem.replace("-", " ") or "the target object"


FRAME_PREFIX = "frame-"
# Matches record_frame()'s naming exactly (frame-{seq:04d}{suffix}) --
# deliberately "starts with frame-", not "isn't walk.jsonl/tags.json", so a
# future sidecar file doesn't silently get counted as a frame the way
# tags.json briefly would have under a suffix-exclusion check.


def require_secret(x_app_secret: str = Header(default="")):
    """This service's own secret -- deliberately not the brain's. Being
    able to review or delete recorded walks is a different privilege than
    being able to start a mission, and the two processes are independently
    deployed, so there is no reason for them to share one."""
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


def create_app(config_path=None, store=None) -> FastAPI:
    config = load_brain_config(config_path)
    # The one place this service decides where walks live. Injectable so a
    # test can hand in a store without a config file, the same way the
    # Bedrock client factory is injectable.
    store = store if store is not None else walk_store_from_config(config)

    app = FastAPI(title="vision-picar admin server")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["GET", "PUT", "DELETE", "OPTIONS"],
        allow_headers=["*"],
    )

    def _require_walk(walk: str) -> str:
        """Validate the name and confirm the walk exists, or 400/404.

        The traversal guard now lives in control/walk_store.py, which
        refuses a separator or a `..` on either backend -- this kept
        comparing resolved parents, which only ever worked because
        Path.resolve() collapses `..` and an S3 key does not."""
        if not WALK_NAME.match(walk):
            raise HTTPException(status_code=400, detail="Bad walk name.")
        # A day of mission metrics shares the store but is not a walk
        # (handoff 4c): it must not be readable, replayable or DELETABLE
        # through a walk route.
        if walk.startswith(METRICS_PREFIX) or not store.walk_exists(walk):
            raise HTTPException(status_code=404, detail="No such walk.")
        return walk

    def _require_frame(walk: str, filename: str) -> str:
        if not FRAME_FILENAME.match(filename):
            raise HTTPException(status_code=400, detail="Bad frame filename.")
        if not store.file_exists(walk, filename):
            raise HTTPException(status_code=404, detail="No such frame.")
        return filename

    # The write half of the recording API. In the cloud this process IS the
    # thing holding the bucket, so /recording/frame and /recording/finish
    # belong here rather than on a brain that is going back to the Pi and
    # should not carry AWS credentials. Mounted from the same module
    # control/brain_server.py mounts, so there is one implementation.
    mount_recording_routes(app, store, require_secret=require_secret)
    # Tier metrics: same store, same service, same argument the
    # recording write path makes -- the write follows the storage.
    register_metrics_routes(app, store, require_secret)

    @app.get("/recording/health")
    async def recording_health():
        """Same answer as /health, under the prefix this service owns.

        Two services sit behind one API Gateway and each needs a health path
        that routes to it; "/health" can only point at one of them. This is
        the infrastructure showing through slightly, and it is preferable to
        the alternative of a health check that silently reports on the wrong
        process."""
        return await health()

    @app.get("/recording/walks", dependencies=[Depends(require_secret)])
    async def list_walks():
        if not store.available():
            return {"recording_dir": store.describe(), "walks": []}
        walks = []
        for name in store.list_walks():
            frames = [(f, size) for f, size in store.list_files(name)
                      if f.startswith(FRAME_PREFIX)]
            walks.append({
                "walk": name,
                "frames": len(frames),
                "bytes": sum(size for _, size in frames),
                "label": _read_label(store, name),
                "model_id": _walk_model_id(store, name, _walk_entries(store, name)),
                "finished": store.file_exists(name, META_FILE_NAME),
                "recorded_at": _walk_recorded_at(name, store, name),
                "eval": _eval_summary(name),
                "replays": _replay_summaries(name),
            })
        return {"recording_dir": store.describe(), "walks": walks}

    @app.get("/recording/walks/{walk}", dependencies=[Depends(require_secret)])
    async def get_walk(walk: str):
        _require_walk(walk)
        frames = sorted(
            ({"file": f, "bytes": size} for f, size in store.list_files(walk)
             if f.startswith(FRAME_PREFIX)),
            key=lambda f: f["file"],
        )
        entries = _walk_entries(store, walk)
        return {
            "walk": walk, "frames": frames, "entries": entries,
            "label": _read_label(store, walk),
            "model_id": _walk_model_id(store, walk, entries),
            "meta": store.read_json(walk, META_FILE_NAME),
            "eval": store.read_json(walk, EVAL_FILE_NAME),
        }

    @app.get("/recording/walks/{walk}/frames/{filename}", dependencies=[Depends(require_secret)])
    async def get_frame(walk: str, filename: str):
        _require_walk(walk)
        _require_frame(walk, filename)
        suffix = "." + filename.rsplit(".", 1)[-1]
        media_type = FRAME_CONTENT_TYPE.get(suffix, "application/octet-stream")
        return Response(content=store.read_bytes(walk, filename), media_type=media_type)

    @app.get("/recording/walks/{walk}/download", dependencies=[Depends(require_secret)])
    async def download_walk(walk: str):
        _require_walk(walk)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for name, _size in store.list_files(walk):
                zf.writestr(name, store.read_bytes(walk, name))
        data = buf.getvalue()

        # Where the backend can hand out a URL, redirect instead of
        # returning the bytes. On S3 + Lambda that is not an optimisation:
        # a function can return at most ~6MB and two walks in the corpus zip
        # to 8.64MB and 6.68MB, so proxying them through is already broken.
        # Backend decides, not size -- see WalkStore.download_url().
        url = store.download_url(f"{walk}.zip", data, "application/zip")
        if url:
            return RedirectResponse(url, status_code=307)
        return Response(
            content=data,
            media_type="application/zip",
            headers={"Content-Disposition": f'attachment; filename="{walk}.zip"'},
        )

    @app.put("/recording/walks/{walk}/tag", dependencies=[Depends(require_secret)])
    async def tag_walk(walk: str, req: TagRequest):
        _require_walk(walk)
        if req.label is None or req.label == "":
            if store.file_exists(walk, TAG_FILE_NAME):
                store.delete_file(walk, TAG_FILE_NAME)
            return {"walk": walk, "label": None}
        if req.label not in VALID_LABELS:
            raise HTTPException(
                status_code=400,
                detail=f"label must be one of {sorted(VALID_LABELS)} or omitted to clear it.",
            )
        store.write_json(walk, TAG_FILE_NAME, {"label": req.label})
        return {"walk": walk, "label": req.label}

    # ---- the scorecard (control/walk_eval.py) ----

    def _evaluate(walk_name: str, use_judge: bool) -> dict:
        """Compute and persist one walk's scorecard. Synchronous and
        blocking (the judge is several Bedrock calls), so routes run it in a
        worker thread rather than on the event loop."""
        entries = _walk_entries(store, walk_name)
        metrics = walk_eval.compute_metrics(entries)

        def frame_bytes_for(entry):
            name = entry.get("file") or ""
            if not FRAME_FILENAME.match(name):
                return None
            try:
                return store.read_bytes(walk_name, name)
            except FileNotFoundError:
                return None

        judge = None
        if use_judge and JUDGE_ENABLED and entries:
            try:
                judge = walk_eval.judge_walk(
                    _bedrock_client(), JUDGE_MODEL_ID, entries, frame_bytes_for,
                    _walk_target(store, walk_name), sample=JUDGE_SAMPLE_FRAMES,
                )
            except Exception as e:  # noqa: BLE001 -- fall back to metrics-only
                logger.warning("judge tier failed for %s: %s", walk_name, e)
                judge = {"judged": 0, "sensible_rate": None, "error": str(e)[:200]}

        collisions = None
        if use_judge and JUDGE_ENABLED and entries:
            try:
                collisions = walk_eval.check_collisions(
                    _bedrock_client(), JUDGE_MODEL_ID, entries, frame_bytes_for,
                    target_object=_walk_target(store, walk_name),
                    max_checks=COLLISION_MAX_CHECKS)
            except Exception as e:  # noqa: BLE001 -- degrade, don't fail the walk
                logger.warning("collision check failed for %s: %s", walk_name, e)

        scored = walk_eval.score_walk(metrics, judge, collisions)
        result = {
            "schema": walk_eval.SCHEMA_VERSION,
            "walk": walk_name,
            "model_id": _walk_model_id(store, walk_name, entries),
            "target_object": _walk_target(store, walk_name),
            **scored,
            "metrics": metrics,
            "judge": judge,
            "collisions": collisions,
        }
        try:
            store.write_json(walk_name, EVAL_FILE_NAME, result)
        except (OSError, WalkStoreError) as e:
            logger.warning("could not persist eval for %s: %s", walk_name, e)
        return result

    def _marked(replay: dict) -> dict:
        """A replay as sent to a client: `stale` when an older scorer wrote
        it (3.64 E2), computed on the way out and never stored."""
        return {**replay, "stale": replay.get("schema") != walk_eval.SCHEMA_VERSION}

    def _replay_summaries(walk_name: str) -> list:
        """The compact form the walk list shows -- never the per-frame diff,
        which is large and only wanted on one walk at a time."""
        out = []
        for name in store.list_names(walk_name, REPLAY_PREFIX, ".json"):
            r = store.read_json(walk_name, name)
            if r is None:
                continue
            out.append({**{k: r.get(k) for k in
                           ("model_id", "prompt_variant", "score", "verdict", "flags",
                            "agreement", "errors", "coverage")},
                        # 3.64 E2: scored by an older scorer; replay it again.
                        "stale": r.get("schema") != walk_eval.SCHEMA_VERSION})
        return out

    def _eval_summary(walk_name: str) -> Optional[dict]:
        """The compact form the walk list shows -- never the full per-frame
        judge output, which is large and only wanted on one walk at a time."""
        ev = store.read_json(walk_name, EVAL_FILE_NAME)
        if not ev:
            return None
        # 3.64 E2: an older scorer's card is not this scorer's score. Marked,
        # not hidden: listing it as unscored would have the console's
        # auto-scorer re-run the paid judge over the whole corpus after a
        # schema bump. Re-score is the person's call; the summary skips it.
        return {**{k: ev.get(k) for k in ("score", "verdict", "flags", "basis", "model_id")},
                "stale": ev.get("schema") != walk_eval.SCHEMA_VERSION}

    @app.post("/recording/walks/{walk}/evaluate", dependencies=[Depends(require_secret)])
    async def evaluate_walk(walk: str, judge: bool = True):
        """Score one walk, overwriting any previous scorecard. `judge=false`
        runs the free deterministic tier only."""
        _require_walk(walk)
        return await asyncio.to_thread(_evaluate, walk, judge)

    @app.get("/recording/walks/{walk}/evaluation", dependencies=[Depends(require_secret)])
    async def get_evaluation(walk: str):
        """The stored scorecard, computing it lazily on first read.

        This is the fallback half of the trigger design: POST
        /recording/finish marks a walk complete, but a phone that lost its
        connection (or a tab closed mid-walk) never sends it, and a walk
        nobody can score is worse than one scored a few seconds late.
        """
        _require_walk(walk)
        existing = store.read_json(walk, EVAL_FILE_NAME)
        if existing and existing.get("schema") == walk_eval.SCHEMA_VERSION:
            return existing
        return await asyncio.to_thread(_evaluate, walk, True)

    # ---- replay: the same pixels, a different model ----

    def _replay(walk_name: str, model_id, prompt_variant=None) -> dict:
        # Checked before any frame is read: a deployment with no vision
        # service configured should say so once, not once per frame.
        if not config["vision_url"]:
            raise RuntimeError(
                "No vision service configured. Set brain.vision_url in "
                "config/robot.yaml, or VISION_URL in this task's environment.")
        entries = _walk_entries(store, walk_name)
        target = _walk_target(store, walk_name)

        def frame_bytes_for(entry):
            name = entry.get("file") or ""
            if not FRAME_FILENAME.match(name):
                return None
            try:
                return store.read_bytes(walk_name, name)
            except FileNotFoundError:
                return None

        def call(image_bytes, target_object, chosen, variant):
            # replay_timeout_s, NOT the mission's vision_timeout_s -- see
            # control/brain_config.py for why those are different questions.
            return post_navigate(config["vision_url"], config["replay_timeout_s"],
                                 image_bytes, target_object, chosen, variant)

        out = walk_replay.replay_walk(
            entries, frame_bytes_for, target, call, model_id=model_id,
            prompt_variant=prompt_variant)

        # How much of the walk actually came back. A replay is only evidence
        # about a model or a wording to the extent the service answered; see
        # REPLAY_MIN_COVERAGE.
        attempted = out["frames"]
        coverage = round(len(out["entries"]) / attempted, 3) if attempted else 0.0

        # Score the replay with the SAME scorer the original gets, by handing
        # it entries in the same shape -- so the two numbers are comparable
        # and nothing about replays is special-cased in walk_eval.
        metrics = walk_eval.compute_metrics(out["entries"])

        # Including the collision check. Without it a replay cannot answer the
        # question the feature exists for -- "would this model or this wording
        # have driven into that wall" -- and would report a clean-looking
        # score for a run that hit something, which is worse than reporting
        # nothing. It costs up to COLLISION_MAX_CHECKS extra calls on top of
        # the replay's own.
        usable = coverage >= REPLAY_MIN_COVERAGE
        collisions = None
        if JUDGE_ENABLED and out["entries"] and usable:
            try:
                collisions = walk_eval.check_collisions(
                    _bedrock_client(), JUDGE_MODEL_ID, out["entries"], frame_bytes_for,
                    target_object=target, max_checks=COLLISION_MAX_CHECKS)
            except Exception as e:  # noqa: BLE001 -- degrade, don't lose the replay
                logger.warning("collision check failed for replay of %s: %s", walk_name, e)

        if usable:
            scored = walk_eval.score_walk(metrics, None, collisions)
        else:
            # Deliberately not a low score: a low score is a claim about the
            # model, and this is a statement about the harness. Scoring these
            # anyway is what made three timed-out variants of one walk look
            # like three equivalent wordings. A null score also keeps them
            # out of /recording/summary, which already skips one.
            logger.warning("replay of %s under %s/%s returned %d of %d frames -- not scoring",
                           walk_name, model_id or "(service default)",
                           prompt_variant or "default", len(out["entries"]), attempted)
            scored = {"score": None, "verdict": "unusable", "flags": ["incomplete"]}
        result = {
            "schema": walk_eval.SCHEMA_VERSION,
            "walk": walk_name,
            "model_id": model_id or "(service default)",
            "prompt_variant": prompt_variant or "default",
            "target_object": target,
            "replayed_at": time.time(),
            **scored,
            "metrics": metrics,
            # Frames attempted, which is not metrics.frames -- that counts
            # only the ones that came back, so the two differ when the
            # vision service dropped some.
            "frames": out["frames"],
            # What fraction of those came back. The number that says whether
            # the rest of this record is evidence about the model at all.
            "coverage": coverage,
            "agreement": out["agreement"],
            "errors": out["errors"],
            "collisions": collisions,
            "diff": out["diff"],
        }
        name = _replay_file(result["model_id"], prompt_variant)
        if not usable:
            # 3.64 C3: one file per (model, prompt), so an unusable attempt
            # would erase a scored replay -- the comparison it was meant to
            # add to. The scored record is kept and notes the attempt, which
            # is also how the console knows this request has come back.
            try:
                kept = store.read_json(walk_name, name)
            except (OSError, WalkStoreError):
                kept = None
            if kept and kept.get("score") is not None:
                # Even an older scorer's: its per-frame answers were paid
                # for and can be re-scored; an unusable attempt has none.
                kept["last_unusable"] = {k: result[k] for k in
                                         ("replayed_at", "coverage", "frames", "errors")}
                result = kept
        try:
            store.write_json(walk_name, name, result)
        except (OSError, WalkStoreError) as e:
            logger.warning("could not persist replay for %s: %s", walk_name, e)
        return _marked(result)

    @app.post("/recording/walks/{walk}/replay", dependencies=[Depends(require_secret)])
    async def replay_walk_route(walk: str, req: ReplayRequest):
        """Re-ask this walk's frames under another model.

        The only controlled model comparison available: identical pixels,
        one variable. Comparing two live walks instead mixes model quality
        with where the operator pointed the phone.
        """
        _require_walk(walk)
        try:
            return await asyncio.to_thread(_replay, walk, req.model_id,
                                           req.prompt_variant)
        except RuntimeError as e:
            raise HTTPException(status_code=503, detail=str(e))

    @app.get("/recording/models", dependencies=[Depends(require_secret)])
    async def replay_models():
        """Relay the vision service's own allow-list, so the console never
        carries a copy of model ids -- same rule the twin's picker follows.

        Its own route rather than letting the page fetch /navigate/models
        directly: that only happens to work because both services sit behind
        one load balancer, and it would break the console when admin is run
        on its own in local dev.
        """
        vision_url = config["vision_url"]
        if not vision_url:
            return {"models": [], "default": None, "detail": "No vision service configured."}

        def fetch():
            import httpx

            headers = {}
            secret = os.environ.get("VISION_SHARED_SECRET") or os.environ.get("APP_SHARED_SECRET")
            if secret:
                headers["x-app-secret"] = secret
            with httpx.Client(timeout=config["request_timeout_s"]) as client:
                r = client.get(vision_url.rstrip("/") + "/navigate/models", headers=headers)
            r.raise_for_status()
            return r.json()

        try:
            return await asyncio.to_thread(fetch)
        except Exception as e:  # noqa: BLE001 -- an empty picker beats a broken page
            logger.warning("could not fetch navigate models: %s", e)
            return {"models": [], "default": None, "detail": str(e)[:200]}

    @app.get("/recording/walks/{walk}/replays", dependencies=[Depends(require_secret)])
    async def list_replays(walk: str):
        """Every stored replay of this walk, so it accumulates a comparison
        table instead of overwriting one."""
        _require_walk(walk)
        out = []
        for name in store.list_names(walk, REPLAY_PREFIX, ".json"):
            r = store.read_json(walk, name)
            if r is not None:
                out.append(_marked(r))
        return {"walk": walk, "replays": out}

    @app.put("/recording/walks/{walk}/meta", dependencies=[Depends(require_secret)])
    async def set_meta(walk: str, req: MetaRequest):
        """Set walk-level metadata -- in practice the model, for the walks
        recorded before /navigate echoed it back per frame."""
        _require_walk(walk)
        meta = store.read_json(walk, META_FILE_NAME) or {}
        for field in ("model_id", "target_object", "note"):
            value = getattr(req, field)
            if value is not None:
                meta[field] = value
        store.write_json(walk, META_FILE_NAME, meta)
        return {"walk": walk, "meta": meta}

    @app.delete("/recording/walks/{walk}", dependencies=[Depends(require_secret)])
    async def delete_walk(walk: str):
        _require_walk(walk)
        store.delete_walk(walk)
        return {"deleted": walk}

    @app.delete("/recording/walks/{walk}/frames/{filename}", dependencies=[Depends(require_secret)])
    async def delete_frame(walk: str, filename: str):
        _require_walk(walk)
        _require_frame(walk, filename)
        store.delete_file(walk, filename)
        if store.file_exists(walk, "walk.jsonl"):
            lines = [ln for ln in store.read_text(walk, "walk.jsonl").splitlines()
                     if ln.strip()]
            kept = [ln for ln in lines if json.loads(ln).get("file") != filename]
            store.write_text(walk, "walk.jsonl", "".join(ln + "\n" for ln in kept))
        return {"deleted": filename, "walk": walk}

    @app.get("/admin")
    async def admin_page():
        return FileResponse(_ADMIN_HTML)

    @app.get("/admin.js")
    async def admin_script():
        """Extracted out of admin.html -- see web-twin/app.js's banner for
        why these are external now and why they're served no-cache rather
        than under a content-hashed filename."""
        return FileResponse(
            _ADMIN_JS,
            media_type="application/javascript",
            headers={"Cache-Control": "no-cache"},
        )

    @app.get("/recording/summary", dependencies=[Depends(require_secret)])
    async def summary():
        """Per-model totals across every scored walk AND every stored replay.

        The aggregation that was being done by hand -- and, being done by
        hand, was done differently each time and had to be retracted twice.
        Replays are folded in beside recordings because a replay is the more
        trustworthy evidence: it holds the pixels fixed and varies one thing,
        where two live walks vary the operator's path as well.
        """
        if not store.available():
            return {"rows": []}

        acc = {}

        def add(model_id, prompt_variant, ev, source):
            if not ev or ev.get("score") is None:
                return
            if ev.get("schema") != walk_eval.SCHEMA_VERSION:
                return                  # 3.64 E2: an older scorer's number
            key = (model_id or "(unknown)", prompt_variant or "default", source)
            row = acc.setdefault(key, {
                "model_id": key[0], "prompt_variant": key[1], "source": source,
                "walks": 0, "scores": [], "reached": 0, "collisions": 0,
                "flags": {}, "frames": [],
            })
            row["walks"] += 1
            row["scores"].append(ev["score"])
            metrics = ev.get("metrics") or {}
            if metrics.get("target_reached"):
                row["reached"] += 1
            if metrics.get("frames"):
                row["frames"].append(metrics["frames"])
            if "collision" in (ev.get("flags") or []):
                row["collisions"] += 1
            for f in (ev.get("flags") or []):
                row["flags"][f] = row["flags"].get(f, 0) + 1

        for name in store.list_walks():
            entries = _walk_entries(store, name)
            ev = store.read_json(name, EVAL_FILE_NAME)
            add(_walk_model_id(store, name, entries),
                (ev or {}).get("prompt_variant"), ev, "recorded")
            for fname in store.list_names(name, REPLAY_PREFIX, ".json"):
                r = store.read_json(name, fname)
                if r is None:
                    continue
                add(r.get("model_id"), r.get("prompt_variant"), r, "replay")

        rows = []
        for row in acc.values():
            scores = sorted(row.pop("scores"))
            frames = sorted(row.pop("frames"))
            mid = len(scores) // 2
            rows.append({
                **row,
                "mean_score": round(sum(scores) / len(scores)),
                "median_score": scores[mid] if len(scores) % 2 else
                                round((scores[mid - 1] + scores[mid]) / 2),
                "best": scores[-1], "worst": scores[0],
                "reach_rate": round(row["reached"] / row["walks"], 2),
                "median_frames": frames[len(frames) // 2] if frames else None,
            })
        # Best first, but a model that hit something is never "best".
        rows.sort(key=lambda r: (r["collisions"] > 0, -r["mean_score"]))
        return {"rows": rows}

    @app.get("/stats", dependencies=[Depends(require_secret)])
    async def stats():
        if not store.available():
            return {"walks": 0, "frames": 0, "bytes": 0}
        walk_count = 0
        frame_count = 0
        total_bytes = 0
        for name in store.list_walks():
            walk_count += 1
            for fname, size in store.list_files(name):
                if fname.startswith(FRAME_PREFIX):
                    frame_count += 1
                    total_bytes += size
        return {"walks": walk_count, "frames": frame_count, "bytes": total_bytes}

    @app.get("/health")
    async def health():
        return {"status": "ok", "recording_dir": store.describe(),
                "recording_dir_exists": store.available(),
                # Which deployment this is. Unauthenticated on purpose --
                # /health already is, and the console has to know which
                # environment it is before anyone has typed a secret, which
                # is exactly when the confusion happens. The value is a
                # label, not a credential.
                "env_label": os.environ.get("ENV_LABEL", "").strip()}

    return app


app = create_app()
