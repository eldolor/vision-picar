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
import shutil
import time
import zipfile
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from control.brain_config import load_brain_config
from control import walk_eval, walk_replay

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


def _read_json_sidecar(walk_dir: Path, name: str) -> Optional[dict]:
    path = walk_dir / name
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return None


def _read_label(walk_dir: Path) -> Optional[str]:
    tags = _read_json_sidecar(walk_dir, TAG_FILE_NAME)
    return tags.get("label") if tags else None


def _walk_entries(walk_dir: Path) -> list:
    jsonl_path = walk_dir / "walk.jsonl"
    if not jsonl_path.exists():
        return []
    entries = []
    for line in jsonl_path.read_text().splitlines():
        if line.strip():
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return sorted(entries, key=lambda e: e.get("seq", 0))


def _walk_model_id(walk_dir: Path, entries: list) -> Optional[str]:
    """Per-frame model_id (present since the model picker shipped) wins;
    meta.json is the fallback for walks recorded before that."""
    for e in entries:
        mid = (e.get("navigate") or {}).get("model_id")
        if mid:
            return mid
    meta = _read_json_sidecar(walk_dir, META_FILE_NAME) or {}
    return meta.get("model_id")


# newWalkName() in web-twin builds "<target>-<model tag>-YYYYMMDD-HHMMSS".
WALK_TIMESTAMP = re.compile(r"(\d{8})-(\d{6})$")


def _walk_recorded_at(walk_name: str, walk_dir: Path) -> Optional[float]:
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
    meta = _read_json_sidecar(walk_dir, META_FILE_NAME) or {}
    return meta.get("finished_at")


def _walk_target(walk_dir: Path, walk_name: str) -> str:
    """The target object, for the judge's prompt. meta.json if recorded;
    otherwise recovered from the walk name, which newWalkName() builds as
    "<target>-<model tag>-<timestamp>"."""
    meta = _read_json_sidecar(walk_dir, META_FILE_NAME) or {}
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


def _is_frame_file(p: Path) -> bool:
    # Matches record_frame()'s naming exactly (frame-{seq:04d}{suffix}) --
    # deliberately "starts with frame-", not "isn't walk.jsonl/tags.json",
    # so a future sidecar file doesn't silently get counted as a frame the
    # way tags.json briefly would have under a suffix-exclusion check.
    return p.is_file() and p.name.startswith("frame-")


def require_secret(x_app_secret: str = Header(default="")):
    """This service's own secret -- deliberately not the brain's. Being
    able to review or delete recorded walks is a different privilege than
    being able to start a mission, and the two processes are independently
    deployed, so there is no reason for them to share one."""
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


def create_app(config_path=None) -> FastAPI:
    config = load_brain_config(config_path)

    app = FastAPI(title="vision-picar admin server")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["GET", "PUT", "DELETE", "OPTIONS"],
        allow_headers=["*"],
    )

    def _resolve_walk_dir(walk: str) -> Path:
        if not WALK_NAME.match(walk):
            raise HTTPException(status_code=400, detail="Bad walk name.")
        base = Path(config["recording_dir"]).resolve()
        walk_dir = (base / walk).resolve()
        if base != walk_dir.parent or not walk_dir.is_dir():
            raise HTTPException(status_code=404, detail="No such walk.")
        return walk_dir

    def _resolve_frame_path(walk_dir: Path, filename: str) -> Path:
        if not FRAME_FILENAME.match(filename):
            raise HTTPException(status_code=400, detail="Bad frame filename.")
        frame_path = (walk_dir / filename).resolve()
        if walk_dir != frame_path.parent or not frame_path.is_file():
            raise HTTPException(status_code=404, detail="No such frame.")
        return frame_path

    @app.get("/recording/walks", dependencies=[Depends(require_secret)])
    async def list_walks():
        base = Path(config["recording_dir"]).resolve()
        if not base.is_dir():
            return {"recording_dir": str(base), "walks": []}
        walks = []
        for walk_dir in sorted(p for p in base.iterdir() if p.is_dir()):
            frames = [p for p in walk_dir.iterdir() if _is_frame_file(p)]
            walks.append({
                "walk": walk_dir.name,
                "frames": len(frames),
                "bytes": sum(p.stat().st_size for p in frames),
                "label": _read_label(walk_dir),
                "model_id": _walk_model_id(walk_dir, _walk_entries(walk_dir)),
                "finished": (walk_dir / META_FILE_NAME).exists(),
                "recorded_at": _walk_recorded_at(walk_dir.name, walk_dir),
                "eval": _eval_summary(walk_dir),
                "replays": _replay_summaries(walk_dir),
            })
        return {"recording_dir": str(base), "walks": walks}

    @app.get("/recording/walks/{walk}", dependencies=[Depends(require_secret)])
    async def get_walk(walk: str):
        walk_dir = _resolve_walk_dir(walk)
        frames = sorted(
            ({"file": p.name, "bytes": p.stat().st_size}
             for p in walk_dir.iterdir() if _is_frame_file(p)),
            key=lambda f: f["file"],
        )
        entries = _walk_entries(walk_dir)
        return {
            "walk": walk, "frames": frames, "entries": entries,
            "label": _read_label(walk_dir),
            "model_id": _walk_model_id(walk_dir, entries),
            "meta": _read_json_sidecar(walk_dir, META_FILE_NAME),
            "eval": _read_json_sidecar(walk_dir, EVAL_FILE_NAME),
        }

    @app.get("/recording/walks/{walk}/frames/{filename}", dependencies=[Depends(require_secret)])
    async def get_frame(walk: str, filename: str):
        walk_dir = _resolve_walk_dir(walk)
        frame_path = _resolve_frame_path(walk_dir, filename)
        media_type = FRAME_CONTENT_TYPE.get(frame_path.suffix, "application/octet-stream")
        return Response(content=frame_path.read_bytes(), media_type=media_type)

    @app.get("/recording/walks/{walk}/download", dependencies=[Depends(require_secret)])
    async def download_walk(walk: str):
        walk_dir = _resolve_walk_dir(walk)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for p in sorted(walk_dir.iterdir()):
                if p.is_file():
                    zf.write(p, arcname=p.name)
        return Response(
            content=buf.getvalue(),
            media_type="application/zip",
            headers={"Content-Disposition": f'attachment; filename="{walk}.zip"'},
        )

    @app.put("/recording/walks/{walk}/tag", dependencies=[Depends(require_secret)])
    async def tag_walk(walk: str, req: TagRequest):
        walk_dir = _resolve_walk_dir(walk)
        tag_path = walk_dir / TAG_FILE_NAME
        if req.label is None or req.label == "":
            tag_path.unlink(missing_ok=True)
            return {"walk": walk, "label": None}
        if req.label not in VALID_LABELS:
            raise HTTPException(
                status_code=400,
                detail=f"label must be one of {sorted(VALID_LABELS)} or omitted to clear it.",
            )
        tag_path.write_text(json.dumps({"label": req.label}))
        return {"walk": walk, "label": req.label}

    # ---- the scorecard (control/walk_eval.py) ----

    def _evaluate(walk_dir: Path, walk_name: str, use_judge: bool) -> dict:
        """Compute and persist one walk's scorecard. Synchronous and
        blocking (the judge is several Bedrock calls), so routes run it in a
        worker thread rather than on the event loop."""
        entries = _walk_entries(walk_dir)
        metrics = walk_eval.compute_metrics(entries)

        def frame_bytes_for(entry):
            name = entry.get("file") or ""
            if not FRAME_FILENAME.match(name):
                return None
            path = walk_dir / name
            return path.read_bytes() if path.is_file() else None

        judge = None
        if use_judge and JUDGE_ENABLED and entries:
            try:
                judge = walk_eval.judge_walk(
                    _bedrock_client(), JUDGE_MODEL_ID, entries, frame_bytes_for,
                    _walk_target(walk_dir, walk_name), sample=JUDGE_SAMPLE_FRAMES,
                )
            except Exception as e:  # noqa: BLE001 -- fall back to metrics-only
                logger.warning("judge tier failed for %s: %s", walk_name, e)
                judge = {"judged": 0, "sensible_rate": None, "error": str(e)[:200]}

        collisions = None
        if use_judge and JUDGE_ENABLED and entries:
            try:
                collisions = walk_eval.check_collisions(
                    _bedrock_client(), JUDGE_MODEL_ID, entries, frame_bytes_for,
                    target_object=_walk_target(walk_dir, walk_name),
                    max_checks=COLLISION_MAX_CHECKS)
            except Exception as e:  # noqa: BLE001 -- degrade, don't fail the walk
                logger.warning("collision check failed for %s: %s", walk_name, e)

        scored = walk_eval.score_walk(metrics, judge, collisions)
        result = {
            "schema": walk_eval.SCHEMA_VERSION,
            "walk": walk_name,
            "model_id": _walk_model_id(walk_dir, entries),
            "target_object": _walk_target(walk_dir, walk_name),
            **scored,
            "metrics": metrics,
            "judge": judge,
            "collisions": collisions,
        }
        try:
            (walk_dir / EVAL_FILE_NAME).write_text(json.dumps(result, indent=1))
        except OSError as e:
            logger.warning("could not persist eval for %s: %s", walk_name, e)
        return result

    def _replay_summaries(walk_dir: Path) -> list:
        """The compact form the walk list shows -- never the per-frame diff,
        which is large and only wanted on one walk at a time."""
        out = []
        for path in sorted(walk_dir.glob(REPLAY_PREFIX + "*.json")):
            try:
                r = json.loads(path.read_text())
            except (json.JSONDecodeError, OSError):
                continue
            out.append({k: r.get(k) for k in
                        ("model_id", "prompt_variant", "score", "verdict", "flags",
                         "agreement", "errors", "coverage")})
        return out

    def _eval_summary(walk_dir: Path) -> Optional[dict]:
        """The compact form the walk list shows -- never the full per-frame
        judge output, which is large and only wanted on one walk at a time."""
        ev = _read_json_sidecar(walk_dir, EVAL_FILE_NAME)
        if not ev:
            return None
        return {k: ev.get(k) for k in ("score", "verdict", "flags", "basis", "model_id")}

    @app.post("/recording/walks/{walk}/evaluate", dependencies=[Depends(require_secret)])
    async def evaluate_walk(walk: str, judge: bool = True):
        """Score one walk, overwriting any previous scorecard. `judge=false`
        runs the free deterministic tier only."""
        walk_dir = _resolve_walk_dir(walk)
        return await asyncio.to_thread(_evaluate, walk_dir, walk, judge)

    @app.get("/recording/walks/{walk}/evaluation", dependencies=[Depends(require_secret)])
    async def get_evaluation(walk: str):
        """The stored scorecard, computing it lazily on first read.

        This is the fallback half of the trigger design: POST
        /recording/finish marks a walk complete, but a phone that lost its
        connection (or a tab closed mid-walk) never sends it, and a walk
        nobody can score is worse than one scored a few seconds late.
        """
        walk_dir = _resolve_walk_dir(walk)
        existing = _read_json_sidecar(walk_dir, EVAL_FILE_NAME)
        if existing and existing.get("schema") == walk_eval.SCHEMA_VERSION:
            return existing
        return await asyncio.to_thread(_evaluate, walk_dir, walk, True)

    # ---- replay: the same pixels, a different model ----

    def _replay(walk_dir: Path, walk_name: str, model_id, prompt_variant=None) -> dict:
        # Checked before any frame is read: a deployment with no vision
        # service configured should say so once, not once per frame.
        if not config["vision_url"]:
            raise RuntimeError(
                "No vision service configured. Set brain.vision_url in "
                "config/robot.yaml, or VISION_URL in this task's environment.")
        entries = _walk_entries(walk_dir)
        target = _walk_target(walk_dir, walk_name)

        def frame_bytes_for(entry):
            name = entry.get("file") or ""
            if not FRAME_FILENAME.match(name):
                return None
            path = walk_dir / name
            return path.read_bytes() if path.is_file() else None

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
        try:
            (walk_dir / _replay_file(result["model_id"], prompt_variant)).write_text(
                json.dumps(result, indent=1))
        except OSError as e:
            logger.warning("could not persist replay for %s: %s", walk_name, e)
        return result

    @app.post("/recording/walks/{walk}/replay", dependencies=[Depends(require_secret)])
    async def replay_walk_route(walk: str, req: ReplayRequest):
        """Re-ask this walk's frames under another model.

        The only controlled model comparison available: identical pixels,
        one variable. Comparing two live walks instead mixes model quality
        with where the operator pointed the phone.
        """
        walk_dir = _resolve_walk_dir(walk)
        try:
            return await asyncio.to_thread(_replay, walk_dir, walk, req.model_id,
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
        walk_dir = _resolve_walk_dir(walk)
        out = []
        for p in sorted(walk_dir.glob(REPLAY_PREFIX + "*.json")):
            try:
                out.append(json.loads(p.read_text()))
            except (json.JSONDecodeError, OSError):
                continue
        return {"walk": walk, "replays": out}

    @app.put("/recording/walks/{walk}/meta", dependencies=[Depends(require_secret)])
    async def set_meta(walk: str, req: MetaRequest):
        """Set walk-level metadata -- in practice the model, for the walks
        recorded before /navigate echoed it back per frame."""
        walk_dir = _resolve_walk_dir(walk)
        meta = _read_json_sidecar(walk_dir, META_FILE_NAME) or {}
        for field in ("model_id", "target_object", "note"):
            value = getattr(req, field)
            if value is not None:
                meta[field] = value
        (walk_dir / META_FILE_NAME).write_text(json.dumps(meta, indent=1))
        return {"walk": walk, "meta": meta}

    @app.delete("/recording/walks/{walk}", dependencies=[Depends(require_secret)])
    async def delete_walk(walk: str):
        walk_dir = _resolve_walk_dir(walk)
        shutil.rmtree(walk_dir)
        return {"deleted": walk}

    @app.delete("/recording/walks/{walk}/frames/{filename}", dependencies=[Depends(require_secret)])
    async def delete_frame(walk: str, filename: str):
        walk_dir = _resolve_walk_dir(walk)
        frame_path = _resolve_frame_path(walk_dir, filename)
        frame_path.unlink()
        jsonl_path = walk_dir / "walk.jsonl"
        if jsonl_path.exists():
            lines = [ln for ln in jsonl_path.read_text().splitlines() if ln.strip()]
            kept = [ln for ln in lines if json.loads(ln).get("file") != filename]
            jsonl_path.write_text("".join(ln + "\n" for ln in kept))
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
        base = Path(config["recording_dir"]).resolve()
        if not base.is_dir():
            return {"rows": []}

        acc = {}

        def add(model_id, prompt_variant, ev, source):
            if not ev or ev.get("score") is None:
                return
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

        for walk_dir in sorted(p for p in base.iterdir() if p.is_dir()):
            entries = _walk_entries(walk_dir)
            ev = _read_json_sidecar(walk_dir, EVAL_FILE_NAME)
            add(_walk_model_id(walk_dir, entries),
                (ev or {}).get("prompt_variant"), ev, "recorded")
            for path in sorted(walk_dir.glob(REPLAY_PREFIX + "*.json")):
                try:
                    r = json.loads(path.read_text())
                except (json.JSONDecodeError, OSError):
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
        base = Path(config["recording_dir"]).resolve()
        if not base.is_dir():
            return {"walks": 0, "frames": 0, "bytes": 0}
        walk_count = 0
        frame_count = 0
        total_bytes = 0
        for walk_dir in base.iterdir():
            if not walk_dir.is_dir():
                continue
            walk_count += 1
            for p in walk_dir.iterdir():
                if _is_frame_file(p):
                    frame_count += 1
                    total_bytes += p.stat().st_size
        return {"walks": walk_count, "frames": frame_count, "bytes": total_bytes}

    @app.get("/health")
    async def health():
        base = Path(config["recording_dir"]).resolve()
        return {"status": "ok", "recording_dir": str(base), "recording_dir_exists": base.is_dir()}

    return app


app = create_app()
