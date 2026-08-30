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
robot or the vision service.

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
import zipfile
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from control.brain_config import load_brain_config
from control import walk_eval

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
# Off by default so this service keeps its old behaviour (and needs no
# Bedrock permissions) unless the deployment opts in.
JUDGE_ENABLED = os.environ.get("WALK_JUDGE_ENABLED", "").lower() in ("1", "true", "yes")


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

# Walk-level facts the frames themselves don't carry -- written by
# brain_server's POST /recording/finish, and backfillable for the walks
# recorded before that route existed (see PUT /recording/walks/{walk}/meta).
META_FILE_NAME = "meta.json"


class TagRequest(BaseModel):
    label: Optional[str] = None  # None/omitted clears the label


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


def _walk_target(walk_dir: Path, walk_name: str) -> str:
    """The target object, for the judge's prompt. meta.json if recorded;
    otherwise recovered from the walk name, which newWalkName() builds as
    "<target>-<model tag>-<timestamp>"."""
    meta = _read_json_sidecar(walk_dir, META_FILE_NAME) or {}
    if meta.get("target_object"):
        return meta["target_object"]
    stem = re.sub(r"-\d{8}-\d{6}$", "", walk_name)
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
                "eval": _eval_summary(walk_dir),
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

        judge = None
        if use_judge and JUDGE_ENABLED and entries:
            def frame_bytes_for(entry):
                name = entry.get("file") or ""
                if not FRAME_FILENAME.match(name):
                    return None
                path = walk_dir / name
                return path.read_bytes() if path.is_file() else None

            try:
                judge = walk_eval.judge_walk(
                    _bedrock_client(), JUDGE_MODEL_ID, entries, frame_bytes_for,
                    _walk_target(walk_dir, walk_name), sample=JUDGE_SAMPLE_FRAMES,
                )
            except Exception as e:  # noqa: BLE001 -- fall back to metrics-only
                logger.warning("judge tier failed for %s: %s", walk_name, e)
                judge = {"judged": 0, "sensible_rate": None, "error": str(e)[:200]}

        scored = walk_eval.score_walk(metrics, judge)
        result = {
            "schema": walk_eval.SCHEMA_VERSION,
            "walk": walk_name,
            "model_id": _walk_model_id(walk_dir, entries),
            "target_object": _walk_target(walk_dir, walk_name),
            **scored,
            "metrics": metrics,
            "judge": judge,
        }
        try:
            (walk_dir / EVAL_FILE_NAME).write_text(json.dumps(result, indent=1))
        except OSError as e:
            logger.warning("could not persist eval for %s: %s", walk_name, e)
        return result

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
