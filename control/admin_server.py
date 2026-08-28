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

This module is intentionally thin: read-only inventory, JPEG bytes, and
delete, over the same recording_dir control/brain_server.py's
POST /recording/frame already writes to (that route stays in brain_server.py
-- it is the live capture path, tightly coupled to an active Robot-view
session, unlike browsing history after the fact). No mission logic, no
RemoteRobot, no vision policy -- this process never talks to the robot or
the vision service at all.

    GET    /admin                                  the viewer/deleter page
    GET    /recording/walks                        list saved walks (incl. label)
    GET    /recording/walks/{walk}                  one walk's frames + navigate log
    GET    /recording/walks/{walk}/frames/{file}    one frame's image bytes
    GET    /recording/walks/{walk}/download         the whole walk as a .zip
    PUT    /recording/walks/{walk}/tag              set/clear this walk's label
    DELETE /recording/walks/{walk}                  delete a whole walk
    DELETE /recording/walks/{walk}/frames/{file}    delete one frame
    GET    /stats                                   walk/frame/byte totals
    GET    /health
"""

import io
import json
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

_ADMIN_HTML = Path(__file__).resolve().parent / "admin.html"

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


class TagRequest(BaseModel):
    label: Optional[str] = None  # None/omitted clears the label


def _read_label(walk_dir: Path) -> Optional[str]:
    tag_path = walk_dir / TAG_FILE_NAME
    if not tag_path.exists():
        return None
    try:
        return json.loads(tag_path.read_text()).get("label")
    except (json.JSONDecodeError, OSError):
        return None


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
        jsonl_path = walk_dir / "walk.jsonl"
        entries = []
        if jsonl_path.exists():
            for line in jsonl_path.read_text().splitlines():
                if line.strip():
                    entries.append(json.loads(line))
        return {"walk": walk, "frames": frames, "entries": entries, "label": _read_label(walk_dir)}

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
