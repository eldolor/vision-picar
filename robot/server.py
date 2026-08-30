"""
server.py

Phase 9 (the sim-testable portion) -- the real Wi-Fi control API from the
build plan, exercised entirely against the mock backend for now. Since
RobotInterface is identical for sim and hardware, this file doesn't
change at all when Phase 11 swaps in the real backend -- only
config/robot.yaml's `mode` does.

Endpoints:
    GET  /            serves web-twin/index.html (the digital twin UI)
    GET  /app.js                  the twin's client script (was inline)
    GET  /manifest.json           PWA manifest, for "Add to Home Screen"
    GET  /icons/icon-192.png      referenced by manifest.json and
    GET  /icons/icon-512.png      index.html's <link rel="icon">
    GET  /icons/apple-touch-icon.png
                                   iOS home-screen icon -- manifest.json's
                                   icons are ignored by iOS Safari's "Add
                                   to Home Screen", which only reads this
                                   apple-touch-icon <link> tag
    POST /action   {"action": "FORWARD", "speed": 50, "duration": 0.5}
    POST /stop     always-available stop
    GET  /distance
    GET  /frame
    POST /teleop/frame   {"image_base64": "...", "media_type": "image/jpeg"}
                   only meaningful when config/robot.yaml's mode is
                   "teleop" (PLAN-teleop-robot.md) -- pushes a live phone
                   frame into sim/teleop_robot.py's TeleopRobot, which
                   GET /frame then hands back on the next pull. 400 on any
                   other mode, since a frame with nothing to read it would
                   silently vanish otherwise.
    GET  /health   watchdog status + last command age

Every /action call goes through robot/safety.py, same as the sim agent
loop -- a human driving over Wi-Fi gets the exact same collision
protection an AI decision does. This is a deliberate extension of "AI
sits at the bottom of the safety hierarchy": nothing that can move the
robot bypasses the safety layer, regardless of who's driving.

Auth: /action, /stop, /distance, /frame, /teleop/frame require a matching
x-app-secret header when APP_SHARED_SECRET is set in the environment (see
require_secret() below) -- added when this server started being
deployed publicly (ECS Fargate, service/twin/), not just run on a home
LAN. /health stays open (the ALB health check can't send custom
headers) and / stays open (the page has to load before a user can enter
the secret in the UI).

Watchdog (build plan Phase 9; failsafe B3.1 in
PLAN-brain-relocation.md): if commands stop arriving for ~1 second, the
motors stop. `last_command_at` is updated by /action and /stop only -- a
sensing read is not a command, and counting one would let a passive
observer (the twin polling /frame while it watches a mission) hold the
watchdog off indefinitely. A background task polls it and calls
robot.stop() once it's stale.

ROUTE_PREFIX (env var, unset/empty by default): prepended to every route
below. Exists so a second instance of this exact file -- same image, same
code, different ECS task -- can share the twin's load balancer instead of
needing one of its own (cloudformation/teleop-robot.yaml, PLAN-teleop-robot.md).
The twin's own deployment leaves this unset, so its URLs are unaffected.

ROBOT_CONFIG_PATH (env var, unset by default): overrides which
config/robot.yaml the module-level `app` below loads, same pattern as
ROUTE_PREFIX/ROBOT_MODE. `create_app(config_path)` already took an
explicit path for tests that construct an app object directly; this is
what lets a *live* `uvicorn robot.server:app` subprocess do the same --
tests/test_watchdog_integration.py (Phase S4) is the one thing that needs
it, to point a live server at a temp config with a short
watchdog_timeout_s and sim.realtime: true without editing the real one.

Its original description -- "detects a dead MacBook" -- narrows once the
brain runs on the Pi and talks to this server over localhost, since a
dead link is no longer the likely cause. The job it keeps is the one that
actually matters on hardware: **if a movement call energises the motors
and then crashes before px.stop(), nothing else catches it.** It is also
the only guard that sees a command dispatched microseconds before a
mission was stopped. The brain's own failsafes (B3.2's vision-failure
budget, B3.3's hung-tick dead-man, both in control/) cover different
failures and cannot substitute for this one.

The go/no-go decision (watchdog_should_stop) is a pure function tested
directly in tests/test_server.py -- the async polling loop itself needs a
running event loop and is exercised by actually running the server (see
README), not in the automated unit suite.
"""

import os
import time
import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel

from robot.factory import get_robot, load_config
from robot.safety import SafetyController, SafetyViolation

logger = logging.getLogger("server")

_TWIN_INDEX_HTML = Path(__file__).resolve().parent.parent / "web-twin" / "index.html"
_TWIN_APP_JS = Path(__file__).resolve().parent.parent / "web-twin" / "app.js"
_TWIN_MANIFEST_JSON = Path(__file__).resolve().parent.parent / "web-twin" / "manifest.json"
_TWIN_ICONS_DIR = Path(__file__).resolve().parent.parent / "web-twin" / "icons"


def require_secret(x_app_secret: str = Header(default="")):
    """Gate for movement/sensing routes once this server is reachable from
    the public internet (the ECS Fargate deployment), not just a home LAN.
    /health is deliberately excluded -- the ALB health check can't send
    custom headers. Inert (no-op) when APP_SHARED_SECRET is unset, which is
    how local dev and the test suite run."""
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


class ActionRequest(BaseModel):
    action: str
    speed: int = 50
    duration: float = 0.5
    angle: int = 90


class TeleopFrameRequest(BaseModel):
    image_base64: str
    media_type: str = "image/jpeg"


def watchdog_should_stop(last_command_at: float, now: float, timeout_s: float) -> bool:
    """Pure decision logic -- see module docstring for why this is
    tested separately from the async polling loop that calls it."""
    return (now - last_command_at) > timeout_s


def create_app(config_path: Optional[str] = None) -> FastAPI:
    """Builds a fresh app + robot + safety controller. Tests call this
    directly to get an isolated instance per test; the module-level
    `app` below is what `uvicorn robot.server:app` actually serves."""
    config = load_config(config_path) if config_path else load_config()
    watchdog_timeout = config.get("safety", {}).get("watchdog_timeout_s", 1.0)
    min_distance = config.get("safety", {}).get("min_distance_cm", 20.0)
    # Same resolution get_robot() uses internally -- reported in /health so
    # a client can tell "wrong deployment" (e.g. Robot view's "drive via
    # brain" pointed at a mode: sim server) apart from "not reachable at
    # all", which a bare connectivity check can't distinguish.
    mode = os.environ.get("ROBOT_MODE") or config.get("mode", "sim")

    robot = get_robot(config_path) if config_path else get_robot()
    safety = SafetyController(robot, min_distance_cm=min_distance)
    state = {"last_command_at": time.monotonic()}

    async def watchdog_loop():
        while True:
            await asyncio.sleep(0.1)
            if watchdog_should_stop(state["last_command_at"], time.monotonic(), watchdog_timeout):
                robot.stop()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        task = asyncio.create_task(watchdog_loop())
        try:
            yield
        finally:
            task.cancel()

    app = FastAPI(title="vision-picar robot server", lifespan=lifespan)

    # See module docstring. "" (the default) reproduces every route exactly
    # as before this existed -- prefix + "/frame" == "/frame".
    prefix = os.environ.get("ROUTE_PREFIX", "").rstrip("/")

    # This server is designed to be reached from a browser (the web twin --
    # locally on the same LAN for real-hardware use, or the public
    # service/twin/ ECS deployment). Same reasoning as
    # service/vision_analyze/app.py's CORS handling: permissive origins by
    # default, with require_secret() as the actual access control once this
    # is reachable from the public internet, not CORS.
    allowed_origins = config.get("server", {}).get("allowed_origins", ["*"])
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )

    @app.get(prefix + "/")
    def twin_ui():
        return FileResponse(_TWIN_INDEX_HTML)

    @app.get(prefix + "/app.js")
    def twin_app_js():
        """The twin's client, extracted out of index.html.

        no-cache (not no-store): the browser still caches the file but must
        revalidate, and FileResponse already sends an ETag, so an unchanged
        script costs one 304 and a changed one is picked up immediately. This
        is what makes a redeploy safe without hashed filenames -- index.html
        and app.js can never end up a version apart, which for a page whose
        HTML and JS are deployed as a single image is the only failure mode
        worth designing against.
        """
        return FileResponse(
            _TWIN_APP_JS,
            media_type="application/javascript",
            headers={"Cache-Control": "no-cache"},
        )

    # Explicit routes rather than a generic /icons/{filename} + StaticFiles
    # mount -- there are exactly three icon files, and exact routes mean no
    # path-traversal surface to reason about at all, matching this file's
    # existing minimal-surface style elsewhere.
    @app.get(prefix + "/manifest.json")
    def twin_manifest():
        return FileResponse(_TWIN_MANIFEST_JSON, media_type="application/manifest+json")

    @app.get(prefix + "/icons/icon-192.png")
    def twin_icon_192():
        return FileResponse(_TWIN_ICONS_DIR / "icon-192.png")

    @app.get(prefix + "/icons/icon-512.png")
    def twin_icon_512():
        return FileResponse(_TWIN_ICONS_DIR / "icon-512.png")

    @app.get(prefix + "/icons/apple-touch-icon.png")
    def twin_icon_apple():
        return FileResponse(_TWIN_ICONS_DIR / "apple-touch-icon.png")

    @app.post(prefix + "/action", dependencies=[Depends(require_secret)])
    def do_action(req: ActionRequest):
        state["last_command_at"] = time.monotonic()
        try:
            result = safety.check_and_execute(
                req.action, speed=req.speed, duration=req.duration, angle=req.angle
            )
            return {"executed": True, "result": result}
        except SafetyViolation as e:
            return {"executed": False, "detail": str(e)}
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))

    @app.post(prefix + "/stop", dependencies=[Depends(require_secret)])
    def stop():
        state["last_command_at"] = time.monotonic()
        return {"executed": True, "result": robot.stop()}

    @app.get(prefix + "/distance", dependencies=[Depends(require_secret)])
    def distance():
        return {"distance_cm": robot.get_distance()}

    @app.get(prefix + "/frame", dependencies=[Depends(require_secret)])
    def frame():
        try:
            return robot.get_camera_frame()
        except Exception as e:
            # Backend-agnostic on purpose -- no import of a specific
            # backend's exception type here (e.g. sim/teleop_robot.py's
            # TeleopStall), so this file stays what its docstring promises:
            # unchanged regardless of which backend config/robot.yaml's
            # mode selects. 503 rather than the generic 500 an uncaught
            # exception would otherwise produce: the camera is what's
            # unavailable, not this route or the service around it. Found
            # via a real deployment -- a stalled TeleopRobot's real message
            # ("no frame has ever been pushed...") was getting swallowed
            # into an opaque "Internal Server Error" by the time it reached
            # a mission's log through RemoteRobot.
            raise HTTPException(status_code=503, detail=str(e))

    @app.post(prefix + "/teleop/frame", dependencies=[Depends(require_secret)])
    def teleop_frame(req: TeleopFrameRequest):
        # Duck-typed rather than an isinstance check against TeleopRobot,
        # so this file stays what its own docstring promises: unchanged
        # regardless of which backend config/robot.yaml's mode selects.
        push = getattr(robot, "push_frame", None)
        if push is None:
            raise HTTPException(
                status_code=400,
                detail="This robot is not in teleop mode -- set mode: teleop "
                "in config/robot.yaml (see PLAN-teleop-robot.md). A pushed "
                "frame would otherwise have nothing to read it.",
            )
        return {"received": True, **push(req.image_base64, req.media_type)}

    @app.get(prefix + "/health")
    def health():
        age = time.monotonic() - state["last_command_at"]
        return {
            "status": "ok",
            "seconds_since_last_command": round(age, 2),
            "watchdog_timeout_s": watchdog_timeout,
            # The single source of truth for the safety threshold this
            # server actually enforces -- see web-twin/index.html's
            # renderWatchdog(), which reads this into state.minDistanceCm
            # so the JS local-brain's own clearance checks and the map's
            # safety-collar drawing stay in sync with whatever
            # config/robot.yaml's safety.min_distance_cm actually is,
            # instead of a second hardcoded constant that can drift from
            # it (PLAN-sim-hardening.md definition of done, item 10).
            "min_distance_cm": min_distance,
            "mode": mode,
        }

    return app


app = create_app(os.environ.get("ROBOT_CONFIG_PATH"))
