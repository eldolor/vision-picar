"""
server.py

Phase 9 (the sim-testable portion) -- the real Wi-Fi control API from the
build plan, exercised entirely against the mock backend for now. Since
RobotInterface is identical for sim and hardware, this file doesn't
change at all when Phase 11 swaps in the real backend -- only
config/robot.yaml's `mode` does.

Endpoints:
    GET  /         serves web-twin/index.html (the digital twin UI)
    POST /action   {"action": "FORWARD", "speed": 50, "duration": 0.5}
    POST /stop     always-available stop
    GET  /distance
    GET  /frame
    GET  /health   watchdog status + last command age

Every /action call goes through robot/safety.py, same as the sim agent
loop -- a human driving over Wi-Fi gets the exact same collision
protection an AI decision does. This is a deliberate extension of "AI
sits at the bottom of the safety hierarchy": nothing that can move the
robot bypasses the safety layer, regardless of who's driving.

Auth: /action, /stop, /distance, /frame require a matching x-app-secret
header when APP_SHARED_SECRET is set in the environment (see
require_secret() below) -- added when this server started being
deployed publicly (ECS Fargate, service/twin/), not just run on a home
LAN. /health stays open (the ALB health check can't send custom
headers) and / stays open (the page has to load before a user can enter
the secret in the UI).

Watchdog (build plan Phase 9): if the MacBook stops sending commands for
~1 second, the Pi stops the motors. `last_command_at` is updated on
every request; a background task polls it and calls robot.stop() once
it's stale. The go/no-go decision (watchdog_should_stop) is a pure
function tested directly in tests/test_server.py -- the async polling
loop itself needs a running event loop and is exercised by actually
running the server (see README), not in the automated unit suite.
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

    @app.get("/")
    def twin_ui():
        return FileResponse(_TWIN_INDEX_HTML)

    @app.post("/action", dependencies=[Depends(require_secret)])
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

    @app.post("/stop", dependencies=[Depends(require_secret)])
    def stop():
        state["last_command_at"] = time.monotonic()
        return {"executed": True, "result": robot.stop()}

    @app.get("/distance", dependencies=[Depends(require_secret)])
    def distance():
        return {"distance_cm": robot.get_distance()}

    @app.get("/frame", dependencies=[Depends(require_secret)])
    def frame():
        return robot.get_camera_frame()

    @app.get("/health")
    def health():
        age = time.monotonic() - state["last_command_at"]
        return {
            "status": "ok",
            "seconds_since_last_command": round(age, 2),
            "watchdog_timeout_s": watchdog_timeout,
        }

    return app


app = create_app()
