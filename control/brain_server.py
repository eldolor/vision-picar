"""
brain_server.py

Phase B2 -- the brain as a service.

Runs on :8001 next to robot/server.py on :8000 (both on the Pi, in the
target topology) and owns exactly one thing: the autonomy loop. It drives
a MissionRunner as an asyncio background task, so a mission survives the
HTTP request that started it -- which is the point. Today the browser IS
the loop, so backgrounding the phone's tab halts autonomy.

    POST /mission/start   {"target_object": "red backpack", "max_steps": 120,
                           "policy": "frontier" | "vision", "fault": "none"}
    POST /mission/stop    always available, always stops the robot too
    GET  /mission/status  what the loop is doing right now
    POST /recording/frame save one frame of a Robot-view walk, for replay
    GET  /health

Viewing and deleting recorded walks after the fact (GET/DELETE
/recording/walks/...) is deliberately NOT here -- see
control/admin_server.py's docstring for why it is a separate process rather
than more routes on this one.

**Two processes, not one** (PLAN-brain-relocation.md, "Why not one
process"): a synchronous block in the agent loop would block the event
loop robot/server.py's watchdog polls on, and merging them would break
the brain/robot separation the project has kept since Phase 0. The cost
is a localhost round trip per call, against a loop that spends seconds
waiting on vision.

This module pulls in nothing from sim/ -- no simulator, no backend, no
robot server -- and touches robot/ only for the interface and the
SafetyViolation type that is part of it. The robot is reachable only
through control/remote_robot.py. That is what makes "brain on the Pi" and
"brain on the MacBook" a config difference rather than a code difference,
and tests/test_brain_server.py asserts the import surface directly.

`policy: "vision"` hands each decision to the model via
brain/navigate.py, and needs both `brain.vision_url` and a backend whose
frames carry pixels (`sim/replay_robot.py` today; MockRobot cannot until
phase S2). `fault` runs one of control/drills.py's failsafe drills instead
of a normal mission -- the only way to demonstrate B3.2 and B3.3 from the twin,
since neither can be provoked by pressing anything. See that module.

Failsafe B3.3 lives here: the loop's own dead-man. If a tick doesn't
return within `tick_timeout_s`, the robot is stopped and the mission ends
failed. This is distinct from robot/server.py's watchdog (B3.1), which
cannot see a loop that is alive but stuck, and from the runner's vision
budget (B3.2), which only sees calls that return or time out.
"""

import asyncio
import base64
import binascii
import json
import logging
import os
import re
from pathlib import Path
from typing import Callable, Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from brain.navigate import vision_fn_for
from control import drills
from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from robot.interface import RobotInterface

logger = logging.getLogger("brain_server")


def require_secret(x_app_secret: str = Header(default="")):
    """Same gate, same env var, same semantics as robot/server.py's --
    this process is reachable on the LAN too, and anyone who can start a
    mission can drive the robot. Inert when APP_SHARED_SECRET is unset,
    which is how local dev and the test suite run. /health stays open."""
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


# A Robot-view walk, arriving one frame at a time from the phone. The
# frames land in a directory sim/replay_robot.py can play back, and each
# one's live /navigate answer is appended to walk.jsonl beside them -- so a
# replay can be compared against what the service said at the time, on the
# same pixels.
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


class MissionStartRequest(BaseModel):
    target_object: Optional[str] = None
    target_room: Optional[str] = None
    mission: Optional[str] = None
    max_steps: Optional[int] = None
    policy: str = "frontier"
    # One of control/drills.FAULTS. Breaks exactly one thing so a failsafe
    # can be watched firing; "none" is an ordinary mission.
    fault: str = "none"


def create_app(
    config_path: Optional[str] = None,
    robot_factory: Optional[Callable[[], RobotInterface]] = None,
    runner_factory: Optional[Callable[..., MissionRunner]] = None,
) -> FastAPI:
    """Builds a brain app. Tests call this directly to inject a robot
    (an in-process RemoteRobot, or a recording stub) and a runner; the
    module-level `app` below is what `uvicorn control.brain_server:app`
    serves."""
    config = load_brain_config(config_path)
    secret = os.environ.get("APP_SHARED_SECRET")
    # Distinct from `secret` above, which gates callers OF this brain.
    # These gate the brain's OWN outbound calls to the robot and vision
    # services. On a Pi or a MacBook there is conventionally one shared
    # secret for the whole system, so both default to `secret` and nothing
    # changes there. They diverge once the robot and vision service are each
    # independently deployed with their own generated secret (e.g. the
    # interim ECS Fargate brain -- see PLAN-brain-relocation.md).
    robot_secret = os.environ.get("ROBOT_SHARED_SECRET", secret)
    vision_secret = os.environ.get("VISION_SHARED_SECRET", secret)

    def default_robot_factory() -> RobotInterface:
        return RemoteRobot(
            config["robot_url"], secret=robot_secret, timeout=config["request_timeout_s"]
        )

    def default_runner_factory(robot: RobotInterface, req: MissionStartRequest) -> MissionRunner:
        vision_fn = None
        if req.policy == "vision":
            # The policy exists (brain/vision_agent.py); what it needs is an
            # endpoint to ask and a target to look for. Both are checked here
            # rather than failing on the first tick, so a misconfigured brain
            # says so at start time instead of after a paid call.
            if not config["vision_url"]:
                raise ValueError(
                    "The vision policy needs a vision service. Set brain.vision_url "
                    "in config/robot.yaml (or VISION_URL in the environment)."
                )
            if not req.target_object:
                raise ValueError(
                    "The vision policy searches for an object -- /navigate takes a "
                    "target_object. A target_room-only mission needs policy='frontier'."
                )
            vision_fn = vision_fn_for(
                req.target_object,
                vision_url=config["vision_url"],
                secret=vision_secret,
                timeout_s=config["vision_timeout_s"],
            )

        kwargs = dict(
            target_object=req.target_object,
            target_room=req.target_room,
            mission=req.mission,
            max_steps=req.max_steps or config["max_steps"],
            min_distance_cm=config["min_distance_cm"],
            policy=req.policy,
            vision_fn=vision_fn,
            vision_timeout_s=config["vision_timeout_s"],
            max_vision_failures=config["max_vision_failures"],
        )
        runner_class, kwargs, tick_timeout_s = drills.apply(req.fault, kwargs, config)
        # The dead-man deadline is per mission, not per process, so a drill
        # can shorten its own without touching anything else.
        state["tick_timeout_s"] = tick_timeout_s
        return runner_class(robot, **kwargs)

    make_robot = robot_factory or default_robot_factory
    make_runner = runner_factory or default_runner_factory

    # One robot client for the process. It is built up front (construction
    # does no I/O) so that POST /mission/stop can stop the car even when no
    # mission has ever run.
    state: dict = {
        "robot": None, "runner": None, "task": None,
        "fault": drills.NONE, "tick_timeout_s": config["tick_timeout_s"],
    }

    def robot() -> RobotInterface:
        if state["robot"] is None:
            state["robot"] = make_robot()
        return state["robot"]

    app = FastAPI(title="vision-picar brain server")

    # The twin will call this from a browser once B4 lands (Stage 4).
    # Same reasoning as robot/server.py: permissive origins, with
    # require_secret() as the actual access control.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )

    async def mission_loop(runner: MissionRunner):
        """Drives tick() off the event loop, with B3.3's dead-man on it."""
        try:
            while runner.is_running():
                try:
                    await asyncio.wait_for(
                        asyncio.to_thread(runner.tick), timeout=state["tick_timeout_s"]
                    )
                except asyncio.TimeoutError:
                    # The tick thread is abandoned, still running, possibly
                    # forever. abort() stops the car on this thread rather
                    # than waiting for a loop that may never come back.
                    reason = f"brain loop hung: tick exceeded {state['tick_timeout_s']}s"
                    logger.error(reason)
                    await asyncio.to_thread(runner.abort, reason)
                    break
                if config["tick_interval_s"]:
                    await asyncio.sleep(config["tick_interval_s"])
        except asyncio.CancelledError:
            # Synchronous on purpose: awaiting inside a cancellation
            # handler is how a stop gets silently skipped. One localhost
            # POST is a fine thing to block the loop for.
            runner.stop("mission loop cancelled")
            raise

    @app.post("/mission/start", dependencies=[Depends(require_secret)])
    async def start_mission(req: MissionStartRequest):
        runner = state["runner"]
        if runner is not None and runner.is_running():
            # Reject rather than race -- two loops driving one robot is
            # exactly the failure this service exists to prevent.
            raise HTTPException(status_code=409, detail="A mission is already running.")

        try:
            runner = make_runner(robot(), req)
        except drills.DrillNotAllowed as e:
            raise HTTPException(status_code=403, detail=str(e))
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))

        runner.start()
        state["fault"] = req.fault or drills.NONE
        state["runner"] = runner
        state["task"] = asyncio.create_task(mission_loop(runner))
        return {"started": True, "status": runner.status()}

    @app.post("/mission/stop", dependencies=[Depends(require_secret)])
    async def stop_mission():
        """Always available. Stops the loop AND the car -- stopping the
        thinking is not stopping the robot."""
        runner = state["runner"]
        task = state["task"]
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        state["task"] = None

        if runner is not None:
            await asyncio.to_thread(runner.stop)
            return {"stopped": True, "status": runner.status()}

        # No mission has ever run: stop the car anyway.
        await asyncio.to_thread(robot().stop)
        return {"stopped": True, "status": _idle_status()}

    @app.get("/mission/status", dependencies=[Depends(require_secret)])
    async def mission_status():
        runner = state["runner"]
        if runner is None:
            return _idle_status()
        # `fault` belongs to the request, not the mission -- the runner has
        # no idea it is a drill, which is the point of running the same
        # class either way.
        return {**runner.status(), "fault": state["fault"]}

    @app.post("/recording/frame", dependencies=[Depends(require_secret)])
    async def record_frame(req: RecordFrameRequest):
        """Save one walk frame. Off by default on any brain that should not
        be accepting writes from whoever can reach it."""
        if not config["allow_recording"]:
            raise HTTPException(status_code=403, detail="Recording is disabled on this brain.")
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

        base = Path(config["recording_dir"]).resolve()
        walk_dir = (base / req.walk).resolve()
        # The name is already sanitised; this is the belt to that braces.
        if base != walk_dir.parent:
            raise HTTPException(status_code=400, detail="Bad walk name.")
        walk_dir.mkdir(parents=True, exist_ok=True)

        # "starts with frame-", not "isn't walk.jsonl": control/admin_server.py
        # can leave a tags.json sidecar in this same directory, which a
        # suffix-exclusion check would miscount as a frame.
        existing = sorted(p for p in walk_dir.iterdir() if p.name.startswith("frame-"))
        if len(existing) >= MAX_FRAMES_PER_WALK:
            raise HTTPException(
                status_code=409,
                detail=f"This walk already has {MAX_FRAMES_PER_WALK} frames.",
            )

        suffix = FRAME_SUFFIX.get(req.media_type, ".jpg")
        path = walk_dir / f"frame-{req.seq:04d}{suffix}"
        path.write_bytes(image)
        with (walk_dir / "walk.jsonl").open("a") as f:
            f.write(json.dumps({
                "seq": req.seq, "file": path.name, "media_type": req.media_type,
                "navigate": req.navigate,
            }) + "\n")

        return {"saved": path.name, "walk": req.walk, "frames": len(existing) + 1,
                "dir": str(walk_dir)}

    @app.get("/health")
    async def health():
        runner = state["runner"]
        return {
            "status": "ok",
            "robot_url": config["robot_url"],
            "mission_running": bool(runner is not None and runner.is_running()),
            "drills_allowed": bool(config["allow_drills"]),
            "recording_allowed": bool(config["allow_recording"]),
            "faults": list(drills.FAULTS),
        }

    return app


def _idle_status() -> dict:
    return {"running": False, "outcome": "idle", "step": 0, "log_tail": [], "fault": drills.NONE}


app = create_app()
