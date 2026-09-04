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

ROUTE_PREFIX (env var, unset/empty by default): prepended to every route
below, same mechanism and reason as robot/server.py's -- lets a second
instance of this file share the twin's load balancer with its own
non-colliding paths instead of needing a load balancer of its own
(cloudformation/teleop-brain.yaml, PLAN-teleop-robot.md).

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
import time
from typing import Callable, Optional

import httpx
from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, ConfigDict

from brain.navigate import vision_fn_for
from control import drills
from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot
from control.walk_store import WalkStoreError, walk_store_from_config
from robot.identity import log_identity
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


class MissionStartRequest(BaseModel):
    target_object: Optional[str] = None
    target_room: Optional[str] = None
    mission: Optional[str] = None
    max_steps: Optional[int] = None
    policy: str = "frontier"
    # Which model answers /navigate for this mission (policy: "vision" only).
    # Omitted falls back to brain.navigate_model_id, then to the vision
    # service's own default. Validated against the service's published
    # allow-list at start -- see _validate_navigate_choices().
    model_id: Optional[str] = None
    # Which wording of the /navigate prompt this mission runs. Same three-step
    # fallback and the same start-time validation as model_id, for the same
    # reason: the 3x3 matrix in CLAUDE.md moved one model's FORWARD rate from
    # 0.000 to 1.000 by wording alone, so a mission that cannot name its
    # variant is a mission whose result cannot be attributed.
    #
    # This field is also why extra="forbid" is on this model. The twin has
    # been sending prompt_variant on "Drive via brain" since the picker
    # shipped, and pydantic's default is to DROP an unknown field silently:
    # the mission ran the service default while the UI said otherwise, which
    # is precisely the NavigateModelId trap the Stage 0 notes record. An
    # unknown field is now a 422 naming it.
    prompt_variant: Optional[str] = None
    # One of control/drills.FAULTS. Breaks exactly one thing so a failsafe
    # can be watched firing; "none" is an ordinary mission.
    fault: str = "none"

    model_config = ConfigDict(extra="forbid")


def _validate_navigate_choices(model_id: Optional[str], prompt_variant: Optional[str],
                               vision_url: str, secret: Optional[str],
                               timeout_s: float) -> None:
    """Reject an unusable model or prompt variant at mission start rather
    than mid-tick.

    Without this, a bad value becomes a 400 from the vision service *inside*
    a tick, which counts against failsafe B3.2's vision-failure budget: the
    mission limps through three failures and then dies reporting "vision
    failed 3 times", which says nothing about the actual cause and costs
    three round trips to say it.

    Neither allow-list is duplicated here. Both live in the vision service
    (vision_core.NAVIGATE_MODEL_CHOICES and NAVIGATE_PROMPT_VARIANTS,
    published together at GET /navigate/models), and this asks that service
    what it accepts -- one HTTP call per mission start, covering both axes,
    against lists that keep changing as Bedrock's catalogue and this
    project's wording experiments do. A copy in control/ would be wrong the
    first time either grew and nobody thought to update two places.

    A service that cannot answer is NOT treated as a rejection: an unknown
    value is a caller error, but an unreachable models endpoint is an outage,
    and refusing to start a mission because a *validation* call failed would
    turn a soft problem into a hard one. The mission proceeds and the normal
    B3.2 budget covers whatever happens next. The same rule covers a service
    too old to publish `prompts` at all: an empty list is "could not ask",
    not "nothing is allowed".
    """
    if not model_id and not prompt_variant:
        return
    url = vision_url.rstrip("/") + "/navigate/models"
    headers = {"x-app-secret": secret} if secret else {}
    try:
        with httpx.Client(timeout=timeout_s) as client:
            resp = client.get(url, headers=headers)
        resp.raise_for_status()
        body = resp.json()
        allowed = {m["id"] for m in body.get("models", []) if "id" in m}
        prompts = {p for p in body.get("prompts", []) if isinstance(p, str)}
    except (httpx.HTTPError, ValueError, KeyError, TypeError) as e:
        logger.warning("Could not verify model_id %r / prompt_variant %r against %s: %s",
                       model_id, prompt_variant, url, e)
        return
    if model_id and allowed and model_id not in allowed:
        raise ValueError(
            f"model_id {model_id!r} is not offered by the vision service. "
            f"Available: {', '.join(sorted(allowed))}."
        )
    if prompt_variant and prompts and prompt_variant not in prompts:
        raise ValueError(
            f"prompt_variant {prompt_variant!r} is not offered by the vision "
            f"service. Available: {', '.join(sorted(prompts))}."
        )


def create_app(
    config_path: Optional[str] = None,
    robot_factory: Optional[Callable[[], RobotInterface]] = None,
    runner_factory: Optional[Callable[..., MissionRunner]] = None,
    store=None,
) -> FastAPI:
    """Builds a brain app. Tests call this directly to inject a robot
    (an in-process RemoteRobot, or a recording stub) and a runner; the
    module-level `app` below is what `uvicorn control.brain_server:app`
    serves."""
    # Phase M5, same reason as robot/server.py's: first line out.
    ident = log_identity("vision-picar brain server", config_path)
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

    # Where recorded walks go. Built even when allow_recording is false, so
    # that flipping the flag needs no restart-time reasoning about which
    # half of the config was read; a store is inert until something writes.
    store = store if store is not None else walk_store_from_config(config)

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
            model_id = req.model_id or config["navigate_model_id"] or None
            prompt_variant = (
                req.prompt_variant or config["navigate_prompt_variant"] or None
            )
            _validate_navigate_choices(
                model_id, prompt_variant, config["vision_url"], vision_secret,
                config["request_timeout_s"],
            )
            vision_fn = vision_fn_for(
                req.target_object,
                vision_url=config["vision_url"],
                secret=vision_secret,
                timeout_s=config["vision_timeout_s"],
                model_id=model_id,
                prompt_variant=prompt_variant,
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

    # See module docstring. "" (the default) reproduces every route exactly
    # as before this existed.
    prefix = os.environ.get("ROUTE_PREFIX", "").rstrip("/")

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

    @app.post(prefix + "/mission/start", dependencies=[Depends(require_secret)])
    async def start_mission(req: MissionStartRequest):
        runner = state["runner"]
        if runner is not None and runner.is_running():
            # Reject rather than race -- two loops driving one robot is
            # exactly the failure this service exists to prevent.
            raise HTTPException(status_code=409, detail="A mission is already running.")

        try:
            # Off the event loop: building a vision runner now makes a
            # blocking GET to the vision service to validate model_id, and a
            # slow or dead service would otherwise stall the loop for the
            # whole request timeout -- hanging /mission/status and /health
            # for the twin that is polling them. Construction is pure
            # otherwise, so a thread is safe.
            runner = await asyncio.to_thread(make_runner, robot(), req)
        except drills.DrillNotAllowed as e:
            raise HTTPException(status_code=403, detail=str(e))
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))

        runner.start()
        state["fault"] = req.fault or drills.NONE
        state["runner"] = runner
        state["task"] = asyncio.create_task(mission_loop(runner))
        return {"started": True, "status": runner.status()}

    @app.post(prefix + "/mission/stop", dependencies=[Depends(require_secret)])
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

    @app.get(prefix + "/mission/status", dependencies=[Depends(require_secret)])
    async def mission_status():
        runner = state["runner"]
        if runner is None:
            return _idle_status()
        # `fault` belongs to the request, not the mission -- the runner has
        # no idea it is a drill, which is the point of running the same
        # class either way.
        return {**runner.status(), "fault": state["fault"]}

    async def _proxy_recording(route: str, req: BaseModel) -> dict:
        # The routes this forwards touch no robot/runner state at all --
        # they're pure storage -- which is exactly why they're safe to
        # forward to a peer brain when *this* brain has no storage of its
        # own (e.g. teleop-brain, no EFS mount). Mission-control routes could
        # never do this: a brain's RemoteRobot is bound to one robot_url for
        # its whole process lifetime, so proxying /mission/* would tick the
        # WRONG robot. See PLAN-teleop-robot.md's "Recording proxy" section.
        # `route` is therefore restricted to /recording/* by construction --
        # both call sites pass a literal, never anything caller-supplied.
        url = config["recording_proxy_url"].rstrip("/") + route
        headers = {}
        if config["recording_proxy_secret"]:
            headers["x-app-secret"] = config["recording_proxy_secret"]

        def _forward() -> httpx.Response:
            # Sync client run in a thread, not httpx.AsyncClient, matching
            # RemoteRobot/vision_fn_for's existing style elsewhere in
            # control/ and brain/ -- and specifically off the event loop
            # (asyncio.to_thread) since this fires once per recorded frame
            # (up to ~2/s during an active walk), not a one-off call like
            # the teleop-mode health check above.
            with httpx.Client(timeout=config["recording_proxy_timeout_s"]) as client:
                return client.post(url, json=req.model_dump(), headers=headers)

        try:
            resp = await asyncio.to_thread(_forward)
        except httpx.HTTPError as e:
            raise HTTPException(status_code=502, detail=f"Recording proxy unreachable: {e}")
        if resp.status_code >= 400:
            # Surface the peer's own error (its own 403, a bad-walk-name
            # 400, ...) rather than inventing a new one -- the caller should
            # see exactly what the brain that actually owns storage said.
            raise HTTPException(status_code=resp.status_code, detail=resp.text)
        return resp.json()

    @app.post(prefix + "/recording/frame", dependencies=[Depends(require_secret)])
    async def record_frame(req: RecordFrameRequest):
        """Save one walk frame. Off by default on any brain that should not
        be accepting writes from whoever can reach it."""
        if not config["allow_recording"]:
            if config["recording_proxy_url"]:
                return await _proxy_recording("/recording/frame", req)
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
        }) + "\n")

        return {"saved": name, "walk": req.walk, "frames": len(existing) + 1,
                "dir": store.location(req.walk)}

    @app.post(prefix + "/recording/finish", dependencies=[Depends(require_secret)])
    async def finish_walk(req: FinishWalkRequest):
        """Mark a recorded walk complete and record what produced it.

        Only ever writes meta.json beside the frames -- it deliberately does
        not score anything. Scoring lives in control/admin_server.py, which
        is the process that owns reviewing recordings and the one that has
        Bedrock permissions; the brain's job ends when the frames are safely
        on the volume.
        """
        if not config["allow_recording"]:
            if config["recording_proxy_url"]:
                return await _proxy_recording("/recording/finish", req)
            raise HTTPException(status_code=403, detail="Recording is disabled on this brain.")
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
        store.write_json(req.walk, "meta.json", meta)
        return {"walk": req.walk, "meta": meta}

    @app.get(prefix + "/health")
    async def health():
        runner = state["runner"]
        running = bool(runner is not None and runner.is_running())
        status = runner.status() if runner is not None else None
        return {
            "status": "ok",
            "identity": ident,
            "robot_url": config["robot_url"],
            "mission_running": running,
            # Phase M5. The verdict input for "the loop is alive but
            # stuck": if a mission is running and no tick has COMPLETED
            # within the mission's own tick deadline, the loop that was
            # supposed to notice that has itself stopped running. B3.3
            # aborts a hung tick from inside that loop -- which is exactly
            # why it cannot report a loop that is no longer there. The
            # deadline is reported alongside so the check does not carry a
            # second copy of it.
            "seconds_since_last_tick": (
                status["seconds_since_last_tick"] if running and status else None
            ),
            "tick_timeout_s": state["tick_timeout_s"],
            # Description, never verdict -- see MissionRunner._tick_rate_hz()
            # for why a rate has no knowable target here.
            "tick_rate_hz": status["tick_rate_hz"] if running and status else None,
            "drills_allowed": bool(config["allow_drills"]),
            # Which model this brain pins for policy: "vision", or null for
            # "whatever the vision service defaults to". Reported so the twin
            # can SHOW the model a mission would run rather than leaving it
            # to be inferred -- the failure this project actually hit was a
            # week of walks attributed to a model that was never running.
            # Deliberately not resolved against the vision service here: that
            # would put an HTTP call in a health check that gets polled. The
            # twin already fetches GET /navigate/models and can resolve the
            # null case itself.
            "navigate_model_id": config["navigate_model_id"] or None,
            # The other lever, reported for exactly the same reason. A walk
            # attributed to the wrong wording is as wrong as one attributed to
            # the wrong model, and wording moved a FORWARD rate from 0.000 to
            # 1.000 in this project's own 3x3 matrix.
            "navigate_prompt_variant": config["navigate_prompt_variant"] or None,
            # "Will a POST /recording/frame actually succeed here" -- true
            # either because this brain stores locally, or because it
            # forwards to one that does (recording_proxy_url). Before the
            # recording proxy existed these were the same thing; now a
            # brain can have allow_recording: false and still legitimately
            # accept recordings, so checking allow_recording alone here
            # would make web-twin/index.html's pre-flight check in
            # startGuidance() (Guide/Robot view) reject a walk that would
            # have actually worked.
            "recording_allowed": bool(config["allow_recording"]) or bool(config["recording_proxy_url"]),
            "faults": list(drills.FAULTS),
        }

    return app


def _idle_status() -> dict:
    return {"running": False, "outcome": "idle", "step": 0, "log_tail": [], "fault": drills.NONE}


app = create_app()
