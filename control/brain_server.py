"""
brain_server.py

Phase B2 -- the brain as a service.

Runs on :8001 next to robot/server.py on :8000 (both on the Pi, in the
target topology) and owns exactly one thing: the autonomy loop. It drives
a MissionRunner as an asyncio background task, so a mission survives the
HTTP request that started it -- which is the point. Today the browser IS
the loop, so backgrounding the phone's tab halts autonomy.

    POST /mission/start   {"target_object": "red backpack", "max_steps": 120,
                           "policy": "frontier" | "vision" | "tiered",
                           "fault": "none"}
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
phase S2). `policy: "tiered"` is the same call wrapped in
brain/tiered.py's trigger discipline (P2): a local YOLO + CLIP pipeline
looks at every frame for free and the paid call goes out only on
`mission_start`, `candidate_sighting` or `cold_search`. **Those models
run in THIS process**, which is why the pipeline is built at mission
start rather than on the first tick: `ultralytics`/`torch` are optional
(requirements-perception.txt), and a missing one has to be a 400 naming
it, never a vision failure discovered three ticks into a mission that is
already driving a robot.

`fault` runs one of control/drills.py's failsafe drills instead of a
normal mission -- the only way to demonstrate B3.2 and B3.3 from the twin,
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
from brain.perceive import (DEFAULT_CLIP, DEFAULT_DETECTOR,
                            DEFAULT_MATCH_PROBABILITY, PerceptionUnavailable)
from brain.tiered import tiered_vision_fn_for
from control import drills
from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from control.recording_routes import (  # noqa: F401 -- re-exported, see above
    FRAME_SUFFIX,
    MAX_FRAME_BYTES,
    MAX_FRAMES_PER_WALK,
    WALK_NAME,
    FinishWalkRequest,
    RecordFrameRequest,
    mount_recording_routes,
)
from control.remote_robot import RemoteRobot
from control.remote_world import RemoteWorld
from control.walk_store import walk_store_from_config
from robot.identity import git_revision, log_identity
from robot.interface import RobotInterface
from world.interface import NullWorld, WorldInterface

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
# The walk naming contract, the frame models and the two write routes now
# live in control/recording_routes.py so the recordings Lambda can mount the
# same code -- see that module's docstring for why the write path follows
# the storage rather than the brain. Re-exported here because this module
# was their home and both tests and callers still import them from it.
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


# The policies that reach the cloud vision service, and therefore need a
# vision_url, a target object and a validated model/wording. "tiered"
# reaches it far less often (that is the whole point of P2), but "less
# often" is not "never" -- mission_start alone guarantees one call.
CLOUD_POLICIES = ("vision", "tiered")


def _perception_available() -> bool:
    """Whether this process COULD build a tiered mission's pipeline.

    `find_spec`, not an import: importing torch costs seconds and this is
    answered inside a health check the twin polls. The question being
    answered is "is the package installed", which find_spec answers
    exactly; whether the weights then load is a mission-start question and
    stays one -- _tiered_vision_fn() is the thing that actually knows.

    Reported so the twin can say the policy is unavailable BEFORE the
    operator picks it, rather than only after a 400. It is a description,
    never a verdict: control/health.py owns what "unhealthy" means, and a
    brain with no perception extras installed is a perfectly healthy brain
    for the other two policies.
    """
    from importlib.util import find_spec

    try:
        return all(find_spec(m) is not None
                   for m in ("ultralytics", "open_clip", "torch"))
    except (ImportError, ValueError):
        # find_spec raises on a half-installed package rather than
        # returning None. Same answer either way: it cannot be used.
        return False


def _frames_are_simulated(robot: RobotInterface) -> bool:
    """Does this robot's camera show a raycaster render?

    Asked once, at mission start, because it decides which perception the
    tiered policy may use -- and 1.12 makes that a rule rather than a
    preference: no detector runs on a render. Read off the frame's own
    `metadata.source`, which is PROVENANCE: `MockRobot` stamps "sim" and
    nothing with a real camera does. Using provenance to pick a perception
    source is what the field is for; a policy deciding where to drive on it
    would not be.

    Any failure answers False -- "assume a real camera" -- because the cost
    of that mistake is loading models that then see nothing, and the cost of
    the opposite is steering a real robot on detections nobody measured.
    """
    try:
        frame = robot.get_camera_frame()
    except Exception:  # noqa: BLE001 -- see docstring
        return False
    return (frame.get("metadata") or {}).get("source") == "sim"


def _tiered_vision_fn(target: str, cloud_vision_fn, config: dict,
                      simulated: bool = False):
    """Wrap the cloud vision_fn in brain/tiered.py's trigger discipline.

    **This is where the optional heavy dependencies are actually loaded**,
    and it is deliberately on the mission-start path rather than the first
    tick. `ultralytics` and `torch` are kept out of requirements.txt on
    purpose (brain/perceive.py's own note: a multi-gigabyte install that
    no automated test may need), so a checkout that has never installed
    requirements-perception.txt is a normal state, not a broken one -- and
    the honest answer to it is a 400 naming the missing package before a
    single motor turns.

    Discovering it inside a tick would be much worse than merely late: a
    `PerceptionUnavailable` there is counted by failsafe B3.2 as a vision
    failure, so the mission would limp through three of them and die
    reporting "vision unavailable 3 times in a row" -- the same
    diagnose-the-wrong-thing failure `_validate_navigate_choices()` was
    written to prevent for model ids, with a robot standing in a room for
    the duration.
    """
    kwargs = {
        "consecutive_frames": config["tier_consecutive_frames"],
        "cold_search_after": config["tier_cold_search_after"],
        # 0 in config means "off"; the policy takes None for that, because
        # a 0cm bar would fire on every frame rather than never.
        "cold_search_after_cm": (config["tier_cold_search_after_cm"] or None),
        "async_cloud": bool(config["tier_async_cloud"]),
        "hold_goal": bool(config["tier_hold_goal"]),
        "steer_on_sight": bool(config["tier_steer_on_sight"]),
        "spin_guard_after": config["tier_spin_guard_after"],
        "hold_bearing": bool(config["tier_hold_bearing"]),
        "hold_bearing_max_m": config["tier_hold_bearing_max_m"],
        "stale_after": config["tier_stale_after"],
        "max_calls": config["tier_max_calls"] or None,
        # 1.11a, reported and not enforced -- see brain/tiered.py's
        # corroboration block. It changes no decision; it puts a number on
        # the panel and in the walk so the next walks measure it.
        "corroboration_bar": config["tier_corroboration_bar"],
    }
    if simulated:
        # 1.12, built at last (PLAN-ros-alignment.md R1): the simulator
        # reports what its geometry says is visible, and nothing loads YOLO
        # or CLIP to look at a raycaster wall. Before this a tiered mission
        # on the Sim tab ran both models on renders -- forbidden, and blind,
        # so the tier could never steer there. The panel's Models line reads
        # "sim ground truth", and every tier frame carries `synthesised`.
        from brain.perceive import FrameReportedPipeline

        return tiered_vision_fn_for(target, cloud_vision_fn,
                                    pipeline=FrameReportedPipeline(target), **kwargs)
    pipeline_kwargs = {}
    if config["perception_detector"]:
        pipeline_kwargs["weights"] = config["perception_detector"]
    if config["perception_clip_model"]:
        pipeline_kwargs["clip_model"] = config["perception_clip_model"]
    if config["perception_match_probability"]:
        pipeline_kwargs["match_probability"] = float(config["perception_match_probability"])
    if config["perception_match_margin"]:
        pipeline_kwargs["match_margin"] = float(config["perception_match_margin"])
    if config["perception_crop_path"]:
        pipeline_kwargs["crop_path"] = config["perception_crop_path"]
    if config["perception_floor_mask"]:
        pipeline_kwargs["floor_mask"] = True
    try:
        from brain.perceive import pipeline_for

        pipeline = pipeline_for(target, **pipeline_kwargs)
    except PerceptionUnavailable as e:
        # Re-raised as ValueError because start_mission() maps that to a
        # 400 with the message intact, and the message is the useful part:
        # it names the pip command.
        raise ValueError(f"The tiered policy cannot start: {e}") from e
    except Exception as e:  # noqa: BLE001 -- a bad weights file, a dead download
        raise ValueError(
            f"The tiered policy could not load its perception models: {e}"
        ) from e
    return tiered_vision_fn_for(target, cloud_vision_fn, pipeline=pipeline, **kwargs)


def create_app(
    config_path: Optional[str] = None,
    robot_factory: Optional[Callable[[], RobotInterface]] = None,
    runner_factory: Optional[Callable[..., MissionRunner]] = None,
    world_factory: Optional[Callable[[RobotInterface], WorldInterface]] = None,
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

    def default_world_factory(robot: RobotInterface) -> WorldInterface:
        """The WORLD half, and `control/`'s first consumer of it.

        **It takes the robot, and that is the whole point of this
        signature.** `robot/server.py` serves both halves today, so the
        world lives wherever the body does -- and reading the URL off the
        robot that was actually built is what stops the two pointing at
        different machines. Deriving it from `config["robot_url"]` instead
        looked identical and was wrong: any caller supplying its own
        `robot_factory` (which every integration test does, and which is how
        a second robot would ever be addressed) silently got a world on the
        default URL, answering 401s about a robot it was not watching.

        `brain.world_url` still wins when set, because R5 moves the pose and
        the map behind the SLAM bridge while the body stays on the robot
        runtime -- at which point these two genuinely are different hosts and
        one string says so. `RemoteWorld` reads JSON and contains no hint
        that ROS exists, which is the test of whether that wall was drawn in
        the right place.

        `robot_secret` rather than a third one: the world routes are gated by
        whichever server hosts them, and today that is the robot's.
        """
        return RemoteWorld(
            config.get("world_url") or getattr(robot, "base_url", None)
            or config["robot_url"],
            secret=robot_secret,
            timeout=config["request_timeout_s"],
        )

    def default_runner_factory(robot: RobotInterface, req: MissionStartRequest) -> MissionRunner:
        vision_fn = None
        if req.policy in CLOUD_POLICIES:
            # The policy exists (brain/vision_agent.py); what it needs is an
            # endpoint to ask and a target to look for. Both are checked here
            # rather than failing on the first tick, so a misconfigured brain
            # says so at start time instead of after a paid call.
            #
            # The tiered policy needs both for the same reasons and one
            # more of its own: the target string is what CLIP scores every
            # crop against, so a mission with no object has nothing to
            # perceive as well as nothing to ask about.
            if not config["vision_url"]:
                raise ValueError(
                    f"The {req.policy} policy needs a vision service. Set "
                    "brain.vision_url in config/robot.yaml (or VISION_URL in the "
                    "environment)."
                )
            if not req.target_object:
                raise ValueError(
                    f"The {req.policy} policy searches for an object -- /navigate "
                    "takes a target_object. A target_room-only mission needs "
                    "policy='frontier'."
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
            if req.policy == "tiered":
                vision_fn = _tiered_vision_fn(req.target_object, vision_fn, config,
                                              simulated=_frames_are_simulated(robot))

        kwargs = dict(
            target_object=req.target_object,
            target_room=req.target_room,
            mission=req.mission,
            max_steps=req.max_steps or config["max_steps"],
            stuck_after=config["stuck_after"],
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
        runner = runner_class(robot, world=world(), **kwargs)
        # Metrics shipping, configured on the runner rather than passed
        # through its constructor: MissionRunner's signature is the
        # harness contract (AGENT-HARNESS.md) and a dashboard is not part
        # of it. Off entirely unless a URL is set.
        runner.metrics_url = config.get("metrics_url", "")
        runner.metrics_secret = (config.get("metrics_secret")
                                 or os.environ.get("WALKS_SHARED_SECRET", ""))
        runner.git_revision = git_revision()
        # The knobs whose effect the dashboard exists to watch. A trend
        # line without the config that produced it is a set of numbers
        # with no cause attached.
        runner.metrics_config = {
            k: config.get(k) for k in (
                "tier_cold_search_after", "tier_cold_search_after_cm",
                "tier_async_cloud", "tier_hold_goal", "tier_steer_on_sight",
                "tier_hold_bearing", "tier_hold_bearing_max_m",
                "tier_spin_guard_after", "tier_consecutive_frames",
                "perception_floor_mask", "perception_crop_path",
                "navigate_model_id", "navigate_prompt_variant",
            ) if k in config
        }
        return runner

    make_robot = robot_factory or default_robot_factory
    make_world = world_factory or default_world_factory
    make_runner = runner_factory or default_runner_factory

    # One robot client for the process. It is built up front (construction
    # does no I/O) so that POST /mission/stop can stop the car even when no
    # mission has ever run.
    state: dict = {
        "robot": None, "world": None, "runner": None, "task": None,
        "fault": drills.NONE, "tick_timeout_s": config["tick_timeout_s"],
    }

    def robot() -> RobotInterface:
        if state["robot"] is None:
            state["robot"] = make_robot()
        return state["robot"]

    def world() -> WorldInterface:
        """The world client, built once, and never able to stop a mission.

        Falls back to `NullWorld()` if construction fails at all. The world
        is ADVISORY to every policy that reads it -- the frontier explorer
        uses the pose to choose between directions the distance sensor has
        already called clear, so losing it costs exploration efficiency and
        nothing else. A misconfigured or unreachable mapper must therefore
        degrade, exactly as `MissionAgent._pose()` degrades when a read
        fails; letting it raise here would turn a soft problem into a
        refusal to start, which is the same trade this file already refuses
        to make for an unreachable /navigate/models endpoint.
        """
        if state["world"] is None:
            try:
                state["world"] = make_world(robot())
            except Exception as exc:  # noqa: BLE001 -- see docstring
                logger.warning(
                    "could not build a world client (%s) -- missions will "
                    "explore by the right-hand rule instead of the map", exc)
                state["world"] = NullWorld()
        return state["world"]

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

    # Mounted, not defined here: the recordings Lambda mounts the same two
    # routes against the same store. `proxy` keeps this brain's
    # forward-to-a-peer behaviour (teleop-brain has no storage of its own),
    # and allow_recording is read per request so flipping it needs no
    # restart-time reasoning.
    mount_recording_routes(
        app, store,
        prefix=prefix,
        require_secret=require_secret,
        allow_recording=lambda: bool(config["allow_recording"]),
        proxy=_proxy_recording if config["recording_proxy_url"] else None,
    )

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
            # Phase P2. Whether `policy: "tiered"` can run here at all, and
            # which models it would load if it did -- so the twin can name
            # the detector on screen before anything is spent, and grey the
            # policy out instead of offering one that will 400. Same reason
            # navigate_model_id is reported above: the failure this project
            # actually hit was a week of walks attributed to a model that
            # was never running.
            "perception_available": _perception_available(),
            "perception_detector": config["perception_detector"] or DEFAULT_DETECTOR,
            "perception_clip_model": config["perception_clip_model"] or DEFAULT_CLIP,
            "perception_match_probability": (config["perception_match_probability"]
                                             or DEFAULT_MATCH_PROBABILITY),
            "perception_match_margin": config["perception_match_margin"] or None,
            "perception_crop_path": config["perception_crop_path"],
            "perception_floor_mask": bool(config["perception_floor_mask"]),
            "tier_consecutive_frames": config["tier_consecutive_frames"],
            "tier_cold_search_after": config["tier_cold_search_after"],
            "tier_cold_search_after_cm": config["tier_cold_search_after_cm"],
            "tier_async_cloud": bool(config["tier_async_cloud"]),
            "tier_hold_goal": bool(config["tier_hold_goal"]),
            "tier_steer_on_sight": bool(config["tier_steer_on_sight"]),
            "tier_spin_guard_after": config["tier_spin_guard_after"],
            # Published so the panel can name the bar it is reporting
            # against. A verdict shown without the number it was taken at
            # is not readable, and 1.11a's whole finding is that the bar
            # for corroborating is not the bar for detecting.
            "tier_corroboration_bar": config["tier_corroboration_bar"],
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
