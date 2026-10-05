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

Auth: every route except /health, / and the page's static assets requires a
matching x-app-secret header when APP_SHARED_SECRET is set in the
environment (see require_secret() below) -- needed because the server is
reachable from the public internet through service/tunnel/. /health stays
open (a health check cannot always send custom headers) and / stays open
(the page has to load before a user can enter the secret in the UI).

Watchdog (build plan Phase 9; failsafe B3.1 in
PLAN-brain-relocation.md): if commands stop arriving for ~1 second, the
motors stop. `last_command_at` is updated by /action, /stop and non-zero
/wheels (under drive: ros, every actuator post) -- a sensing read is not a
command, and counting one would let a passive observer (the twin polling
/frame while it watches a mission) hold the watchdog off indefinitely. A background task polls it and calls
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

import gc
import os
import threading
import math
import time

import httpx
import asyncio
import logging
from contextlib import asynccontextmanager, contextmanager
from pathlib import Path
from typing import List, Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel

from robot.factory import get_robot, load_config
from robot.identity import log_identity
from robot.interface import (
    DRIVER_AUTONOMOUS, DRIVER_MANUAL, DRIVER_UNKNOWN, WheelFeedbackLost, driver_priority)

# The ROS container's driver name on POST /wheels (R2b, R4).
DRIVER_ROS = "ros"
from robot.safety import SafetyController, SafetyViolation, path_zone_indices
from world.factory import get_world

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


class WheelsRequest(BaseModel):
    """R2b: left/right wheel angular velocities, rad/s, positive forward."""
    left_rad_s: float
    right_rad_s: float


class TeleopFrameRequest(BaseModel):
    image_base64: str
    media_type: str = "image/jpeg"


# How often the watchdog loop wakes. Named (phase M5) because /health now
# publishes it: a health check has to know what "fresh" means for this
# server's own poll, and hardcoding a second copy of 0.1 in control/health.py
# is how the two would drift apart.
WATCHDOG_POLL_INTERVAL_S = 0.1
# R2b: how often a standing wheel-velocity command is vetted and, in the sim,
# integrated. 20Hz is the rate nav2's local controller emits cmd_vel at, so
# the loop never runs slower than the thing feeding it.
WHEEL_LOOP_INTERVAL_S = 0.05
# 3.24 G3: how long the actuator plugin may be silent before the ROS stack
# counts as DOWN. It posts /wheels at 20 Hz, so 0.5 s is ten missed posts --
# long past jitter, well inside the watchdog.
ROS_SILENCE_S = 0.5
# After a stop that ends a nav2 goal (spec review 3, V2): how often the
# server re-reads the goal and re-sends the cancel until nav2 reports it no
# longer pending or active. Non-zero `ros` wheel commands are held at zero
# meanwhile, and for GOAL_STOP_SETTLE_S after -- the same derivation as
# robot/ros_drive.py's STOP_HOLD_S (twist_mux's input timeout plus the
# controller's cmd_vel_timeout, which add, plus one plugin period).
GOAL_STOP_POLL_S = 0.25
GOAL_STOP_SETTLE_S = 0.6


@contextmanager
def _held(lock):
    """Release a lock already acquired with `acquire(blocking=False)`."""
    try:
        yield
    finally:
        lock.release()


def watchdog_should_stop(last_command_at: float, now: float, timeout_s: float) -> bool:
    """Pure decision logic -- see module docstring for why this is
    tested separately from the async polling loop that calls it."""
    return (now - last_command_at) > timeout_s


def create_app(config_path: Optional[str] = None) -> FastAPI:
    """Builds a fresh app + robot + safety controller. Tests call this
    directly to get an isolated instance per test; the module-level
    `app` below is what `uvicorn robot.server:app` actually serves."""
    # Phase M5. First line out of the process, before anything can fail in
    # a way that makes you wonder which build you are looking at.
    ident = log_identity("vision-picar robot server", config_path)
    config = load_config(config_path) if config_path else load_config()
    watchdog_timeout = config.get("safety", {}).get("watchdog_timeout_s", 1.0)
    min_distance = config.get("safety", {}).get("min_distance_cm", 20.0)
    sensor_to_bumper = config.get("safety", {}).get("sensor_to_bumper_cm", 0.0)
    # Same resolution get_robot() uses internally -- reported in /health so
    # a client can tell "wrong deployment" (e.g. Robot view's "drive via
    # brain" pointed at a mode: sim server) apart from "not reachable at
    # all", which a bare connectivity check can't distinguish.
    mode = os.environ.get("ROBOT_MODE") or config.get("mode", "sim")

    robot = get_robot(config_path) if config_path else get_robot()
    # The WORLD model -- the map and the robot's pose on it (N1). Its own
    # abstraction, because a map is world state and RobotInterface holds
    # body state only (CLAUDE.md section 2). `none` by default, in which
    # case both routes below answer "unusable" and nothing changes.
    #
    # The robot is handed over opaquely: only world/factory.py looks
    # inside it, and only in its sim branch, where the map and the body
    # have to be two views of ONE GridWorld.
    world_model = get_world(config_path, robot=robot) if config_path \
        else get_world(robot=robot)
    safety = SafetyController(robot, min_distance_cm=min_distance,
                              sensor_to_bumper_cm=sensor_to_bumper)
    # R4: under `drive: ros` every verb is velocities through the ROS
    # container (robot/ros_drive.py), which is then the one wheel writer.
    by_velocity = bool(getattr(robot, "drives_by_velocity", False))
    # 3.24 G3, the FALLBACK (decided by the user: only a person drives on it).
    # When the ROS stack is down, a person's verbs run on the robot UNDER the
    # ROS wrapper through drive: direct's guarded path -- the same
    # SafetyController, re-vetted every period -- and autonomy is refused.
    fallback_safety = (SafetyController(robot.inner, min_distance_cm=min_distance,
                                        sensor_to_bumper_cm=sensor_to_bumper)
                       if by_velocity else None)
    state = {
        # True while a direct-mode verb is being carried out (3.22).
        "verb_active": False,
        "last_command_at": time.monotonic(),
        "wheel_posts": 0,
        # 3.24 G3: when the actuator plugin last posted /wheels -- ROS's pulse.
        "last_ros_post_at": None,
        "refusal_counts": {},
        # Phase M4. Who last drove, when, and what was last refused. The
        # server has never had a notion of a driver at all: the D-pad and a
        # remote mission both posted to /action and the later one simply
        # won. That is last-writer-wins between two loops that each read
        # the other's moves as the world changing under them.
        # None until someone actually drives -- "nobody has touched this
        # robot yet" is a real state and should not read as an anonymous
        # driver who has since gone quiet.
        "driver": None,
        "driver_at": 0.0,
        "last_refusal": None,
        # 3.18 part 2: the wheel loop's own timing, while the wheels turn.
        # A vet covers the travel until the NEXT vet, so a late period is a
        # period the robot moved unvetted -- this is how a live run says
        # whether that happened, rather than leaving it to be inferred.
        "wheel_loop": {"moving_ticks": 0, "late_ticks": 0, "max_dt_s": 0.0},
        # Phase M5. When the watchdog's own loop last ran, which is a
        # different fact from `last_command_at` -- that measures the
        # CLIENT's silence, this measures whether the guard measuring it is
        # still alive. Nothing noticed before if the asyncio task died: the
        # server kept answering every request, /health kept reporting a
        # growing silence, and the thing that was supposed to act on that
        # silence was gone. Seeded at start-up so a server that has not yet
        # polled once does not read as stalled.
        "watchdog_polled_at": time.monotonic(),
    }

    def authority_holder(now: float):
        """Who currently holds the robot, or None if authority has lapsed.

        **Authority lapses on silence, on the same clock the watchdog
        already keeps.** A driver holds the robot only while it is
        actively driving it; `watchdog_timeout_s` after its last command
        the motors are stopped and the claim is released, so the next
        driver -- whoever it is -- starts clean. That reuses a deadman
        this server already has rather than inventing a second lifetime,
        and it means no explicit release call exists to be forgotten.
        """
        if now - state["driver_at"] > watchdog_timeout:
            return None
        return state["driver"]

    def refuse(reason: str, detail: str, driver: str):
        """Record a refusal and answer with it.

        Every refusal carries a machine-readable `reason` as well as
        prose, because a teleop UI showing the stick forward and the robot
        still is unusable (Microduck's `robotd-design` section 3.2). The
        prose is for a person; the reason is what the twin colours and
        what a client can branch on -- `RemoteRobot` has to tell a safety
        veto (retry later) from a preemption (stop, you are not driving)
        and cannot do that by reading a sentence.
        """
        state["last_refusal"] = {
            "reason": reason,
            "detail": detail,
            "driver": driver,
            "at": time.monotonic(),
        }
        # R6's criterion 6: how often each collar acted, not only the last time.
        state["refusal_counts"][reason] = state["refusal_counts"].get(reason, 0) + 1
        logger.warning(f"refused ({reason}) for {driver}: {detail}")
        return {"executed": False, "reason": reason, "detail": detail}

    async def watchdog_loop():
        while True:
            await asyncio.sleep(WATCHDOG_POLL_INTERVAL_S)
            now = time.monotonic()
            state["watchdog_polled_at"] = now
            # A verb in progress is a busy brain, not a silent one: it is
            # bounded, re-vetted every period, and /stop still ends it (3.22).
            if state["verb_active"]:
                continue
            if watchdog_should_stop(state["last_command_at"], now, watchdog_timeout):
                robot.stop()
                # Phase M4: the watchdog is a stop, not a refusal of any
                # particular command -- but it is the reason the motors are
                # off, and "why is the robot not moving" is the question
                # /health has to be able to answer. Recorded once per
                # silence rather than ten times a second.
                last = state["last_refusal"]
                if not (last and last["reason"] == "watchdog"
                        and last["at"] >= state["last_command_at"]):
                    state["last_refusal"] = {
                        "reason": "watchdog",
                        "detail": (f"no command for over {watchdog_timeout}s -- "
                                   "motors stopped"),
                        "driver": state["driver"],
                        "at": now,
                    }

    def ros_up(now: float) -> bool:
        """Is the ROS stack alive? Its actuator plugin posts /wheels at
        20 Hz; ROS_SILENCE_S without one is a dead container (3.24 G3).

        **And the bridge must be answering** (handoff 2026-10-02 1d): a
        failed send to it marks ROS down until it answers again, because a
        dead bridge behind a live plugin otherwise refused every verb -- a
        person's included -- with no fallback at all."""
        last = state["last_ros_post_at"]
        if last is None or now - last >= ROS_SILENCE_S:
            return False
        bridge_up = getattr(robot, "bridge_up", None)
        return bridge_up() if bridge_up is not None else True

    def arbitrate(driver: str, now: float):
        """Refusal dict if `driver` may not drive right now, else None.

        Phase M4's order, shared by `/action` and `/wheels`: a strictly
        lower-ranked driver is refused while a higher one is driving. **R2b
        adds one rule:** at the AUTONOMOUS rank the holder is exclusive until
        it lapses -- the brain and the ROS stack are equal rank, and letting
        equal ranks through (right for two taps of one person's D-pad) would
        let two autonomous writers interleave commands on one robot. One
        writer at a time is the industry convention `twist_mux` encodes.

        **3.23 adds a second:** a nav2 goal in progress is the ROS stack
        driving, so every OTHER autonomous driver is refused while one is
        pending or active. The goal is not a claim on the watchdog's clock --
        nav2 may pause for seconds while it plans -- so it is asked of the
        world itself, the same state the twin draws.
        """
        if driver != DRIVER_ROS and driver_priority(driver) == DRIVER_AUTONOMOUS:
            goal = goal_in_progress()
            if goal:
                return refuse(
                    "preempted",
                    f"a nav2 goal is {goal.get('state')} -- one autonomous driver "
                    f"at a time, so {driver} was refused until it ends or is cancelled",
                    driver,
                )
        holder = authority_holder(now)
        if not holder or holder == driver:
            return None
        mine, theirs = driver_priority(driver), driver_priority(holder)
        if mine < theirs:
            return refuse(
                "preempted",
                f"{holder} is driving -- {driver} is lower priority and was refused",
                driver,
            )
        if mine == theirs == DRIVER_AUTONOMOUS:
            return refuse(
                "preempted",
                f"{holder} is driving -- one autonomous driver at a time, "
                f"so {driver} was refused until it lapses",
                driver,
            )
        return None

    def goal_in_progress():
        """The world's nav2 goal while it is pending or active, else None.

        Only a world that can plan has goals (world/ros_world.py). If its
        bridge cannot be read, answer None: with the bridge down no goal can
        be sent or followed through it, and refusing every autonomous
        command on a read error would turn an outage into a lockout.
        """
        if not hasattr(world_model, "get_goal"):
            return None
        try:
            goal = (world_model.get_goal() or {}).get("goal")
        except Exception:  # noqa: BLE001 -- an unreadable bridge, see above
            return None
        if goal and goal.get("state") in ("pending", "active"):
            return goal
        return None

    # Serialises everything that moves the robot: /action and /wheels run on
    # the threadpool, the wheel loop on the event loop, and MockRobot is not
    # thread-safe. Held for microseconds -- nothing inside it waits.
    motion_lock = threading.Lock()
    # A stop ending a nav2 goal (see _end_goal_after_stop): the generation,
    # and whether nav2's wheel commands are held at zero meanwhile.
    goal_stop = {"gen": 0, "hold": False}
    goal_stop_lock = threading.Lock()

    async def wheel_loop():
        """Vet and (in the sim) integrate a standing wheel command -- R2b.

        Every period: if the wheels are commanded to turn, re-vet the command
        against CURRENT clearance -- a forward command that was safe when it
        was sent stops being safe as the wall approaches -- and let `dt`
        elapse. A clamp is recorded as a refusal once per command, so /health
        says why the robot stopped. The watchdog still owns silence: it calls
        `stop()`, which zeroes the standing command.
        """
        last = time.monotonic()
        while True:
            await asyncio.sleep(WHEEL_LOOP_INTERVAL_S)
            now = time.monotonic()
            dt, last = now - last, now
            # Never WAIT for the lock here: this runs on the event loop, and
            # a direct-mode verb holds the lock for as long as it drives --
            # a second or two on real motors. Waiting froze every route with
            # it, /stop included (3.22). A verb re-vets itself every period
            # (`SafetyController.run_verb()`), so a skipped tick loses nothing.
            if not motion_lock.acquire(blocking=False):
                continue
            try:
                with _held(motion_lock):
                    w = robot.get_wheel_state()
                    if not w.get("usable"):
                        continue
                    left = w["left"]["velocity_rad_s"]
                    right = w["right"]["velocity_rad_s"]
                    if left == 0 and right == 0:
                        # 3.30: a person keeps walking while the robot waits.
                        # Sim bodies only; a real one has no clock to pass.
                        pass_time = getattr(robot, "pass_time", None)
                        if pass_time is not None:
                            pass_time(dt)
                        continue
                    wl = state["wheel_loop"]
                    wl["moving_ticks"] += 1
                    wl["late_ticks"] += dt > 2 * WHEEL_LOOP_INTERVAL_S
                    wl["max_dt_s"] = round(max(wl["max_dt_s"], dt), 4)
                    new_left, new_right, reason = safety.vet_wheel_velocity(left, right)
                    if (new_left, new_right) != (left, right):
                        # Clamped, or only SLOWED near the line (3.24 G2's look-
                        # ahead): either way the vetted speeds are what drive.
                        robot.set_wheel_velocity(new_left, new_right)
                    if reason:
                        last_ref = state["last_refusal"]
                        if not (last_ref and last_ref["reason"] == "safety_distance"
                                and last_ref["at"] >= state["last_command_at"]):
                            refuse("safety_distance", reason, state["driver"] or DRIVER_UNKNOWN)
                    robot.advance(dt)
            except Exception:  # noqa: BLE001 -- the loop must outlive one bad tick
                logger.exception("wheel loop tick failed")

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        task = asyncio.create_task(watchdog_loop())
        wheels_task = asyncio.create_task(wheel_loop())
        # 3.37: everything start-up built (the app, the backend, every module
        # imported) out of the collector's reach. On the Jetson a full
        # (generation-2) collection walked it all every ~75 s and paused the
        # wheel loop 55-92 ms -- the late ticks 3.33's headroom run counted.
        # Unfrozen at shutdown, so an app started and stopped in-process (the
        # test suite, hundreds of times) leaves nothing pinned behind it.
        gc.collect()
        gc.freeze()
        try:
            yield
        finally:
            task.cancel()
            wheels_task.cancel()
            gc.unfreeze()

    app = FastAPI(title="vision-picar robot server", lifespan=lifespan)

    # See module docstring. "" (the default) reproduces every route exactly
    # as before this existed -- prefix + "/frame" == "/frame".
    prefix = os.environ.get("ROUTE_PREFIX", "").rstrip("/")
    env_label = os.environ.get("ENV_LABEL", "").strip()

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
    def do_action(req: ActionRequest, x_driver: str = Header(default="")):
        now = time.monotonic()
        driver = (x_driver or "").strip() or DRIVER_UNKNOWN

        # Phase M4 -- arbitration, before anything moves. A decided order,
        # not last-writer-wins: see robot/interface.py's DRIVER_PRIORITY and
        # AGENT-HARNESS.md 4.1. A strictly lower-ranked driver is refused
        # while a higher one is driving; equal MANUAL rank passes, so two
        # D-pad taps never fight; the AUTONOMOUS rank is exclusive (R2b).
        refused = arbitrate(driver, now)
        if refused:
            return refused

        state["last_command_at"] = now
        state["driver"] = driver
        state["driver_at"] = now
        try:
            if by_velocity and not ros_up(now):
                # 3.24 G3: the ROS stack is down. A PERSON drives on through
                # drive: direct's guarded verb on the robot underneath; the
                # autonomy is refused, and the brain ends its mission on it.
                if driver_priority(driver) == DRIVER_AUTONOMOUS:
                    robot.stop()
                    return refuse("ros_unavailable",
                                  "the ROS stack is down (no /wheels from the actuator for "
                                  f"over {ROS_SILENCE_S} s) -- only a person may drive on the "
                                  "fallback", driver)
                with motion_lock:
                    state["verb_active"] = True
                    try:
                        result = fallback_safety.check_and_execute(
                            req.action, speed=req.speed, duration=req.duration, angle=req.angle
                        )
                    finally:
                        state["verb_active"] = False
                        state["last_command_at"] = time.monotonic()
                result = {**result, "via": "direct-fallback"} if isinstance(result, dict) else result
            elif by_velocity:
                # R4, `drive: ros`: the verb is a stream of twists that the ROS
                # chain turns back into POST /wheels -- which needs
                # motion_lock, as does the wheel loop that moves the robot.
                # Holding it here would deadlock the verb against its own
                # motion, so it is not held; MockRobot's reads are safe
                # alongside the wheel loop, and M4 above already decided who
                # may drive.
                # A verb allowed to drive lifts the stop's goal hold, as it
                # lifts RosDriveRobot's own stop hold: under this drive its
                # twists come back as `ros` wheel posts. The ending loop keeps
                # cancelling the goal regardless.
                goal_stop["hold"] = False
                try:
                    with robot.driving_as(driver):
                        result = safety.check_and_execute(
                            req.action, speed=req.speed, duration=req.duration, angle=req.angle
                        )
                except (httpx.HTTPError, RuntimeError) as e:
                    # The ROS side is unreachable. Stop directly -- the stop
                    # does not go through ROS -- and say why, by name.
                    robot.stop()
                    return refuse("ros_unavailable", f"drive: ros, and the bridge failed: {e}", driver)
            else:
                with motion_lock:
                    state["verb_active"] = True
                    try:
                        result = safety.check_and_execute(
                            req.action, speed=req.speed, duration=req.duration, angle=req.angle
                        )
                    finally:
                        state["verb_active"] = False
                        # The verb's end is the brain's last word, not its start.
                        state["last_command_at"] = time.monotonic()
            return {"executed": True, "result": result, "driver": driver}
        except SafetyViolation as e:
            return refuse("safety_distance", str(e), driver)
        except WheelFeedbackLost as e:
            # 3.34: the body cannot measure its wheels, so it will not move
            # them. Not a veto to steer around: RemoteRobot ends the mission.
            return refuse("no_feedback", str(e), driver)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))

    @app.post(prefix + "/wheels", dependencies=[Depends(require_secret)])
    def set_wheels(req: WheelsRequest, x_driver: str = Header(default="")):
        """A STANDING wheel-velocity command -- phase R2b, what
        `picar_sim_hardware.write()` sends at R4.

        Arbitrated exactly like `/action` (M4, plus one autonomous writer at a
        time), vetted by `SafetyController.vet_wheel_velocity()` -- forward
        against the path clearance, reverse against the lidar's rear beams,
        rotation never -- and then held: `wheel_loop()` re-vets it every
        period as the room changes, and the watchdog zeroes it on silence.
        A backend with no motors answers `unsupported`, never a silent no-op.
        """
        now = time.monotonic()
        driver = (x_driver or "").strip() or DRIVER_UNKNOWN
        if by_velocity:
            # R4, `drive: ros`: this is the ACTUATOR's route, and the ROS
            # container is its only writer. The people and programs that
            # drive reach the wheels THROUGH it -- /action is where M4 decides
            # between them -- so it is not arbitrated against them, and it does
            # not take authority from them. It still feeds the watchdog: if
            # the container dies, these posts stop and the wheels stop.
            if driver != DRIVER_ROS:
                return refuse("not_the_actuator",
                              f"drive: ros -- only '{DRIVER_ROS}' writes the wheels; "
                              f"{driver} drives through /action", driver)
            state["last_command_at"] = now
            state["wheel_posts"] += 1
            state["last_ros_post_at"] = now
            bridge_up = getattr(robot, "bridge_up", None)
            if bridge_up is not None and not bridge_up():
                # The bridge is dead and the plugin is not (spec review 3,
                # V8): a person is driving the fallback, so ROS's actuator
                # must not stay a second writer. The post still proves the
                # plugin alive (stamped above); it moves nothing.
                # Answered directly, not through refuse(): the plugin posts at
                # 20 Hz, and logging each would bury the log.
                return {"executed": False, "reason": "ros_unavailable",
                        "detail": "the bridge is down -- ROS's wheel commands are not "
                                  "applied while a person drives the fallback"}
            if goal_stop["hold"]:
                req = WheelsRequest(left_rad_s=0.0, right_rad_s=0.0)
        else:
            if driver == DRIVER_ROS and goal_stop["hold"]:
                # nav2 still driving a goal a stop is ending: a zero command.
                req = WheelsRequest(left_rad_s=0.0, right_rad_s=0.0)
            if req.left_rad_s == 0 and req.right_rad_s == 0:
                # A ZERO command is not driving, so it never takes or refreshes
                # authority. Found live 2026-09-26: the ROS container, left
                # running beside a `drive: direct` server, posts zeros every
                # cycle as its heartbeat; each post claimed the autonomous slot
                # and a brain mission ended `preempted` on its first step. It
                # stops the wheels only for the driver that holds them; from
                # anyone else it is a no-op, so it cannot cut off their move.
                if authority_holder(now) != driver:
                    return {"executed": True, "driver": driver, "ignored": True,
                            "detail": "zero command from a driver not holding the robot"}
                with motion_lock:
                    robot.set_wheel_velocity(0.0, 0.0)
                return {"executed": True, "driver": driver,
                        "applied": {"left_rad_s": 0.0, "right_rad_s": 0.0}, "clamped": None}
            refused = arbitrate(driver, now)
            if refused:
                return refused
            state["last_command_at"] = now
            state["driver"] = driver
            state["driver_at"] = now
        with motion_lock:
            left, right, reason = safety.vet_wheel_velocity(req.left_rad_s, req.right_rad_s)
            try:
                robot.set_wheel_velocity(left, right)
            except NotImplementedError as e:
                return refuse("unsupported", str(e), driver)
            except WheelFeedbackLost as e:
                if driver == DRIVER_ROS:
                    # The plugin posts at 20 Hz: answered, not logged each time
                    # (as for a dead bridge above).
                    return {"executed": False, "reason": "no_feedback", "detail": str(e)}
                return refuse("no_feedback", str(e), driver)
        if reason:
            refuse("safety_distance", reason, driver)
        return {"executed": True, "driver": driver,
                "applied": {"left_rad_s": left, "right_rad_s": right},
                "clamped": reason}

    @app.post(prefix + "/stop", dependencies=[Depends(require_secret)])
    def stop(x_driver: str = Header(default="")):
        """Always allowed, from anyone, and it neither claims authority nor
        releases it -- phase M4.

        Top of the order in AGENT-HARNESS.md, and the one command that must
        never be arbitrated: a stop that could be refused because someone
        else is driving is not a stop. It does not claim authority either,
        because stopping is not a bid to drive; the holder keeps its claim
        and keeps it only as long as it keeps commanding, exactly as
        before.

        **A stop also ENDS a nav2 goal** (handoff 2026-10-02 1b, decided by
        the user): otherwise nav2 keeps its goal and drives again the moment
        the stop hold ends. The cancel runs on a background thread AFTER
        `robot.stop()`, so the stop never waits on ROS; a person re-sends a
        goal to resume.

        **Except an autonomous stop** (spec review 3, V1; decided by the
        user 2026-10-03): a stop from an autonomous driver other than `ros`
        -- the brain -- zeroes the wheels and spares the goal. While a goal
        holds the robot the brain cannot drive (3.23), so its stop can only
        be a loser's teardown: `MissionRunner` stops the robot on its way
        out of a mission the goal preempted, and that stop used to cancel
        the very goal that won."""
        state["last_command_at"] = time.monotonic()
        driver = (x_driver or "").strip() or DRIVER_UNKNOWN
        result = robot.stop()
        spares_goal = driver != DRIVER_ROS and driver_priority(driver) == DRIVER_AUTONOMOUS
        if hasattr(world_model, "cancel_goal") and not spares_goal:
            with goal_stop_lock:
                goal_stop["gen"] += 1
                goal_stop["hold"] = True
                gen = goal_stop["gen"]
            threading.Thread(target=_end_goal_after_stop, args=(gen,), daemon=True,
                             name="stop-end-goal").start()
        return {"executed": True, "result": result, "driver": driver}

    def _end_goal_after_stop(gen: int):
        """Make the stop's "the goal is ended" true, not merely attempted
        (spec review 3, V2). The goal lives in nav2, not in the bridge, so a
        cancel that fails, or that reaches a goal nav2 has not yet accepted,
        leaves it able to drive. So: cancel, then re-read the goal and
        re-send the cancel every GOAL_STOP_POLL_S until nav2 reports it
        neither pending nor active, holding `ros` wheel commands at zero the
        whole time and for GOAL_STOP_SETTLE_S after. A newer stop or a new
        goal supersedes this loop (the generation), so there is one at a
        time and a person's next goal is never cancelled by it."""
        settled_at = None
        warned = False
        while True:
            try:
                with goal_stop_lock:
                    if goal_stop["gen"] != gen:
                        return
                    goal = (world_model.get_goal() or {}).get("goal") if settled_at is None else None
                    if settled_at is None and goal and goal.get("state") in ("pending", "active"):
                        world_model.cancel_goal()
                    elif settled_at is None:
                        settled_at = time.monotonic()
                    elif time.monotonic() - settled_at >= GOAL_STOP_SETTLE_S:
                        goal_stop["hold"] = False
                        return
            except Exception as e:  # noqa: BLE001 -- keep holding, keep trying
                settled_at = None
                if not warned:
                    logger.warning(f"stop: could not end the nav2 goal yet, retrying: {e}")
                    warned = True
            time.sleep(GOAL_STOP_POLL_S)

    @app.get(prefix + "/distance", dependencies=[Depends(require_secret)])
    def distance():
        return {"distance_cm": robot.get_distance()}

    @app.get(prefix + "/odometry", dependencies=[Depends(require_secret)])
    def odometry():
        """How far the robot has travelled and which way it faces -- phase B.

        Its own route for the same reason `/depth` is: a different sensor,
        and a wedged camera must not take it down with it. A backend that
        cannot measure motion answers `usable: False` here and still
        returns real pixels at `/frame`.

        `RemoteRobot` treats a 404 as "this server predates the route",
        not as a transport failure -- the deployed stacks are redeployed
        one at a time and a mixed fleet is a real state.
        """
        return robot.get_odometry()

    @app.get(prefix + "/wheels", dependencies=[Depends(require_secret)])
    def wheels():
        """Per-wheel position, velocity and encoder count -- phase R2.

        What `picar_sim_hardware`'s `read()` polls (R4). Its writing half is
        `POST /wheels` below (R2b). A backend with no encoders answers
        `usable: False`; one with wheels it cannot measure yet adds
        `awaiting_feedback: true` (handoff 2a).
        """
        return robot.get_wheel_state()

    @app.get(prefix + "/scan", dependencies=[Depends(require_secret)])
    def scan(max_range_m: Optional[float] = None):
        """A 360-degree range scan -- phase R2, what `sim_scan_node` will
        republish as `sensor_msgs/LaserScan` at R4. BODY state (the robot's
        own reading), so it lives beside `/depth`, not under `/world/`: see
        `RobotInterface.get_scan()`. A backend with no lidar answers
        `usable: False`.

        `stamp_unix` is when the scan was TAKEN (R5). The bridge does NOT use
        it for the ROS stamp any more -- see picar_bridge's _poll_scan() for
        how a capture stamp froze nav2's costmaps in R6 -- but it is the
        honest capture time and costs nothing to serve."""
        stamp = time.time()
        return {**robot.get_scan(max_range_m=max_range_m), "stamp_unix": stamp}

    @app.get(prefix + "/depth", dependencies=[Depends(require_secret)])
    def depth():
        """The depth grid, phase M2 -- its own route rather than a field on
        /frame, because it is a different sensor. A camera that has wedged
        must not take the clearance reading down with it (M9), and a
        backend with no depth sensor answers here with an all-unusable grid
        while still returning real pixels there.

        Unauthenticated backends and pre-M2 servers are the reason
        `RemoteRobot` treats a 404 here as "this server has no depth
        route", rather than as a transport failure.

        **`path` is the safety layer's own reduction, published rather than
        recomputed** (phase M3). The twin draws which zones the veto reads
        and what it read off them, and it must not work that out for
        itself: which zones are "the path" is robot-runtime safety logic,
        and CLAUDE.md section 6 allows the browser to duplicate the *brain*
        role only. Same reason `/health` publishes `min_distance_cm`
        instead of letting the page carry a second copy of the threshold.
        The grid itself is untouched -- `zones` is exactly what the backend
        returned."""
        grid = robot.get_depth_grid()
        clearance, source = safety.path_clearance()
        veto_cm, veto_source = safety.forward_clearance()
        grid["path"] = {
            "indices": path_zone_indices(
                int(grid.get("rows", 1)), int(grid.get("cols", 0)),
                grid.get("fov_deg"), grid.get("pan_deg"),
            ),
            "clearance_cm": clearance,
            "source": source,
            # What the veto would say about a FORWARD issued right now --
            # the cone above AND the chassis' swept corridor (3.18), so this
            # can read blocked while the cone's own clearance reads clear:
            # something off a front corner. None is "nothing within range".
            "blocked": veto_cm is not None and veto_cm < min_distance,
            "veto_cm": veto_cm,
            "veto_source": veto_source,
            "min_distance_cm": min_distance,
            # Published for the same reason `clearance_cm` is: the strip
            # must draw what the veto reads. `clearance_cm` above is
            # already bumper-relative, so a page that re-added this would
            # subtract it twice -- it is here to be *shown* ("measured from
            # the bumper, sensor is Ncm back"), never to be applied.
            "sensor_to_bumper_cm": sensor_to_bumper,
        }
        return grid

    @app.get(prefix + "/world/truth", dependencies=[Depends(require_secret)])
    def world_truth():
        """Where the robot ACTUALLY is -- sim-only, phase R2.

        A pass-through like `/world/pose`. Every non-sim world answers
        `usable: False`, for ever: nothing in a real room knows the truth.
        Its only job is R5's error readout -- estimate against truth -- and
        no decision may read it.
        """
        return world_model.get_truth()

    # ---------- things that move, sim only (PLAN-ros-alignment.md 3.30) ----------

    def _sim_grid():
        """The simulated house under this robot, through any wrapper
        (`drive: ros`'s `inner`, the fake motor board's sim body). A robot
        with none answers 501: there is no furniture to move in a real room
        from here, and a silent no-op would read as done."""
        r = robot
        for _ in range(3):
            grid = getattr(r, "world", None)
            if grid is not None and hasattr(grid, "move_object"):
                return grid
            r = getattr(r, "inner", None)
            if r is None:
                break
        raise HTTPException(status_code=501, detail=(
            "this robot has no simulated house -- /sim/* routes are sim-only "
            "(PLAN-ros-alignment.md 3.30)"))

    @app.get(prefix + "/sim/objects", dependencies=[Depends(require_secret)])
    def sim_objects():
        """Every object in the simulated house, movers marked, and the sim
        clock. Ground truth, like `/world/truth`: no decision may read it."""
        return _sim_grid().describe_objects()

    class MoveObjectRequest(BaseModel):
        src: List[int]
        dst: List[int]

    @app.post(prefix + "/sim/objects/move", dependencies=[Depends(require_secret)])
    def sim_objects_move(req: MoveObjectRequest):
        """Move a piece of furniture, as someone rearranging the room would.
        Seen at once by the scan, collision and the discovered map."""
        grid = _sim_grid()
        if len(req.src) != 2 or len(req.dst) != 2:
            raise HTTPException(status_code=422, detail="src and dst are [x, y] cells")
        with motion_lock:
            try:
                grid.move_object(req.src, req.dst)
            except ValueError as exc:
                raise HTTPException(status_code=409, detail=str(exc))
        return sim_objects()

    class GoalRequest(BaseModel):
        x_m: float
        y_m: float

    def _goals():
        """R6: only a world that can plan takes goals (world/ros_world.py);
        every other answers `unsupported`, never a silent no-op."""
        if not hasattr(world_model, "set_goal"):
            raise HTTPException(status_code=501, detail=(
                "this world cannot take goals -- goals need nav2 (WORLD_MODE=ros, "
                "PLAN-ros-alignment.md R6)"))
        return world_model

    @app.post(prefix + "/world/goal", dependencies=[Depends(require_secret)])
    def world_goal_set(req: GoalRequest, x_driver: str = Header(default="")):
        """A place to go, in the house frame. nav2 plans and drives; its
        commands still pass collision_monitor and robot/safety.py (3.15).

        3.23: a goal is an AUTONOMOUS driver (`ros`), arbitrated like one --
        refused while the brain (or anyone who outranks it) holds the robot,
        so it never reaches nav2 to interleave with a mission.

        Handoff 3b (decided 2026-10-05): a goal that NAMES a person (the
        twin's tap sends `x-driver: twin-dpad`) is that person driving, ranked
        as one -- it takes the robot from the brain, whose next command is
        refused `preempted`, exactly as a D-pad press does. A goal naming no
        one keeps 3.23's rank: scripts and the nav suites post those."""
        world = _goals()
        now = time.monotonic()
        person = (x_driver or "").strip()
        if person and driver_priority(person) == DRIVER_MANUAL:
            refused = arbitrate(person, now)
            if refused:
                return {"accepted": False, **refused}
            state["driver"], state["driver_at"] = person, now
        else:
            refused = arbitrate(DRIVER_ROS, now)
            if refused:
                return {"accepted": False, **refused}
        # A person re-sending a goal after a stop: it supersedes the stop's
        # ending loop, which must never cancel this new goal.
        with goal_stop_lock:
            goal_stop["gen"] += 1
            goal_stop["hold"] = False
            try:
                return world.set_goal(req.x_m, req.y_m)
            except httpx.HTTPError as e:
                # Handoff 4d: refused by name, not a 500.
                return {"accepted": False, "reason": "ros_unavailable",
                        "detail": f"the ROS bridge did not take the goal: {e}"}

    def _bridge_call(fn):
        """A goal read or cancel against a bridge that is down answers 503
        `ros_unavailable`, not a 500 (handoff 4d)."""
        try:
            return fn()
        except httpx.HTTPError as e:
            raise HTTPException(status_code=503,
                                detail=f"ros_unavailable: the ROS bridge did not answer: {e}")

    @app.get(prefix + "/world/goal", dependencies=[Depends(require_secret)])
    def world_goal_get():
        return _bridge_call(_goals().get_goal)

    @app.delete(prefix + "/world/goal", dependencies=[Depends(require_secret)])
    def world_goal_cancel():
        return _bridge_call(_goals().cancel_goal)

    @app.get(prefix + "/world/error", dependencies=[Depends(require_secret)])
    def world_error():
        """The estimate against the truth -- R5's readout, sim-only.

        `position_error_m` / `heading_error_deg` compare `get_pose()` (SLAM's
        estimate under `world: ros`) with the simulator's truth; `odom_*` do
        the same for dead reckoning alone, where the world can say, so the
        readout shows what SLAM corrects. `usable: false` wherever there is
        no truth -- on hardware, always. Never an input to a decision.
        """
        truth = world_model.get_truth()
        pose = world_model.get_pose()
        if not truth.get("usable") or not pose.get("usable"):
            return {"usable": False, "source": None, "position_error_m": None,
                    "heading_error_deg": None, "odom_position_error_m": None,
                    "odom_heading_error_deg": None}

        def err(p):
            if not p.get("usable"):
                return None, None
            d = math.hypot(p["x_m"] - truth["x_m"], p["y_m"] - truth["y_m"])
            h = (p["heading_deg"] - truth["heading_deg"] + 180.0) % 360.0 - 180.0
            return round(d, 4), round(h, 3)

        pos, head = err(pose)
        odom = getattr(world_model, "get_odom_pose", None)
        opos, ohead = err(odom()) if odom else (None, None)
        return {"usable": True, "source": pose.get("map_id"),
                "position_error_m": pos, "heading_error_deg": head,
                "odom_position_error_m": opos, "odom_heading_error_deg": ohead,
                "truth": truth}

    @app.get(prefix + "/world/pose", dependencies=[Depends(require_secret)])
    def world_pose():
        """Where the robot is on the map -- phase N1.

        A PASS-THROUGH. This server computes nothing here; it returns what
        the world backend said, exactly as `/distance` returns what the
        body said. The decision not to give world state its own service
        and its own Settings URL is in `PLAN-mapping.md` section 4: the
        usual argument for splitting a service (the admin console's "no
        reason to go down when the mission server restarts") does not
        apply, because the map's durability is a storage decision (1.5's
        DynamoDB plus a working copy) rather than a process-topology one.

        **Not `/pose`**, and the prefix is not decoration: it is the seam.
        Every other route here is a body route, and a reader who has to
        ask which kind a route is has already lost the distinction this
        phase exists to draw.

        `RemoteRobot`'s rule applies to `RemoteWorld` too -- a 404 here is
        "this server predates the route", never a transport failure.
        """
        return world_model.get_pose()

    @app.get(prefix + "/world/map", dependencies=[Depends(require_secret)])
    def world_map():
        """The house, as discovered so far -- phase N1.

        Also a pass-through. Note the payload is the big one on this
        server: ~10^5 cells on a real house against a handful of numbers
        everywhere else. `map_version` is what lets a client avoid asking
        for it -- poll `/world/pose` freely, re-read this only when the
        version moves.
        """
        return world_model.get_map()

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
            # R4: how verbs reach the wheels, and the evidence for "one
            # writer" -- under `ros`, verbs sent through the chain and the
            # actuator's posts; a verb executed on the robot directly would
            # not appear in either.
            "drive": {"mode": "ros" if by_velocity else "direct",
                      "verbs_through_ros": getattr(robot, "verbs_through_ros", None),
                      "wheel_posts_from_ros": state["wheel_posts"] if by_velocity else None,
                      # 3.24 G3: whether ROS is alive, and so whether a person's
                      # verbs are running on the direct fallback.
                      "ros_up": ros_up(time.monotonic()) if by_velocity else None,
                      # Which half of ROS is down (spec review 3, V9): the
                      # bridge, or the actuator plugin's posts.
                      "bridge_up": (robot.bridge_up() if by_velocity
                                    and hasattr(robot, "bridge_up") else None),
                      "ros_post_age_s": (round(time.monotonic() - state["last_ros_post_at"], 3)
                                         if by_velocity and state["last_ros_post_at"] is not None
                                         else None)},
            "watchdog_timeout_s": watchdog_timeout,
            "wheel_loop": {**state["wheel_loop"], "period_s": WHEEL_LOOP_INTERVAL_S},
            # 3.34: the motor board's link -- frame age, frames, reboots --
            # for a body that has one; None otherwise. Description only
            # (control/health.py): a quiet board is a fact about the car, not
            # about the release.
            "motor_board": (robot.feedback_status()
                            if callable(getattr(robot, "feedback_status", None)) else None),
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
            # Which house the simulator built, so a live test can ask the
            # SERVER what it is standing in rather than trusting its own
            # environment -- the SLAM lap once ran its starter-house route
            # inside the user's house. Read off the world the factory built
            # (sim/maps/__init__.py names it), so it is right for every body
            # standing in a sim house -- `mode: hardware` with the fake motor
            # board included. It used to re-read SIM_MAP only when
            # `mode == "sim"`, so under the fake board it was None and the
            # live suites SKIPPED: 3.33's G4 would have "passed" on five runs
            # of skips (docs-review/SPEC-REVIEW.md, finding 2). None when no
            # sim house stands behind the robot.
            "sim_map": getattr(getattr(robot, "world", None), "map_name", None),
            # Phase M5. Which build answered, for anyone reading a health
            # report rather than a log. Same content as the start-up line.
            "identity": ident,
            # Phase M5. How long since the watchdog loop itself ran. A
            # verdict input for `python -m control.health`, and the only
            # field here that says anything about whether this server's own
            # guard is alive rather than about the robot.
            "seconds_since_watchdog_poll": round(
                time.monotonic() - state["watchdog_polled_at"], 2),
            "watchdog_poll_interval_s": WATCHDOG_POLL_INTERVAL_S,
            # Phase M4. Who last drove, and whether they still hold the
            # robot -- `holder` is None once authority has lapsed on
            # silence, which is a different statement from "nobody has ever
            # driven". The twin shows both beside the watchdog readout.
            "driver": state["driver"],
            "authority_holder": authority_holder(time.monotonic()),
            "refusal_counts": dict(state["refusal_counts"]),
            "last_refusal": (
                {**state["last_refusal"],
                 "seconds_ago": round(time.monotonic() - state["last_refusal"]["at"], 2)}
                if state["last_refusal"] else None
            ),
            # Which deployment this is, for the twin's environment banner.
            # Unset (production) means the banner never renders, so the
            # same image is safe everywhere -- an environment marks itself
            # rather than the page guessing from a hostname, which breaks
            # the moment a CloudFront domain changes. Empty string, not
            # null, so a client can treat it as a plain falsy string.
            "env_label": env_label,
        }

    return app


app = create_app(os.environ.get("ROBOT_CONFIG_PATH"))
