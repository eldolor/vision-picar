---
kind: engineering
domain: mission
status: current
verified: 2026-10-08
parent: docs/mission/ARCHITECTURE.md
---

# Mission -- engineering

How the brain service is built today. The what and why -- two processes,
three separate failsafes, the halt gate, the outcome set -- are in the
[architecture spec](../../mission/ARCHITECTURE.md). This document is true
only until the code changes, and is updated in the same commit that changes
it. `AGENT-HARNESS.md` is the long-form walkthrough of the tick and its
threads; where it and this file disagree, check the code.

## Implementation

| File | What it does |
|---|---|
| `control/brain_server.py` | FastAPI app (`create_app()`, module-level `app`). One mission slot in a `state` dict, the asyncio `mission_loop()` that runs `runner.tick` on a worker thread under B3.3's deadline, start-time validation, the policy wiring (`vision_fn_for`, `_tiered_vision_fn`), and the health route. |
| `control/mission_runner.py` | `MissionRunner` (lifecycle, outcomes, status, B3.2), `_HaltGate` (a `RobotInterface` wrapper), `call_with_timeout()` (daemon-thread timeout), the outcome constants and `POLICIES`. |
| `control/brain_config.py` | `load_brain_config()`: reads only the `brain:` block of `config/robot.yaml`, merges it over `DEFAULTS`, applies env overrides, and raises `ValueError` on an unknown key. Deliberately does not import `robot/factory.py`. |
| `control/drills.py` | `FAULTS`, `apply()` (folds a fault into the runner kwargs), `TickHangRunner`, the stand-in vision functions. |
| `control/remote_robot.py` | `RemoteRobot`, the body client; sends `x-driver: brain` (`DEFAULT_DRIVER`). Owned by the body domain; listed because the brain builds it. |
| `control/remote_world.py` | `RemoteWorld`, the world client. Owned by the world domain. |

**Agent selection.** `MissionRunner` builds `VisionAgent` for `policy` in
`VISION_POLICIES` (`"vision"`, `"tiered"`) and `ObjectSearchAgent` for
`"frontier"` (both in [policy](../policy/ENGINEERING.md)). With no
`vision_fn`, the frontier agent's own `sensed_scene()` is used.

**Threads** (`AGENT-HARNESS.md` section 7):

| Thread | Runs |
|---|---|
| asyncio event loop | every HTTP handler and `mission_loop()` |
| worker (`asyncio.to_thread`) | one `runner.tick()`; also `make_runner` at start (it may make a blocking HTTP call) |
| daemon `vision-call` thread | one `vision_fn` call under `call_with_timeout()`; abandoned, never killed, on timeout. A tick that judges an arrival starts a second one for `confirm_arrival` |
| `tiered-cloud` worker | the tiered policy's async cloud call (policy domain); shut down by `runner._finish()` via `vision_fn.close()` |

`MissionRunner` guards state with an `RLock`, never held across
`robot.stop()`.

**What one `tick()` does, in order** (`control/mission_runner.py`):

1. On the first tick only: `self._gated.look_center()` (3.20), not counted.
2. `self.agent.step()`; the agent's vision call goes through
   `_guarded_vision()`, which pushes `searched_rooms` into the vision
   function if it has `set_searched_rooms`, attaches
   `frame["odometry"] = robot.get_odometry()` (failure ignored), and wraps the
   call in `call_with_timeout(..., vision_timeout_s)`, converting any error to
   `VisionUnavailable`. On an arrival the agent makes a second guarded call,
   `_guarded_confirm()` -- the policy's `confirm_arrival(frame)` under the same
   timeout and error mapping, so it counts against the same B3.2 budget
   ([policy engineering](../policy/ENGINEERING.md), "Arrival"). A policy with
   no `confirm_arrival` never confirms, and its arrivals never end `found`.
3. Exceptions: `MissionHalted` -> return False (stop landed mid-tick);
   `VisionUnavailable` -> `_handle_vision_failure()` (on the budget's last
   failure: `arrived_unconfirmed` if `agent.unconfirmed_arrival` is set, the
   readout copied to `status.arrival` with state `unconfirmed`, else
   `failed`); `Preempted` -> finish
   `preempted`; anything else -> finish `failed` ("step failed: ...").
4. Under the lock: bump `ticks`, reset `vision_failures`, record the action,
   the turn counters, and copy `_tier`, `_perception` and `_arrival` off the
   scene (held, not overwritten with None); `last_frame_seq` is read from
   `result.frame`'s `metadata.seq`, not from the scene.
5. Count consecutive refused FORWARDs (reset by any executed move).
6. Outside the lock: `found`/`room_reached` if `memory.is_complete()`;
   `blocked` if the refused count reaches `stuck_after`; `max_steps` if
   `len(agent.history) >= max_steps`.

### How a tiered mission is built

This is the canonical description. The policy and perception specs link
here. The chooser between the sim stand-in and real models is
`control/brain_server.py` (`_tiered_vision_fn()`), not `brain/perceive.py`,
which only supplies both pipelines. `POST /mission/start` with `policy` `vision` or `tiered`, in
`control/brain_server.py`:

1. Refuse with 400 if `vision_url` is empty or there is no `target_object`.
2. Resolve `model_id` and `prompt_variant` (request, then config, then the
   service default). Check them once against `GET {vision_url}/navigate/models`
   (`_validate_navigate_choices()`). An unknown value is a 400. An unreachable
   endpoint is logged and tolerated.
3. Build the cloud step: `vision_fn_for(target_object, vision_url=...,
   secret=VISION_SHARED_SECRET, timeout_s=vision_timeout_s, model_id=...,
   prompt_variant=...)`. For `vision`, that is the whole `vision_fn`.
4. For `tiered`, call `_tiered_vision_fn(target, vision_fn, config,
   simulated=_frames_are_simulated(robot))`:
   - **The sim probe.** `_frames_are_simulated()` calls
     `robot.get_camera_frame()` once, at start, before the first tick. It is
     true only if that probe frame's `metadata.source == "sim"`, and any
     error answers false (assume a real camera).
   - **Simulated.** It wraps the cloud step in
     `tiered_vision_fn_for(..., pipeline=FrameReportedPipeline(target))`.
     No model is loaded.
   - **Real camera.** It builds `pipeline_for(target, **overrides)`. A
     non-empty or non-zero `perception_*` key overrides the module default.
     `PerceptionUnavailable` becomes a 400 `The tiered policy cannot start:
     ...`, and any other exception from `pipeline_for()` becomes a 400 `The
     tiered policy could not load its perception models: ...` -- including a
     construction error that has nothing to do with models, such as a bad
     `perception_crop_path`. The pipeline is then wrapped the
     same way.
   - The `tier_*` keys become `TieredVision` keyword arguments. A 0
     `tier_cold_search_after_cm` or `tier_max_calls` becomes `None` (off).
5. `drills.apply(fault, kwargs, config)`, then `MissionRunner(**kwargs)` on
   a worker thread. An unknown `policy` is refused there (`ValueError` ->
   400).

The heavy imports happen in step 4, on the start path, so a missing install
is a refusal and never a B3.2 vision failure three ticks in.

### Terminal paths

Every terminal path is `_finish(outcome, note)`: set not-running (closing the
gate), record the outcome (first one wins), `_safe_stop()` (logs, never
raises), `vision_fn.close()` if present, then ship one metrics row if
`metrics_url` is set.

## Interfaces

### HTTP routes (`control/brain_server.py`)

Every route is prefixed with `ROUTE_PREFIX` (empty by default). All but
`/health` require `x-app-secret` when `APP_SHARED_SECRET` is set (401
otherwise). CORS allows any origin.

| Method, path | Body | Success | Errors |
|---|---|---|---|
| `POST /mission/start` | `MissionStartRequest` (below) | `{"started": true, "status": {...}}` | 409 `"A mission is already running."`; 400 config or validation error with its message; 403 drill on a brain with drills off; 422 an unknown field or a bad type |
| `POST /mission/stop` | none | `{"stopped": true, "status": {...}}` -- cancels the loop task, then `runner.stop()`; with no mission ever run, stops the robot anyway and returns the idle status | -- |
| `GET /mission/status` | none | `MissionRunner.status()` plus `fault`; before any mission `{"running": false, "outcome": "idle", "step": 0, "log_tail": [], "fault": "none"}` | -- |
| `GET /mission/inventory` | none | `{"inventory": null}` before any mission; else the current or last mission's object inventory (3.46; shape in the [perception engineering spec](../perception/ENGINEERING.md), "Inventory"). On mission end the report is also saved by `control/inventory_store.py` on a daemon thread | -- |
| `GET /health` | none | liveness and configuration description; the verdict rules are the operations domain's | -- |
| `POST /recording/frame`, `POST /recording/finish` | mounted from `control/recording_routes.py` | owned by the recordings domain | -- |

**`MissionStartRequest`** (`extra="forbid"`):

| Field | Type, default | Notes |
|---|---|---|
| `target_object` | str, None | required for `vision`/`tiered` |
| `target_room` | str, None | `frontier` only in practice; one of the two targets is required (`ValueError` -> 400) |
| `mission` | str, None | free text; defaulted from the targets |
| `max_steps` | int, None | falls back to config |
| `policy` | str, `"frontier"` | a plain `str` in the model, not a `Literal`. A value outside `POLICIES` (`frontier`, `vision`, `tiered`) passes validation and is refused by `MissionRunner` with `Unknown policy: ...`, which is a **400**, not a 422 |
| `model_id`, `prompt_variant` | str, None | request, then config, then the vision service's default; checked once against `GET {vision_url}/navigate/models` |
| `fault` | str, `"none"` | a plain `str`. `drills.apply()` refuses a value outside `drills.FAULTS` (`none`, `vision_error`, `vision_hang`, `tick_hang`) with `Unknown fault: ...`, which is a **400**. It does so before the drills-off check, so an unknown fault is a 400 even when drills are off |

### `MissionRunner.status()` fields

`running`, `outcome`, `error`, `policy`, `mission`, `target_object`,
`target_room`, `step`, `max_steps`, `found`, `room_reached`, `complete`,
`last_action`, `turns` (`count`, `reversals`, `last_turn_deg`, `share`,
`spinning`), `last_reasoning`, `rooms_visited`, `rooms_searched`,
`vision_failures`, `tier`, `perception`, `arrival`, `last_frame_seq`,
`ticks`, `seconds_since_last_tick`, `tick_rate_hz`, `sighting`, `log_tail`
(last `LOG_TAIL_LINES` = 20). `tier`/`perception` are null under any policy
without a perception tier. `AGENT-HARNESS.md` section 8 describes each.

**Outcome strings:** `idle`, `running`, `found`, `room_reached`, `stopped`,
`max_steps`, `blocked`, `preempted`, `failed`.

### In-process

- `MissionRunner(robot, target_object=None, target_room=None, mission=None, max_steps=120, min_distance_cm=20.0, vision_proximity_veto=False, policy="frontier", vision_fn=None, vision_timeout_s=20.0, max_vision_failures=3, world=None, stuck_after=5, ..., inventory=True, detections_fn=None)` (3.46: the object inventory, recorded and reported only; `runner.inventory_sink` is set by the brain server like the metrics fields);
  `start()`, `tick() -> bool`, `stop(reason)`, `abort(reason)`,
  `is_running()`, `status()`. `start()` on a used runner raises.
- `vision_fn(frame: dict) -> dict` -- the scene schema (policy domain).
  Optional attributes the runner uses if present: `set_searched_rooms(list)`,
  `close()`, and `confirm_arrival(frame)` (handoff 1a; called through
  `_guarded_confirm()` only when the agent judges an arrival).
- `_HaltGate.MOVEMENT`: `drive_forward`, `reverse`, `turn_left`,
  `turn_right`, `look_left`, `look_right`, `look_center`,
  `set_wheel_velocity`; plus `verb_plan()` (guarded) and `verb_done()` /
  `stop_count` (forwarded) since 3.32. Raises `MissionHalted`.

## Parameters and configuration

The `brain:` block of `config/robot.yaml`, read by
`control/brain_config.py`. "Shipped" is the yaml value; "code" is
`DEFAULTS` (what a config without the key gets).

| Key | Shipped | Code | Unit | Read by | Why |
|---|---|---|---|---|---|
| `robot_url` | `http://127.0.0.1:8000` | same | URL | `default_robot_factory` | the one-line brain relocation |
| `world_url` | (unset) | `""` | URL | `default_world_factory` | empty = the URL of the robot actually built; diverges if the world moves behind SLAM |
| `vision_url` | `""` | `""` | URL | start validation, `vision_fn_for` | required for `vision`/`tiered` |
| `navigate_model_id`, `navigate_prompt_variant` | (unset) | `""` | -- | start | headless default; never guessed |
| `max_steps` | 120 | 120 | steps | runner | set when the reference frontier hunt took 83 steps (61 since 2026-09-28); every vision step is a paid call, so lower it for those policies |
| `stuck_after` | (unset) | 5 | refused FORWARDs | runner | an unlucky policy gets 1-2 refusals then turns; the first watched R1 run spent 19 on one jamb |
| `min_distance_cm` | 20.0 | **30.0** | cm | agent's brain-side pre-check | 20 matches `safety.min_distance_cm`; 30 is one sim cell and vetoed ~half of legal moves with sensor noise on. See Known gaps |
| `request_timeout_s` | 10.0 | 10.0 | s | robot and world clients, allow-list check | |
| `vision_timeout_s` | 20.0 | 20.0 | s | B3.2 per-call timeout | |
| `max_vision_failures` | 3 | 3 | calls | B3.2 budget | B3 plan's suggestion; three abandoned threads at most |
| `tick_timeout_s` | 30.0 | 30.0 | s | B3.3 dead-man | must stay above `teleop.stall_timeout_s` (15 s) so the specific teleop error fires first. **Not** above two `vision_timeout_s` (40 s): under a SYNCHRONOUS tier a tick that judges an arrival holds the vision call and the confirmation, each up to 20 s, so a slow pair is ended by B3.3 as "loop hung" rather than counted by B3.2. The shipped asynchronous tier has only the confirmation in the tick (review 3, fix 18; left as is, recorded) |
| `tick_interval_s` | 0.25 | 0.0 | s | `mission_loop` | pacing so a sim mission is watchable; 0 in code so tests stay fast; can be 0 on hardware |
| `allow_drills` | true | true | -- | `drills.apply` | off for any brain reachable beyond the LAN |
| `drill_vision_timeout_s` | 2.0 | 2.0 | s | vision drills | a demo in seconds, same guard |
| `drill_tick_timeout_s` | 3.0 | 3.0 | s | `tick_hang` drill | the hang lasts 3x this (`drills.OVERSHOOT`) |
| `metrics_url`, `metrics_secret` | `""` | `""` | -- | `_ship_metrics` | empty disables shipping |

The `tier_*` and `perception_*` keys are passed straight through to the
policy and perception pipeline; see [policy](../policy/ENGINEERING.md) and
[perception](../perception/ENGINEERING.md). Recording keys belong to the
recordings domain.

**Environment variables:** `ROBOT_URL`, `VISION_URL`, `NAVIGATE_MODEL_ID`,
`NAVIGATE_PROMPT_VARIANT`, `METRICS_URL`, `METRICS_SECRET` override the yaml
(`load_brain_config`). `APP_SHARED_SECRET` gates callers of the brain;
`ROBOT_SHARED_SECRET` and `VISION_SHARED_SECRET` gate the brain's own
outbound calls and default to `APP_SHARED_SECRET`; `WALKS_SHARED_SECRET` is
the metrics fallback secret; `ROUTE_PREFIX` prefixes every route (the tunnel
uses `/brain`).

**Constants:** `LOG_TAIL_LINES` 20; `SPIN_MIN_STEPS` 10,
`SPIN_TURN_SHARE` 0.75, `SPIN_MAX_REVERSAL_SHARE` 0.1 (the `turns.spinning`
readout only -- the first watched R1 run was 98 turns in 120 steps with 0
reversals); `drills.OVERSHOOT` 3.0.

## Procedures

**Run both halves locally:**

```bash
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001
```

The brain logs an identity line first. A healthy start looks like:

```text
vision-picar brain server starting -- git=<short sha> executable=<path>/.venv/bin/python config=(default)
```

Then `curl -s localhost:8001/health` answers immediately, with no robot
call, and begins:

```text
{"status": "ok", "identity": {"service": "vision-picar brain server", "git_revision": "<short sha>", ...}, "robot_url": "http://127.0.0.1:8000", "mission_running": false, "seconds_since_last_tick": null, ...
```

`perception_available` is `true` only with `requirements-perception.txt`
installed. If another session owns 8000/8001, run the pair on other ports and
set `ROBOT_URL=http://127.0.0.1:<robot port>` for the brain.

**Start, watch, stop a mission:**

```bash
curl -s -X POST localhost:8001/mission/start -H 'content-type: application/json' \
     -d '{"target_object": "red backpack"}'          # {"started": true, "status": {"outcome": "running", ...}}
curl -s localhost:8001/mission/status                 # step, last_action, log_tail advancing
curl -s -X POST localhost:8001/mission/stop           # {"stopped": true, "status": {"outcome": "stopped", ...}}
```

A frontier mission in the starter house ends `found` in 61 steps (the trace
pinned in `tests/data/frontier_trace_centred.json`, re-pinned 2026-09-28 when
the Rover chassis and guarded verbs changed it from 83), about 15 s at
`tick_interval_s` 0.25. A vision or tiered mission needs
`VISION_URL` set and spends money on every cloud call.

**Run a failsafe drill** (add `"fault"` to the start body):

| Fault | Expected | Log line |
|---|---|---|
| `vision_error` | no step completes; `failed` on the third failed call | `vision failure 1/3: vision call failed: drill: ...` ... `mission ended (failed): vision unavailable 3 times in a row` |
| `vision_hang` | each call times out at 2 s; `failed` after the third | `vision timed out after 2.0s` |
| `tick_hang` | one step, then `failed` after 3 s | `brain loop hung: tick exceeded 3.0s` |

Every drill must end with the robot stopped. On a brain with drills off, a
fault is a 403 naming `brain.allow_drills`.

**Failure signatures at start:**

| Response | Cause |
|---|---|
| 400 `The {policy} policy needs a vision service. Set brain.vision_url ...` (`vision` or `tiered`) | `vision_url` empty |
| 400 `... searches for an object -- /navigate takes a target_object` | room-only mission under a cloud policy |
| 400 `model_id 'x' is not offered by the vision service. Available: ...` | bad model; same shape for `prompt_variant` |
| 400 `The tiered policy cannot start: ... pip install -r requirements-perception.txt` | perception extras missing (perception domain) |
| 400 `Unknown policy: 'x'. Known: frontier, vision, tiered` | bad `policy` value (a 400, not a 422) |
| 400 `Unknown fault: 'x'. Known drills: none, vision_error, vision_hang, tick_hang` | bad `fault` value |
| 400 `The tiered policy could not load its perception models: ...` | any other error building the pipeline: weights failed to load or download, or a bad `perception_*` value (e.g. an unknown `perception_crop_path`) |
| 422 naming `extra_forbidden` | an unknown field in the start body |
| 422 naming the type error, e.g. `int_parsing` (`max_steps: "abc"`) or `int_type` (`max_steps: [1]`) | a wrong type |
| `ValueError: Unknown keys in config brain block` at boot | a typo in `config/robot.yaml`'s `brain:` block |

**Demonstrate the HTTP boundary is invisible:** `python -m
tests.demo_brain_over_http` runs the same mission three ways (in process,
ASGI, a live socket) and prints the same step count for each.

## Verification

Run `pytest tests/test_mission_runner.py tests/test_brain_server.py
tests/test_failsafes.py tests/test_camera_centred_start.py
tests/test_mission_guarded_verbs.py -q`.

| Test file (count on 2026-10-02) | What it pins |
|---|---|
| `tests/test_mission_runner.py` (32) | tick-driven equals blocking `run_mission()` step for step; outcomes and idempotent `_finish`; the gate refuses after stop; a stop landing mid-tick records no step; tiered readouts reach status and survive a scene with no tier; the tick keeps advancing while an async call is in flight |
| `tests/test_brain_server.py` (60) | **the import boundary, by mechanism:** a subprocess imports the brain app and lists `sys.modules`, so a forbidden import fails the suite and does not depend on review; two-hop mission completes; 409 on a second start; stop with no mission stops the car; secret gating; `ROUTE_PREFIX`; **the brain process loads no `sim`, and from `robot`/`world` only `robot.interface`, `robot.safety`, `robot.identity`, `world.interface`**; model and wording validation (refused at start, outage tolerated, one round trip); unknown field is 422; tiered start refusals and sim-vs-real perception choice |
| `tests/test_failsafes.py` (15) | B3.2 budget and hang; one failure survivable; no movement after stop; B3.3 dead-man; every drill ends `failed` with the robot stopped; no-fault builds the ordinary mission; drills can be switched off |
| `tests/test_camera_centred_start.py` (11) | 3.20's four criteria: centred first decision under `frontier` and `vision` (tiered is not parametrised), no step or vision call spent, centred start unchanged against `tests/data/frontier_trace_centred.json`, a refused centring ends the mission like any refused move |
| `tests/test_mission_guarded_verbs.py` (3) | 3.32: a mission's verbs go through the safety layer's guarded plan; the gate forwards plan and stop count; a finished mission refuses a plan |
| `tests/test_bearing_turns.py` | `blocked` after `stuck_after` refusals, never on an arrival, switchable off, reset by progress |
| `tests/test_robot_contract.py` | `_HaltGate` passes the body conformance suite as a backend |
| `tests/test_remote_robot.py` | an identical action sequence and step count in-process, over ASGI and over a live socket |

**Checklist for a change here:** a new movement method on `RobotInterface`
must be added to `_HaltGate.MOVEMENT`; a new sensing method must be passed
through explicitly (inheriting the interface's default would make every
mission report the sensor missing); a new terminal path must go through
`_finish()`; a new `brain:` key goes into `DEFAULTS` in the same commit as
the yaml, or the walks Lambda fails at cold start.

## Known gaps

- **`min_distance_cm` code default is 30.0** (`control/brain_config.py`
  `DEFAULTS`) while `config/robot.yaml` and
  `DEFAULT_MIN_DISTANCE_CM` in `control/mission_runner.py` say 20.0, and the
  yaml's own comment says "NOT 30.0". Any config file without the key gets 30.
- **The cloud-unreachable degraded mode** (`PLAN-onboard-perception.md` 2.5)
  is not built: a blown vision budget ends the mission `failed`.
- **No mission persistence** across a brain restart, one mission per brain,
  `MissionMemory` per mission (`AGENT-HARNESS.md` section 12).
- **No boot units (B5)** on the Jetson.
- **S7 (chaos and soak)** not run.
