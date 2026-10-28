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
| `control/reconfirm.py` | 3.53: `Reconfirmer` (one late identity check for one finished runner, on a daemon thread: free `GET <vision_url>/health` probes, then ONE paid `runner.late_ask()`; bounded by `reconfirm_window_s` and two paid attempts; `cancel(reason)` records `dropped`) and `health_probe()`. 3.56: `CloudWatch` (`status.cloud`: `unknown` / `reachable` / `unreachable` since a time, from the same free probe, only while a cloud-policy mission runs). Never touches the robot. |
| `control/brain_config.py` | `load_brain_config()`: reads only the `brain:` block of `config/robot.yaml`, merges it over `DEFAULTS`, applies env overrides, and raises `ValueError` on an unknown key. Deliberately does not import `robot/factory.py`. |
| `control/drills.py` | `FAULTS`, `apply()` (folds a fault into the runner kwargs), `TickHangRunner` (its own tick count, `_drill_ticks`: sharing the runner's `_ticks` skipped the first tick's camera centring and counted every tick twice, 3.76 N1), the stand-in vision functions. |
| `control/remote_robot.py` | `RemoteRobot`, the body client; sends `x-driver: brain` (`DEFAULT_DRIVER`). Owned by the body domain; listed because the brain builds it. |
| `control/remote_world.py` | `RemoteWorld`, the world client. Owned by the world domain. |

**Agent selection.** `MissionRunner` builds `VisionAgent` for `policy` in
`VISION_POLICIES` (`"vision"`, `"tiered"`) and `ObjectSearchAgent` for
`"frontier"` (both in [policy](../policy/ENGINEERING.md)); `ExploreAgent`
for `"explore"`, and for `"tiered"` with `tier_search="explore"` (3.80.G,
`TIER_SEARCHES`; `self.explores` then also gates the navigator wrapper, its
end at `_finish`, and the status's `explore` block). With no
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
   error mapping, so it counts against the same B3.2 budget
   ([policy engineering](../policy/ENGINEERING.md), "Arrival"). A policy with
   no `confirm_arrival` never confirms, and its arrivals never end `found`.
   Since 3.56 the confirmation has its own deadline,
   `confirm_timeout_s()` = min(`arrival_confirm_timeout_s` (8 s),
   `vision_timeout_s`); the brain builds the tier's `confirm_vision_fn` with
   that HTTP timeout, so a hang ends in httpx as `CloudUnavailable`. The
   runner's guard, `confirm_guard_s()`, sits a quarter of the deadline (at
   most 2 s) above it as a backstop; `call_with_timeout(on_start=...)` hands
   back the thread. The whole step shares ONE deadline, `vision_timeout_s`
   from its start, as before 3.56, so the arrival tick stays inside B3.3's
   tick deadline: first the policy's `wait_inflight()` (if it has one)
   drains an async trigger call still out, so a slow but healthy trigger is
   never charged to the confirmation's own clock (with the call cap spent it
   returns at once and `confirm_arrival()` refuses locally); then, if the
   last confirmation's thread is still alive, the next waits for it up to
   one guard, and still out starts no call and fails from
   `CloudUnavailable`; then, if the runner has stopped during those waits,
   a refusal with no call (on every path since 3.76 K2: only a local-led
   arrival re-read it, so a Stop during them bought one); then the call,
   its guard cut to what is left. Each
   wait draws on what is left, and none is ever passed a zero timeout
   (which `call_with_timeout` would run unbounded). `late_ask()` (3.53)
   honours the same thread: it waits up to `vision_timeout_s`, then raises
   `TimeoutError` with nothing sent, and registers its own call's thread.
   **3.80.C:** when the policy has `confirm_local` and `confirm_cloud` and a
   local tier is configured (`local_identity_fn`; without one, nothing here
   changes),
   `_guarded_confirm()` first calls `confirm_local(frame)`, before any
   `wait_inflight()` (a local answer needs no cloud call to land), with
   what is left of the step minus `confirm_guard_s()`, or half of what is
   left if that is more (a drill's short deadline). A verdict -- confirmed,
   refused or undecided -- is returned as is; a local call that outlives its
   share is the policy's `abandon_local()` verdict (undecided), never a
   cloud failure. None (no local tier, absent, or its tries spent) goes on
   to the steps above with `confirm_cloud` as the confirmation -- unless the
   runner has stopped meanwhile, which returns an undecided verdict with no call (3.76 L2: it read as a refusal). `late_ask()`
   is unchanged: the policy is closed by then, so it asks the cloud.
3. Exceptions: `MissionHalted` -> return False (stop landed mid-tick);
   `VisionUnavailable` -> `_handle_vision_failure()`. An arrival
   confirmation that raised carries that tick's readout as
   `arrival_readout` (set by the agent); the runner holds it in
   `_failed_arrival` for the current run of failures, cleared with the count
   by a tick that succeeds. On the budget's last failure the mission ends
   `arrived_unconfirmed` if `_failed_arrival` is set AND that last failure
   has `CloudUnavailable` (`brain/navigate.py`: a transport error, a 5xx, a
   408 or a 429, raised where the HTTP call is made; not a URL with no or a
   wrong scheme, `httpx.UnsupportedProtocol`, which is our configuration and
   a plain `RuntimeError` since 3.76) in its chain of
   explicit causes (`_caused_by()`, `__cause__` only). A readout is kept
   only when its confirmation itself failed that way; a confirmation that
   times out is attributed to the cloud, a trigger call that times out is
   not. The "vision failure" line is logged under the lock. `_finish()` then sets
   `status.arrival` to the readout with state `unconfirmed`, and refreshes
   `_tier.stats` from the policy's own counters (`_policy_stats()`, read
   outside the lock, never raising), in the same locked step as the outcome.
   A failure that lands after `stop()`/`abort()` returns at once: not
   counted, not logged, nothing written. Otherwise `failed`; `Preempted` ->
   `_finish_preempted()` (3.70 E5): `cancelled` with `cancelled_by: "dpad"`;
   a `StoppedByPerson` (the robot server refused the brain `person_stop`: a
   person pressed Stop, the D-pad's included; 3.84) ends `cancelled` with
   `cancelled_by: "stop"` and `end_reason` `stopped by a person: <detail>`;
   otherwise `dpad`,
   unless the refusal names an autonomous holder (`"<holder> is driving"`,
   or `"<holder> took over"` for a verb cut off under `drive: ros`, 3.76,
   holder `brain` or `ros`) -- then `failed`, an arbitration fault. A goal in
   progress (the phone's tap-to-goal) and a refusal naming no holder stay
   `cancelled`; anything else -> finish `failed` ("step failed: ...").
   Each person's end tells `_finish` how it came (`via`: `robot_stop` for
   `StoppedByPerson`, `preempted` with the holder as `by`; `stop()` passes
   `brain_stop`, or `loop_cancelled` when the brain server set
   `runner.loop_cancelled` because its loop was cancelled without a
   person's Stop -- no intervention), for 3.99's intervention line.
4. Under the lock: bump `ticks`, reset `vision_failures`, record the action,
   the turn counters, and copy `_tier`, `_perception` and `_arrival` off the
   scene (held, not overwritten with None); `last_frame_seq` is read from
   `result.frame`'s `metadata.seq`, not from the scene.
5. Count consecutive refused FORWARDs (reset by any executed move).
6. Outside the lock: `found`/`room_reached` if `memory.is_complete()`,
   after `_hear_a_person()` (3.104): one `look_center()` through the halt
   gate under `call_with_timeout(PERSON_PROBE_S)` (2 s), because the
   arrival's confirmation sends no verb and a person's Stop, or a take-over
   still held, in that window is otherwise never heard. A person's refusal
   (`StoppedByPerson`, a `Preempted` naming a manual holder or none, or a
   goal in progress -- `GOAL_IN_PROGRESS` -- when this mission never had
   one accepted or lost its answer, `_GatedNavigator.sent_goal`: the phone's tap-to-goal) ends
   through `_finish_preempted()`; a veto, `MissionHalted`, a goal refusal
   (`a nav2 goal is pending/active`) when it did -- a refused send does not
   count -- an autonomous holder
   (`ros`'s lapsing authority included; logged) or any other error (logged)
   keeps the found;
   a survey's room labeller looks (below); `exhausted` if the explore agent
   set `exhausted`; `blocked` if the
   refused count reaches `stuck_after`; then the budget check again; then
   `out_of_budget` if `len(agent.history) >= max_steps` (the step backstop).

**The budget (3.70 E4).** `_out_of_budget()` runs at the top of every tick
(before the cooldown wait and before any step, so a tick that starts past
the limit moves nothing) and again at the end. It ends the mission
`out_of_budget` when `self.clock() - start >= mission_time_limit_s` (the
mission's clock: sim time in process, `time.monotonic()` live), or when the
battery reads under `battery_floor_v` on `BATTERY_LOW_READINGS` (2)
consecutive low samples, read `BATTERY_CHECK_S` (0.5 s) apart on that clock.
A low read counts as a new sample only when its value differs from the last
one counted or `BATTERY_SAMPLE_S` (12 s) has passed since it (3.76 M2.7: the
Rover's board refreshes its reading every 10 s and repeats it between, so a
repeat inside that is the same sag read twice); any read at or over the
floor resets the count. A moving reading (the fake board's drain) still
ends inside 2 s; a value that holds ends 12-12.5 s after its first read.
`robot.get_battery()` with `usable: false`, or a read that raises, never
ends a mission (logged once). `max_steps` defaults to
`derived_max_steps(mission_time_limit_s)` = limit / 0.5 s x 2 (120 with no
limit); an explicit `max_steps` overrides it. Under `policy: "vision"` only,
the check also ends it when `_cloud_calls` reaches `vision_max_calls`: one
paid `/navigate` call per step, counted in `_guarded_vision` per call sent,
failed ones included (a time limit is not a call limit; `tiered` has
`tier_max_calls`).

**The report (3.70 E3-E5).** For `exhausted`, `out_of_budget` and
`cancelled`, `_finish()` starts a daemon thread after `_safe_stop()` that
builds `status.report` (it reads the map, an HTTP call up to the world's
timeout when remote, and walks it) and then sets `runner.report_done` (set at
once for other outcomes). Never inline: the brain's loop calls `stop()` on
its event loop when a Stop cancels it. `POST /mission/stop` waits up to 2 s
for `report_done` so its answer usually carries it. The report holds:
`rooms_entered` (the frames' rooms plus the explore agent's labelled rooms),
`goals_reached` (`[x_m, y_m, kind]`), `coverage` and `mapped_free_m2`
(`brain/mission_report.py map_coverage()`: known free floor over free plus
walled-in unknown patches that border free floor -- an estimate on the map
alone, within 2 points of truth once a search is done and 20-77 points HIGH
part way through, so the twin shows `mapped_free_m2` instead), `map_id` and `map_version` (the map the mission ran on; no saved
version until 3.50). So a status read in the moment between the stop and
the report has `report: null`; the twin reads once more.

**A survey (3.70 S1-S2).** `job="survey"` (default `"find"`; `JOBS`) is a
mission with no target under `policy="explore"` only: `MissionRunner`
refuses a survey with a target or another policy, and an unknown job, with
`ValueError`. The explore agent runs unchanged -- with no target it never
sights one, so it never approaches, never asks the arrival confirmation and
never ends `found`; it ends `exhausted` (or out of budget, or cancelled).
Its budget is `survey_time_limit_s` (brain_server picks it for a survey).
A find mission owns one too when `find_room_labels` (3.116.B: `off` |
`frame` | `cloud`, `find_room_label_max_calls` 20) is on and explore
chooses its goals under `explore_goal_policy: rooms` (anything else is
refused: the config, the runner; brain_server builds none for another
search). It asks exactly as a survey does, but on the map coarsened to the
room policy's 0.30 m (`goal_policy.coarse`, made once per map version and
not at all once asking has ended), and its layer goes only to the agent's
`goal_labels` as (the coarse map it was cut on, the layer), looked up by
metres; it is cut again whenever the map or the answers change (an older
cut would leave a frontier goal on new floor untyped). An ask holds the tick as a
survey's does (3.116 amendment 5: measured on the car before `cloud`);
`room_labels()` stays None and status reports the asks as `room_labels`,
not `survey`.
A survey owns a `RoomLabeller` (`brain/room_labels.py`): once a tick, after
the step, the runner reads the pose (`world.get_pose()`) and the explore
agent's last map and calls `observe(map, pose, robot.get_camera_frame)`. It
asks `room_label_fn(frame)` -- a frame taken THEN, at that pose, never the
step's own frame, taken before its move -- only when the robot stands in
space no earlier ask covers, at most `room_label_max_calls` times; reaching
the cap sets `capped` and leaves the rest unlabelled (the survey goes on).
Coverage is by `room_label_mode`. Both start from a cut of the map into
rooms by its shape (a watershed of clearance split at necks under
`NECK_RATIO` 0.75 of the lower peak: doorways). `spacing`, the default --
whenever the robot is more than `SPACING_M` (2.0 m) from every earlier ask,
by walking distance over the cut where each step from one of its rooms into
another costs `DOOR_M` (3.0 m) more; each cell takes the answer of the ask
nearest it by that distance, so a doorway gets a fresh ask and labels stop
there, and an open plan is asked every 2 m with labels meeting halfway. An
answer labels at most `REACH_M` (6.0 m: its room and one doorway on) of
that distance, an `unclear` answer holds its space and names nothing, and a
seed whose cell is no longer floor starts from the nearest floor within
`SNAP_M` (0.3 m).
`rooms` -- once per room of the cut, asked only from wider than
`NECK_RATIO` inside the room (fewer calls; an open plan is one room).
The default label source is `frame_room()`: the frame's `room`, which only a
simulated frame carries. A frame with no `room` key (a real camera) is
`NO_SOURCE`: nothing counted, and the labeller stops asking -- and fetching
frames -- for the rest of the mission (`status.survey.no_source`). With
`room_label_source: cloud` brain_server passes `room_label_fn_for()`
(`brain/navigate.py`): each ask is one call to the vision service's
`/room` (a name or `unclear`; "unknown" reads as `unclear`), its HTTP
timeout `room_label_timeout_s` held to half of `tick_timeout_s` (the ask
runs inside the tick), bounded on the wall clock: the call runs on its own
thread and the tick waits that long at most (httpx's timeouts are per
phase and per gap between bytes); an abandoned call's answer is dropped,
and while it is still in flight an ask is `BUSY` (one paid call in
flight at most; nothing counted, the back-off untouched). A late refusal
from an abandoned call is dropped too: the next call finds it. An outage (`CloudUnavailable`, a call past its bound
included) and a 4xx other than 401/403/404 (400 a frame that did not
decode, 413, 422, a passing 409) raise into the back-off below, and three
such 4xx in a row (any other outcome resets the count) stop the asking; a
refusal no retry fixes -- 401/403, a `/room` not deployed (404), a bad
URL, a reply that is not JSON -- is
`SourceRefused`: the call stays counted, the labeller stops asking and
`status.survey.source_error` names it (the twin still shows only
`no_source`). A frame
with no pixels is `NOT_HERE`. A cloud source with no `vision_url` refuses
the start (400). Every answer goes through `clean_room_name()` (lowercase,
single spaces, no leading article, "none"/"n/a"/"unknown" are `unclear`),
so one room has one spelling (3.76 L2). The ask runs after the step in the
same tick, so the runner asks only when the step left the label call's
timeout plus `LABEL_TICK_MARGIN_S` (2 s) of `tick_timeout_s` -- a drill's
shorter one too, so no paid ask runs under a drill (`_label_fits()`) --
checked by `observe`'s `fits` once an ask is due (after the pose and the
cut) and again after the frame; otherwise a later tick asks, and
`status.survey.skipped_for_time` counts asks that were due and skipped.
A `room` of "unknown" (off the sim's rooms; teleop and replay frames)
is `NOT_HERE`: nothing counted, asked again after `RETRY_S`. A source that
raises keeps its call counted (sent may be billed) and backs off `RETRY_S`
(10 s), doubling per failure in a row up to `RETRY_MAX_S` (300 s), so an
outage spends a few calls, not the cap. The cap is checked first (no cut,
no frame once reached), and `capped` is set on the ask that reaches it.
The map is re-cut at most every `RESEGMENT_S` (5 s, on the runner's clock)
-- a cut is pure Python, 0.2-0.9 s on a 200-400 cell square map, and a SLAM
map changes size and origin about once a second while exploring. Between
cuts a place is looked up by POSITION in the last cut's own grid (`_Cut`),
and floor that cut has not seen is never asked about until the next;
`spacing` mode's walking distances are kept with the cut they were
measured on and redone for a new cut or a new ask. A call is
counted when sent. `_label_frame()` returns None once the mission has
stopped, and `alive` (`is_running`) is checked again after the frame, so a
stop landing during a cut or a frame read asks nobody. Any error in the
labeller (pose, frame, label source, cut) skips that step's label, logged
once -- never the survey. `MissionRunner.room_labels()` is the map's
`room_labels` block cut fresh from the world's current map (`{"names",
"cells"}`, the shape `brain/explore.py` reads for E3); saving it with the
map is S4's.

### How a tiered mission is built

This is the canonical description. The policy and perception specs link
here. The chooser between the sim stand-in and real models is
`control/brain_server.py` (`_tiered_vision_fn()`), not `brain/perceive.py`,
which only supplies both pipelines. `POST /mission/start` with `policy` `vision` or `tiered`, in
`control/brain_server.py`:

0. If the previous runner has not finished closing (`runner.finished`
   unset: `is_running()` is false before `_finish` closes the policy, which
   frees its perception models), wait for it, up to `PREVIOUS_CLOSE_WAIT_S`
   (10 s), then build anyway with a warning (3.81.E).
1. Refuse with 400 if `vision_url` is empty, or the `target_object` is missing, blank after stripping, or over `brain.navigate.MAX_TARGET_CHARS` (200, the vision service's own limit; 3.76 H2 round 3). The vision service answers both 400, so a mission that started would fail every call. `frontier` and `explore` keep their own rules (no length cap; explore refuses only a missing target).
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
`metrics_url` is set (kept as `_metrics_row`).

### Asking again after `arrived_unconfirmed` (3.53)

The failed confirmation's error also carries the frame (`arrival_frame`,
set by the agent beside `arrival_readout`); the runner holds it with the
readout and `_finish()` keeps it as `_arrival_frame`. When `mission_loop()`
ends on its own and `runner.late_confirmation_pending()`,
`start_reconfirm()` starts a `Reconfirmer` (`control/reconfirm.py`):

1. `late_update(state="waiting")`; then, every `reconfirm_probe_s` until
   `reconfirm_window_s` has passed, one `GET <vision_url>/health` (free; the
   vision service's existing route, timeout `min(3, reconfirm_probe_s)`).
2. On a 200, `runner.late_ask()`: it returns None without calling if the
   record is already final (checked under the lock), else the policy's
   `confirm_arrival(frame)` on the stored frame under
   `call_with_timeout(vision_timeout_s)`. `paid_calls` is counted in the
   same locked step as the check, so a cancel that lands while the call is
   out ships a record that includes it, and when that call returns the row
   is sent again with counters that include it; a cap refusal (decided
   locally, no I/O) takes the count back, on a final record too, and that
   record is re-sent; `_tier.stats` is refreshed from the policy afterwards. A raise
   is retried on a 200 no sooner than `reconfirm_window_s / 2` later (capped
   inside the window; a window that runs out after a failed call ends
   `failed`, not `expired`)
   (`/health` is shallow: it answers while Bedrock is down, so only a paid
   call sees that outage), at most two paid attempts, then `failed`. A
   call that outlives its guard (`TimeoutError`, "the late confirmation
   hung") is a failed attempt like any other (3.76 G1: it used to end the
   check at once, so a slow but working cloud lost its second attempt). It
   stays out on its abandoned thread and ends into its own box
   (`late_pending()`: `out`, then `value` or `error`); one that ends
   between the guard's look and `late_ask()`'s is used at once. The loop settles that
   first on every pass, before the deadline and without a probe: a value is
   the verdict (paid, never asked twice), an error (`LateCallFailed`, wherever it
   surfaces) is the reason if no attempt is left, never a second failed
   attempt. On the last attempt allowed the loop waits
   only while that call is out. A call that ends after a cancel made the
   record final re-sends it with that call's counters. While any
   confirmation is out (the mission's own, or a late one) `late_ask()`
   raises `LateCallInFlight` at once with nothing sent -- it never waits, so
   probes keep their pace and a cancel is seen at the next one -- and the
   loop asks again later; never two calls at once. A window that ends with
   a call still out after a probe answered is `failed`; with no probe ever
   answering it is `expired`. Not `_guarded_confirm()`, which maps the
   timeout away.
3. The verdict becomes `confirmed`, `refused` or `not_asked` (no call made:
   the policy's cap). The window running out is `expired`.

`late_update()` merges under the lock and never overwrites a final state
(the first answer wins, like the outcome); it returns None on a runner that
did not end `arrived_unconfirmed` with a frame. A final state logs one line
and re-sends `_metrics_row` with `stats.late_confirmation` (same `run_id`
and `finished_at`, so the same stored object). The re-sent row's counters
come from the policy at that moment (so they include the late call), and it
is sent only after every earlier send of the key has finished (the
original's `_metrics_thread`, then any earlier late row's
`_late_ship_thread`). Each late row is built, chained and queued in one
locked step from the record as it is then, so a row queued later is never
staler than one queued before it. A `POST /mission/start` whose
runner was built (a refused start changes nothing) bumps
`state["generation"]` and calls `cancel_reconfirm()`, which records
`dropped` before setting the cancel flag, so `late_ask()` refuses from that
moment. `POST /mission/stop` decides on the runner: it bumps the generation
only when the runner is still running; when the mission had already ended
it leaves a waiting check alone, and starts it itself if the loop was
cancelled in its post-tick sleep before it could. That Stop reads the
generation and claims the slot (`state["reconfirm"] = _CLAIMED`) before its
first await, so a second Stop cannot start another check beside it and a
Start meanwhile (which empties the slot and bumps the generation) leaves it
unable to start one for the old runner; `start_reconfirm()` also refuses
while the slot is held, and a claim still held when the handler raises is
released in a `finally`. After awaiting the old loop it clears
`state["task"]` only if that is still the task it awaited (3.61): a Start
during the await installed its own loop, which a later Stop must still be
able to cancel. Its closing `runner.stop()` is on the runner it captured,
which a Start can only follow once finished, so `_finish` returns without a
robot stop -- nothing reaches the new driver. `app.state.brain_state`
exposes this state, read-only, for tests. It starts the check after waiting for
`runner.finished` (set as `_finish`'s last act, which
`late_confirmation_pending()` also requires). A Stop that lands while the
runner is running marks it `operator_stopped`, and no later Stop starts a
check for it (the Guide tab sends Stop on every close; amendment 1). `mission_loop()` starts
a check only if its generation is still current. Both
`reconfirm_probe_s` and `reconfirm_window_s` must be positive, or it is
off. The outcome is never rewritten, `memory.found` stays false, and
nothing on this path calls the robot.

**`status.cloud` (3.56).** `create_app()` builds one `CloudWatch`
(`control/reconfirm.py`) when `vision_url` is set and `cloud_probe_s` > 0,
probing with `health_probe()` (timeout `min(3, cloud_probe_s)`). Its daemon
thread, started by the first `vision`/`tiered` start (idempotent) and
stopped on app shutdown, probes immediately and then every `cloud_probe_s`
while `state["runner"]` is running a `vision`/`tiered` mission, and
otherwise waits in 0.5 s steps without probing. `start_reconfirm()` wraps
its own probe in `cloud_watch.recording()`, so 3.53's late check feeds the
same record while no mission runs. `record(up)` sets `reachable` or
`unreachable`, moving `since` (wall-clock epoch seconds) only on a change;
a probe that raises is `unreachable`. `GET /mission/status` and the start
response carry `cloud: {state, since, checked_at, age_s, probes, probe_s}`
(`age_s` measured on the brain's monotonic clock -- a wall-clock step at boot once aged a fresh answer by hours, 3.76 G4 -- and `probe_s` the interval of whichever prober last answered, so the twin can tell an old answer
from a current one without comparing clocks; `state`
is `unknown` before the first probe), or `cloud: null` with no watch; so
does `POST /mission/stop`'s status (with a runner or idle), the last one the
twin draws before its poll stops. It is
not in `/health` (M5) and nothing reads it but the twin.

## Interfaces

### HTTP routes (`control/brain_server.py`)

Every route is prefixed with `ROUTE_PREFIX` (empty by default). All but
`/health` require `x-app-secret` when `APP_SHARED_SECRET` is set (401
otherwise). CORS allows any origin.

| Method, path | Body | Success | Errors |
|---|---|---|---|
| `POST /mission/start` | `MissionStartRequest` (below); first, after the 409 check and before the build, `robot().resume_autonomy()` (`POST /autonomy/resume` on the robot server) lifts a person's Stop's latch (3.84) -- a failure is logged, never a refusal; a start refused after it puts the latch back (`relatch_autonomy()`, 3.76 K2). The build slot is held until a stopped build's runner is discarded, so a start meanwhile gets the 409 and 3.81.E's wait never reads an older runner | `{"started": true, "status": {...}}` | 409 `"A mission is already running."` (also while another start is still building its runner, 3.76 C2); 409 `"Stopped while the mission was being set up."` when a Stop landed during the build (the runner is discarded, never started); 400 config or validation error with its message (explore: an unreachable robot server or a refused secret is named as such, 3.76); 403 drill on a brain with drills off; 422 an unknown field or a bad type |
| `POST /mission/stop` | none | `{"stopped": true, "status": {...}}` -- cancels the loop task, then `runner.stop()`; with no mission ever run, stops the robot anyway and returns the idle status | -- |
| `GET /mission/status` | none | `MissionRunner.status()` plus `fault`; before any mission `{"running": false, "outcome": "idle", "step": 0, "log_tail": [], "fault": "none"}` | -- |
| `POST /mission/explore/pick` | `GoalPickRequest` (`extra="forbid"`): `choice` int >= 1, `candidate` str 1-32 chars, as `status.explore.ask` publishes them (3.88 A) | `{"accepted": true, "choice", "candidate"}` -- `runner.pick_goal()` stores it under the runner's and the ask's locks; the next tick sends it to nav2 through the gated navigator. Never `/world/goal` | 409 with the reason, nothing sent: no mission, a mission that does not ask, a choice that is not the open one, an id not offered, a second pick, a pick at or after the timeout; 422 a bad body |
| `GET /mission/inventory` | none | `{"inventory": null}` before any mission; else the current or last mission's object inventory (3.46; shape in the [perception engineering spec](../perception/ENGINEERING.md), "Inventory"). On mission end the report is also saved by `control/inventory_store.py` on a daemon thread | -- |
| `GET /health` | none | liveness and configuration description; the verdict rules are the operations domain's | -- |
| `POST /recording/frame`, `POST /recording/finish` | mounted from `control/recording_routes.py` | owned by the recordings domain | -- |

**`MissionStartRequest`** (`extra="forbid"`):

| Field | Type, default | Notes |
|---|---|---|
| `target_object` | str, None | required for `vision`/`tiered`, where it must be non-blank and at most 200 characters (400 otherwise); no cap for `frontier`/`explore` |
| `target_room` | str, None | `frontier` only in practice; one of the two targets is required (`ValueError` -> 400) |
| `mission` | str, None | free text; defaulted from the targets |
| `max_steps` | int, None | the step backstop; omitted, derived from the time limit (3.70 E4) |
| `mission_time_limit_s` | float, None; `0 < x <= 86400` (else 422) | this mission's time limit; omitted, the config's `mission_time_limit_s` (3.70 E4) |
| `job` | str, `"find"` | 3.70: `"survey"` takes no target and `policy: "explore"` only; its time limit is the config's `survey_time_limit_s`. Checked by `MissionRunner` (400) |
| `policy` | str, `"frontier"` | a plain `str` in the model, not a `Literal`. A value outside `POLICIES` (`frontier`, `vision`, `tiered`) passes validation and is refused by `MissionRunner` with `Unknown policy: ...`, which is a **400**, not a 422 |
| `model_id`, `prompt_variant` | str, None | request, then config, then the vision service's default; checked once against `GET {vision_url}/navigate/models` |
| `explore_ask` | bool, None | 3.88 A: ask mode for this mission, over the config's `explore_ask`; plain `policy: "explore"` only (a find or a survey) -- `true` for any other policy, a tiered search included, is a 400, and the config's `true` leaves those unasked. With it on, `house` is the robot server's `/health` `sim_map` (`RemoteRobot.sim_map()`, None on the car) and the choices go to `<explore_choice_dir>/<mission id>.jsonl` (`runner.choice_log`, `runner.mission_id`); `metrics_config.explore_ask` records it |
| `fault` | str, `"none"` | a plain `str`. `drills.apply()` refuses a value outside `drills.FAULTS` (`none`, `vision_error`, `vision_hang`, `tick_hang`) with `Unknown fault: ...`, which is a **400**. It does so before the drills-off check, so an unknown fault is a 400 even when drills are off |

### `MissionRunner.status()` fields

`running`, `outcome`, `end_state` (3.70: one of the four end states, or
null), `end_reason`, `cancelled_by` (`stop`, `dpad` or null), `report`
(above; null until built, and for other outcomes), `budget`
(`time_limit_s`, `elapsed_s`, `paused_s` -- 3.88 A: of the elapsed time,
how much was paused on a person's pick, which the time limit leaves out --
`battery_floor_v`, `battery_v`,
`cloud_calls`, `cloud_call_limit`; both null except under `vision`, and
the limit null with the cap off), `error`,
`policy`, `job` (3.70: `find` or `survey`), `survey` (null for `find`;
`{mode, label_calls, label_call_limit, capped, rooms_named, seeds}`, each
seed `[x_m, y_m, name]`), `mission`, `target_object`,
`target_room`, `step`, `max_steps`, `found`, `room_reached`, `complete`,
`last_action`, `turns` (`count`, `reversals`, `last_turn_deg`, `share`,
`spinning`), `last_reasoning`, `rooms_visited`, `rooms_searched`,
`vision_failures`, `tier`, `perception`, `arrival`, `arrival_verdicts`,
`late_confirmation`, `last_frame_seq`, `ticks`, `seconds_since_last_tick`, `tick_rate_hz`,
`sighting`, `explore` (null unless explore searches: `goals_sent`,
`goals_failed`, `last_event`, `camera_seen_m2`, `choice_ms`, and 3.88 A's
`asks`, `ask` -- the open choice, `{choice, candidates, default, picked,
waited_s, timeout_s, remaining_s}`, or null -- and `choices`, the count
recorded), `log_tail` (last `LOG_TAIL_LINES` = 20). `tier`/`perception`
are null under any policy without a perception tier. `late_confirmation`
(3.53) is null except after an `arrived_unconfirmed` ending: `{state,
probes, paid_calls, reason, at, confirmed?}`, `state` one of `waiting`,
`confirmed`, `refused`, `failed`, `not_asked`, `expired`, `dropped`.
`AGENT-HARNESS.md` section 8 describes each. A decided arrival identity
(`arrival.identity`, confirmed or refused, not undecided) carries
`verdict_s` (3.80.E): seconds from the arrival's first identity question to
the verdict, across undecided frames, the clock restarting after
`ARRIVAL_GONE_FRAMES` (3) frames off the arrival rule (`brain/agent.py`).
`arrival_verdicts` lists the mission's decided identities, newest last, at
most `ARRIVAL_VERDICTS_KEPT` (50): `{n, tier, confirmed, verdict_s,
local_score, cloud_called}`, `n` counting from 1 past the cap.

**Outcome strings:** `idle`, `running`; the end states `found`,
`exhausted`, `out_of_budget`, `cancelled` (`END_STATES`); beside them
`room_reached`, `blocked`, `arrived_unconfirmed`, `failed`. 3.70 E1 retired
`searched`, `max_steps`, `stopped` and `preempted`: never written, and read
through `read_outcome(outcome, cancelled_by=None) -> (outcome,
cancelled_by)` (`stopped` -> `cancelled`/`stop`, `preempted` ->
`cancelled`/`dpad`). The twin and `brain_view.py` hold the same map.

### In-process

- `MissionRunner(robot, target_object=None, target_room=None, mission=None, max_steps=None, min_distance_cm=20.0, policy="frontier", vision_fn=None, vision_timeout_s=20.0, max_vision_failures=3, world=None, stuck_after=5, ..., inventory=True, detections_fn=None, mission_time_limit_s=None, battery_floor_v=None)` (3.70: with no time limit -- the in-process harnesses -- the backstop is 120 and no clock limit applies; the brain server always passes the config's) (3.46: the object inventory, recorded and reported only; `runner.inventory_sink` is set by the brain server like the metrics fields);
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
- `_GatedNavigator` (3.76 C1-C4), what `explore` gets in place of the
  navigator: `set_goal` answers `{"accepted": false, "reason":
  "mission_ended"}` once the mission has ended, cancels a goal accepted
  while it ended, and raises `Preempted` when a goal is refused because a
  person (`DRIVER_MANUAL`) holds the robot; `cancel_goal` does nothing once
  ended. `_finish` starts `end()` on a thread (`mission-goal-end`) before
  its stop: it cancels the mission's OWN goal if nav2 still holds one
  within `OWN_GOAL_TOLERANCE_M` (0.05 m) of it -- the robot server spares a
  goal on the brain's stop -- then closes the client once no goal is on its
  way. Whichever finishes last (`end()` or a goal still on its way) first
  clears, when the navigator has `clear_keepout` (`RemoteNavigator`:
  `DELETE /world/keepout`), the keepout marks the mission's wedges left
  (3.76 K5.3, the user 2026-10-21; a failure is logged, never raised).
- The intervention line (3.99, `brain/intervention_log.py`): `_finish`
  builds one record per started mission under its lock
  (`_intervention_line()`: mission id, house, policy, job, target, start and
  end in UTC epoch seconds, `autonomous_s` = elapsed minus 3.88's pauses,
  outcome, `end_via`, `person_picks`, and `intervention` -- null unless the
  end is `cancelled` and not `loop_cancelled`, then `kind` (=
  `cancelled_by`), `via`, `by`, `detail`, `at_s`, `step`, `last_action`,
  `explore_event`). The start and end are wall-clock; `autonomous_s` and
  `at_s` are the runner's clock, which in-process sweeps inject. After the
  robot's stop a thread (`mission-intervention`, `runner._intervention_writer`;
  not a daemon, so the interpreter waits for it at exit) adds
  `pose_after_stop` -- the world's pose, read on a daemon thread of its own
  and waited for at most `POSE_READ_S` (2 s), or null with `pose_note`
  `unusable[: reason]`, `read failed: ...` or `read timed out after 2 s` --
  appends the line to `runner.intervention_log` (an `InterventionLog`,
  3.88's durable `ChoiceLog` append; None writes nothing), keeps it as
  `runner.intervention`, and sets `runner.intervention_done` whatever
  happened. A mission never started, or a second end, writes nothing. Lines
  are in the order written (`ended_at` orders missions). The brain server
  builds ONE log in `create_app` (`<intervention_dir>/interventions.jsonl`)
  and hands it to every mission, so one lock covers every writer; it reads
  `house` (`RemoteRobot.sim_map()`, a `/health` GET) at every start, not
  only for ask mode. `python -m control.intervention_report [dir]` (default:
  `load_brain_config()`'s `intervention_dir`) prints the per-hour report
  (`brain/intervention_log.py`'s `summarise()`, `report()`).
- `discard()` does the same for a runner built and never started, and also closes its policy (perception models, the local VLM client); a start refused after the tiered policy was built, or whose vision drill replaced it, closes that policy too (3.80 rebase review, 2026-10-17).

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
| `mission_time_limit_s` | 1200 | 1200.0 | s | runner | 3.70 E4: set by the user 2026-10-12 (900 s of explore never finished `home_first_floor`); re-measured on the Rover. Ends `out_of_budget`; the step backstop is derived from it. 0 = no limit. A start request may override it |
| `battery_floor_v` | 10.5 | 10.5 | V | runner | 3.70 E4: set by the user 2026-10-12 (3.5 V a cell, 3S) until a discharge curve is measured on the Rover. Under the fake board's 12.00 V. 0 = off |
| `survey_time_limit_s` | 1800 | 1800.0 | s | brain_server | 3.70 S1: a survey's time limit, proposed as twice 3.45's 900 s. 0 = no limit |
| `room_label_mode` | `spacing` | `spacing` | -- | runner | 3.70 S2: when a survey asks for a room's name -- every 2 m walked and through every doorway (`spacing`), or once per room of the map's shape (`rooms`) |
| `room_label_source` | `frame` | `frame` | -- | brain_server | 3.70 S2: who names a survey's rooms -- the frame's own `room` (the simulator's, free; on the car nothing is asked) or `cloud` (`/room`, one paid call an ask). Off by default until the user approves the cost |
| `room_label_timeout_s` | 15 | 15.0 | s | brain_server | 3.70 S2: the `/room` call's HTTP timeout, held to half of `tick_timeout_s` at use (not refused at load, 3.56's reason) |
| `room_label_max_calls` | 60 | 60 | calls | runner | 3.70 S2: asks per survey; `spacing`'s 9 sim surveys' median (29) x 2, rounded up to the most any of 100 ProcTHOR surveys asked (60). Each is a paid call on the car. Reaching it leaves the rest unlabelled. 0 = no cap. Published on the brain's `/health` (null for 0) so the phone states it before a survey (3.70 S5) |
| `vision_max_calls` | 120 | 120 | paid calls | runner | 3.70: the user's call 2026-10-12, what the 120-step cap allowed before 3.70; `vision` only. Ends `out_of_budget`. 0 = no cap; a negative or fractional value is refused at load |
| `stuck_after` | (unset) | 5 | refused FORWARDs | runner | an unlucky policy gets 1-2 refusals then turns; the first watched R1 run spent 19 on one jamb |
| `min_distance_cm` | 20.0 | **30.0** | cm | agent's brain-side pre-check | 20 matches `safety.min_distance_cm`; 30 is one sim cell and vetoed ~half of legal moves with sensor noise on. See Known gaps |
| `request_timeout_s` | 10.0 | 10.0 | s | robot and world clients, allow-list check | |
| `vision_timeout_s` | 20.0 | 20.0 | s | B3.2 per-call timeout | `load_brain_config()` refuses one that is not a finite number > 0, as it does `tick_timeout_s`, `request_timeout_s` and the two drill deadlines (3.76 N1: 0 timed out every call, or aborted every mission on its first tick); `arrival_confirm_timeout_s` must be finite too |
| `max_vision_failures` | 3 | 3 | calls | B3.2 budget | B3 plan's suggestion; three abandoned threads at most |
| `reconfirm_probe_s` | 15.0 | 15.0 | s | `start_reconfirm` | 3.53: one free `/health` probe this often after an `arrived_unconfirmed` ending |
| `reconfirm_window_s` | 600.0 | 600.0 | s | `start_reconfirm` | 3.53: give up (`expired`) after this; 0 turns the late check off. Worst case 41 probes and 2 paid calls per arrival |
| `tick_timeout_s` | 30.0 | 30.0 | s | B3.3 dead-man | must stay above `teleop.stall_timeout_s` (15 s) so the specific teleop error fires first. **Not** above two `vision_timeout_s` (40 s): under a SYNCHRONOUS tier a tick that judges an arrival holds the vision call and the confirmation, each up to 20 s, so a slow pair is ended by B3.3 as "loop hung" rather than counted by B3.2. The shipped asynchronous tier has only the confirmation in the tick (review 3, fix 18; left as is, recorded) |
| `tick_interval_s` | 0.25 | 0.0 | s | `mission_loop` | pacing so a sim mission is watchable; 0 in code so tests stay fast; can be 0 on hardware |
| `allow_drills` | true | true | -- | `drills.apply` | off for any brain reachable beyond the LAN |
| `drill_vision_timeout_s` | 2.0 | 2.0 | s | vision drills | a demo in seconds, same guard |
| `drill_tick_timeout_s` | 3.0 | 3.0 | s | `tick_hang` drill | the hang lasts 3x this (`drills.OVERSHOOT`) |
| `metrics_url`, `metrics_secret` | `""` | `""` | -- | `_ship_metrics` | empty disables shipping |
| `explore_ask` | false | false | -- | brain_server | 3.88 A: explore pauses at each goal choice for a person's pick; env `EXPLORE_ASK` (0/1/true/false). A start's `explore_ask` overrides it |
| `explore_ask_timeout_s` | 60.0 | 60.0 | s | runner (`GoalAsk`) | 3.88 criterion 1: no pick in this long takes the rule's (`rule_timeout`). Must be > 0 |
| `explore_choice_dir` | `recordings/explore-choices` | same | path | brain_server | 3.88 A: one `<mission id>.jsonl` a mission asked in; env `EXPLORE_CHOICE_DIR`. `recordings/` is git-ignored |
| `intervention_dir` | `recordings/interventions` | same | path | brain_server | 3.99: `interventions.jsonl`, one line per mission end; env `INTERVENTION_DIR`. Local only, never uploaded; the suite points it at a temporary directory (`tests/conftest.py`) |

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
curl -s -X POST localhost:8001/mission/stop           # {"stopped": true, "status": {"outcome": "cancelled", "cancelled_by": "stop", ...}}
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
| 400 `... searches for an object -- /navigate takes a target_object` | room-only mission, or a blank target, under a cloud policy |
| 400 `... target_object is over 200 characters; the vision service refuses it` | an over-long target under `vision`/`tiered` |
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
| `tests/test_mission_ends.py` (43) | 3.70 E1-E5: the four end states named, the retired names gone and read as the new ones, `end_state`/`end_reason` per outcome; Stop and a D-pad takeover end `cancelled` with `cancelled_by`, the wheels at zero within 0.5 s; the time limit ends `out_of_budget` within one tick, the derived backstop with a stuck clock, an explicit `max_steps`; the battery under its floor twice (not once) within 2 s, no reading never ends one; the fake board drains `v` and `HardwareRobot.get_battery()` reads it; the report (rooms, coverage, map); a labelled room never entered keeps a mission from ending `exhausted`; coverage within 2 points of truth at the end of a search |
| `tests/test_arrival_hears_stop.py` (15) | 3.104: a person's Stop latched while the arrival's confirmation waits (the cloud then says yes) ends `cancelled`/`stop` with a `stop`/`robot_stop` intervention line, a D-pad take-over there `cancelled`/`dpad`; without a press `found`; a veto, an autonomous holder (review) and an unreachable robot server each keep `found` after one probe; a goal pending, active or lapsing keeps it when the mission sent one, and ends it `cancelled`/`dpad` (a person's tap) when it never did -- the server's real refusal texts; a goal whose reply was lost counts as sent and one the server refused does not, end to end through `_GatedNavigator` |
| `tests/test_intervention_log.py` (22) | 3.99: one line per started mission; a brain Stop is `stop`/`brain_stop` with the step and a pose, a D-pad takeover `dpad`/`preempted` naming the driver, 3.84's latch `stop`/`robot_stop`; an autonomous preemption, an abort and a time limit carry no intervention; a second end and an unstarted mission write nothing; a failed pose read still writes the line, an unwritable log changes no outcome and skips no stop, the stop never waits on the pose; the report's counts, hours and rate (per policy), a cut last line skipped, no file "no missions recorded"; the config key and its env; the brain server writes the line for a stopped mission; review round 1: a loop cancelled without a person's Stop (in process and through the brain server's task) is none, the writer is no daemon, the pose says why it is missing, the report's default directory is the brain config's; round 2: a hung pose read holds the writer only `POSE_READ_S`, the brain server shares one log between missions |
| `tests/test_survey.py` (93) | 3.70 S1-S2: a survey needs no target and ends `exhausted`; never `found`, never a confirmation; refused with a target, under another policy, or an unknown job; the room labeller cuts a map at its door, asks once per room and never from the doorway, never past its cap (the survey still ends `exhausted`), keeps `unclear`, spaces asks in `spacing` mode, asks again through a doorway within the spacing and stops `spacing` labels at the doorway, lets an `unclear` hold its room, leaves rooms two doorways from any answer unlabelled, keeps (and does not re-ask beside) a seed whose cell turned occupied, and names the room a survey starts in; a label source or frame read that raises skips the label (the survey still ends `exhausted`); a frame with no room spends no call; no ask after a stop; the cap is checked before any cut; cuts are rate-limited on the labeller's clock; `room_labels()` is a full map layer cut from the map it is given; a boolean `survey_time_limit_s` is refused at load; a growing, shifting map is not re-cut every tick and still finds the robot's room by position; `spacing` never asks on floor its cached distances have not seen; no source is remembered (no more frames); an `unknown` frame spends nothing and waits; a failing source backs off; `capped` is set on the ask that reaches the cap; a stop after the frame never reaches the source; the cloud source asks `/room` with the frame and the secret, reads a reply naming no room as `unclear`, spends nothing on a frame with no pixels, reads 5xx/408/429 as an outage and a 401 as our fault, needs a URL, and a survey through a `/room` outage backs off and still ends `exhausted`; a refused secret or a 200 that is not JSON stops the asking with the reason on the status; one room has one spelling; a paid ask is skipped when the step, the frame read (checked again after it) or a drill's deadline left too little of the tick, and the skips are counted; a 400/413 backs off while 401/403/404 stop the asking; punctuation and an article's quotes are no second spelling, a bare article is `unclear`; a hung call gives the tick back on time; three frame refusals in a row stop the asking, a 422 is a frame refusal; no skip is counted when no ask was due; no second paid call while an abandoned one is in flight; refusals count in a row only; another 4xx backs off before it stops the asking; a busy source does not lengthen the back-off |
| `tests/test_survey_sweep.py` (4, slow) | 3.70 S1/S2 pinned: the 9 sim surveys end `exhausted` with coverage >= 95%; label calls and rooms labelled >= 95% no worse than recorded, per mode |
| `tests/test_mission_runner.py` (32) | tick-driven equals blocking `run_mission()` step for step; outcomes and idempotent `_finish`; the gate refuses after stop; a stop landing mid-tick records no step; tiered readouts reach status and survive a scene with no tier; the tick keeps advancing while an async call is in flight |
| `tests/test_brain_server.py` (78) | **the import boundary, by mechanism:** a subprocess imports the brain app and lists `sys.modules`, so a forbidden import fails the suite and does not depend on review; two-hop mission completes; 409 on a second start; stop with no mission stops the car; secret gating; `ROUTE_PREFIX`; **the brain process loads no `sim`, and from `robot`/`world` only `robot.interface`, `robot.safety`, `robot.identity`, `world.interface`**; model and wording validation (refused at start, outage tolerated, one round trip); unknown field is 422; tiered start refusals and sim-vs-real perception choice; a survey's room source (3.70 S2): the frame's by default, the cloud's `/room` with the vision URL, secret and a timeout held to half the tick's, refused at start with no vision URL, never built for a find, reported on `/health`; a bad source or timeout refused at load |
| `tests/test_failsafes.py` (15) | B3.2 budget and hang; one failure survivable; no movement after stop; B3.3 dead-man; every drill ends `failed` with the robot stopped; no-fault builds the ordinary mission; drills can be switched off |
| `tests/test_camera_centred_start.py` (11) | 3.20's four criteria: centred first decision under `frontier` and `vision` (tiered is not parametrised), no step or vision call spent, centred start unchanged against `tests/data/frontier_trace_centred.json`, a refused centring ends the mission like any refused move |
| `tests/test_mission_guarded_verbs.py` (3) | 3.32: a mission's verbs go through the safety layer's guarded plan; the gate forwards plan and stop count; a finished mission refuses a plan |
| `tests/test_bearing_turns.py` | `blocked` after `stuck_after` refusals, never on an arrival, switchable off, reset by progress |
| `tests/test_arrival_reconfirm.py` (26) | 3.53: once the cloud is back, exactly one paid call on all 12 clear starts for a cloud that sees the target and one that does not; the outcome unchanged; no robot method called; free and bounded probing (`expired`); two paid attempts, half a window apart, then `failed`, and a server-side outage that clears in the window `confirmed`; a cancel never pays, also when it lands between the probe and the call; a cap refusal counts no paid call; a hung call is never doubled (a second attempt only once it has ended, 3.76 G1); only an unconfirmed arrival records anything; the metrics row re-sent, not added; through `brain_server`, a new mission drops it, a refused start and a Stop after the ending (also one in the loop's last sleep) do not; the brain's own loop asks |
| `tests/test_review_376g.py` (14) | 3.76 G1, G4: a late answer that outlived its guard is used once it lands (not paid twice, not held to the retry), also on the last attempt, in the window's last interval, and when it lands at the guard itself; its error landing during a probe is not counted twice, and the attempt then due is sent at once; a last call ending in an error ends the check with that error; a call ending after a cancel ships its cost; a call out all window with the cloud never up is `expired`; a confirmation still out delays the ask without ending the window or blocking a probe; a window that ends with nothing out is not "still in flight"; the cloud watch's `age_s` survives a wall-clock step |
| `tests/test_robot_contract.py` | `_HaltGate` passes the body conformance suite as a backend |
| `tests/test_review_376c.py` (13) | 3.76 batch C: Stop ends explore's own nav2 goal (not a person's), also one accepted as Stop lands on another thread; no goal after the end; a person at the D-pad ends explore; explore waits on `ros` just after its goal; "took over" parsed; one start at a time while building, and a Stop during the build means no start; unreachable or 401 robot named at start |
| `tests/test_review_376k2.py` (8) | 3.76 K2: a Stop during the arrival's waits (a trigger call still out, the last confirmation) sends no cloud confirmation; a start refused (400, 403) or stopped during its build leaves a person's Stop latched, a started one lifts it; a start during a stopped build's discard gets the 409; a late call ending at its guard after a cancel still ships its cost (a guard) |
| `tests/test_review_376n1.py` (14) | 3.76 N1: a guarded verb on the wall clock stops at an aged line (a guard); feedback lost on the verb's last read keeps a stopped or completed verb's outcome, and a short end reads `no_feedback`; the tick-hang drill centres the camera and counts each tick once; a deadline of 0 is refused; a tick inside its deadline is not a loop not turning; the ROS world follows the drive's bridge URL; an empty `TRACK_SCRUB` is unset |
| `tests/test_remote_robot.py` | an identical action sequence and step count in-process, over ASGI and over a live socket |
| `tests/test_offline_mode.py` (11) | 3.49, with every socket connect refused and the real HTTP cloud client: the robot server loads no cloud client (a subprocess's `sys.modules`, and every file under `robot/`); `frontier`/`explore` still `found`; tiered (sync, async) searches from out of sight and ends `arrived_unconfirmed` at the target; `vision` fails without moving; no offline tiered run `failed`; a hanging cloud parks for at most budget x timeout + 1 s; the twin references no external host |

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
  is built only at arrival: an outage there ends `arrived_unconfirmed`
  (3.47) and is asked again when the cloud returns (3.53). Anywhere else a
  blown vision budget ends the mission `failed`.
- **The late check lives in memory** (3.53): a brain restart loses it, and
  the mission's metrics row then keeps `arrived_unconfirmed` with no late
  verdict.
- **No mission persistence** across a brain restart, one mission per brain,
  `MissionMemory` per mission (`AGENT-HARNESS.md` section 12).
- **No boot units (B5)** on the Jetson.
- **S7 (chaos and soak)** not run.
