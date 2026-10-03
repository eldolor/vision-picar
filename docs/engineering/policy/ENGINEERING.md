---
kind: engineering
domain: policy
status: current
verified: 2026-10-02
parent: docs/policy/ARCHITECTURE.md
---

# Policy -- engineering

How the move-deciding code is built today. Why it is split this way -- one
scene schema, event-driven deliberation, arbitration by question, arrival by
lidar -- is in the [architecture spec](../../policy/ARCHITECTURE.md). This
file is true only until the code changes and is updated in the same commit.

## Implementation

| File | What it does |
|---|---|
| `brain/agent.py` | `ConstrainedAgent` (allowed actions, `step()`, `sensed_scene()`, the stuck-breaker, the optional proximity veto), `MissionAgent` (memory, arrival review in `_review_scene()`, room backfill, world pose, frontier preference in `decide()`), `ObjectSearchAgent` (look-around on first entering a room), `StepResult`. |
| `brain/vision_agent.py` | `VisionAgent(MissionAgent)`: `decide()` skips the frontier logic and calls `ConstrainedAgent.decide()` on the model's `safest_direction`. |
| `brain/navigate.py` | `navigate_scene()` (one POST to `{vision_url}/navigate`), `to_scene()` (pure mapping), `vision_fn_for()` (binds target, URL, secret, model, wording; carries `set_searched_rooms`). |
| `brain/tiered.py` | `TieredVision` (a callable `vision_fn`), `TierStats`, `corroboration_for()`, `turn_for()`, `tiered_vision_fn_for()`. Trigger policy, free-frame stand-in with its precedence ladder, async dispatch with an epoch, sized turns, spin guard. |
| `brain/arrival.py` | `ArrivalCheck.observe(scene, robot)`, `arrived_scene()`. |
| `brain/goal_pose.py` | `OdomTracker` (integrates `get_odometry()` path length along heading), `GoalPose` (`sight()`, `bearing_from()`, `distance_from()`, `clear()`; `is_point`). |
| `brain/memory.py` | `MissionMemory` (visited/searched rooms, `Sighting` with a map-frame pose, `ActionRecord`, `is_complete()`, `summary()`, `as_context()`). |
| `brain/rooms.py` | `ROOM_FEATURES`, `identify_room()`. Tested, but nothing in the mission path calls it: sim frames carry a room label and real frames take the cloud's `room_guess`. |

**Which agent runs.** `control/mission_runner.py` builds `VisionAgent` for
`vision` and `tiered` and `ObjectSearchAgent` for `frontier`. The difference
between `vision` and `tiered` is only which `vision_fn` is bound. How the
tiered `vision_fn` and its perception pipeline are built at mission start is
described once, in [mission](../mission/ENGINEERING.md) ("How a tiered
mission is built").

**One `ConstrainedAgent.step()`:** `get_camera_frame()` -> `vision_fn(frame)`
-> `_review_scene()` (arrival) -> `decide()` -> proximity veto (if enabled)
-> `self.safety.check_and_execute(action, angle=turn_deg)` (a
`robot/safety.py` `SafetyController` over the gated body; `SafetyViolation`
means not executed) -> `StepResult` appended to `history`.
`MissionAgent.step()` then backfills `frame["room"]` from
`_navigate.room_guess` when the frame says `unknown`, attaches the world pose
to the frame, and records observation, searched room and action in memory.

**Frontier `decide()`:** complete -> STOP; right after an executed turn ->
FORWARD; else peek right, left, centre with `get_distance()` against
`side_clearance_cm + VERB_MIN_MOVE_M*100`; with a usable pose, prefer the
first of FORWARD/RIGHT/LEFT whose lookahead bucket is unvisited, else the
first clear one; with no pose, the right-hand rule; boxed in -> the
stuck-breaker (three STOPs in a row -> RIGHT).

**Tier free-frame precedence** (`TieredVision._local_scene()`): measured
local bearing (`_steer_to`) > dead-reckoned bearing (`_dead_reckoned_direction`,
only if `hold_bearing`) > held cloud goal (if `hold_goal`, or while a call is
in flight) > `SCAN_ACTION` ("RIGHT"); then the spin guard replaces a turn
with FORWARD once consecutive turning reaches `spin_guard_after * 90`
degrees and the target is not detected. A cloud turn is sized from the local
bearing when both agree on the side (`_annotate`), else it is a
`SCAN_TURN_DEG` search step. The tier never overrides the cloud's direction
on a call frame, but `MissionAgent._review_scene()` runs after the vision
step: on a synchronous call frame that satisfies arrival it rewrites the
scene to STOP and `found` (`arrived_scene()`), whatever the cloud said.

**What the hysteresis does and does not gate.** `consecutive_frames` is
read only in `_trigger_for()`. `_steer_to()` returns a direction for any
frame whose `status` is `detected` with a bearing, with no streak, so one
detected frame steers. On a free frame the steer outranks the held cloud goal.
Under `async_cloud` (shipped) every frame's scene is a local one and a landed
cloud answer only replaces `_last_cloud_scene`. A cloud `not_visible`
therefore never overrides a local sighting. Synchronously, only the call
frame uses the cloud's direction. Checked 2026-10-02 with a fake pipeline and
a cloud that always answers `not_visible`: a lone `detected` frame after four
`absent` returned FORWARD with no trigger; in sync mode the
`candidate_sighting` frame returned the cloud's LEFT and the next free frame
was FORWARD again.

**Triggers** (`_trigger_for`): first frame -> `mission_start` (and seeds the
edge detector); `unavailable` -> nothing, clears the run; confirmed edge into
`detected` -> `candidate_sighting`; `absent` streak >= cold-search bar, or
distance since last call >= `cold_search_after_cm` -> `cold_search`; frames
since last call >= `stale_after` -> `staleness`. `max_calls` caps the total.

## Interfaces

**Scene** (every `vision_fn` returns this; `brain/vision.py` defined it):

| Key | Values |
|---|---|
| `obstacles_ahead` | list of str |
| `free_space` | `none` / `some` / `clear`, or `unknown` (a prompt variant that did not ask; every tier free frame) |
| `doorway_visible` | always False today |
| `important_objects` | list; this is what marks memory `found`. Under `vision` the target appears here only when the cloud answers `target_reached` (`brain/navigate.py` `to_scene()`) or `arrived_scene()` fires. Under `tiered` with `tier_async_cloud: true` (shipped) only `arrived_scene()` puts it here: `_collect_inflight()` stores a landed cloud scene in `_last_cloud_scene` and every returned scene is `_local_scene()`, with `important_objects: []` and `target_reached: False`. Synchronously, the call frame returns the annotated cloud scene, so its `target_reached` can end the mission too. Checked 2026-10-02 with a fake pipeline (always `absent`) and a cloud that always answers `target_reached` with the target named: over 20 frames and 5 calls, async put the target here on 0 frames, sync on 5 (every call frame). Under `frontier` `sensed_scene()` copies the frame's `objects_visible`, so the target is `found` on first sight |
| `safest_direction` | `FORWARD` `LEFT` `RIGHT` `REVERSE` `STOP` (`LOOK_*` from the explorer); anything else becomes STOP |
| `turn_deg` | optional int, the size of a LEFT/RIGHT; absent means the executor's default 90 |
| `_navigate` | `target_visible`, `target_direction` (`left`/`center`/`right`/`not_visible`/`unknown`), `target_reached`, `obstacle_ahead` (None = not asked), `room_guess`, `distance_estimate`, `reasoning` |
| `_perception` | `Perception.as_dict()` (perception domain) |
| `_tier` | `cloud_called`, `trigger`, `cloud_landed`, `models`, `corroboration`, `in_flight`, `holding`, `vocabulary`, `pacing`, `stats` (`frames`, `cloud_calls`, `frames_per_call`, `triggers`, `perception`, `claims`, `corroborated`, `verdicts`, `cloud_ms`, `perception_ms`) |
| `_arrival` | `state` (`arrived`/`approaching`/`not_judged`), `streak`, `reason`, `bearing_deg`, `range_m`, `radius_m`. `observe()` checks detection before the scan, so a frame without a detection reads `approaching` ("target not detected") even with no scan; `not_judged` needs a detection plus a panned camera or an unusable scan, or a policy with no `_perception` at all |

**Frame keys the policy reads:** `image_base64`, `media_type`, `room`,
`objects_visible` (explorer), `detections` (sim only, via the perception
pipeline), `pan_deg`, `metadata.seq`, and `odometry` (attached by the
runner: `usable`, `distance_m`, `heading_deg`).

**Signatures:**
`vision_fn_for(target_object, vision_url=None, secret=None, timeout_s=60.0, client=None, model_id=None, prompt_variant=None)`;
`TieredVision(pipeline, cloud_vision_fn, *, consecutive_frames=2, cold_search_after=6, cold_search_after_cm=None, stale_after=8, max_calls=None, corroboration_bar=0.5, oov_cold_search_after=None, async_cloud=False, hold_goal=True, steer_on_sight=True, spin_guard_after=8, hold_bearing=False, hold_bearing_max_m=1.0)`
with `close()`, `reset_epoch()`, `set_searched_rooms()`;
`ArrivalCheck(radius_m=0.40, centre_deg=3.0, frames=2)`;
`MissionAgent(robot, memory, side_clearance_cm=30.0, world=None, min_distance_cm=20.0, vision_fn=None, max_consecutive_stops=3, vision_proximity_veto=False)`.

## Parameters and configuration

**Tier keys** (`brain:` block of `config/robot.yaml`, read by
`control/brain_config.py`, passed by `control/brain_server.py`
`_tiered_vision_fn()`):

| Key | Shipped | Unit | Why |
|---|---|---|---|
| `tier_consecutive_frames` | 2 | frames | 6.1: 1 frame gives 2.8x fewer calls than frames, 2 gives 4.1x, 3 gives 4.5x for a frame of lag |
| `tier_cold_search_after` | 6 | absent frames | measured 2026-09-12 with async on: 2, 3, 4 and 6 all covered 14 of 15 visible spans; 2 cost 41 more calls |
| `tier_cold_search_after_cm` | 0 (off) | cm | needs odometry; no real-pixels backend has it yet |
| `tier_async_cloud` | true | -- | 2026-09-12 (`evaluations/gpu/pacing/`): 14 of 15 spans looked at vs 12, 94 calls vs 100, worst latency 2 frames vs 4. The class default is False |
| `tier_hold_goal` | true | -- | Phase F: on five rig walks the per-frame scan outvoted the cloud 5:1 |
| `tier_steer_on_sight` | true | -- | Phase G: 28 RIGHT / 4 FORWARD on 37 locally-detected frames before it |
| `tier_spin_guard_after` | 8 | quarter turns (x 90 degrees) | 720 degrees; counted in degrees since R1c (84.5% -> 98.3-98.6% arrival at 90% detection, starter house, 2026-09-25) |
| `tier_hold_bearing` | false | -- | P25: off until an A/B shows it helps |
| `tier_hold_bearing_max_m` | 1.0 | m | a starting value, not derived |
| `tier_stale_after` | 8 | frames | 6.1's figure; contributed 5.7% of triggers there |
| `tier_max_calls` | 0 (no cap) | calls | `max_steps` bounds the mission |

`oov_cold_search_after` exists on `TieredVision` but has no config key.
`tier_corroboration_bar` is also passed to `TieredVision`; it changes no
decision, and its value and evidence are kept in one place,
[perception](../perception/ENGINEERING.md).

**Constants:**

| Constant | Value | Where | Why |
|---|---|---|---|
| `ALLOWED_ACTIONS` | 7 actions | `brain/agent.py` | the constrained set |
| `FRONTIER_LOOKAHEAD_M` | 0.30 | `brain/agent.py` | one nominal move |
| `DEFAULT_VISIT_BUCKET_M` | 0.30 | `brain/agent.py` | fallback when the map gives no `resolution_m` |
| `_PIVOT_DEG` | 90 | `brain/agent.py` | the prediction must match the default turn actually issued |
| `SCENE_CLEAR_CM` | 90 | `brain/agent.py` | descriptive "some" vs "clear" boundary; three sim cells |
| `max_consecutive_stops` | 3 | `ConstrainedAgent` | stuck-breaker |
| `side_clearance_cm` | 30.0 | `MissionAgent` | peek threshold (plus `VERB_MIN_MOVE_M` 0.01 m from `robot/safety.py`) |
| `DEFAULT_TIMEOUT_S` | 60.0 | `brain/navigate.py` | library default; a mission passes `vision_timeout_s` (20) |
| `DISTANCE_ESTIMATES` | 4 values | `brain/navigate.py` | mirrors the service's allow-list |
| `SCAN_ACTION` | `RIGHT` | `brain/tiered.py` | one direction makes oscillation obvious in logs |
| `CENTER_BAND_DEG` | 10.0 | `brain/tiered.py` | reporting vocabulary, matches `/navigate` |
| `STEER_BAND_DEG` | 3.0 | `brain/tiered.py` | R1b: all 28 blocked search runs had driven 5-6 degrees off the door line |
| `MIN_TURN_DEG`, `MAX_TURN_DEG` | 5, 90 | `brain/tiered.py` | floor stops a zero turn; ceiling re-measures behind targets |
| `SCAN_TURN_DEG` | 45 | `brain/tiered.py` | R1b: under the 60-degree sim and 66-degree camera field of view; 86% -> 100% of search starts found |
| `TierStats.MAX_SAMPLES` | 500 | `brain/tiered.py` | bounds the status payload |
| `ARRIVAL_RADIUS_M` | 0.40 | `brain/arrival.py` | the scan reads 0.465 m one move out and 0.165 m at the collar |
| `ARRIVAL_CENTRE_DEG` | 3.0 | `brain/arrival.py` | duplicates `STEER_BAND_DEG`; a test pins them equal |
| `ARRIVAL_FRAMES` | 2 | `brain/arrival.py` | one bad frame never ends a mission |
| `ARRIVAL_BEAM_HALF_DEG` | 2 | `brain/arrival.py` | median of 5 beams; the first version used the nearest and declared `found` 95 cm out |
| `ARRIVAL_EDGE_M` | 0.10 | `brain/arrival.py` | 3.32: a jamb window is refused |

Arrival reads the scan in the body frame using `LIDAR_X_M` from
`robot/safety.py`; its value is in the canonical chassis table,
[platform](../platform/ENGINEERING.md) "Parameters and configuration".

## Procedures

**Run the policy suites** (free, no models, no network):

```bash
pytest tests/test_agent.py tests/test_mission_agent.py tests/test_object_search.py \
       tests/test_vision_policy.py tests/test_tiered.py tests/test_bearing_turns.py \
       tests/test_arrival.py tests/test_goal_pose.py tests/test_memory.py tests/test_rooms.py -q
```

The sweeps in `tests/test_bearing_turns.py` and `tests/test_arrival.py`
run hundreds of in-process missions; expect them to dominate the time.

**Print the R1 A/B:** `python -m tests.demo_hold_bearing_ab`. It still
builds the **starter house** (`build_starter_world()`), while
`tests/test_bearing_turns.py` moved to the scaled house in 3.32, and the
Rover's corridor clips the starter house's door jamb. Its output is starter
-house history, not today's record. When last quoted (2026-09-25), sized
turns closed about 4.1 cells with about 1 reversal, and quarter turns ended
further away with about 5. Not re-run for this spec.

**Run the free explorer:** `python -m tests.demo_active_search`. It builds
`ObjectSearchAgent` directly, with no world and `min_distance_cm=30`.
**Observed 2026-10-02:** it does not find the backpack. Output ends:

```text
=== Find the red backpack. (active scanning) ===

Steps taken: 150
Look-around scans performed: 6 pan actions across 2 room entries
Rooms searched: ['hallway', 'living room']

NOT FOUND after 150 steps.

Memory summary: Hallway, living room searched. No red backpack found.
```

132 lines of `Blocked ...` precede it, almost all `Blocked LEFT: stopped
after 0.0deg -- turn left clamped: a corner would come within 1.3cm < 1.3cm
(scan_footprint)`. See Known gaps.

**Run a tiered mission over a recorded walk** (paid on every trigger, needs
`VISION_URL` and `pip install -r requirements-perception.txt`):
`python -m tests.demo_replay_mission <walk dir> "red backpack" --policy tiered`.
Expect the trigger beside each step and a calls-and-frames counter at the end.
**This caveat's canonical home is here; other specs link to it.** A replay
has no scan, so arrival is never judged, and under the shipped asynchronous
tier a landed cloud `target_reached` cannot end the mission either (see the
scene's `important_objects` row). The outcome is not a navigation result.

**Read a live mission** (`GET /mission/status`, mission domain):
`turns.spinning` true means mostly one-way turning with few reversals;
`tier.stats.frames_per_call` should sit well above 1 (6.1 measured 4-6x).
On a backend without a scan (teleop, replay), `arrival.state` reads
`approaching` ("target not detected") on frames with no detection, and
`not_judged` ("no range sensor") on frames with one. It never reads
`arrived`.

## Verification

| Test file (count 2026-10-02) | What it pins, with recorded numbers |
|---|---|
| `tests/test_bearing_turns.py` (17) | Runs in the **scaled house** since 3.32 (`HOUSE = "scaled_house"`). R1: sized turns arrive from off-axis while quarter turns are the defect (relative bars). R1b: every search start sees the target within 12 steps and arrives. R1c: at 90% per-frame detection >= 95% of missions arrive. Spin guard counts degrees; stuck -> `blocked`; spin named a spin. **3.32 recorded only that these pass in the scaled house**; the numbers below are history |
| `tests/test_arrival.py` (14) | Uses `tests/test_bearing_turns.py`'s `_build()`, so the scaled house. **Recorded 2026-10-01 (PLAN 3.32, guarded verbs):** of missions that arrived, 69/69 (perfect detection), 689/689 (90%) and 677/689 = 98.3% (80%) end `found` (bar 95%); 0 false arrivals in 69 / 690 / 690; farthest `found` 0.51-0.52 m from the target's centre (bar 0.60 m, so 0.08 m of margin). Also: not judged without scan, with a panned camera, or without local perception; two frames needed; range read at the bearing; edges refused |
| `tests/test_tiered.py` (92) | triggers and hysteresis, call cap, staleness floor, async dispatch and epoch drop, an async failure reaching B3.2, landed verdicts shown once, local bearing beats a stale cloud goal, spin guard never overrides a sighting, timing on the worker |
| `tests/test_vision_policy.py` (46) | visible is not found; absent `obstacle_ahead` is `unknown`; room guess backfill and `searched_rooms`; the policy does not peek; replay missions; proximity veto off by default and never over a real sensor |
| `tests/test_agent.py`, `tests/test_mission_agent.py`, `tests/test_object_search.py` | constrained loop, frontier preference, look-around scan |
| `tests/test_goal_pose.py` (12), `tests/test_memory.py` (9), `tests/test_rooms.py` (7) | anchoring under rotation, honesty without range; memory completion; room matching |

**Starter-house history** (2026-09-25, before 3.32 moved the sweeps; kept
only so older plan entries can be read, not as today's record): R1 4.06 cells
closed / 0.6 reversals vs -1.23 / 4.8 for quarter turns; R1b 171 starts,
100% found / 100% arrive; R1c 98.3-98.6% arrival at 90% and 80% detection
over 690 missions each; 3.11 arrival 69/69, 676/678, 676/678, farthest
`found` 0.386 m. The R1 figure was not re-measured in the scaled house. PLAN
3.5 records the demo's every-frame row falling from 4.06 to 3.69 once stuck
detection landed. That was also measured in the starter house.

**Checklist for a change:** measure through `MissionRunner` -> agent ->
`robot/safety.py` -> backend, never a harness that turns the robot itself;
pair any stability metric (reversals, `median_command_run`) with a progress
metric (distance closed, arrival); if a step size changes, re-check every
threshold counted in steps.

## Known gaps

- The room-level planner is not built; `MissionMemory.as_context()` feeds
  nothing.
- `tier_hold_bearing` is off; a 1-in-3 detector closes only ~1.6 cells and
  needs a range-anchored goal pose (R1, R1b).
- The vision proximity veto cannot be turned on from the brain service
  (`control/brain_server.py` never passes it); only a direct `MissionRunner`
  can.
- Code prose that has drifted: `brain/tiered.py`'s module docstring says
  three triggers and staleness "deliberately absent" (it is implemented);
  `_held_direction()`'s docstring says the goal is held only while a call is
  in flight (with `hold_goal` it is held on every free frame);
  `brain/vision_agent.py` says "no room memory yet" (built, via
  `brain/navigate.py`). Several docstrings cite `AGENT-HARNESS.md` section 12
  for room memory; it is section 10.
- Arrival cannot be judged on a phone walk, and under `tier_async_cloud:
  true` a landed cloud `target_reached` is never applied, so tiered phone
  walks end `max_steps` when they arrive (P7e). Whether to apply it is an
  open question in the [architecture spec](../../policy/ARCHITECTURE.md).
  `brain/arrival.py`'s module docstring (lines 6-9) says that before arrival
  "only a paid cloud call could end a mission"; that holds only for a
  synchronous tier, not the shipped asynchronous one.
- Arrival's panned-camera refusal reads `perception.pan_deg`, which is 0 for
  every sim frame: `sim/mock_robot.py`'s frame carries no `pan_deg`,
  `FrameReportedPipeline` builds `Perception` without one, and the sim's
  detection bearings are already camera-relative (`GridWorld.view_angle()`).
  So the refusal works on real frames only. Harmless today: `VisionAgent`
  never peeks and 3.20 centres the camera at mission start.
- **Local false positives steer, and can end `found`.** One detected frame
  steers, the hysteresis gates only triggers, and arrival never consults the
  cloud's identity (see "What the hysteresis does and does not gate").
  `brain/tiered.py`'s comment above `DEFAULT_STEER_ON_SIGHT` says the two-frame
  hysteresis has to pass before steering and that a wrong lock-on "is
  corrected at the next paid call". Neither is what the code does. The
  comment is left as is, and the open question is in the
  [architecture spec](../../policy/ARCHITECTURE.md).
- **`tests/demo_active_search.py` ends NOT FOUND after 150 steps** (observed
  2026-10-02). Almost every step is a LEFT that the pivot guard clamps to
  0 degrees, so the robot never moves. The stuck-breaker counts STOPs, not
  refused turns. **Probable diagnosis, UNCONFIRMED** (read from
  `brain/agent.py`'s `MissionAgent.decide()`, not traced): the demo passes no
  world, so `decide()` uses the right-hand rule. A refused LEFT is recorded
  as not executed, so the next `decide()` skips the "FORWARD after an
  executed turn" branch and peeks again. The left ray clears
  `side_clearance_cm + 1` (31 cm), so LEFT is chosen again, and the pivot
  guard again refuses the swept corner -- LEFT forever. The boxed-in
  fallback (the stuck-breaker) never fires because LEFT reads clear. Two
  definitions of "clear" disagree: a peek ray along the turned heading,
  and the chassis' swept corners during the pivot.
- **`tests/demo_hold_bearing_ab.py` still builds the starter house**, which
  the tests left in 3.32.
- More code prose that has drifted: `config/robot.yaml`'s
  `tier_spin_guard_after` comment says "after this many consecutive turns",
  but the guard counts degrees (`spin_guard_after * 90`, since R1c). The
  `max_steps` comment's stale "83 steps" is listed in
  [mission](../mission/ENGINEERING.md).
