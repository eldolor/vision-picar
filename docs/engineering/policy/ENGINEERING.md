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
| `brain/agent.py` | `ConstrainedAgent` (allowed actions, `step()`, `sensed_scene()`, the stuck-breaker), `MissionAgent` (memory, arrival review in `_review_scene()`, room backfill, world pose, frontier preference in `decide()`), `ObjectSearchAgent` (look-around on first entering a room), `StepResult`. |
| `brain/vision_agent.py` | `VisionAgent(MissionAgent)`: `decide()` skips the frontier logic and calls `ConstrainedAgent.decide()` on the model's `safest_direction`. |
| `brain/navigate.py` | `navigate_scene()` (one POST to `{vision_url}/navigate`), `to_scene()` (pure mapping), `vision_fn_for()` (binds target, URL, secret, model, wording; carries `set_searched_rooms`). |
| `brain/tree.py` | 3.94, on `py_trees` 2.6.0 since 3.95: the node kinds `Sequence` and `Fallback` (py_trees' `Sequence` and `Selector`, `memory=False`), `Condition` and `Action` (`Behaviour`), `Decorator` (py_trees ticks the child, then `after(bb, child_status)`); py_trees' statuses. Adds settings per node, `tick(bb)` (hands the blackboard to every node of the tree for that tick), `describe()`, `walk()`, `find()`. A node returning a non-status raises. |
| `brain/tiered.py` | `TieredVision` (a callable `vision_fn`), `TierStats`, `corroboration_for()`, `turn_for()`, `tiered_vision_fn_for()`. Trigger policy, free-frame stand-in with its precedence ladder, async dispatch with an epoch, sized turns, spin guard. |
| `brain/arrival.py` | `ArrivalCheck.observe(scene, robot)`, `arrived_scene()`, the readout states (`REFUSED` is set by the agent, never here: this module makes no calls). |
| `brain/goal_pose.py` | `OdomTracker` (integrates `get_odometry()` path length along heading), `GoalPose` (`sight()`, `bearing_from()`, `distance_from()`, `clear()`; `is_point`). |
| `brain/memory.py` | `MissionMemory` (visited/searched rooms, `Sighting` with a map-frame pose, `ActionRecord`, `is_complete()`, `summary()`, `as_context()`). |
| `brain/explore.py` | `ExploreAgent(MissionAgent)`, the `explore` policy over nav2 (3.31): frontier, view and approach goals through a `Navigator`, the retry rule, the wedged/boxed escape (`_queue_escape()`, `_do_pending()`: REVERSE, CREEP, OPEN). Since 3.45 every decision is a record in `agent.events` and an INFO line `explore {json}`: `goal_sent` (kind, goal, and for a frontier its `rank`, `candidates`, `path_m`, `size_m`, `score`, `skipped`; since 3.77 every frontier, view or room goal also carries `choice_ms`, the wall time of the goal choice that sent it), `goal_refused`, `goal_ended` (`state`, `seconds`, `moved_m`; `state` may be `stalled`, 3.83), `escape` (`why`: wedged or boxed), `creep` (3.74: `way`, `executed`, `waited`, the safety layer's `forward` and `floor` readings just before, each `[cm, source]` off one depth grid, and `floor_blind`, the band zones reading unusable), `verb` (action, `executed`, `waited`, the refusal), `searched`; each with the SLAM `pose`. A verb refused for the mission's own goal (`OWN_GOAL_REFUSAL`) or for `ros` still holding the robot just after it (`OWN_GOAL_HOLDER`, 3.76 C5) waits; any other refusal ends the mission. A goal refused because a person holds the robot raises `Preempted` from the runner's gated navigator (3.76 C4); one refused `person_stop` (a person pressed Stop; explore's goals carry `x-driver: brain` through `RemoteNavigator` since 3.84) raises `StoppedByPerson`, and the mission ends `cancelled` by that Stop. Since 3.76 E: a goal nav2 holds that is not the mission's own (`OWN_GOAL_TOLERANCE_M`, 0.05 m, shared with the runner) -- live or just finished -- raises `Preempted`, and `_verb`/`_cancel` cancel only the mission's own (a leftover before its first goal is still cancelled); a goal call that fails transiently is a wait, up to `BRIDGE_BLIPS_MAX` (40) in a row, and any other error still ends the mission -- a transport error or a 5xx from the robot server, unless its `x-bridge-retryable: false` header says its bridge refused it for a fault no wait mends, which raises `_setup_fault()` (3.82); a goal the robot server refuses with `retryable: false`, or a map that is unusable with `retryable: false` (its bridge refuses the robot server: a 401 for mismatched secrets; 3.82), raises `_setup_fault()`, so the mission ends `failed` naming the status and `hint()`'s advice, where any other refusal or missing map still waits; an approach already within standoff cancels a live goal. A goal with no house position (`RosWorld`'s `unanchored`: a new SLAM session not yet anchored) is never the mission's own -- its goals are sent on an anchored session whose anchor never changes -- so it is treated exactly as any goal that is not the mission's own (`_is_own` is False without `x_m`; 3.76 round 7, replacing rounds 3-6's special cases). Known consequence: before the mission's first goal, such a goal is a leftover and `_verb` cancels it, as it cancels a placed leftover; on `dev` before 3.76 H1 an unanchored session's goal read as no goal and was not cancelled. |
| `brain/frontier.py` | `find_frontiers()`, `reachable()`, `RetryBook`, `camera_seen()`, `find_view_gaps()`: pure functions over `get_map()`'s grid (3.31). `RetryBook.fail(known=)` takes a count or a callable measured at the entry's own position; explore passes its map since 3.76, which turns on the early retry (`early_growth`, 25% more known cells around a failed place) that had never run. Since 3.77 the grid work is numpy and gives exactly the answers of the pure-Python version it replaced (frozen in `tests/frontier_reference.py`, compared by `tests/test_frontier_fast*.py`), where an order or a tie decides an answer, the old order is kept. `reach()` returns a `Reach` (the stops in BFS order, their path lengths, `steps` as a dict on demand); explore computes it once per goal choice and passes it to `find_frontiers(..., reach_map=)`, `find_view_gaps(..., reach_map=)` and the boxed-in check. |
| `brain/choice_log.py` | 3.88 A: `GoalAsk` (one open goal choice at a time: `open()`, `pick()` from the brain's HTTP thread, `poll()` from the tick, `close()`, `end()`, `paused_s()`, `snapshot()`; one lock, nothing called out under it), `ChoiceLog` (one mission's JSON lines, each flushed and fsynced; the file made at the first record), `read_choices()` (skips a cut-off last line). |
| `brain/rooms.py` | `ROOM_FEATURES`, `identify_room()`. Tested, but nothing in the mission path calls it: sim frames carry a room label and real frames take the cloud's `room_guess`. |

**Which agent runs.** `control/mission_runner.py` builds `VisionAgent` for
`vision` and `tiered` and `ObjectSearchAgent` for `frontier` -- except a
`tiered` mission with `tier_search: explore` (3.80.G), which runs
`ExploreAgent(sight_from="perception")` (below, "Explore drives a tiered
search"). The difference
between `vision` and `tiered` is only which `vision_fn` is bound. How the
tiered `vision_fn` and its perception pipeline are built at mission start is
described once, in [mission](../mission/ENGINEERING.md) ("How a tiered
mission is built").

**One `ConstrainedAgent.step()`:** `get_camera_frame()` -> `vision_fn(frame)`
-> `_review_scene()` (arrival) -> `decide()`
-> `self.safety.check_and_execute(action, angle=turn_deg)` (a
`robot/safety.py` `SafetyController` over the gated body; `SafetyViolation`
means not executed) -> `StepResult` appended to `history`.
`MissionAgent.step()` then backfills `frame["room"]` from
`_navigate.room_guess` when the frame says `unknown` (stripped and
lowercased, `unclear`/`unknown` dropped: free text, 3.76 L2), attaches the world pose
to the frame, and records observation, searched room and action in memory.

**Frontier `decide()`:** complete -> STOP; right after an executed turn ->
FORWARD; else peek right, left, centre with `get_distance()` against
`side_clearance_cm + VERB_MIN_MOVE_M*100`; with a usable pose, prefer the
first of FORWARD/RIGHT/LEFT whose lookahead bucket is unvisited, else the
first clear one; with no pose, the right-hand rule; boxed in -> the
stuck-breaker (three STOPs in a row -> RIGHT; never on a scene whose
`_navigate.target_reached` is true, the frame an arrival is found, 3.76 L2).

**Tier free-frame precedence** (`TieredVision._local_scene()`). Since 3.94
it is a behavior tree (`brain/tree.py`; built by `_build_what_tree`, printed
by `describe_tree()`). It is ticked inside the who tree: every route that
answers a frame builds its scene through `_local_scene`.
* **The order of the tree.**
  1. Take this frame's evidence: the local VLM's located yes is spent here
     even when a better rung wins, and the held goal is read.
  2. The rungs, a fallback, best evidence first:
     * the measured local bearing (`_rung_steer` -> `_steer_to`);
     * the local VLM's located yes (`_rung_located`);
     * the dead-reckoned bearing (`_rung_reckoned`, only if `hold_bearing`);
     * YOLOE + CLIP's uncertain candidate (`_rung_candidate`, only if
       `steer_on_candidate`);
     * the held goal (`_rung_held`; if `hold_goal`, or while a call is in
       flight);
     * `SCAN_ACTION` ("RIGHT").
  3. The spin guard, a decorator over the rungs, decided after they ran
     (py_trees ticks a decorator's child first). It replaces a turn with
     FORWARD once consecutive turning reaches `spin_guard_after * 90`
     degrees, the target is not detected, and the rung is not a bearing
     from this frame (`RUNGS_THIS_FRAME`).
  4. The turn's size: a bearing rung's own, else `SCAN_TURN_DEG`.
  5. The turn counted for the guard.
* **The reference.** `tests/tiered_reference.py` freezes the code the tree
  replaced. `tests/test_tier_tree.py` checks that the two make the same
  decision on 10,000 random states. A cloud turn is sized from the local
bearing when both agree on the side (`_annotate`), else it is a
`SCAN_TURN_DEG` search step. The tier never overrides the cloud's direction
on a call frame, but `MissionAgent._review_scene()` runs after the vision
step: on a synchronous call frame that satisfies arrival -- and whose arrival
the cloud then confirms, see "Arrival" below -- it rewrites the scene to
STOP and `found` (`arrived_scene()`), whatever the trigger's answer said.

**Arrival** (handoff 2026-10-02 1a). `MissionAgent._review_scene()` runs
`ArrivalCheck.observe()`; on `arrived` it calls `_confirm_identity(readout,
frame)`, which asks `arrival_confirm_fn(frame)` -- `MissionRunner` passes
`_guarded_confirm`, the policy's `confirm_arrival` under B3.2's
`vision_timeout_s`, a failure raising `VisionUnavailable` into the budget;
without a runner the agent falls back to `vision_fn.confirm_arrival`.
**3.98: in the simulator the cloud step is the sim's own geometry.**
With `sim_identity_fn` (the brain server sets it to `geometry_identity` on
the simulator's frames under `sim_arrival_confirm: geometry`),
`confirm_cloud()` first asks it: of the frame's `detections` within
`SIM_CONFIRM_CENTRE_DEG` (10 deg) of the centre, the nearest must be the
target (the label test `FrameReportedPipeline` uses). Its answer returns
`{"confirmed", "cloud_called": false, "tier": "sim"}` before the call cap
and the call itself (`MissionRunner._guarded_confirm` may still have waited
for a trigger's call in flight); the agent labels the step like any
confirmation (`identity_tier: sim`), and the metrics row carries
`sim_arrival_confirm`. A sim frame with no
`detections` is refused. It answers None on a frame that is not the
simulator's (`metadata.source`), which then goes to the cloud as below. A
local tier, when configured, still answers first; but a local-led arrival
(3.80.O, cloud-only) is answered by the geometry too, so a sim evaluation
that means to measure the cloud's verdict on such arrivals (3.80.O's blind
runs) sets `sim_arrival_confirm: cloud`.

`TieredVision.confirm_arrival(frame)` makes one synchronous call to the
cloud (even under `async_cloud`) -- through `confirm_vision_fn` when one was
given (3.56: the brain's second client, built with the confirmation's
shorter HTTP deadline), else the triggers' `cloud_vision_fn`, resolved at
call time; `close()` drops `confirm_vision_fn`, so 3.53's late check (which
only runs after the runner has closed the policy) asks on the triggers'
client and B3.2's full deadline. Since 3.76 `close()` also marks the policy
closed, under the lock `_dispatch` takes: no trigger call is sent once it has
run, even from a frame that was in perception when Stop landed (the frame's
epoch, taken at its start, travels with the call), and no worker pool is
created again; a landed async answer annotated for its side effects is not
counted by the spin guard; a `candidate_sighting` held back by a call in
flight re-arms. Since 3.81.E `close()` also swaps the pipeline for a
released stand-in (`RELEASED_PIPELINE`, every frame `unavailable`) so the
ended mission's models can be freed; a frame already in perception finishes on
the old one. `confirm_arrival()` still works after `close()`, for 3.53;
`set_searched_rooms()` reaches both clients, and
`wait_inflight()` drains an async call still out so the runner can do that
on B3.2's timeout before the confirmation's own clock starts -- counts it in `cloud_calls`
and in `triggers["arrival_confirmation"]` (`TRIGGER_ARRIVAL`), and confirms
only on `_navigate.target_visible is True`; at `max_calls` it refuses
without calling. Not confirmed, or no confirmer on the policy: the readout
becomes `refused`, the scene is not rewritten, and `_identity_refused`
stops further calls until `observe()` stops returning `arrived`. A call
that raises gets the readout attached as `arrival_readout` and goes on to
B3.2; nothing is left on the agent. Whether the failure was the cloud being
unreachable is decided by `brain/navigate.py`'s `CloudUnavailable`, not
here (3.47; mission engineering). The
readout carries the verdict under `identity`, and every later refused frame
carries the refusing verdict too (`_refusal`), so the final status says why.
Since spec review 3 (fixes 8-10): `confirm_arrival()` first waits for an
async call still in flight and applies it (`_collect_inflight()`, whose take
of the landed call -- future, trigger, perception -- is one step under
`_collect_lock` since 3.56's follow-up, so a `wait_inflight()` thread the
runner abandoned and the tick thread can never both apply one answer; and
when the runner's wait times out it calls `abandon_inflight()`, which marks the call charged (it stays in `_inflight`, so the gate stays closed), so a call
B3.2 already counted is never collected and counted again), so one
call is in flight at a time; it returns the tier's `stats` after the call,
and on the frame that asked, the agent's `_label_confirmation()` sets the
scene's `_tier` to `cloud_called: true`, `trigger: "arrival_confirmation"`
and those stats -- so the log reads `[cloud: arrival_confirmation]` and a
mission that ends `found` publishes the call in `status.tier.stats` and its
metrics row. A refused frame's `_navigate.reasoning` is prefixed `arrival
refused -- <reason>`. A mission then ended `blocked` by `stuck_after` says it
stopped at an object the cloud did not confirm, not that the way was
obstructed (`MissionRunner.tick()`).

**Arrival through the local tier (3.80.C).** With `local_identity_fn` set
(the brain builds `brain/local_vlm.py`'s `LocalVLMClient` when
`tier_local_vlm_url` / `LOCAL_VLM_URL` is non-empty; empty, the default, is
everything above unchanged), `confirm_arrival()` is `confirm_local()` and,
only when that returns None, `confirm_cloud()` (the cloud call described
above). `confirm_local()` never raises: the client's `LocalVLMAbsent` (a
refused or dropped connection, a 503) or any other exception returns None,
so the cloud is asked on the same frame. A score >= `LOCAL_YES_AT` (0.5)
confirms; <= `local_no_below` (0.1) refuses, as the cloud's no does; a score
between, or status `unparsed`/`timeout`/`busy`/`error` (the client's own
timeout, `tier_local_timeout_s` + 2 s, comes back as `timeout`), is
`undecided`. The agent turns `undecided` into readout state `checking`
(`brain/arrival.py` `CHECKING`): not refused, asked again on the next
arrival frame. After `local_max_unable` (2) undecided answers,
`next_confirm_is_local()` is False and the next arrival frame goes to the
cloud. The agent calls the policy's `arrival_ended()` on every frame the
arrival rule does not hold; `LOCAL_RESET_FRAMES` (3) of them in a row reset
the count, fewer are the same arrival flickering. `abandon_local()` (the
runner gave up waiting) counts one unable answer, status `timeout`, and a
sequence number makes the abandoned call's late answer change nothing.
`escalations` counts arrival questions the cloud was asked after the local
tier's tries were spent. Verdicts carry `tier`
(`local`/`cloud`) and `local_called`; `_label_confirmation()` sets
`_tier.identity_tier`, keeps `cloud_called` only for a real cloud call, and
the runner's log line reads `[local: arrival_confirmation]`. `TierStats`
counts `local_calls`, `local` (`yes`, `no`, `absent`, `unable_<status>`,
`unable_band`, `unable_noscore`, `unable_runner_timeout`), `local_ms` and `escalations`. `close()` closes and drops the
client, so 3.53's late check is the cloud's.

**Every frame's question through the local tier (3.80.D).** With
`local_sighting_fn` set (the brain sets it with `local_identity_fn`, to the
same client's `ask(frame, box=True)`), `_trigger_for()` adds two questions
and `__call__` puts every question to the local tier first:
* `uncertain_band` (`TRIGGER_BAND`): an absent frame whose best candidate's
  CLIP P is in [`local_band_low` (config `tier_local_band_low`, 0.5), the
  pipeline's `match_probability` (0.8)) on `consecutive_frames` frames in a
  row; asked once per run (`_band_asked`), armed again by any other frame.
* `out_of_vocabulary` (`TRIGGER_OOV`): `vocabulary.in_vocabulary` false and
  `_absent_streak >= consecutive_frames`; checked before `cold_search`.
* `_local_first()` is the sighting call unless `_frame_unable` has reached
  `local_max_unable`; then the question takes today's cloud path, counted in
  `frame_escalations["unable"]`, and the count starts again.
* `_ask_local_first()` asks nothing while the arrival's identity call is
  still out (`_local_inflight`, one the runner gave up on): the VLM would
  answer `busy`, counted unable, and two bought a paid call (3.76 K3); the
  question stays open, a one-shot trigger's edge armed again (`_rearm`, as
  for an unable answer). Otherwise it counts `local_triggers[trigger]` and runs
  `_ask_tiers()` on the async worker (`_dispatch(..., tier="local", job=...)`,
  so one question is in flight across both tiers; `_tier.in_flight_tier`
  says which) or inline. `_ask_tiers()` returns a `_LocalAnswer`, or -- the
  call raised (`LocalVLMAbsent` or any fault) -- counts
  `local_frame["absent"]` and `frame_escalations["absent"]` and returns the
  cloud's scene about the same frame (cloud counted with its trigger, under
  `_stats_lock`, cap checked; a Stop since dispatch returns None and pays
  nothing). A cloud scene lands as before and drops `_local_held`.

* **What the cloud said is kept (3.91 A).** Every cloud answer that
  lands, async (`_collect_inflight()`, whichever thread collects it; noted
  even if `_annotate` raises) or synchronous (`_annotate_noted()`, on the
  main path and the local-first path's cloud fallback), and every failed
  call, is summarised by `_note_cloud_answer()` into `_cloud_answers` under
  `_collect_lock`: `action` (the scene's `safest_direction`) and `reply`,
  the whole `_navigate` through `_json_safe()` (`CLOUD_TEXT_CHARS`,
  `CLOUD_FIELDS_KEPT`; `to_scene` adds the reply's own `reply_action` and
  `model_id` there for this), the
  robot seq it was asked on (`_dispatch()`), `via_local_tier` when the cloud
  answered a local-first question; a failure as its type and HTTP status
  (`_http_status`), never its text. At most `CLOUD_ANSWERS_KEPT` (64) wait;
  older ones go, counted (`stats.cloud_answers_dropped`). Scenes never carry them: `take_cloud_answers()`
  hands each out once, to `MissionRunner._note_decision()` (a frame's first
  decision only, after its seq check -- on a body that has never numbered a
  frame they are dropped, there being no row or recorder) and to `_finish`
  before the end is visible (`final=True`: a call that landed but that no
  frame collected is noted with its verdict and NOT applied -- it runs on the
  Stop's thread before the motors stop -- and a later collect does not note
  it again; `unfiled_cloud_answers`). Known limit: a tick that has taken a
  landed call but not yet noted it when the Stop's take runs loses that note.
  `reset_epoch()` drops what was not taken. Read-only: nothing decides on it.
* `_apply_local()` (on landing; a different epoch is ignored): yes (>= 0.5)
  and no (<= `local_no_below`) reset `_frame_unable` and the pacing counts
  as a landed cloud answer does. A yes whose reply has `box_frac` gives a
  bearing (`_bearing_of()`: box centre over the pipeline's `hfov_deg`, 66
  if none, plus the frame's pan, less the heading odometry measured since
  the question's `_Question.pose`), stored as a one-shot `_local_turn`
  (sized by `turn_for()`, or FORWARD inside `STEER_BAND_DEG`) and
  `_local_held = "FORWARD"`, which `_held_direction()` returns before the
  cloud's goal. A no drops both. Anything else is unable
  (`unable_<status>`, `unable_band`, `unable_noscore`): `_frame_unable` + 1
  and the question stays open -- the pacing counts are not reset, a
  candidate's edge and the band are armed again, a `mission_start` is
  asked again (`_retry_start`).
* `_local_scene()` precedence: perception's steer, then `_local_turn`
  (used once), then dead reckoning, then the held goal, then the scan; the
  spin guard skips a located turn. `_tier` adds `local_called` (the
  question asked this frame) and `local_landed` (`trigger`, `answer`,
  `score`, `bearing_deg`), and the reasoning reads `[reactive tier, local
  question: <trigger>]`.
* `TierStats` adds `local_triggers`, `local_frame`, `local_frame_calls` and
  `frame_escalations`; `cloud_calls` and `triggers` count only questions
  that reached the cloud, and `max_calls` caps only those. A local failure
  never reaches B3.2; a cloud failure after an escalation does.
* The local VLM serves one request at a time, so `confirm_local()`, on the
  local tier's turn only (`next_confirm_is_local()`), first waits for the
  LOCAL phase of a frame's question still out (`_Question.local_done`, set
  when the sighting call returns or fails) -- never for a cloud call it fell
  through to, and never applying its answer, which the tick thread does.
* A question due for the cloud is never `out_of_vocabulary` (it waits for
  `cold_search`), and an absent local tier hands an `out_of_vocabulary`
  question to the cloud only past cold_search's frame or distance bar
  (`_Question.cloud_ok`). With the call cap spent, `_local_first()` keeps
  the question local whatever the unable count. Any cloud answer resets
  `_frame_unable`, and so does a cloud failure that is not fatal (an
  explore search, `_count_cloud_error`): a cloud that gave no answer is no
  better than a spent cap (3.76 K3); `frame_escalations["unable"]` counts only a dispatch that
  went out. Per-frame latency is `local_frame_ms`, apart from the arrival's
  `local_ms`. The simulator's pipeline reads boxes in the render's 60 deg
  (`control/brain_server.py` `SIM_HFOV_DEG`).

**The local tiers steer (3.80.F).** Two additions, each off unless the
brain has a local tier (`local_identity_fn`):
* **The local VLM's direction** (`local_direction_fn`, the client's
  `direction`; config `tier_local_direction`, env `TIER_LOCAL_DIRECTION`,
  **default false**: criterion F4 failed, VE answered `forward` on every
  frame). On the worker, after a `SEARCH_TRIGGERS` question
  (`mission_start`, `cold_search`, `staleness`) whose sighting reply is not
  a yes, finished (`answered` or `unparsed`, not a `timeout`/`busy` still
  draining) and about a frame perception did not detect the target on,
  `_ask_direction()` asks the direction question about the same frame.
  `_Question.local_done` is set only after it, so an arrival check waits
  for the whole local phase. `_apply_direction()` on landing: a word holds
  `LEFT`/`RIGHT`/`FORWARD` in `_local_held` (`_local_held_why =
  "direction"`), closes the search question (the pacing counts reset,
  `_retry_start` cleared) and resets `_frame_unable`; an unable reply sets
  `_frame_unable` to one more than before the sighting answer (so a no plus
  an unable direction still escalates after `local_max_unable`); `absent`
  or `unsupported` (a service without the route: 404) change nothing. A
  later sighting no keeps a held direction (it drops only a located
  FORWARD); a yes drops it; a cloud answer drops either. Counted in
  `local_direction` (`left`, `right`, `forward`, `unable_<status>`,
  `absent`, `unsupported`) and `local_direction_ms`.
* **YOLOE + CLIP's uncertain candidate** (`steer_on_candidate`; config
  `tier_steer_on_candidate`, env `TIER_STEER_ON_CANDIDATE`, default true
  since F5). `_steer_to_candidate()`: an absent frame, camera centred, whose
  strongest crop (by probability: an absent `Perception` has no `best`) has
  P in [`local_band_low`, `match_probability`) and a bearing turns toward
  it as `_steer_to()` does (FORWARD within `STEER_BAND_DEG`). A local no
  about a frame that had such a crop (`_Question.band`) sets
  `_candidate_rejected` until the band run breaks. It never anchors dead
  reckoning and never feeds the arrival rule. Counted in
  `candidate_steers`. The simulator's pipeline has no sub-gate crops, so
  it never acts there.
* `_local_scene()` precedence is now: perception's steer, `_local_turn`,
  dead reckoning, the candidate, the held goal (the local VLM's, else the
  cloud's), the scan; the spin guard skips a candidate turn.

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
`detected` -> `candidate_sighting` (a sighting held back by a call in flight
fires once it lands, 3.76); every pacing count below runs from when the
last answer LANDED, not from when it was asked (3.76: a slow call used to
leave a bar already passed, and a second call went out back to back);
`absent` frames since then >= cold-search bar (a `detected` frame does not
reset the count), or
distance since last call >= `cold_search_after_cm` -> `cold_search`; frames
since last call >= `stale_after` -> `staleness`. `max_calls` caps the total,
the arrival confirmation included (a sixth key in `stats.triggers`,
`arrival_confirmation`; see "Arrival").

Since 3.94.B the triggers are a behavior tree (`_build_when_tree`,
printed by `describe_when_tree()`).
* **First,** the frame vote is tallied.
* **Then the first of these that fires:**
  * the opening question;
  * an `unavailable` frame (no trigger; it clears the runs);
  * the opening question again;
  * this frame's evidence (the band run, the status run, the confirmed
    edge), followed by the events, in order:
    * local tracking;
    * `candidate_sighting`;
    * (no trigger) the absent edge, which only moves `_last_confirmed`;
    * for an `absent` frame, the absent count, then the band,
      out-of-vocabulary, and cold search by frames, then by metres;
    * `staleness`;
    * none.

`tests/tiered_reference.py` freezes the code it replaced, and
`tests/test_tier_tree.py` checks the two give the same trigger and state
on 10,000 random sequences of 1-8 frames, with the dispatch path's changes
between frames.

**Who answers** (`__call__`, since 3.94.C a tree: `_build_who_tree`,
printed by `describe_who_tree()`).
* No trigger: no question (`_since_call` counts on).
* Otherwise an edge held back by a call in flight is re-armed, and the
  local tier is chosen unless it has been unable `local_max_unable` times
  in a row. Then the first of these answers:
  * the call cap (only with no local tier to ask);
  * one call at a time;
  * the mission ended (closed, or the epoch moved while the frame was in
    perception);
  * the local tier (`_ask_local_first`);
  * the cloud, sync or dispatched.

The plumbing stays in `TieredVision`: the worker, the epoch, `close()`'s
races and the stats.

**The tree from config (3.94.D).** `TieredVision(tree=...)` takes
`brain.tier_tree`. Its rungs and absent events are built in the configured
order, and its numbers replace the module constants inside the policy:
`self.steer_band_deg`, `self.scan_turn_deg`, `_turn_for`'s bounds and
`self.local_yes_at`. The module's `STEER_BAND_DEG`, `SCAN_TURN_DEG`,
`turn_for` and `LOCAL_YES_AT` stay the shared defaults. `TREE_DEFAULTS`
equals `control/brain_config.py`'s `TIER_TREE_DEFAULTS` (a test checks).

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
| `_arrival` | `state` (`arrived`/`approaching`/`not_judged`/`refused`), `streak`, `reason`, `bearing_deg`, `range_m`, `radius_m`, and on an arrival `identity` (`confirmed`, `cloud_called`, `reason`, `cloud_reasoning`). `observe()` checks detection before the scan, so a frame without a detection reads `approaching` ("target not detected") even with no scan; `not_judged` needs a detection plus a panned camera or an unusable scan, or a policy with no `_perception` at all |

**Frame keys the policy reads:** `image_base64`, `media_type`, `room`,
`objects_visible` (explorer), `detections` (sim only, via the perception
pipeline), `pan_deg`, `metadata.seq`, and `odometry` (attached by the
runner: `usable`, `distance_m`, `heading_deg`).

**Signatures:**
`vision_fn_for(target_object, vision_url=None, secret=None, timeout_s=60.0, client=None, model_id=None, prompt_variant=None)`;
`TieredVision(pipeline, cloud_vision_fn, *, consecutive_frames=2, cold_search_after=6, cold_search_after_cm=None, stale_after=8, max_calls=None, corroboration_bar=0.5, oov_cold_search_after=None, async_cloud=False, hold_goal=True, steer_on_sight=True, spin_guard_after=8, hold_bearing=False, hold_bearing_max_m=1.0)`
with `close()`, `reset_epoch()`, `set_searched_rooms()`, `confirm_arrival(frame) -> {confirmed, cloud_called, reason, cloud_reasoning, stats}`;
`ArrivalCheck(radius_m=0.40, centre_deg=3.0, frames=2)`;
`MissionAgent(robot, memory, side_clearance_cm=30.0, world=None, arrival_confirm_fn=None, min_distance_cm=20.0, vision_fn=None, max_consecutive_stops=3)`.

## Parameters and configuration

**Tier keys** (`brain:` block of `config/robot.yaml`, read by
`control/brain_config.py`, passed by `control/brain_server.py`
`_tiered_vision_fn()`):

| Key | Shipped | Unit | Why |
|---|---|---|---|
| `tier_consecutive_frames` | 2 | frames | 6.1: 1 frame gives 2.8x fewer calls than frames, 2 gives 4.1x, 3 gives 4.5x for a frame of lag |
| `tier_sighting_span_s` | 0.25 | seconds | 3.90 A: when frames carry `captured_at` (the car's camera stream, the phone), that run must also span this -- today's two frames at the 0.25 s tick -- so a faster camera does not shorten it (R1c). 0: counted only. Unstamped (sim) frames are counted only |
| `tier_frame_vote` | `off` | `off`, `2of3`, `3of5` | 3.90 E: `candidate_sighting` on k detected of the last n frames perception took, all inside n x 0.25 s when stamped, instead of `tier_consecutive_frames` in a row. A frame read twice (same seq) votes once. Per mission `frame_vote` on `/mission/start` overrides. A switch for comparing walks; adoption waits on 3.90 C's Rover data. Status: `tier.stats.vote`, and `frames_skipped` (gaps in the robot's frame seq: frames perception never took) |
| `tier_cold_search_after` | 6 | absent frames | measured 2026-09-12 with async on: 2, 3, 4 and 6 all covered 14 of 15 visible spans; 2 cost 41 more calls; env `TIER_COLD_SEARCH_AFTER` (3.80.P). A cloud-only mission's pace |
| `tier_local_cold_search_after` | 1 | absent frames | 3.80.P: the pace while the local VLM tier takes the question (its questions are free); a question the cloud answers -- the local tier absent, or unable `tier_local_max_unable` times -- still waits `tier_cold_search_after`'s bar (`TieredVision._cloud_cold_search_bar`), so a dead VLM never brings this pace to the paid cloud; the Jetson at pace 1: perception p90 171 ms, `MemAvailable` min 823 MB, 0 OOM; env `TIER_LOCAL_COLD_SEARCH_AFTER` |
| `tier_cold_search_after_cm` | 0 (off) | cm | needs odometry; no real-pixels backend has it yet |
| `tier_async_cloud` | true | -- | 2026-09-12 (`evaluations/gpu/pacing/`): 14 of 15 spans looked at vs 12, 94 calls vs 100, worst latency 2 frames vs 4. The class default is False |
| `tier_hold_goal` | true | -- | Phase F: on five rig walks the per-frame scan outvoted the cloud 5:1 |
| `tier_steer_on_sight` | true | -- | Phase G: 28 RIGHT / 4 FORWARD on 37 locally-detected frames before it |
| `tier_spin_guard_after` | 8 | quarter turns (x 90 degrees) | 720 degrees; counted in degrees since R1c (84.5% -> 98.3-98.6% arrival at 90% detection, starter house, 2026-09-25) |
| `tier_tree` | `{}`: today's values (`TIER_TREE_DEFAULTS`) | a mapping | 3.94.D: `what_rungs` (the steering rungs' order; `scan` last; only `reckoned`, `candidate`, `held` may be left out), `absent_events` (the order of `band`, `oov`, `cold_frames`, `cold_metres`; any may be left out), `steer_band_deg` 3.0 (at most the arrival's `ARRIVAL_CENTRE_DEG`, or the robot stops turning toward a target the arrival never counts centred), `min_turn_deg` 5 and `max_turn_deg` 90 (whole, at most 90), `scan_turn_deg` 45 (whole, inside the bounds), `local_yes_at` 0.5 (frame questions only; the arrival keeps `LOCAL_YES_AT`). Under `tier_search: explore` the band and bounds keep their defaults, because explore steers with the module's. Validated in `load_brain_config`, and the policy checks its names itself. An unknown key is refused, including any naming who-to-ask's order, when-to-ask's head or the arrival. The metrics row carries it as one compact JSON string |
| `tier_hold_bearing` | false | -- | P25: off until an A/B shows it helps |
| `tier_hold_bearing_max_m` | 1.0 | m | a starting value, not derived |
| `tier_stale_after` | 8 | frames | 6.1's figure; contributed 5.7% of triggers there |
| `tier_max_calls` | 0 (no cap) | calls | `max_steps` bounds the mission |
| `tier_search` | `cloud` | -- | 3.80.G: `explore` = explore's nav2 goals choose where to search; env `TIER_SEARCH` |
| `sim_arrival_confirm` | `geometry` | -- | 3.98 (the user, 2026-10-24): on the simulator's frames (`_frames_are_simulated()`), the arrival's cloud step is `brain/arrival.py` `geometry_identity()` (no call); `cloud` keeps the paid call. Real frames always ask; env `SIM_ARRIVAL_CONFIRM` |

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
| `TierStats.MAX_SAMPLES` | 500 | `brain/tiered.py` | bounds the status payload. Since 3.76 a reservoir: a uniform sample of the whole mission (seeded), not its first 500; `n` and `max` are the true count and maximum |
| `ARRIVAL_RADIUS_M` | 0.40 | `brain/arrival.py` | the scan reads 0.465 m one move out and 0.165 m at the collar |
| `ARRIVAL_CENTRE_DEG` | 3.0 | `brain/arrival.py` | duplicates `STEER_BAND_DEG`; a test pins them equal |
| `ARRIVAL_FRAMES` | 2 | `brain/arrival.py` | one bad frame never ends a mission |
| `ARRIVAL_SPAN_S` | 0.25 s | `brain/arrival.py` | 3.90 A: with stamped frames the `ARRIVAL_FRAMES` run must also span this, at any frame rate. `run_spans()` (shared with the tier's `candidate_sighting`) allows `SPAN_SLACK_S` 0.05 s short of it, at most a fifth of the span: a 30 fps camera read at the 0.25 s tick gives stamps 0.217-0.283 s apart. A negative span (the wall clock stepped back) counts as unstamped |
| `ARRIVAL_BEAM_HALF_DEG` | 2 | `brain/arrival.py` | median of 5 beams; the first version used the nearest and declared `found` 95 cm out |
| `ARRIVAL_EDGE_M` | 0.10 | `brain/arrival.py` | 3.32: a jamb window is refused |

**Explore constants** (`brain/explore.py`, `brain/frontier.py`):

| Name | Value | Where | Why |
|---|---|---|---|
| `leave_clearance_m(min_distance_cm)` | half of `FOOTPRINT_LENGTH_M` + `min_distance_cm` = 0.3265 m at 20 cm (7 map cells at 5 cm) | `brain/explore.py`, passed as `clearance_m` to `find_frontiers()` and `find_view_gaps()` | 3.45: room to drive away from the goal at any heading. `evaluations/slam-345/goal_sweep.py`: 36% of chosen goals leavable at 0.22 m, 52% at 0.30 m, 98% at this |
| `GOAL_CLEARANCE_M` | 0.22 m | `brain/frontier.py` | 3.31's default: the half-diagonal plus sized as a margin over nav2's old 0.12 m inflation; since 3.67's 0.35 m it sits inside the inflation band at a cost nav2 still plans to (about 149 of 252, below inscribed). Still used by approach points and the boxed-in check |
| `PASS_CLEARANCE_M` | 0.15 m | `brain/frontier.py` | room to pass through (the den's 0.6 m door) |
| `GOAL_TIMEOUT_S` | 120 s | `brain/explore.py` | a goal nav2 never finishes is failed |
| `STALL_S`, `STALL_M` | 20 s, 0.15 m | `brain/explore.py` | 3.83: a live goal during which the robot moved less than `STALL_M` over the last `STALL_S` ends `stalled` (cancelled), and is handled as a wedge: the escape backs out and choosing starts again. The window is anchored at the goal's start pose and moves on whenever the robot gets `STALL_M` from it; an unusable pose or an unreadable bridge decides nothing, and the window starts again when readings return. A stall's back-out is not charged to `MAX_ESCAPES` (`escapes_charged`), which only boxed-in and never-left escapes spend. Since 3.85 a stall after the robot got under way (`STUCK_MOVED_M` from the goal's start) is a retry failure of the goal's place at once; a never-left goal keeps 3.38's first-wedge forgiveness. An escape whose REVERSE the safety layer refuses with room ahead (`forward_clearance()` at least `FORWARD_GAIN_CM`, 5 cm, over the bar the safety layer vets a speed-50 FORWARD against, `required_clearance_cm()`) drives FORWARD once before the creep and the turn (3.83's breakfast pocket: 1.3 cm behind, about 30 cm ahead). An approach counts as made (`_approach`) only once nav2 took its goal, and a refused one clears the record (3.85: the pantry livelock). Beside 3.80.G's close-in (perception sightings only), within `_at_target()`'s band the two 3.85 changes defer to it as `dev` did before: an approach that stalls there after getting under way (`STUCK_MOVED_M`) is a plain failure (a retry failure, no escape: the escape backs and turns the robot away from the target; one that never left still backs out), and a refused approach send there stays on record (`_approach`), so close-in runs on the next frame rather than a goal re-sent on every frame. Under the dining table every goal had run its full 120 s (3.70 M5) |
| `MAX_ESCAPES` | 3 | `brain/explore.py` | escapes before a search may end while boxed in |
| `CREEP_SPEED`, `CREEP_PIECES`, `CREEP_ROOM_CM` | 5 (0.03 m/s), 3, 1.0 cm | `brain/explore.py` | 3.44: the escape creeps at creep speed, which the safety layer lets come to 3 cm (`CREEP_MARGIN_CM`), toward an end the full bar refuses. 3.89: a creep its time ran out on (`stopped_short: "timeout"` with the body's `VERB_MAX_TAG` in its reason: the `VERB_MAX_S` cap, not a direct-mode verb that ran out its periods) is followed by another the same way, up to `CREEP_PIECES` in all, while that end still has more than `CREEP_ROOM_CM` over the 3 cm line (a creep that reached the line just before the cut reads as cut too); one that reached the line, stalled or was refused is not, and a later piece never falls back to the other end (it would give back the gain). Under `drive: ros` nav2's collision_monitor halves a forward twist with something 13-33 cm ahead (`PolygonSlow`), so a forward creep from the bar takes two pieces. Each `creep` event carries `piece` and `cut_by_time` |
| `FACE_MIN_TURN_DEG` | 10 deg | `brain/explore.py` | 3.69 B: a turn to face the target that the pivot guard refuses is retried at half the angle down to this; then REVERSE, face, a guarded FORWARD, once, and the remembered sighting is let go |
| `LOOK_FROM_CANDIDATES` | 40 | `brain/frontier.py` | 3.69 D: places tried per unseen patch for one with a line of sight to it (facing its middle, else its nearest part); the old straight-line choice only if none has |

**Explore's sighting rules (3.69).** A target detected on a PANNED frame is
a direction, not a place: `_glimpse()` queues `LOOK_CENTER` and a turn of
the body toward it, and the next centred frame decides (event `glimpse`).
A centred sighting takes its range from the lidar window at the bearing --
the median, unless the window is an edge (`ArrivalCheck._is_edge`: two
surfaces), when it takes the nearest return, which is short of the target,
never past it. A view goal (`ViewGap.face`) turns toward the part of the
patch it was chosen to see. What the camera has seen (`ExploreAgent.seen`,
`_note_seen()`, and the status's `camera_seen_m2`) is kept as cell centres
on the first map's lattice, a later map placed on it by its origin's offset
rounded to whole cells -- not worked out from each map's origin, which a
SLAM map moves by fractions of a cell as it grows (3.76 L2: the same floor
was added again after every shift; a lattice at the house's 0 put centres
on cell borders for a half-cell origin).

**Explore drives a tiered search (3.80.G).** `brain.tier_search` (env
`TIER_SEARCH`, `cloud` | `explore`, default `cloud`). Under `explore` the
runner (`MissionRunner(tier_search=...)`, `self.explores`) builds
`ExploreAgent(sight_from="perception")` with the tiered `vision_fn`, and
the brain server builds the `RemoteNavigator` (refusing the start with no
nav2, and with no local tier: every question would be a cloud call whose
direction is unused) before any tier client exists. The tiered `vision_fn`
runs every step as before; its `safest_direction` is never executed, and
`cloud_errors_fatal=False` (a cloud failure on an escalated question is
logged and counted in `stats.cloud_errors`, not raised into B3.2).
* `_hits()` reads the step's scene (`self._scene`, set in `_choose`):
  `_perception` detected with a bearing (less its `pan_deg`, so the in-frame
  bearing, to which explore adds the camera's pan as for a detection), else
  the local VLM's located yes on the frame it landed
  (`_tier.local_landed.bearing_deg`, a body bearing, less the pan). Never
  `frame["detections"]`. The `frame` mode (the `explore` policy) is
  unchanged.
* `_close_in()`: with no goal live and a sighting within `_at_target()`
  (the 0.7 m band `_back_to_sighting` calls "here"), and the retry rule
  allowing it, turn by perception's bearing (`tiered.turn_for`, outside
  `STEER_BAND_DEG`) or FORWARD, through `_verb()`. Holds still while the
  arrival readout is arrived/checking/refused. A verb that waits on the
  mission's own goal is dropped (the next frame re-sizes it); a refused one
  counts toward `CLOSE_IN_REFUSED` (3 in a row: the sighting fails,
  REVERSE; any accepted goal starts the count again, 3.76 K3), and a
  refused turn over `FACE_MIN_TURN_DEG` queues explore's FACE.
* `_note_refusal()`: an arrival REFUSED drops the place it was at
  (`RetryBook` failed to its limit), once per refusal, and explore searches
  on.
* Found only through `MissionAgent._review_scene` (the arrival rule and
  the identity verdict); the runner's status reports `tier_search`, the
  `explore` block, and no `last_turn_deg` from the tiers' scene.

**The local box finishes the approach (3.80.O).** Under `tier_search:
explore` with a local tier, a target only the local VLM sees is approached,
closed in on and arrived at on its box; such an arrival is confirmed by the
cloud alone.
* `brain/tiered.py`: `located_local_bearing(scene)` / `local_said_no(scene)`
  read `_tier.local_landed` (the one reader). `set_local_track(on)` arms
  trigger `local_track` (`TRIGGER_TRACK`): every frame perception does not
  detect the target asks the local VLM (its frames do not grow the absent
  streak, so no paid cold search follows tracking at once), while
  `_local_first()` takes it
  (after `local_max_unable` unable answers the usual triggers run); its
  question is never handed to the cloud (`cloud_ok=False`), and the absent
  edge (`_last_confirmed`) still moves. `confirm_arrival(frame,
  local_ok=False)` and `confirm_cloud(frame, guarded=True)` count
  `stats.guarded`.
* `brain/arrival.py`: `ArrivalCheck(clock=...)` (explore with
  `sight_from="perception"` passes its clock; without one the rule reads
  perception alone). The bearing is perception's detection, else this
  frame's located local yes (a body bearing); a local "no" misses; a frame
  with neither, inside `LOCAL_FRESH_S` (3.0 s) of the streak's last counted
  frame and with a local frame in the streak, holds the last readout after
  re-reading the lidar at its bearing (a range past the radius or an edge
  ends the hold; not judged when panned). A local answer more than `LOCAL_FRESH_S` after
  the streak's last frame starts it again; the agent calls `refresh()`
  after every identity check, so a slow or hanging cloud does not restart
  the streak (3.47's `arrived_unconfirmed` still comes). The readout's
  `source` is `local` if any frame of the streak was.
* `brain/agent.py`: a `source: local` arrival calls the confirmer with
  `local_ok=False` (a confirmer without that keyword is not called: the
  arrival stays `checking`, unconfirmed). A local-led arrival within
  `LOCAL_REFUSED_RADIUS_M` (0.5 m) of one the cloud refused is refused again
  unpaid. `control/mission_runner.py`:
  `_guarded_confirm(frame, local_ok=False)` waits for a local question still
  in flight (`TieredVision.wait_local_free`, its local phase, within the
  step's deadline less the cloud guard or half of it; still out: undecided,
  asked next frame, never a cloud outage), returns unpaid and undecided if a
  Stop landed during any of its waits, and calls `TieredVision.cloud_only_confirmer()`
  (`confirm_cloud(guarded=True)`); 3.53's late check does the same for a
  stored `source: local` arrival. A confirmer without `local_ok` leaves a
  local arrival `checking` (unconfirmed), never refused.
* `brain/explore.py`: `_update_track()` (after each `_choose`) sets the
  tier's tracking through `local_track_fn` (the runner hands over
  `set_local_track`: the agent's `vision_fn` is the guarded wrapper) for
  `LOCAL_FRESH_S` after a located yes, and while standing within the
  approach band of a sighting only the local VLM made -- never while an
  arrival is arrived, checking or refused. A local-only yes more
  than `LOCAL_FACE_DEG` (15 deg) off centre, or off centre with no range, is
  faced first (`_glimpse`) and not ranged (`_sighting`); a fresh yes that is
  a sighting pre-empts a pending look-around; `_close_in()` turns by the
  landed yes's bearing when perception is silent; between answers, from the
  first yes, it WAITs at most `LOCAL_WAIT_MAX_S` (6 s) a stretch, near the
  target too (a ranged sighting restarts the stretch, so only an unranged
  park runs out). A landed yes counts as seen on every tick, an escape's
  included. Where a glimpse's turns were given up (refused, backed off),
  `_glimpse_spent` stops a local yes from starting them again until the
  robot has moved `APPROACH_MOVED_M` away (amendment 3).
  `_sighting()` and the pan (`_pan()`) are read once a tick.
* Evaluation only: `FrameReportedPipeline(target, blind=True)`
  (`SIM_PERCEPTION_BLIND=1` on the brain server) never reports the target.

**Explore's learned goal choice (3.65 phase 2).** `explore_goal_policy`
(`brain:` block; `rule` | `learned` | `network` | `rooms`, default `rule`; the
`EXPLORE_GOAL_POLICY` environment variable overrides it for live runs; validated by
`load_brain_config`) reaches `MissionRunner(explore_goal_policy=...)`, which
gives `ExploreAgent` a `goal_policy` from `brain/goal_policy.goal_policy_for`
(None for the rule). With one set, `_next_frontier` calls `_learned_goal`:
every frontier not reached twice and available in the retry book, and every
view gap not already looked at or stood at, is pooled; `features()` gives
each ten values (path, turn, is-view, frontier size, view area, unknown and
camera-unseen shares within 1 m, retry failures, times reached, is-view x
goals sent), each scaled to about 0..1; `LinearGoalPolicy.best()` sends
the highest `w . f` (ties keep the rule's order). Turn is measured as the
FACE turn measures it (`atan2(dx, -dy)`). When nothing is allowed, the
boxed/cooling/searched logic runs as under the rule, with its skip counts.
The weights live in `brain/explore_goal_weights.json` with their feature
names, and loading refuses a mismatch. Training is
`evaluations/explore-365/train.py` (cross-entropy method: 24 samples, best
6 refit, 20 iterations, 40 training houses per iteration, selection every 4
on usable 100-199); `harness.py` adds metres (each succeeded goal's
true-floor path plus the verbs' odometer) and the reward, a crash scoring
-100 so it can never beat a miss. `tests/test_goal_policy.py` pins that
the rule written as weights sends the rule's goals. `network`
(amendment 2) is `NetGoalPolicy`: the same pool and features, scored
`w . f + MLP(f)` (MLP 10 -> 32 -> 32 -> 1, tanh) by a numpy forward pass
from `brain/explore_goal_net.json` (the linear weights plus each layer's
`W`, `b`). It is trained by PPO in `evaluations/explore-365/train_net.py`
(torch, trainer only): goals drawn from softmax(scores) while training,
each mission's advantage its reward against the mean of 8 rollouts of the
same house, the MLP's last layer starting at zero so training starts at the
linear policy.

`rooms` (3.96.C) is `RoomGoalPolicy`, 3.96's arm F-large: the same pool,
whose ten features `_pool` extends with seven room features
(`goal_policy.room_features`, `ROOM_FEATURES`) when the policy says
`rooms`. The map is resampled to 0.30 m (`coarse()`: occupied if any cell
is, free if half are), cut into rooms by `brain/room_labels.segment()`, and
each candidate gets its room's newness (never stood in, by
`ExploreAgent._trail`: one pose per 5 cm square, `TRAIL_M`, noted with the
room in `_note_room`), area, whether it is another room than the robot's,
the pool's share in that room, and -- only where the map carries a
`room_labels` layer -- the room type's prior for the target
(`_goal_types`; the prior is the training houses', in the weights file). A
find mission's map has no labels, so those three read 0. Scored
`w . f + MLP(f)` (17 -> 32 -> 32 -> 1, tanh) in float32 exactly as the test
run's `evaluations/goal-learn-396/model.py` `score_np`, argmax with a
non-finite score last; parameters and prior in
`brain/explore_goal_rooms.json`, copied unchanged from the test run's
checkpoint (`tests/test_goal_policy_rooms.py` pins that, and the score).
`evaluations/goal-learn-396/wired.py` checks the flag's missions against the
harness's F-large with every label withheld, mission for mission. A whole
choice costs ~0.3 s p90 on the Jetson (3.96 criterion 6, failed: bar 0.1 s),
during which the robot stands still. Under ask mode the published
candidates carry all 17 features by name, so the choice log keeps them.
Known limits: the prior knows only the ten training targets and ProcTHOR's
four room types (any other target or label reads `type_known` 1 with a
prior of 0, a case training saw only where the prior was 0), and the trail
is never re-anchored, so after a loop closure or a fresh map an old point
can mark the wrong room as stood in.

**Explore asks a person (3.88 A).** With `explore_ask` on (brain config,
or a start's own `explore_ask`), `MissionRunner` gives `ExploreAgent` a
`GoalAsk` (timeout `explore_ask_timeout_s`, 60 s) and a `choice_sink`. Then
`_choose_next` calls `_open_ask` where the rule or `_learned_goal` would
send: the same pool as 3.65 (`_pool()`, frontiers first in the rule's order,
then view gaps), published as `status.explore.ask` -- each candidate `{id:
"<choice>.<index>", kind, x_m, y_m, path_m, rank, features}` (features by
`FEATURES` name, rounded to 4 places) -- with `default` the rule's pick (the
pool's first) or, under a learned policy, its `best()`. Nothing is sent and
the step is a WAIT; choice time (`choice_ms`) ends at the opening. Later ticks
reach `_answer_ask` from `_next_frontier`: a pick taken by
`GoalAsk.pick()` (`person`), or the default once `timeout_s` has passed
(`rule_timeout`), is sent by `_send_pooled` with `picked_by` and `pick` on
its `goal_sent` event, and once nav2's goal is accepted, recorded
(`_record_choice`: choice, step, pose, every candidate's features, default,
pick, by, `waited_s`; the runner adds `mission_id` and `house`) -- a record
is a choice carried out. A send refused before nav2 saw it is sent
again on later ticks, and each attempt re-checks the retry book; a pick no
longer available withdraws the choice. A tick that does not come back to the
choice withdraws it unrecorded (`ask_withdrawn`) if it acted (any result but
WAIT), ran a queued verb (`_do_pending` marks it) or left one queued, or
saw the target (`_sighting`); a plain wait elsewhere leaves it open with
its ids. While waiting, `_probe_authority()` centres the camera
every `ASK_PROBE_S` (1 s): it moves nothing, and a person's Stop latch or a
person holding the robot is refused by name and ends the mission as the
next goal would on `dev`; the mission's own goal holding the robot or a
veto is ignored. Events: `ask_opened`, `goal_picked`, `ask_withdrawn`.

Arrival reads the scan in the body frame using `LIDAR_X_M` from
`robot/safety.py`; its value is in the canonical chassis table,
[platform](../platform/ENGINEERING.md) "Parameters and configuration".

## Procedures

**Run the policy suites** (free, no models, no network):

```bash
pytest tests/test_agent.py tests/test_mission_agent.py tests/test_object_search.py \
       tests/test_vision_policy.py tests/test_tiered.py tests/test_bearing_turns.py \
       tests/test_arrival.py tests/test_arrival_confirmation.py tests/test_goal_pose.py tests/test_memory.py tests/test_rooms.py -q
```

The sweeps in `tests/test_bearing_turns.py` and `tests/test_arrival.py`
run hundreds of in-process missions; expect them to dominate the time.

**Print the R1 A/B:** `python -m tests.demo_hold_bearing_ab` (about 50 s).
It builds the scaled house since handoff 5b -- it had been building the
starter house around the scaled-house coordinates `test_bearing_turns.py`
moved to in 3.32. **Recorded 2026-10-03**, 16 starts, 60 steps, cells closed
/ reversals: detector every frame, sized 6.36 / 0.2 (hold-bearing the
same), quarter turns -0.29 / 12.3; detector 1 frame in 3, sized 4.48 / 0.2,
with hold-bearing 4.95 / 0.2, quarter 0.69 / 8.2. R1's conclusion holds:
sized turns dominate, and dead-reckoning adds about half a cell only when
the detector misses frames; `tier_hold_bearing` stays off.

**Run the free explorer:** `python -m tests.demo_active_search`. It builds
`ObjectSearchAgent` with the world `world/factory.py` gives the robot (as
`robot/server.py` does) and `min_distance_cm=30`. Expected, starter house:
`FOUND: red backpack in the hallway, at step 60` -- the reference hunt
(`tests/data/frontier_trace_centred.json`, 61 steps). **Failure signatures:**
`NOT FOUND after 150 steps` with no `Blocked` lines means the agent got no
world (the right-hand rule wanders); a run of `Blocked LEFT ... (scan_footprint)`
lines means a refused turn is being chosen again (handoff 5a, fixed
2026-10-03; `tests/test_refused_turn_loop.py`).

**A refused move is not offered again until a move executes** (handoff 5a).
`MissionAgent.decide()` drops from its options any of FORWARD, LEFT, RIGHT
refused since the last executed movement (`_refused_since_last_move()`; a
STOP or a camera peek does not clear it). With nothing left it falls to the
boxed-in fallback (`ConstrainedAgent`'s stuck-breaker). Before this, the
side ray cleared a side at `side_clearance_cm` while the pivot guard (3.19)
refused the swept corner, and the agent chose the same refused LEFT 130
times in one demo run.

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
`arrived` or `refused`. On a tiered mission with a scan, `refused` means the
lidar judged an arrival and the cloud did not confirm the target; `identity`
carries the cloud's reason, and the twin's Arrival row shows it in the
alert colour.

## Verification

| Test file (count 2026-10-02) | What it pins, with recorded numbers |
|---|---|
| `tests/test_bearing_turns.py` (17) | Runs in the **scaled house** since 3.32 (`HOUSE = "scaled_house"`). R1: sized turns arrive from off-axis while quarter turns are the defect (relative bars). R1b: every search start sees the target within 12 steps and arrives. R1c: at 90% per-frame detection >= 95% of missions arrive. Spin guard counts degrees; stuck -> `blocked`; spin named a spin. **3.32 recorded only that these pass in the scaled house**; the numbers below are history |
| `tests/test_arrival.py` (14) | Uses `tests/test_bearing_turns.py`'s `_build()`, so the scaled house. **Recorded 2026-10-01 (PLAN 3.32, guarded verbs):** of missions that arrived, 69/69 (perfect detection), 689/689 (90%) and 677/689 = 98.3% (80%) end `found` (bar 95%); 0 false arrivals in 69 / 690 / 690; farthest `found` 0.51-0.52 m from the target's centre (bar 0.60 m, so 0.08 m of margin). **Re-measured 2026-10-02 with the arrival confirmation (handoff 1a) and a cloud that reports `target_visible`:** identical -- 69/69, 689/689, 677/689, 0 false, farthest 0.507/0.515/0.515 m. Also: not judged without scan, with a panned camera, or without local perception; two frames needed; range read at the bearing; edges refused |
| `tests/test_arrival_confirmation.py` (9) | Handoff 2026-10-02 1a, scaled house, cloud faked: the right object ends `found` after one `arrival_confirmation` call; a cloud that disagrees refuses the arrival (ends `blocked`, at most 2 confirmation calls); a false-positive run (absent, absent, detected...) cannot end `found` and the same run with an agreeing cloud does; the call is counted; at the call cap it refuses without calling. Spec review 3: under `async_cloud` the confirmation never overlaps a trigger call (peak concurrency 1 with a 0.2 s cloud); a `found` mission's `status.tier.stats` counts it; the paid step is logged `[cloud: arrival_confirmation]`; a refused mission's final `status.arrival` keeps `identity` and its end line does not say "obstructed" |
| `tests/test_refused_turn_loop.py` (3) | Handoff 5a, starter house: a refused turn is never chosen again from the same spot (was 130 times); with the world the active-search demo finds the backpack in 61 steps (the reference trace); with no world the boxed-in fallback runs instead of a refused LEFT alternating with STOP |
| `tests/test_tier_tree.py` | 3.94: the tree module's statuses; on 10,000 seeded random states the tree's "what to do this frame" gives exactly the frozen code's scene and state (`tests/tiered_reference.py`), every rung exercised; "when to ask" (3.94.B) the same trigger and state on 10,000 random frame sequences, every trigger fired; "who to ask" (3.94.C) the same scene, state, stats and model calls through the whole `__call__` on 2,500 random runs of 1-10 frames (sync and async, local failures, cloud failures, the cap, calls in flight, a Stop mid-frame); `tier_tree` (3.94.D): the shipped yaml is today's values, sixteen bad trees refused at load (the arrival's band, the scan outside the bounds, a fractional turn, a required rung left out among them), explore's band and bounds kept, the band cap pinned to the arrival's, and order, numbers and `local_yes_at` each take effect; p90 under 1 ms a frame; `describe()` reads settings live; a ladder that chooses nothing raises |
| `tests/test_tiered.py` (92) | triggers and hysteresis, call cap, staleness floor, async dispatch and epoch drop, an async failure reaching B3.2, landed verdicts shown once, local bearing beats a stale cloud goal, spin guard never overrides a sighting, timing on the worker |
| `tests/test_review_376k3.py` (5) | 3.76 K3: a failing cloud (not fatal, explore) hands a frame's question back to the local tier; no frame question goes out beside an arrival identity call still running, and it is asked once that ends (a candidate's edge too); close-in's refusals before an accepted goal do not count after it, and an estimate that flips still trips the bound |
| `tests/test_vision_policy.py` (39) | visible is not found; absent `obstacle_ahead` is `unknown`; room guess backfill and `searched_rooms`; the policy does not peek; replay missions |
| `tests/test_review_376l2.py` (6) | 3.76 L2: the same view on a shifted map origin adds no seen cells, and on a half-cell origin the seen centres map back to exactly the cells the camera saw; one room named two ways is one room, `unclear`/`unknown` none; the found frame is not turned away from; a room-label seed off the grid owns no floor; a Stop after the local identity check is undecided, not a refusal |
| `tests/test_agent.py`, `tests/test_mission_agent.py`, `tests/test_object_search.py` | constrained loop, frontier preference, look-around scan |
| `tests/test_goal_pose.py` (12), `tests/test_memory.py` (9), `tests/test_rooms.py` (7) | anchoring under rotation, honesty without range; memory completion; room matching |

**Starter-house history** (2026-09-25, before 3.32 moved the sweeps; kept
only so older plan entries can be read, not as today's record): R1 4.06 cells
closed / 0.6 reversals vs -1.23 / 4.8 for quarter turns; R1b 171 starts,
100% found / 100% arrive; R1c 98.3-98.6% arrival at 90% and 80% detection
over 690 missions each; 3.11 arrival 69/69, 676/678, 676/678, farthest
`found` 0.386 m. The R1 figure was re-measured in the scaled house on 2026-10-03 ("Print the R1 A/B" above). PLAN
3.5 records the demo's every-frame row falling from 4.06 to 3.69 once stuck
detection landed. That was also measured in the starter house.

**Checklist for a change:** measure through `MissionRunner` -> agent ->
`robot/safety.py` -> backend, never a harness that turns the robot itself;
pair any stability metric (reversals, `median_command_run`) with a progress
metric (distance closed, arrival); if a step size changes, re-check every
threshold counted in steps.

## Known gaps

- **Explore's ask mode (3.88 A), kept as is after review:** the pool is
  frozen at the opening and sent up to `explore_ask_timeout_s` later with
  only the retry book re-checked, not the map (a mover or a SLAM shift in
  the meantime is nav2's abort and the retry rule, as for any goal); a pick
  the brain took can still be dropped unrecorded if the next tick acts first
  (a sighting), after the twin said it was taken; in process the timeout
  runs on sim time (live, on wall time); the pause is not bounded by the time
  limit, so an unattended ask mission runs up to 60 s longer per choice; a
  choice can offer dozens of candidates (61 in `home_first_floor`'s live
  check, mostly view gaps). Ask mode is for the plain `explore` policy (a
  find or a survey) only: a tiered search's questions would run on through
  a pause, and skipping them would hand its arrival the simulator's free
  scene (review round 2), so a tiered start with `explore_ask` is a 400.
- The room-level planner is not built; `MissionMemory.as_context()` feeds
  nothing.
- `tier_hold_bearing` is off; a 1-in-3 detector closes only ~1.6 cells and
  needs a range-anchored goal pose (R1, R1b).
- **Tiered phone walks end `max_steps` when they arrive** (P7e), by design:
  arrival needs the lidar, and a landed cloud `target_reached` is never
  applied ([architecture spec](../../policy/ARCHITECTURE.md), "The cloud
  confirms identity at arrival; the lidar decides distance").
- Arrival's panned-camera refusal reads `perception.pan_deg`, which is 0 for
  every sim frame: `sim/mock_robot.py`'s frame carries no `pan_deg`,
  `FrameReportedPipeline` builds `Perception` without one, and the sim's
  detection bearings are already camera-relative (`GridWorld.view_angle()`).
  So the refusal works on real frames only. Harmless today: `VisionAgent`
  never peeks and 3.20 centres the camera at mission start.
- **Local false positives steer.** One detected frame steers and the
  hysteresis gates only triggers (see "What the hysteresis does and does not
  gate"). Accepted by the 2026-10-02 decision; only `found` is guarded, at
  arrival.
- **A refused arrival leaves the robot parked.** The tier keeps steering at
  the object the cloud rejected, so the mission ends in front of it --
  `blocked` after 17-18 steps in both of `tests/test_arrival_confirmation.py`'s
  refused runs, having paid for one confirmation. Nothing turns the robot
  away from a rejected object.
