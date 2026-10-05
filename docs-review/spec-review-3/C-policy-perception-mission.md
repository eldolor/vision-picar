# Spec review, third pass -- reviewer C: policy, perception, mission

Scope: docs/{policy,perception,mission}/ARCHITECTURE.md and
docs/engineering/{policy,perception,mission}/ENGINEERING.md, with priority on
commit 50b2293 (cloud confirms identity at arrival). Linter: 0 errors, 0
warnings. Read-only: nothing edited. All probes ran with fakes (no network, no
models); scripts in this scratchpad (`probe.py`, `probe2.py`, `probe3.py`).
"CONFIRMED" = reproduced by a safe command or fake-pipeline run;
"UNCONFIRMED" = established by reading only.

---

## (a) Findings in today's changes (50b2293)

### A1. The published counter and the metrics row miss the confirmation call on every `found` mission -- CONFIRMED, med
`TieredVision.confirm_arrival()` increments `stats.cloud_calls` and
`stats.triggers["arrival_confirmation"]` (brain/tiered.py:756-757), but it runs
inside `MissionAgent._review_scene()` (brain/agent.py:376-377) *after*
`TieredVision.__call__` has already built this frame's `_tier` snapshot.
`MissionRunner.tick()` copies `scene["_tier"]` (control/mission_runner.py:515),
and on `found` the mission ends that tick, so no later snapshot is taken.
Fake run (`tests/test_arrival_confirmation.py::_run(_quiet_cloud)`):
final `status.tier.stats.cloud_calls == 3`, triggers without
`arrival_confirmation`; live object says 4. `control/metrics_client.py:73-74`
builds the metrics row from `status["tier"]`, so the shipped row undercounts
too. The spec claims the call is "counted in `cloud_calls` and in
`triggers["arrival_confirmation"]`" (docs/engineering/policy/ENGINEERING.md:71-73)
and the architecture calls the counter 6.3's watchable number -- true of the
object, false of every surface a person reads for a mission that ends `found`.
On a refused arrival later frames re-snapshot, so the count does appear there.
Fix: have `_review_scene` refresh `scene["_tier"]["stats"]` after the call (or
the runner read `vision_fn.stats` at finish).

### A2. Two cloud calls in flight under the shipped async tier -- CONFIRMED, med
`confirm_arrival()` is synchronous and does not check `self._inflight`
(brain/tiered.py:753-760); `__call__` enforces "one call at a time"
(brain/tiered.py:708-715) only for triggers. With a fake cloud sleeping 0.3 s
and `async_cloud=True`, max concurrency was **2** in both an agreeing and a
disagreeing run (`probe.py`: `async ... maxconcurrent 2`). This contradicts
policy ARCHITECTURE.md:255-257 ("dispatched without blocking the tick, at most
one in flight") with no stated exception, and the code's own reason for the
rule (brain/tiered.py:709-712: stacking on one vision task, "spend twice for
one answer"). Not a crash: TierStats only appends, and `navigate.vision_fn_for`
state is read-only per call (brain/navigate.py:249-261) -- UNCONFIRMED for
thread-safety of `TierStats.record()` under contention. The confirmation also
*blocks* the tick, which the same decision's title ("never block on it")
rejects. Either the architecture names the confirmation as the one deliberate
blocking, overlapping call (eng spec already says "even under `async_cloud`",
ENGINEERING.md:72), or `confirm_arrival` waits for / reuses the in-flight call.

### A3. Synchronous tier pays twice on the same frame -- CONFIRMED, low
When a trigger fires on the arrival frame, the trigger's own scene already
carries `_navigate.target_visible` for exactly that frame, and
`confirm_arrival` calls again. Sync tier over the 12 `CLEAR_STARTS`
(`probe2.py`): 2 of 12 missions made two cloud calls on one frame
(`[1, 3, 12, 12]`, `[1, 8, 17, 17]`). Shipped mode is async
(config/robot.yaml:224), so cost only; but policy ARCHITECTURE.md:246 "One
paid cloud call per arrival" should read "at least one", or the agent should
reuse a same-frame `_navigate.target_visible`.

### A4. Arrival on the Sim tab now puts a raycaster render to the real model -- UNCONFIRMED (not run: paid), med
On a live sim mission `control/brain_server.py:308-315` builds
`FrameReportedPipeline` (ground-truth detections) but the cloud `vision_fn` is
the real `/navigate`. `confirm_arrival(frame)` therefore sends the rendered
JPEG to Opus 4.5 and `found` now depends on the model naming the target in a
flat-shaded billboard render. Every number the commit and specs quote for the
new rule comes from `_quiet_cloud` (tests/test_bearing_turns.py:96-112), whose
`target_visible` is `bool(frame detections)` -- the *same* ground truth that
made arrival fire, so the confirmation is tautological in every sweep. The
re-measure line (docs/engineering/policy/ENGINEERING.md:245, "identical ...
with a cloud that reports `target_visible`") proves the plumbing does not
break arrivals; it measures nothing about refusal rates with a real model.
CLAUDE.md section 7 item 2 asks for at least one live mission through the
brain API; the commit records none. Specs claiming the target is **Met**
(policy ARCHITECTURE.md:301, perception ARCHITECTURE.md:258) should say "met
with a faked cloud; live refusal rate unmeasured", and the policy eng spec
should flag the tautology.

### A5. "Cannot be asked ... not put to the cloud again" is wrong for a failed call -- CONFIRMED, low-med
policy ARCHITECTURE.md:217-220: "If the cloud disagrees, or cannot be asked,
the mission does not end `found`, and the same arrival is not put to the cloud
again". A *failed* confirmation raises `VisionUnavailable`
(control/mission_runner.py:708-714) before `_identity_refused` is set
(brain/agent.py:362-368), so the next tick re-asks; three failures end the
mission `failed` (`probe3.py`: `failed 13 ... arrival confirmation failed:
bedrock 503 confirm attempts 3 vision_failures 3`). That is the right
behaviour (and matches eng ENGINEERING.md:68-69); the architecture sentence
should split "disagrees or is capped -> refused, not re-asked" from "errors
-> counted against the failure budget, re-asked".

### A6. Budget and dead-man: a sync tick can now hold two 20 s waits under a 30 s deadline -- UNCONFIRMED (arithmetic), low
`_guarded_vision` and `_guarded_confirm` each take `vision_timeout_s`
(20.0, config/robot.yaml:306) in one tick; `tick_timeout_s` is 30.0
(config/robot.yaml:318; control/brain_server.py:562). A slow sync trigger plus
a slow confirmation is ended by B3.3 as "brain loop hung" rather than counted
by B3.2 -- the misdiagnosis the mission architecture's failsafe split exists to
avoid. Async (shipped) has only the confirmation in the tick, so fine there.
The mission eng row for `tick_timeout_s` (docs/engineering/mission/ENGINEERING.md:189)
records only the teleop-stall constraint; add "and above two
`vision_timeout_s` under a synchronous tier", or bound the tick's total.

### A7. `_identity_refused` reset -- correct, one readout gap -- CONFIRMED, low
Reset happens on any non-`arrived` readout for a perception-carrying scene
(brain/agent.py:376-379): target lost (`_miss`), off-centre/too-far
(`approaching`), panned/no scan (`not_judged`). While the rule keeps holding
it stays set. At 90% and 80% simulated per-frame detection with a disagreeing
cloud, 10/10 runs paid exactly one confirmation and ended `blocked` in 17-18
steps (`probe.py`, `flaky` rows) -- the flicker re-ask the code permits did
not occur because `stuck_after` ends the run first. Gap: frames after the
first refusal take the early-return branch (brain/agent.py:357-360) which
drops `identity`, so the *final* `status.arrival` of a refused mission has
no `identity`/`cloud_reasoning` (CONFIRMED: `'identity' in s['arrival'] ->
False` for both refused runs). docs/engineering/policy/ENGINEERING.md:114 says
`identity` is carried "on an arrival"; it is carried only on the frame the
call was made. Carry the verdict forward with the refused state.

### A8. Which policies reach `_confirm_identity` -- CONFIRMED correct
`vision` and `frontier` scenes have no `_perception`, so `_review_scene`
returns at brain/agent.py:374-375 before the confirmation. Drills replace the
whole `vision_fn` (control/drills.py:127), so no drill reaches it either.
Only `tiered` does; `_guarded_confirm` finds `confirm_arrival` on
`TieredVision` because `tiered_vision_fn_for` returns the bare object
(brain/tiered.py:1326-1337; brain_server wraps nothing around it).

### A9. B3.2 accounting -- CONFIRMED correct
A confirmation failure is a `VisionUnavailable` from inside `agent.step()`,
caught at control/mission_runner.py:466-467, stops the robot, counts, and ends
`failed` at the budget (probe3). A success resets the count via the normal
tick path (:497). Matches docs/engineering/policy/ENGINEERING.md:65-70.

### A10. Stale sentences the commit missed

| Where | Says | Now |
|---|---|---|
| perception ARCHITECTURE.md:15-17 | "The tiered arrival rule does not yet honour that (see "Detect, then verify")." | false since 50b2293; that section now says the opposite (:103-109). **high** (contradiction in the intro) |
| perception ARCHITECTURE.md:147-151 | a false positive "near the wrong object ... can satisfy the arrival rule and end the mission `found` (see "Where that intent is not yet met" above ...). The decided cloud check at arrival, once built, removes the last of these" | heading was renamed to "At arrival too" (:103), so the link text points nowhere, and "once built" is stale. med |
| policy ARCHITECTURE.md:14 | "the arrival rule that lets a mission end `found` on its own evidence" | needs the cloud's yes too. low |
| policy ARCHITECTURE.md:44 (diagram) | "arrival review (local perception + lidar) -> may rewrite to STOP/found" | no cloud box; the review now calls the cloud. low-med |
| policy ARCHITECTURE.md:61 (Arrival check row) | "'found' from detection + bearing + lidar range" | plus cloud identity; and name who owns the call (the mission agent, not the arrival check -- brain/arrival.py:68-70 "this module never calls out"). low-med |
| policy ARCHITECTURE.md:186-189 | "A mission ends `found` **locally** when the target is detected, centred ... on consecutive frames" | no longer local; add "and the cloud confirms identity (below)". med |
| policy ARCHITECTURE.md:156-158 | the arrival review "rewrites that same frame to a stop and `found` against the cloud's answer" | it now asks the cloud on that frame and needs its yes, so it can override the trigger's *direction* but not its identity. Eng spec already changed to "whatever the trigger's answer said" (ENGINEERING.md:63). low |
| policy ARCHITECTURE.md:255-257 | "never block on it ... at most one in flight" | see A2. med |
| policy ARCHITECTURE.md:281-288 (Contracts) | Mission runner row "the agent calls the vision function"; Cloud vision row "vision step calls the service" | the agent also calls an arrival confirmer through the runner's guard. low |
| policy ENGINEERING.md:123-124 | `TieredVision(...)` "with `close()`, `reset_epoch()`, `set_searched_rooms()`" | also `confirm_arrival()` (CONFIRMED via `dir`). low |
| policy ENGINEERING.md:126 | `MissionAgent(robot, memory, side_clearance_cm=30.0, world=None, min_distance_cm=20.0, vision_fn=None, ...)` | missing `arrival_confirm_fn=None` (CONFIRMED via `inspect.signature`). low |
| policy ENGINEERING.md:93-97 (Triggers) | five trigger outcomes, "`max_calls` caps the total" | `arrival_confirmation` is a sixth key in `triggers` and is capped by the same `max_calls`; point at "Arrival". low |
| policy ENGINEERING.md:235-238 (Read a live mission) | lists `approaching`/`not_judged`/never `arrived` | add what `refused` means and that it appears only on a scan-bearing tiered mission. low |
| policy ENGINEERING.md:246 | refused run "ends `blocked`" | true (CONFIRMED 17/18 steps) but the test asserts only `!= FOUND` (tests/test_arrival_confirmation.py:89). Say "observed", or pin it. low |
| mission ENGINEERING.md:50-55 (tick step 2) | only `_guarded_vision` | `_guarded_confirm` (control/mission_runner.py:700-714) is the second guarded cloud call in a tick, same timeout, same `VisionUnavailable`. med -- the mission domain owns B3.2 and was not touched by the commit |
| mission ENGINEERING.md:163-164 | "Optional attributes the runner uses if present: `set_searched_rooms(list)` and `close()`" | and `confirm_arrival(frame)`. low-med |
| mission ENGINEERING.md:40 (threads) | one `vision-call` thread per `vision_fn` call | a tick can spawn two; under async the confirmation's thread runs beside the `tiered-cloud` worker. low |
| brain/tiered.py:17-31 (code prose) | "The three triggers ... `mission_start` ... the one genuinely blocking call" | already listed as drifted (policy ENG:276-277); now also wrong because `confirm_arrival` is the one blocking call under async. Add to that Known-gaps line. low |

---

## (b) HOW in architecture specs

- policy ARCHITECTURE.md:217-220 -- "it is asked once more only after the
  arrival rule has stopped holding". This is the re-ask mechanism
  (`_identity_refused`), one of several reasonable ways to bound cost
  (per-arrival, per-object, per-mission budget). Keep the commitment ("a
  refused arrival is not paid for again while the robot stays there") and move
  the reset condition to engineering, where ENGINEERING.md:76-77 already has it.
- Otherwise the new text is decision-shaped (rejected alternatives present,
  trade-off stated). Numbers in it (0.60 m, 95%) are acceptance bars and may
  stay.
- No new technology-as-fact. Class/file names in the new architecture
  prose: none (good).

## (c) Restatement in engineering specs

- perception ENGINEERING.md:282-285 correctly links to the policy eng spec
  rather than repeating the mechanism -- good.
- policy ENGINEERING.md:283-290 (Known gaps, P7e) still restates the
  architecture decision ("Decided by the user ... the arrival rule stays the
  only way a tiered mission ends `found` on the car ... by design") and then
  says "Nothing to build". Now that 1c is closed this is no longer a gap;
  reduce it to the code-prose item (brain/arrival.py:6-9, already HANDOFF 4k)
  and link the decision.

## (d) Verification failures (spec vs code)

1. policy ENG:71-73 "counts it in `cloud_calls` and `triggers`" vs
   control/mission_runner.py:515 + control/metrics_client.py:73-74: never
   surfaced on a `found` mission (A1). CONFIRMED.
2. policy ARCH:255-257 "at most one in flight" vs brain/tiered.py:753-760 (A2). CONFIRMED.
3. policy ARCH:246 "One paid cloud call per arrival" vs sync double-pay (A3). CONFIRMED.
4. policy ARCH:217-220 "cannot be asked -> not asked again" vs
   brain/agent.py:362-368 + mission_runner.py:708-714 (A5). CONFIRMED.
5. policy ENG:114 `identity` "on an arrival" vs brain/agent.py:357-360 (A7). CONFIRMED.
6. policy ENG:126 / :123-124 signatures (A10). CONFIRMED.
7. perception ARCH:15-17 and :147-151 vs brain/agent.py:376-377 (A10). CONFIRMED by code + fake run.
8. policy ARCH:27-28 "the twin's readouts (turns, tier counters, arrival) are
   its outputs" vs web-twin/app.js: no reader of `status.arrival` (grep for
   `.arrival` finds only a comment at :3342). The arrival readout exists only
   in the JSON status. Not raised in passes 1-2. CONFIRMED by grep, low-med:
   with `refused` now a state an operator needs to see, either render it or
   drop "arrival" from that sentence.
9. mission ENG:50-55, :163-164 (A10). CONFIRMED.

## (e) Pair / boundary / coverage / duplication

- **Boundary: who owns the identity call.** Policy arch row "Arrival check"
  (:61) implies the arrival check; code puts the decision in the mission
  agent (brain/agent.py:353-368), the call in the tier (brain/tiered.py:740),
  and the timeout/budget in the runner (control/mission_runner.py:700). The
  architecture owner is policy (fine), but the components table should give
  "identity confirmation at arrival" to the mission agent and say the arrival
  check stays call-free -- the code already states that split
  (brain/arrival.py:68-70).
- **Pair:** mission ARCHITECTURE.md's B3.2 row ("vision calls error or hang")
  is broad enough to cover the confirmation; its engineering pair is not
  (A10, mission ENG:50-55). Pair agreement needs the eng side updated.
- **Duplication:** the 3.11 sweep numbers (69/69, 689/689, 677/689) now live
  in policy ENGINEERING.md:245 and in the HANDOFF strike note
  (HANDOFF-2026-10-02-spec-review.md:37-40). The spec is canonical; the
  handoff copy is a dated record and fine as such.
- **Coverage:** no new uncovered code; all four changed modules map to policy
  (brain/*) and mission (control/mission_runner.py).
- From 6e4b11e (outside these domains): an autonomous verb refused
  `ros_unavailable` becomes `RobotTransportError` (control/remote_robot.py:215-220)
  and ends the mission via the generic `failed` branch. Mission ARCH's failure
  table covers it under "A body read or move raises anything else"; no
  defect, but the G3 behaviour ("mission ends failed") is documented only in
  safety/ros specs. Optional cross-link. d549878: no effect on these domains.

## (f) Proposed fixes

| # | Fix | Sev | Effort |
|---|---|---|---|
| 1 | perception ARCH:15-17 -- delete "does not yet honour that"; :147-151 -- fix the dead heading reference and drop "once built" | high | S |
| 2 | Refresh the `_tier` stats after `confirm_arrival` (or read `vision_fn.stats` at `_finish`) so status/metrics count the confirmation; add a test that a `found` mission's `status.tier.stats.triggers` includes `arrival_confirmation` | med | S |
| 3 | Decide the async overlap: make `confirm_arrival` wait for / reuse an in-flight call, or record in policy ARCH ("Hold the cloud's goal; never block on it") that the arrival confirmation is the one deliberate blocking call and may overlap one in-flight trigger | med | S (doc) / M (code) |
| 4 | Qualify "Met" (policy ARCH:301, perception ARCH:258) and the re-measure line (policy ENG:245): the sweep's cloud answers from the same ground truth; live refusal rate on renders and on real pixels is unmeasured. Run one live Sim-tab tiered mission (paid) per CLAUDE.md section 7 item 2 and record whether Opus confirms the render | med | S doc / S paid run |
| 5 | policy ARCH:14, :44, :61, :186-189, :156-158, contracts rows -- name the cloud confirmation where arrival is described | med | S |
| 6 | mission ENG:50-55, :163-164, :40 -- add `_guarded_confirm`, the `confirm_arrival` optional attribute, and the second vision-call thread | med | S |
| 7 | policy ARCH:217-220 -- split disagree/capped (refused, not re-asked) from error (budget, re-asked); move the reset condition to eng | low-med | S |
| 8 | Carry `identity` forward on later refused frames (brain/agent.py:357-360) so the final status says why; then ENG:114 is true | low | S |
| 9 | Reuse a same-frame `_navigate.target_visible` on a sync trigger frame, or say "at least one" call per arrival | low | S |
| 10 | mission ENG:189 -- `tick_timeout_s` must exceed two `vision_timeout_s` under a sync tier (or enforce it in `load_brain_config`) | low | S |
| 11 | policy ENG:123-126 signatures; :93-97 triggers note; :235-238 `refused`; :246 "observed" vs pinned; tiered.py docstring line into the existing drift item | low | S |
| 12 | policy ARCH:27-28 -- the twin does not render `status.arrival`; render it (now that `refused` matters) or reword | low-med | S/M |

## (g) Checked and found correct (about 60 claims)

- `REFUSED = "refused"` set only by the agent; brain/arrival.py makes no calls (arrival.py:68-71; ENG:24).
- `TRIGGER_ARRIVAL = "arrival_confirmation"` (tiered.py:91); confirmation counted in `cloud_calls`/`triggers` on the object; cap check refuses without calling (tiered.py:753-755; test passes).
- Confirms only on `_navigate.target_visible is True` (tiered.py:764-765); `brain/navigate.py:170,193-194` does populate `_navigate.target_visible` on real replies, so the check works against the real service shape.
- `MissionRunner` passes `arrival_confirm_fn=self._guarded_confirm` (mission_runner.py:378); guard uses `vision_timeout_s` and maps timeout/error to `VisionUnavailable` (:700-714) -- as ENG:67-70 says.
- Fallback to `vision_fn.confirm_arrival` without a runner (agent.py:359).
- Not confirmed / no confirmer -> `refused`, scene not rewritten (agent.py:364-368, 380-383).
- `_identity_refused` set on refusal, cleared on any non-`arrived` judged frame (agent.py:376-379); re-ask did not occur at 90%/80% detection (10/10 runs, 1 call each).
- Non-tiered policies and drills never reach `_confirm_identity` (A8).
- B3.2: confirmation failure counted, robot stopped each time, ends `failed` at 3 (A9).
- Refused runs end `blocked` after 17 and 18 steps with one confirmation (ENG:304-308) -- CONFIRMED exactly.
- Right object ends `found` after exactly one `arrival_confirmation` across 4 starts (probe), matching ENG:246.
- Test counts in policy ENG verification table: test_arrival_confirmation 5, test_arrival 14, test_tiered 92, test_bearing_turns 17, test_vision_policy 46, test_goal_pose 12, test_memory 9, test_rooms 7 -- all match `pytest --collect-only`.
- `tests/test_arrival_confirmation.py` passes (5 in 1.1 s); `_quiet_cloud` now returns `_navigate.target_visible` (test_bearing_turns.py:109-112).
- The tiered.py comment above `DEFAULT_STEER_ON_SIGHT` (tiered.py:221-232) now matches the code (one frame steers; async landed answer never overrides a local sighting).
- `tier_async_cloud: true`, `tier_max_calls: 0` shipped (config/robot.yaml:224,277; brain_config.py:109,140) as policy ENG:139,146 say.
- `tiered_vision_fn_for` returns a bare `TieredVision` (tiered.py:1326-1337); brain_server adds no wrapper that would hide `confirm_arrival`.
- `ArrivalCheck` keeps returning `arrived` on every frame after the streak completes (arrival.py:118-120), which is what makes "not asked again while the rule holds" work.
- Perception ENG:282-285 (local false positives still steer; `found` guarded) matches code.
- Mission ARCH failure table, B3 split, halt gate, outcome list: unchanged and still consistent with mission_runner.py:440-560, 716-760.
- Linter: 0 errors, 0 warnings.
