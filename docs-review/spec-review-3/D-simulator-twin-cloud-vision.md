# Spec review, third pass -- reviewer D: simulator, twin, cloud-vision

Scope: `docs/{simulator,twin,cloud-vision}/ARCHITECTURE.md` and their
engineering specs, against the code at `50b2293` (dev). Read-only.
Linter: `0 error(s), 0 warning(s)`.

Commands used (all offline): `tools/spec_lint.py`; `pytest --collect-only`
on the twin/sim/vision test files; `pytest -q tests/test_stop_cancels_goal.py
tests/test_arrival_confirmation.py` (9 passed); `pytest -q
service/vision_analyze/tests` (58 passed); dry imports of `sim.mock_robot`,
`sim.maps`, `vision_core`; and one scratch probe (`scratchpad/arr.py`) that
re-runs `tests/test_arrival_confirmation.py`'s `_run()` with the agreeing
and the disagreeing fake cloud and prints `status["arrival"]`, the tier
triggers, `last_reasoning` and the log tail. Everything not marked
UNCONFIRMED was established by one of these or by grep.

---

## (a) Findings in today's changes

### A1. Twin: "a non-zero D-pad movement cancels [a goal] too" is only true under `drive: ros` with ROS up -- med

- Spec: `docs/twin/ARCHITECTURE.md:232-235` (edited by `d549878`): "the one
  thing it ends is a navigation goal ... and a non-zero D-pad movement
  cancels one too." Same unqualified claim in `FEATURES.md:720` ("a D-pad
  twist cancels it").
- Code: the only goal-cancel on movement is in the bridge, on a non-zero
  `twin-dpad` TWIST (`service/slam/src/picar_bridge/picar_bridge/bridge.py:453-455`).
  A D-pad verb becomes a `twin-dpad` twist only on the `elif by_velocity:`
  branch (`robot/server.py:566-571`, `robot/ros_drive.py:149-168`). Under
  `drive: direct` with `WORLD_MODE=ros` (the very configuration in which 1b
  was found, ~50 ms resume) and on the 3.24 G3 direct fallback
  (`robot/server.py:546-561`), `/action` never touches the goal (grep:
  `cancel_goal` appears in `robot/server.py` only in `/stop` and `DELETE
  /world/goal`). So a person who takes the robot with the D-pad under
  direct drive gets the robot back only while tapping; nav2 resumes when
  they stop (UNCONFIRMED live; by reading the routes). The ros spec states
  it correctly ("under the ROS drive mode ... inside ROS",
  `docs/ros/ARCHITECTURE.md:181-186`).
- Fix: qualify the twin sentence ("under the ROS drive, a non-zero D-pad
  move also cancels one; under direct drive only STOP does") and FEATURES.md
  3. Whether `/action` from a person should cancel a goal the way `/stop`
  now does is a safety-domain decision (owner: safety, "Who drives"); it is
  the same gap 1b closed, one route over.

### A2. A refused arrival is invisible on the twin, and the mission's end message names the wrong cause -- med

- Probe (`scratchpad/arr.py`, disagreeing cloud, scaled house): `status.arrival`
  = `{"state": "refused", "reason": "the cloud did not confirm this arrival;
  not asking again until the arrival ends", ...}`, but `last_reasoning` and
  every log line read `target center -- [reactive tier, no cloud call] no
  trigger (detected) -- steering on local sighting`, and the mission ends
  `blocked: FORWARD refused 5 times in a row by the safety layer -- the way
  ahead is obstructed. Going around it is route planning (nav2 ...)`.
- The twin renders no arrival readout at all: `web-twin/app.js` has no
  consumer of `status.arrival` (grep `\.arrival` -> none), and
  `web-twin/index.html:1436-1492` has no Arrival row. `tests/test_ui.py` has
  no arrival test.
- So the one surface an operator watches reports a wrong-object refusal as
  an obstacle and points them at nav2. The refusal reason exists only in
  `GET /mission/status`'s `arrival`.
- Spec wrong elsewhere because of it: `docs/policy/ARCHITECTURE.md:26-27`
  says "the twin's readouts (turns, tier counters, arrival) are its
  outputs". The twin specs say nothing either way; the twin ENG
  `/mission/status` row (`docs/engineering/twin/ENGINEERING.md:70`) does not
  list which status fields the panel renders.
- Fix: build an Arrival row (state incl. `refused`, `identity.reason`,
  range/streak) with a UI test red first, or correct policy ARCH:26-27 and
  add a twin Known gap. Separately (mission domain), the `blocked` message
  should not blame the path when `arrival.state == "refused"`.

### A3. The paid arrival-confirmation step is logged as "no cloud call" -- med

- Probe (agreeing cloud): step 13's log line and `last_reasoning` read
  `STOP (ok) -- target reached -- arrived -- target centred at 0.355 m, 2
  frame(s) running; [reactive tier, no cloud call] no trigger (detected)
  ...` on the step that made the `arrival_confirmation` call (tier triggers
  `{..., 'arrival_confirmation': 1}`).
- Cause: `TieredVision.confirm_arrival()` (`brain/tiered.py:740-768`) counts
  the call but does not annotate the scene's `_tier`, and
  `MissionRunner._describe()` labels `[cloud: <trigger>]` only from
  `scene["_tier"]["cloud_called"]` (`control/mission_runner.py:838-841`);
  `arrived_scene()` (`brain/arrival.py:190-203`) prepends to the local
  reasoning.
- Claims now false: `FEATURES.md:687` ("paid steps are marked `[cloud:
  <trigger>]` in the log") and `CLAUDE.md` P2 row ("The mission log names
  the paid steps"). The refused case (A2) is the same: the paid step reads
  "no cloud call".
- Fix (owner mission/policy; the twin only renders the line): mark the
  arrival step `[cloud: arrival_confirmation]`, or record it on
  `identity` and have `_describe()` read it.

### A4. The twin's tiered cost hint understates what spends money -- low

- `web-twin/app.js:2005-2007`: "A paid `/navigate` call goes out only on a
  trigger: mission start, a candidate sighting, or a cold search." The tier
  also pays on `staleness` (`brain/tiered.py:96`, pre-existing, seen firing
  in the probe) and now on `arrival_confirmation` (`brain/tiered.py:91`).
  This is the pre-Start spend statement. Not pinned by any UI test (grep
  "cold search" in `tests/test_ui.py` -> none).
- Related: `stats.frames_per_call` (`brain/tiered.py:486-499`), which the
  Deliberation row prints as "1 per N", now includes confirmation calls,
  so it is no longer the same quantity 6.1's 4-6x measured. One call per
  arrival; small, but say so where the ratio is quoted.

### A5. Cloud vision's caller description is stale now that `target_visible` gates `found` -- med

- `docs/cloud-vision/ARCHITECTURE.md:26-29`: the brain's policies "send a
  frame and a target and get back one move. The tiered policy calls only
  on a trigger." Since `50b2293` the tiered policy also asks the move route
  an identity question on the arrival frame and acts only on
  `target_visible` (`brain/tiered.py:762-763`, `is True`). The code itself
  says it is "not a trigger the policy fires on its own"
  (`brain/tiered.py:88-91`).
- `docs/cloud-vision/ARCHITECTURE.md:133-145` ("Arrival is its own field")
  presents `target_reached` as the arrival signal. For the tiered policy
  that field is now ignored (1c) and the field that decides `found` is
  `target_visible`. The arch should name both consumers: the vision policy
  and Robot view use `target_reached`; the tiered policy uses
  `target_visible` on an arrival frame as the identity verdict (policy owns
  the rule; cloud-vision should state that its visibility flag carries
  that weight).
- `docs/engineering/cloud-vision/ENGINEERING.md:56` lists
  `target_visible`'s readers as `brain/navigate.py` and the twin; add
  `brain/tiered.py` `confirm_arrival()` (reads the already-normalised
  `_navigate.target_visible`).
- **Consequence for 4b (not a re-report).** On the brain path the string
  "false" can no longer produce a wrong `found`: `brain/navigate.py:170`
  normalises with `is True` and `confirm_arrival()` re-checks `is True`.
  The type gap now bites in the OTHER direction: a string `"true"` (or any
  non-bool) reads as not visible, the arrival is refused, and the refusal
  is sticky (`brain/agent.py` `_identity_refused`, cleared only when the
  arrival rule stops holding), so the robot parks in front of the right
  object and the mission ends `blocked` after `stuck_after` with the
  misleading message of A2. That is a liveness cost, not a safety one;
  coercing in the service (4b) would also fix it. Whether any allow-listed
  model ever emits string booleans is UNCONFIRMED (nothing recorded).
  The twin's truthiness reads (`app.js:3024`, `:3320`, `:3087`) are
  unchanged by today's work.

### A6. The identity verdict's accuracy at arrival range is unmeasured -- med (open question)

`target_visible` at <= 0.40 m from a floor-height camera is now the gate
on `found`. No measurement of `/navigate`'s visibility answer on
arrival-range frames exists in the cloud-vision specs or the policy spec's
numbers (the 3.11 re-measure used a faked cloud). The rig walks end at
arrival and have adjudicated `labels.json`, so replaying their last frames
through the default model would measure it cheaply (recordings owns
replay). Add to `docs/cloud-vision/ARCHITECTURE.md` Open questions (or
policy's, with a link).

### A7. Simulator: arrival's identity half is an oracle in every sweep, and a real VLM-on-render call in every live Sim-tab mission -- med

- Every in-process sweep and test supplies `tests/test_bearing_turns.py:85-111`
  `_quiet_cloud`, which sets `target_visible` from the frame's SYNTHESISED
  detections -- i.e. a geometry-perfect identity oracle. The 3.11 re-measure
  ("identical -- 69/69, 689/689, 677/689") therefore measures the distance
  half only.
- A live Sim-tab tiered mission (brain with the real `/navigate`) now pays
  one real call on a raycaster render and ends `found` only if the model
  reports a flat billboard as the target. That is exactly the simulator's
  open question "Does the lit renderer help a vision run?"
  (`docs/simulator/ARCHITECTURE.md:268-269`), and it now decides a live sim
  mission's outcome.
- `docs/simulator/ARCHITECTURE.md:157-158` ("The sim tests the consumers of
  perception: triggers, steering, arrival and arbitration") should say the
  identity half of arrival is stubbed in sweeps and is a real VLM-on-render
  call live. Decide (simulator + policy) whether, by analogy with 1.12, a
  `metadata.source == "sim"` frame should be confirmed from geometry, or
  whether the live sim deliberately exercises the real call.

### A8. Small wording defects from `d549878` -- low

- `docs/twin/ARCHITECTURE.md:237-238`: "Whether a person's goal should rank
  as a person is also open." The edit removed the open item "also" referred
  to; drop "also".
- `FEATURES.md` section 3 (tap-to-goal, `:716-725`) does not say STOP now
  ends a goal; twin ENG `:15-16` says section 3 was checked 2026-10-02. Add
  to the FEATURES drift Known gap or fix FEATURES.
- Twin ENG `:53` could say the cancel is best-effort: a failure is logged
  and the goal is not ended (`robot/server.py:688-694`).

### A9. Cross-domain lead for the safety/ros reviewer -- low, UNCONFIRMED

`_cancel_goal_quietly()`'s docstring (`robot/server.py:689-690`) reasons
"an unreachable bridge has no goal to resume". nav2's goal lives in
`bt_navigator`, not the bridge; in 1d's own scenario (bridge dead, plugin
and nav2 alive) a stop's cancel fails and the goal may resume after
`STOP_HOLD_S`. Not tested by `tests/test_stop_cancels_goal.py` (its hung
cancel case checks the stop's latency, not the goal). Twin ARCH:232-234
states the stop ends a goal unconditionally.

### A10. The twin shows nothing of the ROS fallback -- low

After `6e4b11e` a dead bridge sends a person's D-pad through
`direct-fallback` (`robot/server.py:546-561`, `result.via`). `web-twin/app.js`
reads neither `/health`'s `drive` block nor `via` (grep `ros_up|via|drive`
-> none), so an operator cannot see they are driving on the fallback. 4e
covers only `control/health.py`. Add a twin Known gap or a readout.

---

## (b) HOW in the architecture specs

Prior passes cleaned these specs well; nothing new of substance.

- `docs/cloud-vision/ARCHITECTURE.md:169-171` "the brain accepts only a
  literal true for both flags, while the twin reads them by truthiness":
  per-caller implementation detail, already duplicated in ENG Known gaps
  (`:343-355`). Keep "callers differ, and only the service can close it for
  all of them" in arch; the per-caller detail belongs in ENG. Low. (It is
  also now incomplete, A5.)
- Numbers checked and judged legitimate commitments/evidence: simulator
  ARCH 98.3%/98.6%/98.4% (3.9 acceptance), 0.90 m doors (the house's
  definition), "0 runs under 18 cm" (bar); cloud-vision ARCH "60% of 80
  frames", "$110 of $159" (evidence for rejections); twin ARCH "about 3 s"
  (dated evidence), "40 px chevron" (motivating bug).

## (c) Restatement in the engineering specs

- Twin ENG `:53` and `:58` carry decision provenance ("decided by the user
  2026-10-02", "the safety domain's rule") rather than linking; harmless,
  short. No other restatement found in the three ENG specs.

## (d) Verification failures (spec vs code)

| # | Spec claim | Code | Sev |
|---|---|---|---|
| D1 | twin ARCH:234-235, FEATURES.md:720 -- D-pad movement cancels a goal (unqualified) | only `bridge.py:453-455` on a `twin-dpad` twist, reached only under `drive: ros` (`robot/server.py:566`); none under direct/fallback | med (A1) |
| D2 | policy ARCH:26-27 -- twin readouts include arrival | no `status.arrival` consumer in `web-twin/app.js`; no row in `index.html:1436-1492` | med (A2) |
| D3 | FEATURES.md:687, CLAUDE.md P2 row -- paid steps marked `[cloud: <trigger>]` | arrival-confirmation step logs `[reactive tier, no cloud call]` (probe; `mission_runner.py:838-841`, `tiered.py:740-768`) | med (A3) |
| D4 | cloud-vision ARCH:26-29 -- tiered "calls only on a trigger", "get back one move" | also an identity call at arrival (`tiered.py:740-768`) | med (A5) |
| D5 | cloud-vision ENG:56 -- `target_visible` readers are navigate.py and the twin | also `tiered.py:762` | low (A5) |
| D6 | cloud-vision ENG:59 -- "Robot view's obstacle cue (`if (result.obstacle_ahead)`) reads it by truthiness" | mixed: the OBSTACLE banner is strict (`app.js:3046-3047`, `=== true`); the HUD OBS row (`:3027`) and haptic/audio (`:3641`) are truthy. Name which | low |
| D7 | twin app.js tiered hint (not a spec, but the page's spend promise) -- three triggers | five can spend (`tiered.py:85-96`) | low (A4) |
| D8 | simulator ARCH:157-158 -- sim tests arrival | identity half stubbed by a geometry oracle in sweeps (`test_bearing_turns.py:85-111`) | med (A7) |

## (e) Pair / boundary / coverage / duplication

- **Boundary (arrival readout).** Policy owns the arrival state, mission
  publishes it (`docs/engineering/mission/ENGINEERING.md:150`), the twin
  owns rendering it -- and does not. Owner of the fix: twin (row) or policy
  (stop claiming it). A2.
- **Boundary (log label).** `[cloud: <trigger>]` is produced by mission
  (`_describe`) from policy's `_tier`; FEATURES/CLAUDE describe it as twin
  behaviour. Canonical home should be mission ENG; it is not stated in any
  spec today (grep `\[cloud:` in `docs/` -> none). A3.
- **Boundary (who cancels a goal).** Safety owns the rule; ros D6 states the
  D-pad cancel with its condition; twin restates it without the condition.
  Canonical: safety ARCH "Who drives" + ros D6; twin should link, not
  paraphrase. A1.
- **Duplication.** The truthiness-of-flags fact is in cloud-vision ARCH
  (`:163-172`, `:330-334`) and ENG (`:56`, `:59`, `:343-355`) and HANDOFF 4b.
  Canonical: ENG Known gaps; ARCH should keep only the decision/open
  question.
- **Coverage.** No uncovered code in the three domains' directories.
  `web-twin/README.md` overlaps twin ENG procedures (run/deploy/test) but
  ENG names it as the how-to; fine.

## (f) Proposed fixes

| # | Fix | Sev | Effort |
|---|---|---|---|
| 1 | Qualify twin ARCH:232-235 and FEATURES.md:720 -- the D-pad cancels a goal only under `drive: ros`; raise to safety whether a person's `/action` should cancel a goal like `/stop` (A1) | med | S (spec) / S-M (code, if decided) |
| 2 | Add an Arrival row to the Remote brain panel (state incl. `refused`, identity reason) with a UI test red first; or correct policy ARCH:26-27 and add a twin Known gap. Make the `blocked` end message not blame the path when the arrival was refused (A2) | med | M |
| 3 | Label the arrival-confirmation step `[cloud: arrival_confirmation]` in the mission log; state the labelling rule in mission ENG (A3) | med | S |
| 4 | Cloud-vision ARCH Purpose + "Arrival is its own field": say the tiered policy reads `target_visible` on an arrival frame as its identity verdict, and `target_reached` is ignored there; ENG:56 add `confirm_arrival` (A5) | med | S |
| 5 | Record in cloud-vision (or policy) Open questions: `target_visible` accuracy at arrival range is unmeasured; measure by replaying the rig walks' arrival frames (A6) | med | S (spec) / M (measure) |
| 6 | Note on 4b in the handoff/ENG Known gaps: on the brain path the gap now costs liveness (a string "true" makes a sticky refusal -> `blocked`), not safety (A5) | low | S |
| 7 | Simulator ARCH:157-158 and Open questions: arrival identity is an oracle in sweeps and a real VLM-on-render call live; decide whether sim frames should confirm from geometry (A7) | med | S (spec) / S (decision) |
| 8 | Twin cost hint (`app.js:2006-2007`) to name every spending trigger, or say "on a trigger, and once at arrival"; note `frames_per_call` now includes confirmations (A4) | low | S |
| 9 | cloud-vision ENG:59 -- say the banner is strict and the HUD row and haptic/audio cue are truthy (D6) | low | S |
| 10 | Drop "also" at twin ARCH:237; FEATURES section 3 to say STOP ends a goal; twin ENG:53 "best-effort, failure logged" (A8) | low | S |
| 11 | Pass to safety/ros: does a stop end a goal when the bridge is dead but nav2 lives? (A9, UNCONFIRMED) | low | S (check) |
| 12 | Twin Known gap or readout for the ROS fallback (`drive.ros_up`, `via: direct-fallback`) (A10) | low | S / M |

## (g) Checked and found correct (about 95 claims)

**Today's changes, twin.** `/stop` cancels on a daemon thread after
`robot.stop()` and returns without waiting (`robot/server.py:680-694`;
`tests/test_stop_cancels_goal.py` 4 collected, passing); the page sends
`x-driver: twin-dpad` on `/stop` (`app.js:476-477`); tap-to-goal sends no
`x-driver` and `world_goal_set()` arbitrates as `DRIVER_ROS`
(`robot/server.py:863-875`, `app.js:713`); a refused goal shows "Could not
send the goal: <reason>" (`app.js:714-716`); goal states map to words incl.
`canceled -> cancelled` (`app.js:689-690`), so a stopped goal reads
"goal: cancelled"; "nothing in the page changed" for 1b holds.

**Today's changes, tier/arrival as rendered.** `refused` is a new
`arrival.state` (`brain/arrival.py` `REFUSED`); `identity` appears on the
readout with `confirmed/cloud_called/reason/cloud_reasoning` (probe);
`arrival_confirmation` lands in `stats.triggers` and `cloud_calls`
(probe; `tiered.py:756-757`); the twin's Deliberation row reads
`stats.cloud_calls`/`stats.frames` only and is not broken by the new
trigger key (`app.js:1652-1669`); the corroboration row is unaffected
(`claims` not incremented by `confirm_arrival`); a tiered phone walk is
NOT_JUDGED and never asks (policy ENG:114 consistent).

**Twin ENG.** All 20 constants in the Parameters table match `app.js`
(grep, values and names incl. `NGROK_HOST` regex, `TURN_STEPS_DEG`,
`DRIVER_DPAD`); `DRIVER_LOCAL_BRAIN` still defined (`app.js:469`, Known gap
true); app.js ~5,400 lines (5380); test counts 110 / 5 / 3 / 10 / 11 all
match `--collect-only`; Remote brain rows present as listed
(`index.html:1436-1492`).

**Simulator ENG.** `MAX_SUBSTEP_CELLS` 0.1, `MAX_SUBSTEP_RAD` 5 deg,
`FOOTPRINT_SKIN_CM` 0.2, `PAN_ANGLE_RAD` pi/2, `SIM_PERCEPTION_RANGE_CELLS`
3.0 (`sim/grid_world.py:141-181`); `CELLS_PER_SECOND_AT_FULL_SPEED` 2.0,
`DEFAULT_CELL_CM` 30.0, `ENCODER_COUNTS_PER_REV` 660, `LIDAR_RANGE_M` 12.0,
`WHEEL_MAX_RAD_S` = 15.0 by dry import (`sim/mock_robot.py:61-112`);
`FPV_FOV` 60 deg, `FPV_MAX_DIST` 14, `FPV_STEP` 0.05, 320x200, JPEG 82,
both colours, `VISIBLE_EXTENT_RAYS` 9 (`sim/renderer.py:92-236`);
`MOVER_KEEPOUT_M` 0.20, `hop_s` 1.0 (`sim/movers.py`); starter house
13x10, robot (2,2) E, backpack (10,7); scaled house 27x14, robot (6,4) E,
backpack (23,5), sofa at (2,2) so the move example is right (dry import of
`sim.maps.build_world`); 3.9 numbers 98.3% +/- 1.0 at 90%/80% vs 98.6/98.4
before (`PLAN-ros-alignment.md:778-779`); `test_solid_objects.py` uses
(8.5, 7.5).

**Cloud-vision ENG.** Default navigate model Opus 4.5; the 7-entry
allow-list in picker order with the stated labels; 6 prompt variants;
`DEFAULT_PROMPT_VARIANT` `default`; region pins for both Fable 5.1 ids to
us-east-1; guidance model Nova Lite (dry import of `vision_core`);
`maxTokens` 500/300/300/500 (`vision_core.py:290,812,968,1047`);
`MAX_IMAGE_BYTES` 5 MiB and the default `ALLOWED_ORIGINS`
(`app.py:152-161`); `target_reached` forced `is True` and false when not
visible (`vision_core.py:856-858`); brain reads all three flags with
`is True` (`brain/navigate.py:169-178`); twin truthiness reads at
`app.js:3024`, `:3087-3088`, `:3320`; service suite 58 collected and
passing, `tests/test_vision.py` 9.
