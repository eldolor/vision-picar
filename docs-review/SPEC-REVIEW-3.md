# Spec review, third pass -- all 15 domains

Reviewed 2026-10-03 against `dev` at `50b2293`, following
`docs/SPEC_REVIEW_PROMPT.md`. This pass came right after the three commits
that built the 2026-10-02 decisions:

- `d549878`: a stop ends a nav2 goal (handoff 1b);
- `6e4b11e`: bridge failures mark ROS down (1d);
- `50b2293`: the cloud confirms identity at arrival (1a; closes 1c).

Nobody had reviewed those commits. They were the first priority.

**Method:**

- Five fresh reviewers took three domains each. None of them had written the
  commits.
- Each one's full notes are in `docs-review/spec-review-3/` (A-E). The notes
  hold every probe, quote and count. This file merges and orders them.
- Everything was read-only. The probes ran in-process against fakes, with no
  ports, no ROS stack and no paid calls.
- **CONFIRMED** means a reviewer reproduced it with a safe command or a fake.
  **UNCONFIRMED** means it was established by reading only.
- About 555 claims were checked.

The file is named `SPEC-REVIEW-3.md` rather than `SPEC-REVIEW.md`, because
`HANDOFF-2026-10-02-spec-review.md` cites the first report's § and fix
numbers.

## 1. Summary

The specs were edited carefully: every test count, constant and line
citation in the edited domains checks out, apart from three moved line
numbers. The three commits behind them, however, close less than their
"Met" rows claim. **The worst defect is a regression of 3.23 that came in
with 1b (CONFIRMED in-process).** A nav2 goal causes a brain mission to be
refused `preempted`. The runner then ends the mission and calls
`POST /stop`, and the stop now cancels the very goal that won. The loser of
an arbitration ends the winner's drive, which is exactly what safety ARCH
rule 2 exists to prevent. On 1a, the "unchanged sweep" proves less than it
seems: the sweep's fake cloud answers identity from the same ground truth
that fired the arrival. The real model's verdict at arrival range has never
been measured, and no live mission has been run through the brain's API
(CLAUDE.md §7 item 2).

## 2. Linter results

`python tools/spec_lint.py` reports 0 errors and 0 warnings. All five
reviewers re-ran it. There is nothing to judge.

## 3. HOW in architecture specs

There are few leaks; the earlier passes cleaned these specs well.

- **policy ARCH:217-220.** "It is asked once more only after the arrival rule
  has stopped holding." That is the re-ask mechanism (`_identity_refused`),
  one of several reasonable ways to bound cost. Keep the commitment ("a
  refused arrival is not paid for again while the robot stays there") and
  move the reset condition to policy ENG, which already states it at
  :76-77. [C]
- **operations ARCH:196-200.** "Until the wheel plugin's start-up race is
  fixed (HANDOFF item 2a) ..." is a temporary workaround tied to a handoff
  number. Keep "container last, and why". operations ENG:238-248 already
  carries the workaround. [E]
- **body ARCH:212.** The row "Stop while a nav2 goal is active" sits in the
  body's own failure table, but the body does not cancel goals; body ENG
  :64-68 says so. This is a boundary error as much as HOW. Point to safety's
  row instead. [A]
- **cloud-vision ARCH:169-171.** "The brain accepts only a literal true ...
  the twin reads them by truthiness" is per-caller detail. It is already in
  ENG Known gaps; keep only the decision in ARCH. [D]
- **Numbers left in place on purpose**, because each is an acceptance bar or
  evidence for a decision:
  - safety: 2 cm, 18.0 cm, 95%;
  - simulator: 98.3%, 0.90 m doors;
  - platform: $86, 250 ms, 1 GB;
  - operations: $159 and $110;
  - motor-board: 3.3 degrees;
  - ros: 40-150 ms;
  - policy: 0.60 m and 95%.

## 4. Restatement in engineering specs

- **safety ENG has no prose for 1d.** The commit names safety as the owner,
  but the mechanism is written only in ros ENG:63:
  - `ros_up()` = a fresh actuator post AND `bridge_up()`;
  - `_send()` marks the bridge down;
  - the probe is lazy and runs at most every 0.5 s;
  - the first call after a mark always answers false.

  safety ENG has only a parameter row and a test row. Add the mechanism under
  Interfaces and have ros ENG link to it. [A]
- **The `BRIDGE_PROBE_*` row is duplicated word for word** in safety
  ENG:156 and ros ENG:157. The constants live in `robot/ros_drive.py`, whose
  other constants are canonical in **body ENG** (:160-162). Make body ENG
  canonical and link from the other two. [A, B]
- **body ENG says nothing about `bridge_up()`.** Body owns
  `robot/ros_drive.py`, but its `RosDriveRobot` row and "The ROS drive stop"
  never mention `bridge_up()`, `_mark_bridge_down()`, `_probe()`, or the fact
  that step 3's zero posts now set and clear the mark. control-api ENG:24
  omits `bridge_up()` too. [A]
- **ros ENG:141** re-describes the stop's cancel mechanism. Trim it to the
  ROS-side fact (`DELETE /goal`) plus a link. [B]
- **policy ENG:283-290 (P7e)** still restates the 1c decision and then says
  "Nothing to build". Reduce it to the code-prose item and link the
  decision. [C]
- **The metrics-delete hazard is written out in full four times**: operations
  ENG:299-303, recordings ENG:367-372, and recordings ARCH:104-109 and :281.
  Make recordings ENG canonical. [E]
- **platform ENG:57 describes itself wrongly.** It says it "only adds pass
  rules" to `tools/jetson/README.md`, but it holds the only complete G4
  command set. Say it is canonical for G4 until the README catches up. [E]

## 5. Verification failures

### 5.1 The three commits

**From 1b (`d549878`): a stop ends a nav2 goal.**

| # | Spec claim | Code | Status |
|---|---|---|---|
| V1 | safety ARCH:189-190, rule 2: "Stop claims nothing. Otherwise the loser of an arbitration takes the robot back by giving up". 3.23: a goal excludes other autonomous drivers | A goal refuses the brain `preempted` (`robot/server.py:350-358`). The runner then goes `_finish(PREEMPTED)` -> `_safe_stop()` (`control/mission_runner.py:468-477`, `:790-796`) -> `RemoteRobot.stop()` (`control/remote_robot.py:106-110`) -> `POST /stop` -> `_cancel_goal_quietly()` (`robot/server.py:681-693`). Probe: goal accepted; brain FORWARD `preempted`; brain `/stop`; then `bridge.goal is None`, cancelled 1 | **CONFIRMED** (the server leg in-process; the runner leg by reading) [A] |
| V2 | safety ARCH:299 and body ARCH:212: "the wheels stop and the goal is ended ... **Met**"; ros ARCH:311: "a hung bridge delays only the cancel ... Met"; world ARCH:220-222 | **(a)** Under `drive: direct` with `WORLD_MODE=ros` there is no hold, so nav2's next `/wheels` post as `ros` is allowed until the cancel lands. A probe read (3.0, 3.0) rad/s 50 ms after `/stop`. **(b)** A failed cancel (bridge hung past its 2 s timeout, or dead) is logged and dropped (`robot/server.py:688-693`), and nav2, which holds the goal, drives on. Under `drive: ros` it resumes after `STOP_HOLD_S`. **(c)** A goal still `pending` is not cancelled: `bridge.py:303-311` cancels only an accepted handle, and `_on_goal_response()` (`:279-290`) then makes it `active` | (a) and (b) **CONFIRMED** with fakes; (c) UNCONFIRMED [A, B] |
| V3 | safety ENG:123-124 and the comment at `robot/server.py:689-690`: "the robot is already stopped" / "an unreachable bridge has no goal to resume" | The goal lives in nav2's `bt_navigator`, not in the bridge's HTTP thread; see V2 | UNCONFIRMED for a dead bridge node [A, B, D] |
| V4 | safety ARCH:186 "Stop is never arbitrated"; control-api ARCH:59 | `POST /action {"action":"STOP"}` is arbitrated (`robot/server.py:538-540`): the brain is refused `preempted` during a goal, and a person's STOP verb leaves the goal `active`. Shipped clients use `/stop` | **CONFIRMED** [A] |
| V5 | twin ARCH:234-235 and FEATURES.md:720: a non-zero D-pad move cancels a goal | Only the bridge does that, on a `twin-dpad` twist (`bridge.py:453-455`), which happens only under `drive: ros`. Under `drive: direct` with the ROS world, and on the G3 fallback, `/action` never cancels a goal | UNCONFIRMED [D] |
| V6 | `service/slam/README.md:255`, which ros ENG calls "the one table of failure signatures": "Expected today: a stop pauses a goal ... the rule is undecided" | Built in `d549878` | CONFIRMED [B] |

**From 1d (`6e4b11e`): bridge failures mark ROS down.**

| # | Spec claim | Code | Status |
|---|---|---|---|
| V7 | "A failed send to the bridge marks ROS down" (safety ARCH:246, ENG:156; ros ENG:63) | `_send()` catches every `httpx.HTTPError`, including `HTTPStatusError` (`robot/ros_drive.py:165-176`). So the bridge's **400** for an unknown driver (`bridge.py:443-446`; unnamed, `teleop`, `teleop-operator`) marks ROS down while ROS is alive, and so does a **401** from a wrong secret. The unauthenticated `/health` probe then marks it up again, so the state flaps | CONFIRMED at the wrapper; the server-level race is UNCONFIRMED [A, B] |
| V8 | ros ARCH:307: with the bridge dead and the plugin alive, "the row above applies" (a person drives direct, ROS's writer is gone) | `/wheels` from `ros` is accepted without asking `ros_up()` (`robot/server.py:614-626`). A live plugin and nav2 can still write while a person's verbs run `direct-fallback`: two writers, against ros D6 | UNCONFIRMED [B] |
| V9 | `service/slam/README.md:254`: `drive.ros_up` false with the container up means the plugin went silent, so restart it | It can now also mean that one send to the bridge failed. `/health` publishes neither half on its own, and a restart is the wrong remedy | CONFIRMED by reading the AND [B] |
| V10 | operations ENG:319-321 and handoff 4e: "a dead container shows only as `drive.ros_up`" | A dead bridge now shows there too | CONFIRMED [E] |

**From 1a (`50b2293`): the cloud confirms identity at arrival.**

| # | Spec claim | Code | Status |
|---|---|---|---|
| V11 | policy ARCH:301 and perception ARCH:258 "**Met**"; policy ENG:245, the re-measure is "identical" | `_quiet_cloud` (`tests/test_bearing_turns.py:85-112`) sets `target_visible` from the frame's synthesised detections, the same ground truth that fired arrival. The confirmation is therefore tautological in every sweep. A live Sim-tab mission now sends a raycaster render to Opus. Real pixels at 0.40 m or less are unmeasured, and no live mission was recorded | CONFIRMED (the oracle); the live behaviour is unmeasured [C, D, E] |
| V12 | policy ARCH:255-257: "at most one in flight ... never block on it" | `confirm_arrival()` is synchronous and ignores `_inflight` (`brain/tiered.py:753-760`). Under the shipped `async_cloud` a probe measured a peak concurrency of **2** | CONFIRMED [C] |
| V13 | policy ENG:71-73: the call is counted in `cloud_calls` and `triggers` | It is counted on the object. But the frame's `_tier` snapshot is taken before the call, and a `found` mission ends on that tick. So `status.tier.stats` and the metrics row (`control/metrics_client.py:73-74`) miss it: the probe read 3 against 4 on the object | CONFIRMED [C] |
| V14 | policy ARCH:26-27: "the twin's readouts (turns, tier counters, arrival)" | Nothing in `web-twin/app.js` reads `status.arrival`, and `index.html:1436-1492` has no Arrival row. A refused arrival therefore shows as `blocked: ... the way ahead is obstructed ... nav2`, and the paid confirmation step is logged `[reactive tier, no cloud call]` (`control/mission_runner.py:838-841`). That contradicts FEATURES.md:687 and CLAUDE.md's "paid steps are marked" | CONFIRMED (probe and grep) [C, D] |
| V15 | policy ARCH:217-220: an arrival that "cannot be asked" is not asked again | A failed call raises `VisionUnavailable` before the refusal flag is set, so it is asked again and counts against B3.2 (it ends `failed` at 3). The behaviour is right; the sentence is wrong | CONFIRMED [C] |
| V16 | policy ENG:114: `identity` is carried "on an arrival" | Only on the frame that made the call. Later refused frames take the early-return branch (`brain/agent.py:357-360`), so the final `status.arrival` has no `identity` | CONFIRMED [C] |
| V17 | perception ARCH:15-17: "The tiered arrival rule does not yet honour that" | It does now, and the same spec's :103-109 says so. :147-151 points at a renamed heading and says "once built" | CONFIRMED [C] |
| V18 | policy ARCH:14, :44, :61, :156-158 and :186-189, and the contract rows: arrival described as local only ("ends `found` **locally**") | It needs the cloud's yes too. The call belongs to the mission agent, not the arrival check (`brain/arrival.py:68-70`: "this module never calls out") | CONFIRMED [C] |
| V19 | mission ENG:50-55, :163-164 and :40: the tick, the optional `vision_fn` attributes, the threads | `_guarded_confirm` is missing (`control/mission_runner.py:700-714`): a second guarded cloud call in a tick, plus the `confirm_arrival` attribute and a second vision-call thread | CONFIRMED [C] |
| V20 | cloud-vision ARCH:26-29 ("calls only on a trigger", "get back one move") and :133-145 ("Arrival is its own field": `target_reached`); ENG:56 lists the readers of `target_visible` | The tiered policy now asks an identity question on the arrival frame and decides `found` on `target_visible` | CONFIRMED [D] |
| V21 | policy ARCH:246: "one paid cloud call per arrival" | A synchronous tier whose trigger fires on the arrival frame pays twice for one frame (2 of 12 missions in the probe) | CONFIRMED [C] |
| V22 | policy ENG:123-126: signatures | `confirm_arrival()` and `arrival_confirm_fn=None` are missing | CONFIRMED by `inspect` [C] |
| V23 | operations ENG §Mission metrics and ARCH:217 | Since `50b2293`, `stats.cloud_calls` includes the arrival call. Release-over-release metrics will show a step that no spec explains | CONFIRMED [E] |
| V24 | simulator ARCH:157-158: "the sim tests ... arrival" | The identity half is an oracle in every sweep, and a real model-on-render call in a live mission | CONFIRMED [D] |

### 5.2 Other mismatches

| # | Spec claim | Code | Status |
|---|---|---|---|
| V25 | CLAUDE.md:209-211 lists "one user decision (tiered arrival on a wrong object)" as open; the `brain/arrival.py` repo-map entry (:385-389) and the P7e row (:176) give the pre-1a rule | That decision is decided and built | CONFIRMED [E] |
| V26 | PLAN-ros-alignment 3.33 criterion 2: "same verdict as the laptop, CLIP P within 0.01" | Nothing performs that comparison: `tools/jetson/setup.sh` asserts only the device; `bench_perception.py` compares nothing; neither README §3 nor platform ENG step 4 does it | CONFIRMED [E] |
| V27 | world ENG:237 and :240; ros ENG:282: line citations into `robot/server.py` | Moved by today's code: now `:863-883` (or `:850-883`), `:378-393` and `:579-583` | CONFIRMED [A, B] |
| V28 | world ENG:71: `DELETE /world/goal` answers the bridge's `{cancelled: bool}` | `RosWorld.cancel_goal()` returns `.json()` whatever the status, so a 401 comes back as a 200 carrying an error body (`world/ros_world.py:240-241`) | UNCONFIRMED [B] |
| V29 | operations ENG:154, the secret guard line | It only echoes "STOP", so the deploy carries on if the block is pasted whole | CONFIRMED [E] |
| V30 | operations ENG:304-305 and ARCH:62, :234: `build.sh` "warns" | The text is printed unconditionally and never checks the variables (`service/lambda/build.sh:82`) | CONFIRMED [E] |
| V31 | operations ENG:39: cache class `day` | `public, max-age=86400` | CONFIRMED [E] |
| V32 | cloud-vision ENG:59: Robot view's obstacle cue reads the flag by truthiness | Mixed: the OBSTACLE banner is strict (`app.js:3046-3047`); the HUD row and the haptic/audio cue are truthy | CONFIRMED [D] |
| V33 | twin cost hint, `app.js:2005-2007`: three paid triggers | Five can spend, including `staleness` and `arrival_confirmation`. `frames_per_call` now includes confirmations | CONFIRMED [D] |
| V34 | PLAN 3.33 step 5: "Clone from GitHub" | The canonical procedure pushes over SSH (`tools/jetson/README.md:30-42`) | CONFIRMED [E] |
| V35 | CLAUDE.md:21 and :550: test counts | 1667 collected | CONFIRMED [E] |

## 6. Pair, boundary, coverage and duplication

- **Pair (safety).** The 1d decision has no ENG prose (§4). 1b's ENG text
  omits the limits in V1-V4.
- **Pair (world).** world ARCH D9 says the cancel is built, but world ENG
  has no pointer or test for it. Add `tests/test_stop_cancels_goal.py` to
  world ENG and to ros ENG's checklist (:235-237). Add it to control-api
  ENG Verification as well.
- **Pair (mission).** mission ARCH's B3.2 row covers the confirmation call;
  mission ENG does not (V19).
- **Boundary: who owns the identity call.** policy's components table gives
  arrival to the arrival check. The code splits it three ways: the decision
  is in the mission agent, the call is in the tier, and the timeout and
  budget are in the runner. Give "identity confirmation" to the mission agent
  in policy ARCH and say the arrival check stays call-free.
- **Boundary: the arrival readout and the log label.** Policy owns the state
  and mission publishes it, but nobody renders it (V14). The `[cloud:
  <trigger>]` label is produced by mission, yet no spec states the rule;
  grep finds none in `docs/`. Its canonical home should be mission ENG.
- **Boundary: goal cancel on a person's movement.** Safety owns the rule.
  ros D6 states the D-pad cancel with its condition, while the twin restates
  it without the condition (V5). The twin should link rather than
  paraphrase. **Decision for safety:** should a person's `/action` cancel a
  goal, the way `/stop` now does?
- **Coverage.**
  - Four `tools/*.py` files still have no owner, as in pass 2:
    `contact_sheet.py`, `label_prepass.py`, `steer_check.py` and
    `yoloworld_crops.py`.
  - `service/vision_analyze/.coverage` is tracked in git. Untrack it and
    ignore it.
  - Today's commits added no unowned modules.
- **Duplication.**
  - `BRIDGE_PROBE_*` (§4).
  - The unknown-driver gap appears in ros ENG:278-284, safety ENG:301-303
    and README:256. All three need V7's correction. Make ros ENG canonical.
  - The metrics-delete hazard (§4).
  - G4 commands: platform ENG is canonical until `tools/jetson/README.md` §4
    catches up.
  - The truthiness-of-flags fact: cloud-vision ENG Known gaps is canonical.

## 7. Fix list

Ordered by how badly each defect would mislead someone building or operating
the robot.

| # | Fix | Effort |
|---|---|---|
| 1 | **Decide and build:** a preempted mission's teardown stop must not cancel the goal that preempted it (V1). Candidates: cancel only on a stop from a driver that is not the one a goal preempted; skip the stop in `_finish(PREEMPTED)`; or condition the cancel on who stops. Write the rule into safety ARCH "Who drives". Test: goal active, brain preempted, runner finishes, goal still `active` | M |
| 2 | **Close what 1b claims (V2, V3):** hold non-zero `ros` `/wheels` at zero after a `/stop` until the cancel is acknowledged, in both drive modes. Retry a failed cancel. Cancel a goal that is still `pending` when nav2 accepts it (`_on_goal_response`, on `cancel_reason`). Extend the fakes with a pending phase, a nav2 tick right after `/stop`, and a failing cancel. **Until then**, change the "Met" rows (safety ARCH:299, body ARCH:212, ros ARCH:311, world ARCH:220-222) to "met when the cancel succeeds", and delete the false comment at `robot/server.py:689-690` | M (code) / S (specs) |
| 3 | Rewrite `service/slam/README.md:254-256`: the stop row (V6), what `ros_up` false now means (V9), and the unknown driver (V7) | S |
| 4 | perception ARCH:15-17 and :147-151: delete "does not yet honour", and fix the dead heading reference and "once built" (V17) | S |
| 5 | `RosDriveRobot._send()`: mark the bridge down only on `httpx.TransportError` and 5xx, never on 4xx (V7). Then correct ros ENG:140 and :278-284 and safety ENG:301-303 | S |
| 6 | While `bridge_up()` is false, refuse or zero non-zero `ros` `/wheels` posts, so a person on the fallback is the only writer (V8). Publish `drive.bridge_up` and the plugin's post age on `/health` (V9) | M |
| 7 | 1a's evidence: qualify "Met" and the re-measure line as "with an oracle cloud; live verdict unmeasured" (V11). Run one live tiered mission through the brain's API, as CLAUDE.md §7 item 2 requires: Sim tab, paid, about 3-5 calls. Add `target_visible` accuracy at arrival range to an Open question; replaying the rig walks' arrival frames would measure it. **Decision for simulator and policy:** should sim frames be confirmed from geometry, by analogy with 1.12? | S each; the run is paid |
| 8 | Async overlap (V12): either `confirm_arrival` waits for or reuses the in-flight call, or policy ARCH names it as the one deliberate blocking, overlapping call | S (doc) / M (code) |
| 9 | Make a refused arrival visible (V14, V16): an Arrival row on the Remote brain panel with a UI test that fails first; a `blocked` message that does not blame the path when `arrival.state == "refused"`; `[cloud: arrival_confirmation]` on the paid step; `identity` carried on later refused frames | M |
| 10 | Count the confirmation on surfaces people read (V13, V23): refresh `_tier.stats` after the call. Add a note to operations ENG metrics and ARCH:217 | S |
| 11 | Stop semantics in the specs (V4, V5): write "the stop route" in safety ARCH:186 and control-api ARCH:59, and add the `/action` STOP row to safety ENG. Qualify the twin's D-pad claim and FEATURES.md:720 with "under `drive: ros`". **Decision for safety:** should a person's verb cancel a goal? | S |
| 12 | Update CLAUDE.md (V25): no open decision; the arrival rule includes the cloud confirmation; status rows for 1a, 1b and 1d; the test count (V35) | S |
| 13 | Bring the policy, mission and cloud-vision text up to 1a (V15, V18-V22): the arrival sentences, the components and contract rows, the signatures, `_guarded_confirm` in mission ENG, "at least one call" or reuse of a same-frame `target_visible`, and both consumers in cloud-vision ARCH and ENG. The 4b note: a string "true" now costs liveness (a sticky refusal, then `blocked`), not safety | S |
| 14 | Simulator ARCH:157-158 (V24) | S |
| 15 | Give PLAN 3.33 criterion 2 a procedure (V26): a `bench_perception.py --compare <laptop.json>`, or a one-frame check with expected values; add it to platform ENG step 4 and README §3 | S/M |
| 16 | Write 1d's mechanism in safety ENG; move the `BRIDGE_PROBE_*` row to body ENG; put `bridge_up()` in body ENG and control-api ENG (§4) | S |
| 17 | Make the stop's cancel single-flight. Check the bridge's status in `RosWorld.cancel_goal()` and `get_goal()` (V28; this joins handoff 4d) | S |
| 18 | Under a synchronous tier, `tick_timeout_s` (30) holds two `vision_timeout_s` (20) waits, so a slow call can be misread as a hung loop. Record the constraint in mission ENG:189, or enforce it in `load_brain_config` | S |
| 19 | Operations: make the secret guard stop the run (V29); describe `build.sh`'s text as unconditional (V30); the cache header (V31); "dead container or bridge" (V10, and handoff 4e) | S |
| 20 | Small items: the line citations (V27); cloud-vision ENG:59 (V32); the twin's cost hint (V33); PLAN 3.33 step 5 (V34); drop "also" at twin ARCH:237; FEATURES §3 on STOP; the handoff §1 preamble ("which says 'decided fix not yet built'"); world ARCH:46's diagram caption; platform ENG:57; operations ARCH:196-200; policy ARCH:217-220's re-ask mechanism to ENG; P7e's restatement; the metrics-hazard copies; owners for the four tools; untrack `.coverage` | S |

## 8. What was checked and found correct

So nobody re-checks these. Each reviewer's §(g) has the full list.

- **Linter:** 0 errors and 0 warnings, re-run by all five.
- **Test counts:** every count cited in all 15 ENG specs matches `pytest
  --collect-only`, more than 80 files in all. Examples: test_stop_cancels_goal
  4, test_ros_fallback 8, test_arrival_confirmation 5, test_arrival 14,
  test_tiered 92, test_robot_contract 134, wall linters 18.
- **Constants and config:** all values quoted in the safety, body,
  control-api, ros, world, motor-board, twin, simulator and policy ENG
  tables match the code and `config/robot.yaml`. Also matching: the
  `twist_mux` and controller timeouts, the nav2, slam_toolbox and
  collision_monitor yaml, and the motor-board fake's constants.
- **What today's code does that the specs describe correctly:**
  - **The stop's cancel:**
    - `/stop` stops first, then starts a `stop-cancel-goal` daemon thread;
    - only `/stop` cancels, and the watchdog does not;
    - the bridge's `DELETE /goal` exists, is authenticated, and cancels an
      accepted handle;
    - `/health` is unauthenticated on the real bridge;
    - only `RosWorld` has `cancel_goal`.
  - **The bridge-down rule:**
    - `ros_up()` ANDs in `bridge_up()`;
    - `bridge_up()` never blocks, and its probe is single and rate-limited;
    - the verb that finds the bridge dead is refused and not retried.
  - **The arrival confirmation:**
    - `confirm_arrival` confirms only on `_navigate.target_visible is True`,
      and `brain/navigate.py` fills that field on real replies;
    - at the cap it refuses without calling;
    - `MissionRunner` passes `_guarded_confirm`, and a failure counts
      against B3.2 and ends the mission `failed` at 3;
    - only `tiered` reaches the confirmation (not vision, frontier or
      drills);
    - `_identity_refused` resets correctly, and at 90% and 80% detection
      each run paid exactly one call;
    - refused runs end `blocked` after 17-18 steps;
    - the `tiered.py` comment above `DEFAULT_STEER_ON_SIGHT` now matches
      the code.
- **Wall linters:** 11 duplicates and 13 bridge routes, both exactly at
  budget. No new wall duplicate was introduced.
- **G4:** unaffected. The chain suite runs `frontier`, and 18 live tests
  are still collected.
- **Replay:** it never judges arrival, because the replay and teleop bodies
  have no scan. So recordings ENG's "one per trigger" still holds.
- **docs/README.md:** it indexes all 15 domains, and its description of the
  spec system matches CLAUDE.md §2 and decision 0001.
- **Earlier corrections:** pass 2's citations into `robot/safety.py`,
  `robot/interface.py`, `hardware_robot.py`, `picar_sim_hardware.cpp` and
  `bridge.py` still hold.

## 9. Resolution

**Fixes 1 and 2: built 2026-10-03.**

- **Fix 1 (V1).** The user chose the rule: a `/stop` from an autonomous
  driver other than `ros` (the brain) zeroes the wheels and spares a goal in
  progress. A preempted mission's exit stop now leaves the goal that beat it
  running.
- **Fix 2 (V2, V3).** A person's stop now runs a loop that cancels, re-reads
  the goal, and re-cancels until nav2 reports it neither pending nor active.
  `ros` wheel commands are held at zero for that time and for 0.6 s after.
  A new goal supersedes the loop, and a verb on the ROS drive path lifts the
  hold. The bridge also cancels a goal that a cancel reached while it was
  still `pending`, once nav2 accepts it. That bridge change is
  **unverified live**: it is container code, and the server's loop covers
  the case without it.
- **Tests.** `tests/test_stop_cancels_goal.py` grew from 4 tests to 13. All
  five new V1/V2 cases failed against `50b2293`.
- **Specs.** Updated: safety ARCH and ENG (the two new decisions, the
  mechanism and the constants), body ARCH (the row is now a pointer), ros
  ARCH and ENG, world ARCH, control-api ENG, and `service/slam/README.md`
  §4 (V6).

**Fixes 3-6: built 2026-10-03.**

- **Fix 3.** `service/slam/README.md` §4 now says what `ros_up` false
  means. There is a new row for a dead bridge, and the unknown-driver row
  says its 400 does not mark ROS down.
- **Fix 4.** perception ARCH's intro and trade-off now say the cloud
  confirms identity at arrival.
- **Fix 5 (V7).** `RosDriveRobot._send()` marks the bridge down only on a
  transport error or a 5xx. A 4xx is the bridge answering, so it clears an
  earlier mark instead. `tests/test_ros_drive.py` +6; the three 4xx cases
  failed before the fix.
- **Fix 6 (V8, V9).** While the bridge is down, the actuator's `/wheels`
  posts still prove the plugin alive, but they are answered
  `ros_unavailable` (unlogged) and move nothing. `/health` publishes
  `drive.bridge_up` and `drive.ros_post_age_s`. `tests/test_ros_fallback.py`
  +2, both of which failed before the fix.
- **Part of fix 16.** The 1d mechanism is now written in safety ENG ("ROS
  liveness"), and the `BRIDGE_PROBE_*` row has moved to body ENG.

Fixes 7-15 and 17-20 are open.
