# Spec review, second pass -- all 15 domains

Reviewed 2026-10-02 against `dev` at `f92b7df`, following
`docs/SPEC_REVIEW_PROMPT.md`. This pass reviewed the specs as the first
pass's fix agents rewrote them (`docs-review/SPEC-REVIEW.md` §9). That
rewrite had not been checked by anyone.

**Method:**

- Five fresh reviewers took three domains each. None had written or fixed
  the specs it reviewed.
- Their priorities, in order: claims the fix pass made wrong; HOW still in
  the architecture specs; anything the first pass missed.
- Defects the specs already record honestly as Known gaps were not
  re-reported.
- Everything was read-only, apart from dry imports, fake-pipeline checks
  and offline test files. Findings established by reading alone are
  marked **UNCONFIRMED**.
- About 690 claims were checked again.

This pass is written to its own file rather than over `SPEC-REVIEW.md`,
because `HANDOFF-2026-10-02-spec-review.md` cites that report's section and
fix numbers.

## 1. Summary

The fix pass got most of its targets right. Every one of these holds:

- each value in the canonical chassis table (platform ENG);
- the three `bridge_url` rows;
- the source-build table;
- the 3.32 arrival record;
- the deployed-replay gap analysis;
- the new tiered-build section;
- all three canonical links into operations ENG.

**One high finding is in code that nothing had recorded: a stop does not
stop a nav2 goal.** Two reviewers found it independently:

- `POST /stop` only calls `robot.stop()` (`robot/server.py:656-669`).
- `RosDriveRobot.stop()` zeroes the twist_mux inputs but never cancels the
  goal.
- The bridge cancels a goal only on a **non-zero** `twin-dpad` twist
  (`picar_bridge/bridge.py:453-455`).
- nav2 keeps publishing on `cmd_vel/nav`, so the wheels resume once the
  stop hold ends.
- Under `drive: direct` with `WORLD_MODE=ros` they resume on the plugin's
  next post, within about 50 ms.

The safety, ros and twin specs all promise the opposite. The behaviour
predates today's work: before fix 1 a stop under a goal was a ~50 ms pause
as well. This needs a decision (§5, item 1).

**The other high finding blocks G4.** As written, the G4 procedure in
platform ENG can never pass. It starts no brain, does not set the house in
pytest's own environment, does not gate the container on `/wheels` being
usable, and does not name the container `picar-ros`. Each omission forces
a skip, and under the new rule a skip is not a pass.

**One error in today's own code was fixed during this pass.** Fix 1's
`STOP_HOLD_S` was 0.4 s, derived from twist_mux's 0.25 s timeout alone.
twist_mux's timeout and the controller's `cmd_vel_timeout` add up (the
yaml says so), so a stale twist can reach the wheels for about 0.55 s. It
is now **0.6 s** (`robot/ros_drive.py`). The tests pass.

## 2. Linter

`python tools/spec_lint.py` reports 0 errors and 0 warnings, so there is
nothing to rule on. Coverage is unchanged: four one-off scripts under
`tools/` belong to no domain (`contact_sheet.py`, `label_prepass.py`,
`steer_check.py`, `yoloworld_crops.py`).

## 3. Findings that change behaviour or block work

| # | Finding | Where | Severity |
|---|---|---|---|
| H1 | A stop does not stop a nav2 goal (above). | safety ARCH:185, ros ARCH D6:185, :300, twin ARCH:223, body ARCH:209 | high, code |
| H2 | The G4 procedure cannot pass. It needs: a brain on :8001 under `ROUTE_PREFIX=/brain` (`tests/test_ros_chain_live.py:170-174`); `SIM_MAP=scaled_house` in pytest's environment as well (`tests/demo_nav_goals.py:71`); the container started only after `GET /wheels` is usable; and the container named `picar-ros` (`tests/test_ros_chain_live.py:33`). `tools/jetson/README.md` §4 has none of these. | platform ENG:186-196 | high, procedure |
| M1 | Cross-check of today's `STOP_HOLD_S` derivation: fixed in code (0.6 s). The specs still say 0.4 s and give the wrong reason. | body ENG:154, ros ENG:157 | med, fixed in code |
| M2 | **ROS liveness is judged only from the plugin's posts.** If the bridge dies but the plugin lives, `ros_up()` stays true, every verb fails at `_send`, and `/action` refuses even a person `ros_unavailable`. If twist_mux or the controller dies, verbs stall and come back as `safety_distance`. No architecture spec has a row for part of the container dying. UNCONFIRMED. | safety ARCH:228-234, :272; control-api and ros ARCH | med-high |
| M3 | **The cloud's `target_reached` never ends a tiered mission under the shipped asynchronous setting.** A landed cloud scene only sets `_last_cloud_scene` (`brain/tiered.py:788`), and every returned scene is `_local_scene`, with `target_reached: False`. Only `brain/arrival.py` can end it `found`. Confirmed with a fake pipeline: async put the target in `important_objects` on 0 of 20 frames, synchronous on 3. This is the real reason tiered phone walks end `max_steps` (P7e). `brain/arrival.py:6-9` is stale. | policy ARCH:89-92, ENG:89 | med |
| M4 | **The deploy procedure deploys with no authentication.** `build.sh` prints a command that expands `$VISION_SHARED_SECRET` and `$WALKS_SHARED_SECRET`, but the secrets files define `VISION_SECRET` and `WALKS_SECRET`. Pasted as printed, the parameters are empty, which means no auth (`serverless.yaml:66`, `:74`). | operations ENG:148-163 | med |
| M5 | **`target_probe`'s "peak" is read from `r.best`,** which is set only on DETECTED (P >= 0.8). So a 0.000 peak means "never passed 0.8", not "inert prompt", and `--gate` below 0.8 has no effect. `CLAUDE.md` repeats the same wrong reading. | perception ENG:213-214; `control/target_probe.py:65-69` | med, code |
| M6 | **Under a driverless car, every FORWARD is refused**, not only REVERSE: `get_distance()` is 0.0 with no sensors. Motor-board D7 says "a straight verb that overshot keeps its overshoot", and its first-contact procedure sends a `T:1` that the server would clamp. Only turns move. | motor-board ARCH D7:204-207, ENG:183-184, :195 | med |
| M7 | **Unfinished walks are never auto-scored.** The console's `scorePendingWalks()` filters `w.finished && !w.eval` (`control/admin.js:285-297`). | recordings ARCH:235-237, :259, ENG:63 | med |
| M8 | **The walks Lambda's import-failure signature is wrong.** `create_app()` raises `WalkStoreError: S3WalkStore needs a bucket name.` before `_assert_configured()` can run (`walks_handler.py:60`, `walk_store.py:291`). No test covers that guard. | recordings ENG:29, :250 | med |
| M9 | **A restarted robot server can kill the plugin, not just a slow start.** A restarted `HardwareRobot` reports `usable: false` until its first frame. "Recovers by itself" and "a container started first is fine" hold only for `MockRobot`. UNCONFIRMED. | ros ARCH:298, `service/slam/README.md:116-117` | med |
| M10 | **The cloud-vision coercion claim is wrong in both directions.** Three fields are left uncoerced, not two: `obstacle_ahead` is not type-checked either, and the string `"false"` passes as truthy. And `room_guess` becomes `unclear` only when it is not a string or is empty; `"spaceship"` passes through unchanged. | cloud-vision ARCH:148-163, :302, ENG:59, :270 | med |
| M11 | **The local-vision notes miss the tunnel's own vision route.** A page served through the tunnel must use `<proxy>/vision`. "Vision URL = the site" holds only for a page served from CloudFront. | cloud-vision ENG:162-170, twin ENG:107-113 | med |
| M12 | **The B5 start order contradicts the ros runbook.** Operations says the container starts before the robot server. Until handoff 2a lands, the server must start first and the container only once `/wheels` is usable. | operations ARCH:203-204, ENG:221-222 | med |
| M13 | **Platform's bench command fails as written.** It is missing the required `--recordings`, and it must be run with `python -m`. | platform ENG:178-179 | med |
| M14 | **A checklist step does nothing.** control-api ENG says robot routes are covered by `test_serverless_routes.py`. That test covers only the vision and walks apps; robot routes go through the tunnel. | control-api ENG:200, :207 | med |
| M15 | **A promised procedure does not exist.** Platform ENG points to a motor-board "odometry over a measured metre" procedure. There is none, and no udev rule either. | platform ENG:210-211, :74 | med |

## 4. Broken "one canonical home" promises

The first pass promised each duplicated fact one home. These are not yet
true:

- **Chassis constants.** platform ENG:96-98 says the other specs "link here
  rather than restating values". They do not all link:
  - body ENG:141-147 restates the constants with no link;
  - safety ENG:124-127 restates them with no link;
  - policy ENG:159 restates `LIDAR_X_M`;
  - motor-board and ros ENG link, but restate the values too.

  Platform's own list of where the copies live is also incomplete. It is
  missing:
  - `sim/fake_esp32.py` `MAIN_TYPES[2]`;
  - nav2.yaml's three footprint copies;
  - slam.yaml's `max_laser_range`;
  - the encoder-count pin in `tests/test_ros_driver_board.py`.
- **The ROS-drive stop** is written out in full in both body ENG and ros
  ENG, and the two already disagree. Body ENG is canonical; ros should
  link to it.
- **`brain.min_distance_cm`** is still carried in safety ENG:118. Mission
  is canonical.
- **The ROS start-up runbook** is copied into ros ENG:183-211 from
  `service/slam/README.md` §3. The README is canonical.
- **The metrics filter has two owners.** operations ENG:277-280 says
  operations owns the fix. Both architecture specs say recordings
  implements the filter and operations owns the layout. Also missing from
  every spec: `DELETE /recording/walks/metrics-YYYY-MM-DD` deletes a day of
  mission rows.
- **The deployed-replay gap** is written out four times (recordings ARCH
  and ENG, operations ARCH and ENG). Recordings ENG is canonical.
- **The lidar-less driving rule** is stated four times. Safety ENG is
  canonical.
- **The model warm-up command** appears in operations ENG:190.
  Perception ENG is canonical.
- **The tiered `demo_replay_mission` caveat** appears in recordings
  ENG:204. Policy ENG is canonical.
- **Serial-port ownership** is still stated as body's own decision at body
  ARCH:165-173. Motor-board D1 owns it.
- **Who owns where the lidar driver lives** is claimed by both ros ARCH
  ("this domain owns that decision") and body ARCH ("Owner: the user").
  One of them has to go.

## 5. Fix list

Code items marked **decision** are for the user. Every other item is a
spec edit.

1. **(S, code, decision)** H1: `POST /stop` (and the twin's Stop) cancels
   any active nav2 goal. Cancel on a background thread after
   `robot.stop()`, so the stop never waits on the bridge. Pin it with a
   test: an active goal, then `/stop`, and the wheels stay at zero past
   `STOP_HOLD_S`. Record the rule in safety ARCH; ros D6 and twin link to
   it. **Until then, every spec must say a stop pauses a goal, it does not
   end it.**
2. **(M, procedure)** H2: give G4 a runnable command set in platform ENG:
   - the robot server, with its env;
   - a brain on :8001 with `ROUTE_PREFIX=/brain`;
   - wait for `/wheels` to be usable, then start the container as
     `picar-ros`;
   - `SIM_MAP=scaled_house pytest tests/test_ros_chain_live.py tests/test_nav_live.py`;
   - expected output: 0 skipped.

   `tools/jetson/README.md` §4 needs the same; it belongs to the 3.33
   session, so it is listed in the handoff rather than edited here.
3. **(M, code, decision)** M2: count the bridge's HTTP side toward
   `ros_up`, or shut the container down when the bridge exits. Meanwhile,
   add the failure row to the specs.
4. **(S, decision)** M3: either apply a landed cloud `target_reached`, or
   record that under async only arrival ends a tiered mission (spec + P7e).
5. **(S, code)** M5: compute `target_probe`'s peak over
   `max(c.probability for c in r.candidates)`. Fix perception ENG and
   `CLAUDE.md`.
6. **(S)** M4: make the deploy procedure export
   `VISION_SHARED_SECRET=$VISION_SECRET` and
   `WALKS_SHARED_SECRET=$WALKS_SECRET`, and give the no-auth failure
   signature. Better: make `build.sh` refuse to print the command when
   either is empty.
7. **(S)** M6-M15 and §4, applied to the specs per domain.
8. **(S)** The lower-severity items in each reviewer's report: HOW lines,
   wording and counts. The list is in the agent prompts that applied them,
   and the result is recorded in §6.

## 6. Resolution (2026-10-02, same day)

**Code.** `STOP_HOLD_S` went from 0.4 s to 0.6 s in `robot/ros_drive.py`,
so the hold now outlasts the two timeouts added together. No other code
changed. The four decisions (H1, M2, M3, and the M5 code fix) are left to
the user and are in `HANDOFF-2026-10-02-spec-review.md` as 1b, 1c, 1d and
4h. The `build.sh` empty-secret guard is 4i.

**Specs.** Five agents applied every finding in §3 and §4, plus each
reviewer's lower-severity items, re-checking each against the code before
editing. Where nothing was decided, the specs now state what the code
does: a stop pauses a nav2 goal and does not end it, and only arrival ends
a tiered mission under the async cloud call. Every canonical home in §4 is
now real: body, safety and policy link to platform's chassis table rather
than restating it.

**New procedures** (all UNCONFIRMED, not yet run):

- G4 has a complete command set: 18 tests, which count only with 0
  skipped.
- motor-board ENG gains a flash command, a udev rule template, a pyserial
  heartbeat check and an odometry-over-a-metre check.
- The deploy procedure exports the secrets it needs.

`service/slam/README.md` gained the deactivated-plugin and
stop-during-a-goal failure signatures, along with the two world readouts
that used to live in world ENG.

**Not applied:**

- `tools/jetson/README.md` §4, which belongs to the 3.33 session (handoff
  2-pre).
- `CLAUDE.md`'s wrong "inert prompt" reading (handoff 4h).
- Three "Why" cells in ros ENG that have no recorded reason.

`spec_lint` reports 0 errors and 0 warnings, and the stop, blind-reverse,
`sim_map` and lint suites pass (55).
