# Spec review, third pass -- reviewer E: recordings, operations, platform (+ whole-set coverage)

Read-only. `python tools/spec_lint.py`: 0 errors, 0 warnings. Offline tests
run: `test_health`, `test_metrics`, `test_tunnel_proxy`, `test_static_assets`,
`test_serverless_routes`, `test_spec_lint` = 85 passed. Every cited test count
in the three engineering specs re-collected and matched (see g).

Bottom line: **none of today's three commits makes anything in recordings,
operations or platform false.** Two statements are now incomplete (ROS
liveness, cloud-call counts), and `CLAUDE.md` still lists 1a as an open user
decision and describes arrival without the cloud confirmation. The rest
are procedure gaps on the 3.33 return-window gate, and small prose items.

---

## (a) Findings in today's changes (d549878, 6e4b11e, 50b2293)

**A1. operations ENG:319-321 is now incomplete after 6e4b11e (1d).** It says
"Under `drive: ros`, a dead container shows only as `drive.ros_up` on the
robot's health route, not in the verdict." Since 1d, `ros_up()` is also false
after a failed send to the bridge:

- `robot/server.py:319-331` ANDs `robot.bridge_up()` into the result;
- `robot/ros_drive.py` `_send()` and `_mark_bridge_down()` set the flag;
- `/health` computes `drive.ros_up` by calling `ros_up()` (`robot/server.py:1000`).

The verdict still ignores the field (`control/health.py:84-122` reads no
`drive` key). Reading `drive.ros_up` now also starts a background bridge probe
(`bridge_up()` starts one at most every `BRIDGE_PROBE_S`). That side effect
is harmless, but nobody writing the 4e fix would expect it.

- Fix: change the line to "a dead container or bridge". HANDOFF 4e
  (`HANDOFF-2026-10-02-spec-review.md`, section 4, "4e") has the same wording.
- Severity low, effort S.

**A2. Cloud-call counts changed meaning at 50b2293, and operations does not
say so.**

- `TieredVision.confirm_arrival()` adds one paid call per arrival episode. It
  increments both `stats.cloud_calls` and
  `stats.triggers["arrival_confirmation"]` (`brain/tiered.py`,
  `TRIGGER_ARRIVAL`).
- The metrics row copies tier stats verbatim (`control/metrics_client.py:73-86`).
- The dashboard plots and tabulates `stats.cloud_calls` (`control/metrics.js:51`, `:186`).

So a tiered mission that reaches its target is one call more expensive from
this commit on, and the release-over-release comparison that operations ARCH
gives as metrics' purpose (`docs/operations/ARCHITECTURE.md:31-32`, `:175-191`)
shows a step that no spec explains.

- operations ARCH:217 ("Brain calls on triggers") is still loosely true
  but omits the arrival call.
- Fix: one sentence in operations ENG §Implementation, "Mission metrics":
  "`stats` is the tier's stats verbatim; since 50b2293 `cloud_calls`
  includes one `arrival_confirmation` per arrival". In ARCH:217 add "and
  once per arrival to confirm identity".
- Severity low, effort S.

**A3. CLAUDE.md is stale against 50b2293 (and 1b and 1d).**

- `CLAUDE.md:209-211` lists the open work as "one user decision (tiered
  arrival on a wrong object)". That decision was taken and built as 1a.
  Nothing in HANDOFF section 1 is open any more.
- `CLAUDE.md:385-389`, the repo map's `brain/arrival.py` entry, says
  "`found` when the target is in the steering band and the LIDAR ... reads it
  within 0.40 m, two frames running". That is no longer sufficient. Since
  50b2293, `brain/agent.py` `_confirm_identity()` also requires a cloud
  confirmation, so the rule as written there is the pre-1a rule.
- The P7e status row (`CLAUDE.md:176`) says the same.
- No status row records 1a, 1b or 1d.
- Severity med: this is the orientation file every session loads, and it
  states the arrival contract wrong. Effort S.

**A4. HANDOFF section 1 preamble is stale** (`HANDOFF-2026-10-02-spec-review.md:31-34`).
It says "the mechanism is in the matching engineering spec's Known gaps,
which says 'decided fix not yet built' until it lands". All four items are
built, and a grep of every spec finds no "not yet built" for 1a, 1b or 1d
(the only hit is world ENG:259, which is about something else). The strikes
on 1a-1d match the commits. Severity low, effort S.

**A5. Cross-domain lead for policy and simulator, UNCONFIRMED.** The
evidence that 1a does no harm (the 3.11 sweep "unchanged" and
`tests/test_arrival_confirmation.py`) uses an oracle cloud. `_quiet_cloud`
(`tests/test_bearing_turns.py:85`) reads `target_visible` from ground truth.

- A live Sim-tab tiered mission now ends `found` only if the real
  `/navigate` recognises the target on a raycaster render, where objects are
  billboards (`sim/renderer.py`).
- No measurement of that exists.
- If the model says no, the mission pays one call and then ends
  `blocked` or `max_steps` at the target.
- Not my domains. Passing it to the policy and simulator reviewers.

**A6. Checked and unaffected:**

- **G4.** `tests/test_ros_chain_live.py:176` starts `policy: "frontier"`, so
  no arrival confirmation is made. `--collect-only` on the two live files
  still gives 18 (13 chain, 5 nav). platform ENG:204-207 still holds.
- **1d and G4's start order.** While the procedure waits on `/wheels` before
  `docker run`, the watchdog's `robot.stop()` (`robot/server.py:301-302`)
  zero-sends to the absent bridge (`robot/ros_drive.py:329-337`), which marks
  the bridge down. The watchdog keeps stopping every 0.1 s while the robot is
  silent, and the first zero-send that succeeds after the container starts
  clears the flag (`_send` sets `_bridge_down_since = None`). G4's first verb
  therefore does not hit a stale "down". Reasoned from the code, not run:
  UNCONFIRMED.
- **The replay demo.** recordings ENG:212-213 ("one per trigger under
  `tiered`") stays true. A replay never judges arrival, because `ReplayRobot`
  and `TeleopRobot` define no `get_scan` and fall back to the unusable default
  (`robot/interface.py:507`). So `confirm_arrival` never fires there.
- **1b (stop cancels goal).** Nothing in my three domains describes `/stop`
  or goals.

---

## (b) HOW in architecture specs

**B1. operations ARCH:196-200** reads "until the wheel plugin's start-up race
is fixed (`HANDOFF-2026-10-02-spec-review.md` item 2a), a container that
comes up first can lose its wheels for good (docs/engineering/ros/ENGINEERING.md)".
The decision is "container last, after the robot server reports usable
wheels". The temporary workaround, tied to a handoff item number, is HOW.
operations ENG:238-248 already carries it. Keep only the decision and its
reason in ARCH. Low, S.

No other HOW leaks found. Numbers judged on each read:

- **platform ARCH.** $86 and $250 (:91-92) and "about 5 A" (:125-126) are
  trade-off evidence. 250 ms, "0 late ticks" and "1 GB" (:239-240) are
  user-set acceptance bars. 3.3 degrees (:170) is the reason the vendor
  nodes were rejected.
- **operations ARCH.** $159 and $110 (:85-86) are the cost decision's
  evidence. "Five times" (:227) is history.
- **recordings ARCH.** "Six walks" (:95) is the scorecard's stated trust
  limit.

All may stay.

---

## (c) Restatement in engineering specs

**C1. platform ENG:57 describes itself wrongly.** It says `tools/jetson/README.md`
is canonical and "Procedures below links to it and adds only expected outputs
and pass rules". Procedures (:212-249) carries a complete G4 command set that
the README does not have (the README's §4 is three lines: `tools/jetson/README.md:77-82`).
Known gaps :310-313 records that the README lags. Until the README catches
up, platform ENG is the canonical home for G4. Say so at :57, e.g. "except
G4, whose full command set is here until README §4 carries it". Low, S.

**C2. The metrics deletion hazard is stated in full in four places:**

- operations ENG:299-303;
- recordings ENG:367-372;
- recordings ARCH:104-109 and :281;
- operations ARCH:186-191, more briefly.

The deployed-replay gap already uses "canonical in recordings ENG, link
here". The deletion hazard is the same shape. Make recordings ENG canonical,
because it owns the filter, and reduce operations ENG:299-303 to a link. Low, S.

**C3. The "container after `/wheels`" order** is written out in operations ENG:238-248,
platform ENG:219, ros ENG and `service/slam/README.md` §3. Every copy links
onward, so this is tolerable. The canonical one is `service/slam/README.md`
§3, as the second review decided. No action needed beyond B1.

---

## (d) Verification failures

| # | Spec claim | Code | Severity |
|---|---|---|---|
| D1 | **PLAN 3.33 criterion 2's pass rule has no procedure.** It requires "the shipped pipeline returns the same verdict on a corpus frame as the laptop (same detection status, CLIP probability within 0.01)" (`PLAN-ros-alignment.md` 3.33, acceptance criterion 2). Platform ENG Procedures step 4 (`docs/engineering/platform/ENGINEERING.md:185-190`) lists setup's expected output only. | `tools/jetson/setup.sh` builds `pipeline_for("red backpack")` and asserts only `scorer.device == "cuda"`; it never runs a frame. `bench_perception.py` records `status` per frame but compares nothing with a laptop run. The README §3 (`tools/jetson/README.md:47-71`) has no comparison either. On the return-window gate, risk 1 can be closed without ever checking the board's answers. | **med**, S/M |
| D2 | operations ENG:154 is the guard line `[ -n "$VISION_SHARED_SECRET" ] && [ -n "$WALKS_SHARED_SECRET" ] \|\| echo "STOP: a secret is empty"` | It only echoes. If the block is pasted whole, `build.sh` runs next anyway. With the no-auth consequence spelled out at :159-172, the guard should actually stop the run, e.g. chain the build with `&&`, or use `\|\| { echo STOP; false; }` plus `set -e` advice. Distinct from Known gap 4i, which is about `build.sh` itself. | low-med, S |
| D3 | operations ENG:304-305: "`build.sh` prints its deploy command even with empty secrets. It only warns." Also operations ARCH:62 and :234 ("The build prints a warning"). | `service/lambda/build.sh:82` prints "(Pass both secrets: empty means the functions run with NO auth.)" **unconditionally**. It never inspects either variable, so there is no conditional warning to rely on. | low, S |
| D4 | operations ENG:39: cache `day` = `max-age=86400` | `service/static/sync.sh`: `day) cc="public, max-age=86400"` | low, S |
| D5 | **PLAN 3.33 step 5 contradicts the canonical procedure.** It reads "Clone from GitHub (`dev` pushed first)", but the README's §2 (`tools/jetson/README.md:30-42`) and platform ENG:78 say the repo is private, the Jetson gets no GitHub credentials, and the Mac pushes over SSH. A plan section used as the live criteria list disagrees with its own procedure. Fix the plan line, or annotate it as superseded by the README. | -- | low, S |
| D6 | `CLAUDE.md:21`: "about 1420 passed, 51 skipped as of 2026-09-28"; `CLAUDE.md:550`: "~1450 tests" | `.venv/bin/pytest --collect-only -q tests/` gives **1667** collected. The repo map line does say to run `--collect-only` for today's count. | low, S |

Re-verified correct in passing (no change): operations ENG's claim that the
walks Lambda's `ENV_LABEL` is read only by `admin_server.py` and the robot
server (`control/admin_server.py:897`, `robot/server.py:470`; no reader in
`service/vision_analyze/`).

---

## (e) Pair, boundary, coverage and duplication

**Pair agreement.** All three pairs agree. Each ARCH decision with an
implementation has its ENG counterpart:

- operations: serverless, assumed role, tunnel split, M5 verdict, identity,
  metrics, B5 as planned;
- recordings: one store, write path, proxy limit, separate service, human
  ground truth, advisory scores, replay, retention, redirects, fire-and-forget;
- platform: Jetson, Rover, JetPack 6, 15 W, vendor software off, CAD
  geometry, lidar in the robot process, bring-up first.

No ENG contradicts its parent.

**Boundaries.**

- **Metrics rows.** Settled consistently in all four documents: operations
  owns the layout and recordings owns the `list_walks()` filter. The fix is
  still open (HANDOFF 4c), and the specs record it honestly.
- **Vision-service secret on the walks function.** Recordings ENG is
  canonical, and operations links to it.
- **Lidar ownership.** platform ARCH:190-209 states the user decision (lidar
  driver in the robot process) and defers to safety ENG for the refusal
  rule. Consistent.

**Coverage, whole set.** I took `git ls-files` under `robot/ world/ brain/
control/ sim/ service/ firmware/ tools/ web-twin/` (119 source files after
excluding data, config, tests and `__init__`). Each file's path or basename
was matched against the concatenated text of all 30 specs and
`docs/README.md`. Directory-level ownership was then checked by hand:
`tools/hailo/` and `tools/gpu/` are owned by perception ENG:30-31, and
`tools/trt/trt_owlv2.py` by perception ENG via `brain/perceive_lab.py`.

Unowned, unchanged since the second pass:

- `tools/contact_sheet.py`, `tools/label_prepass.py`, `tools/steer_check.py`
  and `tools/yoloworld_crops.py`. Each is a one-off evaluation helper.
  Proposed owner: perception (or recordings for `contact_sheet.py` and
  `label_prepass.py`, which operate on walks). One table row each in
  perception ENG §Implementation, tagged historical.
- `service/slam/src/picar_bridge/setup.py`: packaging boilerplate; ros
  implicitly owns it. No action.
- **New observation:** `service/vision_analyze/.coverage` is a tracked
  coverage database in git. It is an artifact, not code. Remove it from
  git and add it to `.gitignore`. Low, S.

Today's commits added no new modules. Their new tests are cited in the
owning specs: `tests/test_stop_cancels_goal.py` in safety ENG:238, the 1d
cases in `tests/test_ros_fallback.py` at safety ENG:239, and
`tests/test_arrival_confirmation.py` in policy ENG:189.

**Duplication.** C2 and C3 above. Also: the G4 command set exists in
platform ENG (complete), `tools/jetson/README.md` §4 (incomplete; already
in Known gaps and HANDOFF 2-pre) and PLAN 3.33 step 6 (incomplete,
historical). Canonical: platform ENG until the README is updated (C1).

**docs/README.md.**

- The index lists all 15 domains in both the reading path and the table.
- The links resolve, judging by the linter's clean run.
- The descriptions match each ARCH's scope.
- The only decision record is 0001, and `docs/decisions/` holds only that.

`CLAUDE.md` section 2's description of the spec system (lines 115-129)
matches `docs/README.md` and decision 0001. Nothing to fix there; see A3
for `CLAUDE.md`'s stale arrival text.

---

## (f) Proposed fixes, in order of how badly each would mislead

| # | Fix | Where | Severity | Effort |
|---|---|---|---|---|
| 1 | Give 3.33 criterion 2's "same verdict as the laptop, CLIP P within 0.01" a procedure. Either a `bench_perception.py --compare <laptop.json>` that diffs per-frame `status` and CLIP P, or a one-frame check in `setup.sh` with the laptop's expected values. Add the expected output to platform ENG step 4. | `tools/jetson/`, platform ENG:185-190, README §3 | med | S/M |
| 2 | Update CLAUDE.md for 1a, 1b and 1d. The open-work bullet (:209-211) no longer names a pending decision. The arrival.py repo-map entry (:385-389) and the P7e row (:176) gain "and one cloud call on the arrival frame confirms identity (1a)". Optionally add a status row for the 2026-10-02 fixes. | `CLAUDE.md` | med | S |
| 3 | Make the deploy procedure's secret guard actually stop the run. | operations ENG:151-156 | low-med | S |
| 4 | Change "a dead container" to "a dead container or bridge (1d)". | operations ENG:319-321; HANDOFF 4e | low | S |
| 5 | Note that `stats.cloud_calls` includes `arrival_confirmation` from 50b2293, and add "and once per arrival" to the brain-to-vision contract. | operations ENG §Mission metrics; operations ARCH:217 | low | S |
| 6 | Say platform ENG is canonical for G4 until README §4 catches up. | platform ENG:57 | low | S |
| 7 | Drop the handoff-item workaround from ARCH. ENG keeps it. | operations ARCH:196-200 | low | S |
| 8 | Reduce the operations copy of the metrics-delete hazard to a link to recordings ENG. | operations ENG:299-303 | low | S |
| 9 | Say the warning is unconditional text, not a check, in ENG:304-305, ARCH:62 and ARCH:234; fix `day` = `public, max-age=86400`. | operations ENG:39, :304-305; ARCH:62, :234 | low | S |
| 10 | Correct PLAN 3.33 step 5 ("Clone from GitHub") to point at the README's push-over-SSH. | `PLAN-ros-alignment.md` 3.33 | low | S |
| 11 | Strike "which says 'decided fix not yet built' until it lands" from the HANDOFF section 1 preamble. | HANDOFF:31-34 | low | S |
| 12 | Give the four unowned `tools/*.py` an owner row, and untrack `service/vision_analyze/.coverage`. | perception ENG; git | low | S |
| 13 | Refresh the test counts (1667 collected). | CLAUDE.md:21, :550 | low | S |
| -- | Hand A5 (the real cloud must recognise a raycaster billboard for a Sim-tab tiered `found`, unmeasured) to the policy and simulator reviewers. | policy, simulator | lead | -- |

---

## (g) Checked and found correct (about 140 claims)

**Operations (about 60 claims).**

- Stack table vs `cloudformation/serverless.yaml`:
  - Python 3.12, arm64 and 1024 MB, both functions (:175-202);
  - timeouts 30 and 900;
  - `NavigateModelId` default Opus 4.5 (:55);
  - `APP_SHARED_SECRET` omitted when a secret parameter is empty (:187, :209);
  - `ENV_LABEL` on both functions (:188, :210);
  - `RECORDING_BACKEND: s3` (:205).
- Outputs:
  - `SiteUrl` carries `https://` and a trailing `/`;
  - `StaticBucketName` and `DistributionId` exist;
  - recordings exports `<stack>-BucketName`, `-BucketArn` and `-AccessPolicyArn`;
  - the deploy bucket's `ExpireAfterDays` is 30.
- Templates: the nine deleted and three live templates match the files in
  `cloudformation/`.
- Public routes: all nine gateway `RouteKey`s and ten CloudFront patterns
  match the routes table.
- `build.sh`:
  - argument order `<bucket> [region]`, region defaulting to us-east-2;
  - `manylinux2014_aarch64`, py3.12;
  - the walks zip copies `control/*.py`, `admin.html`/`.js` and `config/robot.yaml`;
  - the expected `name: <n>MB -> s3://` line;
  - the printed command expands `$VISION_SHARED_SECRET`/`$WALKS_SHARED_SECRET`.
- `sync.sh`:
  - uses `put-object` with an explicit type;
  - refuses an empty manifest;
  - fails with `MISSING: <src>`;
  - checks the upload count;
  - invalidates `/*`.
- `assets.json` holds `index.html` and `app.js` (both `none`), the
  manifest and icons (both `day`), and extensionless `admin` and `metrics`.
- `run.sh`:
  - requires the secrets file;
  - sources the optional serverless secrets;
  - sets `APP_SHARED_SECRET`/`ROBOT_SHARED_SECRET` to `LOCAL_SECRET`;
  - takes the `VISION_URL` literal default;
  - sets `METRICS_URL`/`METRICS_SECRET`;
  - maps `WORLD_MODE` from `ROBOT_MODE`;
  - binds :8000, :8001 (`/brain`) and :8080 to 127.0.0.1;
  - kills only its own children;
  - still prints the stale `yolo11s.pt` hint (an already-recorded gap).
- `restart.sh`:
  - ports 8000, 8001 and 8080;
  - `kill -9` after a timeout;
  - log path `~/.vision-picar-tunnel.log`;
  - the `OK:`/`FAILED:` strings;
  - the ngrok warning.
- `proxy.py`:
  - three upstream variables;
  - 204 for `OPTIONS` on `/vision`;
  - request drops `host` and `content-length`;
  - response drops four headers;
  - 502 "upstream ... unreachable";
  - 120 s timeout.
- `control/health.py`:
  - `WATCHDOG_POLL_SLACK` 10 and `DEFAULT_TIMEOUT_S` 5.0;
  - the default URLs;
  - verdict inputs and description fields exactly as tabled;
  - exit 0 or 1;
  - `--json`, `--secret` and the `build git= exe=` render.
- `robot/identity.py`: logs at warning; `GIT_REVISION`, then git, then `unknown`.
- Robot `/health`: field list, `WATCHDOG_POLL_INTERVAL_S` 0.1, wheel loop 0.05.
- Metrics:
  - `RUN_ID_RE`, `MAX_DAYS` 90, `days` default 14 and clamped;
  - `extra="forbid"`, UTC day containers `metrics-YYYY-MM-DD`;
  - the `{stored, day}` and `{days, runs, count}` responses;
  - `row_for` fields.
- Environment readers: `ROUTE_PREFIX`, `ROBOT_SHARED_SECRET`,
  `VISION_SHARED_SECRET`, `WALKS_SHARED_SECRET`, `METRICS_URL`,
  `METRICS_SECRET`, `VISION_URL`, and `APP_SHARED_SECRET` in both factories.
- Test counts: 6, 10, 8, 15 and 14 all match; `metrics.html` is indeed
  unchecked.

**Recordings (about 45 claims).**

- Constants:
  - `WALK_NAME` (two copies), `SAFE_NAME` 128, `MAX_FRAME_BYTES` 4 MiB,
    `MAX_FRAMES_PER_WALK` 500;
  - `SCHEMA_VERSION` 9 and the five flag thresholds;
  - score weights 0.45/0.30/0.25 and 0.55/0.45, the cap at 40, verdicts
    at 70 and 45;
  - `NAVIGATE_ATTEMPTS` 4, `REPLAY_MIN_COVERAGE` 0.8, `DEFAULT_WORKERS` 6;
  - `HIGH`/`LOW` 0.90/0.30, `expires_s` 900;
  - judge model id, 8 sample frames, 12 collision checks.
- Config defaults: `allow_recording` true, `recording_dir` `recordings`,
  backend `local`, bucket empty, prefix `recordings`.
- Timeouts: `recording_proxy_timeout_s` 10.0 with no environment override,
  `replay_timeout_s` 60.0.
- Environment overrides: `RECORDING_*`, `ALLOW_RECORDING`, `VISION_URL`.
- Review route list vs `control/admin_server.py`: 18 routes, with
  `/recording/health` delegating to `/health`.
- Bucket: lifecycle (IA at 30 d, noncurrent 90 d, multipart 7 d, `exports/`
  1 d), Retain/Retain, versioning, `RecordingsBucketName` output.
- Test counts: 76, 80, 41 and 11.
- Replay and teleop: neither has a scan, so 1a's arrival call cannot fire
  on a replay.

**Platform (about 35 claims).**

- Chassis constants, all equal in code, xacro and yaml:
  - `WHEEL_RADIUS_M` 0.040 and `TRACK_WIDTH_M` 0.172 (sim, hardware, xacro, `controllers.yaml`);
  - 660 counts per rev (sim and hardware);
  - footprint 0.253 x 0.231;
  - `LIDAR_X_M` 0.040 = xacro `laser_x`;
  - `LIDAR_TO_REAR_BUMPER_CM` formula;
  - `LIDAR_RANGE_M` 12.0 = `slam.yaml` `max_laser_range`;
  - xacro `laser_z`, `pan_x`/`pan_z`, `camera_x`/`camera_up`;
  - the three `[PLACEHOLDER]`s: `camera_pitch`, `pan_limit`, `deck_height`.
- `setup.sh`:
  - R36 check;
  - index URL `jp6/cu126`;
  - torch 2.8.0 and torchvision 0.23.0;
  - `libcusolver-12-6`;
  - asserts only CLIP's device (a recorded gap);
  - all four failure strings.
- `bench_perception.py`:
  - `BUDGET_MS` 250, `DEFAULT_N` 60;
  - `--recordings` required;
  - every per-frame and summary key named in Interfaces;
  - the `devices:` and `budget ... WITHIN|OVER` lines;
  - detector device read from the predictor.
- G4:
  - `ROBOT_MODE=hardware` with `SIM_MOTOR_BOARD=fake` is accepted by
    `robot/factory.py:111-134`;
  - live suites collect 18;
  - the chain fixture skips without a secret, with the wrong house, or
    without the brain.
- `/health` `wheel_loop.late_ticks` counts `dt > 2 x 0.05 s`.
- README §0-§3 matches Procedures steps 1-5.
