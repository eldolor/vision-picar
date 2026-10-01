# Handoff -- 2026-09-30: safety fixes, and `drive: ros` gates G1-G3

For a fresh session. `CLAUDE.md` remains the orientation document; this is
what one long session (2026-09-27..30) did, where it is recorded, what is
open, and what it learned the hard way. Its sibling,
`HANDOFF-2026-09-30.md`, covers the robot-base purchase -- read both.

**State at handoff: `dev` = `origin/dev` = `e84bf31`, working tree clean,
one worktree. Full offline suite on the code: 1458 passed, 51 skipped
(`cf6a880`; only `.md` files changed after). Live ROS suites green (below).**

**Hardware (2026-09-30):** the Jetson Orin Nano Super has ARRIVED and is
kept **UNOPENED until the Rover is delivered** (the user's decision -- do not
propose opening it earlier). The Waveshare UGV Rover PT Jetson Orin ROS2 Kit
is **ORDERED** (Amazon, ~$730, arriving Oct 19 - Nov 11). Still to buy: a
separate Jetson battery + fused cable (`JETSON-BOM.md` section 9).

---

## 1. What was built (all on `dev`, each phase's criteria written FIRST)

Every row: metrics and thresholds in `PLAN-ros-alignment.md` before
building, red on the unfixed code, each fix mutation-checked (removed ->
only its own criterion red), judged on GROUND TRUTH, never on the readings
the code under test uses.

| Plan | What | Key numbers |
|---|---|---|
| 3.18 | Oblique-approach escape: a **swept-corridor** check off the 360-degree scan in `robot/safety.py` (`forward_clearance`/`reverse_clearance`), in series with the cone; footprint-rectangle collision in `sim/grid_world.py`; exact short safety scan (`renderer.cast_ray_exact`, `get_scan(max_range_m=)`) | 0/5760 runs under 18 cm (was 25/1440, worst 0.0) |
| 3.18 pt 2 | The "flaky 18.0 cm" wall stop was a **camera left panned** (the cone is cast along the camera), not a stalled loop. Grid publishes `pan_deg`; cone picks zones by BODY bearing | live 5/5 (was 2/5) |
| 3.19 | Pivots can't swing a corner into furniture (`pivot_scale`, scaled not zeroed) | 0/360 within 1 cm (was 120/120) |
| 3.20 | A mission starts with the camera centred (`MissionRunner` first tick) | pinned to the pre-change trace |
| 3.24 G1 | The live ROS chain is reliable: bridge polls over **kept-open connections** (`picar_bridge/keepalive.py`; Docker Desktop stalled ~1 new connection in 10); chain suite runs in the **scaled house** (the starter house's 30 cm door can't take the 23.1 cm Rover chassis) | 20/20 consecutive live runs |
| 3.24 G2 | The ROS verb path meets 3.18/3.19's bars (`tests/ros_verb_sweep.py`): wheel vet **looks ahead** one period; a verb the vet held is a refusal on the ROS path too; executor settles to its tolerance, turns at 0.05 rad/s within 0.5 deg | 0/1440 under 18 cm; clear turns within 0.64 deg 80% -> 100%; live chain 10/10, nav2 5/5 |
| 3.24 G3 | When ROS dies, **only a person drives** (user decision): ROS's pulse is the plugin's 20 Hz `/wheels`; 0.5 s silent = down; a person's verbs run `drive: direct`'s guarded path, autonomy refused `ros_unavailable`, the mission ends `failed` | live: mission ended 2.04 s after a container kill; D-pad drove 0.36 s after; back at the first post |

Also: `docs-review` merged (its goal-arbitration section renumbered 3.23);
the ROS-containment test skips `.claude/` (worktrees live there); the
brain-view live test reads `/diagnostics` until the bridge's own entry.

## 2. What is open, in order

1. **The Rover's numbers in the sim -- NEXT, and now due (the Rover is
   bought).** Encoder constant **1650 -> 660** in `sim/mock_robot.py:84`,
   `robot/hardware_robot.py:52`, `sim/fake_esp32.py:59`. The Rover's **ROS
   Driver** board (`ugv_base_ros`) is closed loop and sends measured
   odometry (`odl`/`odr`) in its feedback; today `sim/fake_esp32.py` emits
   the General Driver's `T:1001` with wheel SPEEDS only, and
   `robot/hardware_robot.py` integrates them. **Make the fake emit
   `odl`/`odr` BEFORE switching the backend to read them**, or that change
   is untested. Criteria first (`CLAUDE.md` section 7). The wall linter
   (`tests/test_wall_linters.py`) registers chassis constants -- check
   whether the encoder count needs registering. Details:
   `HANDOFF-2026-09-30.md` section 5 item 2, `JETSON-BOM.md` section 9.
2. **3.24 G4 -- the ROS stack on the Jetson**, five consecutive live passes
   against `SIM_MOTOR_BOARD=fake`. Waits for the Rover delivery (the Jetson
   stays unopened until then). Do item 1 first so the sim matches the car.
3. **`CLAUDE.md` has no status rows for 3.21 (the UGV Rover chassis) or
   3.22 (guarded verbs)** -- another session's work; the user may ask for
   them to be written from the plan.
4. **Plan open item 6** (a search that uses the map, saving it, S3 backup)
   -- decided by the user 2026-09-27, not started.
5. **Residue, recorded in 3.18-3.19:** the chassis numbers and the 3 cm
   corridor side margin are placeholders until measured on the Rover (the
   RPLidar-era +/-3 cm rating no longer applies -- the Rover ships a D500).
6. **User actions:** the Jetson battery; a low-voltage shutdown plan for
   both packs (unwritten anywhere); the docs fixes in
   `docs-review/REPORT-robot-base.md`.
7. Local `main` has not been updated from `dev` -- a separate decision.

## 3. How to run what this session ran

* **Offline:** `pytest tests/ -q` (~11 min). Safety sweeps:
  `python -m tests.demo_footprint_sweep`, `tests/ros_verb_sweep.py`.
* **Live ROS** (`service/slam/README.md`): **rebuild the image first** if
  anything under `service/slam/` changed -- a stale container runs old
  chassis numbers against the new sim (it happened). Chain suite:
  `SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros bash
  service/tunnel/restart.sh`, fresh container, then `pytest
  tests/test_urdf.py tests/test_ros_chain_live.py
  tests/test_brain_view_live.py` with **`LOCAL_SECRET` exported and
  `APP_SHARED_SECRET` NOT** (else the in-process tests demand it). nav2:
  `SIM_MAP=scaled_house pytest tests/test_nav_live.py`. The SLAM lap stays on
  the starter house.
* **Leave the local stack as found:** `SIM_MAP=home_first_floor
  ROBOT_DRIVE=ros`, container on `vision-picar-ros:latest`.

## 4. Learned the hard way

* **Other sessions commit to `dev` in parallel and sometimes edit the main
  checkout directly.** Work in a worktree on a branch, merge when green.
  Staging a whole file swept another session's uncommitted edits into a
  commit once (split afterwards). And `git commit` after `merge
  --no-commit` commits only what is STAGED -- edits made after the merge
  started were silently left out of a merge commit once (amended).
* **A suspected cause is not a cause.** 3.17 confidently blamed a stalled
  wheel loop; a timing readout showed zero late ticks and the cause was a
  panned camera. Instrument first.
* **Criteria can contradict each other -- the first run tells you.** A
  free-angle metric measured to CONTACT contradicted a 1 cm no-contact bar;
  "clear" for progress was first defined on truth, which scored the vet's
  deliberate caution as a ROS shortfall. Both corrected and RECORDED as
  corrections, thresholds unchanged.
* **Test harnesses lie in specific ways.** Found and fixed here: a fake
  chain that skipped the server's vet; a heartbeat posting zeros that
  fought the chain; queued twists landing after a "dead" container;
  `test_urdf.py` expanding the UNTAGGED image (use `PICAR_ROS_IMAGE` when
  comparing versions); a worktree's shared `.venv` is fine (imports resolve
  to the cwd) but the Docker image is not.
* **Flaky live tests: alternate versions run by run** to cancel machine
  drift, and fresh vs kept-alive containers differ (SLAM's map grows).
* **Background watchers:** point a `Monitor` at the task ID the tool just
  returned -- twice this session it watched a stale file and expired.
