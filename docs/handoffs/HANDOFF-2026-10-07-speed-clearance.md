# Handoff 2026-10-07 -- speed-dependent clearance (PLAN 3.44), half built

For a fresh session continuing 3.44. **First read
`docs/guides/PARALLEL-SESSIONS.md`**: other sessions now work on the plan at
the same time. This section's file is
`docs/plans/ros-alignment/3.44-speed-clearance.md` (3.43's is
`3.43-explore-wedge.md`), and its own ports, ROS domain, container and image
come from `eval "$(python tools/plan_section.py env)"` -- use those, never
the shared defaults. Run `python tools/plan_section.py overlap` before editing
`robot/safety.py`. Read, in order:
`docs/plans/PLAN-ros-alignment.md` **3.43** (the explore wedge) and **3.44**
(this work, confirmed by the user 2026-10-07), then
`HANDOFF-2026-10-07-lidar-and-speed.md` section 2 (what the lidar driver
means for this). CLAUDE.md section 7 is the definition of done.

## 1. Where the work lives

- Worktree `/Users/anshugaind/vision-picar/.claude/worktrees/frontier-search`,
  branch **`plan/3.44-speed-clearance`** (renamed from `merge-into-dev`, 2026-10-07; it also carries 3.43, which predates the one-branch-per-section rule) -- the user asked to stay on it. It is
  `origin/dev` (`9832fe5`) plus local commits, **none pushed**:
  - 3.43 (explore wedge): criteria, part (a) the escape tries both sides
    (`brain/explore.py`, `tests/escape_sweep.py`, a pinning test), part (b)
    nav2 inflation measured and NOT adopted (`evaluations/slam-343/`).
  - the renumber after rebasing on dev (3.42 on dev is the lidar driver),
    and CLAUDE.md 3b's corrected SLAM bullet.
  - 3.44's criteria, then `58b1b01` **WIP**: the code below, one test red.
- Ask the user before pushing. `origin/dev` is the merge target; other
  sessions push there too -- fetch and rebase first.

## 2. The rule (3.44, confirmed)

A straight move needs `CREEP_MARGIN_CM` (3 cm) if ASKED at <= `CREEP_M_S`
(0.03 m/s); otherwise `max(min_distance_cm, stop_distance_cm(v))`, with
`stop = 3 + 100 (v * 0.15 + v^2 / (2 * 0.5))` cm. Today's speeds keep exactly
20 cm. The 0.15 s reaction excludes the lidar's age (`_aged()` owns it);
`DECEL_M_S2` 0.5 is a `[PLACEHOLDER]` until 6.9(d) measures braking.

## 3. What is built (in `58b1b01`, `robot/safety.py` only)

- Constants and `stop_distance_cm()`, `verb_speed_m_s()` (verb speed 0-100
  -> m/s, 100 = 0.6), `SafetyController.required_clearance_cm(v, creep)`.
- `check_and_execute()` now wraps `_check_and_execute(action, need)`: a
  straight verb sets `_asked_v_m_s` / `_creep_asked` for its whole run and
  clears them in `finally`. The robot server uses ONE SafetyController for
  `/action` and the wheel loop, so under `drive: ros` the twists of a creep
  verb are vetted at the creep bar; nav2's twists (no verb) never are.
- `vet_wheel_velocity()` and `run_verb()`'s look-ahead compare against
  `required_clearance_cm()` instead of `min_distance_cm`.
- `_scan()`'s range hint is `scan_hint_m()` = half the chassis + 1.5 x the
  bar at `MAX_LINEAR_M_S` (~0.85 m), so the simulator is not blind where the
  car sees.

## 4. The red test, and the decision it needs

`tests/test_guarded_verbs.py::test_criterion_4a_a_clear_move_is_still_a_whole_cell`:
335/370 whole cells. `tests/verb_sweep.py` drives `SPEEDS = (50, 100)`;
at speed 100 (0.6 m/s) the bar is now 48 cm, so "clear" moves (clear by the
old 20 cm) stop short. Intended by the rule ("a faster move needs more
room"); no production caller asks speed 100. Decide with the user: judge
4a against the new bar for each speed (likely), or drop speed 100 from the
"today's speeds" sweeps. Also not yet checked: the scan hint growing from
0.6 to ~0.85 m may expose obstacles the old sweeps never saw -- if other
sweep tests move, that is the handoff's sim-only trap being fixed, not a
regression; confirm on ground truth.

Safety tests run so far: footprint, pivot, wheels-command, explore, lidar
safety all passed (47); stopped at the first failure above. The full suite
has not been run on the WIP.

## 5. Not started (3.44's criteria)

1. 3.18 / 3.19 sweeps in both modes (instant and `lidar=True`).
2. The creep sweep: creep forward/reverse from 1440 starts within 25 cm,
   both modes, 0 contacts, 0 samples under 2 cm of true gap. New file.
3. A test that a command the look-ahead slowed below creep is still refused
   at 20 cm (red on a version keyed on the slowed speed), and R6 6/6 with
   nearest surface >= 19 cm.
4. 0.4 / 0.5 m/s sweeps, and a test that the scan hint covers the bar.
5. The escape's creep steps in `brain/explore.py` (`_queue_escape()`: creep
   REVERSE then FORWARD at speed <= 5, a few cm), then `tests/escape_sweep.py`
   on the 200 wedged poses plus the traps: >= 95% freed.
6. Three furnished-home runs, no >= 120 s stall
   (`evaluations/slam-343/parked.py`), >= 8/9 goals, 3.40's SLAM bars.
7. `docs/safety/ARCHITECTURE.md` and `docs/engineering/safety/ENGINEERING.md`
   in the same commit as the finished code.

## 6. Instruments and stacks

- `python -m tests.escape_sweep 200` (in-process, ~10 min).
- Live runs: first `eval "$(python tools/plan_section.py env)"` (ports
  9440-9445, ROS domain 44, container `picar-ros-344`, image
  `vision-picar-ros:plan-3.44` -- build it from this branch), then `python -m tests.demo_slam_home 1
  --tour 1 --explore-first 900`, judged by `evaluations/slam-343/parked.py`;
  `evaluations/slam-340/with_stack.py` wraps R5/R6 instruments (env
  `SLAM_YAML`, `NAV_YAML`). A heavy run takes the machine-wide lock and waits
  for another session's to finish; other sessions' containers are theirs.

## 7. Open, outside 3.44

- 3.31's frontier-search batch (~7 h) waits on 3.43/3.44.
- 3.42's relayed-scan time stamp (lidar handoff, item 5): the user's call,
  to settle before any nav2 speed increase.
