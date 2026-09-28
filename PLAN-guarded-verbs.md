# Guarded verbs -- re-check direct-mode moves while they drive

**Status: BUILT and measured 2026-09-28 -- all seven criteria met.** Written 2026-09-28, from the
finding in `PLAN-ros-alignment.md` 3.20 (the UGV Rover chassis). The user
took all three recommendations in section 6. The acceptance criteria, as
written before building (with one refinement -- a look-ahead, below), are
`PLAN-ros-alignment.md` 3.21; results are recorded there.

---

## 1. Background: `drive: direct` vs `drive: ros`

Both take the same `/action` verbs -- FORWARD (one 0.30 m move), REVERSE,
LEFT/RIGHT by an angle. They differ in how a verb becomes wheel motion, and
what guards it on the way.

**`drive: direct`** -- the default, no ROS.

- `/action` -> `SafetyController.check_and_execute()` checks clearance
  **once, before the move**, then calls the backend's own verb.
- In the sim, `MockRobot` realises the whole move at once
  (`_drive_cells()` / `_pivot()`), stopped only by the sim's own collision:
  the half-cell head-on cap and the chassis rectangle.
- On the car, `HardwareRobot._run()` sends fixed wheel speeds to the ESP32
  for a fixed time. It never looks at the lidar while moving.

**`drive: ros`** -- opt-in (`ROBOT_DRIVE=ros`), needs the Docker container
(`service/slam/`).

- `robot/factory.py` wraps the backend in `RosDriveRobot`
  (`robot/ros_drive.py`). A verb becomes a 20 Hz stream of twists: bridge ->
  `twist_mux` -> `diff_drive_controller` -> `picar_sim_hardware` ->
  `POST /wheels`.
- It closes on the **encoders**, not a timer.
- Every command is vetted by `SafetyController.vet_wheel_velocity()`, and
  the robot server's 20 Hz wheel loop re-vets the standing command every
  50 ms -- which is why it stops at 20 cm from a wall.
- nav2 goals only exist in this mode.

**So the car is guarded while it moves only under `ros`.** Under the
default, a verb is checked at its start and then drives blind.

---

## 2. The problem, and two more found while reading the code

1. **A FORWARD/REVERSE verb is vetted once, then drives blind.**
   `check_and_execute()` allows a move with >= 20 cm of clearance, and the
   move then covers 30 cm. On the car in `drive: direct` that is **up to 10
   cm of travel past a wall**, on either chassis. The sim hides it: its
   half-cell cap stops the robot's centre 15 cm from the wall, and nothing
   like that exists on hardware. (On the UGV Rover the same cap parks the
   body where its 17.1 cm corner radius cannot turn, which is what fails 17
   rule-based mission tests -- `PLAN-ros-alignment.md` 3.20, criterion 5.)
2. **Turn verbs have the same gap.** 3.19's pivot guard
   (`SafetyController.pivot_scale()`) only runs inside the per-period vet,
   so a direct-mode turn on the car can swing a corner into furniture.
3. **`/stop` does not stop a hardware verb in progress.**
   `HardwareRobot._run()` re-sends the verb's wheel speed every 50 ms until
   its timer ends, so a `/stop` -- or the watchdog's stop -- is overwritten
   within 50 ms. The sim cannot show this, because its verbs finish
   instantly.

---

## 3. Design

**One guarded verb runner inside `SafetyController`** -- the one place
every verb already passes, in-process and through the server. Used for
FORWARD, REVERSE, LEFT and RIGHT under `drive: direct`:

1. Set the verb's wheel velocities (`set_wheel_velocity()`).
2. Every 50 ms, run the **same** `vet_wheel_velocity()` the wheel loop uses.
   If it clamps, stop and end the verb early.
3. End on the **encoders** (odometry distance / heading), as
   `robot/ros_drive.py` does -- never on the clock.
4. Check a stop flag every period; `stop()` sets it, so `/stop` and the
   watchdog end a verb within one period.
5. Keep time correctly for each backend: the sim advances **simulated** time
   (`robot.advance(dt)`), so tests stay fast and deterministic; hardware
   sleeps real time (its `advance()` re-sends the command, which also feeds
   the ESP32 heartbeat).

Under `drive: ros` nothing changes -- the wrapper is already guarded, and
the runner must dispatch to it as today rather than add a second loop.

The sim's half-cell cap and rectangle collision stay, as the simulator's
truth about contact -- but they should no longer be what ends a verb.

**Refinement, found while writing the criteria: look ahead.** Vetting once
per period still lets a move coast up to one period's travel past the line
-- 3 cm at a verb's 0.6 m/s, which would break 3.18's 18 cm bar. So each
period a translation may cover at most (clearance - `min_distance_cm`), and
stops AT the line; `pivot_scale()` already does the same for turns.

**Rejected alternative:** check once and shorten the move to (clearance -
20 cm). Simpler, but it does not re-check while driving, so it misses a
person or pet stepping in.

**Things to watch while building:**

- In the server's direct path `/action` holds `motion_lock` for the whole
  verb, which blocks the wheel loop. That is fine while the runner does its
  own vetting, but `/stop` must never wait on that lock.
- The server's wheel loop also calls `robot.advance()` for any non-zero
  standing command. The runner and the loop must not both integrate the
  same motion (double speed in the sim).
- `sim.realtime: true` sleeps a verb's duration after moving it; under the
  runner that becomes a real-time sleep per period.

---

## 4. Acceptance criteria -- to be written into `PLAN-ros-alignment.md` (3.21) before building

All measured on **ground truth**, with bars reused from 3.18 and 3.19 rather
than invented. Criteria 1-3 are to be confirmed RED on today's code first.

1. **Moves stop safely.** A sweep of FORWARD and REVERSE verbs from seeded
   starts in all three houses, with the sim's half-cell cap **disabled** so
   the sim cannot be what saves them: after every verb, travel-to-contact
   `T >= 18.0 cm` and gap `G` never below `min(G at start, 1.0 cm)`
   (3.18's bars).
2. **Turns stop safely.** The same sweep for LEFT/RIGHT: `G` never below
   `min(G at start, 1.0 cm)` (3.19's criterion 1).
3. **Stop works mid-verb.** A `/stop` during a verb on the fake ESP32
   (`sim/fake_esp32.py`) zeroes the wheels within 100 ms, and the verb does
   not resume.
4. **Moves still mean what they meant.** Of verbs with >= 60 cm free, at
   least 95% still cover the full 30 cm (section 7 rule 4's progress half).
5. **Missions.** The pinned R1, R1b, R1c and arrival results hold within
   their existing bars, and the 17 tests that fail on the UGV Rover pass --
   no threshold loosened.
6. **Live.** One mission through the brain's HTTP API on `drive: direct`,
   and one through the robot server over the fake ESP32 serial line
   (`SIM_MOTOR_BOARD=fake`).
7. **`drive: ros` unchanged.** `tests/test_ros_drive.py` and the live chain
   suite pass.

---

## 5. Order of work

1. Write the criteria (section 4) into `PLAN-ros-alignment.md` as 3.21.
2. Write the failing tests; show criteria 1-3 red on today's code.
3. Build the runner.
4. Add the stop flag, and fix `HardwareRobot._run()` overwriting a stop.
5. Re-measure, including 3.20's criterion 5 on the UGV Rover.
6. Live runs (criterion 6).
7. Record the numbers in the plan and pin them in tests.

Work in a git worktree. Build on the `ugv-rover-chassis` branch
(`~/vision-picar-ugv`), because the Rover's 17 failures depend on this fix.

---

## 6. Decisions -- all three taken as recommended (2026-09-28)

1. **What does a move cut short mean?** Recommended: **executed**, with
   `moved` against `requested` in the result. A move refused before it
   starts stays a refusal (`SafetyViolation`). A move that covers under
   1 cm counts as refused for R1b's stuck detector (`stuck_after`), so a
   robot pinned at a wall still ends `blocked` instead of looping.
2. **Include turn verbs?** Recommended: **yes** -- same gap (section 2,
   item 2).
3. **Where should it live?** Recommended: **`SafetyController`**, so
   in-process agents and the server behave identically. The alternative,
   the server's wheel loop only, would leave in-process sim runs unguarded.
