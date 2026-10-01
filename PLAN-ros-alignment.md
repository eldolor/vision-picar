# Plan: ROS 2, with the twin still doing the proving

Status: **R0-R7 BUILT (R0-R1c 2026-09-25, R2-R7 and arrival 2026-09-26); R8-R9 need the hardware** · Date: 2026-09-25 · Phase IDs: `R0`-`R9`,
alongside `S*` (`PLAN-sim-hardening.md`), `B*` (`PLAN-brain-relocation.md`),
`M*` (`PLAN-microduck-transplants.md`), `T*` (`PLAN-teleop-robot.md`),
`N*` (`PLAN-mapping.md`), `C*`/`P*` (`PLAN-onboard-perception.md`).

Written against `dev` at `672a9bb`. **R0 landed the same day** -- see its
row in section 3 and section 3.1 below for what it measured and what it
still owes.

---

## 0. What this decides

`PLAN-mapping.md` §0 adopted (b+) -- ROS 2 as one service behind an HTTP wall
exposing `GET /pose`, `GET /map`, `POST /goto`. **This plan revises that in two
ways**, both settled 2026-09-25 by the user:

1. **ROS 2 is adopted properly, not minimally.** `nav2`, `slam_toolbox`,
   `ros2_control`, `tf2`, `robot_localization` and `twist_mux` all run, and
   ROS owns the metric layer down to the hardware interface. The reason is
   integration, not algorithms: those packages already agree on message types,
   REP-103 units and REP-105's `map -> odom -> base_link`. Wiring five
   standalone libraries together means writing all of that by hand.
2. **`POST /goto` as specified is withdrawn.** It answered `PLAN-mapping.md`
   §7 open question 2, which deferred to C4. The answer is that a goal route
   is the wrong shape: nav2's local controller emits `cmd_vel` **continuously
   at ~20Hz**, and a three-route wall has no velocity route. The wall moves
   DOWN to `hardware_interface`, and UP to guard `brain/`.

**The definition of done -- CHANGED 2026-09-25.** This plan was written
under §7's old rule ("done when someone holding a phone can watch it"),
stated non-negotiable that morning. Later the same day the user replaced it:
*"remove that rule, use logs and data to make a data-driven decision."* So
each phase below is done when its **acceptance data** meets thresholds
written down before the run, measured through the real mission path and
pinned in a test (CLAUDE.md §7). **Read the table's "Proof" column as that
acceptance data**; where it describes something to watch, the numbers behind
it are what decide. §2's argument gets stronger, not weaker, under the new
rule: the sim's ground truth is exactly the data hardware cannot produce.

---

## 1. The mechanism: one contract, two plugins

`hardware_interface::SystemInterface` is `RobotInterface` by another name --
`read()` pulls state, `write()` pushes commands, and **nothing requires the
hardware to be real**. So there are two implementations:

```
                    nav2 -> twist_mux -> collision_monitor -> ros2_control
                    slam_toolbox · robot_state_publisher · robot_localization
    ============ hardware_interface::SystemInterface -- THE SEAM ============
    picar_sim_hardware                      |  picar_hardware
      -> HTTP -> robot/server.py mode:sim   |    -> USB JSON -> ESP32
      -> MockRobot + MockWorld              |    -> RPLidar via sllidar_ros2
    (today, laptop only)                    |  (hardware day)
```

Everything above the seam is written once. Hardware day is a plugin swap.

**Revised at R7 (2026-09-26, 3.16): hardware day is a BACKEND swap, not a
plugin swap.** A `picar_hardware` plugin owning the serial port would take
`robot/safety.py` out of the nav path -- 3.15 put the collars in series with
it as the last word -- and would give the board two masters. So
`picar_sim_hardware` stays on the car too, still talking HTTP to
`robot/server.py`, and the seam below the robot server is the one this
project has always had: `RobotInterface`, with `robot/hardware_robot.py`
(the ESP32 over serial) beside `MockRobot`. The diagram above is kept as
first drawn; read its right-hand column as `robot/hardware_robot.py`.

**No Gazebo.** The simulator already exists; a second one is a second thing to
disagree with the first.

**Containment is unchanged.** All of this lives in `service/slam/` (rename to
`service/nav/` is a live question), `tests/test_ros_containment.py` still
passes, and the twin speaks HTTP only.

---

## 2. Why the twin gets BETTER, not merely preserved

On hardware you can watch SLAM draw a map and judge whether it looks like your
house. You cannot measure its error, because nothing in the room knows where
the robot actually is.

`GridWorld` does. So the twin can draw **the estimated pose against the true
pose**, plot the error, and show it collapsing when a loop closes.

| Question | On hardware | In the twin |
|---|---|---|
| Is the map right? | Looks about right | **Error in metres, per frame** |
| Did the loop close? | It stopped looking doubled | **Pose jumped, error fell to X** |
| Is the TF tree right? | Things drive oddly | **A fixed offset between est. and true** |
| Does the controller flicker? | Looks jittery | **P25's median run-length metric** |
| Does the lidar survive glass? | **Only here** | No -- a raycaster is geometrically perfect |

`GET /world/truth` is **sim-only and must be named so**: the hardware backend
returns unavailable, never a plausible guess. Same rule as `WorldInterface`'s
honest all-unusable defaults.

---

## 3. The phases

`R0`-`R7` need no purchase. Each ships something to press.

| ID | What | Proof |
|---|---|---|
| **R0** | **DONE 2026-09-25, and WATCHED -- the user confirmed the turn step on a phone the same day.** **Continuous pose + diff-drive kinematics.** C2 and 1.14 merged. `GridWorld` holds float `x`/`y`/`theta`; `MockRobot.set_wheel_velocity()` / `step()` / `get_wheel_state()` take **left/right wheel angular velocities** and integrate over `dt`; encoder counts fall out of that integration; continuous collision via `renderer.cast_ray()`. Wheel velocities rather than a twist **on purpose**: it puts `diff_drive_controller`'s kinematics under test with the parameters that will ship | D-pad (with a 15° / 45° / 90° turn step) rotates through non-cardinal angles; map view, FPV and depth strip track smoothly -- see 3.1 |
| **R1** | **BUILT 2026-09-25 (not yet watched) -- and the answer was turn SIZE, see 3.3.** **P25's A/B, finally runnable.** `brain/goal_pose.py` is built and default OFF because the sim turned in 90° quanta against a 10° centre band. Wire into `brain/tiered.py`; add median-run-length and reversal metrics to `control/walk_eval.py`; run it. **Before ROS**, so R6 has a baseline | Run-length rises above 1.0; no more LEFT/RIGHT alternation on a stationary target |
| **R2** | **Three routes.** `GET`/`POST /wheels` (per-wheel position + velocity); `GET /world/scan` (`MockWorld` already casts 360 rays, one per degree -- publish the ranges, not only the cells they marked); `GET /world/truth` | A ground-truth ghost on the twin's map. Identical today, which is the point |
| **R3** | **DONE on data 2026-09-26, criterion 4 FAILED and recorded -- see 3.12.** **URDF + TF.** `base_link`, two wheel joints, `laser`, `camera_link` as child of a **revolute pan joint** (ST3215). §900's 11-14cm sensor-to-bumper offset becomes a transform, not a constant. Bearings compose through the pan joint -- the general form of what `goal_pose.py` does by hand | Frames drawn on the map view, swinging as the servo pans |
| **R4** | **DONE on data 2026-09-26, off by default (`drive: ros`) -- see 3.13.** **`picar_sim_hardware`.** Plus `diff_drive_controller`, `joint_state_broadcaster`, `twist_mux` with `AGENT-HARNESS.md` §4.1's order as priorities, and `sim_scan_node` republishing `/world/scan` as `sensor_msgs/LaserScan`. **Exactly one writer to the wheels** from here | D-pad drives through the whole ROS chain; grabbing it mid-mission still ends `preempted`, still names `twin-dpad`, still lapses on silence |
| **R5** | **DONE on data 2026-09-26, off by default (`WORLD_MODE=ros`); criteria 2 and 3 failed on their tight bars -- see 3.14.** **`slam_toolbox` + the error readout.** Bridge serves `/world/pose` and `/world/map` from SLAM instead of `MockWorld` -- the routes the twin already consumes. Then opt-in odometry drift (`sim.odom_drift`, following `sim/sensors.py`'s pattern, default off): without drift there is nothing for loop closure to correct | "map source: sim / slam" toggle, ground-truth ghost, live error number. Drive a lap: error grows, **pose jumps, error collapses**. Hardware cannot show this |
| **R6** | **DONE on data 2026-09-26, on the SCALED house -- see 3.15.** **nav2 + `collision_monitor`.** Costmaps, planner, controller, recovery. `collision_monitor` between the mux and the base, with the footprint term the hand-written collar never had. **`robot/safety.py` is NOT deleted** -- it keeps the teleop and vision-policy paths. Then re-run R1's metric: a DWB/MPPI controller scores continuity in its cost function and should not flicker | Tap a goal on the map, path draws, robot follows. Block it, watch recovery. Read run-length against R1 |
| **R7** | **DONE on data 2026-09-26 -- see 3.16, which also moves the seam: the serial port belongs to `robot/hardware_robot.py`, and `picar_hardware` is not written.** **Fake ESP32 on a pty** speaking `HARDWARE-BOM.md` §4.2's real protocol (`T=1/11/13/126/130/131/136`, `1001`/`1002` frames), and `picar_hardware` written against it. Closes C3's stated blocker: *"nothing in this repo simulates a serial peer"*. Also falsifies §4.2's unverified belief that the heartbeat stops the motors | A drill that severs the link mid-mission; the board's heartbeat expires and reports motors stopped, watchdog quiet |
| **R8** | **Order + bring up.** `JETSON-BOM.md` as priced, plus the **latching e-stop in the motor rail** (1.16 #19, in no bill) and a pack-capacity decision (see `HARDWARE-BOM.md` §6 and the amendment noted in §5 below). `HARDWARE-BOM.md` §5 order unchanged | D-pad moves real wheels; e-stop kills them mid-move with the software none the wiser |
| **R9** | **Swap the BACKEND (3.16): `mode: sim` -> `mode: hardware` + `ROBOT_SERIAL`, and the camera and lidar drivers in place of the sim body's.** `sim_scan_node` -> `sllidar_ros2`. **Nothing above the seam changes.** Then N5's real work: scans sanity-checked in the actual house against glass, mirrors, dark matte, mounting vibration. Re-read P7e here | Same map view, same goal-tap, same recovery -- in a real room. Drive at glass and watch the ring |

---

### 3.1 R0 as built (2026-09-25)

Three files carry it, and the shape is the one the row specified.

* **`sim/grid_world.py`** is no longer a dataclass, because `robot_x`,
  `robot_y` and `heading` are now *views* of the continuous pose rather than
  the state itself, and a field and a property cannot share a name. The
  constructor signature is unchanged, so every caller and every test that
  builds a world by cell and cardinal heading still does -- the getters
  floor, the setters snap to the cell centre. `translate()` and `rotate()`
  are the new primitives; `move()` / `turn_left()` / `turn_right()` are thin
  wrappers kept for the verb layer, and the turns now take degrees.
* **`sim/mock_robot.py`** holds the kinematics and the three chassis
  constants, read off `HARDWARE-BOM.md` 4.3: wheel radius 0.0325m `[V]`,
  1760 counts/rev `[I]`, **track width 0.172m as a flagged PLACEHOLDER**
  (4.3: "unpublished: measure on the chassis"). A test pins the blast radius
  of that one being wrong: a straight line does not depend on it, so a wrong
  value can make the sim pivot at the wrong rate and can never make it
  travel the wrong distance.
* **`sim/mock_world.py` and `sim/renderer.py`** read `world.x` /
  `world.view_angle()` where they read `world.robot_x + 0.5` /
  `HEADING_ANGLE[view.name]`. That is the whole of the change outside the
  two files the phase named.

**Four things worth knowing before extending it.**

1. **The verb layer still means what it meant.** A default `drive_forward()`
   covers one cell and `turn_left()` a quarter turn, because `/action` is a
   verb API and every step budget, recorded demo and safety threshold in
   this repo was measured against that. `WHEEL_MAX_RAD_S` is therefore
   *derived* from `CELLS_PER_SECOND_AT_FULL_SPEED` rather than from the
   motor's datasheet rpm -- the implied 176 rpm happens to sit between the
   part's rated 150 and no-load 300, so it is also a speed the real wheel
   can produce. What changed is that `turn_left(45)` turns 45 degrees
   instead of rounding up to 90.
2. **`stop()` now has something to cancel.** The wheel command is a standing
   command, as a motor driver's is, so a `stop()` that only halted the
   current move would leave the robot integrating forward on the next tick.
   Every failsafe in `control/` (B3.1-B3.3) rests on that.
3. **Collision is deliberately NOT `get_depth_grid()`'s reduction.**
   `cast_ray()` overshoots by up to one `FPV_STEP`, and the depth grid
   subtracts it because it feeds a safety veto and must never overstate
   clearance. Subtracting it in the mover too would leave every ordinary
   one-cell step 1.5cm short of the cell it aimed for and log a wall it
   never touched. The invariant that matters -- the robot's cell is never a
   wall -- holds either way, and is now swept over seven headings by a test
   rather than guaranteed by construction.
4. **N1's prediction held exactly, and that is the phase's real result.**
   `sim/mock_world.py` predicted in writing that a quantised pose was "a
   limitation of the *simulator*, not of the contract" and that C2 would
   make it smooth "without changing one line on either side of the wall".
   Three lines changed in that file and nothing at all in
   `world/interface.py`, `control/remote_world.py` or the twin. That is the
   evidence the wall was drawn in the right place before anything stood
   behind it -- which is the entire bet of (b+) over (c).

**What R0 still owes, stated rather than quietly dropped (section 7's rule).**
The continuous pose is watchable from a phone today: `GET /world/pose`
reports a fractional position and a non-multiple-of-90 bearing, the map view
draws the robot from it, and the FPV and depth strip are both cast from
`view_angle()`. Verified end to end over HTTP -- `POST /action {"action":
"LEFT", "angle": 30}` moves the published bearing from 90 to 60 degrees, and
a `FORWARD` then lands the robot off both grid lines. ~~But no twin control
sends an angle other than 90~~ -- **closed the same day**: the user chose the
angle stepper, and a **15° / 45° / 90° turn step** now sits under the Sim
tab's D-pad, remembered across reloads and defaulting to 90. So R0's "D-pad
rotates through non-cardinal angles" holds from a phone. Press-and-hold
wheel velocity is deferred to R4, where the D-pad goes through `twist_mux`
and a velocity is the natural command.

**Also untouched and worth flagging:** `renderFPV` in `web-twin/app.js` is
still cardinal, so the pre-S2 local-render fallback would draw the wrong view
at a non-cardinal pose. The "frame source: server / local" readout is what
tells you which one drew the picture, and S2's note that `renderFPV` is now
safe to delete has one more reason behind it.

### 3.2 The twin, stripped of its second copies (2026-09-25)

Done alongside R0, prompted by the user telling me two things: that "digital
twin" means the **Guide** tab, and that the **Sim** tab had never been used.

The second is the more useful fact, and it is worth recording rather than
smoothing over. Eleven phases' §7 proofs live in the Sim tab -- M2's depth
strip, M3's path zones, M4's Driving and Last-refusal lines, N1's map, B3's
drills, B4's Remote brain panel, M1/P2/P3's policy picker and tier readouts,
and R0's continuous pose. All were built and signed off; none had been
watched. §7's rule was satisfied by *building* a readout. That is not an
argument against the rule -- it is an argument that the surface has to be one
someone opens, which is why what follows is a reduction rather than an
addition.

**Removed:**

* the **Camera tab** entirely (one panel, one JS block, its CSS). It called
  `/describe` with `/analyze` as a fallback; both routes stay, deployed and
  tested, with no twin client.
* the twin's **hardcoded copy of the house** -- `LAYOUT`, `OBJECTS`, the
  room-boundary rectangles in `roomAt()`, `HEADING_VEC`/`RIGHT_OF`/`LEFT_OF`
  and `CELL_CM` -- and the top-down canvas it fed. N1's map view replaces it,
  drawn from `GET /world/map`: discovered, tri-state, and the same route
  `slam_toolbox` serves at R5. There is no layout to copy for a real room.
* the **cell-shaped telemetry** (room, cardinal facing, free cells, doorway,
  objects). These are `frame_description()`'s grid facts, which
  `robot/interface.py` forbids any hardware-path policy from reading -- and a
  readout is a consumer too.
* the **JS local brain** (`autoStep`, a third copy of the frontier
  algorithm) and the **JS Vision Autopilot**. R4 allows exactly one writer to
  the wheels; a brain that dies with a browser tab was never going to be it,
  and the server's 409 plus M4's authority order already enforce what the
  two-panel arrangement used to.
* **`renderFPV`/`fpvCastRay`** and `tests/test_renderer_parity.py` -- see
  `PLAN-sim-hardening.md` S2, which had licensed this deletion three weeks
  earlier and which R0 turned from safe into necessary.

**Kept, and this is now the whole Sim tab:** the D-pad and its action log,
the Remote brain panel with its policy picker, drills and tier readouts, the
camera canvas, the depth strip, the odometry readout, the world map, and the
frame-source line. Every one of them reads a route the robot serves. That is
what lets the same panel show a simulated room today and a SLAM map of a real
hallway at R5, with no third copy of anything to keep in step.

Net: ~970 lines of JavaScript, and the twin no longer contains a house, a
compass, a renderer or a brain. `tests/test_ui.py` + `test_ui_pipeline.py`
(100 tests) green throughout; full suite 1147 passed.

**The Python cell layer followed the same day, in three stages.**

*A -- `control/` gets the world.* N1 built `/world/pose`, `/world/map` and
`RemoteWorld`, and nothing consumed them. `MissionRunner` now takes a
`world`; `brain_server` builds a `RemoteWorld`, reading its URL off the robot
it actually built (deriving it from `config["robot_url"]` silently pointed any
caller with its own `robot_factory` at a different machine); and
`brain.world_url` exists so R5 can move the pose and map behind the SLAM
bridge with one string. The containment test now covers `world`:
`world.interface` allowed, `world.factory` not, because its `sim` branch
lazily imports the simulator and would pass the existing check.

*B -- the frontier is allocentric.* It asks `get_pose()` for metres and
degrees, predicts where a pivot lands, and buckets visited-ness at the map's
own `resolution_m` -- nav2's frontier search's arithmetic, so it works against
a 5cm SLAM map as well as the sim's 30cm. The three cardinal lookup tables are
gone. **The world is advisory**: a mapper that raises, or cannot even be
constructed, degrades the policy to the right-hand rule with one warning and
never ends or blocks a mission -- the map only chooses between directions the
distance sensor already called clear.

*C -- the grid facts are gone.* `describe_grid_frame()` is deleted and the
free policy's scene is built from sensors by `ConstrainedAgent.sensed_scene()`:
clearance from `SafetyController.path_clearance()` -- so the scene says STOP
exactly where the collar would veto, and nowhere else -- and objects from
perception. The frame lost `position`, `facing`, `free_space_cells` and
`doorway_ahead`; what remains beside the pixels is `room` and
`objects_visible`, the simulator standing in for a detector (1.12), now
computed with the renderer's own visibility test so an object at 45 degrees is
seen and one behind a wall is not. Action acks no longer carry a cell, and
`RemoteRobot._tupleize` went with it. **Sightings carry a map-frame pose**
(`x_m, y_m, heading_deg, map_id`) -- a statement about the house, and the
shape R3's `PoseStamped` will have.

**Kept on purpose:** `GridWorld(robot_x=2, robot_y=2, heading=Heading.E)` as a
constructor, with `robot_x`/`robot_y`/`heading` as convenience properties.
"Put the robot in cell (2,2)" is how a simulated house is authored, by the
starter map and ~30 fixtures; what mattered was that nothing DECIDES in cells,
and nothing now does. **Two test-infrastructure findings:**
`world_over_asgi` built its own app, so pairing it with `robot_over_asgi` gave
a correct map of a different house -- `robot_and_world_over_asgi` fixes that,
and **B0's parity proof now matches action sequences with both halves over a
real socket**, a strictly stronger test than before.

`FEATURES.md` sections 2-3 carry a correction banner and have not been
rewritten.

### 3.3 R1 as built (2026-09-25): the answer was the turn's SIZE

**The premise was incomplete.** R1 was framed as "R0 unblocks P25's A/B of
dead-reckoning". Run for real, the first measurement showed something R0 did
not touch: **every LEFT/RIGHT the brain sent was still the executor's default
90 degrees** -- `safety.check_and_execute(action)` passed no angle -- and
against the tier's 10-degree centre band a 90-degree turn overshoots any
target inside an 80-degree cone. Over sixteen off-axis starts 4-6 cells from
the backpack, with a detector landing every frame, quarter turns closed ZERO
distance and flipped LEFT/RIGHT on 52 of 60 steps. So memory was never the
first problem; the action had no size.

**Built** (user's choice: bearing-sized, over a fixed 15-degree step):

* **A turn chosen from a bearing turns BY that bearing** -- `turn_deg` on the
  tier's scene, `turn_for()` clamping to 5-90 degrees, and `ConstrainedAgent`
  passing it as `angle`. Set by the local steer rung, the dead-reckoning rung,
  and **the cloud's own turns when a local bearing agrees on the side** --
  the cloud keeps the direction (1.11), but `/navigate` answers with no
  magnitude, so a correct "it's to the left" had been going out as a blind
  quarter turn. A scan, a held goal or a disagreeing cloud keeps the default:
  no size nobody measured.
* **1.12's synthetic detections, built at last.** A tiered mission on the Sim
  tab had been loading YOLO and CLIP and running them on raycaster renders --
  forbidden, and blind, so the tier could never steer in the sim. Frames now
  carry `detections` (bearing and range, from the geometry the picture is
  drawn from, out to the renderer's horizon), `FrameReportedPipeline` reads
  them, and `brain_server` picks it by the frame's `metadata.source` -- no
  model loads. The panel's Models line reads "sim ground truth" and every tier
  frame carries `synthesised: true` (a flag that existed, hardcoded False, for
  exactly this).
* **Readouts**: `status.turns` (count, reversals, last size), shown on the
  Remote brain panel as "LEFT 23°" and "N made, M reversed the one before".

**Measured through the whole mission path** (`tests/test_bearing_turns.py`,
`python -m tests.demo_hold_bearing_ab`), collar included:

| turns | detector | closed (cells) | reversals |
|---|---|---|---|
| bearing-sized | every frame | **4.06** | **0.6** |
| quarter (pre-R1) | every frame | -1.23 (ends further away) | 4.8 |

All twelve starts whose straight line clears the kitchen doorway **arrive**.
The four whose line clips the door jamb stop at it, correctly -- going around
is path planning, which is nav2's job; a test pins that as **R6's acceptance
case** and says to promote it when it starts failing.

**What R1 did NOT fix, and it is P25's actual premise.** With a detector that
lands one frame in three, *everything* fails (~0.8 cells, sized or not). Two
causes, both measured: on a blind frame the tier SEARCHES with a quarter turn,
which spins the target out of view before the next sighting; and
dead-reckoning (`tier_hold_bearing`) cannot bridge it, because perception
passes no range, so `goal_pose.py` anchors a DIRECTION -- exact under
rotation (verified: turn 60 away and it answers "RIGHT 60"), useless once the
robot drives. **`tier_hold_bearing` stays OFF**: with sized turns and a good
detector it makes things worse (0.6 -> 12.8 reversals). The repair is to pass
range so the anchor is a POINT (`goal_pose.py` already supports it), and not
to scan on the frames right after a sighting -- or to let R6 own it, where a
sighting becomes a map-frame goal nav2 pursues, which is P7c item 2's design.

**A correction to an earlier number.** A standalone probe run before this
build reported 4.7 cells for the 1-in-3 case. It sized EVERY turn from ground
truth -- search turns included -- and so leaked the answer into turns that
have no bearing. The full-path figure above is the honest one.

### 3.4 Stuck detection (2026-09-25) -- the first phase closed on data

Motivated by the first live R1 run: aimed dead-centre at the backpack from a
row whose straight line clips the door jamb, it said FORWARD into the jamb for
19 steps, paying for cloud calls, until the budget ran out. A mission now ends
`blocked` after `brain.stuck_after` (5) consecutive FORWARDs refused by the
safety layer; any executed move resets the count.

**Acceptance criteria, written before measuring:**

1. Every jamb start ends `blocked`, not `max_steps`, and within 15 steps of
   its first refused FORWARD.
2. Every clear-line start still arrives (within 1.05 cells) -- the detector
   must never fire on the approach.
3. One live mission through the brain's HTTP API ends `blocked` from a jamb
   start, with no cloud call dispatched after the block.

**Measured 2026-09-25 -- all three met, phase closed:**

1. Jamb starts: 4/4 end `blocked`, after 8, 8, 9 and 11 steps in total
   (criterion: within 15 of the first refusal). The run that motivated it
   used 120. Pinned in `tests/test_bearing_turns.py`.
2. Clear-line starts: 12/12 still arrive within 1.05 cells.
3. Live, through the brain's HTTP API on `2f5031a`: from (5.5, 6.5) the
   mission aimed with an 11-degree sized turn, ended `blocked` at step 8, and
   made **one** cloud call in total -- none after the block.

**Found on the way, and it is R1b's first target.** A second live mission,
from the clear start but facing 135 degrees (south-east) with the backpack
out of view, never found it by turning: SEARCH turns go out at the
executor's default 90 degrees against a 60-degree field of view, so from 135
the robot faces 135, 225, 315, 45, 135... and east -- where the backpack is
-- is never in the picture. (First written up as "facing 45"; the pose
readout says 135. The gap arithmetic is the same either way.) **A 90-degree search step leaves a 30-degree blind gap between
views.** The spin guard then forced a FORWARD that took it off the door's
row, and it ended `blocked` at a jamb about a metre short -- which is very
likely what the user's own run did.

### 3.5 R1b -- search that cannot miss (2026-09-25, overnight)

**Hypothesis:** an unsized search turn (the executor's default 90 degrees)
against a 60-66 degree field of view leaves blind gaps, so a target that is
in line of sight but not initially in view is found only if the starting
heading happens to line up. Sizing every unsized tier turn to a SEARCH STEP
smaller than the field of view makes consecutive views overlap, so one
rotation sees the whole circle.

**Acceptance criteria, written before measuring** -- on SEARCH starts: the
row-7 positions of 3.3's clear starts, facing away from the backpack so it is
in line of sight but not in view. *Sampling corrected after the first
baseline, thresholds unchanged:* the first set (every 30 degrees from 60)
could not show the hypothesis -- with 90-degree steps the gap only exists for
offsets 36-54 degrees from a multiple of 90, and that set had none -- and
duly measured 100%. The corrected set is **every 5 degrees, all offsets of
40 degrees or more, both sides** (3 positions x 57 headings = 171 starts):

1. **Found:** at least 95% of search starts detect the target within 12 steps
   (a full rotation at the new step, plus slack). Baseline measured first.
2. **Arrived:** at least 90% end within 1.05 cells of the backpack within 60
   steps, with stuck detection on.
3. **No regression:** 3.3's clear starts still 12/12 arrive, 3.4's jamb
   starts still 4/4 `blocked`.

**Measured -- all three met, phase closed:**

| | before R1b | search step only | + 3-degree steer band | criterion |
|---|---|---|---|---|
| found within 12 steps | 86% | **100%** | 100% (171/171) | >= 95% |
| arrived | 75% | 84% (28 blocked) | **100%** (171/171) | >= 90% |
| clear arrive / jamb blocked | 12/12, 4/4 | 12/12, 4/4 | **12/12, 4/4** | unchanged |

Two changes, each justified by the measurement before it:

* **`SCAN_TURN_DEG = 45`** for every tier turn with no bearing to size it by --
  scans, held cloud goals, cloud turns the local tier cannot size. This
  REVERSES R1's "no size nobody measured" for search turns, on data: that
  choice left a 90-degree step against a 60-degree view.
* **`STEER_BAND_DEG = 3`** for correcting a MEASURED bearing, split from
  `CENTER_BAND_DEG = 10`, which stays as `/navigate`'s reporting vocabulary.
  All 28 blocked runs had driven FORWARD 5-6 degrees off the doorway's line
  -- inside the old 10-degree dead band, harmless while every turn was 90,
  pure drift once turns are sized -- and put a jamb in their own path.

Pinned in `tests/test_bearing_turns.py` on the middle position's 57 offsets,
confirmed red against the pre-R1b tier.

**Live, on `536b13f`:** the exact mission that ended `blocked` a metre short
before R1b -- (5.5, 7.5) facing 135 -- now reaches the backpack (0.00 cells
from its centre) on 3 cloud calls. **It then spins**, and the Turns readout
says so: in the sim **objects are not solid**, so the robot drives onto the
backpack's cell, the target vanishes under its footprint, and the tier
searches for it until the budget runs out. A real backpack would stop the
collar; this is a sim fidelity gap that composes with P7e below.

**Still open -- P25's actual premise.** With a detector landing one frame in
three, sized turns now close 1.61 cells on average (0.83 before R1b), far
from arriving. The repair named in 3.3 stands: pass perception's range so
`goal_pose.py` anchors a POINT, not a direction. (The every-frame row of
`demo_hold_bearing_ab` fell 4.06 -> 3.69 for a known, intended reason: stuck
detection now ends the four jamb starts within ~10 steps instead of letting
them creep closer for 60.) **Also still open: arrival is not recognised**
(P7e) -- every search run ends `max_steps` beside the backpack.

### 3.6 R2, read-only half (2026-09-25, overnight)

**Scope, decided without the user and stated here so it can be undone.**
R2 names three routes. The read-only ones -- wheel state, the scan, ground
truth -- are built. **`POST /wheels` is NOT**: continuous velocity command is
the first path that moves the robot without a verb, which means deciding how
`robot/safety.py` vets a velocity, how M4's authority order applies, and what
the watchdog does to a standing command. Those are safety decisions; its
design is written up in 3.7 for the user.

**One deviation from the row above: `GET /scan`, not `/world/scan`.** A lidar
scan is the robot's own reading -- "how far is everything from me" -- which
CLAUDE.md section 2 classifies as BODY state (egocentric), beside
`get_depth_grid()`. It goes on `RobotInterface` as `get_scan()`, with the same
honest all-unusable default. The world side keeps what the scan is turned
INTO (the map), and `GET /world/truth`, which is allocentric and stays there.

**Acceptance criteria, written before building:**

1. **Contract:** every `RobotInterface` backend answers `get_wheel_state()` and
   `get_scan()` in one shape; a backend with no encoders or no lidar says
   `usable: false` with no numbers -- never zeros.
2. **Scan geometry:** `MockRobot`'s scan equals `renderer.cast_ray()` at every
   beam to within one `FPV_STEP`, and a beam through the kitchen door reaches
   past the doorway.
3. **Truth:** `MockWorld`'s truth equals its pose exactly in the sim today;
   `NullWorld` answers unusable.
4. **Over the wire:** `RemoteRobot` / `RemoteWorld` return exactly what the
   in-process backends do, for all three.
5. **Containment and routing guards stay green** (`control/` imports nothing
   it may not; every public route is accounted for).

**Measured -- criteria 1-5 met, read-only half closed.** Contract: all five
backends answer both new body methods in one shape; the sensorless ones
(`ReplayRobot`, `TeleopRobot`) say `usable: false` with no numbers, and both
wrappers pass through what they wrap. Geometry: every one of 360 beams equals
`renderer.cast_ray()`, the beam through the kitchen door reaches the far
wall (1.95m, not the doorway's 0.75m), and a camera pan leaves the scan
unchanged. Truth equals the pose in the sim; `NullWorld` has none. Over the
wire all three survive the socket unchanged, an older server reads as
`usable: false`, and a 500 still raises. The over-the-wire test was
confirmed to fail against a real bug caught on the way (the route called
`world` where the server's variable is `world_model`). `tests/test_r2_routes.py`
and the contract suites.

**Not built, and not needed by the data rule: the ground-truth "ghost" on the
twin's map.** R2's row names it as the proof; under the data rule the proof
is the equality test above. It becomes worth drawing at R5, when the two
numbers differ.

### 3.6b R1c -- an unreliable detector, and a regression R1b introduced

**Baseline (current code, 69 starts x 3 seeds, detection withheld at random
on in-view frames):** arrival 100% / 84.5% / 73.4% / 53.6% / 47.3% at 100 /
90 / 80 / 50 / 33% per-frame detection. A realistic detector (80-90%) loses
one mission in six. `hold_bearing` (dead-reckoning) does not fix it: 83% at
90%, and its reversal count is polluted by post-arrival dithering (the robot
drives onto the non-solid backpack and orbits it).

**Diagnosis, from traces:** the search sweep passes the target on a missed
frame, and the SPIN GUARD fires -- 8 consecutive turns without a detection
force a FORWARD -- which pushes the robot off the doorway's line so that the
eventual straight approach clips a jamb and ends `blocked`. **That is a
regression R1b introduced**: the guard counts TURNS and was set at 8 when
every turn was 90 degrees (two rotations); R1b's 45-degree search step made
8 turns ONE rotation, so a single missed frame now triggers it. A diagnostic
run with the guard at 16 turns (two rotations again) measured 95.2% at both
90% and 80% detection. *That diagnostic preceded the criteria below; they
are set from the purpose, not from its numbers.*

**Fix:** count DEGREES turned, not turns -- `spin_guard_after` keeps its
meaning of "quarter turns' worth" (8 = 720 degrees = two rotations), so the
guard cannot silently change again when the step size does.

**Acceptance criteria, written before measuring the fix:**

1. At 90% and at 80% per-frame detection, **at least 95%** of missions arrive
   (a realistic detector loses at most one mission in twenty).
2. **No regression:** 100% arrival at 100% detection; every existing test
   (clear, jamb, search) passes.
3. Mean reversals stay **at or below 1**.

**Measured -- all three met, phase closed (narrowly on 1):**

| per-frame detection | arrived before R1c | arrived after | criterion |
|---|---|---|---|
| 100% | 100% | **100%** | no regression |
| 90% | 84.5% | **95.2%** (197/207; 98.6% +/- 0.9 at ten seeds, see 3.9) | >= 95% |
| 80% | 73.4% | **95.2%** (197/207; 98.4% +/- 0.9 at ten seeds, see 3.9) | >= 95% |
| 50% | 53.6% | 81.2% | -- |
| 33% | 47.3% | 56.5% | -- |

Mean reversals 0.2-0.4 throughout. *(First recorded as "the margin on
criterion 1 is thin"; a ten-seed re-measure in 3.9 put both rates near 98.5%
-- the three-seed figure was noise.)* The missions that do fail trace to the
same residue as 3.4: an approach from off the doorway's line clips a jamb,
which is route planning (R6). Below ~50% detection the remaining loss is a
detector too unreliable to steer on at all, and is not this phase's problem.
Pinned in `tests/test_bearing_turns.py` (arrival >= 95% at 90% detection;
the guard allowing two full rotations of 45-degree search), both confirmed
red against the count-based guard.

**Live, on `57b46c2`,** from the verified clear start: the robot reached the
backpack -- 1.00 cell from it by ground truth, read off the new
`/world/truth` -- then lost it, held the real cloud's last FORWARD into a
wall, and stuck detection ended the mission `blocked` after five refusals
and two cloud calls. **It succeeded, and the outcome says otherwise** --
see 3.8.

**One lesson for the next phase that changes a step size:** a threshold
counted in STEPS silently changes meaning when the step does. R1b's search
step halved the spin guard and nothing failed until a detector was allowed
to miss.

### 3.7 `POST /wheels` -- a design for the user to decide, NOT built

The first way to move the robot without a verb, so its safety semantics are
decided before it exists. What R4's `picar_sim_hardware.write()` needs, and a
proposal for each question it raises:

1. **Semantics.** `POST /wheels {"left_rad_s", "right_rad_s"}` sets a STANDING
   command; the server integrates it with `MockRobot.step(dt)` on a fixed
   ~20Hz loop (the rate nav2's controller runs at). On hardware the same
   route forwards the command to the ESP32 (`T=1`) and the board integrates.
2. **Safety -- the decision that matters.** Before each step, the forward
   component of the commanded body velocity is clamped to zero if
   `path_clearance()` is under `min_distance_cm`; rotation is always allowed
   (a differential chassis pivots in place). The clamp is reported, not
   silent: a `refused` field and M4's `last_refusal`. **Open question A:**
   vet REVERSE with the scan's rear beams now that R2 has a 360-degree
   reading, or keep today's rule (no rear check, as for `/action`)?
3. **Authority (M4).** `POST /wheels` carries `x-driver` and is arbitrated
   exactly like `/action`. R4 adds one ROS driver -- `twist_mux` already
   ranks nav2 and teleop INSIDE ROS. **Open question B:** where does that
   driver rank against the brain? Proposal: equal to `brain` (autonomous),
   below the D-pad, so a person still outranks everything.
4. **Watchdog.** A standing command not refreshed within `watchdog_timeout_s`
   is zeroed -- the same silence rule as today, and the same thing
   `diff_drive_controller`'s `cmd_vel_timeout` does inside ROS. Two layers,
   on purpose: the ROS one cannot see a wedged bridge.
5. **Tests before it ships**, as data: a standing forward command into a wall
   stops within one loop period of clearance falling under the threshold;
   a pivot against a wall still turns; silence zeroes the command within the
   timeout; the D-pad preempts it.

### 3.8 The case for deciding P7e now -- for the user, NOT built

P7e (arrival is not recognised) was deliberately left for hardware day,
because its repair resolves differently once a lidar exists. **Tonight's data
says it is now distorting every measurement**, which is a different reason
from the one the deferral weighed:

* **No simulated mission can end `found`.** Every run tonight ended
  `max_steps` or `blocked`, arrived or not -- including the live one above,
  which arrived and was labelled `blocked`. Arrival had to be measured from
  ground truth in every phase (3.3-3.6b), which is exactly what a real room
  cannot provide.
* **It inflates other metrics.** With `hold_bearing` on, reversals read 15.6
  at *perfect* detection -- dithering around a target the robot had already
  reached (3.6b).
* **It compounds a sim fidelity gap:** objects are not solid, so the robot
  drives onto the backpack's cell, loses it under its footprint, and
  searches for something it is standing on (3.5). Making objects solid is a
  sim change with its own cost -- the starter house starts the robot ON the
  sofa's cell, and the rule-based agent's tests are tuned to today's house.

**Proposal, for decision:** a sim-only arrival rule now -- the tier reports
`target_reached` when the detection's range is under an arrival radius and
the target is centred, and the mission ends `found` -- with the lidar version
replacing it on hardware day. Held back only because P7e's deferral was a
deliberate call and this reverses it.

### 3.9 Solid objects (2026-09-26) -- decided by the user

*"Objects must be treated as solid to emulate the real world."* Until now an
object was a label on a floor cell: the robot drove through it, sensors saw
past it, and the map never showed it. A real backpack stops a robot, returns
a lidar beam and appears on a SLAM map.

**Design:** objects are solid to everything that SENSES or MOVES -- collision
(`GridWorld.translate()`), `get_distance()`, the depth grid, the lidar scan,
and `MockWorld`'s map (an object cell is OCCUPIED once seen). They stay
billboards to the CAMERA: the render still draws them as objects rather than
grey wall blocks, and perception's occlusion test still asks only whether a
WALL is in the way (an object behind another is not modelled). The starter
house starts the robot on the sofa's cell, so the sofa moves to the living
room's corner at (1, 1).

**Acceptance criteria, written before building:**

1. **Collision:** driven at the backpack from every clear start, the robot's
   cell is never an object's cell.
2. **Sensing:** facing the backpack from (8.5, 7.5), the scan's forward beam
   reads the backpack's face (0.45 m), not the kitchen wall behind it; the
   scalar distance and the depth grid's path zones see it too.
3. **Map:** once seen, the backpack's cell is OCCUPIED on `/world/map`.
4. **Picture unchanged:** the golden image is byte-identical -- the camera
   draws objects as it always did.
5. **No regression in the tasks that matter:** all 12 clear starts still end
   within 1.05 cells of the backpack (now stopped in front of it rather than
   on it), and every existing test passes or is changed for a stated reason.

**Measured -- all five met, phase closed.** With perfect detection every one
of 69 starts now ends at exactly (9.5, 7.5): stopped by the collar in front
of the backpack, never on it. The forward beam from (8.5, 7.5) reads the
backpack's face at 0.45 m; the map marks it OCCUPIED; the golden image is
byte-identical. Two tests changed, for the reason stated in each: the scan
equals the renderer's ray *with* the solid set, and the kitchen-door beam
now returns off the backpack (1.35 m) rather than the wall behind it. Tests
in `tests/test_solid_objects.py`, confirmed red against the non-solid sim.

**No measurable cost to R1c** -- and a correction to R1c's own record. The
first run read 94.2% at 90% detection, under R1c's 95%, against 95.2% the
night before. Both were three-seed numbers, and with a random detector the
trajectory -- and so which frames get missed -- depends on everything
before it: at 207 missions the noise is about +/- 3 missions. Re-measured
at ten seeds (690 missions per rate), before and after this change:

| | before solid objects | after |
|---|---|---|
| 90% detection | 98.6% +/- 0.9 | **98.3% +/- 1.0** |
| 80% detection | 98.4% +/- 0.9 | **98.3% +/- 1.0** |

So R1c clears its bar comfortably; "95.2%, thin margin" was an unlucky
three-seed estimate. Its test now uses ten seeds.

### 3.10 R2b -- `POST /wheels` (2026-09-26), with the user's decisions

**Decided by the user:** reversing IS checked against the lidar's rear beams.
**Recommended, and the user asked for a recommendation:** the ROS driver
ranks equal to the brain (autonomous), below the D-pad, and **only one
autonomous driver holds the robot at a time** -- industry practice as
`twist_mux` encodes it (e-stop > human > autonomy; one writer to the
motors), and the layering nav2 assumes (mission layer picks goals,
navigation layer moves). A rank is one constant (`DRIVER_PRIORITY["ros"]`).

**A correction to what was said when recommending it:** the robot server did
NOT already keep equal-rank drivers apart -- it deliberately lets equal ranks
through ("so two D-pad taps never fight"), so the brain and a ROS driver would
both have been allowed to drive, interleaved. The fix is narrow: at the
AUTONOMOUS rank the holder is exclusive until it lapses; the manual rank keeps
its pass-through, because two people's taps are one person's intent.

**The rear check applies to every reverse**, the D-pad's `/action` REVERSE
included -- a safety rule that covers one of two ways to back up is not one.

**Acceptance criteria, written before building:**

1. **Forward into a wall** under a standing command: motion stops before the
   path clearance falls more than one control period's travel below
   `min_distance_cm`, and the clamp is reported.
2. **A pivot against a wall still turns** -- rotation is never clamped.
3. **Reverse into a wall** is clamped using the scan's rear beams, on
   `/wheels` and on `/action` REVERSE alike.
4. **Silence zeroes a standing command** within the watchdog timeout.
5. **Authority:** the D-pad preempts `/wheels`; `brain` and `ros` exclude each
   other while one holds the robot; two D-pad taps still never fight.
6. **A backend that cannot take wheel velocities** (teleop, replay) refuses
   with a reason, never silently ignores the command.

**Measured 2026-09-26 -- all six met** (`tests/test_wheels_command.py`, one
test per criterion, a real app with its 20 Hz control loop and watchdog on
wall-clock time):

1. A standing 0.1 m/s command held for 3 s (30 cm of travel from 30 cm of
   clearance) stops at **19.5 cm**, inside the half-centimetre one control
   period allows. With the clamp removed the same command drives to **0.0 cm**.
2. Pure rotation against the wall is never clamped and the heading changes.
3. The second REVERSE from the start is refused (`safety_distance`, "rear"),
   and a negative `/wheels` velocity is clamped for the same reason.
4. Silence zeroes the standing command within the watchdog timeout.
5. The D-pad preempts `ros`; `brain` and `ros` refuse each other while one
   holds the robot ("one autonomous driver at a time"); D-pad taps still pass.
6. `TeleopRobot` refuses `/wheels` with `unsupported`.

**Each criterion was confirmed red against a mutation first**: forward clamp
off -> criterion 1 fails at 0.0 cm; rear check off -> criterion 3 fails;
autonomous exclusivity off -> criterion 5 fails. The first version of
criterion 1 passed with the clamp removed -- one 0.9 s command covers 9 cm and
never reached the collar -- which is exactly the test this check exists to
catch; it now re-sends the command at 3 Hz, as a controller would.

### 3.11 Arrival recognised (2026-09-26) -- P7e's first half, decided by the user

3.8 proposed a *sim-only* rule. The user asked for a recommendation and took
this one instead: **the rule the car will run, tested first in the sim.** A
sim-only rule would read the simulator's own detection distances -- a fact
only the simulator has -- and would pass here while saying nothing about the
robot. The deferral's premise (the repair "resolves differently once a lidar
exists") stopped holding at R2, when the sim gained a scan.

**The rule** (`brain/arrival.py`, applied in `MissionAgent` between perception
and decision, so it sees the scene and holds the robot):

1. local perception reports the target `detected`;
2. its bearing is within `STEER_BAND_DEG` (3 degrees) of dead ahead;
3. **the range sensor** -- `get_scan()`, the lidar -- reads at most
   `ARRIVAL_RADIUS_M` (0.40 m) within +/- 2 degrees of that bearing. Never
   the detector's own distance;
4. all three on `ARRIVAL_FRAMES` (2) consecutive frames.

Then the scene becomes `target_reached`, the action `STOP`, and the mission
ends `found`. Nothing overrides it -- the steer-over-hold precedence that P7e
watched drive into a basket never sees the frame. It refuses to judge rather
than guess when it cannot: no usable scan (teleop, replay), or a panned
camera (the bearing is then not body-relative, `brain/perceive.py`'s note).
Policies with no local perception (rule-based, cloud-only vision) are
untouched.

**Radius, from data rather than taste.** Over the 69 starts of 3.5/3.6b the
scan at the target's bearing reads 0.465 m one move out and 0.165 m where the
collar stops the robot (a 0.30 m move). 0.40 m therefore fires only at the
pose missions already stop at, so the ground-truth arrival metric
(<= 1.05 cells) cannot move; on continuous motion it fires 40 cm out.

**Acceptance criteria, written before building:**

1. **Recognition:** of the missions that physically arrive (ground truth
   <= 1.05 cells), >= 95% end `found` -- at perfect detection over the 69
   starts, and at 90% and 80% per-frame detection over the 10-seed sweeps.
2. **No false arrival:** zero missions end `found` more than 0.60 m (2 cells)
   from the backpack's centre, across every sweep including the jamb starts.
   Ground truth is read by the test only, never by the rule.
3. **No regression:** R1/R1b/R1c's arrival tests pass unchanged.
4. **Honest degradation:** no usable scan, or a panned camera, never
   produces `found` locally.
5. **Live:** at least one mission through the brain's HTTP API ends `found`.

**Measured 2026-09-26** (`tests/test_arrival.py`, 12 tests):

| detection | missions | arrived (truth <= 1.05 cells) | of those, `found` | farthest `found` |
|---|---|---|---|---|
| 100% | 69 | 69 | **69 (100%)** | 0.30 m |
| 90% | 690 | 678 | **676 (99.7%)** | 0.386 m |
| 80% | 690 | 678 | **676 (99.7%)** | 0.386 m |

Criterion 1 met; criterion 2 met (no `found` beyond 0.386 m, bar 0.60 m; the
four jamb starts end `blocked` at 3.1 cells, as they should); criterion 3 met
(every R1/R1b/R1c test green unchanged, 1230 in the suite); criterion 4
pinned (no scan, a panned camera, a policy without local perception). With
the rule switched off the same sweep reads **0 of 69**.

**Criterion 5, live on `21433f8`** through the tunnel and the brain's HTTP
API: the robot driven by D-pad to the hallway start (5.5, 7.5) at +30 degrees
off the target, then `policy: "tiered"` -> **`found` in 7 steps, 1 cloud
call**, one 30-degree sized turn, stopped at (2.85, 2.25) m with the scan at
0.165 m on a 0.0-degree bearing, streak 2. A first live attempt from the
house's default start in the living room ended `blocked` without ever seeing
the backpack -- no line of sight to the kitchen, and a held cloud FORWARD into
a wall: that is search and routing (R6), not arrival.

**Criterion 2 caught a real defect in the first version.** The range at the
bearing was the NEAREST return within +/- 2 degrees, and one 90% mission
declared `found` 95 cm out: stuck on the kitchen door jamb, target dead
centre, the jamb's edge two degrees left at 0.195 m against 0.81 m at the
bearing itself. The range is now the MEDIAN of the window, pinned by its own
test. That is the "facing something else" case the criterion was written for.

**Residual, recorded rather than tuned away:** the 2 arrivals in 678 that do
not end `found` are a detector alternating hit/miss at the target, so the
two-frame streak resets every other frame and stuck detection (5 refused
FORWARDs) ends the mission `blocked` first. "2 of the last 3" would recover
them; it is a different rule from the one measured, and 99.7% does not ask
for it. (80% and 90% read identically because the failing seed's draws
almost never fall between 0.8 and 0.9 -- checked, not a harness fault.)

**What this does not settle:** whether the real detector still recognises a
target at 40 cm. That is a rig walk's question, and the rule needs no change
to ask it -- on a phone walk it refuses to judge (no scan) until the car has
a lidar.

### 3.12 R3 -- URDF + TF (2026-09-26): criteria, written before building

**What it is.** `service/slam/picar_description/urdf/picar.urdf.xacro`:
`base_link` at the midpoint of the drive axle (the diff-drive rotation
centre, REP-105), two `continuous` wheel joints, `laser` fixed at the deck
centre, `pan_link` on a `revolute` joint (the ST3215), `camera_link` fixed to
it at the pan head's pitch. `base_footprint` below `base_link` for nav2.

**Constants.** From `HARDWARE-BOM.md` / `JETSON-BOM.md` where they exist:
wheel radius 0.0325 m, track 0.172 m (**placeholder**, as in `sim/mock_robot.py`),
deck 0.228 x 0.148 m. Where no document gives one, a flagged placeholder in
ONE block at the top of the xacro, never scattered: lidar height, camera
height (0.12 m, Stage 0's 10-13 cm), the pan joint's forward offset, and the
camera's downward pitch (the wedge "still has to be modelled", 10-20 degrees).

**Acceptance criteria:**

1. **Valid.** `xacro` expands and `check_urdf` passes inside the container;
   `robot_state_publisher` starts and publishes the full tree.
2. **Single-sourced chassis.** The URDF's wheel radius and wheel separation
   equal `sim/mock_robot.py`'s `WHEEL_RADIUS_M` and `TRACK_WIDTH_M` exactly --
   a test fails if either side drifts. (R4's controller config reads the same
   two numbers.)
3. **TF is what the URDF says.** `base_link -> laser` and
   `base_link -> camera_link` as published by `robot_state_publisher` (looked
   up with tf2 inside the container) match a pure-Python evaluation of the
   same file within 1 mm and 0.1 degree, at pan angles -90, -45, 0, 45 and 90
   degrees. The Python side reads XML only -- no ROS on the laptop.
4. **How wrong is the hand shortcut?** `goal_pose.py` and the tier compose
   a panned bearing as pan + in-frame azimuth, which ignores the pan axis
   sitting ahead of `base_link`. Through TF, over pan -90..90 and targets at
   0.4-3 m, measure the shortcut's error against the exact composition.
   **Pass if it is under the 3-degree steering band for every target at
   >= 1 m** -- then the shortcut is safe where the tier steers, and the
   measured table says where it is not. (Amended before any measurement: as
   first written, "within 0.5 degree at 3 m", it was impossible by geometry
   -- an 8 cm offset is ~1.5 degrees of parallax at 3 m and 90 degrees of
   pan, which is the very error the criterion should be quantifying.)
5. **Containment holds.** `tests/test_ros_containment.py` green: every
   `rclpy` import lives under `service/slam/`.

The "frames drawn on the map view" proof is not built -- under the
data-driven rule it is a UI nicety, and criterion 3 is the measurement it
stood for.

**Measured 2026-09-26 -- four of five met, criterion 4 FAILED**
(`tests/test_urdf.py`; the live half skips without a container):

1. Met. `xacro` + `check_urdf` pass in the container; the tree is
   `base_footprint -> base_link -> {front_bumper, rear_bumper, laser,
   left_wheel, right_wheel, pan_link -> camera_link -> camera_optical_frame}`,
   and `robot_state_publisher` publishes all of it.
2. Met. Wheel radius and separation are one number across the xacro,
   `controllers.yaml` and `sim/mock_robot.py`, pinned always-run.
3. Met. 15 tf2 lookups (`laser`, `camera_link`, `front_bumper` x pan -90,
   -45, 0, 45, 90) match numpy forward kinematics of the expanded URDF within
   1 mm and 0.1 degree.
4. **Failed.** The shortcut pan + in-frame azimuth, against the exact
   composition through TF, worst case over target bearings within the view:

   | range | pan 0, target within 30 deg | pan 0, target within 3 deg (centred) | pan +/-90 |
   |---|---|---|---|
   | 0.4 m | 6.90 | **0.75** | 11.53 |
   | 0.7 m | 3.63 | 0.39 | 6.56 |
   | 1.0 m | 2.46 | 0.26 | **4.59** |
   | 1.5 m | 1.60 | 0.17 | 3.06 |
   | 3.0 m | 0.78 | 0.08 | 1.53 |

   The pan axis sits 8 cm ahead of `base_link` (placeholder), and a bearing
   taken from there is not a bearing from the rotation centre. **What the
   failure does and does not reach:** with the camera centred and the target
   inside the steering band the shortcut is within 0.75 degree even at
   0.4 m, so the tier's final approach and `brain/arrival.py` (which refuses
   panned frames) are sound. A turn sized from an off-centre target up close
   is off by up to ~7 degrees and re-measured next frame. **A panned bearing
   is not usable without the geometry** -- and the geometry needs a range,
   because a bearing alone from an offset camera is ambiguous. Two options,
   neither taken here: compose panned sightings through TF with the lidar's
   range (the ROS side, R6), or mount the pan axis over the rotation centre
   (a chassis decision for hardware day). Until one is taken, nothing may
   consume a panned bearing as body-relative.
   **And the sim cannot show any of this:** it renders from the robot's
   centre, so its camera has no offset. Moving the sim camera to the URDF's
   pan axis is the follow-up that would make the sim exhibit the error.
5. Met. `tests/test_ros_containment.py` green, now load-bearing: the bridge
   imports `rclpy`, inside `service/slam/` only.

### 3.13 R4 -- `picar_sim_hardware` and one writer (2026-09-26): criteria, written before building

**The design call the table leaves open: how a verb reaches `twist_mux`.**
`robot/server.py` may not import `rclpy`, and the D-pad and the brain speak
verbs (`/action FORWARD`, `LEFT 45`), not velocities. So:

* A **bridge node** in the container (`picar_bridge`, rclpy, HTTP on :8090)
  takes `POST /cmd_vel {driver, linear_m_s, angular_rad_s}` and publishes a
  `Twist` on that driver's `twist_mux` input -- `cmd_vel/teleop` for
  `twin-dpad` (priority 100), `cmd_vel/brain` for `brain` (50), `cmd_vel/nav`
  reserved for nav2 (50, R6). `twist_mux` -> `diff_drive_controller` ->
  **`picar_sim_hardware`** (C++, `hardware_interface::SystemInterface`), whose
  `write()` is `POST /wheels` with `x-driver: ros` and whose `read()` is
  `GET /wheels`. `sim_scan_node` republishes `GET /scan` as `LaserScan`.
* **Verbs become velocity profiles, closed on the encoders**, in
  `robot/ros_drive.py`: a `RobotInterface` wrapper that streams twists to the
  bridge until the wheels' own encoder counts say one move (0.30 m) or the
  commanded angle has been covered, then zeroes. Picked by
  `robot/factory.py` from `drive: ros` -- the only place a backend is chosen.
  The inner `MockRobot` still answers every read.
* **Authority stays where M4 put it, and moves inside ROS for the wheels.**
  `/action` is arbitrated exactly as today (so a D-pad tap still preempts a
  mission and names `twin-dpad`). Under `drive: ros`, `POST /wheels` is the
  actuator's route: only driver `ros` may use it, and it is not arbitrated
  against `/action`'s drivers -- they reach the wheels THROUGH it. The safety
  vet and the wheel loop's re-vet stay on it unchanged.

**Acceptance criteria:**

1. **The chain moves the robot.** A 0.1 m/s twist on `cmd_vel/brain` for 3 s
   moves `MockRobot` 0.30 m +/- 5% (ground truth), and
   `diff_drive_controller`'s `/odom` agrees with `/world/truth` within 2 cm.
2. **One writer.** Under `drive: ros`, every wheel command the sim receives
   arrives through `POST /wheels` from `ros`; the count of verbs executed
   directly on `MockRobot` is 0.
3. **Verbs through the chain.** `/action FORWARD` moves 0.30 m +/- 2 cm,
   `LEFT 45` turns 45 +/- 2 degrees, `RIGHT 90` 90 +/- 2.
4. **Authority preserved.** A running brain mission, then a D-pad tap: the
   mission ends `preempted`, the log names `twin-dpad`, and authority lapses
   on silence -- M4's existing acceptance, re-run under `drive: ros`. Inside
   ROS, a twist on `cmd_vel/teleop` outranks one on `cmd_vel/brain`.
5. **Safety preserved.** A standing forward twist into a wall stops at
   >= 19.4 cm clearance (R2b's number, now through ROS).
6. **Silence stops it twice.** Twists stop: `diff_drive_controller`'s
   `cmd_vel_timeout` zeroes the wheels within 0.5 s; kill the container:
   the robot server's watchdog zeroes them within `watchdog_timeout_s`.
7. **The scan crosses.** `/scan` equals `GET /scan` beam for beam within
   1 mm, at >= 5 Hz.
8. **A mission still finds the target**, live through the brain API with
   `drive: ros`, from 3.11's hallway start.

**Measured 2026-09-26 -- all eight met**, live
(`tests/test_ros_chain_live.py`, 13 tests, skips without the stack) and
always-run (`tests/test_ros_drive.py`, 9 tests, the same verb executor
against a fake chain with the measured latency and jitter):

1. Met, with its measurement amended. Every wheel command during a 0.1 m/s
   twist arrives as exactly 0.1 / r = 3.077 rad/s, and `/odom` equals
   `/world/truth` (0.3582 m against 0.3582 m). **"0.30 m +/- 5% in 3 s" was
   the wrong instrument:** an open-loop timed stream measures the HTTP
   client's clock, and seven runs read 0.267-0.316 m with the wheels
   receiving the exact velocity throughout. Distance accuracy is criterion
   3's, where it is closed on the encoders.
2. Met. Under `drive: ros` every verb goes out through the chain (the
   health readout counts them) and `POST /wheels` refuses any driver but
   `ros` (`not_the_actuator`); the always-run test forbids the robot's own
   verbs outright and they are never called.
3. Met: 14 live verbs, moves within 4.4 mm of 0.30 m and turns within 0.64
   degree. **It took three tunings, and the first is worth recording:** at
   2 rad/s with a ramp gain of 3/s, LEFT 45 came out at 59-74 degrees. The
   chain carries 40-150 ms of jittery delay (measured with a step), and a
   proportional ramp over a delay settles only when gain x delay is well
   under 1. Now 1.2 rad/s, gain 1.5, a 0.1 rad/s floor, and a SIGNED settle
   pass that drives back after a latency spike. The fake chain in the
   always-run test carries that jitter; with a fixed delay it passed the
   overshooting tuning, so it was made jittery until it failed it.
4. Met. A D-pad tap mid-mission ends it `preempted`, the log names
   `twin-dpad`, and authority lapses on silence -- M4, unchanged, through
   ROS. Inside ROS a teleop twist beats a brain twist sent in the same
   period (heading moved, position did not).
5. Met. A standing forward twist into a wall stops at >= 19.4 cm.
6. Met, after a config change made before measuring: silence inside ROS
   stops the wheels within 0.5 s once `twist_mux`'s timeouts and
   `cmd_vel_timeout` are 0.25 s each -- the two ADD, so at 0.5 each it would
   have been a full second. Killing the container freezes the last command
   as a standing one, and the robot server's own watchdog stops it within
   its timeout.
7. Met. `/scan` equals `GET /scan` beam for beam within 1 mm, at ~10 Hz.
8. Met. Live, `drive: ros`, hallway start: **`found` in 7 steps and 12 s**, 1
   cloud call, all 5 of the mission's moves through ROS (243 wheel posts),
   stopped 0.36 m from the backpack.

**A defect found on the way, fixed:** `picar_sim_hardware` returned ERROR
after 20 failed HTTP cycles, and ros2_control then deactivates the component
for good -- which happened across a routine restart of the robot server. The
controllers still read `active` and commanded nothing. An unreachable robot
server is not a hardware fault here (the robot server's watchdog is what
keeps it safe), so the plugin now keeps trying and says so; a robot that
reports no wheels at all is still an ERROR.

**How to run it** -- *the complete, current instructions (secret, ports,
Linux/Jetson, health checks, failure signatures) are `service/slam/README.md`;
this is the R4-era short form:* `docker build -t vision-picar-ros service/slam`, then
`docker run -d --name picar-ros -p 8090:8090 -e APP_SHARED_SECRET
-e ROBOT_URL=http://host.docker.internal:8000 vision-picar-ros ros2 launch
picar_bringup picar.launch.py`, and restart the robot server with
`ROBOT_DRIVE=ros`. The default stays `drive: direct`, so the twin does not
depend on Docker being up.

### 3.14 R5 -- `slam_toolbox` and the error readout (2026-09-26): criteria, written before building

**What it is.** `slam_toolbox` (online async) in the container, fed by
`/scan` and `diff_drive_controller`'s `odom -> base_footprint`. The bridge
serves its pose and map; **`world/ros_world.py`** -- the file `PLAN-mapping.md`
named for N6 -- is the `WorldInterface` backend over them, chosen by
`world: mode: ros`. It converts on ITS side of the wall: ROS's x-forward /
y-left / CCW yaw becomes the project's x-east / y-south / clockwise compass.
Default stays `world: sim`.

**The one design decision: aligning SLAM's frame to the house.** SLAM's map
frame starts wherever the robot happened to be. On hardware that is the map,
full stop -- `map_id` says which one. In the sim, to subtract the estimate
from the truth, `RosWorld` anchors the SLAM frame to the house ONCE, at first
contact, from ground truth (a rotation and a translation) -- the standard
"align the first pose" of trajectory evaluation. After that the truth is
never read by the estimate. Stated because it is the one place truth touches
the pose, and it is a frame choice, not a measurement.

**Opt-in odometry drift** (`sim.odom_drift`, default off, following
`sim/sensors.py`'s pattern): the ENCODERS misreport -- each wheel's reported
position is scaled by its own factor -- while the robot moves truly. So
`diff_drive_controller`'s odometry drifts exactly as a mis-calibrated wheel
radius makes it drift on hardware, and so do R4's verbs, which close on the
encoders.

**The lap.** A fixed route of D-pad verbs through ROS: start room -> hallway
-> kitchen door -> back to the start, about 7 m and six 90-degree turns,
ending where it began (a loop-closure opportunity).

**Acceptance criteria:**

1. **The contract holds.** `RosWorld` passes `tests/test_world_contract.py`
   against a fake bridge; a container restart yields a new `map_id`.
2. **SLAM tracks.** Drift off, the lap: SLAM's pose within 5 cm and 2
   degrees of ground truth at every 1 Hz sample.
3. **SLAM corrects what odometry cannot.** Drift on (the right encoder
   reads 3% long), the same lap: odometry's error at the end >= 20 cm (the
   drift is real), SLAM's error <= 10 cm and 3 degrees at every sample and
   at the end. Whether a loop-closure JUMP appears is recorded, not
   required -- scan matching may keep the error too small to need one.
4. **The map is the house.** >= 90% of SLAM's occupied cells lie within
   10 cm of a true obstacle surface; <= 1% of its free cells lie inside a
   true wall or object.
5. **The error is readable.** `GET /world/error` gives position and heading
   error, sim-only and named so (`usable: false` without truth); the twin's
   map view shows it and the truth as a ghost when the world source is
   `ros`. UI test plus a phone-size screenshot.

**Measured 2026-09-26 over eighteen laps -- 1, 4 and 5 met; 2 and 3 FAILED
on their tight bars, and what does hold is pinned**
(`tests/demo_slam_lap.py` is the instrument; `tests/test_slam_live.py`
asserts only what held on every lap).

**First, the instrument was wrong once and is recorded so it is not wrong
again.** Criterion 2 said "at every 1 Hz sample". Samples taken WHILE MOVING
compare a SLAM pose ~150 ms old with the live truth: at 1.2 rad/s that alone
is ~11 degrees, and odometry -- exact by construction with drift off -- read
up to 7.9 cm "wrong" in motion and 0.0 at rest. So the numbers below are
**at rest**, 0.4 s after each verb. In motion the maxima were 10-26 cm and
7-25 degrees; that is the pose's latency, a real property R6's nav2 handles
by timestamping, not SLAM's accuracy.

1. **Met.** `RosWorld` is in the contract suite with a fake bridge; a
   restarted container is a new `map_id`.
2. **FAILED.** Drift off, 9 laps, worst at-rest error per lap:
   position 5.2, 4.7, 6.8, 6.7, 8.2, 6.8, 6.1, 4.0, 5.9 cm (bar 5 cm: 2 of 9);
   heading 2.5, 0.8, 1.2, 1.0, 0.8, 1.2, 1.0, 2.0, 1.6 deg (bar 2 deg: 7 of
   9); every lap ENDS within 0.1-1.4 cm and 0.6 deg. The mid-lap error builds
   across the long rooms and falls back as the lap closes. **Its cause is not
   established.** A narrow-corridor explanation was offered and withdrawn --
   the map shows the "hallway" is a room several cells wide. It is about one
   map cell (5 cm), which is where a first suspicion belongs.
3. **FAILED on heading, met on position and on the headline.** Right encoder
   3% long, 9 laps: SLAM's position stays within 10 cm at rest on all 9
   (worst 2.0-8.3 cm) and ends within 1.0-4.5 cm on all 9, while
   **odometry ends up to 99 cm and 54 degrees off**. Heading at rest stays
   under 3 degrees on only 4 of 9 (worst 2.1-7.3; a 1.0 s settle helps, 2.7-3.5,
   so part of it is SLAM correcting after a turn). Odometry's end error was
   >= 20 cm on 6 of 9 -- the other three laps drove into walls early, because
   **drift bends R4's verbs too**: they close on the lying encoders, and one
   lap had 15 of 28 moves blocked. Closing on the MAP pose is nav2's job (R6).
4. **Met on every lap.** 95.9-100% of SLAM's occupied cells lie within 10 cm
   of a true wall or object; 0-0.73% of its free cells lie inside one.
5. **Met.** `GET /world/error` (sim-only; `usable: false` without truth) and
   the twin's map: the truth as an outlined ghost and "SLAM error 5.5 cm /
   1.1 deg · odometry alone 30 cm / 19 deg" -- only when the map is SLAM's.
   Two UI tests, the first confirmed red on the old twin.

**A defect the phone-size screenshot found, fixed:** the twin read "SLAM
error 0.0 cm" after eight moves. `RosWorld` anchored SLAM's frame to the
house at FIRST CONTACT, and the page first asked after the moves, so the
anchor swallowed the error. The anchor is now the truth at the session's
START -- the bridge records it at odometry zero, sim-only and never published
into ROS -- pinned by a regression test. The lap numbers above are unaffected:
their sampler asked before the first move.

**Tried and made no difference, kept because it is right:** stamping each
scan when it was TAKEN (the robot server's `stamp_unix`) rather than when it
reached the bridge. **Corrected at R6 (3.15): it was not right.** A capture
stamp is often ahead of the newest odometry transform, which deadlocked
nav2's costmaps; scans are now stamped with the newest time the transform
tree covers. `stamp_unix` is still served, and not used for the ROS stamp.

### 3.15 R6 -- nav2 and `collision_monitor` (2026-09-26): criteria, written before building

**What it is.** nav2 on SLAM's map: global and local costmaps with the
chassis' real footprint (0.228 x 0.198 m), a planner, the Regulated Pure
Pursuit controller, behaviours (spin, back up, wait), and `collision_monitor`
between `twist_mux` and `diff_drive_controller`. Goals enter through the
bridge (`POST /goal` in the house frame, converted by `world/ros_world.py`
the way poses are), and the robot server exposes them as `POST /world/goal`,
`GET /world/goal` and `DELETE /world/goal`. Under `WORLD_MODE=ros` only.

**Open question 3, decided: the two collars run in SERIES.** nav2 -> `twist_mux`
-> `collision_monitor` -> `diff_drive_controller` -> `picar_sim_hardware` ->
`POST /wheels` -> `robot/safety.py`'s vet. Each can only slow or stop, never
speed up, so they cannot disagree about what is allowed -- the more
conservative wins -- and `robot/safety.py` stays the last word on every path,
nav included. What series costs is that the tighter one decides, and the
stop/slow readouts must name which collar acted.

**Open question 2, decided for now: the tier keeps steering by verbs.** R6
adds nav2 as a way to reach a GOAL; it does not re-plumb `brain/tiered.py`.
Turning the tier's sightings into nav2 goals is a mission-policy change with
its own A/B against R1's baseline, and belongs in its own phase once nav2 is
measured -- not folded into the phase that measures it.

**A person still outranks everything.** A D-pad verb goes on
`cmd_vel/teleop` (priority 100, over nav's 50), and the bridge CANCELS an
active nav2 goal when a teleop twist arrives: M4's preemption, inside ROS.

**Acceptance criteria:**

1. **Goals are reached.** Six goals across the house, from the start, each
   in a different place (start room, the big room, the lower room, near the
   kitchen door, back): >= 5 of 6 end SUCCEEDED, and each success ends within
   0.20 m of the goal by ground truth.
2. **No contact.** Over every goal run, the robot's true centre never comes
   within 0.10 m (the chassis' half-width) of a wall or object surface --
   measured from ground truth at >= 5 Hz, because the sim's own collision
   check is one ray ahead and cannot see a side-swipe.
3. **An unreachable goal ends, stopped.** A goal inside a wall: nav2 answers
   ABORTED (or REJECTED) within 60 s and the wheels are at zero.
4. **A person outranks the plan.** A D-pad tap during a goal: the goal is
   CANCELED within 1 s and the robot does what the tap said.
5. **It does not flicker.** Angular-velocity sign reversals (|omega| >
   0.1 rad/s) in the commands actually applied to the wheels: <= 1 per metre
   travelled, pooled over the goal runs. R1's reversals-per-mission cannot be
   compared directly -- verbs against a continuous command -- so this is the
   continuous form of the same question.
6. **The collars in series, observed.** Record how often each acted:
   `collision_monitor` stops/slows and `robot/safety.py` clamps. No
   threshold; the numbers decide whether either is redundant.

**Measured 2026-09-26 -- all six met, on the SCALED house; the starter house
cannot host nav2 with this chassis.** Instrument `tests/demo_nav_goals.py`
(a mapping lap, then the goals, judged by ground truth at 5 Hz); pinned in
`tests/test_nav_live.py`.

| run (final image) | goals | end error (m) | closest to a surface | reversals / m | unreachable | tap -> canceled | `safety.py` clamps | monitor stops / slows |
|---|---|---|---|---|---|---|---|---|
| 1 | **6/6** | 0.090-0.122 | 0.169 m | 16 / 19.9 = **0.81** | aborted 19.0 s, stopped | **0.043 s** | 0 | 0 / 4 |
| 2 | **6/6** | 0.096-0.133 | 0.165 m | 15 / 19.8 = **0.76** | aborted 23.7 s, stopped | **0.048 s** | 0 | 1 / 11 |

Bars: >= 5/6 within 0.20 m; >= 0.10 m; <= 1 per metre; aborted within 60 s,
stopped; within 1 s. Criterion 6's answer: over ~40 m of goals
`robot/safety.py` never had to clamp a nav command and `collision_monitor`
stopped one once -- nav2 plans clear of things by itself, and the two collars
are insurance in series, not load-bearing. Both stay.

**It took five failures to get here, and each was a real finding:**

1. **The starter house's doors are too narrow for the chassis.** 30 cm doors
   (one grid cell, sized for the retired PiCar-X) leave ~5 cm a side; with
   the footprint's inscribed radius nav2's costmap seals a door whenever SLAM
   draws a jamb one 5 cm cell thick. Starter-house runs went 5/6, 1/6 and
   0/6. So `sim/maps/scaled_house.py` (`SIM_MAP=scaled_house`): the same
   0.30 m cell, 90 cm doors, 3 m rooms. Real doors are 70-90 cm; the starter
   house stays for every test written against it.
2. **nav2 cannot plan into a room SLAM has not seen** ("goal off the global
   costmap"), so the instrument maps first -- the ordinary order.
3. **`collision_monitor`'s STOP polygon froze the robot.** Footprint + 2 cm
   reached a door jamb 15.7 cm from the centre; Humble's stop action then
   refuses turning AWAY too and, after 2 s, publishes nothing, so nav2's own
   recoveries failed 16 times. Now `approach`: the exact footprint projected
   along the commanded velocity, which lets it rotate clear.
4. **`slam_toolbox` stamps `map -> odom` with its last PROCESSED scan**, and
   processes one only after the robot moves -- so at rest the transform ages
   until nav2 refuses it. The fix is upstream's `restamp_tf`, which the apt
   release (2.6.10) lacks although the branch at the same version has it;
   `slam_toolbox` is now built from a pinned commit in the image.
5. **Scan timestamps deadlocked nav2's costmaps.** The hardest one: the
   planner planned from the START position and the controller declared every
   goal "reached" in 0.08 s. Both costmaps' TF buffers had stopped taking ANY
   transform seconds after start, while `bt_navigator`, the bridge and fresh
   listeners stayed current; Cyclone DDS (now the image's RMW, per Nav2's own
   Humble advice) did not change it, and neither did clock skew (0.03 s) or
   shared memory (12% used). The cause was R5's decision to stamp scans at
   CAPTURE: a capture stamp is often a few ms ahead of the newest odometry
   transform, each costmap's tf2 `MessageFilter` holds the scan waiting, and
   on Humble that path deadlocked their listeners. Arrival stamps made it
   rarer (one freeze in three runs); stamping each scan with the newest time
   the transform tree already covers was believed to make it impossible --
   **wrong, see the correction below: it was a tf2 deadlock all along**. And the controller hid it: `isGoalReached()` ignores a failed
   transform and compares the robot with a default pose at odometry's origin,
   so a robot near its start "reached" goals metres away.

**CORRECTED 2026-09-26, later the same day: finding 5's cause was wrong.**
The freeze came back on the first run in the user's own house, 55 s after
start, with TF-bounded stamps in place -- so "makes it impossible" above is
false, and three clean scaled-house runs were luck. A debugger on the frozen
`controller_server` settled it: thread 13, a costmap's scan callback, held
`tf2_ros::Buffer`'s lock inside `waitForTransform` and was waiting for
`BufferCore`'s; thread 12, the TF listener, held `BufferCore`'s inside
`testTransformableRequests` and was waiting for the Buffer's. **An ABBA
deadlock inside ROS 2 Humble's tf2** -- fixed upstream in tf2/tf2_ros
0.25.24 (2026-09-15, ros2/geometry2 #982, backported #990) and not yet in
apt, which still ships 0.25.23. The image now builds both from the 0.25.24
tag, and nav2's prebuilt binaries load the fixed library. Scan stamps were
never the cause, only a change in how often a scan waited: every stamping
scheme left the race open. The TF-bounded stamp stays, as harmless.

**The lesson that generalises:** R5 recorded scan stamping as "tried, no
difference, kept because it is right". It was a latent fault that only a
consumer with a `MessageFilter` could trip. A change measured against one
consumer is not measured against the next one.

### 3.16 R7 -- the motor board, faked on a serial line (2026-09-26): criteria, written before building

**First, what the firmware says** -- read from its source
(`waveshareteam/ugv_base_general`, `General_Driver`, GPL-3.0), because a fake
board can only encode what we believe, and three of the beliefs in
`HARDWARE-BOM.md` 4.2 were unverified:

* **The heartbeat DOES stop the motors -- now verified, not believed.**
  `heartBeatCtrl()` calls `setGoalSpeed(0, 0)` once `HEART_BEAT_DELAY`
  (3000 ms default, set by `T=136`) passes with no `T=1`/`11`/`13`. In
  `mainType` 3 that re-enables the PID loop with zero targets; in `mainType`
  1 and 2 it writes zero PWM directly. Either way the wheels stop.
* **`T=1` is NOT metres per second in the mode 4.2's example selects.** In
  `mainType` 1 and 2 (`{"T":900,"main":2}`) `setGoalSpeed()` is OPEN LOOP:
  `PWM = L x 512 x spd_rate`, a fraction of full power. Closed-loop speed
  control exists only in `mainType` 3, whose wheel constants are hard-coded
  for another robot (0.0523 m wheels, 1092 pulses/rev, 0.141 m track).
  Velocity control on this chassis therefore needs a firmware change (mode 3
  with our constants) or a host-side PID; this plan assumes the FIRMWARE
  change, because the board is where a 1 kHz encoder loop belongs.
* **The `1001` frame carries wheel SPEEDS, not positions**:
  `{"T":1001,"L","R","r","p","y","temp","v"[,"pan","tilt"]}`, `L`/`R` in m/s
  computed with the configured mode's wheel constants. No encoder counts.
  So odometry must integrate speeds on the host -- lossier than counting
  ticks -- unless the firmware change also reports counts. Recorded as a
  hardware-day item; R7's fake reports exactly what the real board reports.

**The design decision: the serial port belongs to `robot/hardware_robot.py`,
and ROS keeps the HTTP plugin.** The table says R9 swaps `picar_sim_hardware`
for a `picar_hardware` that talks to the board directly. That would take
`robot/safety.py` out of the nav path, and 3.15 decided the collars run in
SERIES with `safety.py` as the last word on every path. It would also give
the board two masters, since one serial port cannot be shared. So the seam
stays where CLAUDE.md section 2 put it: **`RobotInterface`**.
`robot/hardware_robot.py` is a backend like `MockRobot` -- `set_wheel_velocity`
becomes `T=1`, `get_wheel_state` integrates `1001` -- picked by `mode: hardware`
in `robot/factory.py`, and the ROS container talks HTTP to the robot server
exactly as in the sim. Hardware day becomes the config change the project
has always promised, and `picar_hardware` is not written.

**What R7 builds:** `sim/fake_esp32.py` -- a pseudo-terminal speaking the
real board's newline-delimited JSON as the firmware source defines it, with
the heartbeat, open-loop and PID modes, and `1001` frames, driving a
`MockRobot` body; and `robot/hardware_robot.py` against it.

**Acceptance criteria:**

1. **The fake is the firmware.** For each implemented command (`T=1`, `11`,
   `13`, `130`, `131`, `136`) the fake's behaviour matches the firmware source
   line for line, pinned by tests that cite the source lines.
2. **`HardwareRobot` is a `RobotInterface` backend** and passes
   `tests/test_robot_contract.py` against the fake, as the other backends do.
3. **Wheels over the wire.** `set_wheel_velocity()` for 1 s moves the fake's
   body within 5% of the commanded distance, and `get_wheel_state()`'s
   integrated positions agree with the body's true wheel angles within 2%.
4. **The heartbeat drill.** Sever the host link mid-motion (stop writing):
   the fake's motors stop within `HEART_BEAT_DELAY` + one loop period, and
   the robot server's watchdog is not what stopped them.
5. **Through the whole stack.** `mode: hardware` with the fake on a pty,
   `drive: ros`: the R4 verbs land within R4's tolerances.

**Measured 2026-09-26 -- all five met** (`tests/test_fake_esp32.py`, 12 tests;
`tests/test_robot_contract.py`, 22 more for the new backend; and R4's live
suite re-run on the hardware code path):

1. Met. Nine tests, each citing the firmware function it mirrors: T=1 as m/s
   in mode 3 with the +/-2.0 guard; T=1 as PWM (`x 512`, clamped) in mode 2;
   T=13's `X -/+ Z x TRACK_WIDTH / 2`; T=11 turning the PID off; T=130's one
   frame and its exact keys; T=131's frame on every loop; the heartbeat in
   BOTH modes; the motor's no-load ceiling.
2. Met. `HardwareRobot` over the pty passes all 22 contract tests. The first
   run caught a real defect in it: path length summed each WHEEL's travel,
   so a pivot counted as ground covered -- it is the body's travel now.
3. Met. A 3 rad/s command moves the body within 5% of 3 rad/s x the
   commanded interval, and the positions integrated from `1001` speeds agree
   with the body's true wheel angle within 2%.
4. Met. The host set the board's heartbeat to 1.5 s (`T=136`, above the robot
   server's 1 s watchdog so the server normally acts first) and went silent
   mid-motion; the BOARD stopped the wheels 1.5 s later, within one loop.
5. Met. The robot server in `mode: hardware` (`ROBOT_MODE=hardware
   SIM_MOTOR_BOARD=fake`), `drive: ros`: R4's live suite -- verbs within
   tolerance, a D-pad tap preempting a mission, the wall at >= 19.4 cm, both
   silence tests, the scan -- passes over the serial line. (The scan test
   first read 2 Hz because it followed the kill-the-container test with a
   fixed 10 s wait; that test now waits for scans to flow again.)

**Two host-side defects the pty found that a mock would not:** on macOS,
closing a pty while another thread is blocked reading it hangs the close, so
both readers poll with `select()` and `close()` joins them; and the board
reports ~100 frames a second, which the reader must keep up with or the
kernel buffer fills and the board's writes block.

**What this settles for hardware day:** `picar_hardware` is NOT written --
R9's swap is `mode: sim` -> `mode: hardware` plus `ROBOT_SERIAL`. What it
leaves open, and cannot settle without the board: flashing the firmware with
`mainType` 3 and THIS chassis' constants (or a host-side PID), and whether to
add encoder counts to the `1001` frame.

### 3.17 The wall's costs, measured (2026-09-27)

The wall around ROS (`tests/test_ros_containment.py`) has three costs, named
on 2026-09-27: things that must exist on BOTH sides of it, a bridge that can
grow into a second ROS, and ROS tools that cannot see the brain. The user
asked for each to be made visible. Plus one question: does HTTP really hold
the 20 Hz control loop?

**1. Linters -- `tests/test_wall_linters.py` (static, no ROS).**

* *Things that exist twice.* A registry of the ten concepts defined on both
  sides (wheel radius, wheel separation, the driver priority order, the
  20 Hz control rate, the silence timeouts, lidar range, the linear speed
  ceiling, the chassis footprint, the collision veto, the occupancy
  thresholds), each with a check that its copies AGREE -- drift between two
  copies is what a duplicate actually costs. An UNLISTED duplicate is caught
  where it can be: a non-round number that is a literal in ROS config AND a
  Python constant is almost always a hand-copied physical constant. Budget
  `MAX_DUPLICATES = 10`, raised only in a commit that says why.
* *A thin copy of ROS.* `MAX_BRIDGE_ROUTES = 13` (today's count); NO
  generic routes (a route taking a topic/service/node/param NAME from the
  caller re-exports ROS, which is the definition of a thin copy); every
  route has a consumer outside the container; every route is in the
  bridge's docstring.
* Each rule was confirmed red against a mutation before being trusted:
  radius drift, nav outranking a person, timeouts summing past 0.5 s, rate
  drift, lidar drift, footprint drift, an unlisted copied constant, a
  generic `/topic` route (four tests red at once), and a second copy of the
  occupancy thresholds.

**2. ROS tools can see the brain -- `picar_bridge/brain_view.py`.** The
bridge POLLS the brain's own `GET /mission/status` at 2 Hz and publishes
`/brain/status` (std_msgs/String, the JSON verbatim -- so a bag holds the
brain's record on the same clock as /scan and /tf), `/diagnostics` (one
"brain: mission" entry: OK / WARN for blocked, preempted, max_steps / ERROR
for failed / STALE when unreachable) and `/brain/markers` (a caption over
the robot and an arrow at the target's bearing, in base_footprint). The
brain is unchanged and still imports nothing of ROS: a window in the wall,
not a hole. `BRAIN_URL=""` turns it off. And **`foxglove_bridge` is in the
image, READ-ONLY** (only the `connectionGraph` capability, published on
127.0.0.1:8765): its defaults let a client publish, call services and set
parameters, which would be a second door to the wheels past
`robot/server.py`'s arbitration. `tests/test_brain_view.py` (19, offline)
and `tests/test_brain_view_live.py` (5, live -- the topics inside the container
must say what the brain's own status says, and the capability list is
checked). A RUNNING mission was read off the topics once, by hand; the test
itself starts none, because its first version did and the mission left the
robot where `tests/test_ros_chain_live.py` did not expect it.

**3. HTTP at 20 Hz -- measured.** Criteria written before the pinned run:
the plugin really runs at 19-21 Hz; zero failed requests; no control cycle
longer than 125 ms (half of `cmd_vel_timeout`, the silence that would stop
the wheels). From inside the container, one kept-alive connection (as
libcurl uses), a GET + POST-sized request per cycle, 30 s per rate:

| target | robot server, idle: p50 / p99 / max per request | cycles over budget | bare FastAPI, same hop: p50 / p99 / max |
|---|---|---|---|
| 20 Hz | 1.6 / 20.3 / 31.8 ms | 0.0% | 1.4 / 4.3 / 7.4 ms |
| 50 Hz | 1.7 / 17.7 / 39.8 ms | 1.9% | 1.2 / 3.3 / 6.6 ms |
| 100 Hz | 1.3 / 24.6 / 39.3 ms | 8.7% | 1.0 / 2.7 / 25.7 ms |
| 200 Hz | 1.0 / 21.4 / 39.8 ms | 5.4% | 0.8 / 1.6 / 4.5 ms |

With a mission running, 20 Hz: p99 33.8 ms, max 63.8 ms, 1.5% of cycles
late, none near 125 ms. The plugin posts exactly 200 per 10 s. **So HTTP is
not the limit** -- a bare app over the same Docker hop holds 200 Hz with a
p99 of 1.6-4 ms. The robot server's tail is the SIMULATOR sharing its
Python process: a 360-beam scan is ~13 ms of ray casting, a camera frame
~35 ms, both holding the GIL. The car replaces both with drivers, so its
profile will differ and **must be re-measured on the Jetson** (the test is
the instrument). Pinned: `tests/test_http_rate_live.py`.

When the loop would need more than 20 Hz: faster driving (at 0.6 m/s a
50 ms period is 3 cm of travel between commands), tighter trajectory
tracking in nav2's controller, or fusing an IMU (typically 50-200 Hz,
usually done INSIDE ROS by robot_localization, so it would not cross the
wall). Wheel PID never crosses it: the ESP32 runs it at ~100 Hz (3.16).

**Found on the way -- test gaps, now closed.** An audit of the ROS tests
found: tap-to-goal's house-to-ROS conversion and the `/world/goal` and
`/world/error` routes had NO offline tests (only the demo and live suite) --
`tests/test_ros_goals.py`, 26 tests, confirmed red against a dropped y-flip
(12 fail) and a dropped session check; the bridge's scan reorder and
quaternion maths were live-only -- now `picar_bridge/convert.py`, pure
Python, `tests/test_bridge_convert.py` (16, red against a dropped negation);
and **`tests/test_slam_live.py` drove its starter-house lap inside whatever
house the server was in**, because live tests read `SIM_MAP` from their OWN
environment -- the robot server now reports `sim_map` on `/health` and the
SLAM and nav live suites ask it. Still without unit tests of its own:
`picar_sim_hardware` (C++), exercised only through the live chain suite.

**A SAFETY ESCAPE, found by the audit and NOT fixed (2026-09-27). FIXED the same day -- see 3.18.** In the
furnished home (`SIM_MAP=home_first_floor`), a standing forward twist at
compass 330 from about (8.3, 9.8) m, among the dining furniture, started at
42 cm of path clearance and ended at **3.0 cm** -- `robot/safety.py`'s
forward clamp (bar 20 cm) did not stop it, and neither did
`collision_monitor`. Replayed offline at that pose: the path zones read 4.5 /
7.5 / 10.5 cm on the left-front and the nearest scan beams sit at -74 to -79
degrees (left), 18 cm from the centre. Two mechanisms are plausible and NEITHER
IS ESTABLISHED: (a) the path cone is +/-15.4 degrees (sized for one 30 cm
move), so inside ~30 cm it is NARROWER than the 19.8 cm-wide chassis and an
obstacle at the front corner is outside it -- M3 named exactly this
"off-centre approach" and deferred it to M10, and the axis-aligned starter
house could never show it; (b) the depth grid's one ray per 7.5-degree zone,
marching in 1.5 cm steps, can pass a diagonal corner between two solid
cells. The sim's own collision is also a single centre ray, so the chassis
can overlap furniture without being stopped -- the sim is not honest about
this either. Of twelve headings tried, one escaped. **The repair is a design
decision** (a footprint-rectangle check against the 360-degree scan, as
`collision_monitor` does, in `safety.py`; and a footprint-aware sim
collision), so it is written up here for the user rather than built.

**A SECOND safety finding, pre-existing: R4's wall stop is flaky. FIXED in 3.18, and the suspected cause below was WRONG -- it was a panned camera.** R4 and
R7 recorded "a standing forward twist into a wall stops at >= 19.4 cm" as
met. Re-run five times from a fresh start on the starter house, it reads
**18.0 cm in 3 of 5** -- and **the same 3 of 5 on an image built from the
previous commit**, so it predates this work. 18.0 is one 1.5 cm ray-march
quantum below 19.5. Suspected, NOT established: `robot/server.py`'s wheel
loop vets the command against current clearance, then integrates the ACTUAL
elapsed `dt`; a period stretched by the event loop stalling (the simulator's
ray casting holds the GIL -- and the lidar's reach went from 4.2 m to 12 m
on 2026-09-26, making each scan more expensive) moves the robot further than
was vetted. The general lesson carries to the car: a vet must cover the
travel until the NEXT vet, `v x (period + latency)`, not the travel of an
ideal period. Pinned as a non-strict xfail with this reason rather than left
flaky-red. And `tests/test_ros_chain_live.py` FAILED (twelve KeyErrors on 401
bodies) instead of skipping when run without the secret -- `/health` is open,
so its fixture never noticed; it now probes an authenticated route. Found alongside: **a clean checkout could not build the ROS
image** -- `picar_description`'s CMakeLists installed an empty, untracked
`config/` directory; fixed.

### 3.18 The two safety findings of 3.17, fixed (2026-09-27): criteria, written before building

Asked for by the user: "fix the two safety findings ... starting with the
oblique-approach escape. Write the metrics and thresholds first, confirm the
tests fail against the current code, then build the fix."

**Part 1 -- the oblique approach.** Reproduced offline before anything was
written, which is what fixed the metric: from (8.3, 9.8) m at compass 330 in
the furnished home, a standing 0.1 m/s twist drives the chassis INTO a piece
of furniture -- geometric overlap for ~3 s -- while `path_clearance()` reads
40-80 cm the whole way. The obstacle is off the front corner, outside the
+/-15.4 degree cone. So the fault is mechanism (a) of 3.17, and "path
clearance" is the wrong instrument to judge it by: it is the thing that was
fooled. **Every criterion below is measured on GROUND TRUTH** -- the URDF's
chassis rectangle (0.228 x 0.198 m, centred on `base_link`) against the
house's occupied cells (walls and solid objects), computed by the test from
the layout, never from any sensor the fix reads. Two truth quantities:

* **travel-to-contact `T`** -- how far the chassis could still translate
  along its heading (or astern, for reverse) before touching anything;
* **true gap `G`** -- the Euclidean distance between the chassis outline and
  the nearest occupied cell (0 when touching or overlapping).

**The rule to build** (a design, recorded so the criteria can be read
against it): a forward (reverse) command is clamped when any lidar return
lies in the chassis' swept corridor -- laterally within half the chassis
width plus a 3 cm side margin, and ahead of the front edge (astern of the
rear edge) by less than `min_distance_cm`. It runs in SERIES with the
existing cone, whichever reads less wins; rotation is still never clamped.
Things beside the chassis do not stop forward motion (the frozen-at-a-jamb
failure of R6's stop polygon). And the sim's own collision becomes
footprint-aware, so the simulator stops lying about contact.

**The sweep.** Three houses (starter, scaled, furnished home). Per house,
seeded random starts on free floor at sub-cell positions, rejected if the
chassis already overlaps anything or if nothing is within 40 cm (a start
with nothing in reach tests nothing), x 24 headings (every 15 degrees), forward
AND reverse. A standing 0.1 m/s command re-sent every period, run through
exactly the wheel loop's two calls (`vet_wheel_velocity()`, then
`advance(0.05)`), for 3 s.

**Criteria:**

1. **The stopping distance holds for what is actually in the way.** After
   every period in which the robot translated, `T >= 18.0 cm` -- i.e.
   `min_distance_cm` (20) less one period's travel (0.5 cm) less one
   ray-march quantum (1.5 cm, the most a sim beam can over-read). In every
   run, every house, both directions.
2. **No contact.** In every run, `G` never falls below `min(G at start,
   1.0 cm)`. The rule's 3 cm side margin less the same 1.5 cm quantum leaves
   1.5 cm; 1.0 is the bar.
3. **Progress -- the fix must not freeze the robot (section 7 rule 4).** Of
   the runs whose `T` at start is >= 60 cm (at least 20 cm of free travel
   after a 40 cm stopping band), >= 95% must translate >= 25 cm in the 3 s
   (30 cm is the full run). And every existing pinned mission test (R1,
   R1b, R1c, arrival, R2b's 19.5 cm wall stop) still passes unchanged.
4. **The sim tells the truth about contact.** With the clamp DISABLED, no
   standing command in the sweep drives the chassis INTO anything: the
   rectangle's penetration into any occupied cell stays <= 0.5 cm in every
   run (a pivot at a cell centre legitimately grazes a 30 cm corridor's wall
   by 0.1 cm -- the rectangle's corner radius is 15.1 cm against 15). Today
   the centre-ray collision lets the chassis pass into furniture.
5. **The real path.** One live run through `robot/server.py` (`POST
   /wheels`, its 20 Hz wheel loop on wall-clock time) from the reproduced
   pose ends with `T >= 18.0 cm` and no contact.

Each is to be confirmed RED against today's code before the fix is written.

**Part 2 -- the flaky wall stop. The suspected cause was WRONG, and the
data says what the real one is.** 3.17 suspected a stalled wheel loop
integrating an unvetted `dt`. A timing readout was added first (`/health`
`wheel_loop`: moving ticks, ticks later than two periods, the worst
interval) and 3.17's own procedure replayed -- SLAM on, the whole chain suite
after each fresh restart: **18.0 cm in 2 of 5 runs, with ZERO late ticks and
a worst interval of 63 ms.** The loop never stalled. From a fresh start
alone, 10 of 10 read 19.5 cm on this code and 10 of 10 on the previous
commit; the failure needs the tests that run before it. Traced reading by
reading: the robot moves ~0.5 cm a sample while the reading jumps 24.0 ->
18.0 in one step, and the live depth zones match an offline replay of the
same pose only with **the camera panned 90 degrees left** (exactly, all
eight zones). `test_a_dpad_tap_still_preempts_a_mission_under_drive_ros`
preempts a frontier mission mid-peek and the camera stays panned. The
depth grid -- and so the veto's path cone -- is cast along the CAMERA, so
the robot drove north guarded by a cone looking west. "18.0 cm" was the west
wall at a slant; the true travel-to-contact at that stop was 25.4 cm.
**Offline, on the code 3.17 found it in, the same drive with the camera
panned RIGHT goes until the sim's own collision stops it: 3.1 cm from the
wall, while the cone reads 30 cm to the east.** So the finding was real and
worse than recorded, and mis-attributed. Part 1's corridor check (body frame,
off the scan) already stops that drive at 19.6 cm -- but the cone still
votes along the camera, which stops a panned robot early for things beside
it, and a backend with no scan would have no guard at all.

**Criteria, written before building part 2:**

1. **The path guard does not turn with the camera.** Part 1's sweep, repeated
   with the camera panned fully left and fully right: part 1's criteria 1
   and 2 hold (`T >= 18.0 cm` after every move, no contact).
2. **A panned camera never stops the robot earlier than a centred one.** Same
   starts, same headings: travel with the camera panned >= travel with it
   centred - 0.5 cm, in every run.
3. **No scan and a camera facing away: forward is refused**, never waved
   through (a unit test on a scan-less robot; reason names the path as not
   observed).
4. **Live, in the suite's own order.** SLAM on, the whole chain suite after
   each of 5 fresh restarts: the wall test passes 5 of 5, judged by what the
   veto itself reads (`/depth` `veto_cm`, >= 19.4 cm) and by truth (travel-
   to-contact from `/world/truth`, >= 18.0 cm). The xfail comes off.

**Measured 2026-09-27 -- all nine criteria met** (`tests/test_footprint_safety.py`,
`tests/test_pan_safety.py`, `tests/test_ros_chain_live.py`; the full sweep is
`python -m tests.demo_footprint_sweep`). Every criterion was red first.

| | unfixed (`3ab3058`) | fixed |
|---|---|---|
| Part 1 sweep, 10 starts/house (1440 runs): runs under 18.0 cm of travel-to-contact | **25**, worst 0.0 cm | -- |
| Part 1 sweep, 40 starts/house (5760 runs) | -- | **0**, closest 19.5 cm |
| ... runs that touched something | **9** | **0** |
| ... progress: eligible runs covering 25 cm | 100% | **98.8%** (bar 95%) |
| Sim collision, clamp off: runs deeper than 0.5 cm into something | **214**/1440, deepest 8.0 cm | **0**/5760, deepest 0.28 cm |
| Camera panned +/-90: runs under 18.0 cm (144 each) | **42 / 43**, worst 0.0 cm | 0 / 0 |
| Camera panned: runs stopped earlier than centred | 24 / 24 (some after 0.0 cm) | 0 / 0 |
| Live, suite order, SLAM on: wall test | 18.0 cm in 2 of 5 (and 2 of 4 re-run) | **5 of 5**; veto 19.7-19.9 cm, truth 19.8-19.9 cm |

What was built:

* **`robot/safety.py`: the swept-corridor check** (`footprint_clearance()`,
  `forward_clearance()`, `reverse_clearance()`), off the 360-degree scan in
  the body frame, in series with the cone; the vets, `check_and_execute()`,
  the rule-based agent's scene and `/depth`'s `blocked` (plus `veto_cm` /
  `veto_source`) all read it. The chassis rectangle is registered in
  `tests/test_wall_linters.py`'s footprint entry (budget unchanged: the
  concept was already listed).
* **The cone selects zones by BODY bearing**: `get_depth_grid()` publishes
  `pan_deg`; a grid panned off the path has no opinion
  (`depth_grid_facing_away`), and with no scan either a FORWARD is refused
  (`path_not_observed`).
* **`sim/grid_world.py`: rectangle collision** on every translation, the
  centre ray kept for head-on, so every existing head-on stop is unchanged.
* **`RobotInterface.get_scan(max_range_m=)`**, a hint: the safety layer asks
  for 0.6 m, `MockRobot` casts only that far and casts it EXACTLY
  (`renderer.cast_ray_exact()`, a grid traversal). 13 ms -> 0.4 ms per
  scan, which matters at one scan per control period. It also measured 3.17's
  mechanism (b): **0.1-0.2% of marched beams slip past a diagonal corner**
  (up to 4 cells long). The exact scan cannot; the map and the picture keep
  the march their numbers are pinned to.
* **`/health` `wheel_loop`** -- moving ticks, late ticks, worst interval. It
  is what refuted the stall theory, and it stays so a live run can say so
  again.

Mutation-checked: corridor off (forward) -> criteria 1, 2, 5 red; corridor off
(reverse) -> 1 and 2 red on a reverse run; rectangle collision off -> 4 red;
pan ignored -> part 2's 2 and 3 red; footprint drift -> the linter red.

**Residue, recorded not fixed.** (1) The side margin (3 cm) and the chassis
numbers are the URDF's placeholders -- re-measure at R8, where the lidar's
+/-3 cm rating also belongs. (2) **Fixed in 3.19.** A PIVOT is still never clamped; a rectangle
sweeps 5.2 cm beyond its sides when it turns (corner radius 15.1 cm against a
9.9 cm half-width), so a robot parked beside furniture can swing a corner into
it -- the sim lets a pivot overlap rather than freezing it. (3) **Fixed in 3.20.** A mission
that ends mid-peek leaves the camera panned; nothing re-centres it. That no
longer matters to the veto, but the next policy starts looking sideways.
(4) The cone's own over-read (it measures from a 15 cm half-cell "bumper",
the chassis front is 11.4 cm) means a centred head-on stop leaves 24 cm true;
the corridor stops a panned robot at ~20. Both are >= the bar; they differ.

### 3.19 Pivots (2026-09-27): criteria, written before building

3.18's residue (2): rotation is never clamped, on the argument that a
differential chassis "pivots within its own footprint". A rectangle does not:
its corners sit 15.1 cm from the rotation centre, its sides 9.9 cm, so a pivot
sweeps a 5.2 cm ring beyond the sides. 3.18's sweep could not see this -- it
only started robots with the whole turning circle clear. **Decided (the
standard practice, the user's "go ahead"):** a turn is refused only if the
rotated chassis would come within a margin of something AND that direction
closes the gap; turning away is never refused, which is what keeps R2b's
"a robot facing a wall can pivot away".

**Metric, ground truth as in 3.18:** true gap `G` between the chassis
rectangle and every occupied cell, and -- per start and direction -- the
**free angle**: how far the chassis can turn that way before `G` would fall
below criterion 1's bar, `min(G at start, 1.0 cm)` (1-degree steps, capped
at 180).

*Corrected after the first measurement, and recorded as such:* the free angle
was first defined to CONTACT. That made criteria 1 and 2 contradict each
other: a corner grazing past furniture at 0.5 cm never touches it, so it
counted as free room while criterion 1 forbade entering it. The worst case
read 152 degrees "short" for exactly that reason. The definition now uses
criterion 1's own bar; no threshold moved.

**The sweep.** Three houses; seeded starts where the chassis is clear at its
start heading (`G >= 1 cm`) but something lies INSIDE the 15.1 cm turning
circle -- the only starts where a pivot can hit anything -- each at a
random heading (a heading is part of what makes such a start clear), x both
directions; a standing pivot at 1 rad/s for 3 s (~170 degrees) through
the wheel loop's two calls.

**Criteria:**

1. **No contact from turning.** In every run, `G` never falls below
   `min(G at start, 1.0 cm)`.
2. **Turning is not frozen.** Of the runs whose free angle is >= 30 degrees,
   >= 95% turn at least (free angle - 10 degrees), capped at the ~170 the
   run can reach.
3. **A robot facing a wall still pivots away** -- R2b's criterion 2
   (`tests/test_wheels_command.py`) passes unchanged; and from every start in
   the sweep with `G < 3 cm`, at least one direction turns >= 30 degrees
   when the truth says one can.
4. **The sim tells the truth about pivots.** With the clamp off, no pivot
   leaves the chassis more than 0.5 cm inside anything.
5. **Nothing else moves.** Every pinned mission test (R1, R1b, R1c, arrival)
   and 3.18's criteria still pass.

**Measured 2026-09-27 -- all five criteria met** (full suite 1409 passed; the live chain 3 of 3 from fresh restarts)
(`tests/test_pivot_safety.py`, 20 starts/house pinned; the figures here are
60 starts/house, 360 pivots):

| | before | after |
|---|---|---|
| pivots that came within 1.0 cm (or touched) | **120 / 120** (20 starts/house) | **0 / 360**, closest 1.02 cm |
| pivots with room that turned into it (bar 95%) | -- | **83 / 83**; shortfall median 0.9, worst 2.7 degrees |
| clamp off: pivots > 0.5 cm into something | **98 / 120**, deepest 3.4 cm | **0 / 360**, deepest 0.28 cm |

Built: `SafetyController.pivot_blocked()` / `pivot_scale()` -- every scan
return in the body frame, its distance to the rectangle now and after one
period's turn; a turn that closes to within `PIVOT_MARGIN_CM` (1.2) is SCALED
to the largest fraction that keeps the margin, never reversed and never
applied to a turn that opens the gap. `GridWorld.rotate()` stops where the
rectangle would first touch (a pose already in contact is exempt, as for
translation), and `MockRobot.step()` counts on the encoders only the turn
that happened. Mutation-checked: clamp off -> criterion 1 red; sim pivot
collision off -> criterion 4 red.

**The live chain found a real case the offline suite could not.**
`test_a_verb_through_ros_means_what_it_meant` turned 45 and 90 degrees right
after its FORWARD, which parks the chassis inside the starter house's 30 cm
doorway; there a pivot brings a corner to **0.47 cm** of the jamb (ground
truth), with 27 degrees free to the 1 cm bar. The veto stopped it at ~20
degrees -- correct, and R6's "30 cm doors cannot host this chassis" seen by
the safety layer for the first time. The test is about what a verb MEANS
through ROS, so its turns now come before the FORWARD, where there is room;
the reason is in the test.

Two things the data corrected on the way. The free-angle definition (above).
And **zeroing a turn is the wrong clamp**: it stopped robots a whole control
period (2.9 degrees at 1 rad/s) short of where they could safely turn, and
failed criterion 3 on three starts that had 31-32 degrees of room; scaling
the rate into the margin fixed it and took the median shortfall from 2.0 to
0.9 degrees.

### 3.20 A mission starts with the camera centred (2026-09-27): criteria, written before building

3.18's residue (3). A mission that ends mid-peek -- preempted, stopped,
failed -- leaves the camera panned, and nothing re-centres it. 3.18 made the
VETO immune to that; the next POLICY is not. Its first frame, depth grid and
scene are cast along the camera, so it starts deciding about a room 90
degrees off its heading. Chosen: centre the camera **at the start of a
mission**, on its first tick, before its first decision -- not at the end,
where a preempted mission no longer holds authority and its request would
be refused.

**Criteria:**

1. **The first decision sees a centred camera.** For each of pan -1, -0.5,
   +0.5, +1 left behind before start, and each sim policy that needs no paid
   call (frontier, and the vision loop over a stub `vision_fn`), the camera's
   pan at the policy's FIRST sensor read is 0. Measured through the real path
   (`MissionRunner` -> gated robot -> `MockRobot`).
2. **It costs nothing a mission counts.** The first tick still counts as one
   step, and a vision policy's `vision_fn` is called exactly as often as
   before.
3. **A centred start is unchanged.** From a centred camera the mission's
   action trace is identical, step for step, to the code before this change
   (starter house, frontier, the pinned 83-step run of
   `tests/demo_brain_over_http.py`), and every pinned mission test passes.
4. **A refused centring ends the mission the way any refused move does**
   (preempted -> `preempted`, halted -> no step), never a crash.
5. **Live.** With the camera left at -90 by the chain suite's preempted
   mission, a new mission through the brain's HTTP API: `/depth` reports
   `pan_deg` 0 after its first step.

**Measured 2026-09-27 -- all five met** (`tests/test_camera_centred_start.py`;
full suite 1429 passed). Red first: criterion 1 failed for every pan and
both policies (the first read saw the pan left behind), criterion 4 failed
(nothing tried to centre, so nothing could be refused). Criterion 3 is pinned
against the pre-change trace itself (`tests/data/frontier_trace_centred.json`,
83 steps, `found`), not against a re-derivation of it. Built: `MissionRunner`
calls `look_center()` through its own gate on the first tick, before the
policy's first decision, uncounted.

**Criterion 5 was mis-specified, and the live run showed it.** "`pan_deg` 0
after the first step" cannot tell the fix from its absence: the frontier
policy's first action is itself a peek (`LOOK_LEFT`), so the camera is at
-90 after step 1 either way. Measured instead: `/depth` sampled
continuously from mission start through the first step, three fresh runs
with the camera left at -90 by the chain suite's preempted mission.

| | pan sequence during the first step |
|---|---|
| before (stash of this change) | -90, +90 -- the first decision read at -90 |
| after, 3 runs | -90, **0**, -90, +90 -- centred, then the policy's own peek |

### 3.21 The chassis becomes the Waveshare UGV Rover (2026-09-27): criteria, written before measuring

> **Status 2026-09-30: the Rover is BOUGHT** (Amazon, ordered 09-30, expected
> Oct 19 - Nov 11), after a Hiwonder ROSOrin was ordered 09-29 and cancelled
> (`JETSON-BOM.md` section 9). Waveshare has confirmed the kit's **ROS
> Driver** board (closed loop, encoder odometry to the host) and **660
> pulses per revolution**: the **1650** below was the open-loop firmware's
> stale constant, corrected to 660 in 3.25.

**Decided by the user** ("let's assume that I am buying the car you
recommended"; not yet ordered -- Waveshare has been asked whether the kit's
3S UPS can carry an Orin Nano Super at 25 W). The kit is the **UGV Rover PT
Jetson Orin ROS2 Kit Acce**: 6 wheels, 4 driven, skid steer, the same ESP32
General Driver R7 already fakes, a D500 (LDROBOT STL-19P) lidar, an OAK-D
Lite and an ICM20948 IMU. Its numbers, and where each comes from:

| | was (2WD Yahboom build) | UGV Rover | source |
|---|---|---|---|
| wheel radius | 0.0325 m | **0.040 m** | 80 mm tyres, Waveshare product page `[V]`; firmware `WHEEL_D 0.0800` |
| encoder pulses / rev | 1760 | **1650** | firmware `mm_settings()` mainType 2 = "UGV Rover" `[V]` |
| track width | 0.172 m, PLACEHOLDER | **0.172 m** | the same line `[V]` -- the placeholder was this robot's number all along. Skid steer's *effective* track is larger; that is `wheel_separation_multiplier`, measured on the car |
| outer footprint | 0.228 x 0.198 m | **0.253 x 0.231 m** | product page, 253 x 231 mm `[V]`; rotation centre assumed at its middle `[I]` |
| lidar | RPLidar C1, 12 m | D500, 12 m, 10 Hz | Waveshare wiki `[V]`; the sim's scan is unchanged |

Read from the firmware source on the way, and not built here: **the stock
firmware runs mainType 2 OPEN LOOP** (`usePIDCompute = false`, and
`speedGetA = pwm` -- the reported "speed" is the PWM, not the encoder). R7's
fake board already models the closed-loop firmware change 3.16 called for;
it now carries mainType 2's constants.

**The question this answers before anything is ordered: does this body fit
the house, and does the safety work of 3.18-3.19 hold for it?** The corner
radius goes from 15.1 cm to 17.1 cm, so a pivot in a 30 cm corridor --
every corridor of the starter house -- is now physically impossible. That is
expected, and is the starter house being too small, not the chassis
failing; it is recorded, not engineered around.

**Criteria:**

1. **One chassis.** Every copy of wheel radius, pulses, track and footprint
   (sim, `robot/hardware_robot.py`, `sim/fake_esp32.py`, xacro,
   `controllers.yaml`, `nav2.yaml`, `robot/safety.py`, the truth copies in
   the sweeps) carries the new number: `tests/test_wall_linters.py` and
   `tests/test_urdf.py` pass.
2. **Fit -- the purchase question.** A configuration-space check on ground
   truth (the rectangle against the layout's occupied cells, every 5 degrees
   of heading, at 2.5 cm; a path-connected free C-space is reachable by a
   differential drive): in the furnished home, **every room the old
   footprint reaches, the new one reaches**, both physically (no margin)
   and with the safety layer's 3 cm. Any room lost is named, with the
   passage that loses it.
3. **Safety holds on the new body, bars unchanged.** 3.18 part 1 criteria
   1, 2 and 4, part 2 criteria 1 and 2, and 3.19 criteria 1, 2 and 4, at the
   pinned sample sizes, in every house where the chassis has somewhere to be.
4. **Progress, bars unchanged** (section 7 rule 4): 3.18's >= 95% of
   eligible runs cover 25 cm; 3.19's >= 95% turn into their room.
5. **Missions.** Every pinned mission test (R1, R1b, R1c, arrival, R2b)
   passes unchanged, OR fails because the house it runs in cannot hold this
   body -- shown by criterion 2's check on that house, never by loosening a
   threshold.
6. **nav2 in the furnished home, live:** the room tour reaches at least as
   many rooms as the old footprint did (8 of 9).

**Measured 2026-09-27 -- criteria 1, 2, 3, 4 and 6 met; criterion 5 FAILED,
and why is the finding.** *(Cleared 2026-09-28 by 3.22, guarded verbs: the 17 now pass, with no
threshold moved.)*

| | old chassis (228 x 198) | UGV Rover (253 x 231) |
|---|---|---|
| 2. rooms reached, furnished home, 0 cm / 3 cm margin | 13 / 13 of 13 | **13 / 13 of 13** |
| 2. ... scaled house; starter house | 5 of 5; 3 of 3 | **5 of 5; 3 of 3** |
| 2. floor where the body can turn right round, family room / dining / pantry, m^2 | 21.1 / 4.9 / 0.59 | 20.3 / 4.2 / 0.50 |
| 3-4. pinned safety sweeps (3.18 parts 1-2, 3.19) | pass | **pass**, bars unchanged |
| 6. nav2 home tour, same code and image build, one run each | 8 of 9 (dining: still active at 120 s) | **8 of 9** (dining: aborted at 62 s) |
| 6. ... closest approach, centre-to-surface less the half-width | 5.3 cm | 4.4 cm |
| 6. ... `robot/safety.py` refusals during the tour | 17 | **141** |
| 5. the 17 mission tests below | pass | **fail** |

The fit check (`python -m tests.chassis_fit`) was shown to find a misfit
before its "fits" was trusted: the furnished home first loses rooms (den,
half bath, pantry, laundry) at an 80 x 80 cm body, and the starter house
loses two of its three rooms at the Rover plus 3.5 cm a side -- its 30 cm
doors.

**Criterion 5: 17 failures** (the rest of the offline suite, 1265 tests,
passes; the same 17 pass on the old chassis). Every one is a rule-based or
semantic mission on the starter house or a small test layout. The
mechanism, read from the log: a FORWARD verb is vetted once, at its start
(clearance >= 20 cm), then moves a whole cell, and in the sim only the
half-cell head-on cap stops it -- the robot's CENTRE ends 15 cm from the
wall. The old chassis' 15.1 cm corner radius could still pivot there (14.9
after the 0.2 cm skin); the Rover's 17.1 cm cannot, so 3.19's pivot guard
refuses every turn and the agent retries "BLOCKED turning ... 0.0 of 5.0
deg" until it runs out of steps. The house is not too small (criterion 2
says so): the verb parks the car where it cannot turn. Not fixed, and no
threshold loosened -- it is finding 1.

**Two findings for the user, NOT built:**

1. **A direct-mode FORWARD verb is vetted once and then drives blind.**
   `SafetyController.check_and_execute()` checks for >= 20 cm, the verb
   then covers 30 cm, and `HardwareRobot._run()` never re-checks while
   moving. On the real car under the default `drive: direct` that is up to
   **10 cm of travel past a wall**, on either chassis. The sim hides it
   behind its half-cell cap, which has no counterpart on hardware. `drive:
   ros` does not have the gap: its verbs are twists vetted every 50 ms. The
   repair that matches the rest of the design is for direct-mode verbs to go
   through the same per-period `vet_wheel_velocity()` as the wheel loop. It
   changes what a verb does near a wall (it stops short of a whole cell),
   which is why it is a decision -- and it would also clear criterion 5.
2. **The Rover makes the safety layer refuse far more often** -- 141
   against 17 on the same tour. The outcome is identical, so it is not a
   failure, but the side margin (3 cm) and the path cone
   (`CHASSIS_WIDTH_CM`, still the PiCar-X's 16.5 cm and now 6.6 cm narrower
   than the body) are hardware-day calibrations that now matter more.

Not modelled, and still true: skid steer slips on every turn and the sim has
no slip; the Rover's wheelbase and which pair of motors carries the wired
encoders are unpublished (both are in the email to Waveshare). And from the
firmware source: stock mainType 2 is open loop, so closed-loop speed control
needs 3.16's firmware change.

### 3.22 Guarded verbs (2026-09-28): criteria, written before building

3.21's finding 1, decided by the user ("go with the recommendations" on
`PLAN-guarded-verbs.md` section 6, which holds the full plan). Under `drive:
direct` a verb is vetted once and then drives blind; under `drive: ros` it is
re-vetted every 50 ms. **Decided:** (1) a move cut short is EXECUTED and says
so (`moved` against `requested`); one that covers under 1 cm is REFUSED, so
R1b's stuck detector still ends a pinned robot `blocked`; (2) turn verbs are
guarded too; (3) the runner lives in `SafetyController`, so in-process agents
and the server behave alike.

**The rule to build** (a design, recorded so the criteria can be read
against it): every direct-mode FORWARD, REVERSE, LEFT and RIGHT runs as a
standing wheel command in 50 ms periods, closed on the encoders. Each period
reads the same clearances `vet_wheel_velocity()` reads -- `forward_clearance()`
/ `reverse_clearance()` for a translation, `pivot_scale()` for a turn -- and
**looks ahead**: a translation may cover at most (clearance -
`min_distance_cm`) this period, so it stops AT the line rather than one
period past it (at a verb's 0.6 m/s one period is 3 cm, which would break
3.18's 18 cm bar). A `stop()` from anyone -- `/stop`, the watchdog -- ends the
verb within one period and is never overwritten. `drive: ros` is untouched:
its wrapper is already guarded. The backend supplies what a verb MEANS (its
wheel speeds and its distance or angle), so a verb still covers exactly what
it covered before whenever the way is clear.

**Criteria** (bars reused from 3.18 and 3.19; 1-3 confirmed RED first):

1. **Moves stop safely.** From 3.18's seeded starts in all three houses, x 24
   headings, FORWARD (and REVERSE) verbs at speed 50 and 100, repeated until
   one is refused or six have run, judged on ground truth sampled every
   period: after every verb, travel-to-contact `T >= 18.0 cm`; gap `G` never
   below `min(G at start, 1.0 cm)`.
2. **Turns stop safely.** From 3.19's pivot starts, LEFT and RIGHT verbs of
   45 and 90 degrees, same sampling: `G` never below `min(G at start,
   1.0 cm)`.
3. **A stop ends a verb.** A FORWARD on the hardware backend over the fake
   ESP32 (`sim/fake_esp32.py`), with `stop()` called mid-verb: the board's
   commanded wheel speeds are zero within 100 ms and stay zero until the
   verb returns. And a verb that outlasts the watchdog timeout is still
   carried out -- the watchdog is for a silent brain, not a busy one.
4. **Verbs still mean what they meant.** Of the FORWARD verbs whose `T` at
   start is >= 60 cm, >= 95% cover the full 30 cm (within 1 mm); of the
   turns with >= 10 degrees more room than asked for, >= 95% turn the full
   angle (within 0.5 degrees).
5. **Missions.** The pinned R1, R1b, R1c, arrival and R2b tests pass
   unchanged, and 3.21's 17 failures pass -- no threshold loosened.
6. **Live.** One mission through the brain's HTTP API on `drive: direct`,
   and one FORWARD verb through the robot server over the fake ESP32 serial
   line ending >= 18.0 cm from the wall by `/world/truth`.
7. **`drive: ros` unchanged:** `tests/test_ros_drive.py` passes, and the
   live chain suite against a fresh container.

**Measured 2026-09-28 -- all seven criteria met**, 1-3 red first. The full
sweep is `python -m tests.demo_verb_sweep` (6 starts/house x 24 headings);
`tests/test_guarded_verbs.py` pins a seeded sample.

| | unguarded (`1d0b7fc`) | guarded |
|---|---|---|
| 1. straight runs ending a verb under 18.0 cm of travel-to-contact | **1024 / 1728**, worst 0.0 cm | **0 / 1728** (4204 verbs), closest 19.7 cm |
| 1. ... runs that touched something | **312** | **0** |
| 1b. cut-short verbs whose veto reads under 19.4 cm at the stop | -- (nothing is cut short) | **0 / 1226** |
| 2. turns coming within 1.0 cm | **135 / 288**, worst 0.00 cm | **0 / 288**, closest 1.03 cm |
| 3. board driven again > 100 ms after `stop()` mid-verb | yes (re-sent every 50 ms) | **no** |
| 3. a `/stop` over HTTP during a 2 s hardware verb | not received until the verb ended (0.606 m moved) | **ends it** |
| 3. a verb outlasting the 1 s watchdog | completes | **completes** (0.60 m) |
| 4a. clear moves (T >= 60 cm) covering the whole cell | 100% | **98.8%** (bar 95%) |
| 4b. clear turns turning the whole angle | 100% | **100%** |
| 5. offline suite | 17 fail (3.21) | **1291 pass**; UI suites 126 pass |
| 6. live: frontier mission through the brain's HTTP API, `drive: direct` | -- | **found**, 61 steps, 0 refusals |
| 6. live: FORWARD at a wall over the fake ESP32 serial line | -- | stopped at **22.6 cm** by truth (from 32.2), then refused |
| 7. live ROS chain suite, fresh container | -- | **13 / 13** |

Mutation-checked: look-ahead off -> 1b red (veto reads down to 17.2 cm; 1
stays green, see below); the 5-degree turn step off -> 2 red (12 of 96);
watchdog exemption off -> 3's watchdog test red (the verb is cut at
0.305 m); the wheel loop blocking on the lock -> 3's HTTP stop red.

**What was built.** `robot.interface.carry_out_verb()` is the mechanics --
periods, closed on the encoders, ended by any `stop()` (a `stop_count` on
each backend) -- and a backend's own unguarded verbs run through it too, so a
guarded verb and a raw one are the same motion to the last bit when nothing
is in the way. `RobotInterface.verb_plan()` / `verb_done()` let a backend say
what a verb means; the default is None ("run my own verb"), which is what
`drive: ros`, `RemoteRobot`, replay and teleop keep. `SafetyController.
run_verb()` supplies the limit: a translation looks ahead to (clearance -
`min_distance_cm`), a turn is checked in steps of at most 5 degrees at a
look-ahead of exactly that step. A move under 1 cm, or a turn under 0.5
degrees, is a refusal (decision 1).

**Four things found on the way, all fixed:**

1. **The robot server could not receive a `/stop` during a hardware verb.**
   `/action` held `motion_lock` for the whole verb and the 20 Hz wheel loop
   waited for that lock ON THE EVENT LOOP, freezing every route -- the
   watchdog included -- until the verb ended. The wheel loop now skips a
   tick instead of waiting. Only visible on the wall clock; the sim's verbs
   were instantaneous.
2. **A fast turn could pass a corner through furniture unchecked.** The
   sim's verbs turn at ~400 deg/s -- 20 degrees a period -- and a check that
   compares a chassis before and after a step cannot see what the corner
   crossed in between. Hence the 5-degree step.
3. **Criterion 1 cannot see the look-ahead.** The path cone measures from a
   half-cell "bumper" 2.35 cm ahead of the Rover's real front edge, so truth
   keeps ~2 cm in hand either way. 1b was added AFTER the mutation run, at
   3.18 part 2's existing veto bar (19.4 cm); 19.9 was tried first and the
   real code read 19.5 -- the readings' march quantum.
4. **A robot parked exactly on the line had no forward but was told it
   did.** The rule-based policy called a direction clear at `>=` its
   threshold, and a guarded verb now stops exactly there -- so it chose
   FORWARD 140 times running. Both `sensed_scene()` and the frontier
   policy's three-way look now require the threshold plus 1 cm (decision
   1's own number). This is what cleared 3.21's 17 failures.

Also: the watchdog now leaves a direct-mode verb in progress alone (it is
bounded, re-vetted every period, and `/stop` still ends it), and times its
silence from the verb's END; and two tests changed for reasons stated in
them -- the like-for-like comparand in `test_remote_robot.py` now goes
through a `SafetyController` as the server does, and R2b's reverse test
accepts the wheel loop's clamp a period later, because the first REVERSE
now stops at the line rather than a cell past it.

### 3.23 A nav2 goal is an autonomous driver (2026-09-27): criteria, written before building

**The gap** (found by the documentation review, `docs-review/REPORT.md` V4):
`POST /world/goal` called no `arbitrate()`, and nav2 drives on
`cmd_vel/nav` at twist_mux priority 50 -- the same as `cmd_vel/brain`. So a
goal sent during a brain mission was neither refused at the robot server
nor ordered inside ROS, which contradicts R2b's "one autonomous driver at a
time" and the claim in `twist_mux.yaml`'s own header. **Decided (the
user accepted the recommendation):** a goal is an AUTONOMOUS driver, the
same rank as the brain -- refused while another autonomous driver holds the
robot, and while a goal is pending or active, other autonomous drivers are
refused. A person still outranks both. The robot server learns nothing
about missions: "a goal is active" is the world's own state
(`get_goal()`), the same thing the twin draws.

**Criteria** (`tests/test_goal_arbitration.py`, in-process against the fake
bridge `tests/test_ros_goals.py` already uses):

1. **A goal is refused while the brain holds the robot.** After a brain
   `/action`, `POST /world/goal` answers `accepted: false`,
   `reason: "preempted"`, and the bridge receives NO goal.
2. **The brain is refused while a goal is pending or active.** A brain
   `/action` FORWARD answers `executed: false`, `reason: "preempted"`, the
   detail names the nav2 goal, and the robot does not move. Same for the
   `teleop` driver.
3. **A person is never refused because of a goal.** A `twin-dpad` (and an
   unnamed) `/action` passes during an active goal; `/stop` always passes.
4. **A goal that has ended blocks nothing.** With the goal `succeeded`,
   `aborted` or `canceled`, a brain `/action` passes.
5. **A lapsed claim does not block a goal.** Once the brain has been silent
   past `watchdog_timeout_s`, a goal is accepted.
6. **Nothing else changes.** `tests/test_ros_goals.py`,
   `tests/test_authority.py`, `tests/test_wheels_command.py` and
   `tests/test_ros_drive.py` pass unchanged.

Each is to be confirmed RED against today's code before the fix is written.

**Measured 2026-09-27 -- all six met.** Against the unfixed code criteria 1
and 2 were RED (3 tests: the goal reached the bridge while the brain held
the robot; `brain` and `teleop` FORWARDs executed during an active goal);
3-5 passed before and after, which is their job -- they guard against a
fix that blocks too much. Criterion 6: `test_ros_goals`, `test_authority`,
`test_wheels_command`, `test_ros_drive` and `test_server` pass (88 with the
new file). Full suite 1417 passed / 51 skipped / 1 failed --
`test_remote_robot.py::test_identical_action_sequence_over_a_real_socket`,
which uses no goal-capable world and passed 3 of 3 re-runs alone (a
real-socket timing test; other commits landed during the 8-minute run).

What was built, in `robot/server.py`: `arbitrate()` refuses any autonomous
driver other than `ros` while `goal_in_progress()` (the world's own
`get_goal()`, pending or active) answers; `POST /world/goal` arbitrates as
driver `ros` before anything reaches nav2 and answers
`{"accepted": false, "reason": "preempted", ...}`, which the twin's
tap-to-goal already shows as a toast. An unreadable bridge counts as no goal
(fail-open, argued in the docstring). Not changed: inside ROS, `cmd_vel/nav`
and `cmd_vel/brain` are still equal at 50 -- but the robot server now
admits only one of them at a time. `tests/test_goal_arbitration.py`.

### 3.24 `drive: ros` as the car's default (2026-09-28): criteria, written before building

Asked by the user: "Should we make drive: ROS the standard? Why have
drive:direct when ROS is the standard way to implement robotics?" **Proposed
and accepted ("go ahead"):** ROS becomes the default ON THE CAR (`mode:
hardware`) once the gates below are met; `direct` is kept, but narrowed to
two jobs -- (a) the simulator/test default (the ~1400-test offline suite and
the deployed twin run without Docker) and (b) the car's FALLBACK if the ROS
stack dies, since production robots keep safety and low-level motor control
outside ROS so a crashed ROS process cannot leave the wheels running (this
project's wall, `robot/safety.py` in every path, already embodies that). It
stops being a second, equal way to drive with its own semantics.

**Where the two paths differ today** (read from the code, not assumed --
an earlier answer to the user overstated it and was corrected): both re-vet
a verb every 50 ms (3.22's wording: under `drive: ros` a verb "is re-vetted
every 50 ms" by the wheel loop). They differ in HOW they stop:

* **Where.** Direct mode LOOKS AHEAD -- a period may cover at most
  (clearance - `min_distance_cm`) -- so it stops at the line. The ROS path
  clamps only once the reading is already under the line, so it can end up
  to one period past it. A ROS verb runs at up to 0.6 m/s
  (`robot/ros_drive.py`: 2 moves/s x 0.30 m at speed 100), 3 cm a period:
  plausibly under 3.18's 18 cm bar at full speed. **Unmeasured.**
* **What a blocked move reports.** Direct: under 1 cm moved is a REFUSAL,
  which R1b's stuck detector counts. ROS: the executor ends a stalled verb
  after `STALL_S` and reports it executed with what it covered.

**Gates -- all must hold before the car's default flips. G1 first: it
blocks the rest.**

* **G1 -- the chain is reliable.** The live chain suite
  (`tests/test_urdf.py` + `tests/test_ros_chain_live.py`, SLAM on, suite
  order -- the SCALED house since G1's first step, see below -- a fresh robot-server restart per run) passes **20
  consecutive runs** on this laptop. Today (2026-09-28, image rebuilt from
  `56949d5`): runs with a failure were 0/4 at `be2df51`, 3/8 at `88b510a`,
  and 3/7, 0/4, 5/8 at `56949d5` -- verbs closing at 3-23 cm of 30,
  `/odom` lagging truth, the scan republished at 1.5-4.5 Hz against 5. Two
  causes are already ruled out: dropped wheel-loop ticks (a skip counter read
  0 over 8 runs) and a slower simulator (scan, grid and vet cost the same
  before and after). **First step:** attribute it with 12 runs each of
  `be2df51` and the head, alternated run by run so machine drift cancels,
  plus the Docker VM's CPU per run. The fix, whatever it is, is judged by the
  20-run bar.
* **G2 -- the same safety, whichever path.** 3.18's and 3.19's ground-truth
  sweeps (`tests/footprint_sweep.py`), run through the ROS verb executor
  (in-process, against the fake chain `tests/test_ros_drive.py` already
  uses) at verb speeds 50 and 100: travel-to-contact >= 18.0 cm after every
  move, no contact, pivots within the 1.2 cm margin -- the SAME bars, not
  new ones. And a ROS verb that covers under 1 cm is reported as the same
  refusal direct mode reports, so a pinned robot ends `blocked` on both
  paths. **Progress half:** with the way clear, >= 95% of ROS verbs cover
  their full move / turn (within 3.13's 4.4 mm / 0.64 deg).
* **G3 -- the fallback is real.** With a mission running under `drive: ros`,
  kill the container: the wheels stop within the watchdog (as today);
  **the mission ends** -- it does not resume autonomously on the fallback
  path -- **decided by the user 2026-09-29 ("your recommendation"): only a
  PERSON may drive on the fallback**, so autonomy never continues on a path
  without nav2 and SLAM that nobody chose; a new mission starts only once
  ROS is back; and within **2 s** a D-pad FORWARD
  executes through `direct`, vetted by `robot/safety.py`. When the container
  returns, `drive` goes back to `ros` without a server restart.
* **G4 -- on the car's own computer.** On the Jetson (ordered, arriving Oct
  14-26): the image builds natively, and the live chain and nav suites pass
  **5 consecutive runs** against `SIM_MOTOR_BOARD=fake` (the real motor
  board's code over a pty, R7). Against the real chassis: when it exists.

**G1, first step done (2026-09-29): the flakiness is TWO problems, and both
causes are established.** 12 runs each of `be2df51` and the head, alternated,
a fresh container per run: 1/12 failed at `be2df51` (scan rate), 2/12 at the
head (verbs short); container CPU 44-56% on both -- a fair comparison. Then
each failure mode was traced to its cause, not guessed:

1. **Verbs closing short -- a test in a door the chassis cannot fit.** With
   the ROS executor's stall log raised (throwaway worktree), every short verb
   is "verb stalled at 0.19-0.26 of 0.300 -- ended" and no passing run has
   one. The verb test drives FORWARD into the starter house's 30 cm door; the
   UGV chassis (3.21) is 23.1 cm, so 3.45 cm a side, and 3.18's corridor
   margin is 3 cm a side (sized for the 19.8 cm 2WD build) -- 4.5 mm for
   heading and position error. Offline, deterministic: 0.5 deg of heading
   error stops the move at 21 cm, 0.7 deg at 6 cm, a 3 mm offset at 3 cm --
   with the true gap 13 cm, so the veto is conservative, not wrong. ROS
   turns land within ~0.6 deg (3.13), hence one run in a few. It started at
   the UGV merge because the wider chassis made the door tight. **Not a
   safety or ROS fault; the margin stays.** No direction in the starter
   house's start room has room for a 30 cm move (45-55 cm clear against the
   62.7 cm a move needs); the scaled house's has 105-300 cm.
2. **Scan rate under 5 Hz -- new connections through Docker Desktop.** The
   bridge's `/scan` polls (a new `urllib` connection each) fail with
   "Network is unreachable": a MASKED timeout (the IPv4 connect times out,
   Python falls back to an unroutable IPv6 address and reports that). From
   inside the container, 41 of 433 polls (9.5%) exceed 0.5 s; from the Mac,
   0 of 575, max 50 ms; from inside the container over ONE kept-open
   connection, 0 of 595, max 38 ms. The robot server is not slow; opening a
   connection through Docker Desktop's port-forwarding on macOS stalls about
   one time in ten. The C++ wheel plugin keeps its connection open, which is
   why the wheels never showed it. On the Jetson (native Linux) the proxy is
   not in the path -- but it makes the 20-run bar unmeetable here.
3. **Brain-view test** read the first `/diagnostics` message from any
   publisher; fixed (`a60619f`) to read until the bridge's own entry.

Ruled out on the way: dropped wheel-loop ticks (a counter read 0 in 8 runs)
and a slower simulator (scan, grid and vet cost the same before and after).
Container age raises the failure rate (one container kept across runs: up to
5 of 8) -- consistent with (2), not separately measured.

**Proposed fixes, awaiting the user:** (a) run the chain suite in the scaled
house, with the wall-stop test made house-aware (its ground truth read from
the house the server reports); the SLAM lap stays in the starter house.
(b) the bridge polls the robot server over one kept-open connection. Then
G1's 20-run bar is attempted.

**G1 MET (2026-09-29): 20 of 20 consecutive live runs**, both fixes in:
(b) the bridge polls over kept-open connections (`39b340b`; 600 scans in
60 s = 10.0 Hz, the scan test 20/20 in isolation where it failed 4/10, 0
poll failures), and (a) the chain suite runs in the scaled house -- its
fixture skips any other house -- with the wall test's ground truth read from
the house the server reports and its pass REQUIRING a recorded forward
clamp, since the scaled house's wall is ~1 m off and a timer could otherwise
end the drive first. One container for all 20 runs (the harder condition),
a fresh robot-server restart per run, container CPU 44-51%. Each run: 32
passed, 1 xfailed (R3's pan-bearing limit). The wall test also stopped
comparing the veto's START reading, which is None with the wall beyond the
0.6 m safety look-ahead; it compares truth. The SLAM lap stays in the
starter house (`tests/test_slam_live.py`).

**G2, baseline (2026-09-30), before any fix.** Instrument:
`tests/ros_verb_sweep.py` -- the real `RosDriveRobot` executor entered
through `SafetyController.check_and_execute()` as `/action` enters it,
driving a fake chain that lands twists with R4's measured timing (40-150 ms,
one in ten at 300 ms) and applies the robot server's vet where the server
does (as each twist lands, and every 50 ms wheel-loop tick), on a virtual
clock. Ground truth as in 3.18/3.19. 1440 straight verbs (3 houses x 10
starts x 12 headings x FORWARD/REVERSE x speed 50/100) and 120 pivots:

| criterion | baseline |
|---|---|
| 1 travel-to-contact >= 18.0 cm after every move | **FAIL** 1/1440 (speed 100, just under 18.0) |
| 2 no contact | pass, 0/1560 |
| 3 a verb achieving < 1 cm / 0.5 deg is a refusal | **FAIL** 8 pivots reported executed (all 236 tiny straight moves were refused, by the pre-check) |
| 4 progress | pass, **95.2%** (790/830) -- every miss an OVERSHOOT of 0.4-1.5 cm: the executor accepts a verb within 2x its 4 mm tolerance |

**"The way clear" (criterion 4) was defined after the first run, and is
recorded as such.** The plan did not define it. First operationalized as
truth's travel-to-contact >= 53 cm, that counted as "clear" 24 moves the
safety layer DELIBERATELY stops short -- its 3 cm side margin and the cone's
15 cm body are more cautious than truth -- so it scored the vet's caution as
a ROS shortfall (93.2%). Defined instead as **direct mode, from the same
pose, completes the full move**: which is the parity the gate is about. No
threshold moved.

The fixes follow from 1 and 3: the wheel vet LOOKS AHEAD as direct mode's
verbs do (a period may cover at most the room left before the line), and a
verb the vet stopped before it achieved the minimum is refused on the ROS
path as on the direct one. The executor's overshoot passes and is left alone.

**G2 MET (2026-09-30), offline and live.** Live on the G2 commit: the chain + brain-view suites 10 of 10 consecutive runs (37 passed each, 1 xfail -- R3's pan bearing), the nav2 suite 5 of 5. Three fixes,
each mutation-checked against its own criterion (removed -> only that
criterion red):

| criterion | baseline | met |
|---|---|---|
| 1 travel-to-contact >= 18.0 cm after every move | 1/1440 at 17.93 | **0/1440**, closest 19.73 |
| 2 no contact | 0 | 0 |
| 3 a verb achieving < 1 cm / 0.5 deg is a refusal | 8 unrefused | **0/252** |
| 4 progress, straight | 95.2% | **97.2%** (807/830) |
| 4 progress, turns (measured separately, 240 clear turns) | **80.0%** within 0.64 deg (p95 1.01) | **100%**, worst 0.49 |

1. **The wheel vet LOOKS AHEAD** (`SafetyController.vet_wheel_velocity`): a
   translation may cover at most the room left before the line in one
   period, so it is slowed as the line nears and stops AT it. At the line,
   no room left is a CLAMP with a reason, not a silent slow -- R2b's reverse
   test caught the first version recording no refusal. `robot/server.py`'s
   wheel loop now applies the vetted speeds whenever they differ, and
   records a refusal only with a reason.
2. **A verb the vet held is a refusal on the ROS path too**
   (`_refuse_if_nothing_achieved`, only for a robot that drives by
   velocity): under 1 cm or 0.5 deg raises the same `SafetyViolation` direct
   mode's `_guarded()` raises, so a pinned robot ends `blocked` either way.
3. **The executor settles to its tolerance**, not twice it, and turns
   finish at a 0.05 rad/s floor with a 0.5 deg tolerance (was 0.10 and 0.8).
   At the old floor the chain's 40-300 ms of delay carried each final
   correction 0.2-1.7 deg on.

Two metric corrections on the way, both recorded: a verb that covered
EXACTLY the 1 cm minimum (the look-ahead stops it at the line) read
0.99999 cm in truth's floating point, so the "under the minimum" test takes
a 1e-6 tolerance; and the turn half of criterion 4, which the plan names and
the first verdicts omitted, is now measured. `tests/test_ros_verb_safety.py`
pins all of it, including the one start that broke criterion 1.

**G3, made measurable (2026-09-30), before building.** The gate above says
what; these say how it is judged. Today, with the bridge down, every
`/action` -- a person's included -- is refused `ros_unavailable`, and on the
brain side that refusal is an ordinary `SafetyViolation`, so a mission does
NOT end: it keeps issuing refused moves until the stuck detector calls it
`blocked`. The design: the robot server knows ROS is alive from the
actuator's own heartbeat (`picar_sim_hardware` posts `/wheels` at 20 Hz), so
ROS is DOWN after 0.5 s without a post; while down, a PERSON's `/action`
runs through `drive: direct`'s guarded verb on the robot underneath (the
same `SafetyController` path, re-vetted every period), and every autonomous
`/action` is refused `ros_unavailable`, which the brain treats as the end of
the mission. Back up the moment the posts resume -- no restart.

* **G3.1 -- the mission ends.** Kill the container mid-mission: the wheels
  stop within `watchdog_timeout_s` + 0.35 s (today's bar), and the mission
  is no longer running within **3 s**, outcome `failed`, its reason naming
  ROS; the robot does not move after it ends (truth unchanged over 2 s).
* **G3.2 -- a person can drive.** Within **2 s** of the kill, a D-pad
  FORWARD executes (`executed: true`, through the fallback), and it is
  vetted: toward a wall it stops at the line or is refused
  `safety_distance`, exactly as `drive: direct` does.
* **G3.3 -- autonomy cannot.** During the outage every `/action` from an
  autonomous driver is refused `ros_unavailable`, and a mission started
  then ends `failed` at its first step without moving.
* **G3.4 -- back without a restart.** Restart the container: within **5 s**
  of its first `/wheels` post, `/health` reports ROS up, verbs go through
  ROS again, and a new mission runs.

Offline first (a real app, ROS "alive" while a test posts `/wheels` as
`ros`, "dead" when it stops), then live against the container.

**G3 MET (2026-09-30), offline and live.** Built: the robot server reads
ROS's pulse from the actuator plugin's 20 Hz `/wheels` posts (0.5 s silent =
down); while down a person's `/action` runs `drive: direct`'s guarded verb on
the robot under the ROS wrapper, every autonomous `/action` is refused
`ros_unavailable`, and `RemoteRobot` now ends the mission on that refusal
(`failed`, naming ROS) instead of treating it as a veto. `/health`
`drive.ros_up`. Offline (`tests/test_ros_fallback.py`, a real app, the
plugin's pulse played by a thread): 6 tests, all red first, each part
mutation-checked. The harness was corrected twice, both recorded in the
test: a dead container's queued twists must not land, and the pulse must
carry the plugin's actual command (posting zeros fought the chain and a
turn sometimes went nowhere -- it also showed the ROS-path refusal claiming
"the safety vet held it", which it cannot know; reworded). Live, against
the real container:

| criterion | bar | live |
|---|---|---|
| G3.1 the mission ends | <= 3 s, `failed`, naming ROS, no motion after | **2.04 s**, `failed` (ros_unavailable), unmoved over 2 s |
| G3.2 a person drives | a D-pad FORWARD within 2 s, vetted | executed through the fallback **0.36 s** after the kill |
| G3.3 autonomy cannot | refused; a mission started then ends at once | brain FORWARD refused `ros_unavailable`; mission `failed` at step 0, unmoved |
| G3.4 back without a restart | ROS up <= 5 s after the first post | up at the first post; a D-pad turn and a new mission ran through ROS |

In the first ~0.4 s after a kill -- before 0.5 s of silence -- a person's
verb is refused `ros_unavailable`: that is the detection window, inside the
2 s bar. (The first live script measured G3.3 while the D-pad still held the
robot, so the brain was refused `preempted` -- correct, and not the test;
re-run with the authority lapsed.)

**G4 is next and needs the Jetson** (arriving Oct 14-26) and the robot
base (the Rover, ordered 2026-09-30). The fake board G4 runs against is the
Rover's ROS Driver since 3.25.

**Then** `mode: hardware` defaults to `drive: ros`, and this section records
the numbers. Until every gate holds, `direct` stays the default everywhere.

### 3.25 The Rover's motor board in the sim (2026-09-30): criteria, written before building

The Rover is bought (3.21's status note), and its board is the **ROS
Driver** (`waveshareteam/ugv_base_ros`, `ROS_Driver/`), not the General
Driver R7 faked. Read from its source at commit `2e7df97` before writing
anything below `[V]`:

* **Closed loop by default.** `usePIDCompute = true` at boot and
  `setGoalSpeed()` sets it on every `T:1`; mainType 2 ("UGV Rover") is
  `WHEEL_D 0.0800`, **`ONE_CIRCLE_PLUSES 660`** (`// 1650(v=0.90) ->
  660(v>=0.93)`), `TRACK_WIDTH 0.172`. 3.16's "T=1 is open-loop PWM in mode
  2" is the General Driver's, and the "mainType 3 firmware change" is not
  needed: on this board mainType 3 is the **UGV Beast** (0.0523 m, 1092, 0.141,
  motors reversed).
* **The `1001` frame** is `T, L, R, ax, ay, az, gx, gy, gz, mx, my, mz, odl,
  odr, v`. `L`/`R` are MEASURED wheel speeds (encoder delta x
  `PI*WHEEL_D/660` over the loop's own `micros()`), m/s. **`odl`/`odr` are
  the distance each wheel has rolled since the board booted, as a C `long`
  of centimetres** -- `(long)(pulses/660 * PI * 0.08 * 100)`, truncated
  toward zero: 1 cm resolution, no rollover in practice, zeroed only by a
  reboot. `v` is hundredths of a volt, an int.
* **Feedback streams by default** (`baseFeedbackFlow = 1`), at most one
  frame per `feedbackFlowExtraDelay` = **50 ms** (`T:142` sets it); `T:130`
  goes through the same rate limit. The General Driver fake streamed at the
  board loop's ~100 Hz once asked.
* **`T:13` (ROS twist) does not feed the heartbeat** -- only `T:1` and
  `T:11` set `lastCmdRecvTime`. (This project sends `T:1`.)

**What 1 cm means.** Position from `odl`/`odr` alone is 1 cm of wheel
travel: 3.3 degrees of heading per centimetre of difference, against 3.22's
0.5-degree turn tolerance. So the counters are the **anchor** and the
measured speeds the **interpolation**: the host integrates `L`/`R` as today,
and every frame clamps each wheel's estimate into the 1 cm bucket its
counter allows. The integral's error is then bounded by 1 cm forever instead
of growing with every lost frame or mistimed `dt`, and a reading still
resolves sub-centimetre motion.

**Criteria** (each test cites the firmware line it mirrors; each red on
today's code first):

1. **The fake is the ROS Driver.** Frame keys exactly as above; `odl`/`odr`
   equal `trunc(counts/660 * PI * 0.08 * 100)` from an integer encoder count
   of the body's true wheel travel; `L`/`R` from count deltas; `T:1` in
   mainType 2 is closed loop; `T:13` leaves the heartbeat alone; streaming
   on at boot, frames >= 50 ms apart; `T:900` loads the three mainTypes'
   constants.
2. **One encoder constant.** 660 in `sim/mock_robot.py`,
   `robot/hardware_robot.py` and the fake's mainType 2, pinned together by
   one test. (Not a wall duplicate: nothing on the ROS side holds it --
   `controllers.yaml` and the plugin speak radians -- so
   `tests/test_wall_linters.py`'s registry and budget are unchanged.)
3. **Bounded error.** Over the fake with 5% of `1001` lines dropped on the
   wire and a stop-go drive with turns of at least 30 s, each wheel's
   reported travel stays within **1.0 cm** of the body's truth at every
   frame the host receives -- judged against the truth the board held when
   it BUILT that frame, so this measures estimation and not the wire's
   latency (latency is criterion 5's) -- **and** speed integration alone (the anchor removed) exceeds
   1.0 cm on the same run -- otherwise the scenario tests nothing and is made
   harder, recorded.
4. **A board reboot is not a jump.** The fake's counters reset to 0 mid-run
   (a brownout): the host's reported wheel positions and `get_odometry()`
   move by at most 1 cm across it, and stay within criterion 3's bound
   after it. Odometry by contract never jumps (section 2 of `CLAUDE.md`).
   A reboot also loses the host's set-up -- the heartbeat goes back to the
   firmware's 3000 ms -- so the host must notice and re-send it: within
   0.5 s of the reboot the board's heartbeat is `HEARTBEAT_MS` again.
5. **Verbs still mean what they mean, at 20 Hz feedback** (progress, rule
   4): over the fake, on ground truth, a clear guarded FORWARD covers **30 cm
   +/- 1.0 cm** and clear guarded turns of 15, 45 and 90 degrees land within
   **+/- 1.0 degree**, five of each. The guarded-verb hardware tests (3.22
   criterion 3) and the contract suite pass unchanged.
6. **No regressions:** the offline suite passes.

Not modelled, named so nobody assumes it was: the PID's dynamics (the fake
is an ideal PID) including its `THRESHOLD_PWM` 23 deadband, which may make
slow pivots stick-slip -- a hardware-day check; the IMU (its fields are
zeros); battery drain (`v` constant).

**Measured 2026-09-30 -- criteria 1, 2, 4 (corrected), 3 (corrected), 5
forward and 6 met; criterion 5 FAILED for turns, and why is the finding.**
Built: `sim/fake_esp32.py` is now the ROS Driver (frame, odometers, measured
speeds, 20 Hz stream, `T:142`, `T:900`'s three mainTypes, `T:13` not feeding
the heartbeat, `reboot()`, a lossy wire); `robot/hardware_robot.py` clamps its
speed integral into each odometer's centimetre (`_anchor()`), takes odometers
back at zero with the estimate far away as a reboot (`_absorb_reboot()`:
origin moved, set-up re-sent), and carries positions forward over the last
frame's age on its measured speeds (`_travel_now_m()`, at most 0.1 s); 660 in
all three places. `tests/test_ros_driver_board.py`; red first on the General
Driver fake (15 of 16 criterion-1/2 tests -- the 16th passed vacuously, no
frames, and was fixed) and on the old host against the new fake.

| criterion | old host, new fake | 3.25 |
|---|---|---|
| 3 worst error, 30 s stop-go, 5% lines lost | **9.57 cm** (speed integral alone) | within the corrected bar every frame; worst 1.02 cm where `odl` != 0, 1.63 cm where `odl` = 0 |
| 4 reboot mid-drive | travel jumps with the odometer; heartbeat stays 3000 ms | one detection, <= 1 cm move, heartbeat back to 1500 ms; 8/8 runs |
| 5 clear FORWARD, 30 cm | 30.6 - 32.8 cm | within +/- 1 cm, 5 of 5 |
| 5 clear turns 15/45/90 deg | up to +5.9 deg, mostly over | **unbiased (mean -0.5..+0.5), sd 1.3-1.8, p95 2.3-3.5, worst 4.2 deg over 120 turns -- bar was +/- 1: FAILED** |

**Corrections, made after the first run and recorded as such** (thresholds
not moved where the instrument can meet them):

* **Criterion 3's "1.0 cm" assumed every odometer bucket is a centimetre.**
  `odl` truncates toward zero, so 0 means (-1, 1) cm -- two -- and the
  odometer is built from whole encoder edges (0.038 cm) while the truth is
  continuous. The bar is now the bucket plus one edge: 1.04 cm, 2.04 cm at
  zero. Every excess over 1.0 seen was one of those two (`odl` 0: 1.29, 1.03,
  1.63; elsewhere: 1.015 at most).
* **Criterion 4's "within criterion 3's bound after it" is unreachable by
  information:** a reboot erases the board's absolute reference, so whatever
  error the estimate carried at that instant -- at most the bound it was under
  -- stays in the origin. The bar after a reboot is the bound plus that.
* **The test harness lied once, as the handoff warned they do:** the first
  non-blocking fake could write a PARTIAL line under load, garbling the next
  frame and shifting the frame-to-truth alignment -- one seed read 4.6 cm on
  454 frames. The fake now queues whole lines and loses them whole.

**Found and fixed on the way:** with streaming on from boot, a blocking pty
write stalled the fake's firmware loop while holding its lock whenever the
host stopped reading (the real board's USB bridge drops instead); and the
first reboot rule ("odometer > 3 cm from the estimate") fired falsely in 1 of
6 runs -- lost lines plus bunched arrival moved the odometer 7 -> 11 cm between
two frames the host integrated. A reboot now also needs both odometers within
+/- 1 cm of zero (the board stops its motors as it boots, so its first frame
does read zero); `test_4_a_far_odometer_away_from_zero_is_drift_not_a_reboot`
pins it, red without the guard.

**Criterion 5, turns: the finding.** Diagnosed by removing one cause at a
time, eight turns per angle each: exact (unquantised) speeds from the fake,
+/- 1.7 deg; exact speeds AND no anchor, +/- 2.5; no extrapolation, +3 to +5
deg (that half is fixed); a slowed final approach (5 deg at quarter speed, 3
deg at 0.15), no better. What is left is the host's clock: the `1001` frame
carries **no timestamp**, so the host integrates over the times frames
ARRIVE, and that jitter around a speed step is a few millimetres per wheel in
opposite directions -- 1.5-2.5 degrees on a 17 cm track. No host-side change
removes it. On the car, heading wants the gyro (`gz` is in the same frame)
or SLAM (R5) -- which `drive: ros` already has. Pinned: the +/- 1 deg test as
a non-strict xfail with this reason, and a guard that stays green: the mean of
ten turns within 2.0 deg (3.5 standard errors of the measured sd), none past
6 deg -- it catches the systematic +3 to +5 the stale frame caused. (A first
guard sized from six turns -- mean 1.5, max 4 -- flaked 1 run in 20 and was
resized from the 120-turn sample.)

**Mutation checks** (each fix removed alone): no anchor -> criteria 3, 4 and
a turn guard red; no reboot absorption -> both criterion-4 tests red; no
set-up re-send -> its test red; no extrapolation -> criterion 5 forward and
the turn guards red.

**Not done here, still due on the car:** `wheel_separation_multiplier`; the
PID deadband; whether the real firmware's loop and feedback timing match the
fake's 10 ms / 50 ms; and using `gz` -- a candidate next phase if direct-mode
turns on the car need better than +/- 2.5 deg.

### 3.26 Waveshare's own code, audited (2026-10-01) -- for the user to decide, NOT built

**Asked by the user:** what in Waveshare's code could replace code of ours,
is it any good, could we write it better, and may we flash our own firmware.
Read from source, three repositories:

* `waveshareteam/ugv_base_ros` @ `2e7df97` -- the ROS Driver board's ESP32
  firmware. **C++ as an Arduino sketch** (`ROS_Driver.ino` + ~20 headers, on
  Espressif's Arduino core / ESP-IDF / FreeRTOS). **GPL-3.0.**
* `waveshareteam/ugv_ws` @ `f0b3ad9` -- the kit's ROS 2 Humble workspace for
  the Jetson. Waveshare's own packages declare `<license>TODO</license>` and
  the repo has no root licence, so **no right to copy them**; bundled
  third-party packages carry their own (BSD, MIT, GPL, LGPL, CC-NC).
* `waveshareteam/ugv_jetson` -- the kit's stock Flask/Jupyter app.
  **AGPL-3.0.**

**"ROS Driver" is a board name, not a description.** The firmware contains no
ROS (no micro-ROS, no rosserial): newline-delimited JSON on UART0, plus HTTP
and ESP-NOW. ROS starts on the Jetson, where `ugv_bringup.py` turns `T:1001`
into `imu/data_raw`, `imu/mag`, `odom/odom_raw` and `voltage`, and
`ugv_driver.py` turns `cmd_vel` into `{"T":13,"X":..,"Z":..}`. Both open
**`/dev/ttyTHS1`** on a Jetson -- the 40-pin header UART -- which is the best
evidence yet for how the kit wires the board.

**Findings:**

| Waveshare component | Verdict | Why |
|---|---|---|
| `ugv_bringup.py` + `ugv_driver.py` (JSON <-> ROS) | **Do not use** | They own the serial port and pass `cmd_vel` straight to the motors, which takes `robot/safety.py` out of the path (3.16's reason for owning the port in `robot/hardware_robot.py`). Also weaker than ours: a 1 ms ROS timer that blocks on a serial `readline()` (stalls the executor), stamps at receipt rather than by frame, ignores the measured `L`/`R` speeds, and flushes the whole input buffer on one bad line. **Keep:** the IMU scale factors, accel `/8192` g, gyro `/16.4` LSB per deg/s, mag `x0.15` uT -- facts about the board, read from their code `[V]`, to confirm against the ICM-20948 configuration on the car |
| `ugv_base_node/base_node.cpp` (odometry) | **Do not use** | Integrates whole-centimetre `odl`/`odr` directly, so one count turns the heading by 0.01 / 0.175 rad = **3.3 deg**. First callback differences against an uninitialised `pre_odl` and a zero `last_time_`; "IMU present" is tested as `imu_yaw != 0`; publishes the IMU quaternion as orientation even when position was integrated on odometry yaw; covariance 1e-9 while stopped; track 0.175 m hard-coded against the firmware's 0.172. Ours (`hardware_robot.py`'s anchored integral -> `diff_drive_controller`) measured worst 1.02 cm in 3.25 |
| `ugv_description/urdf/ugv_rover.urdf` | **Reuse the numbers** (BSD) | CAD-derived offsets for this exact kit; they resolve most of the xacro's `[PLACEHOLDER]`s -- table below |
| `ugv_else/ldlidar` (LDROBOT's driver) | **Candidate for the car's lidar** (MIT) | Supports LD19 at 230400 baud on `/dev/ttyACM0`; the D500 is LDROBOT's STL-19P, expected to speak the same protocol `[I]`. Bears on open question 5: run it as the `/scan` source, or port its small parser into the robot server so `safety.py` does not depend on the container |
| `ugv_bringup/param/ekf.yaml` + `imu_filter_param.yaml` | **Take the idea, not the file** | `robot_localization` EKF fusing wheel odometry with the gyro, after `imu_filter_madgwick` -- the fix 3.25 named for turn scatter. Unlicensed as Waveshare's file; the configuration itself is upstream-documented |
| `ugv_nav/param/*.yaml` (nav2) | **Do not copy** | `robot_radius: 0.1` for a 0.253 x 0.231 m body whose corners reach ~0.171 m, so its plans clip furniture. R6 plans with the real rectangle |
| `explore_lite` (BSD) | Already decided | Open question 6 puts exploration in the brain |
| `rf2o_laser_odometry` (GPL-3.0) | Optional | Lidar odometry; SLAM already corrects drift (R5) |
| `ugv_jetson` stock app (AGPL-3.0) | **Disable on arrival** | Replaced by this stack; if it starts at boot it holds the serial port |
| Firmware safety | **Keep the heartbeat as a backstop; nothing replaces `safety.py`** | The board sees no obstacle (lidar and cameras are on the Jetson). Its heartbeat (`T:136`, default 3000 ms) zeroes the wheel speed once on silence; `hardware_robot.py` already sets it to 1500 ms, above the robot server's 1 s watchdog. **`T:0` "emergency stop" releases the RoArm-M2 arm's servo torque, not the wheels** -- never rely on it (our backend does not send it) |

**The URDF's numbers, measured from the floor** (`base_link` is 0.080 m up;
x forward of the wheel centre; CAD, not yet measured on the car):

| | Waveshare URDF | our xacro | effect |
|---|---|---|---|
| wheel axle height | 0.039 m (radius ~0.040) | radius 0.040 `[BOM]` | agrees |
| wheel centres, side to side | **0.1745 m** | `wheel_separation` 0.172 (firmware `TRACK_WIDTH`) | 1.5% apart; the effective skid-steer track is measured on the car anyway |
| wheelbase (driven wheels) | 0.171 m (+/-0.0855) | -- | rotation centre at the middle `[I]` holds |
| lidar | x **+0.040**, **0.120 m** up, **yawed +90 deg** | `laser_z` placeholder | a 90-degree mounting yaw means the scan's zero is the robot's left; TF must carry it or every bearing is wrong |
| OAK-D (`3d_camera_link`) | x +0.065, 0.102 m up | -- | fixed depth camera |
| pan axis | x **-0.009** | `pan_x` +0.080 placeholder | **nearly over the rotation centre** -- R3's criterion 4 failed on an 8 cm offset (4.6 deg at 1 m); on this chassis it may be moot. Re-run that test with these numbers |
| pan joint / tilt axis | 0.168 m / 0.211 m up; tilt -30 to +90 deg | `pan_z`, `camera_up` placeholders | the pan-tilt camera is ~0.21-0.23 m off the floor `[I]`, double Stage 0's 10-13 cm rig height |

The URDF carries both a fixed 3D camera and a pan-tilt camera, and four
wheels for a six-wheel body; which sensors this kit has is an arrival check.

**May we flash our own firmware? Yes.** GPL-3.0 allows modifying the firmware
and running it on your own device; its obligations start only when the
binary is *conveyed* to someone else (then our changed source goes with it).
In practice: dump the stock image first (`esptool.py read_flash`), flash only
after the 30-day-window arrival checks (`JETSON-BOM.md` 9.5), and Waveshare's
ESP32 Download Tool restores stock. Build with "ESP32 Dev Module" (the chip
is the original ESP32 -- `GUIDE-robot-base.md` section 1).

**Firmware changes worth making, smallest first** -- each is a new field, so
stock tools that read `odl`/`odr` keep working:

1. **Odometers in millimetres.** The firmware holds `en_odom_l`/`_r` as float
   metres and truncates them to whole centimetres on output
   (`ugv_advance.h:410-414`). Sending mm (or raw pulse counts) removes the
   centimetre anchor 3.25 had to build around. One line per wheel.
2. **A timestamp.** `"ms": millis()` in `T:1001`. 3.25's failed criterion --
   turns scatter sd 1.3-1.8 deg against a +/- 1 bar -- was traced to frames
   carrying no time of measurement. One line.
3. **On-board yaw** (larger): the ICM-20948's DMP quaternion code is present
   and commented out. The host-side gyro (`gz`) path is the cheaper first
   step.

Wire budget for all of it: a `T:1001` line is ~150 bytes, ~13 ms at 115200
baud; at 20 Hz that is ~26% of the link, so two short fields fit.

**Proposed order:** (1) the URDF's numbers into the xacro, tagged `[CAD]`,
and R3's criterion 4 re-run -- **done as 3.27**, which found the lidar offset
reaches the safety layer; (2) firmware changes 1-2 as one small fork,
mirrored in `sim/fake_esp32.py` first and flashed after the arrival checks;
(3) `gz` into `hardware_robot.py` or an EKF; (4) the lidar driver, once open
question 5 is settled. Each gets criteria before it is built.

### 3.27 The Rover's CAD geometry, and the lidar off-centre end to end (2026-10-01): criteria, written before building

**Asked by the user:** "start on the plan" -- 3.26's step 1. Reading the code
first turned a number swap into a phase: `robot/safety.py`, `brain/arrival.py`
and the sim's `get_scan()` all assume **the lidar sits at the rotation
centre**, and the Rover's CAD puts it **4.0 cm ahead**. Put the CAD number
into the URDF alone and TF would place every sim scan 4 cm from where the sim
cast it; leave it out and the car's safety layer is wrong by 4 cm -- in the
UNSAFE direction astern and on pivots, where a return 4 cm nearer the rear
bumper than the code believes is 4 cm of room that does not exist. So the
offset goes in everywhere at once, as one number.

**Values, from Waveshare's `ugv_rover.urdf`** (`ugv_ws` @ `f0b3ad9`, BSD,
forward kinematics computed here), converted to our frames -- `base_link` at
the rotation centre, 0.040 m above the floor:

| property | was | becomes | tag |
|---|---|---|---|
| `axle_x` | 0.0 | 0.0 (wheels symmetric at +/-0.0855) | `[CAD]` |
| `laser_x` (new) | -- (0) | **0.040** | `[CAD]` |
| `laser_z` | 0.100 | **0.080** (lidar 0.120 m off the floor) | `[CAD]` |
| `pan_x` | 0.080 | **-0.009** | `[CAD]` |
| `pan_z` | 0.070 | **0.128** (pan joint 0.168 m off the floor) | `[CAD]` |
| `camera_x` (new) | -- (0) | **0.048** (lens ahead of the pan axis) | `[CAD]` |
| `camera_up` | 0.0175 | **0.042** (lens 0.210 m off the floor) | `[CAD]` |
| `camera_pitch`, `pan_limit`, `deck_height` | | unchanged | `[PLACEHOLDER]` |

Not taken: the lidar's **+90-degree mounting yaw**. The sim's scan and the
bridge publish `laser` with zero ahead; adding the yaw to the URDF without the
sim casting in that frame would rotate every sim scan a quarter-turn. On the
car it is the D500 driver's angle offset or this joint -- a hardware-day item,
recorded in the xacro beside `laser_x`.

**Acceptance criteria:**

1. **The xacro is the CAD.** Each `[CAD]` value equals the conversion above
   within 1 mm, pinned by a test that carries Waveshare's numbers as literals
   with their source and commit.
2. **One lidar offset.** `laser_x` in the xacro, the sim's scan origin and
   `robot/safety.py`'s offset are one number, enforced by
   `tests/test_wall_linters.py` (a registry entry, inside the existing
   budget or the budget raised in this entry with the reason).
3. **The sim casts from the laser frame.** Every beam of `MockRobot.get_scan()`,
   placed through the URDF's `base_link -> laser` transform, ends on a true
   surface of the layout within 1 cm (ground-truth geometry, not
   `cast_ray()`), at 24 headings in two houses. Confirmed red against the
   unchanged sim.
4. **Safety reads the body frame.** Every scan consumer that judges distance
   to the chassis -- `footprint_clearance`, `pivot_blocked`/`pivot_scale`,
   `rear_clearance`, and `brain/arrival.py`'s range -- converts returns to
   `base_link` through the offset. **3.18's and 3.19's ground-truth bars
   hold with the offset lidar**: 0 runs under 18 cm of travel-to-contact and
   0 contacts (`tests/footprint_sweep.py`, the 3.18 sample), 0 pivots within
   1 cm (3.19), progress no worse than 1 point below 3.18's 98.8%. **And a
   mutation shows it mattered:** the same sweeps with the offset in the sim
   but NOT in safety fail at least one bar -- or, if none fails, that is
   recorded as the finding (4 cm absorbed by the margins) rather than
   tuned until one does.
5. **Arrival unchanged in outcome:** 3.11's sweep still ends `found` on
   69/69 at perfect detection and >= 99% at 90% detection, none beyond
   0.40 m measured from `base_link`.
6. **R3's criterion 4, re-measured** with the CAD pan axis and lens offset,
   and the shortcut measured from `camera_link` (the lens) rather than
   `pan_link` -- identical for the old geometry, where they coincided, which
   a check confirms. **Pass if under 3 degrees for every target at >= 1 m**,
   unchanged. If it passes, the strict xfail comes off; if not, the table is
   recorded.
7. **SLAM still maps:** one live R5 lap (`tests/demo_slam_lap.py`, drift on)
   ends with SLAM within R5's recorded 1-4.5 cm and the map within its
   96-100% of occupied cells near a true surface. A scan placed 4 cm wrong
   would show here first.
8. Whole suite green from `.venv`.

**Measured 2026-10-01 -- six met, criterion 7 met on the map and marginal on
position, criterion 3 amended for the march:**

1. **Met.** Seven `[CAD]` values within 1 mm of Waveshare's forward
   kinematics (`tests/test_cad_geometry.py`).
2. **Met.** `laser_x` = `LIDAR_X_M` = the sim's scan origin; registered in
   `tests/test_wall_linters.py`, **budget raised 10 -> 11** (the offset must
   exist on both sides: TF places scans with it, and `safety.py` judges them
   before ROS, 3.16).
3. **Met, with an amendment written down before the bar was moved.** The
   instrument was checked first: with no offset on either side, 0 of 67,000
   exact-cast beams miss a true surface; with the offset in the truth but
   not the sim, 87% miss. After the fix, 0 miss at 1 cm. **The full scan**
   (SLAM's, the ray march) cannot meet 1 cm: it over-reads by up to ~1.5 cm
   and slips past diagonal corners on 0.1-0.2% of beams from any origin
   (3.18). It is held to 2 cm and a 0.2% slip rate: **0.09% / 0.15%** after
   the fix, **67% / 68%** cast from the centre, 0.07% for a centre-cast
   control against a centre truth.
4. **Met, and the mutation shows it mattered** (60 starts, three houses):

   | | fixed | `safety.py` still assuming a centred lidar |
   |---|---|---|
   | runs under 18 cm travel-to-contact (of 2880) | **0** | **177**, worst 15.7 cm, all astern |
   | runs touching | 0 | 0 |
   | progress | 98.4% | 98.3% |
   | pivots touching (of 120) | **0**, worst gap 1.03 cm | **56**, contact |
   | pivots with room that turned into it | 100% | 83% |

   On the car, before this phase, the safety layer would have backed the
   Rover to within 15.7 cm and swung its corners INTO furniture on almost
   half of close pivots. `LIDAR_TO_REAR_BUMPER_CM` is now derived (12.65 +
   4.0 = 16.65 cm; it was 15.0, half the old sim's 30 cm robot).

   **One threshold moved after seeing data, recorded as such.** The whole
   suite then found one of 3.22's 96 guarded turns coming to **0.96 cm**
   against its 1.0 cm bar (scaled house, a LEFT 90 with 52 degrees of
   room). Cause: with the lidar 4 cm ahead, the rear corners are read from
   ~4 cm farther, and adjacent 1-degree beams land ~0.07 cm further apart
   there -- 3.19 had set `PIVOT_MARGIN_CM` = 1.2 empirically with a centred
   lidar and a closest pivot of 1.02 cm, no slack for that. Raised to
   **1.3** (1.2 plus that spacing, rounded up). Re-measured: 0/96 guarded
   turns within 1 cm (worst 1.1), 0/120 pivots touching (worst 1.03), every
   pivot with room turned into it, and 3.22's other bars unchanged (0/192
   verbs under 18 cm, clear moves 98.9% whole, clear turns 100%).
5. **Met.** 69/69 arrivals `found` at perfect detection, 669/669 at 90%,
   664/666 at 80% (3.11: 676/678 over both); no `found` farther than 0.30 m
   centre to goal.
6. **Met -- R3's criterion 4 now PASSES.** The instrument reproduces 3.12's
   table exactly on the old geometry, from `pan_link` and from
   `camera_link` alike. On the CAD (pan axis 0.9 cm behind the centre, lens
   4.8 cm ahead of it), the worst shortcut error over pan -90..90 is
   **1.89 deg at 1 m** (was 4.59), 1.25 at 1.5 m, 5.01 at 0.4 m; centred,
   0.32 deg at 0.4 m. The strict xfail is gone. What still holds: a panned
   bearing under 1 m is several degrees out, and the sim renders from the
   centre, so it cannot show even this.
7. **Map met, position marginal.** Four laps each, fresh robot and
   container, drift on, same image apart from the URDF:

   | | SLAM final (cm) | worst at rest (cm) | map precision | blocked moves |
   |---|---|---|---|---|
   | control (pre-3.27) | 1.5, 1.8, 3.0, 3.6 | 2.9-4.4 | 100% x4 | 12 x4 |
   | offset lidar | **4.7**, 4.0, 2.6, 2.8 | 3.8-6.3 | 100% x4 | 12 x4 |

   Three of four laps end inside R5's 1-4.5 cm; the first ended at 4.7. The
   means differ by ~1 cm and the ranges overlap; whether the off-centre
   sensor costs SLAM a centimetre or this is lap-to-lap noise is **not
   established** at n=4. Odometry alone ends 8.8 cm off on every lap. The 12
   blocked moves are the same in both arms: the Rover in the starter house's
   30 cm doors (3.21), not this phase.
8. **Met:** the whole suite green from `.venv`, with the live ROS stack up
   (its TF tests pass on the rebuilt image). Three tests compared beams
   with a cast from the robot's centre and now cast from the lidar.

**Still due on the car:** the lidar's +90-degree mounting yaw (D500 driver
angle offset, or this joint once the sim publishes in that frame); a tape
measure on every `[CAD]` value; and the tilt joint, which the URDF does not
model (the pan-tilt's tilt is a second servo).

### 3.28 Two fields in the board's firmware: millimetre odometers and a timestamp (2026-10-01): criteria, written before building

**Asked by the user:** 3.26's step 2. 3.25 measured the two limits the stock
`T:1001` frame puts on the host, and traced both to the frame itself:

* **`odl`/`odr` are whole centimetres.** The firmware already holds each
  wheel's travel as a float in metres (`en_odom_l`, `movtion_module.h`) and
  truncates it on the way out (`long int odl_cm = (en_odom_l * 100)`,
  `ugv_advance.h:410`). 3.25 had to build an anchor-and-interpolate scheme
  around that centimetre, and its bar was the centimetre.
* **The frame carries no time.** The host integrates speed over the frames'
  ARRIVAL times, and arrival jitter around a speed step is what left turns
  at sd 1.3-1.8 deg against a +/- 1 deg bar -- 3.25's failed criterion.

**The change, in three new keys** (new keys rather than changed ones, so
Waveshare's own tools and today's host keep working on a flashed board):

| key | value | source line |
|---|---|---|
| `odlm`, `odrm` | each wheel's travel since boot, **whole millimetres**, a C `long` truncated toward zero | `(long)(en_odom_l * 1000)`, beside the existing `odl_cm` -- so the motor-direction sign handling in `getLeftSpeed()` applies unchanged |
| `ms` | `millis()` when the frame was built: an `unsigned long`, zero at boot, wrapping after 49.7 days | `ugv_advance.h` `baseInfoFeedback()` |

Nothing else in the firmware changes. The fork lives in this repo as a
**patch against `ugv_base_ros` @ `2e7df97`**, with its own GPL-3.0 notice
(3.26: running it on our own board needs nothing more; giving the binary to
someone would mean giving the source too).

**The host** (`robot/hardware_robot.py`) uses the keys when they are present
and is exactly 3.25's code when they are not -- the board arrives stock, and
is flashed only after the arrival checks (`JETSON-BOM.md` 9.5):

* the anchor clamps into the **millimetre** each `odlm`/`odrm` allows (two at
  zero, as 3.25 found for the centimetre);
* speed is integrated over the **board's** `ms` differences, not arrival
  times, and a frame's age (for `_travel_now_m()`) is taken from the board
  clock mapped to the host's by the smallest arrival-minus-`ms` seen -- the
  usual one-way-latency estimate, which a delayed frame can only raise;
* a reboot is `ms` going backwards (beyond what a wrap explains), a more
  direct signal than 3.25's odometers-near-zero rule, which stays as the
  stock fallback;
* a `millis()` wrap is an unsigned difference, not a reboot.

**Acceptance criteria** (each red on today's code first where it can be):

1. **The patch is the change and nothing else.** It applies cleanly to a
   checkout of `2e7df97` and touches only `baseInfoFeedback()`, adding the
   three keys from the sources above (a test reads the patch). **It
   compiles** for the board ("ESP32 Dev Module", `esp32:esp32:esp32`, with
   the libraries the firmware's README lists) under `arduino-cli`; a test
   runs that compile when `arduino-cli` is installed and skips otherwise,
   and the compile is recorded here once.
2. **The fake runs either firmware.** `FakeEsp32(firmware="stock")` is
   3.25's board, unchanged; `firmware="fork"` adds the three keys, with
   `odlm`/`odrm` = `trunc(counts / 660 * PI * 0.08 * 1000)` from the same
   integer counts as `odl`, and `ms` from a board clock that starts at zero
   on boot and on `reboot()` (optionally started near the 2^32 wrap). 3.25's
   tests pass against both.
3. **The wire still has room.** A fork frame is at most 35 bytes longer
   than a stock one, and at the 20 Hz stream uses under 35% of a 115200-baud
   link (stock: ~26%, 3.26), measured on the fake's real frames.
4. **Bounded error, a tenth of 3.25's.** 3.25 criterion 3's drive (30 s
   stop-go with turns, 5% of lines lost), on the fork: each wheel within
   **one millimetre bucket plus one encoder edge** -- 0.138 cm, 0.238 cm
   where the reading is 0 -- of the truth the board held when it built each
   frame, at every frame received. Stock is unchanged (3.25's bound, still
   met).
5. **Turns land on their angle -- 3.25's failed criterion.** Over the fork,
   on ground truth, clear guarded turns of 15, 45 and 90 degrees land
   within **+/- 1.0 degree, 120 turns** (the sample 3.25 failed on), and a
   clear FORWARD covers 30 cm +/- 1.0 cm (five). Stock keeps 3.25's
   recorded result and its non-strict xfail; if the fork passes, its test
   carries no xfail. If the fork still fails, the per-turn table is
   recorded and the cause looked for, not the bar moved.
6. **Reboots, on the board clock.** Fork: 8/8 mid-drive reboots detected
   from `ms`, each moving reported travel by at most one bucket (0.238 cm),
   the set-up re-sent within 0.5 s; and **no false reboot** over criterion
   4's drive or across a `millis()` wrap (a board started 2 s before
   2^32 ms, driven through it: no reboot counted, no jump over one bucket).
7. **Mixed is safe.** A fork board's frames with the new keys stripped (a
   host-side guard against a partial flash or an older fork) behave
   exactly as stock; the contract suite and 3.22's guarded-verb hardware
   tests pass over both firmwares.
8. Whole suite green from `.venv`.

**Not in this phase:** flashing the real board (after the arrival checks);
the gyro (`gz`), which is 3.26's step 3 and may not be needed if criterion 5
passes; and the PID's deadband, still a hardware-day check.

## 4. Honest residue -- what the twin cannot tell you

All physical, all hardware-day, none a gap in this plan.

| Unknown | Why the sim can't | Lands at |
|---|---|---|
| Lidar on glass, mirrors, dark matte | A raycaster returns the geometric answer | R9 / N5 |
| Mounting vibration | No mechanical model | R9 / N5 |
| CPU contention | A laptop is not 6 A78AE cores at 25W | R9, plus P7b's preprocessing work |
| Wheel slip magnitude | Slip is modellable; the coefficient is not known | R8 calibration |
| Serial electrical reality | R7's pty tests the protocol, not the wire | R8 |

**Setup cost:** macOS means Docker. All nodes in one Linux container keeps DDS
container-internal (no multicast-across-Docker problems); it reaches
`robot/server.py` via `host.docker.internal` and publishes the bridge's port
back out. Useful consequence: **RViz is never needed** -- the twin does its
job, on a phone.

---

## 5. What this obliges elsewhere

- **`PLAN-mapping.md` §7 q2** -- answered here, not at C4. `POST /goto` is
  withdrawn; the wall goes read-only for world state.
- **`PLAN-mapping.md` N6** -- becomes R5+R6; N5 stays as written and is R9.
- **`AGENT-HARNESS.md` §4.1** -- the decided order survives as `twist_mux`
  priorities on the hardware path. The decision does not change; its
  implementation moves.
- **`PLAN-onboard-perception.md` C3** -- "a simulated serial peer, which is
  real work nobody has costed" is costed: R7.
- **`HANDOFF-2026-09-15.md` §5 item 1** -- "229 ms/frame resize against four
  cores" was written for a Pi. The board is a Jetson with **six** A78AE cores,
  and there is a third consumer: the Orin Nano has **no hardware encoder**
  (datasheet: 1080p30 per 1-2 CPU cores).
- **`HARDWARE-BOM.md` §4.1** -- add from the verified datasheet: no NVENC,
  2x M.2 Key M (x4 + x2 PCIe Gen3), 12-pin button header, 103 x 90.5 x
  34.77mm footprint (bears on §8's unmeasured chassis deck).
- **`HARDWARE-BOM.md` §6** -- no 3S 5000-6000mAh middle option is costed
  (~2.4x runtime, lower IR, no architecture change, stays inside the driver
  board's 7-13V). And the 4S fallback's **Pololu D36V28F12 is 12V 2.4A**,
  which is exactly two TB6612FNG channels at continuous rating before the
  servo -- undersized as listed.

---

## 6. Open questions

1. **Rename `service/slam/` to `service/nav/`?** It now holds nav2 too. Touches
   `tests/test_ros_containment.py`.
2. **Does `brain/tiered.py` still steer, or only pick goals?** Under nav2 the
   tier's action output competes with nav2's controller. P25's steering rung,
   Phase G's hold and `safest_direction` become inputs to goal selection rather
   than to motion. Settle at R6, against R1's measured baseline.
3. **Two collars, one robot.** `safety.py` on the teleop/vision path,
   `collision_monitor` on the nav path. They can disagree. Decide the
   arbitration before R6, not after.
5. **Who owns the lidar on the car?** (Raised 2026-09-26, after R7.) In the
   sim the robot server supplies the scan to both `robot/safety.py` (rear
   clearance, the depth path) and ROS (through the bridge). On the car the
   RPLidar plugs into the Jetson: either `sllidar_ros2` publishes `/scan` and
   the bridge hands it to the robot server, or a Python driver in the robot
   server reads it and the bridge republishes it as today. The second keeps
   `safety.py` independent of the ROS container being up, which is the same
   argument 3.16 made for the motor board.
6. **The robot's map: a search that uses it, and keeping it.** (Raised
   2026-09-27 by the user's first brain mission in their own furnished house:
   the tiered search held a cloud FORWARD into the dining room and ended
   `blocked` after 14 steps, never having seen the kitchen.) Proposed, not
   built:
   * **A frontier-exploration mission policy in the brain** -- pick a
     frontier on SLAM's map (seen floor next to unknown), send it to nav2,
     let the lidar sweep, repeat; run perception throughout, and on a
     sighting send nav2 to the target and let 3.11's arrival rule end the
     mission `found`; "searched everything reachable" when no reachable
     frontier is left. In the brain (reading `GET /world/map`, sending
     `POST /world/goal`), not ROS's `explore_lite`, so the mission's
     decisions stay in one place. This revisits 3.15's "the tier keeps
     steering by verbs for now".
   * **Saving the map**: `slam_toolbox`'s serialised pose graph (resume and
     localise) and nav2's `map_saver` files (navigate-only), on a disk that
     survives restarts -- a Docker volume in the sim, the NVMe on the car.
   * **Backing it up to AWS S3 -- decided by the user**: a private,
     encrypted, versioned prefix `maps/<robot>/<map_id>/` (the four files
     plus `meta.json`: saved-at, house label, resolution, known-cell count,
     `slam_toolbox` version), uploaded after every save, restored on
     start-up when the car has no local map, a lifecycle rule keeping the
     last few, write access limited to the robot's own prefix. It is a floor
     plan of the user's home; privacy, not cost (under 1 MB), drives the
     design.
4. **`use_sim_time`, or real time?** R0 uses `sim.realtime` and wall-clock,
   which is simplest. A `/clock` publisher would buy determinism for
   regression runs; not needed until it is.
