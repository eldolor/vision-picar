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

**How to run it:** `docker build -t vision-picar-ros service/slam`, then
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
   the transform tree already covers makes it impossible, and both runs above
   use it. And the controller hid it: `isGoalReached()` ignores a failed
   transform and compares the robot with a default pose at odometry's origin,
   so a robot near its start "reached" goals metres away.

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
4. **`use_sim_time`, or real time?** R0 uses `sim.realtime` and wall-clock,
   which is simplest. A `/clock` publisher would buy determinism for
   regression runs; not needed until it is.
