# Plan: ROS 2, with the twin still doing the proving

Status: **R0 BUILT 2026-09-25, R1-R9 proposed** · Date: 2026-09-25 · Phase IDs: `R0`-`R9`,
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
| **R3** | **URDF + TF.** `base_link`, two wheel joints, `laser`, `camera_link` as child of a **revolute pan joint** (ST3215). §900's 11-14cm sensor-to-bumper offset becomes a transform, not a constant. Bearings compose through the pan joint -- the general form of what `goal_pose.py` does by hand | Frames drawn on the map view, swinging as the servo pans |
| **R4** | **`picar_sim_hardware`.** Plus `diff_drive_controller`, `joint_state_broadcaster`, `twist_mux` with `AGENT-HARNESS.md` §4.1's order as priorities, and `sim_scan_node` republishing `/world/scan` as `sensor_msgs/LaserScan`. **Exactly one writer to the wheels** from here | D-pad drives through the whole ROS chain; grabbing it mid-mission still ends `preempted`, still names `twin-dpad`, still lapses on silence |
| **R5** | **`slam_toolbox` + the error readout.** Bridge serves `/world/pose` and `/world/map` from SLAM instead of `MockWorld` -- the routes the twin already consumes. Then opt-in odometry drift (`sim.odom_drift`, following `sim/sensors.py`'s pattern, default off): without drift there is nothing for loop closure to correct | "map source: sim / slam" toggle, ground-truth ghost, live error number. Drive a lap: error grows, **pose jumps, error collapses**. Hardware cannot show this |
| **R6** | **nav2 + `collision_monitor`.** Costmaps, planner, controller, recovery. `collision_monitor` between the mux and the base, with the footprint term the hand-written collar never had. **`robot/safety.py` is NOT deleted** -- it keeps the teleop and vision-policy paths. Then re-run R1's metric: a DWB/MPPI controller scores continuity in its cost function and should not flicker | Tap a goal on the map, path draws, robot follows. Block it, watch recovery. Read run-length against R1 |
| **R7** | **Fake ESP32 on a pty** speaking `HARDWARE-BOM.md` §4.2's real protocol (`T=1/11/13/126/130/131/136`, `1001`/`1002` frames), and `picar_hardware` written against it. Closes C3's stated blocker: *"nothing in this repo simulates a serial peer"*. Also falsifies §4.2's unverified belief that the heartbeat stops the motors | A drill that severs the link mid-mission; the board's heartbeat expires and reports motors stopped, watchdog quiet |
| **R8** | **Order + bring up.** `JETSON-BOM.md` as priced, plus the **latching e-stop in the motor rail** (1.16 #19, in no bill) and a pack-capacity decision (see `HARDWARE-BOM.md` §6 and the amendment noted in §5 below). `HARDWARE-BOM.md` §5 order unchanged | D-pad moves real wheels; e-stop kills them mid-move with the software none the wiser |
| **R9** | **Swap the plugin.** `picar_sim_hardware` -> `picar_hardware`, `sim_scan_node` -> `sllidar_ros2`. **Nothing above the seam changes.** Then N5's real work: scans sanity-checked in the actual house against glass, mirrors, dark matte, mounting vibration. Re-read P7e here | Same map view, same goal-tap, same recovery -- in a real room. Drive at glass and watch the ring |

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
