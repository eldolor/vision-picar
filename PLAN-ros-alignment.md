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

**What does NOT change, and is the binding constraint on everything below:**

> §7's rule. A phase is not done when its tests pass. It is done when someone
> holding a phone can watch the thing it built do its job.

Stated non-negotiable by the user, 2026-09-25, when the first draft of this
plan conceded that Track D could only be watched on hardware. That concession
is withdrawn -- see §2.

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
| **R0** | **DONE 2026-09-25 (not deployed).** **Continuous pose + diff-drive kinematics.** C2 and 1.14 merged. `GridWorld` holds float `x`/`y`/`theta`; `MockRobot.set_wheel_velocity()` / `step()` / `get_wheel_state()` take **left/right wheel angular velocities** and integrate over `dt`; encoder counts fall out of that integration; continuous collision via `renderer.cast_ray()`. Wheel velocities rather than a twist **on purpose**: it puts `diff_drive_controller`'s kinematics under test with the parameters that will ship | Map view draws a robot between cells at a non-cardinal bearing; FPV and depth strip track smoothly. **The D-pad half is still owed** -- see 3.1 |
| **R1** | **P25's A/B, finally runnable.** `brain/goal_pose.py` is built and default OFF because the sim turned in 90° quanta against a 10° centre band. Wire into `brain/tiered.py`; add median-run-length and reversal metrics to `control/walk_eval.py`; run it. **Before ROS**, so R6 has a baseline | Run-length rises above 1.0; no more LEFT/RIGHT alternation on a stationary target |
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
a `FORWARD` then lands the robot off both grid lines. **But no twin control
sends an angle other than 90**, so the phone cannot yet *command* a
non-cardinal turn, and the row's "D-pad rotates through non-cardinal angles"
is unmet. Two candidates, both small: an angle stepper beside the D-pad, or a
press-and-hold that sends a velocity (which is the one R4 will want anyway).
Settle it with R1, whose A/B is the first consumer that cares.

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
