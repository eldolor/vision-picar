# Plan: ROS 2, proved on the sim's data

*(Retitled 2026-10-01: it was "with the twin still doing the proving", from
the watched-on-a-phone rule retired the day it was written -- see section 0.)*

Status (updated 2026-10-01): **R0-R7 BUILT (R0-R1c 2026-09-25, R2-R7 and
arrival 2026-09-26); 3.17-3.30 built on data, with 3.24's gates G1-G3 met
and G4 waiting on the Jetson; R8-R9 need the hardware** · Date: 2026-09-25 · Phase IDs: `R0`-`R9`,
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
   `ros2_control`, `tf2` and `twist_mux` all run, and
   ROS owns the metric layer down to the hardware interface.
   *(Corrected 2026-10-01: this listed `robot_localization` as running too.
   It was planned and is NOT in the container -- nothing in `service/slam/src`
   installs or launches it. It is open question 10's option (b), built only
   if that question's trigger fires on the car.)* The reason is
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

**Redrawn 2026-10-01** for 3.16's moved seam and the UGV Rover. The first
drawing put a `picar_hardware` plugin and "RPLidar via `sllidar_ros2`" in the
right-hand column; neither will exist.

```
  ROS container (service/slam/)
    nav2 -> twist_mux -> collision_monitor -> diff_drive_controller
    slam_toolbox · robot_state_publisher · tf2
    (robot_localization: not built, see open question 10)
    picar_sim_hardware (the ros2_control plugin, on the car too)
    picar_bridge (HTTP :8090)
  ======== HTTP: POST/GET /wheels, GET /scan -- the ROS wall ========
  robot/server.py: robot/safety.py (last word on every path),
                   watchdog, M4 arbitration
  =========== RobotInterface -- the backend swap (3.16) ===========
  MockRobot + MockWorld        |  robot/hardware_robot.py
    (the sim)                  |    -> USB JSON -> ESP32 ROS Driver board
                               |  D500 lidar, OAK-D Lite depth:
                               |    driver placement OPEN (questions 5, 8)
```

Everything above the seam is written once. Hardware day is a plugin swap.

**Revised at R7 (2026-09-26, 3.16): hardware day is a BACKEND swap, not a
plugin swap.** A `picar_hardware` plugin owning the serial port would take
`robot/safety.py` out of the nav path -- 3.15 put the collars in series with
it as the last word -- and would give the board two masters. So
`picar_sim_hardware` stays on the car too, still talking HTTP to
`robot/server.py`, and the seam below the robot server is the one this
project has always had: `RobotInterface`, with `robot/hardware_robot.py`
(the ESP32 over serial) beside `MockRobot`. The diagram above was redrawn
on 2026-10-01 to show this.

**No Gazebo.** The simulator already exists; a second one is a second thing to
disagree with the first.

**Containment is unchanged.** All of this lives in `service/slam/` (rename to
`service/nav/` is a live question), `tests/test_ros_containment.py` still
passes, and the twin speaks HTTP only.

### 1.1 What goes inside ROS, and what stays out -- DECIDED by the user 2026-10-02

Discussed with the user 2026-10-01 and decided 2026-10-02. Nothing in this
section is built yet; each item still gets its criteria written before
building.

**The rule: plumbing in, judgment and safety out.** ROS gets what is
generic to any mobile robot (control, mapping, planning, transforms).
These stay outside:

* **The brain**: mission logic and the LLM calls.
* **`robot/safety.py` and the watchdog**, because they must keep working
  when ROS hangs. ROS did hang: 3.15's tf2 deadlock.
* **The phone**: a web page over HTTP.

**Refinement: a sensor that feeds a veto stays outside, even though a
driver is plumbing.** The test is whether safety or the direct-mode
fallback (3.24 G3) needs it while ROS is down. That keeps these outside:

* the motor board's serial port (3.16);
* the lidar driver (open question 5);
* the OAK-D's depth for the floor band (question 8);
* M4's arbitration;
* the direct-mode verb executor (`SafetyController.run_verb()`).

**Today's line already matches the rule almost everywhere.** One piece of
logic should cross:

* **Sighting geometry: `brain/goal_pose.py` -> TF plus the lidar range.**
  `goal_pose.py` anchors a sighting in odometry by hand, and holds only a
  DIRECTION when it has no range. R3's row calls TF "the general form of what
  goal_pose.py does by hand".
  * A bridge route would take a bearing and a timestamp, read the lidar at
    that bearing, and return a POINT in the map frame. That point is composed
    through the pan joint, which also closes 3.12's warning that a panned
    bearing is not body-relative.
  * This is R1's open item: a detector that lands 1 frame in 3 needs a
    range-anchored goal.
  * The brain still decides what to do with the point.
    `goal_pose.py` stays as the direct-mode fallback.
  * Criteria, before building: the point within X cm of ground truth with
    the camera panned +/-45 degrees; arrival with a 1-in-3 detector against
    R1's ~1.6 cells closed, with the target written before the run.

**Already inside ROS, but not yet switched on.** These need configuration
or a package, not a move across the wall:

* map save and reload in `slam_toolbox`, plus `map_saver` (question 6);
* nested slow zones and cost-regulated speed in `nav2.yaml` (question 9),
  only after `safety.py`'s stop distance depends on speed;
* `robot_localization`, only if question 10's trigger fires.

**Perception: inside later, on a measured trigger only.**

* **The trigger.** The Jetson (3.24 G4) measures the tier over budget, and
  moving image preprocessing onto the GPU (Isaac ROS / NITROS) fixes it.
  P7b found preprocessing, not the model, is the bottleneck.
* **The wall then moves; it does not disappear.** Camera driver, detector
  and CLIP scoring go inside. They publish per-frame
  `detected`/`absent`/`unavailable` with a TF bearing and a lidar range.
* **The brain keeps the judgment:**
  * the match gate;
  * `brain/tiered.py`'s cloud-call triggers (`mission_start`,
    `candidate_sighting`, `cold_search`, `staleness`, and the two-frame
    hysteresis under them);
  * 1.11a's corroboration;
  * arrival;
  * the cloud call itself.

**Cost to plan for:** the bridge already serves 13 routes, exactly
`MAX_BRIDGE_ROUTES` (`tests/test_wall_linters.py`, counted 2026-10-01). Any
new route, whether for sighting geometry or map save/reload, raises the
budget. Raise it deliberately, in a commit that says why.

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
| **R9** | **Swap the BACKEND (3.16): `mode: sim` -> `mode: hardware` + `ROBOT_SERIAL`, and the camera and lidar drivers in place of the sim body's.** `sim_scan_node` -> the D500's driver (`ldlidar`, MIT; NOT `sllidar_ros2`, which was the RPLidar's), wherever open question 5 puts it. **Nothing above the seam changes.** Then N5's real work: scans sanity-checked in the actual house against glass, mirrors, dark matte, mounting vibration. Re-read P7e here | Same map view, same goal-tap, same recovery -- in a real room. Drive at glass and watch the ring |

---

### The sections live one per file

Since 2026-10-07 each `3.N` section is its own file in
`docs/plans/ros-alignment/`, indexed by
[`ros-alignment/README.md`](ros-alignment/README.md), so parallel sessions
never edit the same file (`docs/guides/PARALLEL-SESSIONS.md`). A citation
"`PLAN-ros-alignment.md` 3.18" means `ros-alignment/3.18-*.md`. Start a
new section with `python tools/plan_section.py new <slug> "<title>"`.

## 4. Honest residue -- what the twin cannot tell you

All physical, all hardware-day, none a gap in this plan.

| Unknown | Why the sim can't | Lands at |
|---|---|---|
| Lidar on glass, mirrors, dark matte | A raycaster returns the geometric answer | R9 / N5 |
| Mounting vibration | No mechanical model | R9 / N5 |
| CPU contention | A laptop is not 6 A78AE cores at **15 W** (the user's starting power mode, 2026-10-01; this row assumed 25 W when written, and 15 W makes contention more likely) | R9, plus P7b's preprocessing work |
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
- **SUPERSEDED 2026-10-01** by the UGV Rover (3.21): its ROS Driver board and
  pack replace the General Driver build this bullet priced, and the Jetson's
  separate battery is `JETSON-BOM.md` 9.5. Kept as written:
  **`HARDWARE-BOM.md` §6** -- no 3S 5000-6000mAh middle option is costed
  (~2.4x runtime, lower IR, no architecture change, stays inside the driver
  board's 7-13V). And the 4S fallback's **Pololu D36V28F12 is 12V 2.4A**,
  which is exactly two TB6612FNG channels at continuous rating before the
  servo -- undersized as listed.

---

## 6. Open questions

1. **Rename `service/slam/` to `service/nav/`?** It now holds nav2 too. Touches
   `tests/test_ros_containment.py`. **Dropped 2026-10-01** (closed by Claude,
   delegated by the user): a rename churns every doc and a test for no
   change in behaviour.
2. **Does `brain/tiered.py` still steer, or only pick goals?** Under nav2 the
   tier's action output competes with nav2's controller. P25's steering rung,
   Phase G's hold and `safest_direction` become inputs to goal selection rather
   than to motion. Settle at R6, against R1's measured baseline.
   **Half-answered (2026-10-01 note):** 3.15 decided "the tier keeps
   steering by verbs for now". Question 6's frontier search reopens it,
   because it hands motion to nav2 goals. Settle it with question 6.
   **Taken up as 3.31's `explore` policy**, which sends nav2 goals.
3. **CLOSED -- decided in 3.15: the two collars run in SERIES**, nav2 ->
   `twist_mux` -> `collision_monitor` -> ... -> `robot/safety.py`, with
   `safety.py` the last word on every path. The question as raised:
   *Two collars, one robot.* `safety.py` on the teleop/vision path,
   `collision_monitor` on the nav path. They can disagree. Decide the
   arbitration before R6, not after.
4. **`use_sim_time`, or real time?** R0 uses `sim.realtime` and wall-clock,
   which is simplest. A `/clock` publisher would buy determinism for
   regression runs; not needed until it is.
5. **Who owns the lidar on the car?** (Raised 2026-09-26, after R7. The
   lidar is now the Rover's **D500**, not the RPLidar this was first written
   for.) In the
   sim the robot server supplies the scan to both `robot/safety.py` (rear
   clearance, the depth path) and ROS (through the bridge). On the car the
   D500 plugs into the Jetson: either a ROS driver (`ldlidar`, MIT) publishes
   `/scan` and the bridge hands it to the robot server, or a driver in the
   robot server reads it and the bridge republishes it as today. The second
   keeps `safety.py` independent of the ROS container being up, which is the
   same argument 3.16 made for the motor board.
   **DECIDED by the user 2026-10-02: the second.** See
   1.1: a sensor that feeds a veto stays outside. The cost is porting the
   D500's serial protocol out of `ldlidar`'s ROS node into the robot process.
   Before this goes to the car, the fake board and the sim's scan need a
   matching fake lidar on a pty, as R7 did for the motor board.
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
   * **A saved map is a starting guess, not the truth (decided by the
     user 2026-10-01: "let's go with your recommendation").** Furniture moves between sessions. nav2's
     static layer is drawn from SLAM's `/map`, and the live obstacle layer's
     ray clearing never clears it. So a map loaded once and frozen keeps a
     moved sofa as a phantom obstacle, which can make a room look
     unreachable. **Recommended:** reload `slam_toolbox`'s serialised pose
     graph and keep **mapping** on top of it, rather than localising only
     against a frozen copy. `/map` is then redrawn about every second
     (`map_update_interval` 1.0), and new scans outvote the old ones, so a
     ghost fades as the robot re-sees the spot. **Fallback:** start a fresh
     map when the reloaded one matches badly. The trigger is unchosen
     (scan-match response, or a share of new scans contradicting the map).
     **Not lifelong mode:** it is the purpose-built answer, but it is
     experimental upstream and is only worth trying if pose-graph growth
     across many sessions becomes the measured problem. Open in this
     design: where the robot starts on a reloaded map (a known dock, a
     given pose, or a whole-map localise; `slam_toolbox`'s deserialise
     offers all three). **Criteria, to be confirmed before building:**
     (a) with 7's movable furniture, move an object across a doorway
     between two sessions: on the second session the doorway reads free on
     `/map` within N re-sightings, and a goal through it succeeds;
     (b) localisation on the reloaded map stays within 3.14's bars (5 cm /
     3 deg at rest) with up to M objects moved; (c) a no-change reload is
     no worse than a fresh map on the same lap. N and M are set from a
     first run, before the bar is applied.
   * **An unreachable frontier is retried, not dropped (agreed by the
     user 2026-10-01).** The search sets aside a frontier nav2 cannot reach, for
     example a gap between dining chairs. A person or pet standing in a
     doorway causes the same "unreachable" for a minute, so set-aside must
     be **temporary**: a frontier comes back after a cooldown, or sooner
     when the map changes near it, and is dropped only after K failures at
     different times. "Searched everything reachable" may only be declared
     once every set-aside frontier has had its last retry. **Criteria, to
     be confirmed before building:** (a) a mover parked in the only
     doorway to the target's room for 60 s, then leaving: the search still
     ends `found`; (b) a truly unreachable frontier (the dining-chair gap)
     costs at most K attempts and does not stall the mission; (c) arrival
     rate on the existing search starts is unchanged against R1's baseline.
     **3.30 adds a second case:** R1b's stuck detector ends a mission
     `blocked` after five refused FORWARDs and cannot tell a wall from a
     person who will move (the same mission: `found` in 41 s alone,
     `blocked` after 315 s with someone walking the hallway). Whatever
     retry rule frontiers get, the stuck detector should share it.
7. **Things that move, in the simulator (agreed by the user 2026-10-01;
   first in order).** **BUILT as 3.30** (the results are there; this entry is
   the proposal as written). Every
   object in the sim is static (`GridWorld.objects`, solid since 3.9), so
   nothing in this plan has been tested against moved furniture, people or
   pets. The cheap part already exists in the data model: solidity is
   `solid_cells`, derived from `objects` on every call, and the scan,
   collision and `MockWorld`'s discovered map all read it. So moving an
   object is a dict update, and everything that senses picks it up. Two
   pieces:
   * **Movable furniture.** `GridWorld.move_object(from, to)`, plus a
     sim-only route on the robot server (like `/world/truth`, answering
     "unavailable" on hardware), so a test or the twin can move the sofa
     mid-session or between sessions.
   * **Movers (people, pets).** Scripted solid objects whose cell is a pure
     function of time along a path (deterministic, so tests can inject the
     clock). A mover never steps into the robot's footprint; it waits, so
     the sim never puts the robot inside an obstacle. It is drawn by the
     camera as a billboard, like any object. **Fidelity limit, stated up
     front:** a mover hops whole 30 cm cells. That is good enough for
     "something appeared and later left"; it cannot test a pet darting at
     the robot. A continuous disc in the ray caster is the upgrade if a
     result needs it.
   **Criteria, to be confirmed before building:** (a) a moved object
   appears at its new cell in the scan, collision and `MockWorld` within one
   scan period, with no restart; (b) 3.18's ground-truth sweep, re-run with
   movers crossing the robot's path, never closes under 18 cm of
   travel-to-contact, and the robot never ends inside a mover; (c) a nav2
   goal with a mover crossing the corridor still succeeds at 3.15's rate;
   (d) with movers off, every existing pinned trace is byte-identical.
   This is the prerequisite for testing 6's saved-map and frontier-retry
   items and 9's speed rule, so it goes first.
8. **Seeing below the lidar's plane (proposed 2026-10-01).** The lidar
   sees one horizontal slice, 12 cm off the floor (3.27, from Waveshare's
   CAD). A cat lying flat, a dog's tail, a shoe or a cable sits
   under it, and nothing else on the car looks there: `HardwareRobot`
   answers `get_depth_grid()` with "unusable" (`robot/hardware_robot.py`
   481), so `robot/safety.py` runs on the scan alone. The UGV Rover kit's
   OAK-D Lite is a depth camera, and `RobotInterface.get_depth_grid()` (M2)
   plus the depth-grid veto (M3) are already the path that would carry it.
   **Proposed:** (a) a depth-grid source on the car from the OAK-D Lite;
   (b) a **floor band** in `robot/safety.py` (which, by 1.1's proposed rule,
   puts the OAK-D's depth driver -- `depthai` -- in the robot process, not
   `depthai-ros`): depth returns between the
   floor and the lidar's plane, inside the swept corridor, veto forward
   motion, in series with the scan checks as 3.18 does. (c) The sim cannot
   test this today, because its world is 2D: every object fills the
   lidar's slice. It needs objects with a height, so a **low** object is
   invisible to `get_scan()` and visible to the depth grid. **Criteria, to
   be confirmed before building:** (a) in the sim, a low object in the
   corridor stops the robot at `min_distance_cm` (ground truth, 3.18's
   method) with the scan alone missing it, and the check fails red without
   the floor band; (b) no false vetoes from the floor itself on the
   furnished-home tour (the floor must not read as an obstacle, which is
   1.12's camera-tilt problem); (c) on the car, the same stop against a
   real low object, measured with a tape.

   **What the camera can see, researched 2026-10-01** (vendor sources,
   not measured on a car; `[V]` = read from the source, `[I]` = computed
   from it):
   * **Mount `[V]`.** Waveshare's own description of the Rover
     (`waveshareteam/ugv_ws`, `ugv_description/urdf/ugv_rover.urdf`) puts
     the OAK-D Lite (`3d_camera_link`) **fixed to the chassis, level**
     (rpy 0 0 0), 6.5 cm ahead of `base_link` and 2.2 cm above it. Its
     `base_link` sits 8 cm above `base_footprint`, so the camera is about
     **10.2 cm off the floor** `[I]`. It is **not** on the pan-tilt: that
     carries the separate 5 MP wide-angle camera (`pt_camera_link`), which
     is the perception camera's analogue. The same file puts the D500
     lidar about **12 cm** off the floor `[I]`, which 3.27 has already
     put in the xacro. These are a CAD file's frame origins, not optical
     centres; measure on the car.
   * **Range `[V]`** ([Luxonis, OAK-D Lite](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)):
     stereo pair 640x480, field of view 73 deg x 58 deg (H x V), baseline
     7.5 cm (the page prints "75cm", a typo). Minimum depth ("MinZ") is
     **~20 cm at 400P with extended disparity**, **~40 cm** without it,
     ~80 cm at 800P. Ideal range ~80 cm to 12 m.
   * **What that means for a 20 cm stop `[I]`.**
     - The lens sits about 6 cm behind the front edge (the body is 25.3 cm
       long), so the stop line, 20 cm ahead of the bumper, is **about 26 cm
       from the lens**.
     - With extended disparity (MinZ ~20 cm), the stop line is in range
       with about 6 cm to spare.
     - **Waveshare's stock driver config does not enable extended
       disparity**: `ugv_vision/config/oak_d_lite.yaml` sets only
       `i_subpixel: true`. So out of the box MinZ is ~40 cm, about 34 cm
       past the bumper, and the camera **cannot see the stop line**. The
       floor band needs `400P + extended disparity` (and whether depthai
       allows it together with subpixel on this driver version is to be
       checked).
     - **Floor visibility.** A level camera at 10.2 cm with a 29 deg
       half-angle down sees the floor from about 18 cm ahead of the lens.
       A 5 cm-high object is in view from about 9 cm. So at the stop line
       the geometry is fine and MinZ is the binding limit. Width at 26 cm
       is about 38 cm, wider than the 23.1 cm body.
     - **What it still cannot cover:** anything that enters the last
       ~14 cm in front of the bumper (inside MinZ), the sides while
       pivoting (73 deg forward field only), and reverse. For those, the
       lidar and low speed remain the only protection; this is a forward
       floor band, not a ring.
     - **Grazing floor.** A level camera 10 cm up sees the floor at a
       shallow angle, where stereo depth is noisiest; criterion (b)'s
       false-veto bar is the test of whether that matters.
9. **Speed set by clearance: faster in open space, slower near things
   (agreed by the user 2026-10-01; after 7, and after the speed-dependent
   stop).** Today nav2 is capped flat at 0.2 m/s
   (`desired_linear_vel`), which is what makes a fixed 20 cm stop safe. A
   good part of the dynamic behaviour already exists and only needs
   tuning, not code:
   * `collision_monitor`'s `approach` action brakes on **time** to
     collision (1.0 s). A faster robot therefore starts braking further
     out, automatically.
   * `PolygonSlow` halves the speed within about 20 cm ahead. It can become
     **nested** zones, for example full speed beyond 1 m, a fraction inside
     1 m, the 0.2 m/s of today inside 0.5 m.
   * Regulated Pure Pursuit's `use_cost_regulated_linear_velocity_scaling`
     (off today) slows the controller as costmap cost rises near
     obstacles, and its `max_allowed_time_to_collision_up_to_carrot` is
     already on.

   **The part that is not tuning:** `robot/safety.py`'s `min_distance_cm`
   is a **fixed** 20 cm. `PLAN-onboard-perception.md` 1.14 item 5 ("The
   collar becomes speed-dependent, and 20cm is already marginal") puts it
   at about 0.45 m/s, and only **~0.15-0.2 m/s once its corrected reaction
   time and the sensor offset are applied**. Today's 0.2 m/s is therefore
   already at the edge, and the robot server's stop must become
   speed-dependent, as 1.14 item 5 proposes, **before** any speed is
   raised. That is the same "cover the travel until the next check" lesson
   as 3.18 and 3.22. Two limits stay regardless: the lidar's 10 Hz
   (at 0.5 m/s, 5 cm between scans) and 8's blind band below the lidar,
   which argues for keeping speed low near the floor-level unknown until 8
   is built. **Criteria, to be confirmed before building:** (a) on the
   scaled house and the furnished home, 3.15's goal success and closest
   approach are no worse, and mean speed on open stretches rises by a
   stated factor; (b) 3.18's ground-truth sweep at the new top speed: no
   run under 18 cm of travel-to-contact; (c) with 7's movers crossing:
   never within 20 cm of a mover that was still when the robot committed,
   and the commanded-speed log shows the slow zones engaging; (d) on the
   car, measured stopping distance at each speed band, against the
   formula, before the sim's bands are trusted.

   **The speed-dependent stop is built (3.44, merged 2026-10-07):**
   `required_clearance_cm()` keeps 20 cm up to ~0.39 m/s and grows above it
   (25 cm at 0.4, 35.5 at 0.5), with criterion (b)'s sweep met at 0.4 and
   0.5 m/s in both sensing modes. Still open here: (a), (c), and (d) --
   `DECEL_M_S2` is a placeholder until stopping is measured on the car.

   **What 3.42 (the lidar driver) changes here (added 2026-10-07).** The
   detail is in `HANDOFF-2026-10-07-lidar-and-speed.md`.
   * **Count the lidar's delay once.** `_aged()` already subtracts
     `v x sensor_age_s()`; on the car that is the lidar's age. The stopping
     formula's reaction time must leave that out, or the robot stops
     further out than needed.
   * **Build the bands on the measured scan age.** It was 0.15-0.2 s when
     read, not 0.1 s: 7.5-10 cm at 0.5 m/s before any braking.
   * **Grow the safety scan hint with the clearance.**
     `SafetyController._scan()` asks for about 0.6 m today. The simulator
     reports beams beyond the hint as None and a real lidar ignores it, so
     a larger clearance with the old hint makes the simulator hide
     obstacles. That errs unsafe, and in the sim only.
   * **Criterion (b) is to be run on the lidar-timed sweep too**
     (`fs.sweep(..., lidar=True)`, `fs.pivot_sweep(..., lidar=True)`: the
     D500's timed scan, no depth grid). 3.42 measured it only at 0.1 m/s
     and 1 rad/s.
   * **Settle the scan time stamp before raising nav2's speed.** 3.42's
     open time stamp item, decided by the user, matters more at speed.
10. **Gyro-based heading: a phase, if turns on the car need better than
   about +/- 2 deg (raised 2026-10-01, from 3.25's failed turn criterion;
   for the user to decide).** Where turns stand without it: stock firmware,
   sd 1.3-1.9 deg, worst 4.7 (3.25, 3.28); the 3.28 fork, 104/120 within
   +/- 1, worst 2.1; 3.29's settle pass is the planned close of the +/- 1
   bar. All of that is the **sim's** number, where a wheel never slips.
   **Why the question outlives 3.29:** the Rover is a 4-wheel skid steer,
   and its wheels SCRUB on every turn, so encoder heading is wrong on the
   car in a way no fake board shows -- `wheel_separation_multiplier` corrects
   the average, not the run-to-run spread on carpet against tile. The
   ICM-20948 gyro measures the rotation itself, and `gz` is already in every
   `T:1001` frame.
   * **Trigger -- decide on the car, not before:** the arrival check's turn
     test (3.26) -- commanded 15/45/90-degree turns against a measured
     reference (lidar on a wall, or floor marks), on the floors the robot
     will actually drive. If direct-mode turns miss +/- 2 deg there, or SLAM
     (R5) is visibly fighting odometry heading, this becomes a phase.
     Until then it is not built: 3.29 may make it unnecessary on hard
     floors, and the gyro's bias and noise are unknown until measured.
   * **Two ways to build it** (3.26 step 3): (a) `gz` integrated in
     `robot/hardware_robot.py`, so direct-mode verbs close a turn on the
     gyro and the encoders keep distance -- small, and keeps `safety.py`'s
     path unchanged; (b) `robot_localization`'s EKF fusing wheel odometry
     and the gyro on the ROS side (3.26: take Waveshare's idea, not its
     file), which helps `drive: ros` and SLAM but not direct mode. Likely
     (a) first, since direct mode is the car's fallback (3.24 G3).
   * **Sim first, as always:** the fake would need a gyro (rate with bias,
     noise and the firmware's `/16.4` LSB-per-deg/s scale, 3.26) and the sim
     body would need wheel slip on turns, or the phase's data says nothing
     about the car. **Criteria, to be confirmed before building:** turns
     within +/- 1 deg on ground truth with a slip model ON (where the
     encoder-only host measurably fails), a stationary robot's reported
     heading drifting no more than a stated deg/min, and every 3.22/3.29 bar
     unchanged.
11. **Isaac ROS 5.0: anything worth taking, and on which stack? (raised
   2026-10-05, from NVIDIA's ROSCon announcement; for the user to decide,
   after the Rover.)** Released 2026-09-21/22, free and open source. **It
   requires ROS 2 Lyrical, Ubuntu 24.04 and JetPack 7.2** (the Isaac ROS
   supported-platforms table lists "Jetson Thor ... and Jetson Orin" on
   JetPack 7.2; the Orin Nano is not named). The board runs JetPack 6.2.1
   with the Humble container (3.33, results 2026-10-04), and every phase
   from R3 on was proven there, so **Isaac ROS 5 is out of reach without a
   platform migration.** 5.0 also removes NITROS in favour of Lyrical's
   `rosidl::Buffer`, so anything written against NITROS is ported, not
   upgraded.
   * **Not for us:** FoundationPose (6-DoF object pose -- arrival reads the
     lidar's range, not a pose), cuMotion and the pick-and-place skills
     (arms; the Rover has a pan-tilt), FoundationStereo (a heavy depth
     model competing with YOLOE + CLIP for the GPU inside 3.33's 250 ms a
     frame at 15 W), Isaac Sim (the grid sim and the fake board carry the
     data-driven definition of done).
   * **The idea, not the package:** a GPU image pipeline. P7b found the
     tier's cost is CPU preprocessing (229 ms resizing against 36 ms
     detecting), but Isaac ROS accelerates images that travel the ROS
     graph, and perception lives in the brain process, outside the wall.
     P26 (decode once, batch the crops, resize on the GPU in torch) is the
     same win without moving perception into ROS.
   * **Worth a measured test, after the Rover:**
     (a) **cuVSLAM** -- visual-inertial odometry from the OAK-D Lite's
     stereo pair and the IMU. A candidate answer to 10 (heading on a
     scrubbing skid steer) through its option (b), and a second opinion
     against `slam_toolbox`'s false loop closures in the furnished home.
     (b) **nvblox** -- 3D obstacles from depth into nav2's costmap: what a
     single lidar plane misses (chair crossbars, table aprons). It overlaps
     8, but would sit on nav2's side of the collars; 8's floor band stays
     in `robot/safety.py` either way, by 1.1's rule that a sensor feeding a
     veto stays outside ROS.
     Either lives in `service/slam/`, behind the HTTP wall, so
     `tests/test_ros_containment.py` is unaffected.
   * **Check before migrating:** ~~the release notes put Isaac ROS 4.4/4.5
     as the last on JetPack 6 / Humble (read, not verified).~~ **Corrected
     2026-10-06 (3.41): 3.2 is the last on JetPack 6 / Humble; 4.0 moved to
     JetPack 7, and Orin support returned only in 4.6 on JetPack 7.2.** If an older
     cuVSLAM or nvblox runs there, (a) and (b) can be measured with no
     migration at all. A move to JetPack 7.2 / Lyrical would be its own
     phase, with criteria, and would re-prove R3-R7 and G1-G4 -- the
     source-built tf2 fix and `slam_toolbox` commit included -- never
     mixed into hardware bring-up.
   * **Trigger:** the Rover's arrival checks. If 10's turn test fails, or
     the car's lidar visibly misses obstacles that 8's floor band does not
     cover, try (a) or (b) on the current stack first.
12. **Leaving Humble before its EOL: when, to what, and in which order?
   (raised 2026-10-06; the user agreed to plan it, not to start it.)**
   ROS 2 Humble reaches end of life in **May 2027** and Ubuntu 22.04's
   standard support ends in **April 2027** (ESM runs to ~2032). JetPack
   6.2.1 and CUDA 12.6 have no published end of life; NVIDIA supports
   6.2.x as the Orin line. The dates come from memory, not a lookup:
   check REP-2000 and NVIDIA's Jetson roadmap before scheduling against
   them. EOL means no more patches, not a stack that stops working, and the
   robot is indoors on a home LAN, so this is not urgent.
   * **Not during hardware bring-up.** G1-G4, 3.33's headroom and every
     R-phase were measured on Humble + JetPack 6.2.1. A distro change under
     the Rover's arrival checks would make every failure ambiguous between
     the car and the upgrade. Those results are the baseline the car is
     judged against.
   * **Two upgrades, not one, because of the containment rule.** ROS lives
     only in `service/slam/` (`tests/test_ros_containment.py`), so:
     (a) **The ROS distro is a container rebuild.** A 24.04 userspace runs
     on the 22.04 host, and nav2 / `slam_toolbox` need no CUDA. Nothing in
     `brain/`, `control/`, `robot/` or `world/` changes. Check whether the
     target distro's packages make the two source builds unnecessary (the
     tf2 deadlock fix from the 0.25.24 tag, R6; `slam_toolbox`'s
     `restamp_tf` from a pinned commit). Do not assume they do.
     (b) **JetPack 6 -> 7.2 touches the host:** the torch wheel and its
     `numpy<2` pin, perception latency, 3.33's headroom, and the NVMe boot.
     Gated on JetPack 7.2 supporting the Orin Nano Super by name (11 found
     the Isaac ROS table lists "Jetson Orin" but not the Nano).
   * **The target distro: Jazzy or Lyrical, decided with 11.** Jazzy (LTS,
     May 2029) is the conservative step. Lyrical is what Isaac ROS 5
     requires, so choose it only if 11's cuVSLAM / nvblox tests on the
     current stack show something worth migrating for. Doing (a) to Jazzy
     and then again to Lyrical would cost the re-proof twice.
   * **Criteria, written now so the data decides:** the same bars the
     current stack met, on the board, through the real mission path. These
     are G1 (live chain 20/20), G2 (3.18/3.19's ground-truth sweeps: 0
     under 18 cm, 0 contacts, 0 close pivots touching), G3 (fallback ends a
     mission `failed` within ~2 s of ROS dying; a person still drives), G4
     (5 consecutive runs of 18 on the fork firmware), R5's SLAM error and
     map precision, R6's nav2 goals (6/6 twice, scaled house), and for (b)
     3.33's perception latency (p90 within 10% of 109.9 ms at 15 W) and
     headroom (0 late wheel-loop ticks under nav2 + perception + SLAM).
     Any miss means the upgrade does not ship; Humble stays.
   * **Target: Q1 2027**, after the car's first data-driven phases close,
     so (a) lands with margin before May 2027. If (b) is not ready then,
     (a) goes alone.
