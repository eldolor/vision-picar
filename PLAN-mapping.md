# Plan: map the house while it searches

Status: **PROPOSED, nothing built** · Date: 2026-09-19 · Phase IDs: `N1`-`N7`,
alongside `S*` (`PLAN-sim-hardening.md`), `B*` (`PLAN-brain-relocation.md`),
`M*` (`PLAN-microduck-transplants.md`), `T*` (`PLAN-teleop-robot.md`),
`C*`/`P*` (`PLAN-onboard-perception.md`).

Written against the repo as of `c7ba8c5`.

The next phase of the project is that the robot **maps the house as it drives
around searching for objects**. This document is what that decides, what it
promotes from optional to load-bearing, what it must *not* redesign because it
is already decided, and the seven phases -- four of which need no hardware.

---

## 0. What this decides, in one line

`PLAN-onboard-perception.md` §3.3 surveyed four ways to use the lidar and
picked one **conditionally**:

> **(b+) ROS behind an HTTP wall. CHOSEN as the target if mapping proves to be
> the point.** Run ROS 2 as *one isolated service* on the Pi exposing a tiny API
> -- `GET /pose`, `GET /map`, `POST /goto` -- and nothing else in the system
> knows ROS exists.

Mapping is now the point. **So (b+) is no longer conditional, and ROS 2 enters
the project -- as exactly one process, behind exactly one wall.** Never (c)'s
full adoption, never (b)'s hand-rolled drifting SLAM, and never anywhere near
`brain/`, `control/` or `RobotInterface`.

Two of (b+)'s three stated costs have evaporated since §3.3 was written, by
accident rather than by design:

- **The OS dilemma is gone.** §3.3 costed an awkward choice between Ubuntu (ROS's
  native platform, at the cost of the Pi camera stack) and ROS-in-Docker-on-Pi-OS.
  P19 moved the board to a Jetson and **JetPack is Ubuntu 22.04, ROS 2 Humble's
  native platform** -- §4.7 says so itself.
- **The memory objection is gone.** The standing objection to a Jetson was that
  8GB shared with the GPU cannot hold SLAM, nav2, a detector and a local VLM at
  once. `EDGE-PERCEPTION-BENCH.md` dropped the local VLM -- the board now exists
  to run a **150M ViT** -- which "leaves the memory for SLAM and nav2, removing
  the main objection to the Jetson rather than paying for it." P22 widens that
  further: YOLOE reads 90% at 3 FP in **170ms against OWLv2's 2679ms**, so the
  latency headroom a nav stack wants is there too.

The third cost stands and should not be talked down: **(b+) is weeks, and most
of it is learning rather than writing** -- TF frames, a URDF, nav2 parameters,
launch files, `slam_toolbox` config. None of it hard; all of it unfamiliar.

---

## 1. Two maps, and the join between them is the actual work

"Map the house" is two projects. The project already owns a weak version of one
of them, which is exactly how they get conflated.

**The metric map.** An occupancy grid, a pose you can trust, loop closure so a
lap of the house lines up with itself. This is `slam_toolbox` and nav2. This is
the ROS part, and it is the part that does not exist in any form today.

**The semantic map.** Which rooms exist, which have been searched, what was seen
where. `brain/memory.py` is a weak version of this now: `visited_rooms`,
`searched_rooms`, and `Sighting(step, object_name, room, position)`.

**The join is the new thing, and it is where the value is.** Today
`Sighting.position` is a grid cell from a simulator and `room` is a landmark
match re-guessed from every photograph by `brain/rooms.py`. Under a metric map:

- a **sighting is anchored to a pose**, so "the basket is at (4.1, 2.3)" survives
  the robot turning away from it;
- a **room is a labelled region of the occupancy grid**, so "I searched the
  kitchen" becomes a claim about geometry instead of a claim about vibes;
- **coverage becomes computable**, which is what §1.7 says is missing.

That last one is worth stating as a decision this plan unblocks. §1.7 left a
fifth goal verb out on purpose:

> **`sweep` is deliberately left out.** Its stop condition ("have I covered the
> room?") is a mapping question, and under (a) there is no map.

There is about to be a map. `sweep` becomes implementable for the first time --
**and the rule attached to it still holds**: add it when the trigger log shows
the planner reaching for it, a measured signal rather than a guess. N3 builds
the coverage answer; it does not ship the verb on spec.

---

## 2. What is already decided -- do not redesign this

Three sections of `PLAN-onboard-perception.md` already did the design work and
must be implemented rather than re-argued.

**§1.5 -- ten persistence decisions, all of which survive.** Only the map
persists; `MissionMemory` stays in RAM in the brain, unchanged. The planner is a
**pure function** -- context in, goal out, writes nothing -- and the brain is the
sole writer. House id from **config**. Map keyed **by house**. Every edge carries
last-confirmed plus success/failure counts, both visible to the planner; eviction
deferred. An empty graph is **not a special mode**, but the planner's context says
"map is empty" explicitly. Store is **DynamoDB, AWS durable with a Pi working
copy**, `schema_version` from the first write, and **stored map text is data,
never instruction**.

**§1.6 -- the visual-edge memory stays cancelled, and this plan is why.** Q1
designed a topological map whose edges carried an embedding of what a doorway
looked like, so a robot with no coordinates could navigate by searching for a
remembered view. It was cancelled *because (b+) deletes it*: coordinates need no
interpretation. §1.6 left "the topological graph as a placeholder that (b+) fills
in with real geometry." **This plan is the filling-in.** Nothing here revives
embeddings, bags of views, per-direction memories or match confirmation.

**§3.4 -- the map is the one thing that outlives a mission.** "Persisting
[`MissionMemory`] would buy mid-mission resumption that cannot be used: a robot
that crashed does not know where it is any more, which is the no-map problem
again. The map is what needs to outlive a mission, because the whole value of
mapping a house is that the second trip is cheaper." N4 is that sentence, built.

**And §3.3's own sequencing rule.** (a) is not a fork off (b+); it is the first
two weeks of it, ~80% reusable -- the driver, the `get_depth_grid()` feed, §5.1's
angular fix and the twin readout are needed either way. The reason to stop at (a)
first is not caution, it is diagnostics: **layering SLAM onto scans you have not
validated in the actual house is how a mounting-vibration problem gets debugged
as a mapping problem.**

---

## 3. What this promotes from optional to load-bearing

| Thing | Was | Is now |
|---|---|---|
| **Continuous pose** (`C2`) | un-retired by 1.14, sized as cheap | **a hard prerequisite.** `sim/grid_world.py` is integer cells and a cardinal `Heading`; a SLAM pose cannot be represented in it, so the twin cannot show this phase working -- which §7 makes a blocker, not a nicety. `sim/renderer.py` already takes a float pose in radians |
| **A pose/odometry method** (`C1`) | one of three add-ons to the interface pass | the thing every phase here reads |
| **Encoders + IMU** | required by 1.14 item 6 under (a) too | **required and now doubly so** -- §3.3's table: nav2's local planner wants wheel odometry fused with scan matching, and scan matching alone is meaningfully worse |
| **`brain/planner.py`** (`C8`) | "the main hardware-path gap" (CLAUDE.md), designed, never written | the map is what finally gives it something to plan **over**. N7 extends C8's context; it does not build a second planner |
| **`sweep`** (§1.7) | deliberately omitted, no stop condition | implementable, gated on a measured trigger (N3) |

**One measured failure this is the fix for.** The 2026-09-02 `bearing-only` live
walk went degenerate -- 30 of 33 frames RIGHT -- and the reasoning showed why: it
found the bottle at frame 8, turned to centre it, overshot, lost it at frame 11,
and resumed spinning. CLAUDE.md's own reading is *"a search and memory failure,
not an obstacle failure -- deleting the obstacle question cannot help a model
that has lost the target and has no record of which way it already turned."*
A pose-anchored sighting is that record.

**One it is not the fix for.** P7e -- the walk that physically arrived and the
system did not notice -- is deliberately unbuilt because "the fix resolves
differently once a lidar exists." A lidar is about to exist. **P7e should be
re-read at N5, not re-litigated here**, and until it is repaired every tiered
walk still ends `max_steps` and `control/walk_eval.py`'s completion score stays
structurally zero for all of them.

---

## 4. The phases

Hardware-independent work first, per the simulation-first rule. §7's requirement
is a column, not an afterthought: **a phase is not done when its tests pass, it
is done when someone holding a phone can watch the thing it built do its job.**

| | What | Press this, in the twin | Hardware? |
|---|---|---|---|
| **N1** | **The map as a type, and the wall it lives behind.** An occupancy grid and a pose as validated data structures; `GET /map` and `GET /pose` on `robot/server.py`; `MockRobot` synthesising scans by raycasting `grid_world.py`'s walls (`sim/renderer.py:cast_ray` already does the geometry M2 reuses); `RemoteRobot` over both; conformance cases across all five backends. **The routing entries are part of this phase, not a follow-up** -- a CloudFront behaviour *and* an API Gateway route, two tables to keep in step, the failure that has shipped five times as a silently dead feature. Depends on `C1` and `C2` | A map view: the occupancy grid, the robot's pose on it, updating as the D-pad drives. Empty-map state reads "map is empty", never blank | no |
| **N2** | **The semantic layer, anchored.** `Sighting.position` becomes a real pose; rooms become labelled regions of the grid rather than per-frame guesses from `brain/rooms.py`; `MissionMemory` gains the join and **stays in RAM** (§1.5). `brain/rooms.py` is not deleted -- it becomes the *labeller* of a region, asked once per region instead of once per frame | Sightings drawn on N1's map where they were seen, with what was seen and when. Searched regions shaded. The label came from a photograph; the position did not | no |
| **N3** | **Coverage.** "Have I covered this room" as a computed number over N1's grid and N2's regions -- the question §1.7 says has no answer without a map. **Ships the answer, not the verb**: `sweep` lands only when the trigger log shows the planner reaching for it | A coverage percentage per region, and the unexplored frontier drawn. Drive into an unvisited corner and watch it fill | no |
| **N4** | **Persistence -- §1.5's ten decisions, built.** DynamoDB behind `control/`'s existing storage-abstraction pattern (`control/walk_store.py` is the precedent: one abstraction, two backends, one suite), a VPC endpoint (`network.yaml` has no NAT), house id from config, `schema_version` from the first write, per-edge last-confirmed and success/failure counts. **Stored map text is data, never instruction** -- a test pins that | Run a mission, stop it, restart the brain, start another. The second one begins with the first one's map. The panel names the house, the map's age and its schema version | no |
| **N5** | **The lidar, as a clearance ring -- §3.3's (a).** Driver, scans into Python, **sanity-checked in the actual house** (glass, mirrors, dark matte, mounting vibration), then wired to `robot/safety.py`. `get_depth_grid()` gains a 360 source, and **`fov_deg` stops being decorative**: §5.1's angular path selection exists precisely because "the middle half of the columns" is the entire forward hemisphere on a 360 ring. Re-read P7e here | M2/M3's depth strip under the FPV canvas, now fed by real scans, with the path zones outlined by angle. Drive at glass and watch what the ring does -- that readout is the validation | **yes** |
| **N6** | **ROS behind the wall.** `slam_toolbox` and the lidar's own `sllidar_ros2` driver as **one containerised service** exposing N1's three routes and nothing else. Swap what is behind the wall; change no consumer. Encoder odometry fused with scan matching; a URDF locating the lidar relative to the wheels; TF frames | **This phase changes nothing visible, on purpose, and that is its proof** -- §7 allows that answer once per phase, in writing. The readout that would show it if it broke is a **"map source: sim / slam"** line beside the existing "frame source: server / local" -- the picture should not change; where it comes from should | **yes** |
| **N7** | **The planner plans over the map.** An extension of `C8`, not a second planner: `MissionMemory.as_context()` gains coverage, frontiers and region adjacency, so the room-level decision is "the kitchen is 40% covered and adjacent" rather than a landmark guess. The planner stays a **pure function** (§1.5) | The Remote brain panel naming the region it chose and why, against the coverage number it read. The map redraws as the plan advances | no |

### Why this order

- **N1 first, because the wall is the whole design.** Defining `GET /map` and
  `GET /pose` against `MockRobot` -- before any ROS exists -- is the same move
  that made `RobotInterface` work: the consumers are written against the
  abstraction, and N6 swaps the implementation underneath them. If ROS arrives
  first, the wall gets drawn around whatever ROS happens to emit, and (b+)
  quietly becomes (c).
- **N2 and N3 before N4**, because persisting a schema you have not used is how
  you persist the wrong schema. `schema_version` from the first write (§1.5)
  makes that survivable, not free.
- **N4 before the lidar**, because it needs no hardware and it is the phase that
  delivers §3.4's actual promise -- the second trip is cheaper. It is also the
  one most likely to be skipped under hardware-day pressure and the one hardest
  to retrofit.
- **N5 before N6** -- §3.3's rule, restated: validate scans in the real house
  before layering SLAM on them.
- **N7 last**, because it is the only phase whose quality depends on all the
  others being real.

`N1`-`N4` can be built and watched today, with no purchase.

---

## 5. What this updates elsewhere

- **`PLAN-onboard-perception.md` §3.3** -- (b+)'s condition is met. Its "if
  mapping proves to be the point" should point here.
- **§1.7** -- `sweep`'s blocker is removed by N3; its gating rule is unchanged.
- **§1.6** -- unchanged, and reinforced: nothing here revives the visual edge.
- **§1.5** -- unchanged; N4 is its implementation.
- **`PLAN-sim-hardening.md` S6** -- the continuous-pose half, un-retired by 1.14,
  is now a hard prerequisite (C2). The Ackermann half stays retired.
- **P7e** -- to be re-read at N5, where the lidar it was deferred to exists.
- **The C-series** -- N1 depends on C1 and C2; N7 extends C8. Nothing here
  reorders them.

---

## 6. Open questions

1. **Does the planner get coordinates, or only regions?** §1.5 makes the planner
   a pure function over context; a metric map tempts one to hand it `(x, y)`.
   Region-level keeps the cloud tier semantic and the reactive tier geometric,
   which is 2.7's split. Provisional answer: **regions and frontiers, not
   coordinates** -- decide at N7 against a real trigger log.
2. **Where does `POST /goto` live?** §3.3 lists it as one of (b+)'s three routes,
   but a goal verb is `C4`'s vocabulary and `robot/safety.py` keeps the veto
   under (b+) -- nav2 plans *on top of* a safety layer, it does not replace one.
   Likely `traverse` gains a nav2 implementation rather than `goto` becoming a
   fifth verb. Settle at C4, not here.
3. **How does a region get its name?** N2 asks `brain/rooms.py` once per region
   instead of once per frame, which is cheaper and more stable -- but a region
   that spans two rooms has no right answer. Splitting on doorways is a lidar
   question and may fall out of the map for free.
4. **Does the map survive a camera-configuration change?** §1.5 scopes visual
   memories per camera configuration; a metric map should not care. Confirm it
   does not, rather than assuming.

---

## 7. Definition of done

Start a mission from a phone. The robot drives a house it has never seen,
searching for a named object, and the twin draws the map filling in as it goes
-- with the sightings where they were seen. Stop it. Start a second mission for
a different object **and watch it skip the rooms it already covered.**

That last clause is the whole plan. Everything before it is machinery.
