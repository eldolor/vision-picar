# Simulators, and 1,000 ProcTHOR houses (2026-10-11)

The record behind `PLAN-ros-alignment.md` 3.66 and the conversation that
led to it: why the simulator is our own, which outside simulators could
help, and what happened when ProcTHOR's floor plans were imported. The
shareable version, with four of the houses drawn, is the artifact
"A Thousand Houses"
(https://claude.ai/artifact/6B1jkgvTSrot9gPpW1uwGJ, private to its owner).
It continues section 8 of the learning document "ROS 2 for vision-picar"
(what a simulator can and cannot prove).

## Why this question

vision-picar is an indoor robot that searches a house for a named object.
It is built simulation-first: every phase is judged on numbers from the
simulator, then on the car. The user asked (2026-10-10) whether the
simulator is custom, whether a product like Gazebo would do better, and
whether any other simulator could be used.

## The simulator is custom, and Gazebo was turned down twice

`sim/` is a 2D grid house (0.30 m cells) with a raycaster. One geometry
gives the camera picture, collisions, the depth grid and the lidar. On top:
differential-drive kinematics, sensor noise, moving people and pets, and
fakes of the real ESP32 firmware and D500 lidar on serial ports. The ROS
stack drives it through the same robot-server route the car will use.
Gazebo was rejected in `docs/simulator/ARCHITECTURE.md` and
`PLAN-ros-alignment.md` section 1, for three reasons:

* **One source of truth.** Safety, SLAM tuning and explore are judged on
  this simulator's ground truth. A second simulator could disagree with
  it, and the gap would look like a bug.
* **Speed.** Phases close on sweeps of hundreds of missions in process.
  Gazebo runs about real time.
* **Platform.** Gazebo is poorly supported on macOS and would sit beside
  the robot server's path, not in it.

Known gaps of our own simulator: no physics (slip, braking, track scrub),
a flat world (everything fills the lidar's plane), and synthetic camera
images. These are better closed by measuring the Rover (due Oct 19 -
Nov 11) than by modelling them.

## Other simulators, by the gap each closes

| Simulator | Closes | Use here | Rank |
|---|---|---|---|
| ProcTHOR-10K (AI2, Apache-2.0) | "tuned to four houses" | its floor plans, converted into our grid | **1, built (3.66)** |
| Habitat-Sim + HM3D (Meta) | perception on realistic frames | an offline scorer for YOLOE + CLIP; never for safety. HM3D is non-commercial | 2 |
| MuJoCo | slip, braking | fit a slip model, after the Rover is measured | 3 |
| Webots | physics, lighter than Gazebo | same catch as MuJoCo | -- |
| A phone scan of the real house | the real target environment | most custom work | -- |
| nav2 loopback, Flatland, Stage | nothing we lack | cross-checks only | -- |
| Isaac Sim / Isaac Lab | everything | needs an RTX workstation | -- |

## 3.66: what was built

`SIM_MAP=procthor:val:<0-999>` loads any of the 1,000 houses of
ProcTHOR-10K's `val` split. Only the floor plan crosses over: rooms,
walls, doors, and what stands on the floor (bottom below 0.25 m). No
image, renderer or physics, so there is still one simulator.

* `tools/procthor_import.py` downloads the pinned sources, checks their
  sha256, and writes `sim/maps/procthor_data/val.json.gz` (450 kB). The
  suite never touches the network.
* `sim/maps/procthor.py` rasterises a house into the 0.30 m grid: walls a
  cell thick, door gaps cut, open-plan walls left out, the front door
  shut, tables as four legs. The target, a red backpack, is placed by a
  seed from the house index in a room other than the start's.

## Results (criteria written before building)

| # | Criterion | Bar | Measured | |
|---|---|---|---|---|
| 1 | room connections kept | >= 95% of 1,000 | 1000/1000 | met |
| 2 | outline within 5% of the source | >= 95% | 974/1000 (97.4%) | met |
| 3 | floor furniture placed | >= 98% | 22,254/22,288 (99.85%) | met |
| 4 | usable (Rover reaches every room), of houses with >= 3 rooms | >= 80% | 570/570 | met |
| 5a | explore sweep over the first 100 usable houses ends cleanly | 100% | 100/100 | met |
| 5b | ... finds the target | >= 90 | **84** | **failed** |
| 6 | offline, pinned, quick | < 0.5 s median | 6 ms | met |
| 7 | one live mission through the brain's HTTP API | an outcome | `max_steps`; `/health` names the house | met |

Two things were fixed on the way:

* **Furniture** first read 87.5%: ProcTHOR's walls have no thickness and
  its furniture stands flush against them, so a wall drawn a cell thick
  covered shallow pieces. They now take the nearest floor cell.
* **Door positions** could be read from either end of a wall. ProcTHOR
  keeps doorways clear, so furniture decided it: 17 doorways hold a piece
  under one reading, 465 under the other.

## The 16 misses are all explore's

None is an import fault: every target is on a free cell, reachable.

| Class | Houses | What happened |
|---|---|---|
| A. Seen only while panned | 8 (12, 44, 67, 75, 77, 136, 169, 173) | explore ignores a sighting from a panned frame, so nothing turned the body to look again |
| B. Sighted, never arrived | 4 (98, 101, 132, 165) | furniture left only the far side of the approach ring; the robot stopped beyond the 0.40 m arrival radius |
| C. Lidar ranged the wrong surface | 2 (0, 125) | the target at the frame's edge; the ray passed it and hit a wall 4.4 m away instead of the backpack at 2.7 m |
| D. Never in any frame | 2 (27, 108) | coverage counted the area as seen; not explained yet |

The sweep test pins 84, so a regression fails; the 90 bar stays as
written. Fixing A-D is policy work for its own section.

## Lessons that apply to any robot

| What happened | Why | Lesson |
|---|---|---|
| Furniture placement read 87.5% | the source's walls have no thickness; ours are a cell thick | moving data between two models, check what each assumes about size |
| Door positions could be read two ways | the format did not say which end of the wall they count from | find a test only the right reading passes (17 blocked doorways vs 465) |
| 16 searches failed in new houses | the four hand-drawn houses never set up those cases | a test set you designed yourself rarely surprises you |
| The search bar failed at 84 of 90 | real faults in the policy | keep the bar, pin the number, make the misses the next work |

## Core concepts

* **Kinematic vs physics simulation.** A kinematic simulator moves the
  robot exactly as commanded; a physics one computes forces and friction,
  and is only as good as friction numbers you usually lack until you
  measure the real robot.
* **The sim-to-real gap.** Every simulator is wrong somewhere. What matters
  is whether it is wrong in ways that change your decisions.
* **One simulator per question.** Two simulators judging the same question
  make every failure ambiguous: your code, or the gap between them.
* **Speed buys statistics.** Hundreds of seeded missions turn a phase into
  a number instead of an anecdote.
* **Diversity beats fidelity for generalisation.** Many rough layouts find
  more bugs than one perfect replica; four houses can be memorised.
* **Real-to-sim and system identification.** Build the simulator from
  reality (a scan; measured friction) rather than hope it resembles it.
* **Check an import against its own source.** Topology, area and object
  counts compared to ProcTHOR catch import bugs before they look like
  robot bugs.
* **Choose test cases by rule.** "The first 100 usable houses" cannot
  quietly favour easy ones.
* **A failed bar is still a result.** Recorded as failed, pinned at the
  measured value, with the misses classified into the next work.
