# Handoff 2026-10-06 -- SLAM drifts in the furnished home's open rooms (PLAN 3.39)

For a fresh session picking up the map-based search. Read
`docs/plans/PLAN-ros-alignment.md` sections **3.31** (frontier search),
**3.38** (SLAM false closures, closed) and **3.39** (this problem, open)
before acting. CLAUDE.md section 7 is the definition of done: criteria
written first, measured through the real path, recorded, pinned.

## 1. Where the work lives

- **Branch `frontier-search`**, worktree
  `/Users/anshugaind/vision-picar/.claude/worktrees/frontier-search`. Head
  is this handoff's commit. Pushed to `origin/frontier-search` up to
  `a4cb2a1`; everything after it (3.38's close-out, 3.39) is local only --
  push it when the user agrees. **Not merged into `dev`**, on purpose,
  until 3.31's batch is judged.
- It was taken over on 2026-10-05 from the session that built 3.31 (idle
  since 2026-10-02), at the user's direction, and **`dev` was merged into
  it** (`2beb769`): 3.33-3.37 (Jetson bring-up, the split simulator, the
  GC freeze), handoff 3a-3c. Its own SLAM section was renumbered
  **3.34 -> 3.38** (dev had used 3.34-3.37).
- `dev` / `origin/dev` is at `21d50da`. Local `dev` in the main checkout
  also has another session's unpushed commit `58eb9b0` (an Isaac ROS open
  question) -- not ours; leave it alone (memory: parallel sessions own
  their commits).

## 2. What is settled

- **3.38 closed, candidate D accepted** (`loop_search_space_dimension`
  8 -> 2 m in `service/slam/src/picar_bringup/config/slam.yaml`): no false
  loop-closure jumps (0/9 vs 3/3 originally); criterion 3 met (R5 laps
  2.9 / 3.8 cm with 3% drift; R6 scaled-house goals 6/6 twice); criterion 1
  recorded FAILED (4/6). A control tour showed D is not the cause of the
  home-tour failures.
- **3.39 criterion 1: the home tour's east-side goals were OFF nav2's
  costmap** -- at the start the global costmap is 7.8 x 9.4 m (what SLAM
  has seen); the kitchen, garage hall, laundry, garage and den are beyond
  it, so NavFn "failed to create plan" with the robot never moving. Not a
  narrow passage (that hypothesis was wrong). Fixed in the instrument:
  `tests/demo_slam_home.py --tour 1 --explore-first S` runs a 3.31
  `explore` mission (absent target) for up to S s first, and
  `tests/demo_nav_goals.run_goal` re-sends a goal nav2 `rejected` while
  activating (up to 30 s).
- **The user's direction (2026-10-05):** the furnished home
  (`sim/maps/home_first_floor.py`) is a realistic TEST house, not a replica
  -- "The rover will run in different homes." Fix unrealistic PROVISIONAL
  geometry rather than tuning the robot to it; do not ask the user to
  measure their house. (Memory: `sim_home_need_not_match.md`.)

## 3. The open problem (3.39 criterion 2 NOT met)

Two runs of `python -m tests.demo_slam_home 2 --tour 1 --explore-first 900`
on candidate D, on the Mac:

| run | explore (cap 900 s) | tour goals | goal end error | SLAM max / final |
|---|---|---|---|---|
| 1 | still running, coverage 94% | 6 ok, 2 timed out, 1 aborted | 0.08-0.88 m | 0.86 / 0.01 m |
| 2 | still running, coverage 76% | 2 ok, 2 timed out, 5 aborted | 0.16-12.6 m | 0.57 / 0.55 m |

**The finding:** through all 15 minutes of EXPLORING, SLAM stayed within
0.15 m. The error grew only during the TOUR, within ~20 s, while nav2
drove the long family-room -> kitchen route: run 1 0.16 -> 0.60 m at
(8.7, 2.1), 0.77 m in the kitchen; run 2 0.15 -> 0.45 m family room -> hall
-> kitchen; heading 3-5 deg off; **odometry exact throughout (0.01-0.03 m)**.
So scan matching pulls the pose away in the open family room / kitchen.
Not the den door.

**What "SLAM error" measures.** `/world/error` compares SLAM's pose (in
SLAM's map, converted to the house frame through the start anchor) with the
simulator's TRUE pose. It is a POSE error; it cannot tell a robot misplaced
on a good map from a map that itself bent or rotated in the open rooms --
both read the same, and both move house-frame goals. **Cheap first check:**
score SLAM's map against the house's true walls during/after a drifting run
(`tests/demo_slam_lap.score_map(R)` does this for R5; it scored 96-100% of
occupied cells within 10 cm there). A good map with a bad pose points at
scan matching / localisation; a warped map points at the map's own
construction (node spacing, closures).

**Not established -- the next session's first job.** Candidate causes, each
testable by one explore-then-tour run (~25 min each):

1. **D's narrower closure window** also limits how far a CORRECT closure
   can pull the pose back. Test: the same run on the ORIGINAL slam.yaml,
   `--slam evaluations/slam-339/slam-original.yaml` (the config before D,
   from `1e948f3`).
2. **Speed**: the tour drives faster and straighter than exploring. Test:
   one run with nav2's RPP `desired_linear_vel` lowered (nav2.yaml).
3. **Scan matching trusted over odometry** in an open room with repeated
   furniture legs: slam_toolbox's `correlation_search_space_dimension`,
   `minimum_travel_distance/heading`, the scan matcher's variance
   penalties. Only after 1 and 2 -- and judged with the 3% encoder drift
   too (`--drift 1.0,1.03`), because odometry is exact in the sim and not
   on the car.

The recommendation given to the user was: isolate (1) and (2) first
(~1 hour), then propose a targeted fix with criteria written first. The
user chose to continue in a fresh session; they have not yet picked
between isolating, tuning directly, or parking it as sim-specific.

## 4. Instruments and data

- `python -m tests.demo_slam_home N [--slam yaml] [--drift L,R] [--tour 1]
  [--explore-first S]` -- fresh stack per run on ports **8100/8101/8190**,
  container `picar-ros-explore`, `ROS_DOMAIN_ID=73`
  (`tests/demo_explore.stack`); writes `/tmp/slam_home_<t>.json` with the
  1 Hz series `(t, slam_err, head_err, odom_err, truth_x, truth_y)`.
- Raw records of the two runs above, and the diagnostic scripts used
  (copied from a session scratchpad that will not survive):
  `evaluations/slam-339/` -- `explore-tour-run{1,2}.json`,
  `costmap_diag.py` + `dump_costmap.py` (dump nav2's
  `/global_costmap/costmap` and `/map` from a running container),
  `tour_diag.py` (the tour with nav2's log kept and per-goal map size),
  `global-costmap-at-start.png`, and `slam-original.yaml` (the 8 m config
  before D, for control runs).
- To see where the error grows: load a run's `series`, and report each time
  `slam_err` crosses another 0.15 m with the truth position (the session
  did this inline; `sim.maps.home_first_floor.ROOMS` names the room).
- The ROS image on the Mac is built from this branch (bridge fixes + D):
  rebuild with `docker build -t vision-picar-ros service/slam` after any
  change under `service/slam/`. Docker Desktop must be running.

## 5. Other state worth knowing

- **Jetson** (`ssh picar-jetson`, 192.168.86.26): kept by the user; boots
  from the **NVMe** since 2026-10-05 (microSD untouched, second in
  BootOrder); idle at 15 W; checkout at `jetson-bringup`/`4064a99`. It can
  run SLAM batches: G4 passed there (3.33/3.36/3.37), so it is a valid
  place for long runs while the Mac stays free. Its live ports are the
  defaults (8000-8004, 8090), not the explore ports.
- **3.31's full batch** (`tests.demo_explore`, ~7 h) still waits on SLAM
  being trustworthy in the furnished home -- i.e. on this problem.
- **Soak test (S7)** is the other open item: per-program memory logging
  over an hour+, to explain a slow MemAvailable decline (-110 to -185 MB in
  20 min, on both disks).
- `tests/test_robot_contract.py::test_odometry_heading_...[hardware*]` is a
  known stock-firmware flake (3.25's turn scatter), ~1 in 4 on the laptop.
