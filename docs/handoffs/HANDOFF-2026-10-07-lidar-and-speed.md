# Handoff 2026-10-07 -- the lidar driver (3.42), and what it means for speed-scaled clearance

For the session making `min_distance_cm` scale with speed
(`PLAN-ros-alignment.md` 6.9, `PLAN-onboard-perception.md` 1.14 item 5).
The lidar driver below is merged on `dev` and changes the inputs your
stopping-distance work rests on. Read `PLAN-ros-alignment.md` **3.42** first.
CLAUDE.md section 7 is the definition of done.

## 0. Numbering -- check this first

**3.42 is taken on `dev`**: "The D500 lidar, read by the robot server"
(`f056e67`..`5faf3d3`, pushed 2026-10-06). If your speed-clearance section is
also numbered 3.42 locally, renumber it to the next free number (3.43, unless
something else has landed) before you push. 3.41 is the Isaac ROS evaluation;
3.38-3.40 are `frontier-search`'s SLAM sections.

## 1. What landed on `dev` (3.42)

* **Driver.** `robot/lidar_ld19.py` reads the Rover's D500 (LDROBOT STL-19P,
  LD19 protocol, 230,400 baud `[I]`) **in the robot server, not ROS** (the
  user's 2026-10-02 decision). It:
  * bins 1-degree beams in the body frame, nearest return;
  * applies the mounting yaw `LIDAR_YAW_DEG` -90 `[CAD]`;
  * reports the scan's age via `sensor_age_s()`, measured from its OLDEST
    point.
* **Wiring.** `HardwareRobot(lidar=)`; `ROBOT_LIDAR` on the car,
  `SIM_LIDAR=fake` in the simulator.
* **Fake.** `sim/fake_lidar.py` is the D500 on a pty, each point cast at its
  own instant, so a moving robot's scan is skewed as a real one is.
* **Three changes in `robot/safety.py`** that your work touches:
  1. **`path_clearance()` step 2c.** With the depth grid unusable and a
     usable scan, the cone reads the scan's beams within
     `PATH_HALF_ANGLE_DEG` of ahead, minus `LIDAR_TO_FRONT_BUMPER_CM`
     (8.65 cm).
     * Before this, a lidar-only car fell to the scalar `get_distance()`,
       which `HardwareRobot` answers 0.0, so FORWARD was refused forever.
     * This is the Rover until its OAK-D has a driver.
  2. **`pivot_blocked()` ages rotation.** `_turn_since_scan()` gives the
     turn the scan has missed:
     * preferably from the body's `turn_since_scan()`: `HardwareRobot`'s
       encoder heading history since the scan's oldest point;
     * otherwise yaw rate x age, which undercounts right after a turn
       slows.

     The check then runs from `PIVOT_AGE_SAMPLES` + 1 rotations across
     that arc. Without it, 105 of 120 lidar-timed pivots came within 1 cm;
     with it, 0.
  3. **Unchanged but now load-bearing on the car: `_aged()`.** It subtracts
     `v x sensor_age_s()` from every forward and reverse clearance (3.36).
     On the car the age is the lidar's.
* **Sweep harness.** `tests/footprint_sweep.py` has a lidar-timed mode,
  `sweep(..., lidar=True)` and `pivot_sweep(..., lidar=True)`, via
  `_LidarTimed`.
  * The vet reads the D500's scan through the real driver, cast from a
    MOVING robot, aged.
  * It runs with **no depth grid and no scalar sensor**, which is the car
    until its OAK-D has a driver.
  * Pinned by `tests/test_lidar_safety.py`.

## 2. What it means for speed-scaled clearance

Each item is something to design for, or a check to add to your criteria
before you measure.

1. **Do not count the lidar's delay twice.** `_aged()` already subtracts
   the travel since the scan.
   * If your stopping-distance formula also adds sensor latency, the robot
     stops further out than it needs to.
   * Keep the formula's reaction time to the control loop (50 ms period)
     and the actuator path, or remove `_aged()`'s share: choose one, and
     say which in your criteria.
2. **Use the measured scan age, not 100 ms.** In the lidar-timed sweep the
   scan was **150-200 ms old** when read: a 100 ms revolution, plus the
   time since it completed, plus the period.
   * At today's 0.2 m/s that is 3-4 cm.
   * At 0.5 m/s it is **7.5-10 cm** before any braking distance.
   * Your speed bands should be built on a 0.2 s worst case.
3. **The scan hint must grow with the clearance. This is a sim-only trap,
   and it errs toward UNSAFE.**
   * `SafetyController._scan()` asks for
     `max(SAFETY_SCAN_RANGE_M, half_length + 1.5 x min_distance_cm)`, about
     0.6 m today.
   * The simulator honours the hint and reports beams beyond it as None
     ("nothing there"); a real lidar ignores it.
   * If the required clearance grows past ~0.5 m ahead of the bumper while
     the hint does not, the **simulator hides obstacles the car would see**,
     and a sweep passes that should fail.
   * Make the hint a function of the speed-scaled clearance, and check it
     with a test.
4. **Judge on the lidar-timed sweep, not only the instant one.** 6.9's
   criterion (b) says "3.18's ground-truth sweep at the new top speed".
   * The default sweep reads a perfect instantaneous scan with a perfect
     depth grid.
   * Run `fs.sweep(..., lidar=True)` and `fs.pivot_sweep(..., lidar=True)`
     too: car-realistic timing, lidar only.
   * 3.42 measured these only at 0.1 m/s straight and 1 rad/s pivots.
     Faster is unmeasured, and a 10 Hz lidar is the likeliest thing to
     break at speed.
5. **Higher speed makes the scan time stamp worse.**
   * **Symptom.** 3.42 criterion 5 is borderline: the nav suite's
     smoothness bar (reversals per metre <= 1.0) failed 2 of 3 runs with
     the lidar on the laptop (1.03-1.04/m), and 2 of 2 passed without it.
   * **Suspected cause, not established.** The bridge stamps each scan it
     relays to ROS as current, but a real scan is 50-150 ms old. That is
     3.40's skew, ~7x larger.
   * **Proposed fix, not built (the user has not yet decided).** `/scan`
     carries the scan's age, and the bridge stamps `now - age`.
   * Worth settling before raising nav2's speed, because the same error in
     time is a bigger error in metres at speed.
6. **Where the change goes.** Replacing `self.min_distance_cm` in
   `vet_wheel_velocity()` and the two clearance comparisons carries through
   to step 2c and the pivot fix with no further change. Both end in that
   comparison or in the fixed `PIVOT_MARGIN_CM`.
7. **You will edit the same file.** `robot/safety.py` changed in 3.42.
   Rebase on `dev` (`5faf3d3` or later) before touching it.

## 3. How 3.42 was measured (reuse it)

* **Unit and sweep tests.**
  `.venv/bin/pytest tests/test_lidar_ld19.py tests/test_lidar_safety.py`
  (~2.5 min). Criteria 2 and 3b are strict xfails: failed as written,
  recorded.
* **Live stack on the laptop, beside other sessions.**
  * Robot :8200, brain :8201, body :8202, sensors :8203/8204, bridge :8290.
  * Container `picar-ros-lidar`, image tag `vision-picar-ros:lidar`,
    **`-e ROS_DOMAIN_ID=74`**.
  * **Trap:** two ROS containers on Docker Desktop's default network with
    the default domain cross their topics. The first run failed 8/13 with
    "the bridge has not heard /odom". An old `picar-ros` container
    (domain 0) was running on this Mac.
  * Point the live suites at it with `PICAR_ROBOT_URL`, `PICAR_BRAIN_URL`
    and `PICAR_BRIDGE_URL`. The nav suite also needs
    **`SIM_MAP=scaled_house` in the test's own environment**, or every test
    skips (a skip is not a pass).
  * The recipe: export `SIM_MAP=scaled_house ROBOT_MODE=hardware
    SIM_MOTOR_BOARD=fake SIM_BOARD_FIRMWARE=fork ROBOT_DRIVE=ros
    WORLD_MODE=ros SIM_LIDAR=fake` with the URLs above and
    `ROBOT_LIDAR=/tmp/picar-lidar-<n>`. Start `sim/body_server.py`, two
    `sim/sensor_server.py`, `python -m sim.fake_lidar --link $ROBOT_LIDAR`,
    then the robot and brain servers, then the container with
    `ROBOT_URL=http://host.docker.internal:8200`.
* **The Jetson.**
  * A separate worktree, `~/vp-342` (`dev`), shares the original
    checkout's `.venv` and `recordings` by symlink. The original
    `~/vision-picar` is untouched on `jetson-bringup`.
  * The image is built natively as `vision-picar-ros:lidar`; `latest` is
    untouched.
  * G4 recipe as `tools/jetson/README.md` section 4, plus `SIM_LIDAR=fake`.
    **Name the container `picar-ros`**: the fallback test kills it by that
    name.
  * Result: 18/18, 0 late ticks in 10,070 moving ticks.
  * The board is idle; nothing of 3.42's is left running.

## 4. Still open in 3.42 (not yours unless the user hands it over)

* Criterion 5's time stamp fix (section 2, item 5), for the user to decide.
* Criterion 7 on the car: real bytes decode, 10 Hz, a taped wall within
  2 cm, a box dead ahead at 0 +/- 2 degrees (verifies `LIDAR_YAW_DEG`).
