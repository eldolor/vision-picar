"""
mock_robot.py

Implements the exact function signatures that brain/ will call, backed by
GridWorld instead of GPIO/motors. This is the Phase 0 deliverable from the
build plan: the brain layer should never need to know whether it's talking
to this or to real hardware.

Speed/duration/angle are simulation-time-only concepts here -- they're
converted into wheel velocities and integrated. That conversion is the only
thing that will differ from the real hardware backend; the function
signatures and return shapes stay identical.

**Phase R0 (`PLAN-ros-alignment.md`) put real kinematics underneath them.**
`set_wheel_velocity()` / `step()` / `get_wheel_state()` are the primitive:
left and right wheel angular velocities in, an integrated continuous pose
and encoder positions out, with collision checked against the same
raycaster that draws the picture. The four discrete verbs are now thin
wrappers over that one path.

Wheel velocities rather than a `Twist`, and that choice is the phase: on the
real robot `diff_drive_controller` owns the twist-to-wheels conversion, and
a simulator that accepted a twist would leave that conversion -- and the two
chassis constants it is parameterised by -- unexercised until the day the
wheels are real. R4 plugs `hardware_interface::SystemInterface` into exactly
these three methods.

Two optional, opt-in behaviors, both Phase S4/S5 of
PLAN-sim-hardening.md and both off by default so the unit suite and every
existing demo stay exactly as fast and as exact as they always were:

- `realtime=True` makes `_settle()` actually sleep for a move's declared
  `duration` (config/robot.yaml's `sim.realtime`) -- see `_settle()`'s own
  docstring for why this matters and what it does not change.
- `sensor=` accepts a `sim.sensors.DistanceSensorModel` for noisy/lossy
  distance readings (config/robot.yaml's `sim.sensor_noise`) -- see that
  module's docstring for why `min_distance_cm` is otherwise never crossed
  at anything but exactly 0.0cm.
"""

import logging
import math
import time
from typing import Optional

from robot import interface
from sim.grid_world import (
    MAX_SUBSTEP_CELLS,
    MAX_SUBSTEP_RAD,
    ROBOT_HALF_CELL,
    GridWorld,
)
from sim.sensors import DistanceSensorModel
from sim import renderer
from robot.interface import RobotInterface

logger = logging.getLogger("mock_robot")

# Simulation conversion constants (tune freely; only affects sim realism)
CELLS_PER_SECOND_AT_FULL_SPEED = 2.0  # at speed=100
# `DEGREES_PER_TURN` is gone with R0: the verbs no longer round an angle
# up to a whole quarter-turn, they turn the angle they were given.
# What get_distance() has always multiplied a cell count by, and what
# sim/sensors.py defaults its own `cell_cm` to. Named here because
# get_depth_grid() needs the same number and two copies of a constant
# is how the grid and the scalar would start disagreeing about the same
# wall.
DEFAULT_CELL_CM = 30.0
DEFAULT_CELL_M = DEFAULT_CELL_CM / 100.0

# ---------- the chassis, phase R0 (PLAN-ros-alignment.md) ----------
#
# These are the numbers that will SHIP, read off `HARDWARE-BOM.md` 4.3
# (Yahboom L-type 520 motors on the differential chassis chosen in
# `PLAN-onboard-perception.md` 1.1), and that is the whole point of taking
# wheel velocities rather than a twist: `diff_drive_controller` will be
# configured with exactly these, so R4 puts its kinematics under test
# against parameters that have already been exercised here.
#
# Two are verified and one is not, and the difference is flagged rather
# than averaged away:
WHEEL_RADIUS_M = 0.0325  # 65mm rubber wheels [V]
ENCODER_COUNTS_PER_REV = 1760  # 11 lines x 40:1 gearbox, 4x quadrature [I]
# **PLACEHOLDER.** `HARDWARE-BOM.md` 4.3: "Track width, deck dimensions and
# payload are unpublished: measure on the chassis", and its bring-up item 4
# says to set it then. 0.172m is Waveshare's own firmware default -- the
# right shape and the wrong robot. It scales pivot rate only (a straight
# line does not depend on it), so a wrong value here makes the sim turn at
# the wrong speed and never in the wrong direction.
TRACK_WIDTH_M = 0.172

# What speed=100 means at the wheel. Derived from the cell rate the verbs
# have always used rather than from the motor's datasheet rpm, so that
# `drive_forward(100, 1.0)` still covers the same ground it did before R0 --
# a change in how far a command travels would have silently re-tuned every
# recorded demo and every step budget in the suite. The implied 176 rpm sits
# between the motor's rated 150 and no-load 300 (4.3), so it is also a
# number the real part can actually produce.
# The RPLidar C1's rated range (HARDWARE-BOM.md); the sim's scan casts this far.
LIDAR_RANGE_M = 12.0

WHEEL_MAX_RAD_S = (
    CELLS_PER_SECOND_AT_FULL_SPEED * DEFAULT_CELL_M / WHEEL_RADIUS_M
)

# There used to be a `_HEADING_DEG` table here, so `get_odometry()` could
# report an angle rather than a name. **Gone as of R0**: the heading is a
# float on the world now, `GridWorld.heading_deg` is the one place the
# theta-to-compass conversion happens, and a lookup table keyed by cardinal
# could not have expressed 45 degrees anyway.
# `ROBOT_HALF_CELL` is imported from sim/grid_world.py as of R0 -- the
# footprint is a fact about the thing in the world, and the mover
# (`GridWorld.translate()`) and this reported clearance must agree about it.
# Re-exported here because `get_depth_grid()` below reads it and a reader
# looking for the constant will look in this file.


class MockRobot(RobotInterface):
    """Drop-in backend for robot/ during the simulation-only phase."""

    def __init__(
        self,
        world: GridWorld,
        realtime: bool = False,
        sensor: Optional[DistanceSensorModel] = None,
        render: bool = True,
        render_width: int = renderer.DEFAULT_WIDTH,
        render_height: int = renderer.DEFAULT_HEIGHT,
        encoder_scale: tuple = (1.0, 1.0),
    ):
        self.world = world
        self.realtime = realtime
        # R5's opt-in odometry drift (`sim.odom_drift`): what each ENCODER
        # reports, as a multiple of what its wheel actually turned. The robot
        # moves truly; only the report is wrong -- exactly what a wheel whose
        # radius was measured 3% short does on hardware, and what makes
        # diff_drive_controller's odometry drift for SLAM to correct.
        # (1.0, 1.0) is exact, and the default.
        self.encoder_scale = (float(encoder_scale[0]), float(encoder_scale[1]))
        # Path length, in METRES, actually covered -- see get_odometry().
        # Counted here rather than read off the world because the world
        # knows only where the robot IS, and odometry is about where it
        # has BEEN. Metres rather than the cells this used to hold because
        # R0 integrates a wheel velocity and a wheel radius is in metres;
        # a cell count would have to be un-rounded to get back here.
        self._path_m = 0.0
        # Phase R0: the standing wheel command, and the integrated wheel
        # positions the encoders are read off. Two separate things on
        # purpose -- `ros2_control` writes the first and reads the second,
        # once per control cycle, and conflating them is how a controller
        # ends up reading back its own setpoint instead of the robot.
        self._cmd_left_rad_s = 0.0
        self._cmd_right_rad_s = 0.0
        self._left_rad = 0.0
        self._right_rad = 0.0
        # Phase S2. On by default because the pixels are now part of the
        # RobotInterface contract -- the conformance suite asserts every
        # backend returns a decodable image. `render=False` is the
        # free/offline path: no raycast, no JPEG encode, and
        # get_camera_frame() collapses back to frame_description(). Worth
        # having because a render is ~7ms and the rule-based policy, which
        # is most of the suite, has no use for the picture.
        self.render = render
        self.render_width = render_width
        self.render_height = render_height
        # None (the default) keeps get_distance()'s exact, noiseless,
        # always-30cm-multiple behavior every existing test depends on --
        # see sim/sensors.py's own docstring for why that is a real gap,
        # not just a simplification, and why closing it is opt-in.
        self.sensor = sensor

    # ---------- wheels: the primitive everything else is built on (R0) ----------
    #
    # `PLAN-ros-alignment.md` R0. Left/right wheel angular velocities rather
    # than a `Twist`, deliberately: `diff_drive_controller` owns the
    # twist-to-wheels conversion on the real robot, so a simulator that took
    # a twist would leave exactly that conversion -- and the two chassis
    # constants it is parameterised by -- untested until hardware day. The
    # seam R4 plugs into is `hardware_interface::SystemInterface`, whose
    # `write()` sets wheel velocity commands and whose `read()` returns wheel
    # positions, which is the shape of the three methods below.
    #
    # These are NOT on `RobotInterface`, and not yet. R2 is where `/wheels`
    # becomes a route and where promoting them to the interface (with an
    # honest "this backend has no wheels to report" default, as
    # `get_depth_grid()` and `get_odometry()` both carry) belongs. Until a
    # consumer exists, adding an abstraction is adding a second thing to
    # keep in step.

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        """Command both wheels, in rad/s. Positive is forward on both.

        Standing command: it persists until changed, exactly as a motor
        driver does, and `step()` is what makes time pass. That is why
        `stop()` has to zero it -- a stop that only halted the current move
        would leave the robot rolling on the next tick.
        """
        self._cmd_left_rad_s = float(left_rad_s)
        self._cmd_right_rad_s = float(right_rad_s)
        return self.get_wheel_state()

    def step(self, dt: float) -> dict:
        """Integrate the standing wheel command for `dt` seconds.

        The whole of the continuous stack is this function. Exact
        differential-drive kinematics, sub-stepped so that a curved path is
        checked against the raycaster more than once:

            v     = (v_right + v_left) / 2          metres/second
            omega = (v_right - v_left) / track      radians/second, CCW+

        **The sign of `omega` is the one thing here worth reading twice.**
        `omega` above is REP-103's body yaw rate -- counter-clockwise
        positive, which is a turn to the robot's LEFT -- because that is what
        `diff_drive_controller` and every ROS message mean by it. The grid's
        `theta` increases CLOCKWISE (see `sim/grid_world.py`'s module
        docstring: y grows downward). So the conversion carries a minus sign,
        and it happens here and nowhere else.

        Encoder positions are advanced from what the body ACHIEVED, not from
        what was commanded, which is the same choice `get_odometry()` has
        always made about a blocked move: an encoder measures the wheel, and
        a wheel that is against a wall is not turning. **Slip is therefore
        not modelled at all** -- a real wheel spinning against a blocked
        chassis counts up and this one does not. That is `PLAN-ros-alignment`
        section 4's listed residue ("wheel slip magnitude ... the coefficient
        is not known"), left for R8 calibration rather than guessed at here.
        """
        if dt <= 0:
            return self.get_wheel_state()

        v_left = self._cmd_left_rad_s * WHEEL_RADIUS_M
        v_right = self._cmd_right_rad_s * WHEEL_RADIUS_M
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / TRACK_WIDTH_M

        # One sub-step per MAX_SUBSTEP_CELLS of travel and per
        # MAX_SUBSTEP_RAD of rotation, whichever is stricter.
        travel_cells = abs(v) * dt / DEFAULT_CELL_M
        substeps = max(
            1,
            math.ceil(travel_cells / MAX_SUBSTEP_CELLS) if travel_cells else 1,
            math.ceil(abs(omega) * dt / MAX_SUBSTEP_RAD) if omega else 1,
        )
        dt_i = dt / substeps

        moved_m = 0.0
        turned_rad = 0.0
        blocked = False
        for _ in range(substeps):
            # Rotate first, then translate along the new heading -- the
            # standard explicit integration of a unicycle, and at these
            # sub-step sizes the difference from an exact arc is far below
            # the raycaster's own FPV_STEP resolution.
            d_theta_body = omega * dt_i
            self.world.rotate(-d_theta_body)  # CCW body -> theta decreasing
            turned_rad += d_theta_body

            want_cells = v * dt_i / DEFAULT_CELL_M
            got_cells = self.world.translate(want_cells)
            if abs(got_cells) < abs(want_cells) - 1e-12:
                blocked = True
            got_m = got_cells * DEFAULT_CELL_M
            moved_m += got_m
            self._path_m += abs(got_m)

            # Inverse kinematics, so the encoders describe the motion that
            # actually happened. This is the exact inverse of the forward
            # pair above, which is what makes a round trip through
            # `get_odometry()` consistent with `get_wheel_state()`.
            half = d_theta_body * TRACK_WIDTH_M / 2.0
            self._left_rad += (got_m - half) / WHEEL_RADIUS_M
            self._right_rad += (got_m + half) / WHEEL_RADIUS_M

        return {
            "moved_m": moved_m,
            "moved_cells": moved_m / DEFAULT_CELL_M,
            "turned_deg": math.degrees(turned_rad),
            "blocked": blocked,
            **self.get_wheel_state(),
        }

    def advance(self, dt: float) -> None:
        """The robot server's control loop lets time pass here (R2b): the
        simulator has no clock of its own, so a standing wheel command only
        moves the robot when someone integrates it."""
        self.step(dt)

    def get_wheel_state(self) -> dict:
        """Per-wheel position, velocity and encoder count.

        Positions in radians and velocities in rad/s because that is what
        `hardware_interface` exchanges; the counts are the same positions in
        the units the ESP32 will actually report (`HARDWARE-BOM.md` 4.3's
        1760 per revolution at 4x quadrature), so R7's fake board has
        something to serialise and R2 has something to publish.
        """
        per_rad = ENCODER_COUNTS_PER_REV / (2 * math.pi)
        # What the encoders REPORT -- see `encoder_scale` (R5's drift).
        left = self._left_rad * self.encoder_scale[0]
        right = self._right_rad * self.encoder_scale[1]
        return {
            "usable": True,
            "left": {"position_rad": left,
                     "velocity_rad_s": self._cmd_left_rad_s,
                     "counts": int(round(left * per_rad))},
            "right": {"position_rad": right,
                      "velocity_rad_s": self._cmd_right_rad_s,
                      "counts": int(round(right * per_rad))},
            "wheel_radius_m": WHEEL_RADIUS_M,
            "track_width_m": TRACK_WIDTH_M,
            "counts_per_rev": ENCODER_COUNTS_PER_REV,
        }

    def drive_wheels(self, left_rad_s: float, right_rad_s: float,
                     duration: float) -> dict:
        """Command both wheels and let `duration` elapse. Convenience, and
        the one the discrete verbs below are written in terms of -- so there
        is exactly one path from a command to a change in the pose."""
        self.set_wheel_velocity(left_rad_s, right_rad_s)
        result = self.step(duration)
        self._cmd_left_rad_s = self._cmd_right_rad_s = 0.0
        return result

    # ---------- driving ----------
    #
    # The discrete verb layer, unchanged in signature and in what it
    # commands, but now REALISED through the wheels above rather than by
    # teleporting a cell at a time. A verb still means what it meant: a
    # default `drive_forward()` covers one cell, and `turn_left()` a quarter
    # turn, because `/action` is a verb API and every step budget, demo and
    # recorded walk in this repo was measured against that. What changed is
    # that `turn_left(45)` now turns 45 degrees instead of rounding up to 90
    # -- which is the point of R0, and what makes P25's A/B runnable.

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        cells = self._speed_duration_to_cells(speed, duration)
        result = self._drive_cells(cells, speed)
        self._settle(duration)
        return {"action": "drive_forward", "speed": speed, "duration": duration,
                **result}

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        cells = self._speed_duration_to_cells(speed, duration)
        result = self._drive_cells(-cells, speed)
        self._settle(duration)
        return {"action": "reverse", "speed": speed, "duration": duration,
                **result}

    def turn_left(self, angle: int = 90) -> dict:
        return {"action": "turn_left", "angle": angle,
                **self._pivot(-float(angle))}

    def turn_right(self, angle: int = 90) -> dict:
        return {"action": "turn_right", "angle": angle,
                **self._pivot(float(angle))}

    def stop(self) -> dict:
        """Zero the standing wheel command, then say so.

        Before R0 this was a log line and nothing else, because there was no
        velocity to cancel -- a move was instantaneous. Now there is, and
        every failsafe in `control/` (B3.1-B3.3) depends on `stop()` really
        stopping: a robot with a standing command and a hung brain would
        keep integrating forward on the next tick.
        """
        self._cmd_left_rad_s = 0.0
        self._cmd_right_rad_s = 0.0
        self.world._record("STOP")
        return {"action": "stop"}

    # ---------- camera pan ----------

    def look_left(self) -> dict:
        return {"action": "look_left", **self.world.look_left()}

    def look_right(self) -> dict:
        return {"action": "look_right", **self.world.look_right()}

    def look_center(self) -> dict:
        return {"action": "look_center", **self.world.look_center()}

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        """Grid facts *and* real pixels -- phase S2.

        Until S2 this returned a structured description and nothing else,
        which is what stopped the vision policy driving the simulator at
        all (`PLAN-sim-hardening.md` 2.1 -- the one blocker that would
        have forced `RobotInterface` to change when hardware landed).
        `sim/renderer.py` now supplies `image_base64`/`media_type` in the
        same shape `ReplayRobot` and `TeleopRobot` already return, so all
        four backends answer this call the same way.

        **There are no grid facts here any more** (`PLAN-ros-alignment.md`).
        `position`, `facing`, `free_space_cells` and `doorway_ahead` were
        removed with the cell layer: the frontier policy reads its pose from
        `WorldInterface` and its clearance from `get_depth_grid()`, the two
        places a real robot gets them. What remains beside the pixels is
        `room` and `objects_visible` -- simulated PERCEPTION, standing in
        for the detector that 1.12 forbids running on a render. See
        `GridWorld.frame_description()`.

        `frame_description()` remains the free/offline path: no render and
        no JPEG encode, the same dict minus the three keys added here.
        """
        frame = self.world.frame_description()
        if not self.render:
            return frame
        # Added *after* frame_description(), which logs the dict it built.
        # A base64 JPEG in the log would bury every other line in it.
        frame["image_base64"] = renderer.render_world_base64(
            self.world, self.render_width, self.render_height
        )
        frame["media_type"] = renderer.MEDIA_TYPE
        frame["metadata"] = {
            "source": "sim",
            "render": {"w": self.render_width, "h": self.render_height},
        }
        return frame

    def get_depth_grid(self) -> dict:
        """Depth zones across the field of view, phase M2 -- cast from the
        same raycaster that draws the camera frame.

        Two things make this worth having before any sensor is bought.
        It is the seam M1 argued for (`PLAN-microduck-transplants.md`
        section 2: the camera answers *which way*, a sensor answers *how
        far*), and it is exercisable here: the grid world already knows
        where its walls are, so the sim can stand in for a ToF while the
        consumer that will read it (`robot/safety.py`, M3) is written.

        **`rows: 1`, and that is the honest number.** `cast_ray()` has no
        elevation and the grid world has no floor or ceiling geometry, so
        there is no second row to report. Publishing eight identical copies
        of one row would look like a matrix and be a fiction; the shape
        travels in the data precisely so this can be said out loud.

        Three details behind the numbers:

        * **The view heading, not the body heading**, matching
          `renderer.render_world_image()`. `look_left()` swings the strip
          the same way it swings the picture -- which is the point of a
          peek, and what `MissionAgent.decide()` already assumes about
          `get_distance()`.
        * **Zone rays are the centres of `cols` equal slices of the FOV**,
          so the strip lines up column-for-column with the frame above it.
          With an even `cols` no ray points exactly ahead; the two centre
          zones straddle the axis by half a slice, which is what M3's
          "reduce the centre zones to one scalar" is for.
        * **Half a cell and one ray-march step are subtracted**, so a zone
          reports clearance ahead of the robot's own cell and never more
          than it has. `cast_ray()` measures from the cell *centre* to the
          wall face, and overshoots by up to `FPV_STEP`; `get_distance()`
          counts free cells ahead of the robot. Left uncorrected the grid
          would read ~16cm further than the scalar on the same wall -- a
          silent disagreement between the two numbers M3's veto has to
          choose between, and this project has already paid once for a
          threshold sitting half a cell from where it was assumed to be
          (`min_distance_cm: 30.0`, M1). Corrected, the centre zones and
          `get_distance()` agree exactly on an axis-aligned wall, which is
          the property M3 needs and a test below pins.

        A ray that reaches `FPV_MAX_DIST` without meeting a wall is
        `ZONE_NO_TARGET` -- nothing within range, which is information, and
        deliberately not the same answer as `ZONE_UNUSABLE`.

        **`ZONE_UNUSABLE` comes from `sim.sensor_noise`'s dropout, per zone
        (phase M3).** With no sensor model configured -- the default -- the
        grid is exact and no zone is ever unusable, which is the same
        promise `get_distance()` makes. With one configured, each zone
        draws its own dropout and its own noise, because the zones of a
        real sensor fail independently. That is what gives the tri-state
        something to distinguish in the sim rather than only on hardware,
        and what lets `robot/safety.py` show the property M3 is for: one
        blind zone is not a blind robot.
        """
        cols = interface.DEPTH_COLS_DEFAULT
        cell_cm = self.sensor.cell_cm if self.sensor else DEFAULT_CELL_CM
        base_angle = self.world.view_angle()
        px, py = self.world.x, self.world.y

        zones = []
        for i in range(cols):
            t = (i + 0.5) / cols
            angle = base_angle - renderer.FPV_FOV / 2 + renderer.FPV_FOV * t
            dist_cells = renderer.cast_ray(self.world.layout, px, py, angle,
                                           solid=self.world.solid_cells)
            beyond_range = dist_cells >= renderer.FPV_MAX_DIST
            # cast_ray() overshoots: it marches in FPV_STEP increments and
            # returns the first step already inside the wall, so the true
            # crossing lies in (dist - FPV_STEP, dist]. Report the lower
            # end. Overstating clearance in a number a safety veto will
            # read is the one direction this must not round.
            free_cells = dist_cells - renderer.FPV_STEP - ROBOT_HALF_CELL
            clearance = max(0.0, free_cells) * cell_cm

            if self.sensor is None:
                # Exact, as everywhere else in this backend by default.
                zones.append(
                    {"status": interface.ZONE_NO_TARGET, "distance_cm": None}
                    if beyond_range else
                    {"status": interface.ZONE_RANGE, "distance_cm": round(clearance, 1)}
                )
                continue

            # Phase M3. Each zone draws its own dropout, exactly as the
            # zones of a real VL53L5CX fail independently -- one zone's
            # status byte says nothing about its neighbour's. This is what
            # makes the grid degrade where the scalar cannot: a dropped
            # beam blinds get_distance() completely, while a dropped zone
            # leaves seven others to answer with.
            reading = self.sensor.read_zone_cm(clearance)
            if reading is None:
                zones.append({"status": interface.ZONE_UNUSABLE, "distance_cm": None})
            elif beyond_range:
                zones.append({"status": interface.ZONE_NO_TARGET, "distance_cm": None})
            else:
                zones.append({"status": interface.ZONE_RANGE, "distance_cm": reading})

        self.world._record(
            f"DEPTH view_deg={math.degrees(base_angle + math.pi / 2) % 360:.1f} "
            f"cols={cols}")
        # `fov_deg` is the render's own field of view, and it must be: the
        # zones above are cast on `renderer.FPV_FOV`, so publishing anything
        # else would point `robot/safety.py`'s path cone somewhere the rays
        # never went.
        # `pan_deg`: where the grid points relative to the body (3.18 part
        # 2). It is cast along the camera, so a peek swings it -- and the
        # safety layer must know, or it reads a side wall as the way ahead.
        pan_deg = math.degrees(renderer.normalize_angle(base_angle - self.world.theta))
        return {"rows": 1, "cols": cols, "pan_deg": round(pan_deg, 4),
                "fov_deg": math.degrees(renderer.FPV_FOV), "zones": zones}

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        """A 360-degree scan cast from the robot's own position -- R2.

        One beam per degree off the BODY heading, clockwise-positive with 0
        dead ahead: the lidar is on the deck and does not pan, so this uses
        `theta`, never `view_angle()` -- the opposite choice from
        `get_depth_grid()`, which is the camera's field. Ranges come from
        `renderer.cast_ray()`, the geometry the picture is drawn from and
        the ring `MockWorld` builds its map out of, measured from the
        robot's centre (the sim's robot is a point; a real lidar sits
        11-14cm behind the bumper, which R3 makes a transform).

        **`range_max_m` is the RPLidar C1's 12 m** (`LIDAR_RANGE_M`). It
        was the camera renderer's 4.2 m horizon until a house at real size
        exposed the gap: from the user's own foyer SLAM mapped almost nothing
        and nav2 refused every goal as "off the global costmap". A beam that
        reaches the range has no return and reads None, which is information
        about empty space. **`max_range_m` is honoured** (3.18): beams stop
        there and read None beyond it, while `range_max_m` still states the
        sensor's own 12 m -- the hint trims this call, not the sensor. The
        safety layer asks for ~0.6 m; SLAM asks for everything. A hinted
        beam is also cast exactly rather than by the march (never further,
        see `renderer.cast_ray_exact()`). The first-step
        overshoot is reported at its upper bound here, unlike the depth
        grid: a scan feeds a MAP, which wants the wall where it is, not the
        safety veto, which wants it where it might be.
        """
        rays = 360
        reach_m = LIDAR_RANGE_M if max_range_m is None else min(LIDAR_RANGE_M, max_range_m)
        max_cells = reach_m / DEFAULT_CELL_M
        # A range-hinted scan is the safety layer's, and is cast EXACTLY
        # (`renderer.cast_ray_exact()`): never further than the march, and
        # ~5x cheaper, which matters at one scan per control period. The full
        # scan -- SLAM's -- keeps the march its map is pinned to.
        cast = renderer.cast_ray if max_range_m is None else renderer.cast_ray_exact
        solid = self.world.solid_cells
        ranges = []
        for i in range(rays):
            rel = math.radians(-180 + i)
            dist = cast(self.world.layout, self.world.x, self.world.y,
                        self.world.theta + rel, solid=solid, max_dist=max_cells)
            ranges.append(None if dist >= max_cells
                          else round(dist * DEFAULT_CELL_M, 4))
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.0, "range_max_m": LIDAR_RANGE_M,
                "ranges_m": ranges}

    def get_odometry(self) -> dict:
        """Real odometry, because the grid world knows where it put us.

        Path length rather than displacement, per the interface: every
        metre actually covered adds to it, including a reverse, and a move
        that was BLOCKED adds nothing because the robot did not go
        anywhere. `step()` accumulates what `GridWorld.translate()` really
        managed, so this is a sum of truths rather than of intentions --
        which is what an encoder measures and a commanded distance is not.

        **As of R0 it is integrated from the wheels rather than counted in
        cells**, and that is the point rather than a refactor: a cell count
        could only ever report multiples of 30cm, so a policy pacing itself
        on distance travelled (`brain/tiered.py`'s cold-search interval)
        could not see a 4cm nudge at all. `get_wheel_state()` publishes the
        same motion per wheel, and the two agree by construction -- one is
        the forward kinematics of the other.

        Heading is the body heading, never the view heading: a pan changes
        what the camera sees and moves no wheels. `get_depth_grid()` casts
        off the VIEW angle and this reports the BODY one, and the two
        differing is correct rather than an inconsistency. Continuous now,
        so a 45-degree turn reports 45 degrees.
        """
        return {
            "usable": True,
            "distance_m": round(self._path_m, 4),
            "heading_deg": round(self.world.heading_deg, 4),
        }

    def get_distance(self) -> float:
        """Distance in cm, matching the real ultrasonic sensor's units.
        Grid cells are treated as ~30cm for rough realism.

        With no sensor model configured (the default), this is exactly
        `cells * 30.0` -- unchanged from before Phase S5, and always an
        exact multiple of 30. With one configured, the reading goes
        through `DistanceSensorModel.read()` instead -- see that class's
        docstring for the noise/dropout/range/latency it adds."""
        cells = self.world.distance_ahead()
        if self.sensor is None:
            return round(cells * 30.0, 1)
        if self.realtime and self.sensor.read_latency_s:
            time.sleep(self.sensor.read_latency_s)
        return self.sensor.read(cells)

    # ---------- internal ----------

    def _speed_duration_to_cells(self, speed: int, duration: float) -> int:
        speed = max(0, min(100, speed))
        cells = (speed / 100.0) * CELLS_PER_SECOND_AT_FULL_SPEED * duration
        return max(1, round(cells)) if speed > 0 and duration > 0 else 0

    def _drive_cells(self, cells: float, speed: int) -> dict:
        """Realise a straight-line move of `cells` cells through the wheels.

        Both wheels at the same velocity, for as long as that velocity needs
        to cover the distance. The integration time is therefore NOT the
        caller's `duration` -- a verb's `duration` is quantised into a whole
        number of cells first (`_speed_duration_to_cells()`, unchanged since
        Phase 0), so the two were already only loosely related. `_settle()`
        still sleeps the declared `duration`, because that is what S4's
        watchdog readout is measured against.
        """
        if cells == 0:
            return {"requested": 0, "moved": 0.0}
        w = max(1, min(100, speed)) / 100.0 * WHEEL_MAX_RAD_S
        sign = 1.0 if cells > 0 else -1.0
        dt = abs(cells) * DEFAULT_CELL_M / (w * WHEEL_RADIUS_M)
        moved_cells = self.drive_wheels(sign * w, sign * w, dt)["moved_cells"]
        self.world._record(
            f"MOVE requested={cells} moved={moved_cells:.3f} "
            f"pos=({self.world.x:.2f},{self.world.y:.2f}) "
            f"cell=({self.world.robot_x},{self.world.robot_y}) "
            f"heading_deg={self.world.heading_deg:.1f}"
        )
        # No `position` in the ack any more. It was a grid cell, and a
        # motor driver cannot report one -- the body's own account of how
        # far it went is `get_odometry()`, and where it ended up is the
        # world's to say (`get_pose()`).
        return {"requested": cells, "moved": moved_cells}

    def _pivot(self, degrees: float) -> dict:
        """Turn in place by `degrees` -- positive to the robot's right --
        through counter-rotating wheels.

        A pivot rather than an arc, which is correct for the differential
        chassis chosen in `PLAN-onboard-perception.md` 1.1 and is the whole
        reason S6's Ackermann half could be retired. Zero net travel, so
        `get_odometry()`'s `distance_m` does not move -- a property
        `tests/test_robot_contract.py` pins directly.
        """
        if degrees == 0:
            return {"heading": self.world.heading.name,
                    "heading_deg": self.world.heading_deg}
        w = WHEEL_MAX_RAD_S
        # omega_body = (v_right - v_left) / track, with v_left = -v_right.
        # At full wheel speed on the placeholder track width that is ~400
        # deg/s, so a quarter-turn verb occupies ~0.23s. Faster than the real
        # chassis will pivot, and deliberately not tuned here: the verb layer
        # is instantaneous as far as every existing caller is concerned
        # (`_settle()` sleeps the declared `duration`, not this), and picking
        # a turn rate is a calibration question for a measured track width
        # rather than a guess to bake in now.
        omega = 2 * w * WHEEL_RADIUS_M / TRACK_WIDTH_M
        dt = math.radians(abs(degrees)) / omega
        # theta increases to the RIGHT; the body turns left (omega positive,
        # right wheel forward) when theta should decrease.
        sign = -1.0 if degrees > 0 else 1.0
        self.drive_wheels(-sign * w, sign * w, dt)
        self.world._record(
            f"TURN {degrees:+.1f}deg heading={self.world.heading.name} "
            f"heading_deg={self.world.heading_deg:.1f}"
        )
        return {"heading": self.world.heading.name,
                "heading_deg": self.world.heading_deg}

    def _settle(self, duration: float):
        """No-op unless self.realtime is set (config/robot.yaml's
        sim.realtime, Phase S4). Before this, PLAN-sim-hardening.md
        section 3.1 was literally true: "the watchdog's async loop is
        never executed by any test" -- there was no way for an action to
        occupy any wall-clock time at all, sim or live. Enabling this
        makes a move actually take its declared duration, which is what
        lets robot/server.py's watchdog readout climb mid-move instead of
        only between moves, and what stops speed/duration from being
        decorative. Off by default so the unit suite (and every
        tests/demo_*.py script) stays exactly as fast as it always was --
        this is a live/manual-testing and integration-test knob, not
        something the automated suite should pay for by default."""
        if self.realtime and duration > 0:
            time.sleep(duration)
