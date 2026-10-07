"""
sim/fake_lidar.py

`PLAN-ros-alignment.md` 3.42 -- the Rover's D500 lidar, faked: LDROBOT's
LD19 packets (`robot/lidar_ld19.py`) on a pseudo-terminal, cast from the
simulated body, so the robot server reads the simulator through the SAME
driver the car will use, as `sim/fake_esp32.py` does for the motor board.

**Each point is cast at its own instant.** A real lidar sweeps 360 degrees
in 100 ms while the robot moves, so a moving robot's scan is skewed -- the
front of it was taken from somewhere the robot no longer is. Casting a
whole scan from one pose would hide that, and it is the one thing about a
real lidar's timing the safety layer has never been tested against
(3.42 criterion 4).

    The emitter            `Emitter`: a clock and a `cast_mm(angle, t)` in,
                           packet bytes out, at the device's rate. Shared
                           with `tests/footprint_sweep.py`'s lidar run.
    The program            `python -m sim.fake_lidar --link /tmp/picar-lidar`:
                           reads the body state `sim/body_server.py` publishes
                           (3.36), writes packets to a pty, and points `--link`
                           at it -- the udev symlink's role on the car.
                           `service/tunnel/run.sh` starts it under SIM_LIDAR=fake.

Geometry is `renderer.cast_ray_exact()` from the lidar's own position,
`LIDAR_X_M` ahead of the rotation centre -- the exact traversal, never
further than the truth (3.18), at ~2 microseconds a ray. The mounting yaw
is the driver's `LIDAR_YAW_DEG` in reverse: the fake emits the lidar's OWN
angles, so a sign error in the driver shows up as a scan rotated by twice
the yaw.
"""

import argparse
import math
import os
import random
import struct
import time
import tty
from typing import Callable, List, Optional

from robot.lidar_ld19 import (LIDAR_YAW_DEG, POINTS_PER_PACKET, RANGE_MAX_M,
                              Packet, encode)

RATE_HZ = 10.0              # the D500's nominal scan rate
POINTS_PER_S = 4500.0       # LD19-class: 450 points a revolution at 10 Hz


class Emitter:
    """Packets for every point whose time has come.

    `cast_mm(lidar_angle_deg, t)` returns the distance in mm the lidar
    measures at its own angle at time `t`, or 0 for no return. Point k is
    taken at `t0 + k / POINTS_PER_S` at lidar angle `k * 360 * RATE_HZ /
    POINTS_PER_S`; a packet leaves when its twelfth point has been taken."""

    def __init__(self, cast_mm: Callable[[float, float], int], t0: float = 0.0,
                 rate_hz: float = RATE_HZ, points_per_s: float = POINTS_PER_S):
        self.cast_mm = cast_mm
        self.t0 = t0
        self.rate_hz = rate_hz
        self.points_per_s = points_per_s
        self.deg_per_point = 360.0 * rate_hz / points_per_s
        self.k = 0

    def due(self, t: float) -> List[bytes]:
        return [pkt for _t, pkt in self.due_timed(t)]

    def due_timed(self, t: float) -> List[tuple]:
        """`due()` with each packet's send time -- its last point's instant --
        so an in-process reader can stamp arrivals as a serial line would."""
        out = []
        while self.t0 + (self.k + POINTS_PER_PACKET - 1) / self.points_per_s <= t:
            dists = []
            for j in range(POINTS_PER_PACKET):
                kk = self.k + j
                angle = (kk * self.deg_per_point) % 360.0
                dists.append(int(self.cast_mm(angle, self.t0 + kk / self.points_per_s)))
            start = (self.k * self.deg_per_point) % 360.0
            end = ((self.k + POINTS_PER_PACKET - 1) * self.deg_per_point) % 360.0
            t_last = self.t0 + (self.k + POINTS_PER_PACKET - 1) / self.points_per_s
            out.append((t_last, encode(Packet(
                speed_dps=int(round(360.0 * self.rate_hz)),
                start_cdeg=int(round(start * 100)) % 36000,
                end_cdeg=int(round(end * 100)) % 36000,
                distances_mm=tuple(dists),
                intensities=tuple(200 if d else 0 for d in dists),
                timestamp_ms=int(t_last * 1000) % 30000))))
            self.k += POINTS_PER_PACKET
        return out


def world_caster(world, yaw_deg: float = LIDAR_YAW_DEG,
                 pose_at: Optional[Callable[[float], tuple]] = None):
    """`cast_mm` over a `GridWorld`: the beam leaves the lidar's position
    along body bearing `angle + yaw` (clockwise positive, as the driver
    reads it). `pose_at(t)` -> (x, y, theta) in cells / radians; default is
    the world's pose now."""
    from robot.safety import LIDAR_X_M
    from sim import renderer
    from sim.mock_robot import DEFAULT_CELL_M

    max_cells = RANGE_MAX_M / DEFAULT_CELL_M
    off = LIDAR_X_M / DEFAULT_CELL_M

    def cast_mm(angle_deg: float, t: float) -> int:
        x, y, theta = pose_at(t) if pose_at else (world.x, world.y, world.theta)
        ox, oy = x + off * math.cos(theta), y + off * math.sin(theta)
        ray = theta + math.radians(angle_deg + yaw_deg)
        d = renderer.cast_ray_exact(world.layout, ox, oy, ray, solid=world.solid_cells,
                                    max_dist=max_cells)
        return 0 if d >= max_cells else int(round(d * DEFAULT_CELL_M * 1000))

    return cast_mm


class LossyLine:
    """A UART that drops and corrupts bytes (3.42 criterion 3)."""

    def __init__(self, drop: float = 0.0, corrupt: float = 0.0, seed: int = 0):
        self.drop, self.corrupt = drop, corrupt
        self._rng = random.Random(seed)

    def __call__(self, data: bytes) -> bytes:
        if not (self.drop or self.corrupt):
            return data
        out = bytearray()
        for b in data:
            r = self._rng.random()
            if r < self.drop:
                continue
            if r < self.drop + self.corrupt:
                b ^= 1 << self._rng.randrange(8)
            out.append(b)
        return bytes(out)


# --------------------------------------------------------------------------
# The program
# --------------------------------------------------------------------------

def open_pty(link: Optional[str]) -> tuple:
    """(master fd, slave path). The master is non-blocking: a UART with no
    reader drops bytes rather than stalling the sender, and a blocking pty
    write was one of the harness lies 3.25 found."""
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if link:
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(path, link)
    return master, slave, path


def main(argv=None) -> int:
    from sim.body_state import DEFAULT_NAME, StateReader
    from sim.maps import build_world

    ap = argparse.ArgumentParser(description="The D500 lidar, faked on a pty (PLAN 3.42).")
    ap.add_argument("--link", default=os.environ.get("ROBOT_LIDAR", "/tmp/picar-lidar"))
    ap.add_argument("--shm", default=os.environ.get("SIM_BODY_SHM", DEFAULT_NAME))
    ap.add_argument("--drop", type=float, default=float(os.environ.get("SIM_LIDAR_DROP", 0)))
    ap.add_argument("--corrupt", type=float, default=float(os.environ.get("SIM_LIDAR_CORRUPT", 0)))
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    master, _slave, path = open_pty(args.link)
    print(f"fake D500 on {path} -> {args.link}", flush=True)
    reader = StateReader(args.shm)
    line = LossyLine(args.drop, args.corrupt, args.seed)
    house, world = None, None
    state = {}

    def pose_at(_t):
        return state["x"], state["y"], state["theta"]

    emitter = None
    try:
        while True:
            try:
                snap = reader.read()
            except (ValueError, struct.error):   # a torn read: try again next tick
                snap = None
            if snap is not None:
                if snap["house"] != house:
                    world, house = build_world(snap["house"]), snap["house"]
                world.objects = snap["objects"]
                state.update(snap)
                if emitter is None:
                    emitter = Emitter(world_caster(world, pose_at=pose_at),
                                      t0=time.monotonic())
            if emitter is not None:
                data = b"".join(emitter.due(time.monotonic()))
                if data:
                    try:
                        os.write(master, line(data))
                    except BlockingIOError:
                        pass            # nobody reading: the bytes are lost, as on a UART
            time.sleep(0.002)
    except KeyboardInterrupt:
        return 0
    finally:
        if args.link:
            try:
                os.unlink(args.link)
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
