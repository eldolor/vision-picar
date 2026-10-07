"""PLAN-ros-alignment.md 3.42 -- the D500 lidar read by the robot server.

Criteria 1-3 and 4b: the LD19 protocol, the scan through a real serial line
(a pty) against the simulator's exact geometry, liveness, and the forward
cone on a lidar-only car. Criterion 4's moving sweep is in
`tests/test_lidar_safety.py`."""

import math
import os
import threading
import time

import pytest

from robot.interface import unusable_grid
from robot.lidar_ld19 import (CRC_TABLE, LIDAR_YAW_DEG, PACKET_LEN, Assembler, bin_points,
                              Ld19Lidar, Ld19Scanner, Packet, Parser, bin_scan,
                              crc8, decode, encode)
from robot.safety import SafetyController
from sim import renderer
from sim.fake_lidar import Emitter, LossyLine, open_pty, world_caster
from sim.maps import build_world
from sim.mock_robot import DEFAULT_CELL_M, MockRobot


def _packet(start=0, end=880, dist=1000, ts=0):
    return Packet(speed_dps=3600, start_cdeg=start, end_cdeg=end,
                  distances_mm=(dist,) * 12, intensities=(200,) * 12, timestamp_ms=ts)


# ---------------- criterion 1: the protocol ----------------

def test_crc_table_is_ldrobots():
    # LDROBOT lipkg.cpp CrcTable: first and last entries, and every byte once.
    assert len(CRC_TABLE) == 256 and len(set(CRC_TABLE)) == 256
    assert CRC_TABLE[:4] == bytes((0x00, 0x4D, 0x9A, 0xD7))
    assert CRC_TABLE[-4:] == bytes((0x7F, 0x32, 0xE5, 0xA8))
    assert sum(CRC_TABLE) == sum(range(256))


def test_round_trip_and_layout():
    p = _packet(start=35900, end=880, dist=1234, ts=29999)
    raw = encode(p)
    assert len(raw) == PACKET_LEN and raw[:2] == b"\x54\x2c"
    assert raw[-1] == crc8(raw[:-1])
    assert decode(raw) == p


def test_a_changed_byte_fails_the_crc():
    raw = bytearray(encode(_packet()))
    raw[10] ^= 0x01
    assert decode(bytes(raw)) is None


def test_angles_spread_evenly_across_the_seam():
    a = _packet(start=35560, end=440).angles_deg()      # 355.6 -> 4.4, across 0
    assert a[0] == pytest.approx(355.6) and a[-1] == pytest.approx(4.4)
    steps = [((a[i + 1] - a[i]) % 360) for i in range(11)]
    assert all(s == pytest.approx(0.8) for s in steps)


def test_parser_resyncs_joins_split_reads_and_counts_bad_packets():
    good, other = encode(_packet(dist=500)), encode(_packet(dist=700))
    bad = bytearray(encode(_packet(dist=900)))
    bad[20] ^= 0xFF
    stream = b"\x00\x54\x99garbage\x54" + bytes(bad) + good + b"\x11\x54" + other
    p = Parser()
    out = []
    for i in range(0, len(stream), 7):          # arbitrary read boundaries
        out += p.feed(stream[i:i + 7])
    assert [pk.distances_mm[0] for pk in out] == [500, 700]
    assert p.crc_failed == 1


# ---------------- binning, direction and units ----------------

def _one_return(raw_angle, mm=1000, yaw=LIDAR_YAW_DEG):
    pts = [(k * 0.8 % 360, mm if abs(((k * 0.8 - raw_angle) + 180) % 360 - 180) < 0.5 else 0, 0.0)
           for k in range(450)]
    return bin_scan(pts, yaw)


def test_the_lidar_zero_faces_left_and_angles_are_clockwise():
    # Mounted turned 90 degrees left: the lidar's 0 is the body's -90 (left),
    # its 90 is dead ahead. A reversed direction would put it at -180.
    scan = _one_return(90.0)
    hits = [i - 180 for i, r in enumerate(scan["ranges_m"]) if r is not None]
    assert hits == [0]
    assert scan["ranges_m"][180] == pytest.approx(1.0)       # mm, not cm


def test_an_empty_bin_takes_its_nearer_neighbour_never_none():
    pts = [(a, 2000, 0.0) for a in range(360) if a % 30]     # 12 single holes
    scan = bin_scan(pts, yaw_deg=0.0)
    assert scan["usable"] and all(r == pytest.approx(2.0) for r in scan["ranges_m"])


def test_a_lost_packet_is_never_read_as_clear():
    # A 10-degree hole in one revolution, a wall at 1 m everywhere else: the
    # hole carries the previous revolution's 1 m, or failing that the
    # neighbours' -- never None ("nothing within range").
    full = [(k * 0.8 % 360, 1000, 0.0) for k in range(450)]
    holed = [p for p in full if not (100 <= p[0] < 110)]
    prev = bin_points(full, 0.0)
    for scan in (bin_scan(holed, 0.0, prev=prev), bin_scan(holed, 0.0)):
        assert all(r == pytest.approx(1.0) for r in scan["ranges_m"])


def test_too_much_unknown_is_not_a_scan():
    half = [(k * 0.8 % 360, 1000, 0.0) for k in range(450) if k * 0.8 % 360 < 180]
    assert not bin_scan(half, 0.0)["usable"]


def test_all_zero_points_mean_no_return():
    scan = bin_scan([(k * 0.8, 0, 0.0) for k in range(450)], yaw_deg=0.0)
    assert scan["usable"] and all(r is None for r in scan["ranges_m"])


def test_a_revolution_ends_at_the_wrap_and_the_first_is_dropped():
    a = Assembler()
    revs = a.add([(x, 1, 0.0) for x in (300, 350)])            # partial first
    revs += a.add([(x, 1, 0.0) for x in (10, 180, 350)])
    revs += a.add([(x, 1, 0.0) for x in (5,)])
    assert [len(r) for r in revs] == [3]


# ---------------- criterion 3: liveness (clock injected) ----------------

class _Clock:
    t = 0.0

    def __call__(self):
        return self.t


def _feed_revolutions(scanner, clock, n, cast=lambda a, t: 2000):
    em = Emitter(cast, t0=clock.t)
    for _ in range(int(n * 450 / 12) + 2):
        clock.t += 12 / 4500.0
        scanner.feed(b"".join(em.due(clock.t)), clock.t)


def test_silence_makes_the_scan_unusable_and_packets_bring_it_back():
    clock = _Clock()
    s = Ld19Scanner(clock=clock)
    _feed_revolutions(s, clock, 3)
    assert s.get_scan()["usable"] and s.sensor_age_s() is not None
    clock.t += 0.31
    assert not s.get_scan()["usable"] and s.sensor_age_s() is None
    _feed_revolutions(s, clock, 1.2)                  # resumes within one revolution + a bit
    assert s.get_scan()["usable"]


def test_age_is_since_the_oldest_point():
    clock = _Clock()
    s = Ld19Scanner(clock=clock)
    _feed_revolutions(s, clock, 3)
    assert 0.09 <= s.sensor_age_s() <= 0.2


# ---------------- criterion 2: fidelity through a serial line ----------------

POSES_PER_HOUSE = 10
HOUSES = ("starter_house", "scaled_house", "home_first_floor")


def _poses(house, n, seed=0):
    """Legal starts with something near (the footprint sweep's own picker),
    each at a seeded heading."""
    import random
    from tests.footprint_sweep import starts
    rng = random.Random(f"lidar-{house}-{seed}")
    return [(x, y, rng.uniform(-math.pi, math.pi)) for x, y in starts(house, n, seed)]


def _truth(world):
    """The exact cast at each beam's centre, from the lidar's position."""
    from robot.safety import LIDAR_X_M
    off = LIDAR_X_M / DEFAULT_CELL_M
    ox, oy = world.x + off * math.cos(world.theta), world.y + off * math.sin(world.theta)
    max_cells = 12.0 / DEFAULT_CELL_M
    out = []
    for i in range(360):
        d = renderer.cast_ray_exact(world.layout, ox, oy, world.theta + math.radians(-180 + i),
                                    solid=world.solid_cells, max_dist=max_cells)
        out.append(None if d >= max_cells else d * DEFAULT_CELL_M)
    return out


def _serial_scan(world, line=lambda b: b, revolutions=3):
    """Packets for the pose written to a real pty, read back by `Ld19Lidar`."""
    master, slave, path = open_pty(None)
    lidar = Ld19Lidar(path)
    try:
        em = Emitter(world_caster(world), t0=0.0)
        data = b"".join(em.due(revolutions / 10.0 + 0.01))
        os.set_blocking(master, True)
        os.write(master, line(data))
        deadline = time.monotonic() + 3.0
        while lidar.revolutions < revolutions - 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        return lidar.get_scan(), lidar.parser
    finally:
        lidar.close()
        os.close(master)
        os.close(slave)


def _errors(scan, truth):
    errs, too_far = [], 0
    for got, want in zip(scan["ranges_m"], truth):
        if want is None and got is None:
            errs.append(0.0)
            continue
        if want is None:            # driver saw something the truth says is beyond range
            errs.append(0.0)
            continue
        if got is None:             # driver says clear where there is a surface: too far
            too_far += 1
            errs.append(math.inf)
            continue
        errs.append(abs(got - want))
        if got - want > 0.03:
            too_far += 1
    return errs, too_far


def _fidelity(line=lambda b: b):
    total, within, too_far, bearing_ok = 0, 0, 0, 0
    for house in HOUSES:
        world = build_world(house)
        for x, y, th in _poses(house, POSES_PER_HOUSE):
            world.x, world.y, world.theta = x, y, th
            scan, _ = _serial_scan(world, line)
            assert scan["usable"]
            truth = _truth(world)
            errs, tf = _errors(scan, truth)
            total += len(errs)
            within += sum(1 for e in errs if e <= 0.03)
            too_far += tf
            bearing_ok += _bearing_agrees(scan, truth)
    return within / total, too_far, bearing_ok


@pytest.mark.xfail(strict=True, reason=(
    "3.42 criterion 2 FAILED as written (2026-10-06): 97.9% of beams within 3 cm "
    "(bar 98%) and 47 of 10,800 beams more than 3 cm too far (bar 0). The driver "
    "is exact -- see the next test -- and the gap is the D500's 0.8-degree point "
    "spacing against a reference cast through each bin's exact centre: an edge "
    "thinner than the spacing falls between two points. Criterion 4 judges what "
    "that costs on ground truth."))
def test_criterion_2_the_scan_through_a_serial_line_matches_the_geometry():
    share, too_far, bearing_ok = _fidelity()
    assert share >= 0.98, share
    assert too_far == 0
    assert bearing_ok == len(HOUSES) * POSES_PER_HOUSE


def _ideal(world):
    """The same points the device takes, binned with no serial line or
    codec in between."""
    cast = world_caster(world)
    scan = bin_scan([(k * 0.8 % 360, cast(k * 0.8 % 360, 0.0), 0.0) for k in range(450)],
                    LIDAR_YAW_DEG)
    scan.pop("carried", None)
    return scan


def _bearing_agrees(scan, truth, tol_deg=2, tie_m=0.01):
    """The scan's nearest return lies within `tol_deg` of a truth bearing at
    the truth's nearest range. A corner or a wall seen square-on is equally
    near over several degrees, so "the" nearest bearing is a set, and a
    tie-break picking a different member is not an angular error."""
    r = scan["ranges_m"]
    got = min(range(360), key=lambda i: r[i] if r[i] is not None else math.inf)
    lo = min(t for t in truth if t is not None)
    near = [i for i, t in enumerate(truth) if t is not None and t <= lo + tie_m]
    return any(abs((got - i + 180) % 360 - 180) <= tol_deg for i in near)


def _driver_exactness(line=lambda b: b, revolutions=3):
    same = total = bearing_ok = 0
    for house in HOUSES:
        world = build_world(house)
        for x, y, th in _poses(house, POSES_PER_HOUSE):
            world.x, world.y, world.theta = x, y, th
            scan, _ = _serial_scan(world, line, revolutions)
            assert scan["usable"]
            ideal = _ideal(world)
            same += sum(a == b for a, b in zip(scan["ranges_m"], ideal["ranges_m"]))
            total += 360
            bearing_ok += _bearing_agrees(scan, _truth(world))
    return same / total, bearing_ok


def test_the_serial_path_loses_nothing():
    # Measured 2026-10-06: 10,800 of 10,800 beams identical to binning the
    # device's own points directly, and the nearest obstacle's bearing right
    # at every pose.
    share, bearing_ok = _driver_exactness()
    assert share == 1.0
    assert bearing_ok == len(HOUSES) * POSES_PER_HOUSE


@pytest.mark.xfail(strict=True, reason=(
    "3.42 criterion 3b FAILED as written (2026-10-06): 1% of bytes dropped and 1% "
    "corrupted destroys ~61% of the 47-byte packets (14 of ~37 a revolution "
    "survive), so the scan is UNUSABLE -- the safe answer, but not the one the "
    "criterion asked for. Pinned by the next test."))
def test_criterion_3_a_lossy_line_still_gives_a_usable_faithful_scan():
    share, bearing_ok = _driver_exactness(LossyLine(drop=0.01, corrupt=0.01, seed=3))
    assert share >= 0.98, share


def test_a_line_that_loses_most_packets_gives_no_scan_rather_than_a_wrong_one():
    world = build_world("starter_house")
    for x, y, th in _poses("starter_house", 3):
        world.x, world.y, world.theta = x, y, th
        scan, parser = _serial_scan(world, LossyLine(drop=0.01, corrupt=0.01, seed=3))
        assert not scan["usable"] and parser.crc_failed > parser.packets


def _usable_share(byte_loss, seed=7):
    world = build_world("scaled_house")
    world.x, world.y, world.theta = _poses("scaled_house", 1)[0]
    s = Ld19Scanner(clock=lambda: 0.0)
    line = LossyLine(drop=byte_loss, corrupt=byte_loss, seed=seed)
    s.feed(line(b"".join(Emitter(world_caster(world), t0=0.0).due(20.05))), 0.0)
    return (s.revolutions - s.revolutions_unusable) / s.revolutions


def test_measured_alongside_where_a_lossy_line_stops_giving_scans():
    # Not a criterion -- measured 2026-10-06 beside 3b's failure, 200
    # revolutions each, so the record says where "usable" ends. A healthy USB
    # link is at the top of this table.
    #   bytes lost (drop + corrupt)  packets lost  revolutions usable
    #   0.06%                        2.6%          100%
    #   0.2%                         8.5%           99%
    #   0.6%                         24%            39%
    #   2%  (criterion 3b)           60%             0%
    assert _usable_share(0.0003) == 1.0
    assert _usable_share(0.001) >= 0.98
    assert _usable_share(0.01) == 0.0


# ---------------- criterion 4b: forward on a lidar-only car ----------------

class _LidarOnlyCar:
    """What HardwareRobot answers on the car with this driver and no depth
    camera: a usable scan, an unusable grid, a scalar of 0.0."""

    def __init__(self, ahead_m):
        self.ahead_m = ahead_m

    def get_depth_grid(self):
        return unusable_grid()

    def get_distance(self):
        return 0.0

    def get_scan(self, max_range_m=None):
        ranges = [3.0] * 360
        ranges[180] = self.ahead_m          # dead ahead
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.02, "range_max_m": 12.0, "ranges_m": ranges}

    def get_wheel_state(self):
        return {"usable": True, "left": {"velocity_rad_s": 0.0},
                "right": {"velocity_rad_s": 0.0}, "wheel_radius_m": 0.04,
                "track_width_m": 0.172}


def test_criterion_4b_a_lidar_only_car_may_drive_forward_and_stops_at_the_line():
    clear = SafetyController(_LidarOnlyCar(3.0), 20.0)
    assert clear.forward_clearance()[0] > 200
    assert clear.path_clearance()[1] == "scan_path"
    # A wall 28 cm from the lidar is 28 - 8.65 = 19.35 cm from the bumper.
    near = SafetyController(_LidarOnlyCar(0.28), 20.0)
    cm, _src = near.forward_clearance()
    assert cm < 20.0
