"""
robot/lidar_ld19.py

`PLAN-ros-alignment.md` 3.42 -- the UGV Rover's D500 lidar (LDROBOT's
STL-19P), read by the ROBOT SERVER, not by ROS. The user's 2026-10-02
decision (6.5, 1.1): a sensor that feeds a veto stays outside ROS, so
`robot/safety.py` and the person-drives fallback keep a scan when ROS
hangs. ROS still gets the scan, through the bridge's `GET /scan`, exactly as
it gets the simulator's today.

**The protocol is LDROBOT's**, read from its published driver
(`ldrobotSensorTeam/ldlidar_stl_ros2`, MIT, `lipkg.h` / `lipkg.cpp`,
2026-10-06):

* 47-byte packets: `0x54` (header), `0x2C` (version / 12 points), then
  little-endian `speed` (degrees per second), `start_angle` (0.01 degree),
  twelve points of `distance` (mm; 0 = no return) and `intensity` (one
  byte), `end_angle` (0.01 degree), `timestamp` (ms, wraps at 30000 on the
  device), and a CRC-8 over the first 46 bytes by LDROBOT's own table.
* A packet's points are spread evenly from start to end angle.
* **Angles increase CLOCKWISE seen from above** -- LDROBOT's ROS node
  reverses them for ROS's counter-clockwise convention -- which is this
  project's body convention (`RobotInterface.get_scan()`), so no reversal
  here: only the mounting yaw.

That the D500 speaks this at 230,400 baud is inferred, not verified on the
unit (Waveshare launches LD19 at 230,400; the STL-19P is its sibling).
3.42 criterion 7 checks it on arrival; if it differs, only this file's
decoding changes.

**What the safety layer gets.** One beam per degree off the body heading,
-180 to +179, as `MockRobot.get_scan()` publishes -- the minimum return
in each 1-degree bin (the conservative choice for a veto), None where the
lidar saw nothing within range, and a bin no point fell into filled from
the previous revolution or its neighbours, never left reading "empty"
(`bin_scan()`). Ranges are measured from the
LIDAR, which the safety layer moves into the body frame with `LIDAR_X_M`
(3.27). The scan's age -- since its OLDEST point -- is `sensor_age_s()`,
which `SafetyController._aged()` subtracts the travel since from (3.36).
"""

import math
import os
import select
import struct
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

from robot.interface import unusable_scan

HEADER = 0x54
VER_LEN = 0x2C
POINTS_PER_PACKET = 12
PACKET_LEN = 47
BAUD = 230400

# `[CAD]` -- Waveshare's URDF turns the lidar 90 degrees to the left
# (3.26/3.27): the lidar's zero faces the robot's left, which is -90 in the
# body's clockwise-positive frame. A body bearing is the lidar's own angle
# plus this. Measured on the car by 3.42 criterion 7 (a box dead ahead must
# read 0 +/- 2 degrees); until then it is the CAD's word.
LIDAR_YAW_DEG = -90.0
RANGE_MIN_M = 0.02          # LDROBOT's node: range_min 0.02
RANGE_MAX_M = 12.0          # the D500's rated range; sim/mock_robot.LIDAR_RANGE_M
STALE_S = 0.3               # 3.42 criterion 3: no packets this long -> unusable

# LDROBOT's CRC-8 table, `lipkg.cpp` `CrcTable`, verbatim.
CRC_TABLE = bytes((
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8,
))

_BODY = struct.Struct("<BBHH" + "HB" * POINTS_PER_PACKET + "HH")   # 46 bytes


def crc8(data: bytes) -> int:
    """LDROBOT's `CalCRC8`."""
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


# --------------------------------------------------------------------------
# Packets
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Packet:
    speed_dps: int
    start_cdeg: int                 # 0.01 degree
    end_cdeg: int
    distances_mm: Tuple[int, ...]   # 12; 0 = no return
    intensities: Tuple[int, ...]    # 12
    timestamp_ms: int

    def angles_deg(self) -> List[float]:
        """The 12 points' angles, spread evenly start -> end across the
        0/360 seam (LDROBOT's `Parse`, without its integer truncation)."""
        span = (self.end_cdeg + 36000 - self.start_cdeg) % 36000
        step = span / (POINTS_PER_PACKET - 1) / 100.0
        start = self.start_cdeg / 100.0
        return [(start + i * step) % 360.0 for i in range(POINTS_PER_PACKET)]


def encode(p: Packet) -> bytes:
    fields = [HEADER, VER_LEN, p.speed_dps & 0xFFFF, p.start_cdeg % 36000]
    for d, i in zip(p.distances_mm, p.intensities):
        fields += [min(int(d), 0xFFFF), int(i) & 0xFF]
    fields += [p.end_cdeg % 36000, p.timestamp_ms % 30000]
    body = _BODY.pack(*fields)
    return body + bytes((crc8(body),))


def decode(buf: bytes) -> Optional[Packet]:
    """One 47-byte packet, or None if its framing or CRC is wrong."""
    if len(buf) != PACKET_LEN or buf[0] != HEADER or buf[1] != VER_LEN:
        return None
    if crc8(buf[:PACKET_LEN - 1]) != buf[PACKET_LEN - 1]:
        return None
    f = _BODY.unpack(buf[:PACKET_LEN - 1])
    pts = f[4:4 + 2 * POINTS_PER_PACKET]
    return Packet(speed_dps=f[2], start_cdeg=f[3], end_cdeg=f[-2],
                  distances_mm=tuple(pts[0::2]), intensities=tuple(pts[1::2]),
                  timestamp_ms=f[-1])


class Parser:
    """Bytes in, packets out, across arbitrary read boundaries. Resyncs on
    the two-byte header; a packet whose CRC fails is dropped and counted,
    and the search restarts one byte past its header, so garbage that
    happens to contain 0x54 0x2C cannot swallow a real packet behind it."""

    def __init__(self):
        self._buf = bytearray()
        self.packets = 0
        self.crc_failed = 0
        self.bytes_skipped = 0

    def feed(self, data: bytes) -> List[Packet]:
        self._buf += data
        out = []
        buf = self._buf
        i = 0
        while True:
            j = buf.find(b"\x54\x2c", i)
            if j < 0:
                # Keep a trailing 0x54: the next read may bring its 0x2C.
                keep = 1 if buf.endswith(b"\x54") else 0
                self.bytes_skipped += len(buf) - i - keep
                i = len(buf) - keep
                break
            self.bytes_skipped += j - i
            if len(buf) - j < PACKET_LEN:
                i = j
                break
            pkt = decode(bytes(buf[j:j + PACKET_LEN]))
            if pkt is None:
                self.crc_failed += 1
                i = j + 1
                continue
            out.append(pkt)
            self.packets += 1
            i = j + PACKET_LEN
        del buf[:i]
        return out


# --------------------------------------------------------------------------
# Revolutions and the scan
# --------------------------------------------------------------------------

Point = Tuple[float, int, float]     # (lidar angle deg, distance mm, time s)


class Assembler:
    """Points in, whole revolutions out: a revolution ends where the angle
    wraps past 360 (LDROBOT's `AssemblePacket` looks for < 20 after > 340;
    a fall of more than half a turn is the same test without the
    constants). The first, partial revolution is discarded, as theirs is."""

    def __init__(self):
        self._points: List[Point] = []
        self._last_angle: Optional[float] = None
        self._first = True

    def add(self, points: Sequence[Point]) -> List[List[Point]]:
        done = []
        for p in points:
            if self._last_angle is not None and p[0] < self._last_angle - 180.0:
                if not self._first and self._points:
                    done.append(self._points)
                self._first = False
                self._points = []
            self._points.append(p)
            self._last_angle = p[0]
        return done


MAX_UNCOVERED_BINS = 18     # 5%: more bins than this with no data -> not a scan


def bin_points(points: Sequence[Point], yaw_deg: float = LIDAR_YAW_DEG,
               range_min_m: float = RANGE_MIN_M, range_max_m: float = RANGE_MAX_M):
    """(nearest, seen): per 1-degree body bin, the nearest return (None if
    every point in it read "no return") and whether any point fell in it."""
    nearest: List[Optional[float]] = [None] * 360
    seen = [False] * 360
    lo, hi = range_min_m * 1000.0, range_max_m * 1000.0
    for angle, mm, _t in points:
        body = (angle + yaw_deg + 180.0) % 360.0 - 180.0
        i = int(math.floor(body + 180.0 + 0.5)) % 360
        seen[i] = True
        if lo <= mm <= hi:
            m = mm / 1000.0
            if nearest[i] is None or m < nearest[i]:
                nearest[i] = m
    return nearest, seen


def bin_scan(points: Sequence[Point], yaw_deg: float = LIDAR_YAW_DEG,
             range_min_m: float = RANGE_MIN_M, range_max_m: float = RANGE_MAX_M,
             prev: Optional[tuple] = None) -> dict:
    """A revolution as `RobotInterface.get_scan()`'s dict: 360 one-degree
    beams, body frame, clockwise positive, beam i at -180 + i degrees.

    Each bin is the NEAREST return in [-180 + i - 0.5, -180 + i + 0.5)
    -- conservative for a veto, which is the scan's first reader. A bin
    whose points all read "no return" is None.

    **A bin with no data is never read as clear** (M3's rule: unobserved is
    not empty). It takes, in order: the previous revolution's value if that
    one covered it (`prev` = its (nearest, seen)); else the nearer of the
    closest covered bins either side (a clear side counts only if BOTH
    sides are clear). Point spacing jitter leaves the odd bin empty, and a
    lost packet ~10 consecutive bins. More than `MAX_UNCOVERED_BINS` left
    after the carry and the whole scan is `unusable` -- too much of the
    ring is unknown to vet a move on.

    Returns the scan dict, with `"carried"` set when old data was used, so
    the caller can age the scan by the older revolution."""
    nearest, seen = bin_points(points, yaw_deg, range_min_m, range_max_m)
    carried = False
    if prev is not None:
        p_near, p_seen = prev
        for i in range(360):
            if not seen[i] and p_seen[i]:
                nearest[i], seen[i] = p_near[i], True
                carried = True
    uncovered = [i for i in range(360) if not seen[i]]
    if len(uncovered) > MAX_UNCOVERED_BINS:
        return dict(unusable_scan(), carried=carried)
    ranges: List[Optional[float]] = list(nearest)
    for i in uncovered:
        left = next(((i - k) % 360 for k in range(1, 360) if seen[(i - k) % 360]), None)
        right = next(((i + k) % 360 for k in range(1, 360) if seen[(i + k) % 360]), None)
        sides = [nearest[j] for j in (left, right) if j is not None]
        found = [v for v in sides if v is not None]
        ranges[i] = min(found) if found else None
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "range_min_m": range_min_m, "range_max_m": range_max_m,
            "ranges_m": [None if r is None else round(r, 4) for r in ranges],
            "carried": carried}


def packet_points(pkt: Packet, t_prev: Optional[float], t_now: float) -> List[Point]:
    """A packet's points with their times: spread between the previous
    packet's arrival and this one's, as LDROBOT's `Parse` stamps them."""
    angles = pkt.angles_deg()
    if t_prev is None or t_now <= t_prev:
        times = [t_now] * POINTS_PER_PACKET
    else:
        step = (t_now - t_prev) / (POINTS_PER_PACKET - 1)
        times = [t_prev + k * step for k in range(POINTS_PER_PACKET)]
    return [(a, d, t) for a, d, t in zip(angles, pkt.distances_mm, times)]


class Ld19Scanner:
    """The decoding half, with no port: bytes and their arrival times in, the
    latest scan out. `Ld19Lidar` drives it from a serial line; tests and the
    footprint sweep drive it directly."""

    def __init__(self, yaw_deg: float = LIDAR_YAW_DEG, stale_s: float = STALE_S,
                 clock: Callable[[], float] = time.monotonic):
        self.yaw_deg = yaw_deg
        self.stale_s = stale_s
        self.clock = clock
        self.parser = Parser()
        self._assembler = Assembler()
        self._lock = threading.Lock()
        self._t_prev: Optional[float] = None
        self._scan: Optional[dict] = None
        self._scan_oldest: Optional[float] = None
        self._scan_newest: Optional[float] = None
        self._last_packet_at: Optional[float] = None
        self.revolutions = 0
        self.revolutions_unusable = 0
        self.speed_dps = 0
        self._prev_bins: Optional[tuple] = None
        self._prev_oldest: Optional[float] = None

    def feed(self, data: bytes, t: Optional[float] = None) -> int:
        """Bytes received at time `t`; returns the number of revolutions
        completed by them."""
        t = self.clock() if t is None else t
        done = 0
        for pkt in self.parser.feed(data):
            pts = packet_points(pkt, self._t_prev, t)
            self._t_prev = t
            with self._lock:
                self._last_packet_at = t
                self.speed_dps = pkt.speed_dps
            for rev in self._assembler.add(pts):
                scan = bin_scan(rev, self.yaw_deg, prev=self._prev_bins)
                oldest = rev[0][2]
                if scan.pop("carried", False) and self._prev_oldest is not None:
                    oldest = self._prev_oldest       # some bins are a revolution older
                self._prev_bins = bin_points(rev, self.yaw_deg)
                self._prev_oldest = rev[0][2]
                with self._lock:
                    self._scan = scan if scan["usable"] else None
                    self._scan_oldest = oldest
                    self._scan_newest = rev[-1][2]
                    self.revolutions += 1
                    if not scan["usable"]:
                        self.revolutions_unusable += 1
                done += 1
        return done

    def _fresh(self, now: float) -> bool:
        # Packets arriving is not enough: a revolution must have completed
        # recently too, or a lidar whose angles stopped advancing would keep
        # serving its last scan.
        return (self._scan is not None and self._last_packet_at is not None
                and now - self._last_packet_at <= self.stale_s
                and now - self._scan_newest <= self.stale_s)

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        """The latest whole revolution, or `unusable` once no packet has
        arrived for `stale_s`. `max_range_m` is ignored: a real lidar
        measures everything at once (`RobotInterface.get_scan()` says a
        backend may)."""
        now = self.clock()
        with self._lock:
            if not self._fresh(now):
                return unusable_scan()
            return dict(self._scan, ranges_m=list(self._scan["ranges_m"]))

    def sensor_age_s(self) -> Optional[float]:
        """How old the scan's OLDEST point is: the safety layer subtracts the
        travel since (`SafetyController._aged`, 3.36). None with no usable
        scan -- the vet then sees `usable: false` and refuses on its own."""
        now = self.clock()
        with self._lock:
            if not self._fresh(now):
                return None
            return max(0.0, now - self._scan_oldest)

    def scan_time(self) -> Optional[float]:
        """When the current scan's OLDEST point was taken, on this scanner's
        clock (`time.monotonic` on the car) -- for a body that knows its own
        heading history to say how far it has turned since (3.42)."""
        now = self.clock()
        with self._lock:
            return self._scan_oldest if self._fresh(now) else None

    def status(self) -> dict:
        now = self.clock()
        with self._lock:
            return {"usable": self._fresh(now), "revolutions": self.revolutions,
                    "revolutions_unusable": self.revolutions_unusable,
                    "packets": self.parser.packets, "crc_failed": self.parser.crc_failed,
                    "speed_dps": self.speed_dps,
                    "last_packet_age_s": (None if self._last_packet_at is None
                                          else round(now - self._last_packet_at, 3))}


class Ld19Lidar(Ld19Scanner):
    """The D500 on a serial device (`ROBOT_LIDAR`: a udev name on the car,
    the fake's pty in the simulator). A reader thread feeds `Ld19Scanner`."""

    def __init__(self, port: str, baud: int = BAUD, yaw_deg: float = LIDAR_YAW_DEG,
                 stale_s: float = STALE_S):
        super().__init__(yaw_deg=yaw_deg, stale_s=stale_s)
        self.port = port
        self.link_error: Optional[str] = None
        self._fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
        tty.setraw(self._fd)
        attrs = termios.tcgetattr(self._fd)
        speed = getattr(termios, f"B{baud}", None)
        if speed is not None:
            attrs[4] = attrs[5] = speed
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        self._running = True
        self._thread = threading.Thread(target=self._reader, name="ld19-reader", daemon=True)
        self._thread.start()

    def _reader(self):
        # select() with a timeout, as HardwareRobot's reader: closing a pty
        # under a blocked read() hangs on macOS.
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                chunk = os.read(self._fd, 4096)
            except (OSError, ValueError) as e:
                self.link_error = str(e)
                return
            if not chunk:
                continue
            self.feed(chunk)

    def status(self) -> dict:
        return dict(super().status(), port=self.port, link_error=self.link_error)

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            os.close(self._fd)
        except OSError:
            pass
