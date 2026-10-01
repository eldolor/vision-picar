"""
tests/test_cad_geometry.py

`PLAN-ros-alignment.md` 3.27 -- the UGV Rover's CAD geometry, and the lidar
off-centre end to end. Criteria 1-3; 4-5 are the existing 3.18/3.19/3.11
suites (`test_footprint_safety.py`, `test_pivot_safety.py`,
`test_arrival.py`), which now run with the offset lidar.

Criterion 3 is judged on GROUND TRUTH: beam endpoints against the layout's
occupied cells, with the geometry written out here (`tests/footprint_sweep.py`'s
squares), never `renderer.cast_ray()`.
"""

import math
import re
from pathlib import Path

import pytest

from robot.safety import LIDAR_X_M
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import footprint_sweep as fs

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "service/slam/src/picar_description/urdf/picar.urdf.xacro"
CELL_M = 0.30

# ---- Waveshare's ugv_rover.urdf (waveshareteam/ugv_ws @ f0b3ad9, BSD),
# forward kinematics in its base_footprint frame (metres), as computed for
# PLAN-ros-alignment.md 3.27. Literals on purpose: the xacro is checked
# against the source, not against itself.
WS_ROTATION_CENTRE_X = 0.00046        # base_link's x; wheels at +/-0.0855 about it
WS_WHEEL_Z = 0.0388                   # wheel centre height
WS_LIDAR = (0.0403, 0.120)            # base_lidar_link x, z
WS_PAN_JOINT = (-0.0083, 0.168)       # pt_link1 (the pan joint) x, z
WS_CAMERA = (0.0394, 0.2104)          # pt_camera_link x, z, at pan 0 / tilt 0
OUR_BASE_Z = 0.040                    # base_link = wheel_radius above the floor


def _prop(name):
    m = re.search(rf'<xacro:property name="{name}"\s+value="([^"]+)"', XACRO.read_text())
    return float(m.group(1))


# ---------- criterion 1: the xacro is the CAD ----------

@pytest.mark.parametrize("name, want", [
    ("axle_x", 0.0),
    ("laser_x", WS_LIDAR[0] - WS_ROTATION_CENTRE_X),
    ("laser_z", WS_LIDAR[1] - OUR_BASE_Z),
    ("pan_x", WS_PAN_JOINT[0] - WS_ROTATION_CENTRE_X),
    ("pan_z", WS_PAN_JOINT[1] - OUR_BASE_Z),
    ("camera_x", WS_CAMERA[0] - WS_PAN_JOINT[0]),
    ("camera_up", WS_CAMERA[1] - WS_PAN_JOINT[1]),
])
def test_criterion_1_each_cad_value_is_the_rovers(name, want):
    assert abs(_prop(name) - want) <= 0.001, (name, _prop(name), round(want, 4))


def test_criterion_1_the_base_link_height_is_the_rovers_axle():
    assert abs(_prop("wheel_radius") - WS_WHEEL_Z) <= 0.0015


# ---------- criterion 2: one lidar offset ----------

def test_criterion_2_one_lidar_offset():
    assert _prop("laser_x") == LIDAR_X_M


# ---------- criterion 3: the sim casts from the laser frame ----------

STEP_CELLS = 0.005 / CELL_M        # half a centimetre
ON_SURFACE_CELLS = 0.01 / CELL_M   # 1 cm
CHECK_TO_M = 3.0


def _occupied(world):
    """Every solid cell of the house -- walls and solid objects -- read
    straight off the layout."""
    cells = set(world.solid_cells)
    for cy, row in enumerate(world.layout):
        for cx, ch in enumerate(row):
            if ch == "#":
                cells.add((cx, cy))
    return cells


def _inside(p, occupied):
    return (math.floor(p[0]), math.floor(p[1])) in occupied


def _dist_to_solid(p, occupied):
    """Distance from p to the nearest solid cell's square, in cells; only
    the 3x3 neighbourhood matters at the 1 cm scale asked."""
    best = math.inf
    cx0, cy0 = math.floor(p[0]), math.floor(p[1])
    for cy in range(cy0 - 1, cy0 + 2):
        for cx in range(cx0 - 1, cx0 + 2):
            if (cx, cy) in occupied:
                best = min(best, math.hypot(max(cx - p[0], 0, p[0] - cx - 1),
                                            max(cy - p[1], 0, p[1] - cy - 1)))
    return best


def _starts(world, n):
    """Free cell centres where the chassis touches nothing, spread over the
    house deterministically."""
    out = []
    rows = world.layout
    for cy, row in enumerate(rows):
        for cx, ch in enumerate(row):
            if ch == "#" or (cx, cy) in world.solid_cells:
                continue
            x, y = cx + 0.5, cy + 0.5
            A = fs.chassis(x, y, 0.0)
            if all(fs.penetration(A, B) == 0 for B in fs.occupied_near(world, x, y, 1.5)):
                out.append((x, y))
    step = max(1, len(out) // n)
    return out[::step][:n]


def _beam_errors(house, hinted=True, on_surface=ON_SURFACE_CELLS):
    world = build_world(house)
    robot = MockRobot(world=world)
    occupied = _occupied(world)
    bad, checked = [], 0
    for x, y in _starts(world, 4):
        for k in range(24):
            world.x, world.y, world.theta = x, y, math.radians(15 * k)
            scan = robot.get_scan(max_range_m=CHECK_TO_M) if hinted else robot.get_scan()
            lx = x + LIDAR_X_M / CELL_M * math.cos(world.theta)
            ly = y + LIDAR_X_M / CELL_M * math.sin(world.theta)
            a0, inc = scan["angle_min_deg"], scan["angle_increment_deg"]
            for i, r in enumerate(scan["ranges_m"]):
                if r is None or r > CHECK_TO_M:
                    continue
                checked += 1
                a = world.theta + math.radians(a0 + i * inc)
                dx, dy = math.cos(a), math.sin(a)
                rc = r / CELL_M
                end = (lx + dx * rc, ly + dy * rc)
                # On a surface: within 1 cm of something solid ...
                on = _dist_to_solid(end, occupied) <= on_surface
                # ... and the beam to 1 cm short of it crossed only free space.
                t, clear = 0.0, True
                while t < rc - on_surface:
                    if _inside((lx + dx * t, ly + dy * t), occupied):
                        clear = False
                        break
                    t += STEP_CELLS
                if not (on and clear):
                    bad.append((round(x, 2), round(y, 2), 15 * k, a0 + i * inc, r))
    return bad, checked


@pytest.mark.parametrize("house", ["starter_house", "scaled_house"])
def test_criterion_3_every_beam_ends_on_a_surface_seen_from_the_laser(house):
    bad, checked = _beam_errors(house)
    assert checked > 1000, checked
    assert not bad, f"{len(bad)}/{checked} beams do not end on a true surface from the laser: {bad[:5]}"


MARCH_QUANTUM_CELLS = 0.02 / CELL_M   # the full scan's documented ~1.5 cm over-read


@pytest.mark.parametrize("house", ["starter_house", "scaled_house"])
def test_criterion_3_the_full_scan_is_cast_from_the_laser_too(house):
    """SLAM's scan (no range hint) is the ray MARCH, which over-reads by up
    to ~1.5 cm (`get_scan()`'s docstring), so it is held to 2 cm rather than
    1 -- and it slips past diagonal cell corners on 0.1-0.2% of beams from
    ANY origin (3.18 measured it; a centre-cast control here shows 0.07%).
    So the bar is that slip rate. Measured 2026-10-01: 0.09% / 0.15%; cast
    from the centre instead, 67% / 68%."""
    bad, checked = _beam_errors(house, hinted=False, on_surface=MARCH_QUANTUM_CELLS)
    assert checked > 1000, checked
    assert len(bad) / checked <= 0.002, f"{len(bad)}/{checked}: {bad[:5]}"
