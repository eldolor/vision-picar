"""
tests/test_footprint_safety.py

`PLAN-ros-alignment.md` 3.18, part 1 -- the oblique-approach escape of 3.17.
One test per criterion, written (with their thresholds) before the fix, and
confirmed red against the code 3.17 found it in.

Judged on GROUND TRUTH (`tests/footprint_sweep.py`): the URDF's chassis
rectangle against the house's occupied cells, never a sensor reading. The
path cone is what was fooled, so it cannot be what decides whether it was
fixed. The full sweep is `python -m tests.demo_footprint_sweep`; this pins a
seeded sample of it.
"""

import math
import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from sim.mock_robot import WHEEL_RADIUS_M
from tests import footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
STARTS_PER_HOUSE = 3

# The pose 3.18 reproduced the escape from, offline: the furnished home's
# dining area, compass 330 (theta = compass - 90 in the grid's convention).
ESCAPE = ("home_first_floor", 8.3 / 0.3, 9.8 / 0.3, 330 - 90)


@pytest.fixture(scope="module")
def clamped():
    return fs.sweep(HOUSES, STARTS_PER_HOUSE)


def _show(bad, key):
    worst = sorted(bad, key=lambda r: r[key])[:3]
    return [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()} for r in worst]


def test_the_reproduced_pose_escapes_no_longer():
    r = fs.run(*ESCAPE)
    assert r["T_after_move"] >= fs.T_BAR_CM, r
    assert r["min_G"] >= min(r["G0"], fs.G_BAR_CM), r


def test_criterion_1_the_stopping_distance_holds_for_what_is_in_the_way(clamped):
    bad = [r for r in clamped if r["T_after_move"] < fs.T_BAR_CM]
    assert not bad, f"{len(bad)}/{len(clamped)} runs moved to under {fs.T_BAR_CM}cm " \
                    f"of travel-to-contact: {_show(bad, 'T_after_move')}"


def test_criterion_2_no_contact(clamped):
    bad = [r for r in clamped if r["min_G"] < min(r["G0"], fs.G_BAR_CM) - 1e-6]
    assert not bad, f"{len(bad)}/{len(clamped)} runs touched: {_show(bad, 'min_G')}"


def test_criterion_3_the_fix_does_not_freeze_the_robot(clamped):
    eligible = [r for r in clamped if r["T0"] >= fs.PROGRESS_START_CM]
    assert len(eligible) >= 50, "the sample must contain runs with room to move"
    moved = [r for r in eligible if r["travel"] >= fs.PROGRESS_MIN_CM]
    share = len(moved) / len(eligible)
    assert share >= fs.PROGRESS_SHARE, \
        f"only {len(moved)}/{len(eligible)} ({share:.1%}) runs with room covered " \
        f"{fs.PROGRESS_MIN_CM}cm: {_show([r for r in eligible if r not in moved], 'travel')}"


def test_criterion_4_the_sim_does_not_let_the_chassis_into_anything():
    free = fs.sweep(HOUSES, STARTS_PER_HOUSE, clamp=False)
    bad = [r for r in free if r["max_P"] > fs.PENETRATION_BAR_CM]
    assert not bad, f"{len(bad)}/{len(free)} unclamped runs drove the chassis " \
                    f"into something: {_show(sorted(bad, key=lambda r: -r['max_P']), 'T0')}"


# A second pose for the live check, from the red sweep: 26cm of travel-to-
# contact at the start, so the robot MUST move and then stop -- the escape
# pose alone starts inside the band, where the fixed robot never moves.
DRIVES_THEN_STOPS = ("scaled_house", 11.44, 4.25, 240)


@pytest.mark.parametrize("pose", [ESCAPE, DRIVES_THEN_STOPS], ids=["escape", "drives-then-stops"])
def test_criterion_5_the_real_path(monkeypatch, pose):
    """Through `POST /wheels` and robot/server.py's own 20 Hz wheel loop on
    wall-clock time. Judged exactly as the sweep is: travel-to-contact after
    any interval in which the robot moved, and contact at any time."""
    house, x, y, theta_deg = pose
    monkeypatch.setenv("SIM_MAP", house)
    real_get_robot, built = server.get_robot, {}

    def placed(*a, **kw):
        robot = real_get_robot(*a, **kw)
        robot.world.x, robot.world.y = x, y
        robot.world.theta = math.radians(theta_deg)
        built["robot"] = robot
        return robot

    monkeypatch.setattr(server, "get_robot", placed)
    fwd = fs.SPEED_M_S / WHEEL_RADIUS_M
    with TestClient(server.create_app()) as c:
        world = built["robot"].world
        T0, G0, _ = fs.truth(world)
        worst_T, min_G, last = math.inf, G0, (world.x, world.y)
        end = time.monotonic() + fs.RUN_S
        while time.monotonic() < end:
            c.post("/wheels", json={"left_rad_s": fwd, "right_rad_s": fwd},
                   headers={"x-driver": "ros"})
            time.sleep(0.1)
            T, G, _ = fs.truth(world)
            if (world.x, world.y) != last:
                worst_T, last = min(worst_T, T), (world.x, world.y)
            min_G = min(min_G, G)
        c.post("/stop")
        travelled = math.hypot(world.x - x, world.y - y) * fs.CELL_CM
    if T0 >= fs.T_BAR_CM + 5:
        assert travelled > 1.0, "this pose has room: the robot must actually move"
    assert worst_T >= fs.T_BAR_CM, f"travel-to-contact fell to {worst_T:.1f}cm"
    assert min_G >= min(G0, fs.G_BAR_CM), f"the chassis touched ({min_G:.2f}cm)"


# ---------- the exact ray, and the scan hint that uses it ----------

def test_the_exact_ray_is_never_further_than_the_march():
    """`cast_ray_exact()` is what the safety scan reads, so the one property
    it must have is never to overstate a range the march would have
    reported -- and it must be within one march step of it except where the
    march slipped between two diagonal cells (3.18: 0.1-0.2% of beams)."""
    import random
    from sim import renderer
    from sim.maps import build_world
    rng = random.Random(7)
    slips = 0
    for house in HOUSES:
        w = build_world(house)
        free = [(x, y) for y, row in enumerate(w.layout) for x, c in enumerate(row)
                if c != "#" and (x, y) not in w.solid_cells]
        for _ in range(2000):
            cx, cy = rng.choice(free)
            px, py, a = cx + rng.random(), cy + rng.random(), rng.uniform(-math.pi, math.pi)
            march = renderer.cast_ray(w.layout, px, py, a, solid=w.solid_cells, max_dist=6)
            exact = renderer.cast_ray_exact(w.layout, px, py, a, solid=w.solid_cells, max_dist=6)
            assert exact <= march + 1e-9, (house, px, py, a, exact, march)
            slips += march - exact > renderer.FPV_STEP + 1e-9
    assert slips < 0.01 * 2000 * len(HOUSES)


def test_the_exact_ray_hits_a_wall_where_it_is():
    from sim import renderer
    layout = ["#####", "#...#", "#...#", "#####"]
    # From (1.25, 1.5) east, the wall face is x = 4: 2.75 cells.
    assert renderer.cast_ray_exact(layout, 1.25, 1.5, 0.0) == pytest.approx(2.75)
    # A ray through the shared corner of two diagonal cells is a hit.
    corner = ["....", ".#..", "..#.", "...."]
    assert renderer.cast_ray_exact(corner, 1.5, 2.5, -math.pi / 4) < 1.0


def test_a_range_hinted_scan_stops_at_the_hint_and_is_exact(monkeypatch):
    from sim import renderer
    from sim.maps import build_world
    from sim.mock_robot import MockRobot, DEFAULT_CELL_M
    robot = MockRobot(build_world("starter_house"), render=False)
    full, near = robot.get_scan(), robot.get_scan(max_range_m=0.6)
    assert near["range_max_m"] == full["range_max_m"], "the hint trims the call, not the sensor"
    from robot.safety import LIDAR_X_M      # beams leave the lidar, not the centre (3.27)
    ox = robot.world.x + LIDAR_X_M / DEFAULT_CELL_M * math.cos(robot.world.theta)
    oy = robot.world.y + LIDAR_X_M / DEFAULT_CELL_M * math.sin(robot.world.theta)
    for i, r in enumerate(near["ranges_m"]):
        exact = renderer.cast_ray_exact(robot.world.layout, ox, oy,
                                        robot.world.theta + math.radians(-180 + i),
                                        solid=robot.world.solid_cells, max_dist=0.6 / DEFAULT_CELL_M)
        assert r == (None if exact >= 0.6 / DEFAULT_CELL_M else round(exact * DEFAULT_CELL_M, 4))
        if full["ranges_m"][i] is not None and r is not None:
            assert r <= full["ranges_m"][i] + 1e-9
    # And over HTTP: the robot server honours it, and RemoteRobot sends it.
    with TestClient(server.create_app()) as c:
        body = c.get("/scan", params={"max_range_m": 0.6}).json()
        assert all(r is None or r <= 0.6 for r in body["ranges_m"])
        assert any(r is None for r in body["ranges_m"])
