"""
tests/footprint_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.18, part 1 -- the oblique
approach. Shared by `tests/test_footprint_safety.py` (a sample, pinned) and
`tests/demo_footprint_sweep.py` (the full sweep, printed).

**Everything here is GROUND TRUTH, and deliberately shares no code with what
it judges.** The chassis is the URDF's rectangle; the obstacles are the
house's occupied cells read straight off the layout; the geometry is written
out here from scratch. Not `renderer.cast_ray()`, not `get_scan()`, not
`robot/safety.py` and not the sim's own collision -- a bug in any of those
must not be able to hide in the thing measuring it. The one thing borrowed
is the loop under test: `vet_wheel_velocity()` then `advance()`, exactly the
two calls `robot/server.py`'s wheel loop makes every period.
"""

import math
import random

from robot.safety import SafetyController
from sim import renderer
from sim.maps import build_world
from sim.mock_robot import MockRobot, WHEEL_RADIUS_M

CELL_CM = 30.0
# The URDF body (picar_description's xacro): the UGV Rover's outer 253 x
# 231 mm since 3.21 (the 2WD build's 228 x 198 before). Written out here rather than imported, so the metric does not
# move if the constant under test does -- tests/test_wall_linters.py is what
# keeps the copies in step.
HALF_LENGTH = 12.65 / CELL_CM  # cells
HALF_WIDTH = 11.55 / CELL_CM

PERIOD_S = 0.05          # robot/server.py's WHEEL_LOOP_INTERVAL_S
SPEED_M_S = 0.1          # R2b's test speed: 0.5 cm per period
RUN_S = 3.0
HEADINGS = 24            # every 15 degrees
NEAR_CM = 40.0           # a start with nothing this close tests nothing

# Criteria (PLAN-ros-alignment.md 3.18), written before the sweep was run.
T_BAR_CM = 18.0          # 1: 20 - 0.5 period travel - 1.5 ray-march quantum
G_BAR_CM = 1.0           # 2: no contact
PROGRESS_START_CM = 60.0  # 3: runs with this much travel-to-contact ...
PROGRESS_MIN_CM = 25.0    # ... must cover this much ...
PROGRESS_SHARE = 0.95     # ... this often
PENETRATION_BAR_CM = 0.5  # 4: the sim never lets the chassis in deeper


# ---------------- geometry, in cells ----------------

def chassis(x, y, theta):
    """The four corners, counter-clockwise in the grid's y-down frame."""
    ux, uy = math.cos(theta), math.sin(theta)
    nx, ny = -uy, ux
    return [(x + ux * a + nx * b, y + uy * a + ny * b)
            for a, b in ((HALF_LENGTH, HALF_WIDTH), (HALF_LENGTH, -HALF_WIDTH),
                         (-HALF_LENGTH, -HALF_WIDTH), (-HALF_LENGTH, HALF_WIDTH))]


def square(cx, cy):
    return [(cx, cy), (cx + 1, cy), (cx + 1, cy + 1), (cx, cy + 1)]


def _edges(poly):
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def _point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def penetration(A, B):
    """How deep two convex polygons overlap (0 if they do not): the
    smallest overlap over every separating-axis candidate, in cells."""
    depth = math.inf
    for P in (A, B):
        for a, b in _edges(P):
            ex, ey = b[0] - a[0], b[1] - a[1]
            n = math.hypot(ex, ey)
            nx, ny = ey / n, -ex / n
            pa = [nx * x + ny * y for x, y in A]
            pb = [nx * x + ny * y for x, y in B]
            depth = min(depth, min(max(pa), max(pb)) - max(min(pa), min(pb)))
            if depth <= 0:
                return 0.0
    return depth


def gap(A, B):
    """Distance between two convex polygons; 0 when touching or overlapping."""
    if penetration(A, B) > 0:
        return 0.0
    return min(min(_point_segment(p, a, b) for p in A for a, b in _edges(B)),
               min(_point_segment(p, a, b) for p in B for a, b in _edges(A)))


def _ray_to_segment(p, d, a, b):
    """t >= 0 where p + t*d meets segment ab, or inf."""
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = d[0] * ey - d[1] * ex
    if abs(den) < 1e-12:
        return math.inf
    wx, wy = a[0] - p[0], a[1] - p[1]
    t = (wx * ey - wy * ex) / den
    s = (wx * d[1] - wy * d[0]) / den
    return t if t >= -1e-12 and -1e-12 <= s <= 1 + 1e-12 else math.inf


def travel_to_contact(A, B, d):
    """How far convex A can translate along unit d before touching convex B."""
    if penetration(A, B) > 1e-9:
        return 0.0
    t = min(_ray_to_segment(p, d, a, b) for p in A for a, b in _edges(B))
    back = (-d[0], -d[1])
    return max(0.0, min(t, min(_ray_to_segment(p, back, a, b) for p in B for a, b in _edges(A))))


def occupied_near(world, x, y, reach_cells):
    r = int(math.ceil(reach_cells)) + 1
    out = []
    for cy in range(int(y) - r, int(y) + r + 1):
        for cx in range(int(x) - r, int(x) + r + 1):
            if renderer._cell_at(world.layout, cx, cy) == "#" or (cx, cy) in world.solid_cells:
                out.append(square(cx, cy))
    return out


def truth(world, direction=+1):
    """(T, G, penetration) in cm at the world's current pose."""
    A = chassis(world.x, world.y, world.theta)
    d = (direction * math.cos(world.theta), direction * math.sin(world.theta))
    # Nearest first, and pruned on a bound that cannot be wrong: no two
    # shapes are nearer than their centres' distance less both circumradii.
    reach = math.hypot(HALF_LENGTH, HALF_WIDTH) + math.sqrt(0.5)
    cells = sorted(((math.hypot(B[0][0] + 0.5 - world.x, B[0][1] + 0.5 - world.y), B)
                    for B in occupied_near(world, world.x, world.y, 2.5)),
                   key=lambda c: c[0])
    T, G, P = 2.5, math.inf, 0.0
    for dc, B in cells:
        bound = dc - reach
        if bound >= max(T, G):
            break
        if bound < G:
            g = gap(A, B)
            G = min(G, g)
            if g == 0.0:
                P = max(P, penetration(A, B))
        ahead = (B[0][0] + 0.5 - world.x) * d[0] + (B[0][1] + 0.5 - world.y) * d[1]
        if bound < T and ahead > -reach:
            T = min(T, travel_to_contact(A, B, d))
    return T * CELL_CM, G * CELL_CM, P * CELL_CM


# ---------------- the sweep ----------------

def _point_square(x, y, B):
    (x0, y0), (x1, y1) = B[0], B[2]
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


def starts(house, n, seed=0):
    """`n` seeded start positions on free floor: nothing inside the
    chassis' circumscribed circle (so the start is legal at EVERY heading
    -- a pivot sweeps that circle), and something within NEAR_CM of it."""
    rng = random.Random(f"{house}-{seed}")
    world = build_world(house)
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c != "#" and (x, y) not in world.solid_cells]
    radius = math.hypot(HALF_LENGTH, HALF_WIDTH)
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        x, y = cx + rng.random(), cy + rng.random()
        near = min((_point_square(x, y, B) for B in occupied_near(world, x, y, 2.0)),
                   default=math.inf)
        if near <= radius or (near - radius) * CELL_CM > NEAR_CM:
            continue
        out.append((x, y))
    return out


class _Lagged:
    """3.36 criterion 8: the vet fed readings `lag` wheel-loop periods old,
    as the split simulator's robot server reads a bundle up to
    SENSOR_STALE_S old. Everything else is the live robot."""

    def __init__(self, robot, lag):
        self._robot, self._lag, self._history = robot, lag, []

    def record(self, hint):
        self._history.append((hint, self._robot.get_scan(max_range_m=hint),
                              self._robot.get_depth_grid(), self._robot.get_distance()))
        del self._history[:-(self._lag + 1)]

    def _old(self):
        return self._history[0]

    def get_scan(self, max_range_m=None):
        hint, scan, _, _ = self._old()
        return scan if max_range_m == hint else self._robot.get_scan(max_range_m=max_range_m)

    def get_depth_grid(self):
        return self._old()[2]

    def get_distance(self):
        return self._old()[3]

    def __getattr__(self, name):
        return getattr(self._robot, name)


def run(house, x, y, heading_deg, direction=+1, clamp=True, pan=0.0, lag=0):
    """One standing command through the wheel loop's two calls. Returns a
    dict of the truth it met. `lag` > 0 vets on readings that many periods
    old (3.36); the truth is always now."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    world.pan = pan   # 3.18 part 2: the camera, left where a mission left it
    robot = MockRobot(world, render=False)
    sensed = _Lagged(robot, lag) if lag else robot
    safety = SafetyController(sensed, 20.0)
    hint = None
    if lag:
        from robot.safety import FOOTPRINT_LENGTH_M, SAFETY_SCAN_RANGE_M
        hint = max(SAFETY_SCAN_RANGE_M, FOOTPRINT_LENGTH_M / 2 + 1.5 * 20.0 / 100.0)
    w = direction * SPEED_M_S / WHEEL_RADIUS_M
    T0, G0, _ = truth(world, direction)
    worst_T_after_move, min_G, max_P = math.inf, G0, 0.0
    x0, y0 = world.x, world.y
    for _ in range(int(RUN_S / PERIOD_S)):
        left, right = w, w
        if lag:
            sensed.record(hint)
        if clamp:
            left, right, _reason = safety.vet_wheel_velocity(w, w)
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        robot.advance(PERIOD_S)
        T, G, P = truth(world, direction)
        min_G, max_P = min(min_G, G), max(max_P, P)
        if math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9:
            # Stopped -- and the house is static and the command identical,
            # so every later period would be this one again. Deterministic,
            # which is what makes ending here exact rather than a sample.
            # Not with lagged readings: a stale reading may let the robot
            # move again a period later, so it runs on, and a period with
            # no motion is simply not a move (3.36).
            if not lag:
                break
            continue
        worst_T_after_move = min(worst_T_after_move, T)
    return {"house": house, "x": x, "y": y, "heading": heading_deg, "dir": direction, "pan": pan,
            "T0": T0, "G0": G0, "T_after_move": worst_T_after_move, "min_G": min_G,
            "max_P": max_P, "travel": math.hypot(world.x - x0, world.y - y0) * CELL_CM}


def sweep(houses, starts_per_house, seed=0, clamp=True, directions=(+1, -1), pan=0.0, lag=0):
    out = []
    for house in houses:
        for x, y in starts(house, starts_per_house, seed):
            for h in range(HEADINGS):
                for d in directions:
                    out.append(run(house, x, y, h * 15, d, clamp, pan, lag))
    return out


def verdicts(results):
    """Criteria 1-3 over a clamped sweep: (ok, detail) per criterion."""
    c1 = [r for r in results if r["T_after_move"] < T_BAR_CM]
    c2 = [r for r in results if r["min_G"] < min(r["G0"], G_BAR_CM) - 1e-6]
    eligible = [r for r in results if r["T0"] >= PROGRESS_START_CM]
    moved = [r for r in eligible if r["travel"] >= PROGRESS_MIN_CM]
    share = len(moved) / len(eligible) if eligible else 1.0
    return {
        "1 stopping distance": (not c1, f"{len(c1)}/{len(results)} runs under {T_BAR_CM}cm"
                                         + (f", worst {min(r['T_after_move'] for r in c1):.1f}" if c1 else "")),
        "2 no contact": (not c2, f"{len(c2)}/{len(results)} runs touched"
                                  + (f", worst {min(r['min_G'] for r in c2):.2f}" if c2 else "")),
        "3 progress": (share >= PROGRESS_SHARE,
                       f"{len(moved)}/{len(eligible)} = {share:.1%} eligible runs covered {PROGRESS_MIN_CM}cm"),
    }


# ---------------- pivots (PLAN-ros-alignment.md 3.19) ----------------

PIVOT_RAD_S = 1.0         # body yaw rate: ~170 degrees in RUN_S
FREE_ANGLE_CAP_DEG = 180
FREE_ANGLE_MIN_DEG = 30   # criterion 2: runs with this much room ...
FREE_ANGLE_SLACK_DEG = 10  # ... must turn to within this of it
PIVOT_REACH_DEG = math.degrees(PIVOT_RAD_S * RUN_S) - 2  # what one run can reach


def pivot_starts(house, n, seed=0):
    """`n` seeded (x, y, theta) where the chassis is clear at its heading
    (gap >= 1 cm) but something lies inside its 17.1 cm turning circle --
    the only starts where a pivot can touch anything."""
    rng = random.Random(f"pivot-{house}-{seed}")
    world = build_world(house)
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c != "#" and (x, y) not in world.solid_cells]
    radius = math.hypot(HALF_LENGTH, HALF_WIDTH)
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        x, y, th = cx + rng.random(), cy + rng.random(), rng.uniform(-math.pi, math.pi)
        cells = occupied_near(world, x, y, 1.0)
        if min((_point_square(x, y, B) for B in cells), default=math.inf) >= radius:
            continue
        A = chassis(x, y, th)
        if min((gap(A, B) for B in cells), default=math.inf) * CELL_CM < G_BAR_CM:
            continue
        out.append((x, y, th))
    return out


def free_angle(world, direction):
    """Degrees the chassis can turn (+1 left / CCW, -1 right) before its gap
    would fall below criterion 1's bar -- `min(gap now, G_BAR_CM)` -- in
    1-degree steps, capped.

    To the BAR, not to contact (corrected 3.19, before the fix was
    accepted): a corner grazing past something at 0.5cm never touches it,
    so "room to contact" counted as free a turn criterion 1 forbids, and
    the two criteria contradicted each other on every grazing pass."""
    cells = occupied_near(world, world.x, world.y, 1.0)
    A0 = chassis(world.x, world.y, world.theta)
    bar = min(min((gap(A0, B) for B in cells), default=math.inf), G_BAR_CM / CELL_CM)
    for deg in range(1, FREE_ANGLE_CAP_DEG + 1):
        th = world.theta - direction * math.radians(deg)   # CCW body = theta decreasing
        A = chassis(world.x, world.y, th)
        if any(gap(A, B) < bar - 1e-9 for B in cells):
            return deg - 1
    return FREE_ANGLE_CAP_DEG


def pivot_run(house, x, y, theta, direction, clamp=True):
    """A standing pivot through the wheel loop's two calls."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, theta
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    wheels = robot.get_wheel_state()
    w = direction * PIVOT_RAD_S * wheels["track_width_m"] / 2 / WHEEL_RADIUS_M
    G0 = truth(world)[1]
    room = free_angle(world, direction)
    min_G, max_P, turned = G0, 0.0, 0.0
    for _ in range(int(RUN_S / PERIOD_S)):
        left, right = -w, w
        if clamp:
            left, right, _reason = safety.vet_wheel_velocity(-w, w)
        robot.set_wheel_velocity(left, right)
        th0 = world.theta
        robot.advance(PERIOD_S)
        d = abs((world.theta - th0 + math.pi) % (2 * math.pi) - math.pi)
        if d < 1e-9:
            break      # stopped; static world, identical command: stays stopped
        turned += math.degrees(d)
        _T, G, P = truth(world)
        min_G, max_P = min(min_G, G), max(max_P, P)
    return {"house": house, "x": x, "y": y, "theta": theta, "dir": direction,
            "G0": G0, "min_G": min_G, "max_P": max_P, "turned": turned, "room": room}


def pivot_sweep(houses, starts_per_house, seed=0, clamp=True):
    return [pivot_run(h, x, y, th, d, clamp)
            for h in houses for x, y, th in pivot_starts(h, starts_per_house, seed)
            for d in (+1, -1)]
