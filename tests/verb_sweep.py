"""
tests/verb_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.22 -- guarded verbs. Shared by
`tests/test_guarded_verbs.py` (a sample, pinned) and
`tests/demo_verb_sweep.py` (the full sweep, printed).

3.18's sweep (`tests/footprint_sweep.py`) judged a STANDING wheel command
through the wheel loop's two calls. This one judges the VERBS -- FORWARD,
REVERSE, LEFT, RIGHT -- through the path they really take,
`SafetyController.check_and_execute()`, which is where a direct-mode verb
was vetted once and then left to drive blind.

**Ground truth, as in 3.18:** the chassis rectangle against the house's
occupied cells, via `footprint_sweep.truth()`, which shares no code with
what it judges. It is sampled after EVERY movement the simulator makes (each
`GridWorld.translate()` / `rotate()` sub-step), not only when a verb
returns, so a corner that swings into something and out again mid-turn is
still seen.
"""

import math
import random

from robot.safety import SafetyController, SafetyViolation, verb_speed_m_s
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
SPEEDS = (50, 100)            # a verb's speed: 0.3 and 0.6 m/s; both one cell
DURATION_S = 0.5
MAX_VERBS = 6                 # repeat a straight verb until refused, or this many
TURN_ANGLES = (45, 90)

# Bars: 3.18's and 3.19's, reused rather than invented.
T_BAR_CM = fs.T_BAR_CM        # 1: travel-to-contact after every verb
G_BAR_CM = fs.G_BAR_CM        # 1, 2: no contact
# 1b, added after a mutation run showed criterion 1 cannot see the
# look-ahead: the path cone measures from a half-cell "bumper" 2.35cm ahead
# of the Rover's real front edge, so truth keeps ~2cm in hand whether the
# verb stopped at the line or a period past it. At the line, the veto's own
# reading is `min_distance_cm` less the readings' ~1.5cm march quantum; a
# period past it, up to 3cm less again. The bar is 3.18 part 2's existing
# one for the same reading (`/depth` veto_cm >= 19.4). 19.9 was tried first
# and failed the real code at 19.5 -- that is the quantum, not the rule --
# while the look-ahead mutant reads 17.2.
VETO_BAR_CM = 19.4
# 4: verbs still mean what they meant, where there is room.
FULL_MOVE_START_CM = fs.PROGRESS_START_CM   # T at start >= 60 cm at speed 50 ...
FULL_MOVE_CM = 30.0                          # ... covers the whole cell ...
# 3.44's amendment to 4a: "room" is the speed's own bar + the cell + the
# slack 60 cm always left over 20 cm -- 60 cm at speed 50, 88 at speed 100.
FULL_MOVE_SLACK_CM = FULL_MOVE_START_CM - 20.0 - FULL_MOVE_CM
FULL_MOVE_TOL_CM = 0.1                       # ... to within 1 mm
TURN_ROOM_SLACK_DEG = 10                     # turns with this much spare room ...
TURN_TOL_DEG = 0.5                           # ... turn the whole angle
MEANING_SHARE = 0.95


def _sampled(world, direction, into):
    """Record truth after every translate/rotate the simulator performs."""
    translate, rotate = world.translate, world.rotate

    def t(*a, **k):
        out = translate(*a, **k)
        into.append(fs.truth(world, direction))
        return out

    def r(*a, **k):
        out = rotate(*a, **k)
        into.append(fs.truth(world, direction))
        return out

    world.translate, world.rotate = t, r


def straight_run(house, x, y, heading_deg, action="FORWARD", speed=50):
    """Straight verbs from one pose, repeated until one is refused (or
    MAX_VERBS have run). Returns what the truth said about each verb."""
    direction = +1 if action == "FORWARD" else -1
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    T0, G0, _ = fs.truth(world, direction)
    samples = []
    _sampled(world, direction, samples)
    verbs = []
    for _ in range(MAX_VERBS):
        T_before = fs.truth(world, direction)[0]
        x0, y0 = world.x, world.y
        try:
            result = safety.check_and_execute(action, speed=speed, duration=DURATION_S)
        except SafetyViolation:
            break
        T_after = fs.truth(world, direction)[0]
        # What the veto itself reads where a verb was cut short -- the
        # look-ahead's own measure (see VETO_BAR_CM).
        veto = None
        if result.get("stopped_short") == "clamped":
            veto = (safety.forward_clearance() if direction > 0 else safety.reverse_clearance())[0]
        verbs.append({"T_before": T_before, "T_after": T_after, "veto_at_stop": veto,
                      "speed": speed,
                      "moved": math.hypot(world.x - x0, world.y - y0) * fs.CELL_CM})
    return {"house": house, "x": x, "y": y, "heading": heading_deg, "action": action,
            "speed": speed, "T0": T0, "G0": G0, "verbs": verbs,
            "min_G": min([G0] + [g for _t, g, _p in samples]),
            "max_P": max([0.0] + [p for _t, _g, p in samples])}


def turn_run(house, x, y, theta, action="LEFT", angle=90):
    """One turn verb from one pose."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, theta
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    G0 = fs.truth(world)[1]
    room = fs.free_angle(world, +1 if action == "LEFT" else -1)
    samples = []
    _sampled(world, +1, samples)
    th0 = world.theta
    try:
        safety.check_and_execute(action, angle=angle)
    except SafetyViolation:
        pass        # refused outright (3.22: under VERB_MIN_TURN_DEG) -- turned 0
    turned = abs(math.degrees((world.theta - th0 + math.pi) % (2 * math.pi) - math.pi))
    return {"house": house, "x": x, "y": y, "theta": theta, "action": action,
            "angle": angle, "G0": G0, "room": room, "turned": turned,
            "min_G": min([G0] + [g for _t, g, _p in samples]),
            "max_P": max([0.0] + [p for _t, _g, p in samples])}


def straight_sweep(houses=HOUSES, starts_per_house=2, headings=8, seed=0,
                   actions=("FORWARD", "REVERSE"), speeds=SPEEDS):
    return [straight_run(h, x, y, k * 360 / headings, a, s)
            for h in houses for x, y in fs.starts(h, starts_per_house, seed)
            for k in range(headings) for a in actions for s in speeds]


def turn_sweep(houses=HOUSES, starts_per_house=4, seed=0, angles=TURN_ANGLES):
    """Two kinds of start: 3.19's pivot starts (something inside the
    turning circle -- where a turn can touch anything, criterion 2) and
    3.18's open starts at a seeded heading (where a turn has room, which is
    what criterion 4b needs to have anything to judge)."""
    rng = random.Random(f"turns-{seed}")
    poses = []
    for h in houses:
        poses += [(h, x, y, th) for x, y, th in fs.pivot_starts(h, starts_per_house, seed)]
        poses += [(h, x, y, rng.uniform(-math.pi, math.pi))
                  for x, y in fs.starts(h, starts_per_house, seed)]
    return [turn_run(h, x, y, th, a, ang)
            for h, x, y, th in poses for a in ("LEFT", "RIGHT") for ang in angles]


def full_move_start_cm(speed) -> float:
    """Truth's travel-to-contact from which a verb at `speed` has room for
    the whole cell (3.44's amendment to 4a)."""
    need = SafetyController(None, 20.0).required_clearance_cm(verb_speed_m_s(speed))
    return need + FULL_MOVE_CM + FULL_MOVE_SLACK_CM


def verdicts(straight, turns):
    """Criteria 1, 1b, 2 and 4 of 3.22: {name: (ok, detail)}."""
    all_verbs = [v for r in straight for v in r["verbs"]]
    c1 = [r for r in straight if any(v["T_after"] < T_BAR_CM for v in r["verbs"])]
    c1g = [r for r in straight if r["min_G"] < min(r["G0"], G_BAR_CM) - 1e-6]
    stops = [v["veto_at_stop"] for v in all_verbs if v["veto_at_stop"] is not None]
    c1b = [x for x in stops if x < VETO_BAR_CM]
    c2 = [r for r in turns if r["min_G"] < min(r["G0"], G_BAR_CM) - 1e-6]
    eligible = [v for v in all_verbs if v["T_before"] >= full_move_start_cm(v["speed"])]
    full = [v for v in eligible if abs(v["moved"] - FULL_MOVE_CM) <= FULL_MOVE_TOL_CM]
    t_elig = [r for r in turns if r["room"] >= r["angle"] + TURN_ROOM_SLACK_DEG]
    t_full = [r for r in t_elig if r["turned"] >= r["angle"] - TURN_TOL_DEG]
    worst_T = min((v["T_after"] for v in all_verbs), default=math.inf)
    share = len(full) / len(eligible) if eligible else 1.0
    t_share = len(t_full) / len(t_elig) if t_elig else 1.0
    return {
        "1 moves stop at the line": (not c1 and not c1g,
            f"{len(c1)}/{len(straight)} runs ended a verb under {T_BAR_CM}cm "
            f"(worst {worst_T:.1f}cm); {len(c1g)} touched"),
        "1b a cut-short move stops AT the line": (not c1b and bool(stops),
            f"{len(c1b)}/{len(stops)} cut-short verbs stopped with the veto reading "
            f"under {VETO_BAR_CM}cm" + (f", worst {min(c1b):.1f}cm" if c1b else "")),
        "2 turns do not touch": (not c2,
            f"{len(c2)}/{len(turns)} turns came within {G_BAR_CM}cm"
            + (f", worst {min(r['min_G'] for r in c2):.2f}cm" if c2 else "")),
        "4a a clear move is a whole cell": (share >= MEANING_SHARE,
            f"{len(full)}/{len(eligible)} = {share:.1%}"),
        "4b a clear turn is the whole angle": (t_share >= MEANING_SHARE,
            f"{len(t_full)}/{len(t_elig)} = {t_share:.1%}"),
    }
