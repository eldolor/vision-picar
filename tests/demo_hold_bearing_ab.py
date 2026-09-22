"""Does dead-reckoning a bearing help when the detector is intermittent?

**IT CANNOT ANSWER THAT YET, and the reason is the finding.** Both arms come
back having closed ZERO distance and having never gone FORWARD: the sim
turns in 90-degree quanta against `CENTER_BAND_DEG` of 10, so a target off a
cardinal direction can never be centred -- each turn overshoots and flips
the sign of the error. Kept, rather than deleted, because it is the harness
that will answer the question the moment C2 (continuous pose) lands, and
because a demo that reproduces a blocker is worth more than a note saying
one exists. Run it after C2 and the FORWARD column should stop being empty.

Both arms drive the REAL MockRobot, so the odometry is genuine -- that is the
whole reason P25 says build this in the sim: `TeleopRobot` has no encoders
and the feature is inert on a phone walk.

Perception is synthesised rather than run, and deliberately so. 1.12 forbids
reading a detector's output on raycaster frames, so a real YOLOE run here
would be a number that means nothing. Instead the bearing is computed from
the grid's own geometry -- which makes it CORRECT -- and then withheld on a
fixed schedule, which is the thing under test: a detector that sees the
target only sometimes.

Scored on DISTANCE TO TARGET, not on command stability. P25's own
median_command_run is confounded -- a degenerate spin maximises it -- so the
objective has to be whether the robot actually got closer.
"""
import math
import sys

sys.path.insert(0, "/Users/anshugaind/vision-picar")

from brain.perceive import ABSENT, DETECTED, Box, Candidate, Detection, Perception
from brain.tiered import TieredVision
from control.walk_eval import compute_metrics
from robot.factory import get_robot

TARGET = "red backpack"
STEPS = 40
SEE_EVERY = 3          # the detector lands one frame in three


class GeometricStream:
    """A correct bearing from the grid, withheld on a schedule."""

    def __init__(self, world, goal):
        self.world, self.goal, self.i = world, goal, -1
        self.seen = 0

    def perceive(self, frame):
        self.i += 1
        if self.i % SEE_EVERY:
            return Perception(status=ABSENT, candidates=[], best=None)
        gx, gy = self.goal
        dx, dy = gx - self.world.robot_x, gy - self.world.robot_y
        world_deg = math.degrees(math.atan2(dy, dx))
        heading = {"E": 0, "S": 90, "W": 180, "N": 270}[self.world.heading.name]
        bearing = (world_deg - heading + 180) % 360 - 180
        self.seen += 1
        det = Detection(box=Box(0, 0, 10, 10), label=TARGET, confidence=0.9)
        c = Candidate(detection=det, similarity=0.9, bearing_deg=bearing)
        return Perception(status=DETECTED, candidates=[c], best=c)


def run(hold_bearing):
    robot = get_robot()
    world = robot.world
    goal = next(p for p, name in world.objects.items() if name == TARGET)
    start = math.dist((world.robot_x, world.robot_y), goal)
    tier = TieredVision(GeometricStream(world, goal), lambda f: {},
                        hold_bearing=hold_bearing, hold_goal=True,
                        steer_on_sight=True, hold_bearing_max_m=1.0)
    entries = []
    for _ in range(STEPS):
        scene = tier({"odometry": robot.get_odometry()})
        action = scene.get("safest_direction")
        entries.append({"navigate": {"action": action}})
        if action == "FORWARD":
            robot.drive_forward(50)
        elif action == "LEFT":
            robot.turn_left(50)
        elif action == "RIGHT":
            robot.turn_right(50)
    end = math.dist((world.robot_x, world.robot_y), goal)
    return compute_metrics(entries), start, end, tier._dead_reckoned


print(f"target {TARGET!r}, {STEPS} steps, detector lands 1 frame in {SEE_EVERY}\n")
print(f"{'hold_bearing':13s} {'start':>6s} {'end':>6s} {'closed':>7s} "
      f"{'med run':>8s} {'dominant':>9s} {'reckoned':>9s}  spread")
for hold in (False, True):
    m, start, end, dr = run(hold)
    print(f"{str(hold):13s} {start:6.1f} {end:6.1f} {start - end:+7.1f} "
          f"{m['median_command_run']:>8} {m['dominant_action_share']:>9.2f} "
          f"{dr:>9}  {m['action_spread']}")
