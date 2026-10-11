"""evaluations/explore-369/trace.py -- one house's mission, step by step:
action, detail, pose, and the arrival readout. `python ... trace.py HOUSE`"""
import json, math, sys
sys.path.insert(0, ".")
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.maps.procthor import TARGET
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

house = int(sys.argv[1])
grid = build_world(f"procthor:val:{house}")
robot = MockRobot(grid, render=False)
runner = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=FakeNav(robot),
                       world=MockWorld(grid), max_steps=200, clock=lambda: grid.sim_time,
                       idle=robot.pass_time)
tcell = next(c for c, n in grid.objects.items() if n == TARGET)
runner.start()
agent = runner.agent
orig = agent.step
def step():
    r = orig()
    d = math.hypot(grid.x - tcell[0] - .5, grid.y - tcell[1] - .5) * 0.3
    arr = (r.scene or {}).get("_arrival") or {}
    seen = any(x["label"] == TARGET for x in r.frame.get("detections") or ())
    if seen or d < 1.2 or r.action not in ("WAIT",):
        print(f"{r.action:11s} d={d:4.2f} pan={grid.pan:+d} seen={int(seen)} "
              f"arr={arr.get('state','')}/{arr.get('range_m')}/{arr.get('bearing_deg')} | {str(r.detail)[:80]}")
    return r
agent.step = step
n = 0
while runner.tick() and n < 5000:
    n += 1
print(runner.status()["outcome"])
