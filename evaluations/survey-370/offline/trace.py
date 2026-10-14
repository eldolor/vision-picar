"""Record every tick's pose + true room and the final map for survey runs."""
import json, sys, pickle
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests import survey_sweep as ss

class Rec:
    def __init__(self, grid): self.grid, self.trace = grid, []
    def observe(self, m, pose, look, alive=lambda: True):
        if pose and pose.get("usable"):
            g = self.grid
            self.trace.append((g.sim_time, pose["x_m"], pose["y_m"], g.room_at(g.robot_x, g.robot_y)))
        return None
    def layer(self, m): return None
    def status(self): return {"label_calls": 0}
    calls = 0

def run(house, start=None):
    grid = build_world(house)
    if start is not None:
        (cx, cy), th, _ = start
        grid.x, grid.y, grid.theta = cx + .5, cy + .5, th
    robot = MockRobot(grid, render=False); world = MockWorld(grid)
    r = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot), world=world,
                      mission_time_limit_s=1800, clock=lambda: grid.sim_time, idle=robot.pass_time)
    rec = Rec(grid); r.room_labeller = rec
    r.start(); n = 0
    while r.tick() and n < 40000: n += 1
    return {"house": house, "trace": rec.trace, "map": world.get_map(), "outcome": r.status()["outcome"]}

if __name__ == "__main__":
    out = []
    if sys.argv[1] == "houses":
        for h in ss.HOUSES:
            for st in ss.starts(h):
                out.append(run(h, st)); print(h, out[-1]["outcome"], len(out[-1]["trace"]), flush=True)
    else:
        from sim.maps.procthor import usable
        a, b = int(sys.argv[2]), int(sys.argv[3])
        for i in usable()[a:b]:
            out.append(run(f"procthor:val:{i}")); print(i, out[-1]["outcome"], flush=True)
    pickle.dump(out, open(sys.argv[-1], "wb"))
