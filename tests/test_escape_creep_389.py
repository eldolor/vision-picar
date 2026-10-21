"""3.89: explore's escape creep answered after the brain had given up.

Under `drive: ros`, nav2's collision_monitor halves every forward twist with
the lidar seeing something 0.134-0.33 m ahead (`nav2.yaml`'s
`PolygonSlow`, `slowdown_ratio: 0.5`), so a creep FORWARD from the 20 cm bar
to the 3 cm line ran at ~0.015 m/s instead of its 0.03, and its `/action`
answered after `RemoteRobot`'s 10 s read timeout: 3.88 A's live run 2 ended
`failed` at 759 s (docs/plans/ros-alignment/3.89-escape-creep-timeout.md).

The fix: a verb on the wall clock ends within `VERB_MAX_S` (and says it
was cut by time), and the escape creeps on in the same direction when its
creep was cut that way, so it still reaches the line.
"""

import math

import httpx
import pytest

import robot.interface as interface
import robot.ros_drive as ros_drive
from brain.explore import CREEP_PIECES, ExploreAgent
from brain.memory import MissionMemory
from control.remote_robot import DEFAULT_TIMEOUT_S
from robot.interface import VERB_MAX_REASON, VERB_MAX_S
from robot.safety import LIDAR_X_M, SafetyController, SafetyViolation
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests import ros_verb_sweep as rvs
from tests.test_explore import FakeNav

# nav2.yaml's collision_monitor PolygonSlow: x from, x to, half-width (m).
SLOW_POLYGON = (0.134, 0.33, 0.12)
SLOWDOWN_RATIO = 0.5
# Where the in-process repro first wedged with the front at the bar
# (home_first_floor, cells and radians): FORWARD at speed 50 refused at 19.9 cm.
AT_THE_BAR = (11.18795209371891, 18.890594768591647, 3.5764134230726126)


class CollisionMonitorChain(rvs.SafeChain):
    """3.24's ROS chain with nav2's collision_monitor in it: a forward twist
    is halved while 3 or more scan points lie in the slow polygon."""

    def _land(self, twist):
        v = twist["linear_m_s"]
        if v > 0:
            scan = self.inner.get_scan(max_range_m=0.5)
            inside = 0
            for i, r in enumerate(scan["ranges_m"]):
                if r is None:
                    continue
                a = math.radians(scan["angle_min_deg"] + i * scan["angle_increment_deg"])
                x, y = LIDAR_X_M + r * math.cos(a), r * math.sin(a)
                inside += SLOW_POLYGON[0] <= x <= SLOW_POLYGON[1] and abs(y) <= SLOW_POLYGON[2]
            if inside >= 3:
                twist = dict(twist, linear_m_s=v * SLOWDOWN_RATIO)
        super()._land(twist)


def _ros_entry(world):
    """The robot server under `drive: ros`: ONE SafetyController, both the
    `/action` entry and the `/wheels` vet (live they are the same object, so
    the creep's asked speed holds its 3 cm bar on the twists too)."""
    inner = MockRobot(world, render=False)
    robot = ros_drive.RosDriveRobot(inner, "http://bridge")
    entry = SafetyController(robot, 20.0)
    chain = CollisionMonitorChain(inner, entry, 0, lambda before: None)
    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(chain.handler))
    return entry, chain


def _at_the_bar():
    world = build_world("home_first_floor")
    world.x, world.y, world.theta = AT_THE_BAR
    return world


def _creep(entry, chain, action="FORWARD"):
    with rvs._virtual_time(chain.clock):
        t0 = chain.clock.now
        try:
            out = entry.check_and_execute(action, speed=5)
        except SafetyViolation as e:
            out = {"refused": str(e)}
        return chain.clock.now - t0, out


def test_the_pose_is_at_the_bar():
    entry, _ = _ros_entry(_at_the_bar())
    cm, _ = entry.forward_clearance()
    assert 19.0 <= cm < 20.0


def test_a_ros_creep_toward_a_wall_answers_inside_the_brains_request_timeout():
    """Red on dev: 11.7 s (live 9.8-10.3 s), past RemoteRobot's 10 s."""
    entry, chain = _ros_entry(_at_the_bar())
    seconds, out = _creep(entry, chain)
    assert "refused" not in out
    assert seconds <= VERB_MAX_S + 0.5
    assert seconds <= DEFAULT_TIMEOUT_S - 2.0      # criterion 2's 8.0 s


def test_a_creep_cut_by_time_says_so_and_a_second_reaches_the_line():
    entry, chain = _ros_entry(_at_the_bar())
    _, first = _creep(entry, chain)
    assert first.get("stopped_short") == "timeout"
    seconds, second = _creep(entry, chain)
    assert seconds <= VERB_MAX_S + 0.5
    cm, _ = entry.forward_clearance()
    assert cm <= 4.0                               # at the 3 cm line, within 1 cm
    assert abs(first["moved_m"]) + abs(second["moved_m"]) >= 0.15


def test_a_reverse_creep_is_not_cut():
    """No slow polygon behind: a REVERSE creep reaches the line in one verb."""
    world = _at_the_bar()
    world.theta += math.pi                         # the same wall, behind
    entry, chain = _ros_entry(world)
    cm, _ = entry.reverse_clearance()
    assert 15.0 <= cm < 21.0
    seconds, out = _creep(entry, chain, "REVERSE")
    assert "stopped_short" not in out
    assert seconds <= VERB_MAX_S
    assert entry.reverse_clearance()[0] <= 4.0


def test_the_cap_sits_inside_the_request_timeout_with_room_for_the_reply():
    """A capped verb still settles once (3 control periods) and the reply
    crosses HTTP; criterion 2 keeps 2 s of the 10 s spare."""
    assert VERB_MAX_S + 3.0 / ros_drive.CONTROL_HZ + 0.5 <= DEFAULT_TIMEOUT_S - 2.0


def test_a_slow_wall_clock_verb_in_direct_mode_ends_at_the_cap(monkeypatch):
    """The real board's guarded verbs (`carry_out_verb`, wall clock): the
    same bound, ended 'timeout'."""
    class Clock:
        t = 0.0

        def monotonic(self):
            return self.t

        def sleep(self, dt):
            self.t += dt
    clock = Clock()
    monkeypatch.setattr(interface, "time", clock)

    class Slow:
        """Wheels that turn at a third of what they are asked: never
        stalled, never done."""
        stop_count = 0

        def get_wheel_state(self):
            pos = 0.01 * clock.t / 0.04             # 0.01 m/s on 4 cm wheels
            return {"usable": True, "wheel_radius_m": 0.04, "track_width_m": 0.2,
                    "left": {"position_rad": pos}, "right": {"position_rad": pos}}

        def set_wheel_velocity(self, left, right):
            pass

    plan = {"kind": "straight", "left_rad_s": 0.75, "right_rad_s": 0.75, "target": 0.30,
            "wall_clock": True}
    outcome = interface.carry_out_verb(Slow(), plan)
    assert outcome["ended"] == "timeout"
    assert clock.t <= VERB_MAX_S + 0.1


# ---- the escape: a creep cut by time creeps on, the same way, a bounded number of times


def _agent(results):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    asked = []

    def execute(action, **kw):
        asked.append((action, kw.get("speed")))
        return results.pop(0) if results else {"action": action, "moved_m": 0.0}
    agent.safety.check_and_execute = execute
    return agent, asked


def test_the_escape_creeps_on_after_a_creep_cut_by_time():
    cut = {"action": "drive_forward", "moved_m": 0.10, "stopped_short": "timeout",
           "reason": VERB_MAX_REASON}
    agent, asked = _agent([dict(cut), {"action": "drive_forward", "moved_m": 0.06}])
    agent.safety.forward_clearance = lambda *a, **k: (9.0, "scan_footprint")   # room past the line
    agent._pending = [("CREEP", ["FORWARD", "REVERSE"]), ("OPEN",)]
    agent._do_pending()
    assert agent._pending[0][0] == "CREEP"
    agent._do_pending()
    assert asked == [("FORWARD", 5), ("FORWARD", 5)]
    assert agent._pending == [("OPEN",)]         # the line reached: on to the turn


def test_a_creep_that_ends_at_the_line_is_not_repeated():
    agent, asked = _agent([{"action": "drive_forward", "moved_m": 0.16}])
    agent._pending = [("CREEP", ["FORWARD", "REVERSE"]), ("OPEN",)]
    agent._do_pending()
    assert agent._pending == [("OPEN",)]


def test_the_creep_pieces_are_bounded():
    cut = {"action": "drive_forward", "moved_m": 0.05, "stopped_short": "timeout",
           "reason": VERB_MAX_REASON}
    agent, asked = _agent([dict(cut) for _ in range(10)])
    agent.safety.forward_clearance = lambda *a, **k: (15.0, "scan_footprint")
    agent._pending = [("CREEP", ["FORWARD"]), ("OPEN",)]
    for _ in range(10):
        if agent._pending[0][0] != "CREEP":
            break
        agent._do_pending()
    assert len(asked) == CREEP_PIECES
    assert agent._pending == [("OPEN",)]


@pytest.mark.parametrize("stopped", ["clamped", "stalled", None])
def test_only_a_cut_by_time_creeps_on(stopped):
    result = {"action": "drive_forward", "moved_m": 0.10}
    if stopped:
        result["stopped_short"] = stopped
    agent, _ = _agent([result])
    agent._pending = [("CREEP", ["FORWARD"]), ("OPEN",)]
    agent._do_pending()
    assert agent._pending == [("OPEN",)]


def test_a_timeout_that_is_not_the_cap_does_not_creep_on():
    """Review (round 3): the real board's `carry_out_verb` also ends
    'timeout' when a verb runs out its own periods (reason None); only the
    VERB_MAX_S cut is a creep its time stopped short, as on ROS."""
    out_of_periods = {"action": "drive_forward", "moved_m": 0.10, "stopped_short": "timeout",
                      "reason": None}
    agent, asked = _agent([out_of_periods])
    agent.safety.forward_clearance = lambda *a, **k: (9.0, "scan_footprint")
    agent._pending = [("CREEP", ["FORWARD"]), ("OPEN",)]
    agent._do_pending()
    assert asked == [("FORWARD", 5)]
    assert agent._pending == [("OPEN",)]


def test_a_creep_cut_at_the_line_does_not_creep_on(monkeypatch):
    """Review (both reviewers' race): a creep that reached the line just
    before the cut reads as cut; with no room left past the line, no second
    piece is sent into the wall."""
    cut = {"action": "drive_forward", "moved_m": 0.16, "stopped_short": "timeout",
           "reason": VERB_MAX_REASON}
    agent, asked = _agent([dict(cut)])
    monkeypatch.setattr(agent.safety, "forward_clearance", lambda *a, **k: (3.6, "scan_footprint"))
    agent._pending = [("CREEP", ["FORWARD"]), ("OPEN",)]
    agent._do_pending()
    assert asked == [("FORWARD", 5)]
    assert agent._pending == [("OPEN",)]


def test_a_ros_verb_that_misses_its_own_deadline_reads_as_before():
    """Review (both reviewers): only the VERB_MAX_S cap reports a time cut.
    A speed-50 verb that runs out its own 3x+3 s deadline (6 s, under the
    cap) carries no `stopped_short`, as on dev, so ConstrainedAgent's
    SHORT_MOVES rule sees the ROS path exactly as before 3.89."""
    world = build_world("scaled_house")
    inner = MockRobot(world, render=False)
    robot = ros_drive.RosDriveRobot(inner, "http://bridge")

    class Crawl(rvs.SafeChain):
        def _land(self, twist):             # a tenth of every twist: slow, never stalled
            super()._land(dict(twist, linear_m_s=twist["linear_m_s"] * 0.1))
    entry = SafetyController(robot, 20.0)
    chain = Crawl(inner, entry, 0, lambda before: None)
    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(chain.handler))
    world.x, world.y = _open_spot(entry, world)
    with rvs._virtual_time(chain.clock):
        t0 = chain.clock.now
        out = entry.check_and_execute("FORWARD", speed=50, duration=0.5)
        seconds = chain.clock.now - t0
    assert seconds >= 5.9                          # ran out its own 3 x 1 + 3 s, did not stall
    assert abs(out["moved_m"]) < 0.30 - ros_drive.LINEAR_TOLERANCE_M
    assert "stopped_short" not in out


def _open_spot(entry, world):
    """A cell in the scaled house with a metre clear ahead."""
    from tests.demo_explore import free_cells
    for x, y in sorted(free_cells(world)):
        world.x, world.y = x + 0.5, y + 0.5
        cm, _ = entry.forward_clearance()
        if cm is None or cm > 100:
            return world.x, world.y
    raise AssertionError("no open spot")
