"""
tests/test_in_sight_livelock.py

`PLAN-ros-alignment.md` 3.106: explore inside its approach band with the
target in sight, no goal live and nothing to close in with (the frame's
sightings: close-in is the tiers' perception path only), and the target
never found -- house `train:1395`, where a table leg hides the box's centre
from the simulator's found rule while its detector still reports the box.
Every tick faced it ("already facing it") and nothing counted a failure.
Now CLOSE_IN_REFUSED such ticks count one and the robot backs off.
"""

import math

from brain.explore import CLOSE_IN_REFUSED
from tests.test_explore import _kitchen_door


def _ahead(agent, d, off_deg=0.0):
    p = agent._pose()
    h = math.radians(p["heading_deg"] + off_deg)
    return p["x_m"] + d * math.sin(h), p["y_m"] - d * math.cos(h)


def _stuck(off_deg=0.0):
    """In the band (0.64 m), the target in sight every tick and never found."""
    agent, nav, grid = _kitchen_door(5)
    point = _ahead(agent, 0.64, off_deg)
    agent._sighting = lambda frame: point
    agent._approach = point                      # approached already: not `moved`
    sent = []
    real = agent.safety.check_and_execute

    def spy(action, **kw):
        sent.append(action)
        return real(action, **kw)
    agent.safety.check_and_execute = spy
    return agent, point, sent


def _tick(agent):
    return agent._choose(agent.robot.get_camera_frame(), {})


def test_in_sight_in_the_band_and_never_found_is_bounded():
    agent, point, sent = _stuck()
    outs = [_tick(agent) for _ in range(CLOSE_IN_REFUSED + 3)]
    assert agent.retry.failures_near(*point) == 1, [o[2] for o in outs]
    assert "REVERSE" in sent or "REVERSE" in agent._pending, (sent, agent._pending)
    # After it, the sighting is on the retry rule's cooldown: explore moves
    # on rather than facing it again (the stub still reports it in sight).
    after = [o[2] for o in outs[CLOSE_IN_REFUSED:]]
    assert not any("already facing it" in a for a in after[1:]), after


def test_a_target_at_the_frames_edge_is_still_faced_first():
    """The bound must not skip the turn that finds a target at the edge."""
    agent, point, sent = _stuck(off_deg=20.0)
    first = _tick(agent)
    assert first[0] in ("LEFT", "RIGHT"), first
    assert agent.retry.failures_near(*point) == 0
