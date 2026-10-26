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


def test_the_tick_found_fires_on_neither_counts_nor_backs_off():
    """Thermos on c18acb9: found is recorded after `_choose`, from the scene
    it already holds; the bound must not back off on that very tick."""
    agent, point, sent = _stuck()
    _tick(agent)
    _tick(agent)
    frame = agent.robot.get_camera_frame()
    agent._choose(frame, {"important_objects": ["red backpack"]})
    assert agent.retry.failures_near(*point) == 0
    assert "REVERSE" not in sent and "REVERSE" not in agent._pending


def test_the_count_belongs_to_one_place():
    """Thermos on c18acb9: ticks at one sighting must not add to another's."""
    agent, point, sent = _stuck()
    _tick(agent)
    _tick(agent)
    other = _ahead(agent, 0.64, 2.0)
    other = (other[0] + 1.0, other[1])            # a sighting a metre off
    agent._sighting = lambda frame: other
    agent._approach = other
    agent._at_target = lambda tx, ty: True
    _tick(agent)
    assert agent.retry.failures_near(*other) == 0 and agent.retry.failures_near(*point) == 0


def test_a_detector_that_flickers_does_not_livelock():
    """/code-review on 37d4930 claimed a frame that misses the target restarts
    the count for ever. It does not loop (a guard, not red on dev): a missed
    frame finds the sighting already let go and sends a frontier goal."""
    agent, point, sent = _stuck()
    seen = {"n": 0}

    def flicker(frame):
        seen["n"] += 1
        return point if seen["n"] % 2 else None
    agent._sighting = flicker
    outs = [_tick(agent) for _ in range(4 * CLOSE_IN_REFUSED)]
    assert any(o[0] == "GOAL" for o in outs) or agent.retry.failures_near(*point), outs


def test_a_tick_that_does_not_count_starts_the_count_again():
    """/code-review on cc1ccfa: the count carried over ticks that did not
    count (a queued look, a missed frame), so one stalled tick after them
    gave the sighting up. Only ticks in a row count."""
    agent, point, sent = _stuck()
    _tick(agent)
    _tick(agent)
    agent._pending = ["LOOK_CENTER"]              # a queued look runs this tick
    _tick(agent)
    _tick(agent)
    assert agent.retry.failures_near(*point) == 0
    assert "REVERSE" not in sent and "REVERSE" not in agent._pending
    for _ in range(CLOSE_IN_REFUSED):              # ... and a full run after it still fires
        _tick(agent)
    assert agent.retry.failures_near(*point) == 1
