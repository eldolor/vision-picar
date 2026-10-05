"""
tests/test_refused_turn_loop.py

Handoff 2026-10-02, item 5a: `python -m tests.demo_active_search` ended NOT
FOUND after 150 steps, 132 of them a LEFT the pivot guard (3.19) refused.
Diagnosis (confirmed 2026-10-03): the side ray called LEFT clear at
`side_clearance_cm + 1`, the pivot guard refused the swept corner, and a
refused turn is not "executed" -- so the next `decide()` re-peeked from the
same spot and picked the same LEFT again, forever. Neither the boxed-in
fallback nor the stuck-breaker ever fired.

The rule now: a turn refused at this pose is not offered again until
something executes.
"""

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot


def _demo_run(max_steps=150, world=None):
    robot = MockRobot(build_starter_world(), render=False)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                              world=world(robot) if world else None)
    report = agent.run_mission(max_steps=max_steps)
    return agent, report


def test_a_refused_turn_is_not_chosen_again_from_the_same_spot():
    agent, _ = _demo_run()
    repeats = [
        (a.step, a.action) for a, b in zip(agent.history, agent.history[1:])
        if a.action in ("LEFT", "RIGHT") and not a.executed
        and b.action == a.action and not b.executed
    ]
    assert not repeats, f"the same refused turn was chosen again {len(repeats)} times"


def test_the_active_search_demo_finds_the_backpack():
    """The demo's other defect: it built the agent with no world, so the
    policy ran the right-hand rule instead of its frontier preference. With
    the world it is the reference hunt (tests/data/frontier_trace_centred.json:
    found, 61 steps)."""
    from tests.conftest import mock_world_for

    _, report = _demo_run(world=mock_world_for)
    assert report["found"] and report["steps_taken"] == 61, report


def test_without_a_world_the_agent_still_never_spins_on_a_refusal():
    """The right-hand-rule path (no map) is the one the loop lived in. It
    may not find the backpack in 150 steps -- that is exploration, not this
    defect -- but it must not repeat a refused turn, and it must reach the
    boxed-in fallback rather than alternate a refusal with a STOP."""
    agent, _ = _demo_run()
    stops = [h for h in agent.history if h.action == "STOP"]
    left_refused = [h for h in agent.history if h.action == "LEFT" and not h.executed]
    assert len(left_refused) < 30, f"{len(left_refused)} refused LEFTs"
    assert stops, "the boxed-in fallback never ran"
