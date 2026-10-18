"""
tests/test_review_376l2.py

`PLAN-ros-alignment.md` 3.76, step L2: the bugs the review of the brain's
decision code (`brain/agent.py`, `frontier.py`, `goal_policy.py`,
`goal_pose.py`, `memory.py`, `mission_report.py`, `room_labels.py`,
`rooms.py`, `vision_agent.py`, whole files) confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names.
"""

from brain.agent import MissionAgent
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot

TARGET = "red backpack"


# ---------- L2.2: the cloud's room names, as given ----------


def _with_room_guesses(*guesses):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    answers = iter(guesses)

    def vision(frame):
        return {"safest_direction": "STOP", "important_objects": [],
                "_navigate": {"room_guess": next(answers)}}

    agent = MissionAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         vision_fn=vision)
    for _ in guesses:
        frame = robot.get_camera_frame()
        frame.pop("room", None)                      # a real camera's frame
        robot.get_camera_frame = (lambda f=frame: dict(f))
        agent.step()
    return agent.memory


def test_one_room_named_two_ways_is_one_room_and_unclear_is_none():
    """`room_guess` is free text ("short room-type label"); only the exact
    string "unclear" was dropped, so "Living room" and "living room" were two
    rooms in the report and "Unclear" was a room."""
    memory = _with_room_guesses("Living room", "living room", "Unclear", "unknown")
    assert memory.visited_rooms == {"living room"}, memory.visited_rooms


# ---------- L2.5: the stuck-breaker on the frame the target is found ----------


def test_the_found_frame_is_not_turned_away_from():
    """`arrived_scene` answers STOP; after three STOPs the stuck-breaker made
    it an executed RIGHT on the very frame memory then marked found."""
    from brain.agent import ConstrainedAgent, StepResult
    grid = build_world("scaled_house")
    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET))
    agent.history = [StepResult(step=i, frame={}, scene={}, action="STOP",
                                executed=True, detail="") for i in range(3)]
    scene = {"safest_direction": "STOP", "_navigate": {"target_reached": True}}
    assert ConstrainedAgent.decide(agent, scene) == "STOP"


# ---------- L2.3: a seed off the grid took the map's corner ----------


def test_a_seed_off_the_grid_names_nothing():
    """`cut.cell(...) or (-1, -1)`: a seed outside the current map (a smaller
    map after a session change) became cell (-1, -1), which `_floor_near`
    clamps to floor near the corner -- so it owned, and named, the floor
    there, and suppressed asks in it (both reviewers)."""
    import numpy as np
    from brain.room_labels import _walk
    seg = np.zeros((40, 40), dtype=int)               # one room, all floor
    d, owner = _walk(seg, [(-1, -1)], 0.05, 3.0)     # what both callers passed
    assert (owner < 0).all(), "a seed off the grid owns floor"


# ---------- L2.6: a Stop after the local check read as a refusal ----------


def test_a_stop_after_the_local_check_is_undecided_not_refused():
    """The return for a Stop landing after the local identity check lacked
    `undecided`, unlike the same returns before and after it, so the agent
    recorded a refusal (and a refused place) for a cancelled mission."""
    from control.mission_runner import MissionRunner
    runner = MissionRunner(MockRobot(build_world("scaled_house"), render=False),
                           target_object=TARGET, policy="tiered", vision_fn=lambda f: {},
                           vision_timeout_s=2.0, arrival_confirm_timeout_s=1.0)

    class Policy:
        local_identity_fn = object()

        def confirm_arrival(self, frame, local_ok=True):
            raise AssertionError("not asked")

        def confirm_local(self, frame, *a):
            runner.stop("the operator pressed Stop")
            return None                      # absent: handed to the cloud

        def confirm_cloud(self, frame, guarded=False):
            raise AssertionError("the cloud was asked after a Stop")

        def close(self):
            pass

    runner.vision_fn = Policy()
    runner.start()
    verdict = runner._guarded_confirm({})
    assert verdict["cloud_called"] is False and verdict.get("undecided") is True, verdict


# ---------- L2.4: the camera's seen set under a moving map origin ----------


def test_the_same_view_on_a_shifted_origin_adds_no_cells():
    """Seen cells were keyed by metre centres worked out from the map's
    origin. A SLAM map's origin (the minimum of its rotated corners) moves
    by fractions of a cell as it grows, so the same floor was added again
    under new keys after each shift -- the set (and `camera_seen_m2`) grew
    without bound and every goal choice rebuilt it."""
    from tests.test_tiered_explore import _agent
    agent, _ = _agent()
    m = agent.world.get_map()
    agent._pan = lambda: 0.0
    agent.world.get_map = lambda: m
    agent._note_seen()
    once = len(agent.seen)
    assert once > 0
    for shift in (0.013, 0.031, 0.047):
        shifted = dict(m, origin_x_m=m["origin_x_m"] + shift, origin_y_m=m["origin_y_m"] - shift)
        agent.world.get_map = (lambda s=shifted: s)
        agent._note_seen()
    assert len(agent.seen) < 1.6 * once, (once, len(agent.seen))


def test_seen_cells_map_back_to_the_cells_the_camera_saw_on_a_half_cell_origin():
    """Fixes' round 1 (/code-review): a lattice fixed at the house's 0 put its
    centres on the map's cell borders when the origin sat on a half cell (an
    anchored SLAM map can), so cells collided and `_to_cell` gave the
    neighbour -- 151 of 400 wrong. The lattice is the first map's own."""
    from brain.frontier import camera_seen
    from tests.test_tiered_explore import _agent
    agent, _ = _agent()
    m = agent.world.get_map()
    dx, dy = -29.5 * 0.3, 9.5 * 0.3              # half-cell origins (the sim map: 0.3 m cells)
    half = dict(m, origin_x_m=m["origin_x_m"] + dx, origin_y_m=m["origin_y_m"] + dy)
    real = agent._pose()
    pose = dict(real, x_m=real["x_m"] + dx, y_m=real["y_m"] + dy)   # the same view
    agent._pose = lambda: pose
    agent._pan = lambda: 0.0
    agent.world.get_map = lambda: half
    saw = set(camera_seen(half, pose["x_m"], pose["y_m"], pose["heading_deg"]))
    agent._note_seen()
    back = {agent._to_cell(half, c) for c in agent.seen}
    assert back == saw, (len(saw), len(back), len(saw ^ back))
