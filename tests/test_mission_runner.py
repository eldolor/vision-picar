"""
Phase B1 -- MissionRunner.

Run with: pytest tests/test_mission_runner.py -v

The point of these tests is that turning the loop inside out changed
nothing about the mission: a runner driven a tick at a time reaches the
same result, in the same number of steps, as ObjectSearchAgent's blocking
run_mission() -- which is what tests/demo_active_search.py exercises.
"""

import pytest

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from control.mission_runner import (
    FOUND,
    IDLE,
    MAX_STEPS,
    RUNNING,
    STOPPED,
    MissionRunner,
)
from tests.conftest import RecordingRobot, fresh_mock_robot

BUDGET = 150


def drive_to_completion(runner, max_ticks=500):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


def test_ticking_matches_the_blocking_run_mission():
    """demo_active_search.py's mission, both ways."""
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(fresh_mock_robot(), memory, min_distance_cm=30)
    demo_report = agent.run_mission(max_steps=BUDGET)

    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == FOUND
    assert status["found"] is True
    assert status["step"] == demo_report["steps_taken"]
    assert status["rooms_searched"] == demo_report["rooms_searched"]
    assert status["sighting"]["object_name"] == demo_report["sighting"].object_name
    assert status["sighting"]["room"] == demo_report["sighting"].room


def test_status_before_start_is_idle():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == IDLE
    assert status["step"] == 0
    assert status["found"] is False


def test_status_while_running_reports_progress():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    runner.start()
    assert runner.status()["outcome"] == RUNNING
    for _ in range(5):
        runner.tick()

    status = runner.status()
    assert status["running"] is True
    assert status["step"] == 5
    assert status["last_action"] in {"FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP",
                                     "LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER"}
    assert status["last_reasoning"]
    assert status["rooms_visited"]
    assert len(status["log_tail"]) >= 5


def test_stop_mid_mission_halts_and_stops_the_car():
    """Stopping the thinking is not stopping the robot -- stop() has to
    do both."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot, target_object="red backpack", max_steps=BUDGET)
    runner.start()
    for _ in range(5):
        runner.tick()

    robot.calls.clear()
    runner.stop()

    assert "stop" in robot.calls
    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == STOPPED
    assert status["complete"] is False
    assert runner.tick() is False, "a stopped runner must not keep stepping"
    assert runner.status()["step"] == 5


def test_stop_is_idempotent_and_keeps_the_first_outcome():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    runner.start()
    runner.tick()
    runner.stop()
    runner.abort("should not overwrite a completed mission")
    assert runner.status()["outcome"] == STOPPED


def test_step_budget_ends_the_mission():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=4)
    runner.start()
    status = drive_to_completion(runner)

    assert status["outcome"] == MAX_STEPS
    assert status["step"] == 4
    assert status["found"] is False


def test_room_target_completes_on_arrival():
    runner = MissionRunner(fresh_mock_robot(), target_room="hallway", max_steps=BUDGET)
    runner.start()
    status = drive_to_completion(runner)

    assert status["room_reached"] is True
    assert status["outcome"] == "room_reached"
    assert "hallway" in status["rooms_visited"]


def test_double_start_is_rejected():
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack")
    runner.start()
    with pytest.raises(RuntimeError):
        runner.start()


def test_a_mission_with_no_target_is_rejected():
    """MissionMemory.is_complete() is never true without one, so such a
    mission can only ever end at the step budget."""
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot())


def test_vision_policy_requires_a_vision_fn():
    """The default vision_fn is the offline grid converter. Running the
    vision policy on top of it would produce a mission that looks like it
    used the model and did not."""
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot(), target_object="red backpack", policy="vision")


def test_unknown_policy_is_rejected():
    with pytest.raises(ValueError):
        MissionRunner(fresh_mock_robot(), target_object="x", policy="telepathy")


def test_runner_works_the_same_over_http(robot_over_asgi):
    """The runner never touches a backend directly -- swapping in a
    RemoteRobot must not change the mission."""
    local = MissionRunner(fresh_mock_robot(), target_object="red backpack", max_steps=BUDGET)
    local.start()
    local_status = drive_to_completion(local)

    remote = MissionRunner(robot_over_asgi, target_object="red backpack", max_steps=BUDGET)
    remote.start()
    remote_status = drive_to_completion(remote)

    assert remote_status["outcome"] == local_status["outcome"] == FOUND
    assert remote_status["step"] == local_status["step"]
    assert remote_status["sighting"] == local_status["sighting"]


# ---------- the halt gate, the one-shot rule, and the failure paths ----------
#
# These are the guards that make "stop stops the car" true, and the ones that
# decide whether a mission dies loudly or quietly. Each was reachable only by
# a failure nobody triggers on purpose.


def test_reverse_is_refused_once_a_mission_is_over():
    """The gate covers every movement verb, not just the ones the frontier
    policy happens to use -- a vision policy can emit REVERSE."""
    from control.mission_runner import MissionHalted, _HaltGate

    gate = _HaltGate(fresh_mock_robot(), is_running=lambda: False)
    for call in (gate.drive_forward, gate.reverse, gate.turn_left, gate.turn_right):
        with pytest.raises(MissionHalted):
            call()


def test_a_runner_refuses_to_be_started_twice():
    """One mission per runner. Restarting one would reuse a MissionMemory
    that already believes rooms are searched and the target found."""
    runner = MissionRunner(robot=fresh_mock_robot(), target_object="red backpack",
                           max_steps=2)
    runner.start()
    runner.stop()
    with pytest.raises(RuntimeError) as e:
        runner.start()
    assert "build a new one" in str(e.value)


def test_the_step_budget_ends_the_mission():
    """max_steps is the last of the three budgets and the only one that
    fires on a mission that is working, just not fast enough."""
    runner = MissionRunner(robot=fresh_mock_robot(),
                           target_object="an object that is not in this house",
                           max_steps=3)
    runner.start()
    for _ in range(12):
        if not runner.tick():
            break
    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == "max_steps", status
    assert "budget" in " ".join(status["log_tail"]).lower()


def test_a_policy_that_raises_ends_the_mission_rather_than_the_process():
    """Anything the decision layer throws that is not a vision failure --
    a bug in the agent, a bad frame -- must still stop the robot and end
    the mission, not escape into the brain's event loop."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot=robot, target_object="red backpack", max_steps=5)
    runner.start()

    class Exploding:
        history = []

        def step(self):
            raise ZeroDivisionError("policy bug")

    runner.agent = Exploding()
    assert runner.tick() is False

    status = runner.status()
    assert status["running"] is False
    assert status["outcome"] == "failed"
    assert "policy bug" in (status["error"] or ""), status
    assert "stop" in robot.calls


def test_ticking_a_stopped_mission_is_a_no_op():
    runner = MissionRunner(robot=fresh_mock_robot(), target_object="x", max_steps=5)
    runner.start()
    runner.stop()
    assert runner.tick() is False


def test_a_failing_stop_is_logged_rather_than_raised():
    """stop() is what every failsafe calls last. If it throws, the failsafe
    that was trying to end the mission dies instead."""
    class BadStop(RecordingRobot):
        def stop(self):
            raise RuntimeError("motor controller offline")

    runner = MissionRunner(robot=BadStop(fresh_mock_robot()), target_object="x", max_steps=5)
    runner.start()
    runner.stop("operator")

    assert runner.status()["running"] is False
    assert any("stop command failed" in line for line in runner.status()["log_tail"])


def test_a_room_and_an_object_produce_one_mission_sentence():
    from control.mission_runner import MissionRunner as MR

    assert MR._default_mission("red backpack", "kitchen") == \
        "Find the red backpack in the kitchen."


def test_a_vision_call_with_no_timeout_runs_inline():
    """timeout_s <= 0 means "no dead-man" -- used by the in-process tests and
    by a policy that does its own timing."""
    from control.mission_runner import call_with_timeout

    assert call_with_timeout(lambda x: x * 2, 21, timeout_s=0) == 42
    assert call_with_timeout(lambda x: x * 2, 21, timeout_s=None) == 42


def test_reverse_passes_through_the_gate_while_a_mission_runs():
    """The gate wraps every verb; the allowed path needs covering too, or
    only the refusal is proven."""
    from control.mission_runner import _HaltGate

    robot = RecordingRobot(fresh_mock_robot())
    gate = _HaltGate(robot, is_running=lambda: True)
    gate.reverse()
    assert "reverse" in robot.calls


def test_a_stop_landing_mid_tick_does_not_record_the_step():
    """The race the halt gate exists for: stop() arrives while a step is in
    flight. The gate refuses the movement, and the tick must not then report
    the mission as still running."""
    robot = RecordingRobot(fresh_mock_robot())
    runner = MissionRunner(robot=robot, target_object="red backpack", max_steps=10)
    runner.start()

    class StopsMidStep:
        history = []

        def step(self_inner):
            runner.stop("operator stopped mid-step")

            class Result:
                step = 1
                action = "FORWARD"
                executed = True
                scene = {}

            return Result()

    runner.agent = StopsMidStep()
    assert runner.tick() is False
    assert runner.status()["running"] is False
    assert "stop" in robot.calls


# ---------- policy: "tiered" (PLAN-onboard-perception.md 4.10, phase P2) ----------
#
# The runner's whole involvement with the tiered policy is that it is a
# vision policy whose vision_fn happens to have a trigger discipline
# inside it -- 2.6's first invariant is that `control/` never learns
# perception grew a tier. So what is worth pinning here is exactly two
# things: that the policy is refused without a vision_fn the same way
# "vision" is, and that whatever the scene publishes about the tier
# reaches status() intact rather than being recomputed on this side.


def test_the_tiered_policy_requires_a_vision_fn_too():
    """Same reason as the vision policy's: the default vision_fn is the
    offline grid converter, and a mission that ran on it would look like it
    used the models and did not."""
    from control.mission_runner import POLICIES

    assert "tiered" in POLICIES
    with pytest.raises(ValueError) as exc:
        MissionRunner(fresh_mock_robot(), target_object="red backpack", policy="tiered")
    # Not merely "it raised": an unknown policy raises too, and this test
    # would then pass against a build where "tiered" does not exist at all.
    assert "vision_fn" in str(exc.value), str(exc.value)


def _tier_scene(*, cloud_called, trigger=None, frames=1, calls=1, status="absent"):
    """A scene shaped exactly as brain/tiered.py emits one."""
    return {
        "obstacles_ahead": [], "free_space": "unknown", "doorway_visible": False,
        "important_objects": [], "safest_direction": "RIGHT",
        "_navigate": {"target_visible": False, "target_direction": "not_visible",
                      "target_reached": False, "obstacle_ahead": None,
                      "room_guess": "unclear", "distance_estimate": "unknown",
                      "reasoning": "because"},
        "_perception": {"status": status, "crop_source": "label_gate",
                        "match_margin": 0.21, "similarity": 0.31,
                        "label": "backpack", "candidates": 2,
                        "synthesised": False, "bearing_deg": 4.0},
        "_tier": {"cloud_called": cloud_called, "trigger": trigger,
                  "models": {"detector": "yolo11s.pt", "scorer": "RN50"},
                  "stats": {"frames": frames, "cloud_calls": calls,
                            "frames_per_call": frames / calls if calls else None,
                            "triggers": {}, "perception": {}}},
    }


def _run_one_tick(scenes):
    """One runner, one tick per scene, driven by a stub agent."""
    runner = MissionRunner(robot=fresh_mock_robot(), target_object="red backpack",
                           max_steps=50)
    runner.start()

    class Stub:
        history = []

        def __init__(self):
            self.i = 0

        def step(self):
            scene = scenes[self.i]
            self.i += 1
            Stub.history = list(range(self.i))

            class Result:
                step = self.i
                action = "RIGHT"
                executed = True
            Result.scene = scene
            Result.frame = {"room": "living_room", "facing": "north"}
            return Result()

    runner.agent = Stub()
    for _ in scenes:
        runner.tick()
    return runner


def test_the_deliberation_counter_reaches_the_status_panel():
    """6.3 calls this *"the single number that makes the whole architecture
    watchable"*. It is watchable only if it gets out of the vision_fn --
    which is the one thing this side of the seam has to do."""
    runner = _run_one_tick([_tier_scene(cloud_called=True, trigger="mission_start",
                                        frames=1, calls=1),
                            _tier_scene(cloud_called=False, frames=2, calls=1)])
    status = runner.status()
    assert status["tier"]["stats"]["cloud_calls"] == 1
    assert status["tier"]["stats"]["frames"] == 2
    assert status["tier"]["stats"]["frames_per_call"] == 2.0


def test_the_tri_state_and_the_clip_margin_reach_the_status_panel():
    """6.3's detector readout: the tri-state so a wedged capture never looks
    like a missing target, and the margin -- which is the number that means
    something, not the bare similarity."""
    runner = _run_one_tick([_tier_scene(cloud_called=True, trigger="mission_start",
                                        status="detected")])
    perception = runner.status()["perception"]
    assert perception["status"] == "detected"
    assert perception["match_margin"] == 0.21
    assert perception["label"] == "backpack"


def test_the_detector_name_reaches_the_status_panel():
    """*"Swap the HEF and the name on screen changes; that is the experiment
    loop made watchable."*"""
    runner = _run_one_tick([_tier_scene(cloud_called=True, trigger="mission_start")])
    assert runner.status()["tier"]["models"]["detector"] == "yolo11s.pt"


def test_a_policy_with_no_perception_tier_reports_none_rather_than_zero():
    """An absent readout has to mean "this policy has no perception tier",
    not "it perceived nothing" -- the same distinction the tri-state itself
    exists for, one layer up."""
    runner = MissionRunner(robot=fresh_mock_robot(), target_object="red backpack",
                           max_steps=3)
    runner.start()
    runner.tick()
    status = runner.status()
    assert status["tier"] is None
    assert status["perception"] is None


def test_the_log_says_which_steps_cost_money():
    """Under this policy most steps are free, and a log that did not
    distinguish them would hide the entire point of the architecture."""
    runner = _run_one_tick([_tier_scene(cloud_called=True, trigger="candidate_sighting"),
                            _tier_scene(cloud_called=False, frames=2, calls=1)])
    lines = runner.status()["log_tail"]
    assert any("[cloud: candidate_sighting]" in line for line in lines), lines
    assert sum("[cloud:" in line for line in lines) == 1, lines


def test_the_counters_survive_a_step_whose_scene_carries_no_tier():
    """The counter blinking out mid-mission would be worse than it being
    absent: it reads as the architecture having stopped."""
    runner = _run_one_tick([_tier_scene(cloud_called=True, trigger="mission_start",
                                        frames=1, calls=1),
                            {"safest_direction": "STOP", "important_objects": []}])
    assert runner.status()["tier"]["stats"]["cloud_calls"] == 1


# ---------- which frame a decision was made on (teleop alignment) ----------
#
# Added 2026-09-08, after a walk nearly produced a false published finding.
# Under "Drive via brain" the twin pushes frames on one timer, polls
# /mission/status on another, and the mission ticks on a third: measured at a
# median of 2.5 pushed frames per mission step, range 1-10. So the status a
# recorder saves beside frame N routinely describes a decision taken on an
# earlier frame.
#
# On walk woven-laundry-basket-20260908-212719 that read as both tiers
# confabulating a laundry basket at P=0.998 against a bare wall. Frame-exact
# perception showed the opposite: the decision was correct and had been filed
# against frames captured seconds later. A corpus that cannot say which frame
# a decision saw cannot be scored per-frame at all.


class StampingRobot:
    """A mock robot whose frames carry a teleop sequence number, the way
    sim/teleop_robot.py stamps every pushed frame."""

    def __init__(self):
        self._inner = fresh_mock_robot()
        self.seq = 40

    def get_camera_frame(self):
        frame = dict(self._inner.get_camera_frame())
        self.seq += 1
        frame["metadata"] = {"source": "teleop", "seq": self.seq}
        return frame

    def __getattr__(self, name):
        return getattr(self._inner, name)


def test_the_status_names_the_frame_the_last_decision_saw():
    """sim/teleop_robot.py stamps every pushed frame with a sequence number.
    Carrying it through is what turns a coincidental pairing into an exact
    one."""
    runner = MissionRunner(StampingRobot(), target_object="red backpack",
                           max_steps=BUDGET)
    runner.start()
    runner.tick()
    first = runner.status()["last_frame_seq"]
    runner.tick()
    second = runner.status()["last_frame_seq"]
    assert first == 41, first
    assert second == 42, second


def test_a_backend_that_stamps_no_frame_id_reports_none_rather_than_a_guess():
    """The sim and a replay have no teleop sequence. Reporting 0, or the step
    number, would imply an alignment the walk does not have -- which is the
    exact failure this field exists to prevent."""
    runner = MissionRunner(fresh_mock_robot(), target_object="red backpack",
                           max_steps=BUDGET)
    runner.start()
    runner.tick()
    assert runner.status()["last_frame_seq"] is None
