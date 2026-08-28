"""
Phase S2b (partial) -- the vision policy, and the recorded-walk backend
it runs against.

Run with: pytest tests/test_vision_policy.py -v

No API calls: /navigate is stubbed everywhere here. What is under test is
the mapping between the service's schema and the agent's, the policy's
one decision, and whether a whole mission survives being driven by a
model's answers instead of a rule.
"""

import base64
import json

import pytest

from brain.memory import MissionMemory
from brain.navigate import FrameHasNoImage, navigate_scene, to_scene, vision_fn_for
from brain.vision_agent import VisionAgent
from control.mission_runner import FOUND, MAX_STEPS, MissionRunner
from sim.replay_robot import NO_SENSOR_CM, ReplayRobot
from tests.conftest import RecordingRobot

TARGET = "red backpack"

# One-pixel JPEG-ish payload. Nothing decodes it here -- the backend only
# has to hand bytes to a vision function, and the stub only has to receive
# them.
PIXEL = base64.b64encode(b"\xff\xd8\xff\xd9").decode()


def navigate_reply(action="FORWARD", visible=False, reached=False, obstacle=False,
                   direction="not_visible", reasoning="because"):
    return {
        "target_visible": visible, "target_direction": direction,
        "target_reached": reached, "obstacle_ahead": obstacle,
        "action": action, "reasoning": reasoning,
    }


def write_walk(tmp_path, count):
    """A directory of images standing in for a recorded walk."""
    for i in range(count):
        (tmp_path / f"frame-{i:03d}.jpg").write_bytes(base64.b64decode(PIXEL))
    return tmp_path


# ---------- the schema mapping ----------


def test_a_visible_target_is_not_a_found_target():
    """The decision this module exists to get right. MissionMemory treats
    a match in important_objects as 'found', and found ends the mission --
    so ending on first sight would stop the robot in a doorway across the
    room from the backpack and call it success."""
    seen = to_scene(navigate_reply(visible=True, direction="left"), TARGET)
    assert seen["important_objects"] == []

    arrived = to_scene(navigate_reply(visible=True, reached=True), TARGET)
    assert arrived["important_objects"] == [TARGET]


def test_visibility_is_preserved_even_though_it_is_not_completion():
    scene = to_scene(navigate_reply(visible=True, direction="right"), TARGET)
    assert scene["_navigate"]["target_visible"] is True
    assert scene["_navigate"]["target_direction"] == "right"
    assert scene["_navigate"]["reasoning"] == "because"


def test_the_action_carries_through_as_the_chosen_direction():
    for action in ("FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP"):
        assert to_scene(navigate_reply(action=action), TARGET)["safest_direction"] == action


def test_an_unknown_action_becomes_stop():
    """Matching the service's own defaulting -- a malformed answer must not
    become a movement command."""
    assert to_scene({"action": "LAUNCH"}, TARGET)["safest_direction"] == "STOP"
    assert to_scene({}, TARGET)["safest_direction"] == "STOP"


def test_an_obstacle_is_reported_as_no_free_space():
    assert to_scene(navigate_reply(obstacle=True), TARGET)["free_space"] == "none"
    assert to_scene(navigate_reply(obstacle=False), TARGET)["free_space"] == "clear"


def test_a_frame_with_no_pixels_is_a_clear_error(monkeypatch):
    """MockRobot returns a grid description. The failure should name that,
    not surface as a confusing 400 from the vision service."""
    with pytest.raises(FrameHasNoImage) as excinfo:
        navigate_scene({"room": "kitchen"}, TARGET, "http://vision.test")
    assert "S2" in str(excinfo.value)


def test_vision_fn_for_requires_a_url(monkeypatch):
    monkeypatch.delenv("VISION_URL", raising=False)
    with pytest.raises(ValueError):
        vision_fn_for(TARGET)


def test_navigate_posts_what_the_service_expects(tmp_path):
    """The request shape is a contract with service/vision_analyze/app.py."""
    import httpx

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["url"] = str(request.url)
        seen["secret"] = request.headers.get("x-app-secret")
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=navigate_reply(action="LEFT"))

    client = httpx.Client(transport=httpx.MockTransport(handler))
    scene = navigate_scene(
        {"image_base64": PIXEL, "media_type": "image/jpeg"},
        TARGET, "http://vision.test/", secret="s3cret", client=client,
    )

    assert seen["url"] == "http://vision.test/navigate"
    assert seen["secret"] == "s3cret"
    assert seen["body"] == {
        "image_base64": PIXEL, "media_type": "image/jpeg", "target_object": TARGET,
    }
    assert scene["safest_direction"] == "LEFT"


# ---------- the policy ----------


def test_the_policy_does_what_the_model_says(tmp_path):
    robot = ReplayRobot(write_walk(tmp_path, 3))
    memory = MissionMemory(mission="find it", target_object=TARGET)
    agent = VisionAgent(robot, memory)
    assert agent.decide(to_scene(navigate_reply(action="RIGHT"), TARGET)) == "RIGHT"


def test_the_policy_does_not_peek(tmp_path):
    """The rule-based policy spends three pans and three distance reads per
    decision. Against a camera that costs real moves and buys nothing."""
    robot = RecordingRobot(ReplayRobot(write_walk(tmp_path, 4)))
    memory = MissionMemory(mission="find it", target_object=TARGET)
    agent = VisionAgent(robot, memory, vision_fn=lambda f: to_scene(navigate_reply(), TARGET))

    agent.step()

    assert "look_left" not in robot.calls and "look_right" not in robot.calls


def test_the_policy_stops_once_the_mission_is_complete(tmp_path):
    robot = ReplayRobot(write_walk(tmp_path, 2))
    memory = MissionMemory(mission="find it", target_object=TARGET)
    memory.found = True
    agent = VisionAgent(robot, memory)
    assert agent.decide(to_scene(navigate_reply(action="FORWARD"), TARGET)) == "STOP"


def test_the_stuck_breaker_still_applies(tmp_path):
    """Inherited from ConstrainedAgent: three STOPs in a row forces a turn,
    so a model that keeps answering STOP cannot freeze the mission."""
    robot = ReplayRobot(write_walk(tmp_path, 8))
    memory = MissionMemory(mission="find it", target_object=TARGET)
    agent = VisionAgent(robot, memory, vision_fn=lambda f: to_scene(navigate_reply(action="STOP"), TARGET))

    actions = [agent.step().action for _ in range(5)]

    assert actions[:3] == ["STOP", "STOP", "STOP"]
    assert "RIGHT" in actions[3:]


# ---------- the recorded-walk backend ----------


def test_a_replay_frame_carries_pixels(tmp_path):
    robot = ReplayRobot(write_walk(tmp_path, 3))
    frame = robot.get_camera_frame()

    assert frame["image_base64"] == PIXEL
    assert frame["media_type"] == "image/jpeg"
    assert frame["room"] == "unknown", "a photograph has no room label"
    assert frame["metadata"]["frames_total"] == 3


def test_movement_advances_the_walk_and_pans_do_not(tmp_path):
    robot = ReplayRobot(write_walk(tmp_path, 3))
    assert robot.get_camera_frame()["metadata"]["index"] == 0

    robot.drive_forward()
    assert robot.get_camera_frame()["metadata"]["index"] == 1

    robot.look_left()
    robot.look_right()
    assert robot.get_camera_frame()["metadata"]["index"] == 1, "a pan is not a step"

    robot.turn_right()
    assert robot.get_camera_frame()["metadata"]["index"] == 2


def test_running_off_the_end_holds_rather_than_raising(tmp_path):
    """A paid run should finish on its own terms -- the step budget or the
    arrival signal -- not die with a traceback halfway through."""
    robot = ReplayRobot(write_walk(tmp_path, 2))
    for _ in range(6):
        robot.drive_forward()

    assert robot.exhausted is True
    assert robot.get_camera_frame()["metadata"]["index"] == 1


def test_an_empty_directory_is_refused(tmp_path):
    with pytest.raises(ValueError):
        ReplayRobot(tmp_path)


def test_the_replay_has_no_distance_sensor(tmp_path):
    """And says so with a value no threshold can veto, rather than
    inventing a number that would make a replay look like it had exercised
    the safety layer."""
    robot = ReplayRobot(write_walk(tmp_path, 1))
    assert robot.get_distance() == NO_SENSOR_CM
    assert NO_SENSOR_CM > 100


# ---------- a whole mission, driven by the model ----------


def scripted_vision(replies):
    """A stub /navigate that answers from a list, then keeps answering the
    last one."""
    calls = {"n": 0}

    def vision_fn(frame):
        assert frame.get("image_base64"), "the policy was handed a frame with no pixels"
        reply = replies[min(calls["n"], len(replies) - 1)]
        calls["n"] += 1
        return to_scene(reply, TARGET)

    vision_fn.calls = calls
    return vision_fn


def test_a_full_mission_runs_on_recorded_frames(tmp_path):
    """The first mission the Python brain can run on real pixels: the model
    walks the recording and arrives."""
    robot = RecordingRobot(ReplayRobot(write_walk(tmp_path, 6)))
    vision_fn = scripted_vision([
        navigate_reply(action="FORWARD"),
        navigate_reply(action="LEFT", visible=True, direction="left"),
        navigate_reply(action="FORWARD", visible=True, direction="center"),
        navigate_reply(action="STOP", visible=True, reached=True, reasoning="arrived"),
    ])
    runner = MissionRunner(
        robot, target_object=TARGET, policy="vision", vision_fn=vision_fn, max_steps=20
    )
    runner.start()
    while runner.tick():
        pass
    status = runner.status()

    assert status["outcome"] == FOUND
    assert status["step"] == 4, "one step per vision call, no peeking"
    assert vision_fn.calls["n"] == 4, "one paid call per step -- the cost model"
    assert status["sighting"]["object_name"] == TARGET
    assert "arrived" in status["last_reasoning"], "the model's own words reach the panel"
    assert "stop" in robot.calls, "a finished mission stops the robot"


def test_the_step_budget_bounds_the_bill(tmp_path):
    """Every step is a paid call, so max_steps is the cost cap. A model
    that never reports arrival must not run forever."""
    robot = ReplayRobot(write_walk(tmp_path, 3), loop=True)
    vision_fn = scripted_vision([navigate_reply(action="FORWARD")])
    runner = MissionRunner(
        robot, target_object=TARGET, policy="vision", vision_fn=vision_fn, max_steps=5
    )
    runner.start()
    while runner.tick():
        pass

    assert runner.status()["outcome"] == MAX_STEPS
    assert vision_fn.calls["n"] == 5


def test_a_flaky_service_ends_the_mission_rather_than_walking_blind(tmp_path):
    """The B3.2 budget, exercised through the real vision path."""
    robot = RecordingRobot(ReplayRobot(write_walk(tmp_path, 6)))

    def failing(frame):
        raise RuntimeError("navigate: 502")

    runner = MissionRunner(
        robot, target_object=TARGET, policy="vision", vision_fn=failing,
        max_steps=20, max_vision_failures=3,
    )
    runner.start()
    while runner.tick():
        pass

    assert runner.status()["outcome"] == "failed"
    assert "stop" in robot.calls
    assert "drive_forward" not in robot.calls
