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
                   direction="not_visible", reasoning="because", room_guess="unclear"):
    return {
        "target_visible": visible, "target_direction": direction,
        "target_reached": reached, "obstacle_ahead": obstacle,
        "room_guess": room_guess, "action": action, "reasoning": reasoning,
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


def test_an_absent_obstacle_field_is_unknown_not_clear():
    """The "bearing-only" variant (M1) deletes the obstacle question, so the
    service omits the field rather than sending a default for it. Reading
    that as "clear" would put words in the mouth of a model that was never
    asked -- and it is the same collapse M3 refuses for a failed depth zone:
    "nothing is there" and "I could not tell" are different answers."""
    reply = navigate_reply()
    del reply["obstacle_ahead"]

    scene = to_scene(reply, TARGET)

    assert scene["free_space"] == "unknown"
    assert scene["obstacles_ahead"] == []
    assert scene["_navigate"]["obstacle_ahead"] is None


def test_an_absent_obstacle_field_does_not_stop_the_policy_deciding():
    """The point of removing the question is that the model still answers the
    other four. A missing obstacle field must cost nothing else in the
    mapping -- the only obstacle logic left on the path is robot/safety.py's
    get_distance() re-check before every FORWARD."""
    reply = navigate_reply(action="FORWARD", visible=True, direction="center")
    del reply["obstacle_ahead"]

    scene = to_scene(reply, TARGET)

    assert scene["safest_direction"] == "FORWARD"
    assert scene["_navigate"]["target_direction"] == "center"


def test_a_frame_with_no_pixels_is_a_clear_error(monkeypatch):
    """Every backend carries pixels since S2, so this is now the
    render=False offline path or a broken backend -- either way it should
    say so here, not surface as a confusing 400 from the vision service."""
    with pytest.raises(FrameHasNoImage) as excinfo:
        navigate_scene({"room": "kitchen"}, TARGET, "http://vision.test")
    assert "image_base64" in str(excinfo.value)


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


# ---------- room-level step memory (AGENT-HARNESS.md section 12) ----------


def test_room_guess_reaches_the_scene():
    scene = to_scene(navigate_reply(room_guess="kitchen"), TARGET)
    assert scene["_navigate"]["room_guess"] == "kitchen"


def test_a_missing_or_blank_room_guess_is_unclear_not_a_crash():
    assert to_scene({}, TARGET)["_navigate"]["room_guess"] == "unclear"
    assert to_scene(navigate_reply(room_guess=""), TARGET)["_navigate"]["room_guess"] == "unclear"
    assert to_scene({"room_guess": 7}, TARGET)["_navigate"]["room_guess"] == "unclear"


def test_searched_rooms_is_omitted_when_empty(tmp_path):
    """Backward compatible: a caller that never opts in sends exactly the
    request test_navigate_posts_what_the_service_expects already pins."""
    import httpx

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=navigate_reply())

    client = httpx.Client(transport=httpx.MockTransport(handler))
    navigate_scene(
        {"image_base64": PIXEL}, TARGET, "http://vision.test/", client=client, searched_rooms=[],
    )
    assert "searched_rooms" not in seen["body"]


def test_searched_rooms_is_sent_when_present(tmp_path):
    import httpx

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=navigate_reply())

    client = httpx.Client(transport=httpx.MockTransport(handler))
    navigate_scene(
        {"image_base64": PIXEL}, TARGET, "http://vision.test/", client=client,
        searched_rooms=["kitchen", "hallway"],
    )
    assert seen["body"]["searched_rooms"] == ["kitchen", "hallway"]


def test_vision_fn_for_exposes_a_searched_rooms_setter(monkeypatch):
    """The mechanism control/mission_runner.py's _guarded_vision() relies
    on: a mutable attribute on the callable, not a second positional arg --
    so the vision_fn(frame) -> scene contract every other seam depends on
    is untouched."""
    import httpx

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=navigate_reply())

    client = httpx.Client(transport=httpx.MockTransport(handler))
    vision_fn = vision_fn_for(TARGET, vision_url="http://vision.test/", client=client)

    assert hasattr(vision_fn, "set_searched_rooms")
    vision_fn({"image_base64": PIXEL})
    assert "searched_rooms" not in seen["body"], "nothing searched yet"

    vision_fn.set_searched_rooms({"kitchen"})  # a set, like MissionMemory.searched_rooms
    vision_fn({"image_base64": PIXEL})
    assert seen["body"]["searched_rooms"] == ["kitchen"]


def test_mission_agent_backfills_room_from_the_scenes_room_guess(tmp_path):
    """The other half of the loop: MissionMemory can only track rooms a
    vision-driven mission visits if something turns the model's room_guess
    into frame["room"], since a real/replayed camera frame never carries
    one of its own."""
    robot = ReplayRobot(write_walk(tmp_path, 2))
    memory = MissionMemory(mission="find it", target_object=TARGET)
    agent = VisionAgent(
        robot, memory,
        vision_fn=lambda f: to_scene(navigate_reply(room_guess="kitchen"), TARGET),
    )

    result = agent.step()

    assert result.frame["room"] == "kitchen"
    assert "kitchen" in memory.visited_rooms
    assert "kitchen" in memory.searched_rooms


def test_mission_agent_leaves_a_real_room_label_alone(tmp_path):
    """Sim frames already carry a real room -- the backfill must never
    override ground truth with a model guess."""
    from sim.maps.starter_house import build_starter_world
    from sim.mock_robot import MockRobot

    robot = MockRobot(build_starter_world())
    memory = MissionMemory(mission="find it", target_object=TARGET)
    agent = VisionAgent(
        robot, memory,
        vision_fn=lambda f: to_scene(navigate_reply(room_guess="kitchen"), TARGET),
    )

    result = agent.step()

    assert result.frame["room"] != "kitchen"


def test_a_mission_tells_the_service_what_it_has_already_searched(tmp_path):
    """End to end through MissionRunner: once a room is marked searched,
    the next vision call carries it."""
    robot = RecordingRobot(ReplayRobot(write_walk(tmp_path, 4)))
    seen_searched_rooms = []

    def vision_fn(frame):
        return to_scene(navigate_reply(action="FORWARD", room_guess="kitchen"), TARGET)

    def set_searched_rooms(rooms):
        seen_searched_rooms.append(list(rooms))

    vision_fn.set_searched_rooms = set_searched_rooms

    runner = MissionRunner(
        robot, target_object=TARGET, policy="vision", vision_fn=vision_fn, max_steps=3
    )
    runner.start()
    while runner.tick():
        pass

    assert seen_searched_rooms[0] == [], "nothing searched before the first call"
    assert seen_searched_rooms[-1] == ["kitchen"], "kitchen was marked searched after step 1"


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


# ---------- choosing the model on the wire ----------


def _capture_body(reply=None):
    """A MockTransport that records the request body it was given."""
    import httpx

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=reply or navigate_reply())

    return seen, httpx.Client(transport=httpx.MockTransport(handler))


def test_a_chosen_model_reaches_the_service():
    seen, client = _capture_body()
    navigate_scene({"image_base64": "aGk="}, TARGET, "http://vision.test",
                   client=client, model_id="us.anthropic.claude-opus-4-5-20251101-v1:0")
    assert seen["body"]["model_id"] == "us.anthropic.claude-opus-4-5-20251101-v1:0"


def test_no_model_omits_the_field_entirely():
    """Absent means "the service's own default". Sending null, or a model
    name this module guessed, would both be wrong -- the allow-list and the
    default both live in the vision service."""
    seen, client = _capture_body()
    navigate_scene({"image_base64": "aGk="}, TARGET, "http://vision.test", client=client)
    assert "model_id" not in seen["body"]


def test_vision_fn_for_binds_the_model_without_changing_the_call_contract():
    """The whole point of binding it in the factory: the harness still calls
    vision_fn(frame) with one argument (AGENT-HARNESS.md section 10)."""
    seen, client = _capture_body()
    fn = vision_fn_for(TARGET, vision_url="http://vision.test", client=client,
                       model_id="qwen.qwen3-vl-235b-a22b")

    fn({"image_base64": "aGk="})

    assert seen["body"]["model_id"] == "qwen.qwen3-vl-235b-a22b"


def test_a_chosen_prompt_variant_reaches_the_service():
    """The other lever. Wording moved one model's FORWARD rate from 0.000 to
    1.000 in this project's own 3x3 matrix, so a mission that cannot name its
    variant is a mission whose result cannot be attributed."""
    seen, client = _capture_body()
    navigate_scene({"image_base64": "aGk="}, TARGET, "http://vision.test",
                   client=client, prompt_variant="bearing-only")
    assert seen["body"]["prompt_variant"] == "bearing-only"


def test_no_prompt_variant_omits_the_field_entirely():
    """Same contract as model_id: absent means "the service's own default",
    and the allow-list lives there, not here."""
    seen, client = _capture_body()
    navigate_scene({"image_base64": "aGk="}, TARGET, "http://vision.test", client=client)
    assert "prompt_variant" not in seen["body"]


def test_vision_fn_for_binds_the_prompt_variant_too():
    """Bound in the factory alongside the model, so the harness still calls
    vision_fn(frame) with one argument (AGENT-HARNESS.md section 10)."""
    seen, client = _capture_body()
    fn = vision_fn_for(TARGET, vision_url="http://vision.test", client=client,
                       prompt_variant="center-third-path")

    fn({"image_base64": "aGk="})

    assert seen["body"]["prompt_variant"] == "center-third-path"


def test_a_non_200_from_the_vision_service_is_a_clear_error():
    """The failure MissionRunner's B3.2 budget counts. It has to carry the
    status and the body, or a mission dies reporting nothing useful."""
    import httpx

    def handler(request):
        return httpx.Response(503, text="upstream unavailable")

    client = httpx.Client(transport=httpx.MockTransport(handler))
    with pytest.raises(RuntimeError) as e:
        navigate_scene({"image_base64": "aGk="}, TARGET, "http://vision.test", client=client)
    assert "503" in str(e.value)
    assert "upstream unavailable" in str(e.value)


def test_a_client_this_module_created_is_closed_again():
    """navigate_scene() builds its own httpx.Client when none is passed, and
    has to close it -- the vision policy calls this once per tick for the
    length of a mission."""
    import httpx

    created = []
    real = httpx.Client

    class Tracking(real):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            created.append(self)

    httpx.Client = Tracking
    try:
        try:
            navigate_scene({"image_base64": "aGk="}, TARGET, "http://127.0.0.1:1")
        except Exception:
            pass
    finally:
        httpx.Client = real

    assert created and created[0].is_closed


# ---------- phase S2: the vision policy drives the simulator ----------


def test_the_vision_policy_can_now_drive_the_grid_world():
    """S2's whole point, stated as a test.

    Before the raycaster moved into Python this was impossible: MockRobot
    had no pixels, so `navigate_scene()` raised `FrameHasNoImage` on the
    very first tick and the only backend the vision policy could run
    against was a recorded walk. `AGENT-HARNESS.md` section 1 listed it as
    the headline gap; this is the assertion that closes it.

    The service is stubbed -- what is under test is that real rendered
    frames reach it, not what a model would say about them.
    """
    import httpx

    from sim.maps.starter_house import build_starter_world
    from sim.mock_robot import MockRobot

    sent = []

    def handler(request: httpx.Request) -> httpx.Response:
        sent.append(json.loads(request.content))
        return httpx.Response(200, json=navigate_reply(action="FORWARD"))

    client = httpx.Client(transport=httpx.MockTransport(handler))
    robot = MockRobot(build_starter_world())

    def vision_fn(frame):
        return to_scene(
            navigate_scene(frame, TARGET, "http://vision.test", client=client),
            TARGET,
        )

    runner = MissionRunner(robot=robot, target_object=TARGET, vision_fn=vision_fn,
                           policy="vision")
    runner.start()
    for _ in range(3):
        runner.tick()

    assert sent, "no frame reached the vision service"
    for body in sent:
        assert body["image_base64"], "the sim sent a frame with no pixels"
        assert body["media_type"] == "image/jpeg"

    # And the pixels are a real image, not the grid dict stringified.
    import io
    from PIL import Image

    img = Image.open(io.BytesIO(base64.b64decode(sent[0]["image_base64"])))
    assert img.format == "JPEG" and img.size[0] > 0
