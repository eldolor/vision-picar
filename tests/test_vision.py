"""
Run with: pytest tests/test_vision.py -v

The rule-based policy's scene (`ConstrainedAgent.sensed_scene()`) is pure
logic over a depth grid and needs no API key; it replaced the retired
grid-fact converter `describe_grid_frame` (PLAN-ros-alignment.md).
describe_image is tested against a mocked Anthropic client so the whole
suite runs offline/free -- see tests/manual_describe_image.py for an
actual API call against a real photo.
"""

from unittest.mock import MagicMock, patch
import brain.vision as vision
from brain.agent import ConstrainedAgent
from robot.interface import ZONE_NO_TARGET, ZONE_RANGE


class _DepthRobot:
    """Publishes exactly the depth grid it is given, and nothing else --
    so a scene test states one clearance and cannot pass for some other
    reason. Duck-typed, like tests/test_depth_veto.py's GridRobot."""

    def __init__(self, clearance_cm):
        zone = ({"status": ZONE_NO_TARGET, "distance_cm": None}
                if clearance_cm is None else
                {"status": ZONE_RANGE, "distance_cm": clearance_cm})
        self.grid = {"rows": 1, "cols": 8, "fov_deg": 60.0, "zones": [zone] * 8}

    def get_depth_grid(self):
        return self.grid

    def get_distance(self):
        return 999.0


def _scene(clearance_cm, frame=None):
    agent = ConstrainedAgent(_DepthRobot(clearance_cm), min_distance_cm=20.0)
    return agent.sensed_scene(frame or {"room": "hallway"})


def test_a_long_clear_path_reads_clear_and_forward():
    scene = _scene(150.0)
    assert scene["free_space"] == "clear"
    assert scene["safest_direction"] == "FORWARD"
    assert scene["obstacles_ahead"] == []


def test_the_scene_says_stop_exactly_where_the_collar_would_veto():
    """The one boundary in the scene that changes a decision, and it is the
    safety layer's own `min_distance_cm` rather than a second threshold --
    so the free policy never argues with the collar about the same wall."""
    blocked = _scene(19.0)
    assert blocked["free_space"] == "none"
    assert blocked["safest_direction"] == "STOP"
    assert blocked["obstacles_ahead"]

    allowed = _scene(21.0)
    assert allowed["free_space"] == "some"
    assert allowed["safest_direction"] == "FORWARD"


def test_nothing_within_range_is_clear_never_a_veto():
    """M3's second outcome. An empty path is a fact about the room."""
    assert _scene(None)["free_space"] == "clear"


def test_objects_come_from_perception_and_doorways_are_not_invented():
    scene = _scene(150.0, {"room": "kitchen", "objects_visible": ["red backpack"]})
    assert scene["important_objects"] == ["red backpack"]
    assert scene["doorway_visible"] is False, "nothing measures doorways"


def test_a_backend_with_no_perception_reports_no_objects():
    assert _scene(150.0, {"room": "unknown"})["important_objects"] == []


def test_a_frame_with_no_image_describes_to_the_empty_schema():
    """`describe_frame()` used to fall back to the grid-fact converter, which
    is gone. "Nothing seen" is the truth about a picture that does not
    exist -- and its `safest_direction` is STOP, failing safe."""
    scene = vision.describe_frame({"room": "hallway"})
    assert scene["important_objects"] == []
    assert scene["safest_direction"] == "STOP"


def test_parse_scene_json_handles_markdown_fence():
    raw = '```json\n{"obstacles_ahead": ["chair"], "free_space": "some", "doorway_visible": false, "important_objects": [], "safest_direction": "LEFT"}\n```'
    parsed = vision._parse_scene_json(raw)
    assert parsed["safest_direction"] == "LEFT"
    assert parsed["obstacles_ahead"] == ["chair"]


def test_parse_scene_json_handles_garbage_gracefully():
    parsed = vision._parse_scene_json("not json at all")
    assert parsed["safest_direction"] == "STOP"  # fail safe, not fail open
    assert "_raw" in parsed


@patch("brain.vision._get_client")
def test_describe_image_calls_api_and_parses_response(mock_get_client, tmp_path):
    # Fake a 1x1 pixel PNG so _image_to_base64 has a real file to read.
    fake_image = tmp_path / "test_room.png"
    fake_image.write_bytes(
        bytes.fromhex(
            "89504e470d0a1a0a0000000d4948445200000001000000010802000000"
            "907753360000000a49444154789c6360000002000100ffff03000006"
            "0005579d7a09000000004945"
        )
    )

    mock_response = MagicMock()
    mock_block = MagicMock()
    mock_block.type = "text"
    mock_block.text = (
        '{"obstacles_ahead": ["sofa"], "free_space": "some", '
        '"doorway_visible": true, "important_objects": ["lamp"], '
        '"safest_direction": "RIGHT"}'
    )
    mock_response.content = [mock_block]

    mock_client = MagicMock()
    mock_client.messages.create.return_value = mock_response
    mock_get_client.return_value = mock_client

    result = vision.describe_image(str(fake_image))

    assert result["safest_direction"] == "RIGHT"
    assert result["important_objects"] == ["lamp"]
    mock_client.messages.create.assert_called_once()
    call_kwargs = mock_client.messages.create.call_args.kwargs
    assert call_kwargs["model"] == vision.MODEL
