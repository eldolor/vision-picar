"""
Run with: pytest tests/test_vision.py -v

describe_grid_frame is pure logic and needs no API key.
describe_image is tested against a mocked Anthropic client so the whole
suite runs offline/free -- see tests/manual_describe_image.py for an
actual API call against a real photo.
"""

from unittest.mock import MagicMock, patch
import brain.vision as vision


def test_describe_grid_frame_clear_path():
    frame = {
        "room": "hallway",
        "facing": "E",
        "free_space_cells": 5,
        "doorway_ahead": False,
        "objects_visible": [],
        "position": (5, 5),
    }
    result = vision.describe_grid_frame(frame)
    assert result["free_space"] == "clear"
    assert result["safest_direction"] == "FORWARD"
    assert result["obstacles_ahead"] == []


def test_describe_grid_frame_blocked():
    frame = {
        "room": "kitchen",
        "facing": "N",
        "free_space_cells": 0,
        "doorway_ahead": False,
        "objects_visible": [],
        "position": (10, 6),
    }
    result = vision.describe_grid_frame(frame)
    assert result["free_space"] == "none"
    assert result["safest_direction"] == "STOP"
    assert "wall" in result["obstacles_ahead"]


def test_describe_grid_frame_surfaces_objects_and_doorway():
    frame = {
        "room": "kitchen",
        "facing": "S",
        "free_space_cells": 2,
        "doorway_ahead": True,
        "objects_visible": ["red backpack"],
        "position": (10, 7),
    }
    result = vision.describe_grid_frame(frame)
    assert result["doorway_visible"] is True
    assert result["important_objects"] == ["red backpack"]
    assert result["free_space"] == "some"


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
