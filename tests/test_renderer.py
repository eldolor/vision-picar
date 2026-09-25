"""
tests/test_renderer.py

Phase S2 -- `sim/renderer.py`, the grid world's synthetic camera.

**Why a golden image rather than a set of assertions about pixels.** Under
Q1's answer this render is the vision policy's actual input, so the thing
worth protecting is not any single property of it but the fact that it
does not silently change. A model's behaviour on the sim is only
comparable across runs if the picture is; a renderer edit that shifts
every wall by a pixel would otherwise be invisible here and show up as an
unexplained change in a walk score days later.

The golden is a PNG compared as raw RGB, not the JPEG
`get_camera_frame()` actually ships. JPEG is lossy and its encoder output
can move between Pillow builds, which would make this test fail for
reasons that have nothing to do with the renderer. The JPEG path is
covered by the contract suite (`test_robot_contract.py`) asserting the
bytes decode, and by `test_server.py` over HTTP.

To re-bless the golden after a deliberate change:

    python -c "from sim.maps.starter_house import build_starter_world as b; \\
               from sim import renderer as r; w=b(); \\
               r.render(w.layout, w.objects, 2.5, 2.5, \\
                        r.HEADING_ANGLE['E']).save('tests/golden/fpv_living_room_east.png')"

and look at the result before committing it.
"""

import base64
import io
import math
from pathlib import Path

import pytest
from PIL import Image

from sim import renderer
from sim.grid_world import Heading
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot

GOLDEN = Path(__file__).parent / "golden" / "fpv_living_room_east.png"

# The pose the golden was rendered from: standing in the living room
# looking east at the doorway. Chosen because one frame then contains all
# three things the renderer can draw -- near side walls, a gap where a
# door lets the ray through, and an object billboard.
POSE = (2.5, 2.5, "E")


@pytest.fixture
def world():
    return build_starter_world()


def _render_pose(world, heading="E", px=2.5, py=2.5):
    return renderer.render(
        world.layout, world.objects, px, py, renderer.HEADING_ANGLE[heading]
    )


# ---------- the golden ----------


def test_render_matches_the_golden_image(world):
    got = _render_pose(world).convert("RGB")
    expected = Image.open(GOLDEN).convert("RGB")

    assert got.size == expected.size
    assert got.tobytes() == expected.tobytes(), (
        "the render changed. If that was deliberate, re-bless "
        f"{GOLDEN.name} using the command in this file's docstring -- after "
        "looking at the new image."
    )


def test_render_is_deterministic(world):
    """Nothing in here may depend on time, iteration order or randomness:
    two identical poses must give identical bytes, or a replayed sim walk
    is not actually a replay."""
    assert _render_pose(world).tobytes() == _render_pose(world).tobytes()


# ---------- geometry ----------


def test_walls_are_nearer_at_the_edges_of_a_corridor(world):
    """Standing in a room looking down a doorway, the columns at the edges
    of the field of view hit the near side walls and the ones in the
    middle see much further. If this inverts, the ray march or the angle
    sweep is backwards -- the kind of bug a golden image would also catch
    but would not explain."""
    profile = renderer.wall_profile(
        world.layout, 2.5, 2.5, renderer.HEADING_ANGLE["E"]
    )
    edge_height = profile[0][0]
    middle_height = profile[len(profile) // 2][0]
    assert edge_height > middle_height


def test_a_ray_stops_at_a_wall_and_passes_through_a_door(world):
    """Doors are passable to movement, so they must be passable to a ray
    too -- otherwise a doorway renders as a wall and the vision policy can
    never see a reason to go through one."""
    # Due east from (2,2) is the door at (4,2), then open hallway beyond.
    through_door = renderer.cast_ray(world.layout, 2.5, 2.5, renderer.HEADING_ANGLE["E"])
    # Due north from (2,2) is the living room's outer wall, two cells off.
    into_wall = renderer.cast_ray(world.layout, 2.5, 2.5, renderer.HEADING_ANGLE["N"])
    assert through_door > into_wall
    assert into_wall < 2.0


def test_ray_distance_never_exceeds_the_configured_maximum(world):
    """An unbounded march is an infinite loop in a room with no walls."""
    for angle in [i * math.pi / 8 for i in range(16)]:
        assert renderer.cast_ray(world.layout, 2.5, 2.5, angle) <= renderer.FPV_MAX_DIST


def test_panning_the_camera_changes_the_picture(world):
    """`look_left()` has to be worth something to a vision policy. It only
    is if the render follows the view heading rather than the body's --
    the same thing frame_description()'s `facing` reports."""
    centred = renderer.render_world(world)
    world.look_left()
    panned = renderer.render_world(world)
    assert centred != panned


# ---------- objects ----------


def test_the_target_is_drawn_in_the_alert_colour(world):
    """The backpack is the one thing the robot is looking for, so it is
    the one thing that must not look like furniture. Rendered from the
    kitchen doorway, where the backpack is in view."""
    world.robot_x, world.robot_y, world.heading = 10, 8, Heading.N
    img = renderer.render_world_image(world)
    assert renderer.COLOR_TARGET in {c for _, c in img.convert("RGB").getcolors(1 << 16)}


def test_objects_behind_a_wall_are_not_drawn(world):
    """Standing in the living room, the kitchen's refrigerator is several
    rooms away and behind solid wall. Drawing it would hand the policy a
    sighting no camera could have produced."""
    visible = renderer._visible_objects(
        world.layout, world.objects, 2.5, 2.5, renderer.HEADING_ANGLE["E"]
    )
    assert "refrigerator" not in {o["name"] for o in visible}


# ---------- the MockRobot seam ----------


def test_mock_robot_frame_carries_the_rendered_image(world):
    frame = MockRobot(world).get_camera_frame()
    img = Image.open(io.BytesIO(base64.b64decode(frame["image_base64"])))
    assert img.format == "JPEG"
    assert img.size == (renderer.DEFAULT_WIDTH, renderer.DEFAULT_HEIGHT)
    assert frame["media_type"] == "image/jpeg"
    assert frame["metadata"]["source"] == "sim"


def test_render_false_is_the_free_offline_path(world):
    """The rule-based policy has no use for the picture and most of the
    suite runs it, so opting out has to actually skip the work -- and has
    to leave the grid facts exactly as they were before S2."""
    frame = MockRobot(world, render=False).get_camera_frame()
    assert "image_base64" not in frame
    assert frame == world.frame_description()


def test_perception_rides_alongside_the_pixels_and_grid_facts_do_not(world):
    """The frame carries the picture plus what the simulator's stand-in
    detector saw (`room`, `objects_visible`) -- and, since the ROS
    alignment, NONE of the grid facts. Those were answers a real robot cannot
    give in that form; pinning their absence is what stops a consumer
    growing back onto them because they happened to be there."""
    frame = MockRobot(world).get_camera_frame()
    assert "room" in frame and "objects_visible" in frame
    for key in ("position", "facing", "free_space_cells", "doorway_ahead"):
        assert key not in frame, f"grid fact {key!r} is back on the frame"
