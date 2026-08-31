"""
renderer.py

The grid world's synthetic camera: a first-person raycast render of
`sim/grid_world.py`'s layout, as JPEG bytes.

This is a port of the twin's `renderFPV` (`web-twin/app.js`) into Python,
and phase S2 of `PLAN-sim-hardening.md`. **It is not a visualisation.**
Under Q1's answer the vision policy is the product, so this render is the
model's actual input on the simulated backend -- the same role a real
photograph plays on `ReplayRobot` and the Pi camera will play on
hardware.

## Why this had to move out of JavaScript

`MockRobot.get_camera_frame()` returned grid facts and no pixels, so the
vision policy could not drive the simulator at all (`AGENT-HARNESS.md`
section 1, "what still isn't there"). The only raycaster in the project
lived in the browser, which meant the one backend everything else is
tested against was the one backend the hardware-path policy could not
run on. That is `PLAN-sim-hardening.md` 2.1 -- the blocker that would
have forced `RobotInterface` to change when hardware landed, which is
exactly what `robot/factory.py`'s docstring promises will not happen.

## Fidelity: read this before trusting a result produced from these frames

These are flat-shaded, untextured walls. `PLAN-sim-hardening.md` 3.5 is
blunt about what that means: a VLM's accuracy here "tells you very little
about its accuracy on photographs of a real living room," and it is the
gap least closable in simulation. This render makes the *loop* runnable
end to end in Python -- memory, lifecycle, cost, arrival, the safety
layer with a real distance reading behind it -- and it does not make the
sim a substitute for photographing real rooms. Do both.

## Parity with the JavaScript it replaces

Every constant and every line of the geometry below is a deliberate
one-to-one port, so that the twin can show a server-rendered frame and
the picture does not change (the phase's UI proof). Two things are
*not* bit-identical and are not intended to be:

  * **Antialiasing.** `canvas.fillRect` antialiases a fractional-height
    wall column; PIL does not. Wall band edges can differ by a pixel.
  * **Font.** Object labels use PIL's default bitmap font rather than the
    browser's monospace stack.

`tests/test_renderer_parity.py` therefore compares the wall-height
profile column by column, which is what "the same view" actually means,
rather than demanding identical bytes.
"""

import base64
import io
import math

from PIL import Image, ImageDraw

# ---------- constants, ported verbatim from web-twin/app.js ----------
# The JS names are kept so the two files can be diffed by eye. Changing
# any of these without changing its twin is what the parity test exists
# to catch.

HEADING_ANGLE = {"N": -math.pi / 2, "E": 0.0, "S": math.pi / 2, "W": math.pi}
FPV_FOV = math.pi / 3  # 60 degrees
FPV_MAX_DIST = 14.0  # cells
FPV_STEP = 0.05  # ray march step, in cells
FPV_WALL_RGB = (118, 129, 150)

# From the twin's CSS custom properties. The upper half of the frame is
# painted with --wall and the lower half with --floor; that is the JS
# behaviour, odd-looking variable name and all.
COLOR_SKY = (5, 7, 10)  # --wall:  #05070A
COLOR_FLOOR = (35, 42, 51)  # --floor: #232A33
COLOR_TARGET = (255, 107, 53)  # --accent-alert: #FF6B35
COLOR_OBJECT = (84, 96, 116)  # #546074, inline in renderFPV

# The twin's canvas defaults, before it resizes itself to the viewport.
# A fixed size here keeps a rendered frame reproducible, which is what
# makes the golden-image test meaningful.
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 200

# Matches captureFPVFrame()'s toDataURL("image/jpeg", 0.82).
JPEG_QUALITY = 82

MEDIA_TYPE = "image/jpeg"

CELL_WALL = "#"


def _cell_at(layout, x: int, y: int) -> str:
    """Out of bounds reads as wall, exactly as the JS `cellAt` does."""
    if y < 0 or y >= len(layout) or x < 0 or x >= len(layout[0]):
        return CELL_WALL
    return layout[y][x]


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def cast_ray(layout, px: float, py: float, angle: float) -> float:
    """March until a wall cell, or `FPV_MAX_DIST`.

    Doors ('D') are passable, same as real movement, so a ray keeps going
    through a doorway into whatever room is beyond it -- that is what
    makes a door read as a bright gap rather than a wall.
    """
    dx, dy = math.cos(angle), math.sin(angle)
    dist = 0.0
    while dist < FPV_MAX_DIST:
        dist += FPV_STEP
        cx = math.floor(px + dx * dist)
        cy = math.floor(py + dy * dist)
        if _cell_at(layout, cx, cy) == CELL_WALL:
            return dist
    return FPV_MAX_DIST


def wall_profile(layout, px: float, py: float, base_angle: float, width=DEFAULT_WIDTH,
                 height=DEFAULT_HEIGHT):
    """The per-column geometry, before anything is painted.

    Split out from `render()` because it is the part that has to match the
    JavaScript, and comparing numbers is a far better parity test than
    comparing two rasterisations of them. Returns one
    `(wall_height, shade)` pair per screen column.
    """
    out = []
    for x in range(width):
        t = x / (width - 1)
        ray_angle = base_angle - FPV_FOV / 2 + FPV_FOV * t
        dist = cast_ray(layout, px, py, ray_angle)
        perp = max(0.15, dist * math.cos(ray_angle - base_angle))
        wall_height = min(float(height), (height * 1.1) / perp)
        shade = max(0.15, min(1.0, 1.4 - perp / FPV_MAX_DIST))
        out.append((wall_height, shade))
    return out


def _visible_objects(layout, objects, px: float, py: float, base_angle: float):
    """Objects inside the field of view and not hidden behind a wall.

    Farthest first, so `render()` can paint them in order and let nearer
    ones cover the ones behind -- the painter's algorithm the JS uses.
    """
    visible = []
    for (ox_cell, oy_cell), name in objects.items():
        ox, oy = ox_cell + 0.5, oy_cell + 0.5
        ddx, ddy = ox - px, oy - py
        dist_to_obj = math.hypot(ddx, ddy)
        angle_to_obj = math.atan2(ddy, ddx)
        rel_angle = normalize_angle(angle_to_obj - base_angle)
        if abs(rel_angle) > FPV_FOV / 2 + 0.1:
            continue
        # A wall between us and it. The 0.3 slack keeps an object sitting
        # against a wall from being culled by its own backdrop.
        if cast_ray(layout, px, py, angle_to_obj) < dist_to_obj - 0.3:
            continue
        visible.append({"name": name, "rel_angle": rel_angle, "dist": dist_to_obj})
    visible.sort(key=lambda o: o["dist"], reverse=True)
    return visible


def render(layout, objects, px: float, py: float, base_angle: float,
           width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT) -> Image.Image:
    """One first-person frame, as a PIL image.

    `px`/`py` are in cell units and are cell *centres* -- the caller adds
    the 0.5, matching the JS. `base_angle` is radians, from
    `HEADING_ANGLE`.
    """
    img = Image.new("RGB", (width, height), COLOR_SKY)
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, height // 2, width, height], fill=COLOR_FLOOR)

    for x, (wall_height, shade) in enumerate(
        wall_profile(layout, px, py, base_angle, width, height)
    ):
        color = tuple(round(c * shade) for c in FPV_WALL_RGB)
        top = (height - wall_height) / 2
        # PIL's rectangle is inclusive of both endpoints; canvas fillRect
        # takes a height. Subtracting 1 keeps a column the same number of
        # pixels tall as the JS paints.
        draw.rectangle([x, round(top), x, round(top + wall_height) - 1], fill=color)

    for obj in _visible_objects(layout, objects, px, py, base_angle):
        t = (obj["rel_angle"] + FPV_FOV / 2) / FPV_FOV
        screen_x = t * width
        perp = max(0.3, obj["dist"] * math.cos(obj["rel_angle"]))
        h = min(height * 0.55, (height * 0.65) / perp)
        w = h * 0.7
        cy = height / 2 + h * 0.08
        # The target is the one object that has to be findable, so it gets
        # the alert colour; everything else is furniture-grey.
        fill = COLOR_TARGET if "backpack" in obj["name"].lower() else COLOR_OBJECT
        draw.rectangle(
            [round(screen_x - w / 2), round(cy - h / 2),
             round(screen_x + w / 2), round(cy + h / 2)],
            fill=fill,
        )
        label = obj["name"]
        try:
            tw = draw.textlength(label)
        except AttributeError:  # very old Pillow
            tw = len(label) * 6
        draw.text((screen_x - tw / 2, cy + h / 2 + 4), label, fill=(255, 255, 255))

    return img


def render_jpeg(layout, objects, px: float, py: float, base_angle: float,
                width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT) -> bytes:
    buf = io.BytesIO()
    render(layout, objects, px, py, base_angle, width, height).save(
        buf, format="JPEG", quality=JPEG_QUALITY
    )
    return buf.getvalue()


def render_world_image(world, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT) -> Image.Image:
    """A `GridWorld`'s current pose, as a PIL image.

    Uses the world's *view* heading, not its body heading, so
    `look_left()` / `look_right()` actually change the picture -- the same
    thing `frame_description()` reports in its `facing` field, and the
    reason a peek is worth anything to a vision policy.
    """
    view = world._view_heading()
    return render(
        world.layout,
        world.objects,
        world.robot_x + 0.5,
        world.robot_y + 0.5,
        HEADING_ANGLE[view.name],
        width,
        height,
    )


def render_world(world, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT) -> bytes:
    """JPEG bytes for a `GridWorld`'s current pose."""
    buf = io.BytesIO()
    render_world_image(world, width, height).save(
        buf, format="JPEG", quality=JPEG_QUALITY
    )
    return buf.getvalue()


def render_world_base64(world, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT) -> str:
    return base64.b64encode(render_world(world, width, height)).decode()
