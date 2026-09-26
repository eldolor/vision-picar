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

**One specific way it was worse than "flat-shaded" has been measured and
fixed (2026-09-02).** M1's two closed-loop missions -- 40 paid steps each,
Opus 4.5, two prompt wordings -- ended 13 and 14 cells from a target
neither ever saw, and nearly every decision's reasoning said some version
of *"the image is very dark and unclear"* or *"a blank gray wall"*. The
model was right. The upper and lower halves of the frame were being
painted with the twin's `--wall` (#05070A) and `--floor` (#232A33) CSS
variables -- two near-blacks picked for dark UI chrome -- so a room read as
a black void with two grey slabs in it, and the policy spun looking for a
view it never got.

The ceiling and floor are now lit (`COLOR_CEILING` / `COLOR_FLOOR` below),
and this file no longer borrows the app's theme, so restyling the twin
cannot silently change what the model sees.

**That fix is unmeasured.** It makes the frames legible to a human eye;
whether it makes a closed-loop run able to discriminate between wordings,
models or policies is a question for the next paid run, and until that run
happens `CLAUDE.md`'s replay table is still the instrument for anything
about what the model *sees*. What a sim run measures reliably either way is
the *loop*: cost, wall clock, the budgets, and the safety veto firing on a
real distance reading.

The other half of M1's diagnosis is untouched and still open: the starter
house's start pose faces a near wall, so even a well-lit first frame shows
very little of the room. That is a map question, not a renderer one.

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

# The room the walls stand in. **These are deliberately no longer the twin's
# CSS custom properties**, which is the change that made these frames usable.
#
# The FPV render borrowed `--wall` (#05070A) for the upper half and `--floor`
# (#232A33) for the lower half -- two near-blacks chosen for a dark UI
# chrome, not for something a model has to read. M1's closed-loop runs
# measured the consequence: the ceiling and floor came back as black, the
# model reported "very dark and unclear" or "a blank gray wall" on nearly
# every frame, and two 40-step missions never found a target that was in the
# next room. This render is the policy's *input*, so it is lit like a room
# rather than themed like a panel.
#
# `renderFPV` in web-twin/app.js carries the same two constants, for the same
# reason it carries every other one on this page -- see "Parity" above. It no
# longer reads them from CSS either, so restyling the twin cannot silently
# change what the model sees.
COLOR_CEILING = (198, 203, 211)  # upper half: a lit ceiling, not a night sky
COLOR_FLOOR = (128, 120, 110)  # lower half: floor, distinct from the walls
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


_ROBOT_FOOTPRINT_CELLS = 0.5


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
        # Inside the robot's own footprint: underneath it, not in front of
        # it. The angle to a point at distance zero is atan2(0, 0) = 0, which
        # this test then read as DEAD AHEAD -- so a robot standing on the
        # starter house's sofa cell saw a sofa filling the whole picture,
        # the cloud was shown that on every mission's first call, and the
        # golden image had been blessed with it. Half a cell because that is
        # the footprint `sim/grid_world.py`'s mover uses (ROBOT_HALF_CELL;
        # not imported -- grid_world imports this module).
        if dist_to_obj <= _ROBOT_FOOTPRINT_CELLS:
            continue
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
    img = Image.new("RGB", (width, height), COLOR_CEILING)
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

    Uses the world's *view* angle, not its body heading, so `look_left()` /
    `look_right()` actually change the picture -- the same thing
    `frame_description()` reports in its `facing` field, and the reason a
    peek is worth anything to a vision policy.

    **Continuous as of R0.** This read `HEADING_ANGLE[view.name]` and a cell
    centre, which was the whole reason the twin could not show a
    non-cardinal pose; it now takes the world's real `x`/`y`/`view_angle()`.
    `HEADING_ANGLE` stays where it is -- it is the line-for-line port of the
    JavaScript's own table, and `tests/test_renderer_parity.py` compares
    against it directly. On a cardinal pose the two agree exactly, so no
    rendered frame changed.
    """
    return render(
        world.layout,
        world.objects,
        world.x,
        world.y,
        world.view_angle(),
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
