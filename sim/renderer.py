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
from typing import Optional

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


def cast_ray(layout, px: float, py: float, angle: float, solid=None,
             max_dist: float = FPV_MAX_DIST) -> float:
    """March until a wall cell, or `max_dist` (the camera's horizon by default).

    Doors ('D') are passable, same as real movement, so a ray keeps going
    through a doorway into whatever room is beyond it -- that is what
    makes a door read as a bright gap rather than a wall.

    `solid` is a set of `(x, y)` cells that also stop the ray -- the
    objects, since 2026-09-26 (`PLAN-ros-alignment.md` 3.9). Passed by
    everything that SENSES or MOVES (collision, distance, depth, lidar, the
    map) and deliberately NOT by the camera's wall profile, which draws
    objects as billboards rather than as grey wall blocks. None keeps the
    wall-only behaviour, so the picture is unchanged.
    """
    dx, dy = math.cos(angle), math.sin(angle)
    dist = 0.0
    while dist < max_dist:
        dist += FPV_STEP
        cx = math.floor(px + dx * dist)
        cy = math.floor(py + dy * dist)
        if _cell_at(layout, cx, cy) == CELL_WALL:
            return dist
        if solid and (cx, cy) in solid and (cx, cy) != (math.floor(px), math.floor(py)):
            return dist
    return max_dist


def cast_ray_exact(layout, px: float, py: float, angle: float, solid=None,
                   max_dist: float = FPV_MAX_DIST) -> float:
    """The exact distance, in cells, to the first wall (or `solid`) cell
    along the ray -- a grid traversal (Amanatides & Woo) rather than
    `cast_ray()`'s fixed-step march. PLAN-ros-alignment.md 3.18.

    Two differences from `cast_ray()`, both in the safe direction. It is
    exact where the march over-reads by up to one `FPV_STEP` (1.5cm), and a
    ray through the shared corner of two diagonal cells counts as a hit,
    where the march can step between them. Used where a range must never
    be overstated (the safety layer's short scan); the map and the picture
    keep `cast_ray()`, whose numbers everything else is pinned to. It also
    visits a handful of cells where the march takes ~40 steps a metre, which
    is the other reason it exists: the safety scan runs every control period.
    """
    dx, dy = math.cos(angle), math.sin(angle)
    cx, cy = math.floor(px), math.floor(py)
    own = (cx, cy)
    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1
    t_max_x = ((cx + 1 - px) / dx if dx > 0 else (px - cx) / -dx) if dx != 0 else math.inf
    t_max_y = ((cy + 1 - py) / dy if dy > 0 else (py - cy) / -dy) if dy != 0 else math.inf
    t_dx = abs(1 / dx) if dx != 0 else math.inf
    t_dy = abs(1 / dy) if dy != 0 else math.inf
    while True:
        if t_max_x < t_max_y:
            t, cx, t_max_x = t_max_x, cx + step_x, t_max_x + t_dx
        else:
            t, cy, t_max_y = t_max_y, cy + step_y, t_max_y + t_dy
        if t >= max_dist:
            return max_dist
        if _cell_at(layout, cx, cy) == CELL_WALL:
            return t
        if solid and (cx, cy) in solid and (cx, cy) != own:
            return t


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


VISIBLE_EXTENT_RAYS = 9


def _ray_box(px: float, py: float, angle: float, x0: float, y0: float) -> Optional[float]:
    """Distance along the ray to the unit square at (x0, y0), or None."""
    dx, dy = math.cos(angle), math.sin(angle)
    t_lo, t_hi = -math.inf, math.inf
    for p, d, lo in ((px, dx, x0), (py, dy, y0)):
        if abs(d) < 1e-12:
            if not lo <= p <= lo + 1:
                return None
            continue
        a, b = (lo - p) / d, (lo + 1 - p) / d
        t_lo, t_hi = max(t_lo, min(a, b)), min(t_hi, max(a, b))
    return t_lo if t_hi >= max(t_lo, 0.0) else None


def visible_bearing(layout, px: float, py: float, base_angle: float, cell,
                    solid=None) -> Optional[float]:
    """Where a detector's box would centre on a one-cell object: the
    relative bearing (radians, positive clockwise in the grid's frame, as
    `_visible_objects`' `rel_angle`) of the centre of the part of its face
    the camera can see past the walls -- or None if no part is visible in
    the field of view. PLAN-ros-alignment.md 3.32.

    `_visible_objects()` asks one question per object -- does a ray to its
    CENTRE clear the walls -- and reports the centre's bearing. A detector's
    box covers only what is visible, so for an object half behind a door
    jamb this samples rays across its angular extent and averages the ones
    that reach it. Used for the synthetic DETECTIONS only; the picture keeps
    the single-ray billboard (the golden image is unchanged).

    `solid`: the OTHER objects' cells. Objects are solid (3.9), so a person
    standing in front of the robot hides the backpack behind them as a wall
    does -- found while building 3.31, where the camera saw the backpack
    straight through a person and the arrival rule took the person's range
    for the backpack's."""
    x0, y0 = cell
    corners = [(x0, y0), (x0 + 1, y0), (x0 + 1, y0 + 1), (x0, y0 + 1)]
    angs = [normalize_angle(math.atan2(cy - py, cx - px) - base_angle) for cx, cy in corners]
    lo, hi = max(min(angs), -FPV_FOV / 2), min(max(angs), FPV_FOV / 2)
    if lo > hi:
        return None
    seen = []
    for k in range(VISIBLE_EXTENT_RAYS):
        a = lo + (hi - lo) * (k + 0.5) / VISIBLE_EXTENT_RAYS
        d_box = _ray_box(px, py, base_angle + a, x0, y0)
        if d_box is None:
            continue
        if cast_ray_exact(layout, px, py, base_angle + a, solid=solid,
                          max_dist=d_box + 0.5) >= d_box - 1e-6:
            seen.append(a)
    return sum(seen) / len(seen) if seen else None


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
        # against a wall from being culled by its own backdrop. Another
        # OBJECT between us and it hides it too: objects are solid (3.9), and
        # a person standing in front of the robot hides the backpack behind
        # them (found while building 3.31). One ray down the middle here;
        # `visible_bearing` samples the extent for the detections.
        others = {c for c in objects if c != (ox_cell, oy_cell)}
        if cast_ray(layout, px, py, angle_to_obj, solid=others) < dist_to_obj - 0.3:
            continue
        visible.append({"name": name, "rel_angle": rel_angle, "dist": dist_to_obj,
                        "cell": (ox_cell, oy_cell)})
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

    visible = _visible_objects(layout, objects, px, py, base_angle)
    # One label per piece, on its NEAREST visible cell. A piece of furniture
    # spans many cells (the home's staircase is 33), and labelling each one
    # smeared a word across the frame -- bad for the person watching and for
    # a vision model reading the picture.
    nearest = {}
    for obj in visible:
        if obj["name"] not in nearest or obj["dist"] < nearest[obj["name"]]["dist"]:
            nearest[obj["name"]] = obj
    for obj in visible:
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
        if nearest[obj["name"]] is not obj:
            continue
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
