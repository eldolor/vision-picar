"""
tests/chassis_fit.py

`PLAN-ros-alignment.md` 3.21, criterion 2: **does a chassis fit a house?**
Asked before a purchase, so it is asked of GROUND TRUTH only -- the house's
occupied cells (walls and solid objects, read off the layout) against a
rectangle -- and shares no code with the safety layer, the sim's collision
or nav2, any of which could be wrong in a way that hides a misfit.

**Method: the configuration space.** For each heading (every 5 degrees) the
rectangle is convolved over the occupancy image at 2.5 cm, giving the
centre positions where the body touches nothing. Adjacent positions at one
heading, and adjacent headings at one position, are joined; the component
holding the start pose is everything the body can get to. That is a
statement about a differential drive too, not only a holonomic one: with
forward, reverse and pivot a diff drive is small-time locally controllable,
so any path-connected free C-space is reachable by it.

Resolution, stated rather than hidden: a pixel is occupied-by-the-body when
its CENTRE is inside the rectangle, so every clearance here is good to
about +/-1.25 cm; a 5-degree heading step lets a corner at 17 cm sweep
~1.5 cm between checks. Both chassis are judged by the same instrument, so
the comparison is fair even where the absolute edge is soft.

    python -m tests.chassis_fit              # every house, both chassis
"""

import math

import numpy as np
from scipy import ndimage, signal

from sim import renderer
from sim.grid_world import CELL_WALL
from sim.maps import build_world

CELL_M = 0.30
RES_M = 0.025
PX = round(CELL_M / RES_M)          # pixels per cell
HEADINGS = 72                       # every 5 degrees

# Truth copies, written out so the instrument does not move with the code
# under test. (length, width) across the wheels, metres.
OLD_CHASSIS = (0.228, 0.198)        # the 2WD Yahboom build, 3.18-3.19
UGV_ROVER = (0.253, 0.231)          # Waveshare UGV Rover, 253 x 231 mm [V]

# The safety layer's side margin (robot/safety.py FOOTPRINT_SIDE_MARGIN_CM):
# a passage narrower than the body plus this on each side is one the
# corridor check will refuse to drive through.
SAFETY_MARGIN_M = 0.03


def occupancy(world) -> np.ndarray:
    """True where a wall or a solid object is, at RES_M."""
    h, w = len(world.layout), len(world.layout[0])
    cells = np.zeros((h, w), bool)
    for cy in range(h):
        for cx in range(w):
            if renderer._cell_at(world.layout, cx, cy) == CELL_WALL or (cx, cy) in world.solid_cells:
                cells[cy, cx] = True
    img = np.kron(cells, np.ones((PX, PX), bool))
    # One cell of wall around the whole map: off the layout is not floor.
    return np.pad(img, PX, constant_values=True)


def kernel(length_m: float, width_m: float, theta: float) -> np.ndarray:
    """The rectangle at `theta`, as the pixels whose centres it contains."""
    hl, hw = length_m / 2 / RES_M, width_m / 2 / RES_M
    r = math.ceil(math.hypot(hl, hw)) + 1
    ys, xs = np.mgrid[-r:r + 1, -r:r + 1]
    ux, uy = math.cos(theta), math.sin(theta)
    along = xs * ux + ys * uy
    across = -xs * uy + ys * ux
    return (np.abs(along) <= hl) & (np.abs(across) <= hw)


def free_space(occ: np.ndarray, length_m: float, width_m: float) -> np.ndarray:
    """(HEADINGS, H, W): True where the body's centre can sit at that heading."""
    occ_f = occ.astype(np.float32)
    out = np.empty((HEADINGS,) + occ.shape, bool)
    for k in range(HEADINGS):
        ker = kernel(length_m, width_m, 2 * math.pi * k / HEADINGS).astype(np.float32)
        out[k] = signal.fftconvolve(occ_f, ker, mode="same") < 0.5
    return out


def reachable(free: np.ndarray, start_px, start_k: int) -> np.ndarray:
    """The C-space component holding the start: bool (HEADINGS, H, W)."""
    labels, _ = ndimage.label(free)   # 6-connected: x, y, and heading
    # Heading is a circle: join the last slice to the first.
    parent = {}

    def find(a):
        while parent.get(a, a) != a:
            a = parent[a]
        return a

    both = free[0] & free[-1]
    for a, b in set(zip(labels[0][both].tolist(), labels[-1][both].tolist())):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    sy, sx = start_px
    start = labels[start_k, sy, sx]
    if start == 0:
        return np.zeros_like(free)
    root = find(start)
    keep = [lab for lab in np.unique(labels) if lab and find(lab) == root]
    return np.isin(labels, keep)


def fit(house: str, length_m: float, width_m: float, margin_m: float = 0.0) -> dict:
    world = build_world(house)
    occ = occupancy(world)
    free = free_space(occ, length_m + 2 * margin_m, width_m + 2 * margin_m)
    start_px = (round((world.y + 1) * PX), round((world.x + 1) * PX))
    start_k = round(world.theta / (2 * math.pi / HEADINGS)) % HEADINGS
    comp = reachable(free, start_px, start_k)
    where = comp.any(axis=0)           # any heading
    spin = comp.all(axis=0)            # every heading: can turn right round
    rooms = {}
    for name, cells in (world.rooms or {}).items():
        mask = np.zeros_like(where)
        for cx, cy in cells:
            mask[(cy + 1) * PX:(cy + 2) * PX, (cx + 1) * PX:(cx + 2) * PX] = True
        rooms[name] = {"reached": bool((where & mask).any()),
                       "spin_m2": float((spin & mask).sum() * RES_M ** 2)}
    return {"house": house, "chassis": (length_m, width_m), "margin_m": margin_m,
            "start_free": bool(free[start_k, start_px[0], start_px[1]]),
            "rooms": rooms}


def main():
    for house in ("starter_house", "scaled_house", "home_first_floor"):
        for margin in (0.0, SAFETY_MARGIN_M):
            old = fit(house, *OLD_CHASSIS, margin)
            new = fit(house, *UGV_ROVER, margin)
            print(f"\n{house}, margin {margin * 100:.0f} cm   "
                  f"(start free: old {old['start_free']}, new {new['start_free']})")
            print(f"  {'room':<14} {'old reached':>11} {'new reached':>11} "
                  f"{'old spin m2':>11} {'new spin m2':>11}")
            for name in old["rooms"]:
                o, n = old["rooms"][name], new["rooms"][name]
                flag = "   <-- LOST" if o["reached"] and not n["reached"] else ""
                print(f"  {name:<14} {str(o['reached']):>11} {str(n['reached']):>11} "
                      f"{o['spin_m2']:>11.2f} {n['spin_m2']:>11.2f}{flag}")


if __name__ == "__main__":
    main()
