"""remap -- does a reloaded map need re-mapping? (3.50.D4)

A matched reload localises on the saved map (3.50.D3). In localisation mode
slam_toolbox's /map keeps the saved obstacles and only fills unknown cells
(the D4 probe), so nav2 never learns that an object moved. Two of D4's three
triggers are judged here, against the map AS RELOADED (`SavedMap`, a frozen
copy of the first /map after `ready`); the third is a person's request.

* **Entered unknown** (`entered_unknown`): the robot's centre is in a cell
  the saved map calls unknown, or off its edge.
* **The scans contradict the map** (`Contradictions`), per scan:
  - *something new:* a return with no occupied and no unknown cell within
    NEW_TOL_M on the saved map;
  - *something gone:* a beam whose return lies more than GONE_SHORT_M past
    the first occupied cell the saved map puts on its line; every occupied
    cell from there to GONE_SHORT_M short of the return is contradicted. A
    beam meeting that first surface at under MIN_INCIDENCE_DEG is not judged
    at all: a beam grazing a wall crosses its cells with nothing gone.
  A cell is persistent when, over WINDOW_S, at least MIN_LOOKS scans looked
  at it, at least MIN_SHARE of those contradicted the map there, and its
  contradictions span at least HOLD_S (amendment 1: 10 looks at 10 Hz is
  one second, which a person standing for one step passes); the
  trigger fires on MIN_GROUP persistent cells within GROUP_GAP_M of each
  other. A scan whose returns agree with the map under LOST_BELOW is not
  used and is counted `lost`: a slipped frame is not a changed house, and
  a re-map would save the slip.

Every number was written in the plan before any measurement ("Decided by
the user, 2026-10-23: D4 is option 1 plus option 3"). Plain Python and
numpy (no rclpy), so it is tested on a laptop.
"""

import math
from collections import deque
from typing import Optional, Sequence

import numpy as np

from picar_bridge import relocalise

MAX_RANGE_M = 4.0
NEW_TOL_M = relocalise.AGREE_TOL_M          # 0.10
GONE_SHORT_M = 0.15
MIN_INCIDENCE_DEG = 20.0
WINDOW_S = 10.0
MIN_LOOKS = 10
MIN_SHARE = 0.8
HOLD_S = 5.0
MIN_GROUP = 10
GROUP_GAP_M = 0.15
LOST_BELOW = relocalise.AGREE_MIN           # 0.60
OCCUPIED_AT = relocalise.OCCUPIED_AT        # 65

FREE, OCC, UNKNOWN = 0, 1, 2


def _dilate(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """Every cell whose centre is within `radius_cells` of a True cell's."""
    out = mask.copy()
    h, w = mask.shape
    reach = int(math.ceil(radius_cells - 1e-9))
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if (dr == 0 and dc == 0) or dr * dr + dc * dc > radius_cells * radius_cells + 1e-9:
                continue
            src = mask[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            out[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    return out


class SavedMap:
    """The map as reloaded, frozen: a bridge map record (relocalise's form)."""

    def __init__(self, grid: dict):
        self.res = float(grid["resolution_m"])
        self.w, self.h = int(grid["width"]), int(grid["height"])
        self.ox, self.oy = float(grid["origin_x_m"]), float(grid["origin_y_m"])
        self.oyaw = float(grid.get("origin_yaw_rad") or 0.0)
        data = np.asarray(grid["data"], dtype=np.int16).reshape(self.h, self.w)
        self.state = np.full(data.shape, FREE, dtype=np.int8)
        self.state[data >= OCCUPIED_AT] = OCC
        self.state[data < 0] = UNKNOWN
        # The tolerance relocalise judges a reload with (review): a return
        # within NEW_TOL_M of an occupied cell's square, i.e. its centre
        # within NEW_TOL_M + res/sqrt(2).
        reach = NEW_TOL_M / self.res + 1 / math.sqrt(2)
        self.near_occ = _dilate(self.state == OCC, reach)
        self.near_unknown = _dilate(self.state == UNKNOWN, reach)

    def cells(self, x, y):
        """(col, row) arrays of world points; off the grid stays off it."""
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        dx, dy = np.asarray(x) - self.ox, np.asarray(y) - self.oy
        gx, gy = c * dx - s * dy, s * dx + c * dy
        return np.floor(gx / self.res).astype(np.int64), np.floor(gy / self.res).astype(np.int64)

    def inside(self, col, row):
        return (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)


def entered_unknown(saved: SavedMap, x: float, y: float) -> bool:
    """Trigger 1: the robot's centre on a cell the saved map calls unknown
    (or off its edge: the saved map never saw it either)."""
    col, row = saved.cells(x, y)
    col, row = int(col), int(row)
    if not (0 <= col < saved.w and 0 <= row < saved.h):
        return True
    return bool(saved.state[row, col] == UNKNOWN)


def judge_scan(saved: SavedMap, scan: dict, laser_pose: Sequence[float]) -> dict:
    """One scan against the saved map. Returns the agreement, and flat cell
    indices: `looked` (every cell the scan said something about) and
    `contra` (those it contradicted). `scan` is the bridge's record
    (counter-clockwise, None for no return)."""
    x0, y0, yaw = (float(v) for v in laser_pose)
    r = np.array([np.nan if v is None else float(v) for v in scan["ranges_m"]], dtype=float)
    n = r.size
    a = yaw + scan["angle_min_rad"] + np.arange(n) * scan["angle_increment_rad"]
    valid = np.isfinite(r) & (r > 0)
    ca, sa = np.cos(a), np.sin(a)
    empty = np.zeros(0, dtype=np.int64)
    out = {"agreement": None, "looked": empty, "contra": empty, "new": 0, "gone": 0}

    # Agreement, as relocalise judges a reload, over returns not near unknown.
    hx, hy = x0 + np.where(valid, r, 0) * ca, y0 + np.where(valid, r, 0) * sa
    hc, hr = saved.cells(hx, hy)
    hin = valid & saved.inside(hc, hr)
    hcc, hrr = np.where(hin, hc, 0), np.where(hin, hr, 0)
    near_occ = hin & saved.near_occ[hrr, hcc]
    judgeable = valid & ~(hin & saved.near_unknown[hrr, hcc]) & hin
    if judgeable.sum() == 0:
        return out
    agree = float((near_occ & judgeable).sum()) / float(judgeable.sum())
    out["agreement"] = agree
    if agree < LOST_BELOW:
        return out

    use = valid & (r <= MAX_RANGE_M)
    # Something new: a near return in free space, far from anything known.
    new = use & hin & ~near_occ & ~saved.near_unknown[hrr, hcc]
    flat_ret = hrr * saved.w + hcc

    # March each beam over the saved map for its expected first hit.
    step = saved.res / 2.0
    d = np.arange(1, int((MAX_RANGE_M + GONE_SHORT_M) / step) + 1) * step   # (S,)
    px = x0 + ca[:, None] * d[None, :]
    py = y0 + sa[:, None] * d[None, :]
    pc, pr = saved.cells(px, py)
    pin = saved.inside(pc, pr)
    st = np.full(pc.shape, UNKNOWN, dtype=np.int8)
    st[pin] = saved.state[pr[pin], pc[pin]]
    hit = st != FREE                        # first occupied or unknown on the line
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    rows = np.arange(n)
    has_e = first >= 0
    e_state = np.where(has_e, st[rows, np.maximum(first, 0)], UNKNOWN)
    e = np.where(has_e, d[np.maximum(first, 0)], np.inf)
    e_ok = has_e & (e_state == OCC) & (e <= MAX_RANGE_M)
    # Incidence from the expected ranges of the neighbouring beams:
    # tan(incidence) = r / |dr/dtheta|.
    inc = abs(scan["angle_increment_rad"]) or 1e-9
    e_fin = np.where(e_ok, e, np.nan)
    with np.errstate(invalid="ignore"):
        drdt = np.abs(np.roll(e_fin, -1) - np.roll(e_fin, 1)) / (2 * inc)
        if abs(n * inc - 2 * math.pi) > inc / 2:
            drdt[0] = drdt[-1] = np.nan      # not a full turn: the ends have one neighbour
        incidence = np.degrees(np.arctan2(e_fin, drdt))
    square = e_ok & np.isfinite(incidence) & (incidence >= MIN_INCIDENCE_DEG)
    e_idx = np.maximum(first, 0)
    flat_e = pr[rows, e_idx] * saved.w + pc[rows, e_idx]
    gone = use & square & (r > e + GONE_SHORT_M)
    seen_e = use & square & ((np.abs(r - e) <= NEW_TOL_M) | gone)

    # Free cells a beam crossed clear of its own return: looked at, unchanged.
    clear = pin & (st == FREE) & (d[None, :] < (np.where(use, r, 0) - NEW_TOL_M)[:, None])
    flat_clear = (pr * saved.w + pc)[clear]

    # A beam that went past its expected hit passed through every occupied
    # cell from there to (GONE_SHORT_M short of) its own return.
    through = (gone[:, None] & pin & (st == OCC) & (d[None, :] >= e[:, None])
               & (d[None, :] < (np.where(use, r, 0) - GONE_SHORT_M)[:, None]))
    flat_through = (pr * saved.w + pc)[through]

    looked = np.unique(np.concatenate([flat_clear, flat_e[seen_e], flat_ret[use & hin],
                                       flat_through]))
    contra = np.unique(np.concatenate([flat_e[gone], flat_through, flat_ret[new]]))
    out.update(looked=looked, contra=contra, new=int(new.sum()), gone=int(gone.sum()))
    return out


class Contradictions:
    """Trigger 2: per-cell looks and contradictions over the last WINDOW_S."""

    def __init__(self, saved: SavedMap):
        self.saved = saved
        size = saved.w * saved.h
        self.looks = np.zeros(size, dtype=np.int32)
        self.contras = np.zeros(size, dtype=np.int32)
        self.window = deque()               # (t, looked, contra)
        self.scans = 0
        self.lost = 0

    def add(self, scan: dict, laser_pose: Sequence[float], now: float) -> Optional[dict]:
        """Fold one scan in; the trigger's record if it fires, else None."""
        while self.window and self.window[0][0] < now - WINDOW_S:
            _, lk, ct = self.window.popleft()
            self.looks[lk] -= 1
            self.contras[ct] -= 1
        j = judge_scan(self.saved, scan, laser_pose)
        if j["agreement"] is None:
            return None
        self.scans += 1
        if j["agreement"] < LOST_BELOW:
            self.lost += 1
            return None
        # A contradicted cell was looked at, whichever list said so.
        looked = np.union1d(j["looked"], j["contra"])
        self.looks[looked] += 1
        self.contras[j["contra"]] += 1
        self.window.append((now, looked, j["contra"]))
        group = self.largest_group()
        if len(group) < MIN_GROUP:
            return None
        xs, ys = self._centres(group)
        return {"trigger": "contradiction", "cells": len(group),
                "centre_m": [round(float(np.mean(xs)), 3), round(float(np.mean(ys)), 3)],
                "agreement": round(j["agreement"], 3)}

    def persistent(self, now: Optional[float] = None) -> np.ndarray:
        lk = self.looks
        cand = np.nonzero((lk >= MIN_LOOKS) & (self.contras >= MIN_SHARE * lk))[0]
        if cand.size == 0 or not self.window:
            return cand
        now = self.window[-1][0] if now is None else now
        # The oldest contradiction of each candidate still in the window.
        first = np.full(cand.size, np.inf)
        for t, _, ct in self.window:            # oldest first
            hit = np.isin(cand, ct, assume_unique=True) & ~np.isfinite(first)
            first[hit] = t
            if np.isfinite(first).all():
                break
        return cand[now - first >= HOLD_S]

    def largest_group(self) -> list:
        """The biggest set of persistent cells chained within GROUP_GAP_M."""
        cells = {(int(i % self.saved.w), int(i // self.saved.w)) for i in self.persistent()}
        gap = int(math.floor(GROUP_GAP_M / self.saved.res + 1e-9))
        best, seen = [], set()
        for start in cells:
            if start in seen:
                continue
            seen.add(start)
            group, todo = [], [start]
            while todo:
                c, r = todo.pop()
                group.append((c, r))
                for dc in range(-gap, gap + 1):
                    for dr in range(-gap, gap + 1):
                        nb = (c + dc, r + dr)
                        if nb in cells and nb not in seen:
                            seen.add(nb)
                            todo.append(nb)
            if len(group) > len(best):
                best = group
        return best

    def _centres(self, group):
        res, s = self.saved.res, self.saved
        gx = np.array([(c + 0.5) * res for c, _ in group])
        gy = np.array([(r + 0.5) * res for _, r in group])
        cy, sy = math.cos(s.oyaw), math.sin(s.oyaw)
        return s.ox + cy * gx - sy * gy, s.oy + sy * gx + cy * gy
