"""Offline: ask policies x labelling rules on recorded traces (final map)."""
import heapq, pickle, sys, statistics as st
import numpy as np
from brain.room_labels import clearance
from world.interface import CELL_FREE, CELL_OCCUPIED
from sim.maps import build_world
from tests.survey_sweep import score_labels

def grids(m):
    c = np.asarray(m["cells"]).reshape(m["height"], m["width"])
    free = c == CELL_FREE
    return free, clearance(free, c == CELL_OCCUPIED) * m["resolution_m"]

def cellof(m, x, y):
    r = m["resolution_m"]; return int((y - m["origin_y_m"]) // r), int((x - m["origin_x_m"]) // r)

def wdist(free, cost, seeds):
    """Dijkstra from seeds with per-cell entry cost; returns dist, owner."""
    h, w = free.shape
    d = np.full((h, w), np.inf); own = np.full((h, w), -1)
    pq = []
    for k, (j, i) in enumerate(seeds):
        if 0 <= j < h and 0 <= i < w and d[j, i] > 0:
            d[j, i] = 0; own[j, i] = k; pq.append((0.0, j, i, k))
    heapq.heapify(pq)
    while pq:
        dd, j, i, k = heapq.heappop(pq)
        if dd > d[j, i]: continue
        for nj, ni in ((j+1,i),(j-1,i),(j,i+1),(j,i-1)):
            if 0 <= nj < h and 0 <= ni < w and free[nj, ni]:
                nd = dd + cost[nj, ni]
                if nd < d[nj, ni]:
                    d[nj, ni] = nd; own[nj, ni] = k; heapq.heappush(pq, (nd, nj, ni, k))
    return d, own

def costmap(clr, res, c0, p):
    if c0 <= 0: return np.full(clr.shape, res)
    return res * np.maximum(1.0, (c0 / np.maximum(clr, 1e-3)) ** p)

def policy(run, spacing, c0, p, stride=1):
    m = run["map"]; free, clr = grids(m); res = m["resolution_m"]
    cost = costmap(clr, res, c0, p)
    seeds, names = [], []
    d = None
    for t, x, y, room in run["trace"][::stride]:
        if room == "unknown": continue
        j, i = cellof(m, x, y)
        if not (0 <= j < free.shape[0] and 0 <= i < free.shape[1]) or not free[j, i]: continue
        if d is None or d[j, i] > spacing:
            seeds.append((j, i)); names.append(room)
            d, _ = wdist(free, cost, seeds)
    return seeds, names, free, cost

def label(m, seeds, names, free, cost):
    _, own = wdist(free, cost, seeds)
    uniq = sorted(set(names)); idx = {n: k for k, n in enumerate(uniq)}
    lut = np.array([idx[n] for n in names] + [-1])
    cells = np.where(own >= 0, lut[own], -1)
    return {"names": uniq, "cells": cells.ravel().tolist()}

def evaluate(runs, spacing, c0, p, lc0=None, lp=None):
    rows = []
    for run in runs:
        m = run["map"]
        seeds, names, free, cost = policy(run, spacing, c0, p)
        if lc0 is not None:
            cost = costmap(grids(m)[1], m["resolution_m"], lc0, lp)
        sc = score_labels(build_world(run["house"]), label(m, seeds, names, free, cost), m["width"])
        rows.append((len(seeds), sc))
    calls = [r[0] for r in rows]
    per = [v for _, s in rows for v in s["per_room"].values()]
    return (st.median(calls), max(calls), sum(v >= .95 for v in per), len(per),
            round(st.median(per), 3), st.median(s["wrong_deep"] for _, s in rows))

if __name__ == "__main__":
    runs = pickle.load(open(sys.argv[1], "rb"))
    for spec in sys.argv[2:]:
        a = [float(v) for v in spec.split(",")]
        print(spec, evaluate(runs, *a), flush=True)
