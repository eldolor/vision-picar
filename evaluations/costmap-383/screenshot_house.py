"""3.83: a phone screenshot of the costmap layer over a house-sized map.
The map is scaled_house's walls at 5 cm, partly unexplored; the costmap is
inflated from it with nav2's formula and this repo's parameters (0.35 m,
scaling 5.0). Illustration only -- not a nav2 measurement."""
# Run: pytest evaluations/costmap-383/screenshot_house.py (from the repo root).
import json, math
import numpy as np
from sim.maps import build_world
from tests.test_ui import browser, twin_server, open_twin, _stub_world  # noqa: F401

RES, K = 0.05, 6          # 0.30 m sim cells -> 6 x 6 cells of 5 cm


def _grids():
    w = build_world("scaled_house")
    H, W = w.height * K, w.width * K
    occ = np.zeros((H, W), bool)
    for y, row in enumerate(w.layout):
        for x, ch in enumerate(row):
            if ch == "#":
                occ[y*K:(y+1)*K, x*K:(x+1)*K] = True
    for (x, y) in w.solid_cells:
        occ[y*K+1:(y+1)*K-1, x*K+1:(x+1)*K-1] = True
    known = np.zeros((H, W), bool)
    known[:, : int(W * 0.72)] = True                  # the east end not yet seen
    cells = np.where(known, np.where(occ, 1, 0), -1)
    ys, xs = np.nonzero(occ & known)
    yy, xx = np.mgrid[0:H, 0:W]
    d = np.full((H, W), 9.0)
    for y, x in zip(ys, xs):
        d = np.minimum(d, np.hypot(yy - y, xx - x) * RES)
    cost = np.zeros((H, W), int)
    insc = 0.1155
    infl = (d > insc) & (d <= 0.35)
    cost[infl] = np.round(252 * np.exp(-5.0 * (d[infl] - insc)) * 100 / 255).astype(int).clip(1, 98)
    cost[(d > 0) & (d <= insc)] = 99
    cost[d == 0] = 100
    cost = np.where(known, cost, -1)
    grid = {"usable": True, "map_id": "slam-demo", "map_version": 42, "resolution_m": RES,
            "width": W, "height": H, "origin_x_m": 0.0, "origin_y_m": 0.0,
            "cells": cells.ravel().tolist()}
    cm = dict(grid, costs=cost.ravel().tolist(), costmap_version=17)
    del cm["cells"], cm["map_version"]
    return grid, cm


def test_demo(browser, twin_server):
    grid, cm = _grids()
    pose = {"usable": True, "map_id": "slam-demo", "x_m": 1.6, "y_m": 1.4, "heading_deg": 90.0}
    err = {"usable": False}
    page, errors = open_twin(browser, twin_server)
    _stub_world(page, grid, pose, err)
    plan = [[1.6 + i * 0.1, 1.4 + (0.0 if i < 20 else (i - 20) * 0.1)] for i in range(32)]
    goal = {"goal": {"state": "active", "x_m": plan[-1][0], "y_m": plan[-1][1]}, "plan": plan}
    page.route("**/world/goal", lambda r: r.fulfill(status=200, content_type="application/json",
                                                    body=json.dumps(goal)))
    page.route("**/world/costmap", lambda r: r.fulfill(status=200, content_type="application/json",
                                                       body=json.dumps(cm)))
    page.click("#btn-settings"); page.fill("#cfg-server-url", twin_server); page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#map-costmap", state="visible", timeout=8000)
    page.click("#map-costmap")
    page.wait_for_function("() => document.getElementById('map-readout').textContent.includes('costmap v17')", timeout=5000)
    page.locator(".map-wrap").scroll_into_view_if_needed()
    page.wait_for_timeout(300)
    page.screenshot(path="evaluations/costmap-383/costmap-house.png")
    page.locator("#map-costmap").click()
    page.wait_for_timeout(600)
    page.screenshot(path="evaluations/costmap-383/costmap-house-off.png")
    assert not errors, errors
