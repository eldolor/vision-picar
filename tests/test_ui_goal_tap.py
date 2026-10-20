"""3.88 A, criterion 4: a person picks explore's next goal on the Sim tab.

While the brain's explore asks (`status.explore.ask`), its candidates are
drawn on the map, numbered, and a tap on one sends a pick to the brain
(`POST /mission/explore/pick`) -- never the person's map goal
(`/world/goal`, driver `twin-dpad`), which outranks the brain and would be a
takeover, not a correction (the user, 2026-10-20). Written before the tap
existed and seen to fail; the wiring test fails again with the tap sent to
`/world/goal` (docs/plans/ros-alignment/3.88-goal-tap-learning.md).

`UI_SHOTS=<dir>` keeps the phone-size screenshot.
"""

import json
import os
import urllib.request

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (PHONE, TIERED_BRAIN_HEALTH, browser,  # noqa: F401,E402 -- fixtures
                           open_with_brain, twin_server)


def _json(obj):
    return json.dumps(obj)


def _free_cells(twin_server, n):
    """`n` free-floor cell centres of the robot server's own map, spread out."""
    grid = json.loads(urllib.request.urlopen(twin_server + "/world/map").read())
    free = [(i % grid["width"], i // grid["width"])
            for i, v in enumerate(grid["cells"]) if v == 0]
    picks = [free[0]]
    while len(picks) < n:          # each next one the farthest from those taken
        picks.append(max(free, key=lambda c: min((c[0] - a) ** 2 + (c[1] - b) ** 2
                                                 for a, b in picks)))
    res = grid["resolution_m"]
    return grid, [(grid["origin_x_m"] + (i + 0.5) * res, grid["origin_y_m"] + (j + 0.5) * res)
                  for i, j in picks]


def ask_status(points, *, choice=4, default=0, remaining=42.0, picked=None):
    """A /mission/status body as control/mission_runner.py emits one for an
    explore mission paused on a goal choice."""
    cands = [{"id": f"{choice}.{k}", "kind": "view" if k == len(points) - 1 else "frontier",
              "x_m": x, "y_m": y, "path_m": 1.0 + k, "rank": k,
              "features": {"path": 0.1 * k}} for k, (x, y) in enumerate(points)]
    return {
        "running": True, "outcome": "running", "job": "find", "policy": "explore",
        "mission": "Find the red backpack.", "target_object": "red backpack",
        "step": 6, "max_steps": 4800, "last_action": "WAIT",
        "last_reasoning": f"waiting for a pick among {len(points)} goals", "vision_failures": 0,
        "rooms_searched": [], "log_tail": [], "tier": None, "perception": None,
        "budget": {"time_limit_s": 1200, "elapsed_s": 80.0, "paused_s": 18.0},
        "explore": {"goals_sent": 5, "goals_failed": 0, "last_event": "", "camera_seen_m2": 4.0,
                    "choice_ms": None, "asks": True, "choices": 3,
                    "ask": {"choice": choice, "candidates": cands,
                            "default": cands[default]["id"], "picked": picked,
                            "waited_s": 60 - remaining, "timeout_s": 60.0,
                            "remaining_s": remaining}},
    }


def _record(page, pattern):
    seen = []

    def handler(route):
        req = route.request
        seen.append({"url": req.url, "body": req.post_data_json if req.post_data else None,
                     "headers": dict(req.headers)})
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"accepted": True}))
    page.route(pattern, handler)
    return seen


def _tap_at(page, grid, x_m, y_m):
    page.locator("#world-map").scroll_into_view_if_needed()
    box = page.locator("#world-map").bounding_box()
    page.mouse.click(box["x"] + (x_m - grid["origin_x_m"]) / grid["resolution_m"]
                     / grid["width"] * box["width"],
                     box["y"] + (y_m - grid["origin_y_m"]) / grid["resolution_m"]
                     / grid["height"] * box["height"])


def _open_asking(browser, twin_server, points):
    page, errors = open_with_brain(browser, twin_server, health=TIERED_BRAIN_HEALTH,
                                   status=ask_status(points))
    sync_api.expect(page.locator("#map-readout")).to_contain_text("explore asks", timeout=5000)
    return page, errors


def test_a_tap_on_a_candidate_is_a_pick_never_a_map_goal(browser, twin_server):
    grid, points = _free_cells(twin_server, 3)
    page, errors = _open_asking(browser, twin_server, points)
    picks = _record(page, "**/brain-stub/mission/explore/pick")
    goals = _record(page, "**/world/goal")
    page.wait_for_timeout(300)
    goals.clear()          # the page's own goal reads (GET) are not taps
    # The second drawn number is the first non-default candidate.
    _tap_at(page, grid, *points[1])
    page.wait_for_timeout(500)
    posted = [g for g in goals if g["body"] is not None]
    assert posted == [], f"a tap during an ask was sent as a map goal: {posted}"
    assert [p["body"] for p in picks] == [{"choice": 4, "candidate": "4.1"}], picks
    sync_api.expect(page.locator(".toast").last).to_contain_text("Explore will go there next")
    assert not errors, errors
    page.context.close()


def test_a_tap_off_every_candidate_sends_nothing(browser, twin_server):
    grid, points = _free_cells(twin_server, 2)
    page, errors = _open_asking(browser, twin_server, points)
    picks = _record(page, "**/brain-stub/mission/explore/pick")
    goals = _record(page, "**/world/goal")
    page.wait_for_timeout(300)
    goals.clear()
    page.locator("#world-map").scroll_into_view_if_needed()
    box = page.locator("#world-map").bounding_box()
    # The corner of the map: far from both candidates (a wall or unknown).
    page.mouse.click(box["x"] + 2, box["y"] + 2)
    page.wait_for_timeout(500)
    assert picks == [] and [g for g in goals if g["body"] is not None] == []
    sync_api.expect(page.locator(".toast").last).to_contain_text("numbered goals")
    assert not errors, errors
    page.context.close()


def test_a_stale_pick_says_so(browser, twin_server):
    grid, points = _free_cells(twin_server, 2)
    page, errors = _open_asking(browser, twin_server, points)
    page.route("**/brain-stub/mission/explore/pick", lambda route: route.fulfill(
        status=409, content_type="application/json",
        body=_json({"detail": "choice 4 timed out; the rule's pick stands"})))
    _tap_at(page, grid, *points[0])
    sync_api.expect(page.locator(".toast").last).to_contain_text("timed out", timeout=3000)
    page.context.close()


def test_the_switch_sends_explore_ask_only_for_explore(browser, twin_server):
    page, errors = open_with_brain(browser, twin_server, health=TIERED_BRAIN_HEALTH,
                                   status={"running": False, "outcome": "idle",
                                           "log_tail": [], "tier": None, "perception": None})
    started = []

    def capture(route):
        started.append(route.request.post_data_json)
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"started": True, "status": {"running": True,
                                                              "outcome": "running"}}))
    page.route("**/brain-stub/mission/start", capture)
    sync_api.expect(page.locator("#brain-ask-row")).to_be_hidden()
    page.select_option("#brain-policy", "explore")
    sync_api.expect(page.locator("#brain-ask-row")).to_be_visible()
    page.check("#brain-ask")
    page.click("#btn-brain-mission")
    page.wait_for_timeout(500)
    assert started and started[0]["policy"] == "explore" and started[0]["explore_ask"] is True, started
    assert not errors, errors
    page.context.close()


def test_the_phone_screenshot_of_an_open_choice(browser, twin_server):
    """Evidence (CLAUDE.md section 7): the Sim tab at 390 px with explore
    asking -- numbered goals on the map, the readout's countdown."""
    grid, points = _free_cells(twin_server, 4)
    page, errors = _open_asking(browser, twin_server, points)
    assert page.viewport_size == PHONE
    sync_api.expect(page.locator("#map-readout")).to_contain_text("rule pick in")
    width = page.evaluate("document.documentElement.scrollWidth")
    assert width <= PHONE["width"], f"the page scrolls sideways ({width} px)"
    out = os.environ.get("UI_SHOTS")
    if out:
        page.locator("#world-map").scroll_into_view_if_needed()
        page.screenshot(path=os.path.join(out, "3.88-goal-tap-390.png"))
    assert not errors, errors
    page.context.close()


def test_a_new_missions_choice_1_takes_a_tap_after_the_last_ones_choice_1(browser, twin_server):
    """Review round 3: the one-pick guard was kept by choice number and never
    cleared, and a new mission numbers from 1 again -- every tap on its first
    choice read "Already picked" and sent nothing."""
    grid, points = _free_cells(twin_server, 2)
    page, errors = open_with_brain(browser, twin_server, health=TIERED_BRAIN_HEALTH,
                                   status=ask_status(points, choice=1))
    sync_api.expect(page.locator("#map-readout")).to_contain_text("explore asks", timeout=5000)
    picks = _record(page, "**/brain-stub/mission/explore/pick")
    _tap_at(page, grid, *points[0])
    page.wait_for_timeout(400)
    assert len(picks) == 1
    # The mission ends (no ask), then the next one opens its choice 1.
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json({**ask_status(points, choice=1), "explore": {"asks": True, "ask": None}})))
    page.wait_for_timeout(1500)
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(ask_status(points, choice=1))))
    sync_api.expect(page.locator("#map-readout")).to_contain_text("explore asks", timeout=5000)
    _tap_at(page, grid, *points[1])
    page.wait_for_timeout(400)
    assert len(picks) == 2, "the next mission's choice 1 refused a tap"
    assert not errors, errors
    page.context.close()
