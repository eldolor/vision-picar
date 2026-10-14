"""
tests/test_ui_review_376f.py

3.76 batch F's console fixes in a real browser at 390 px (CLAUDE.md
section 1). Each was red before its fix; the ids are the findings table's in
`docs/plans/ros-alignment/3.76-legacy-review.md`. Screenshots go to
$PICAR_SCREENSHOTS when it is set.
"""

import copy
import json
import os
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT
from tests.test_ui_admin import PHONE, WALKS, admin_server, browser, open_console  # noqa: F401

sync_api = pytest.importorskip("playwright.sync_api")


def _shot(page, name):
    out = os.environ.get("PICAR_SCREENSHOTS")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        page.screenshot(path=str(Path(out) / f"376f-{name}.png"), full_page=True)


def _metrics_page(browser, runs):
    page = browser.new_context(viewport=PHONE).new_page()
    root = Path(REPO_ROOT) / "control"

    def serve(name):
        kind = "text/html" if name.endswith(".html") else "application/javascript"
        return lambda route: route.fulfill(status=200, body=(root / name).read_text(),
                                           content_type=kind)
    for name in ("metrics.html", "metrics.js"):
        page.route(f"http://metrics.test/{name}", serve(name))
    page.route("http://metrics.test/metrics/summary*", lambda route: route.fulfill(
        status=200, content_type="application/json", body=json.dumps({"runs": runs, "days": 14})))
    return page


def test_f14_a_stored_target_is_text_not_markup(browser):
    """Red on dev: a target typed in the twin went into innerHTML, so it ran
    script in the page that holds the walks secret."""
    attack = '<img src=x onerror="window.__pwned=1">'
    runs = [{"run_id": "r1", "finished_at": 1760000000, "policy": "tiered",
             "outcome": "found", "steps": 3, "target_object": attack,
             "git_revision": attack, "config": {"tier_x": attack},
             # The fixes' review: a string count reaches the KPI cards too.
             "stats": {"cloud_ms": {"n": 1, "p50": attack, "p90": 1, "p99": 1},
                       "cloud_calls": attack, "frames": 2}}]
    page = _metrics_page(browser, runs)
    page.goto("http://metrics.test/metrics.html")
    sync_api.expect(page.locator("td .pill")).to_have_count(1, timeout=5000)
    assert page.locator("#runs img, #kpis img").count() == 0
    assert page.evaluate("() => window.__pwned") is None
    assert attack in page.locator("#runs").inner_text()
    _shot(page, "metrics-escaped")
    page.context.close()


def test_f2_a_metrics_only_row_says_so(browser, admin_server):  # noqa: F811
    """Red on dev: the summary mixed both formulas into one row; now they are
    two rows, and the metrics-only one is labelled."""
    model = "us.anthropic.claude-opus-4-5-20251101-v1:0"
    base = {"model_id": model, "prompt_variant": "default", "source": "recorded",
            "walks": 2, "median_score": 70, "best": 80, "worst": 60, "reach_rate": 0.5,
            "median_frames": 8, "collisions": 0, "flags": {}}
    rows = {"rows": [dict(base, basis="judge+metrics", mean_score=70),
                     dict(base, basis="metrics-only", mean_score=88)]}
    page, errors = open_console(browser, admin_server, viewport=PHONE, summary=rows)
    sync_api.expect(page.locator(".summary-row")).to_have_count(2)
    labels = page.locator(".summary-row .status").all_inner_texts()
    assert sorted(labels) == ["recorded", "recorded · no judge"], labels
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 1
    _shot(page, "summary-basis")
    assert errors == []
    page.close()


def test_f4_a_score_from_before_a_frame_delete_says_so(browser, admin_server):  # noqa: F811
    """Red on dev: the card was shown as a current score."""
    walks = copy.deepcopy(WALKS)
    walks["walks"][0]["eval"].update(stale=True, stale_reason="frames deleted")
    page, errors = open_console(browser, admin_server, viewport=PHONE, walks=walks)
    badge = page.locator(".score-badge").first
    sync_api.expect(badge).to_have_text("frames deleted")
    _shot(page, "frames-deleted")
    assert errors == []
    page.close()


def test_f4_deleting_a_frame_marks_the_score_at_once(browser, admin_server):  # noqa: F811
    """The fixes' review: the server marked the scorecard on delete, but the
    console kept the green score until a reload."""
    walks = copy.deepcopy(WALKS)
    page, errors = open_console(browser, admin_server, viewport=PHONE, walks=walks)
    name = walks["walks"][0]["walk"]
    page.route(f"**/recording/walks/{name}", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({
            "walk": name, "frames": [{"file": "frame-0000.jpg", "bytes": 1000}],
            "entries": [{"seq": 0, "file": "frame-0000.jpg",
                         "navigate": {"action": "FORWARD", "reasoning": "r"}}],
            "label": None, "model_id": None, "meta": None, "eval": None})))
    page.route("**/frames/*", lambda r: r.fulfill(
        status=200, content_type="application/json", body="{}")
        if r.request.method == "DELETE" else r.fulfill(
        status=200, content_type="image/jpeg", body=b"\xff\xd8\xff\xd9"))

    walk = page.locator(".walk").first
    sync_api.expect(walk.locator(".score-badge")).to_contain_text("72")
    walk.locator(".walk-actions button", has_text="View").click()
    delete = walk.locator("button", has_text="Delete frame")
    delete.click()
    walk.locator("button", has_text="Really delete").click()
    sync_api.expect(walk.locator(".score-badge")).to_have_text("frames deleted")
    _shot(page, "frame-deleted-live")
    assert errors == []
    page.close()


def _delete_first_frame(browser, admin_server, walks, delete_reply):  # noqa: F811
    page, errors = open_console(browser, admin_server, viewport=PHONE, walks=walks)
    name = walks["walks"][0]["walk"]
    page.route(f"**/recording/walks/{name}", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({
            "walk": name, "frames": [{"file": "frame-0000.jpg", "bytes": 1000}],
            "entries": [{"seq": 0, "file": "frame-0000.jpg",
                         "navigate": {"action": "FORWARD", "reasoning": "r"}}],
            "label": None, "model_id": None, "meta": None, "eval": None})))
    page.route("**/frames/*", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps(delete_reply))
        if r.request.method == "DELETE" else r.fulfill(
        status=200, content_type="image/jpeg", body=b"\xff\xd8\xff\xd9"))
    walk = page.locator(".walk").first
    walk.locator(".walk-actions button", has_text="View").click()
    walk.locator("button", has_text="Delete frame").click()
    walk.locator("button", has_text="Really delete").click()
    sync_api.expect(walk.locator(".frame")).to_have_count(0)
    return page, errors, walk


def test_f4_a_delete_keeps_an_older_scorers_reason(browser, admin_server):  # noqa: F811
    """Second review (/code-review): the console rewrote "old scorer" as
    "frames deleted"; the server says "older scorer" first."""
    walks = copy.deepcopy(WALKS)
    walks["walks"][0]["eval"].update(stale=True, stale_reason="older scorer")
    page, errors, walk = _delete_first_frame(browser, admin_server, walks,
                                             {"deleted": "frame-0000.jpg", "marked": True})
    sync_api.expect(walk.locator(".score-badge")).to_have_text("old scorer")
    assert errors == []
    page.close()


def test_f4_an_unlisted_image_leaves_the_score_current(browser, admin_server):  # noqa: F811
    """Second review (/code-review): the server marks nothing for an image
    the manifest never listed; the console marked the score anyway."""
    walks = copy.deepcopy(WALKS)
    page, errors, walk = _delete_first_frame(browser, admin_server, walks,
                                             {"deleted": "frame-0000.jpg", "marked": False})
    sync_api.expect(walk.locator(".score-badge")).to_contain_text("72")
    _shot(page, "unlisted-delete")
    assert errors == []
    page.close()
