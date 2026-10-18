"""
tests/test_review_376n2.py

`PLAN-ros-alignment.md` 3.76, step N2: the bugs the review of the consoles
(`control/admin.html`, `admin.js`, `metrics.html`, `metrics.js`, whole files)
confirmed. Each test was written before its fix and seen to fail on `dev`
for the reason it names. The page tests drive the real console in a real
browser, its routes stubbed, as `tests/test_ui_admin.py` does.
"""

import io
import json
import struct
import zlib
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT
from tests.test_ui_admin import (  # noqa: F401  (fixtures)
    DESKTOP, MODELS, PHONE, WALKS, admin_server, browser, sync_api)

WALK = "red-backpack-opus-4-5-20260830-104705"


def _json(route, body, status=200):
    route.fulfill(status=status, content_type="application/json", body=json.dumps(body))


def _png(width, height=1):
    """A PNG `width` wide, to tell two frames apart by size."""
    raw = b"".join(b"\x00" + b"\x80\x80\x80" * width for _ in range(height))
    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def _console(browser, admin_server, walks=None, *, evaluation=None, viewport=DESKTOP,
             routes=(), init=None):
    """The console with the list routes stubbed. `routes` are (pattern,
    handler) pairs added AFTER the defaults, so they win."""
    walks = walks or WALKS
    context = browser.new_context(viewport=viewport, accept_downloads=True)
    context.add_init_script(
        '(() => { try { localStorage.setItem("vp_admin_secret", "test"); } catch (e) {} })();')
    if init:
        context.add_init_script(init)
    page = context.new_page()
    page.route("**/recording/models", lambda r: _json(r, MODELS))
    page.route("**/recording/walks", lambda r: _json(r, walks))
    page.route("**/recording/summary", lambda r: _json(r, {"rows": []}))
    page.route("**/stats", lambda r: _json(r, {"walks": 1, "frames": 1, "bytes": 1}))
    page.route("**/evaluation", evaluation or (lambda r: _json(r, {"score": 0})))
    for pattern, handler in routes:
        page.route(pattern, handler)
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    # Not "networkidle": a test that holds a request open never reaches it.
    page.goto(admin_server + "/admin", wait_until="load")
    page.wait_for_selector(".walk", timeout=10_000)
    return page, errors


def _one_walk(**extra):
    w = dict(WALKS["walks"][0])
    w.update(extra)
    return {"recording_dir": "/tmp/r", "walks": [w]}


def _detail(n, navigate=None):
    frames = [{"file": f"frame-{i:04d}.jpg", "bytes": 2048} for i in range(n)]
    entries = [{"file": f["file"], "navigate": navigate or {"action": "FORWARD"}} for f in frames]
    return {"frames": frames, "entries": entries}


# ---------- N2.1: frame token counts went into innerHTML ----------


def test_a_frames_stored_token_counts_are_text_not_markup(browser, admin_server):
    """`navigate.usage` is stored as posted -- under the brain's secret, via
    the twin -- and its two counts were joined into innerHTML unescaped: a
    frame could run script in the page holding the walks secret (Thermos read
    it back)."""
    nav = {"action": "FORWARD", "model_id": "m",
           "usage": {"input_tokens": "<img src=x onerror=\"window.__pwned=1\">", "output_tokens": 3}}
    page, _ = _console(browser, admin_server, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(1, nav))),
        ("**/frames/*", lambda r: r.fulfill(status=200, content_type="image/png", body=_png(2))),
    ])
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frames .frame-model")).to_contain_text("<img src=x")
    page.wait_for_timeout(200)
    assert page.evaluate("window.__pwned") is None
    page.context.close()


# ---------- N2.2: bulk delete stayed armed ----------


def test_bulk_delete_asks_again_after_it_has_run(browser, admin_server):
    """`armConfirm` never disarmed after its action: once one bulk delete had
    run, the next selection went with a single click."""
    deleted = []

    def delete(route):
        if route.request.method == "DELETE":
            deleted.append(route.request.url.rsplit("/", 1)[-1])
            route.fulfill(status=204, body="")
        else:
            route.fallback()

    page, _ = _console(browser, admin_server, routes=[("**/recording/walks/*", delete)])
    boxes = page.locator(".walk input[type=checkbox]")
    boxes.nth(0).check()
    page.click("#btn-bulk-delete")
    page.click("#btn-bulk-delete")
    sync_api.expect(page.locator(".walk")).to_have_count(1)
    page.locator(".walk input[type=checkbox]").nth(0).check()
    page.click("#btn-bulk-delete")
    page.wait_for_timeout(200)
    assert len(deleted) == 1, deleted
    sync_api.expect(page.locator("#btn-bulk-delete")).to_contain_text("Really delete")
    page.context.close()


# ---------- N2.3: a rebuilt row forgot what was out ----------


def test_a_replay_out_survives_a_search_and_lands_in_the_new_row(browser, admin_server):
    """Every search keystroke, sort and Refresh rebuilt the rows with every
    control enabled: a second paid replay one pick away while the first was
    out (F21's verdict assumed the console prevented it), and the first's
    result painted a detached row."""
    held = []
    replays = {"replays": []}
    page, _ = _console(browser, admin_server, routes=[
        ("**/replays", lambda r: _json(r, replays)),
        ("**/replay", lambda r: held.append(r)),
    ])
    sel = page.locator(".replay-select").first
    sel.select_option(index=1)
    page.wait_for_timeout(300)
    assert len(held) == 1
    page.fill("#search", "red")                          # rebuilds the rows
    sync_api.expect(page.locator(".replay-select").first).to_be_disabled()
    held[0].fulfill(status=200, content_type="application/json", body=json.dumps(
        {"model_id": MODELS["models"][0]["id"], "prompt_variant": "default", "score": 61,
         "verdict": "mixed", "agreement": 0.5, "errors": 0, "flags": []}))
    sync_api.expect(page.locator(".walk").first).to_contain_text("score 61")
    sync_api.expect(page.locator(".replay-select").first).to_be_enabled()
    page.context.close()


def test_a_queued_score_cannot_be_clicked_into_a_second_run(browser, admin_server):
    """"Scoring…" was a label on an enabled button: a click sent POST
    /evaluate while the queue's GET was still computing (two judge runs), and
    a queued walk that failed read "Scoring…" until a reload."""
    held, posted = [], []
    walks = _one_walk(eval=None, finished=True)

    def evaluate(route):
        posted.append(route.request.method)
        _json(route, {"score": 1})

    page, _ = _console(browser, admin_server, walks, evaluation=lambda r: held.append(r),
                       routes=[("**/evaluate", evaluate)])
    btn = page.locator(".walk-actions button", has_text="Scoring")
    sync_api.expect(btn).to_be_disabled()
    btn.click(force=True)
    page.wait_for_timeout(200)
    assert posted == []
    held[0].fulfill(status=500, content_type="application/json", body='{"detail": "judge down"}')
    sync_api.expect(page.locator(".walk-actions button", has_text="Evaluate")).to_be_enabled()
    page.context.close()


def test_a_refresh_during_auto_scoring_keeps_the_scores(browser, admin_server):
    """The queue wrote each score onto the walk objects of the list it
    started from; a Refresh had replaced them, so the scores landed nowhere
    on screen (Thermos: none of three shown)."""
    held = []
    walks = _one_walk(eval=None, finished=True)
    page, _ = _console(browser, admin_server, walks, evaluation=lambda r: held.append(r))
    page.wait_for_timeout(200)
    page.click("#btn-refresh")
    page.wait_for_timeout(300)
    for r in held:
        r.fulfill(status=200, content_type="application/json",
                  body=json.dumps({"score": 33, "verdict": "bad", "flags": ["degenerate"]}))
    sync_api.expect(page.locator(".score-badge")).to_contain_text("33")
    page.context.close()


# ---------- N2.4: Download on the bucket ----------


def test_download_works_when_the_store_hands_out_a_link(browser, admin_server):
    """On S3 the route answered 307 to a presigned URL; `fetch()` with the
    secret header followed it as a CORS request the bucket refuses, and the
    click did nothing at all (Thermos reproduced with a CORS-less origin)."""
    def download(route):
        if "link=1" in route.request.url:
            _json(route, {"url": "http://bucket.test/w.zip"})
        else:
            route.fulfill(status=307, headers={"location": "http://bucket.test/w.zip"}, body="")

    page, _ = _console(browser, admin_server, routes=[
        ("**/download*", download),
        ("http://bucket.test/**", lambda r: r.fulfill(
            status=200, body=b"PK\x05\x06" + b"\x00" * 18,
            headers={"content-type": "application/zip",
                     "content-disposition": 'attachment; filename="w.zip"'})),
    ])
    with page.expect_download(timeout=5000) as dl:
        page.locator(".dl-link").first.click()
    assert dl.value.suggested_filename.endswith(".zip")
    page.context.close()


def test_the_route_hands_out_its_link_when_asked(tmp_path):
    from fastapi.testclient import TestClient
    import control.admin_server as admin_server_mod
    from control.walk_store import LocalWalkStore
    from tests.test_admin_server import make_walk

    class Linked(LocalWalkStore):
        def download_url(self, filename, data, content_type, expires_s=900):
            return "https://bucket.example/" + filename

    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  recording_dir: {tmp_path / 'recordings'}\n")
    make_walk(tmp_path, "walk-l", ["FORWARD"])
    c = TestClient(admin_server_mod.create_app(config_path=config,
                                               store=Linked(tmp_path / "recordings")))
    assert c.get("/recording/walks/walk-l/download?link=1").json() == {
        "url": "https://bucket.example/walk-l.zip"}
    r = c.get("/recording/walks/walk-l/download", follow_redirects=False)
    assert r.status_code == 307                          # a plain link still redirects


# ---------- N2.5-N2.11: the lows taken ----------


def test_a_card_opens_its_own_frame_after_a_frame_delete(browser, admin_server):
    """Each card kept the index it was drawn with: after a delete every later
    card opened the next frame, and the last one threw."""
    def frame(route):
        if route.request.method == "DELETE":
            _json(route, {"marked": True})
        else:
            route.fulfill(status=200, content_type="image/png", body=_png(2))

    page, errors = _console(browser, admin_server, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(3))),
        ("**/frames/*", frame),
    ])
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frame")).to_have_count(3)
    delete = page.locator(".frame").nth(1).locator("button.danger")
    delete.click()
    delete.click()
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    page.locator(".frame").nth(1).locator("img").click()
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("2 / 2")
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("frame-0002")
    assert errors == []
    page.context.close()


def test_the_slideshow_shows_the_picture_of_the_frame_it_names(browser, admin_server):
    """Each step started a fetch and the last to arrive won: a slow frame 0
    landed under frame 1's caption."""
    held, asked = [], {"frame0": 0}

    def frame(route):
        if route.request.url.endswith("frame-0000.jpg"):
            asked["frame0"] += 1
        if route.request.url.endswith("frame-0000.jpg") and asked["frame0"] == 2:
            held.append(route)                           # the slideshow's, not the card's
        else:
            width = 7 if route.request.url.endswith("frame-0001.jpg") else 3
            route.fulfill(status=200, content_type="image/png", body=_png(width))

    page, _ = _console(browser, admin_server, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(2))),
        ("**/frames/*", frame),
    ])
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    page.wait_for_timeout(200)
    page.locator(".frame").nth(0).locator("img").click()  # frame 0: its picture held
    page.wait_for_timeout(200)
    page.click("#slideshow-next")
    page.wait_for_function("document.getElementById('slideshow-img').naturalWidth === 7")
    for r in held:
        r.fulfill(status=200, content_type="image/png", body=_png(3))
    page.wait_for_timeout(400)
    assert page.evaluate("document.getElementById('slideshow-img').naturalWidth") == 7
    page.context.close()


def test_a_refresh_shows_a_walks_new_frames(browser, admin_server):
    """`detailCache` outlived a Refresh: a walk still recording said 6 frames
    and View showed the 4 of its first open."""
    n = {"frames": 2}
    page, _ = _console(browser, admin_server, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(n["frames"]))),
        ("**/frames/*", lambda r: r.fulfill(status=200, content_type="image/png", body=_png(2))),
    ])
    view = page.locator(".walk").first.locator("button", has_text="View")
    view.click()
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    n["frames"] = 3
    page.click("#btn-refresh")
    page.wait_for_timeout(300)
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frame")).to_have_count(3)
    page.context.close()


def test_a_failed_tag_puts_the_stored_label_back(browser, admin_server):
    def tag(route):
        _json(route, {"detail": "store down"}, status=500)

    page, _ = _console(browser, admin_server, routes=[("**/tag", tag)])
    sel = page.locator(".label-select").first
    sel.select_option("good")
    sync_api.expect(page.locator(".inline-err")).to_contain_text("Tag failed")
    assert sel.input_value() == ""
    page.context.close()


def test_a_failed_connect_keeps_the_saved_secret(browser, admin_server):
    """Connect saved the typed secret before trying it; a mistyped one
    replaced the working one for good (metrics.js already waits)."""
    page, _ = _console(browser, admin_server)

    def walks(route):
        if route.request.headers.get("x-app-secret") == "test":
            _json(route, WALKS)
        else:
            _json(route, {"detail": "Missing or invalid x-app-secret header."}, status=401)

    page.route("**/recording/walks", walks)
    page.fill("#secret", "typo")
    page.click("#btn-connect")
    sync_api.expect(page.locator("#conn-status")).to_contain_text("Failed")
    assert page.evaluate("localStorage.getItem('vp_admin_secret')") == "test"
    page.click("#btn-refresh")                           # still served under the good one
    sync_api.expect(page.locator(".walk")).to_have_count(len(WALKS["walks"]))
    page.context.close()


def test_a_walk_with_no_time_sorts_last_both_ways(browser, admin_server):
    walks = {"recording_dir": "/tmp/r", "walks": [
        dict(WALKS["walks"][0], walk="b-walk", recorded_at=None),
        dict(WALKS["walks"][0], walk="a-walk", recorded_at=1788000000.0),
        dict(WALKS["walks"][0], walk="c-walk", recorded_at=1787000000.0),
    ]}
    page, _ = _console(browser, admin_server, walks)
    for order in ("name-desc", "name-asc"):
        page.select_option("#sort", order)
        names = [n.split()[0] for n in page.locator(".walk-name").all_inner_texts()]
        assert names[-1].startswith("b-walk"), (order, names)
    page.context.close()


def test_the_metrics_page_paints_only_its_newest_load(browser):
    """Overlapping loads had no order: 30 -> 90 -> 7 days drew the slow
    90-day answer under a "7" selector; and an error left the last window's
    numbers under it."""
    page = browser.new_context(viewport=PHONE).new_page()
    root = Path(REPO_ROOT) / "control"
    def serve(name):
        kind = "text/html" if name.endswith(".html") else "application/javascript"
        return lambda route: route.fulfill(status=200, body=(root / name).read_text(),
                                           content_type=kind)
    for name in ("metrics.html", "metrics.js"):
        page.route(f"http://metrics.test/{name}", serve(name))
    held = []

    def runs(n):
        return [{"run_id": f"r{i}", "finished_at": 1760000000 + i, "policy": "explore",
                 "outcome": "found", "steps": 3, "stats": {}, "config": {}} for i in range(n)]

    def summary(route):
        if "days=90" in route.request.url:
            held.append(route)
        else:
            days = int(route.request.url.split("days=")[1].split("&")[0])
            _json(route, {"runs": runs(1 if days == 7 else 2), "days": days})

    page.route("http://metrics.test/metrics/summary*", summary)
    page.goto("http://metrics.test/metrics.html")
    sync_api.expect(page.locator("td .pill")).to_have_count(2, timeout=5000)
    page.select_option("#days", "90")
    page.wait_for_timeout(200)
    page.select_option("#days", "7")
    sync_api.expect(page.locator("td .pill")).to_have_count(1, timeout=5000)
    held[0].fulfill(status=200, content_type="application/json",
                    body=json.dumps({"runs": runs(5), "days": 90}))
    page.wait_for_timeout(400)
    assert page.locator("td .pill").count() == 1
    # An error clears the last window's numbers.
    page.unroute("http://metrics.test/metrics/summary*")
    page.route("http://metrics.test/metrics/summary*",
               lambda r: _json(r, {"detail": "down"}, status=500))
    page.click("#reload")
    sync_api.expect(page.locator("#error")).to_be_visible()
    assert page.locator("td .pill").count() == 0
    page.context.close()


# ---------- the fixes' round 1 ----------


def test_a_secret_being_tried_is_sent_on_its_own_request_only(browser, admin_server):
    """Round 1 (both): Connect set the page's secret before trying it, so the
    queue and a Refresh went out under a typo meanwhile; and two failed
    Connects answering out of order left the first typo in use."""
    page, _ = _console(browser, admin_server)
    held, sent = [], []

    def walks(route):
        key = route.request.headers.get("x-app-secret")
        sent.append(key)
        if key in ("A", "B"):
            held.append(route)
        else:
            _json(route, WALKS)

    page.route("**/recording/walks", walks)
    page.fill("#secret", "A")
    page.click("#btn-connect")
    page.fill("#secret", "B")
    page.click("#btn-connect")
    page.wait_for_timeout(200)
    page.click("#btn-refresh")
    page.wait_for_timeout(300)
    assert sent[-1] == "test", sent
    for r in held:                                       # A's 401 first, then B's
        _json(r, {"detail": "Missing or invalid x-app-secret header."}, status=401)
    page.wait_for_timeout(200)
    page.click("#btn-refresh")
    page.wait_for_timeout(300)
    assert sent[-1] == "test", sent
    page.context.close()


def test_play_waits_for_a_slow_frame(browser, admin_server):
    """Round 1 (Thermos): each 1.2 s tick made the frame still loading stale,
    so with frames slower than the tick Play showed no picture at all."""
    held = []

    def frame(route):
        held.append(route)

    page, _ = _console(browser, admin_server, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(3))),
        ("**/frames/*", frame),
    ])
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frame")).to_have_count(3)
    page.locator(".frame").nth(0).locator("img").click()
    page.click("#slideshow-play")
    page.wait_for_timeout(2600)                          # two ticks, frame 0 still out
    # Read once, not `expect` (which retries until a cycling caption agrees).
    assert page.locator("#slideshow-pos").inner_text().startswith("1 / 3")
    for r in list(held):
        r.fulfill(status=200, content_type="image/png", body=_png(5))
    held.clear()
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("2 / 3", timeout=4000)
    page.context.close()


def test_bulk_delete_skips_a_walk_whose_own_delete_is_out(browser, admin_server):
    """Round 1 (both): bulk delete sent a second DELETE for a walk whose row
    Delete was out; its 404 counted as a failure and kept a gone walk
    selected."""
    held, deletes = [], []

    def walk(route):
        if route.request.method != "DELETE":
            return route.fallback()
        deletes.append(route.request.url)
        held.append(route)

    page, _ = _console(browser, admin_server, routes=[("**/recording/walks/*", walk)])
    row = page.locator(".walk").first
    row.locator("button", has_text="Delete walk").click()
    row.locator("button", has_text="Really delete").click()
    row.locator("input[type=checkbox]").check()
    page.click("#btn-bulk-delete")
    page.click("#btn-bulk-delete")
    page.wait_for_timeout(300)
    assert len(deletes) == 1, deletes
    # Round 2 (both): skipped, it stays selected.
    sync_api.expect(page.locator("#bulk-count")).to_have_text("1 selected")
    held[0].fulfill(status=204, body="")
    sync_api.expect(page.locator(".walk")).to_have_count(1)
    page.context.close()


def test_a_failed_manual_score_is_not_retried_by_itself(browser, admin_server):
    """Round 1 (/code-review): a walk that arrived while the queue ran read
    Evaluate; if its click failed, nothing marked it, and the queue's rescan
    paid a second judge attempt nobody asked for."""
    first = dict(WALKS["walks"][0], eval=None, finished=True)
    second = dict(WALKS["walks"][1], eval=None, finished=True)
    lists = [{"recording_dir": "/tmp/r", "walks": [first]},
             {"recording_dir": "/tmp/r", "walks": [first, second]}]
    n = {"lists": 0}
    held, gets = [], []

    def walks(route):
        body = lists[min(n["lists"], 1)]
        n["lists"] += 1
        _json(route, body)

    def evaluation(route):
        gets.append(route.request.url)
        if second["walk"] in route.request.url:
            _json(route, {"score": 9})
        else:
            held.append(route)

    page, _ = _console(browser, admin_server, lists[0], evaluation=evaluation,
                       routes=[("**/recording/walks", walks),
                               ("**/evaluate", lambda r: _json(r, {"detail": "judge down"}, status=500))])
    page.click("#btn-refresh")                           # brings the second walk mid-queue
    row = page.locator(".walk", has_text="qwen3")
    row.locator("button", has_text="Evaluate").click()
    sync_api.expect(row.locator(".inline-err")).to_contain_text("Evaluate failed")
    for r in held:
        _json(r, {"score": 40})
    page.wait_for_timeout(500)
    assert not [g for g in gets if second["walk"] in g], gets
    page.context.close()


def test_a_tag_that_crosses_a_refresh_is_shown(browser, admin_server):
    """Round 1 (/code-review): the tag landed on the row and walk it was
    clicked on; a Refresh meanwhile had replaced both."""
    held = []
    page, _ = _console(browser, admin_server, routes=[("**/tag", lambda r: held.append(r))])
    page.locator(".label-select").first.select_option("good")
    page.wait_for_timeout(200)
    page.click("#btn-refresh")
    page.wait_for_timeout(300)
    held[0].fulfill(status=200, content_type="application/json", body='{"label": "good"}')
    sync_api.expect(page.locator(".walk").first.locator(".label-badge")).to_contain_text("good")
    # Round 2 (both): the rebuilt row's dropdown too.
    assert page.locator(".label-select").first.input_value() == "good"
    page.context.close()


# ---------- the fixes' round 2 ----------


def _slides(browser, admin_server, n=3, init=None):
    held = []
    page, errors = _console(browser, admin_server, init=init, routes=[
        (f"**/recording/walks/{WALK}", lambda r: _json(r, _detail(n))),
        ("**/frames/*", lambda r: held.append(r)),
    ])
    page.locator(".walk").first.locator("button", has_text="View").click()
    sync_api.expect(page.locator(".frame")).to_have_count(n)
    page.locator(".frame").nth(0).locator("img").click()
    return page, held


def _release(held, width=400):
    for r in list(held):
        r.fulfill(status=200, content_type="image/png", body=_png(width, 300))
    held.clear()


def test_a_second_next_does_not_close_the_slideshow(browser, admin_server):
    """Round 2 (Thermos, medium): each step emptied the picture; on a desktop
    the box collapsed, Next moved up 240 px, and a second click at the same
    spot hit the backdrop and closed the slideshow."""
    page, held = _slides(browser, admin_server, n=5)
    _release(held)
    page.wait_for_function("document.getElementById('slideshow-img').naturalWidth === 400")
    box = page.locator("#slideshow-next").bounding_box()
    x, y = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
    page.mouse.click(x, y)                               # frame 2, held
    page.wait_for_timeout(150)
    page.mouse.click(x, y)
    page.wait_for_timeout(150)
    assert page.evaluate("document.getElementById('slideshow').classList.contains('show')")
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("3 / 5")
    page.context.close()


def test_play_gives_a_frame_its_time_from_when_it_lands(browser, admin_server):
    """Round 2 (both): Play kept a fixed 1.2 s grid, so a frame landing just
    before a tick was shown for a moment."""
    page, held = _slides(browser, admin_server)
    page.click("#slideshow-play")
    page.wait_for_timeout(1100)
    _release(held)                                       # lands 0.1 s before the old tick
    page.wait_for_timeout(600)
    assert page.locator("#slideshow-pos").inner_text().startswith("1 / 3")
    page.context.close()


def test_a_frame_that_never_answers_does_not_stop_play(browser, admin_server):
    """Round 2 (both): `frameBlobUrl` had no timeout; Play waited on it for
    good, reading Pause."""
    page, held = _slides(browser, admin_server,
                         init="window.ADMIN_FRAME_TIMEOUT_MS = 400;")
    page.click("#slideshow-play")
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("2 / 3", timeout=4000)
    page.context.close()


def test_a_404_that_is_not_the_servers_is_not_a_delete(browser, admin_server):
    """Round 2 (both): bulk delete read any 404 as gone -- a gateway's page,
    with no detail of ours, hid a walk that was still stored."""
    def walk(route):
        if route.request.method != "DELETE":
            return route.fallback()
        route.fulfill(status=404, content_type="text/html", body="<h1>Not Found</h1>")

    page, _ = _console(browser, admin_server, routes=[("**/recording/walks/*", walk)])
    page.locator(".walk input[type=checkbox]").first.check()
    page.click("#btn-bulk-delete")
    page.click("#btn-bulk-delete")
    sync_api.expect(page.locator(".inline-err")).to_contain_text("failed to delete")
    sync_api.expect(page.locator(".walk")).to_have_count(len(WALKS["walks"]))
    page.context.close()


# ---------- the fixes' round 3 ----------


def test_play_moves_on_from_a_frame_that_failed_while_paused(browser, admin_server):
    """Round 3 (/code-review, medium): Play read "still loading" off the
    hidden picture, so after a frame had failed with Play off, pressing Play
    scheduled nothing and the slideshow never moved."""
    page, held = _slides(browser, admin_server)
    for r in list(held):
        r.fulfill(status=500, body="")
    held.clear()
    page.wait_for_timeout(300)                           # frame 0 has failed
    page.click("#slideshow-play")
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("2 / 3", timeout=3000)
    page.context.close()


STALLED_BODY = """
(() => {
  const real = window.fetch;
  window.fetch = function (url, opts) {
    if (!String(url).includes("/frames/frame-0000")) return real.apply(this, arguments);
    const signal = opts && opts.signal;
    // Headers now, a body that ends only if the request is aborted.
    const body = new ReadableStream({ start(c) {
      if (signal) signal.addEventListener("abort", () => c.error(new DOMException("aborted", "AbortError")));
    } });
    return Promise.resolve(new Response(body, { status: 200, headers: { "content-type": "image/png" } }));
  };
  window.ADMIN_FRAME_TIMEOUT_MS = 400;
})();
"""


def test_a_frame_whose_body_stalls_does_not_stop_play(browser, admin_server):
    """Round 3 (/code-review, medium): the deadline was cleared once the
    headers came, so a body that stalled still held Play for good."""
    page, held = _slides(browser, admin_server, init=STALLED_BODY)
    _release(held)
    page.click("#slideshow-play")
    sync_api.expect(page.locator("#slideshow-pos")).to_contain_text("2 / 3", timeout=4000)
    page.context.close()
