"""
tests/test_ui_drive_stream.py

3.90 E, criteria 11-14, in a real browser at a phone's 390 px: under "Drive
via brain" the phone's frames leave on their own timer at the chosen rate,
one upload in flight (newest wins); status is polled on its own 500 ms
pace; "Combine frames" and "Record this walk" travel in /mission/start, and
the phone stops uploading each frame a second time -- the brain records the
stream itself.

The robot server is real and in `mode: teleop`, so the pushes are counted by
the body that receives them (`GET /teleop/frames`). The brain is stubbed
with `page.route`: these tests are about what the PAGE sends.

Skipped, not failed, without Playwright's browser, like tests/test_ui.py.
"""

import json
import os
import subprocess
import sys
import time

import httpx
import pytest

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

sync_api = pytest.importorskip(
    "playwright.sync_api",
    reason="needs `pip install pytest-playwright` + `playwright install chromium`",
)

PHONE = {"width": 390, "height": 844}

BRAIN_HEALTH = {
    "status": "ok", "mission_running": False, "robot_url": "x",
    "recording_allowed": True, "records_streams": True, "perception_available": True,
    "perception_detector": "yoloe-11s-seg.pt", "perception_clip_model": "RN50",
}


def _status(running=True, saved=0, vote=None):
    return {
        "running": running, "outcome": "running" if running else "exhausted",
        "policy": "tiered", "target_object": "red backpack", "step": 3,
        "last_action": "FORWARD", "last_reasoning": "stub", "vision_failures": 0,
        "ticks": 3, "seconds_since_last_tick": 0.2, "log_tail": [],
        "perception": {"status": "absent"},
        "tier": {"trigger": None, "stats": {
            "frames": 30, "frames_skipped": 10, "cloud_calls": 1,
            "vote": vote}},
        "last_frame_seq": 7,
        "recording": {"walk": "w", "saved": saved, "perceived": 3, "lost": 0,
                      "over_cap": 0, "failed": 0, "finished": not running},
        "cloud": {"state": "reachable"},
    }


@pytest.fixture(scope="module")
def teleop_server(tmp_path_factory):
    config = tmp_path_factory.mktemp("cfg") / "robot.yaml"
    config.write_text("mode: teleop\n")
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "robot.server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT, env={**os.environ, "ROBOT_CONFIG_PATH": str(config)},
    )
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f"teleop server exited early ({proc.returncode})")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail("teleop server never became healthy")
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture(scope="module")
def browser():
    with sync_api.sync_playwright() as p:
        b = p.chromium.launch(args=["--use-fake-ui-for-media-stream",
                                    "--use-fake-device-for-media-stream"])
        yield b
        b.close()


def _latest_seq(server):
    return httpx.get(f"{server}/teleop/frames", params={"after": 10**9}).json()["latest_seq"]


def _drive(browser, server, prefs):
    """Robot view, Drive via brain on, a stubbed brain; started. Returns the
    page, page errors, and what the page sent the brain."""
    ctx = browser.new_context(viewport=PHONE, permissions=["camera"])
    seed = {"guidanceMode": "robot", "vp_guide_onboarded": "1",
            "vp_brain_url": server + "/brain-stub", "vp_drive_via_brain": "1",
            "vp_drive_policy": "tiered", **prefs}
    ctx.add_init_script(
        "(() => { const s = %s;"
        " try { for (const k in s) localStorage.setItem(k, s[k]); } catch (e) {} })();"
        % json.dumps(seed))
    page = ctx.new_page()
    errors, sent = [], {"start": None, "status": 0, "recording_frames": 0, "stopped": False}
    page.on("pageerror", lambda e: errors.append(str(e)))

    def health(route):
        route.fulfill(status=200, content_type="application/json", body=json.dumps(BRAIN_HEALTH))

    def start(route):
        sent["start"] = json.loads(route.request.post_data or "{}")
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps({"started": True, "status": _status()}))

    def status(route):
        sent["status"] += 1
        vote = ({"k": 2, "n": 3, "detected": 1, "fired": 0}
                if (sent["start"] or {}).get("frame_vote") else None)
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps(_status(saved=12, vote=vote)))

    def stop(route):
        sent["stopped"] = True
        route.fulfill(status=200, content_type="application/json", body="{}")

    def recording(route):
        sent["recording_frames"] += 1
        route.fulfill(status=200, content_type="application/json", body="{}")

    page.route("**/brain-stub/health", health)
    page.route("**/brain-stub/mission/start", start)
    page.route("**/brain-stub/mission/status", status)
    page.route("**/brain-stub/mission/stop", stop)
    page.route("**/brain-stub/recording/**", recording)
    page.goto(server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.click('.tab-btn[data-tab="guide"]')
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    page.wait_for_timeout(1000)
    assert page.inner_text("#btn-guidance").strip() == "Stop", "the walk never started"
    return page, errors, sent


def test_frames_stream_at_the_chosen_rate_apart_from_status(browser, teleop_server):
    page, errors, sent = _drive(browser, teleop_server, {"vp_stream_fps": "8"})
    before, polls_before = _latest_seq(teleop_server), sent["status"]
    page.wait_for_timeout(5000)
    pushed = _latest_seq(teleop_server) - before
    polls = sent["status"] - polls_before
    # 8 fps for 5 s is 40 pushes; a local server keeps up. Status stays on
    # its own pace, not the push rate: the Guide tick's 500 ms (~10) plus the
    # Remote-brain panel's own 700 ms poll (~7).
    assert pushed >= 30, f"{pushed} frames in 5 s at 8 fps"
    assert polls <= 20, f"{polls} status polls in 5 s"
    tel = page.inner_text("#robot-telemetry")
    assert "FPS" in tel and "VOTE" not in tel, tel      # vote off unless chosen
    page.screenshot(path=os.path.join(
        os.environ.get("UI_SHOT_DIR", "/tmp"), "drive-stream-390.png"))
    page.click("#btn-guidance-close")
    assert not errors, errors
    page.close()


def test_the_vote_and_the_recording_go_to_the_brain_not_frame_by_frame(browser, teleop_server):
    page, errors, sent = _drive(browser, teleop_server, {
        "vp_stream_fps": "4", "vp_frame_vote": "2of3", "vp_record_walk": "1"})
    page.wait_for_timeout(2500)
    body = sent["start"]
    assert body["frame_vote"] == "2of3"
    assert body["record_walk"] and body["record_walk"].startswith("red-backpack")
    assert sent["recording_frames"] == 0, "the phone uploaded frames to /recording a second time"
    tel = page.inner_text("#robot-telemetry")
    assert "VOTE" in tel and "2/3" in tel, tel
    sync_api.expect(page.locator("#record-walk-sub")).to_contain_text("12 frames saved")
    page.click("#btn-guidance-close")
    page.wait_for_timeout(300)
    assert sent["stopped"]
    assert not errors, errors
    page.close()


def test_the_vision_policy_sends_no_vote(browser, teleop_server):
    page, errors, sent = _drive(browser, teleop_server, {
        "vp_drive_policy": "vision", "vp_frame_vote": "2of3"})
    assert "frame_vote" not in sent["start"], sent["start"]
    page.click("#btn-guidance-close")
    assert not errors, errors
    page.close()
