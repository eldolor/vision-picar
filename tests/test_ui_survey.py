"""3.70 S5: a survey is started from the phone.

The Sim tab's Remote brain panel picks the job -- find an object, or survey
the house -- and names a running survey as one. A D-pad tap still takes over
(M4, unchanged). Each test was written before the control existed and seen
to fail (docs/plans/ros-alignment/3.70-survey-and-mission-ends.md S5).
"""

import json

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (PHONE, TIERED_BRAIN_HEALTH, browser,  # noqa: F401,E402 -- fixtures
                           open_with_brain, twin_server)

# The brain's /health as a 3.70 S5 brain answers it: with its label-call cap.
HEALTH = dict(TIERED_BRAIN_HEALTH, room_label_max_calls=60)


def _json(obj):
    return json.dumps(obj)


def survey_status(*, running=True, outcome="running", rooms=("kitchen", "hall"),
                  calls=7, end_state=None, cancelled_by=None, end_reason=None):
    """A /mission/status body shaped as control/mission_runner.py emits one
    for `job: "survey"` (explore's frontier loop, no target)."""
    return {
        "running": running, "outcome": outcome, "job": "survey",
        "policy": "explore", "mission": "Survey the house.",
        "target_object": None, "target_room": None,
        "step": 12, "max_steps": 4800, "last_action": "GOAL",
        "last_reasoning": "frontier 2.1 m east", "vision_failures": 0,
        "rooms_searched": [], "log_tail": [], "tier": None, "perception": None,
        "end_state": end_state, "cancelled_by": cancelled_by, "end_reason": end_reason,
        "survey": {"mode": "spacing", "label_calls": calls, "label_call_limit": 60,
                   "capped": False, "no_source": False,
                   "rooms_named": list(rooms), "seeds": []},
    }


def _start_capture(page, status):
    """Captures the start body; /mission/status answers `status` once a
    start has landed (the idle one before)."""
    started = []

    def capture(route):
        started.append(route.request.post_data_json)
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"started": True, "status": status}))

    page.route("**/brain-stub/mission/start", capture)
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json(status if started else survey_status(
            running=False, outcome=None, rooms=(), calls=0))))
    return started


def test_the_survey_job_starts_a_target_less_explore_mission(browser, twin_server):
    """A survey has no target and runs explore only (MissionRunner refuses
    anything else), so the start body must say exactly that -- a target or
    another policy would be refused at start."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH, status=survey_status(
        running=False, outcome=None, rooms=(), calls=0))
    started = _start_capture(page, survey_status())
    page.select_option("#brain-job", "survey")
    # The find-only inputs are hidden: a survey has no target to type.
    sync_api.expect(page.locator("#brain-target")).to_be_hidden()
    sync_api.expect(page.locator("#brain-policy")).to_be_hidden()
    # The rows themselves, not only their inputs: a hidden select in a shown
    # row leaves an empty 8 px gap (Thermos bug pass, 2026-10-14).
    sync_api.expect(page.locator("#brain-policy-row")).to_be_hidden()
    sync_api.expect(page.locator("#brain-fault-row")).to_have_count(1)
    sync_api.expect(page.locator("#brain-fault-row")).to_be_hidden()
    sync_api.expect(page.locator("#brain-job-hint")).to_contain_text("60")
    page.click("#btn-brain-mission")
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Stop", timeout=5000)
    assert started, "no survey was started"
    body = started[0]
    assert body["job"] == "survey", body
    assert body["policy"] == "explore", body
    assert "target_object" not in body, body
    assert not errors, errors
    page.context.close()


def test_the_find_job_still_sends_its_target_and_policy(browser, twin_server):
    """The picker must not change what Find sends."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH, status=survey_status(
        running=False, outcome=None, rooms=(), calls=0))
    started = _start_capture(page, {"running": True, "outcome": "running", "job": "find",
                                    "policy": "frontier", "step": 1, "log_tail": [],
                                    "tier": None, "perception": None})
    sync_api.expect(page.locator("#brain-job")).to_have_value("find")
    page.fill("#brain-target", "red backpack")
    page.click("#btn-brain-mission")
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Stop", timeout=5000)
    body = started[0]
    assert body.get("job", "find") == "find", body
    assert body["target_object"] == "red backpack", body
    assert body["policy"] == "frontier", body
    assert not errors, errors
    page.context.close()


def test_a_running_survey_is_named_a_survey_with_its_rooms_and_calls(browser, twin_server):
    """While it runs the panel says it is a survey (not "running", which a
    find also says) and what it has named so far against the call cap."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH, status=survey_status())
    sync_api.expect(page.locator("#brain-tel-outcome")).to_contain_text(
        "Surveying", timeout=5000)
    survey = page.locator("#brain-tel-survey")
    sync_api.expect(survey).to_contain_text("kitchen, hall")
    sync_api.expect(survey).to_contain_text("7 of 60")
    # The job picker follows the running mission (another phone started it).
    sync_api.expect(page.locator("#brain-job")).to_have_value("survey")
    sync_api.expect(page.locator("#brain-job")).to_be_disabled()
    assert not errors, errors
    page.context.close()


def test_a_finished_survey_reads_as_done_not_as_a_failed_search(browser, twin_server):
    """A survey ends `exhausted` when no reachable frontier is left -- its
    success. "Nowhere reachable left to look" is a find's wording for a
    target it never saw."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH, status=survey_status(
        running=False, outcome="exhausted", end_state="exhausted",
        rooms=("kitchen", "hall", "bedroom"), calls=11))
    outcome = page.locator("#brain-tel-outcome")
    sync_api.expect(outcome).to_contain_text("Survey done", timeout=5000)
    sync_api.expect(outcome).not_to_contain_text("look")
    sync_api.expect(page.locator("#brain-tel-survey")).to_contain_text("bedroom")
    assert not errors, errors
    page.context.close()


def test_the_d_pad_still_takes_over_a_running_survey(browser, twin_server):
    """M4, unchanged: a tap during a survey goes out as the person's drive,
    and the brain's ending reads as the D-pad's."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH, status=survey_status())
    sync_api.expect(page.locator("#brain-tel-outcome")).to_contain_text(
        "Surveying", timeout=5000)
    sent = []

    def capture(route):
        sent.append((route.request.headers.get("x-driver"), route.request.post_data_json))
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"executed": True, "driver": "twin-dpad", "result": {}}))

    page.route("**/action", capture)
    page.click("#btn-left")
    page.wait_for_timeout(500)
    assert any(d == "twin-dpad" for d, _ in sent), sent

    ended = survey_status(running=False, outcome="cancelled", end_state="cancelled",
                          cancelled_by="dpad")
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(ended)))
    sync_api.expect(page.locator("#brain-tel-outcome")).to_contain_text(
        "D-pad took over", timeout=8000)
    assert not errors, errors
    page.context.close()


# ---- review fixes (/code-review of S5, 2026-10-14), each red first ----


def test_a_brain_cannot_put_markup_in_the_survey_hint(browser, twin_server):
    """The cap is drawn into the hint's markup, and the page holds the
    robot and walks secrets: a brain (any origin a setup link points at)
    that answers a string must not run script. Only a number is a cap."""
    page, errors = open_with_brain(
        browser, twin_server, status=survey_status(running=False, outcome=None, rooms=(), calls=0),
        health=dict(HEALTH, room_label_max_calls='<img src=x id="injected" onerror="window.__pwned=1">'))
    page.select_option("#brain-job", "survey")
    sync_api.expect(page.locator("#brain-job-hint")).to_be_visible()
    page.wait_for_timeout(300)
    assert page.locator("#injected").count() == 0
    assert page.evaluate("window.__pwned") is None
    sync_api.expect(page.locator("#brain-job-hint")).to_contain_text("the brain's own cap")
    assert not errors, errors
    page.context.close()


def test_the_picker_returns_to_the_users_job_when_a_watched_survey_ends(browser, twin_server):
    """A survey started on another phone sets the picker while it runs; once
    it ends, this phone's Start must not quietly start another survey (paid
    calls on the car) where its person had chosen Find."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH,
                                   status=survey_status())
    sync_api.expect(page.locator("#brain-job")).to_have_value("survey", timeout=5000)
    ended = survey_status(running=False, outcome="cancelled", end_state="cancelled",
                          cancelled_by="stop")
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(ended)))
    sync_api.expect(page.locator("#brain-tel-outcome")).to_contain_text(
        "Cancelled", timeout=8000)
    sync_api.expect(page.locator("#brain-job")).to_have_value("find")
    sync_api.expect(page.locator("#brain-target")).to_be_visible()
    assert not errors, errors
    page.context.close()


def test_a_survey_that_finishes_is_shown_and_toasted_as_a_success(browser, twin_server):
    """Exhausted is a survey's success: green on the panel, and the toast
    says the survey is done -- not a find's "nowhere reachable left"."""
    page, errors = open_with_brain(browser, twin_server, health=HEALTH,
                                   status=survey_status())
    sync_api.expect(page.locator("#brain-tel-outcome")).to_contain_text(
        "Surveying", timeout=5000)
    done = survey_status(running=False, outcome="exhausted", end_state="exhausted")
    page.unroute("**/brain-stub/mission/status")
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(done)))
    outcome = page.locator("#brain-tel-outcome")
    sync_api.expect(outcome).to_contain_text("Survey done", timeout=8000)
    sync_api.expect(outcome).to_have_class("val safe")
    toast = page.locator(".toast", has_text="Survey done")
    toast.first.wait_for(timeout=3000)
    assert "ok" in toast.first.get_attribute("class")
    assert page.locator(".toast", has_text="nowhere reachable").count() == 0
    assert not errors, errors
    page.context.close()
