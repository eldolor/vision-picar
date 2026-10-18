"""3.80.B: the local VLM service against a fake driver that speaks
edgellm_identity's protocol (tests/fake_edgellm_driver.py)."""

import base64
import os
import sys
import threading
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from service.local_vlm.driver import DriverGone, EdgeDriver
from service.local_vlm.server import create_app

FAKE = Path(__file__).with_name("fake_edgellm_driver.py")
VOCAB = {6: "[{", 7: "[]", 9: "x",
         # 3.80.D: the rest of a box, `[{"bbox_2d": [282, 40, 362, 160]}]`
         # (centred in a 640-wide frame as Qwen sees it, 644 wide).
         10: '"bbox_2d": [', 11: "282", 12: ", ", 13: "40", 14: "362", 15: "160",
         16: "]}]",
         # 3.80.F: the direction words.
         20: " left", 21: " Left", 22: "right", 23: " forward", 24: "for", 25: " ",
         26: "Move", 27: ":"}
JPEG = b"\xff\xd8\xff\xe0" + b"\0" * 64
IMAGE = base64.b64encode(JPEG).decode()


def make_driver(tmp_path, **env):
    return EdgeDriver([sys.executable, str(FAKE)], dict(os.environ, **env),
                      VOCAB.get, workdir=str(tmp_path))


@pytest.fixture
def drv(tmp_path):
    d = make_driver(tmp_path)
    d.wait_ready(10)
    yield d
    d.close()


def test_yes_is_scored_and_cancelled_at_the_decision(drv, tmp_path):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "yes thing", 5.0)
    assert got["status"] == "answered" and got["parsed"]
    assert got["score"] == pytest.approx(0.9)
    # Returned at the deciding token; the cancelled tail is drained behind it
    # well before 64 filler tokens (1.3 s) would have run.
    assert drv.lock.acquire(timeout=1.0)
    drv.lock.release()
    assert time.monotonic() - t0 < 1.0 and drv.served == 1
    # The request and image files are gone once the tail is drained.
    assert list(tmp_path.iterdir()) == []


def test_no_scores_low(drv):
    got = drv.ask(JPEG, "plain thing", 5.0)
    assert got["status"] == "answered" and got["score"] == pytest.approx(0.05)


def test_deadline_cancels_and_the_driver_stays_usable(drv):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "slow thing", 0.3)
    assert got["status"] == "timeout" and got["score"] is None
    assert time.monotonic() - t0 < 0.8
    assert drv.ask(JPEG, "yes thing", 5.0)["score"] == pytest.approx(0.9)


def test_unparsed_and_error_are_unable_not_answers(drv):
    assert drv.ask(JPEG, "noparse thing", 5.0)["status"] == "unparsed"
    assert drv.ask(JPEG, "error thing", 5.0)["status"] == "error"
    assert drv.ask(JPEG, "yes thing", 5.0)["status"] == "answered"


def test_busy_when_the_runtime_is_held_past_the_deadline(drv):
    slow = threading.Thread(target=drv.ask, args=(JPEG, "slow thing", 1.0))
    slow.start()
    time.sleep(0.1)
    got = drv.ask(JPEG, "yes thing", 0.2)
    slow.join()
    assert got["status"] == "busy"
    # The held request ended cleanly; the next one is answered.
    assert drv.ask(JPEG, "yes thing", 5.0)["status"] == "answered"


def test_a_dead_driver_raises(drv):
    with pytest.raises(DriverGone):
        drv.ask(JPEG, "die thing", 5.0)
    with pytest.raises(DriverGone):
        drv.ask(JPEG, "yes thing", 5.0)


def test_a_wedged_driver_is_killed(drv, monkeypatch):
    import service.local_vlm.driver as d
    monkeypatch.setattr(d, "CANCEL_GRACE_S", 0.3)
    assert drv.ask(JPEG, "wedge thing", 0.2)["status"] == "timeout"
    drv.proc.wait(timeout=5)  # killed by the drain once the grace ran out
    assert not drv.alive
    with pytest.raises(DriverGone):
        drv.ask(JPEG, "yes thing", 5.0)


# --- the HTTP app -----------------------------------------------------------

def app_client(tmp_path, monkeypatch, secret=None, make=None, **env):
    if secret:
        monkeypatch.setenv("APP_SHARED_SECRET", secret)
    else:
        monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    deaths = []
    app = create_app(make or (lambda: make_driver(tmp_path, **env)), info={"engines": "fake"},
                     on_death=lambda: deaths.append(1), ready_timeout_s=10,
                     watch_interval_s=0.05, death_delay_s=0.0)
    return TestClient(app), deaths


def wait_ready(c, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if c.get("/health").json()["ready"]:
            return
        time.sleep(0.05)
    raise AssertionError("not ready")


def test_http_answers_and_reports_health(tmp_path, monkeypatch):
    c, deaths = app_client(tmp_path, monkeypatch)
    with c:
        wait_ready(c)
        r = c.post("/identity", json={"image_b64": IMAGE, "target": "yes thing"})
        assert r.status_code == 200 and r.json()["score"] == pytest.approx(0.9)
        h = c.get("/health").json()
        assert h["alive"] and h["served"] == 1 and h["engines"] == "fake"
    assert deaths == []  # a shutdown we asked for is not a death


def test_http_503_while_loading_and_when_absent(tmp_path, monkeypatch):
    c, deaths = app_client(tmp_path, monkeypatch, FAKE_NEVER_READY="1")
    with c:
        assert c.get("/health").json()["ready"] is False
        r = c.post("/identity", json={"image_b64": IMAGE, "target": "yes thing"})
        assert r.status_code == 503


def test_http_driver_death_is_503_and_reported_once(tmp_path, monkeypatch):
    c, deaths = app_client(tmp_path, monkeypatch)
    with c:
        wait_ready(c)
        r = c.post("/identity", json={"image_b64": IMAGE, "target": "die thing"})
        assert r.status_code == 503
        end = time.monotonic() + 3
        while not deaths and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(0.2)  # the watcher sees the exit too, and must not count it again
        assert deaths == [1]
        assert c.get("/health").json()["alive"] is False
        assert c.post("/identity", json={"image_b64": IMAGE, "target": "yes"}).status_code == 503


def test_http_watcher_sees_a_death_between_requests(tmp_path, monkeypatch):
    c, deaths = app_client(tmp_path, monkeypatch)
    with c:
        wait_ready(c)
        h = c.get("/health").json()
        assert h["alive"]
        # Kill the driver the way the OOM killer would.
        os.system(f"pkill -9 -f {FAKE.name} -P {os.getpid()}")
        end = time.monotonic() + 3
        while not deaths and time.monotonic() < end:
            time.sleep(0.05)
        assert deaths == [1]


def test_http_rejects_bad_input_and_wants_the_secret(tmp_path, monkeypatch):
    c, _ = app_client(tmp_path, monkeypatch, secret="s3")
    with c:
        wait_ready(c)
        body = {"image_b64": IMAGE, "target": "yes thing"}
        assert c.post("/identity", json=body).status_code == 401
        h = {"x-app-secret": "s3"}
        assert c.post("/identity", json=body, headers=h).status_code == 200
        bad = {"image_b64": "!!notb64", "target": "yes"}
        assert c.post("/identity", json=bad, headers=h).status_code == 400
        png = {"image_b64": base64.b64encode(b"\x89PNG....").decode(), "target": "yes"}
        assert c.post("/identity", json=png, headers=h).status_code == 400
        long = {"image_b64": IMAGE, "target": "yes", "deadline_s": 999}
        assert c.post("/identity", json=long, headers=h).status_code == 422


def test_b1_compare_flags_drift_flips_and_missing_frames():
    from tools.jetson.bench_local_vlm import compare
    old = [{"walk": "w", "frame": f"f{i}", "score": s} for i, s in enumerate([0.0, 0.9, 0.52])]
    same = [dict(r, score=r["score"] + 0.005) for r in old]
    assert compare(same, old)["met"]
    drift = [dict(old[0]), dict(old[1], score=0.95), dict(old[2], score=0.49)]
    res = compare(drift, old)
    assert not res["met"] and res["flips"] == ["w/f2"] and len(res["off"]) == 2
    gone = [dict(old[0], score=None)] + old[1:]
    assert compare(gone, old)["missing"] == ["w/f0"]
    assert not compare(old[:2], old)["met"]  # a frame not asked at all


def test_no_time_left_is_a_timeout_and_nothing_is_sent(drv, monkeypatch):
    import service.local_vlm.driver as d
    monkeypatch.setattr(d, "MIN_SEND_S", 0.5)
    got = drv.ask(JPEG, "yes thing", 0.2)
    assert got["status"] == "timeout" and got["driver_ms"] is None
    assert drv.served == 0  # the driver never saw it
    assert drv.ask(JPEG, "yes thing", 5.0)["status"] == "answered"


def test_a_driver_that_cannot_start_is_a_death(tmp_path, monkeypatch):
    def boom():
        raise FileNotFoundError("no edgellm_identity here")
    c, deaths = app_client(tmp_path, monkeypatch, make=boom)
    with c:
        end = time.monotonic() + 3
        while not deaths and time.monotonic() < end:
            time.sleep(0.05)
        assert deaths == [1]
        assert c.get("/health").json()["ready"] is False


def test_the_deadline_bounds_the_reply_even_before_the_first_token(drv):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "prefill thing", 0.3)
    assert got["status"] == "timeout" and time.monotonic() - t0 < 0.5
    # The runtime is held until the cancelled request ends (~1 s): a short
    # request behind it is busy, a patient one is answered.
    assert drv.ask(JPEG, "yes thing", 0.2)["status"] == "busy"
    assert drv.ask(JPEG, "yes thing", 5.0)["score"] == pytest.approx(0.9)


def test_a_death_seen_by_one_reader_is_seen_by_the_next(drv):
    with pytest.raises(DriverGone):
        drv.ask(JPEG, "die thing", 5.0)
    drv.proc.wait(timeout=5)
    for _ in range(2):
        with pytest.raises(DriverGone):
            drv._next(0.1)


# --- 3.80.D: the box, for sightings (D9) -----------------------------------

BOX = [282.0, 40.0, 362.0, 160.0]


def test_d9_box_yes_returns_the_same_score_and_the_box(drv):
    plain = drv.ask(JPEG, "yes thing", 5.0)
    got = drv.ask(JPEG, "yes thing", 5.0, box=True)
    assert got["status"] == "answered" and got["score"] == pytest.approx(plain["score"])
    assert got["box"] == BOX
    # Without `box` the reply carries none: the arrival question is unchanged.
    assert plain.get("box") is None


def test_d9_box_no_ends_at_the_decision_with_no_box(drv):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "plain thing", 5.0, box=True)
    assert got["status"] == "answered" and got["score"] == pytest.approx(0.05)
    assert got["box"] is None and time.monotonic() - t0 < 1.0


def test_d9_a_deadline_after_the_decision_keeps_the_score_without_a_box(drv):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "boxslow thing", 0.5, box=True)
    assert got["status"] == "answered" and got["score"] == pytest.approx(0.9)
    assert got["box"] is None
    assert time.monotonic() - t0 < 0.9
    # The cancelled tail is drained; the runtime answers the next request.
    assert drv.ask(JPEG, "yes thing", 5.0, box=True)["box"] == BOX


def test_d9_http_box_flag(tmp_path, monkeypatch):
    c, _ = app_client(tmp_path, monkeypatch)
    with c:
        wait_ready(c)
        r = c.post("/identity", json={"image_b64": IMAGE, "target": "yes thing", "box": True})
        assert r.status_code == 200 and r.json()["box"] == BOX
        r = c.post("/identity", json={"image_b64": IMAGE, "target": "yes thing"})
        assert r.status_code == 200 and r.json().get("box") is None


# ---------- 3.80.F: the direction question ----------

def test_f_direction_reads_the_first_word_and_its_alternatives(drv):
    got = drv.ask(JPEG, "goleft thing", 5.0, question="direction")
    assert got["status"] == "answered" and got["parsed"] and got["score"] is None
    # `left` 0.6 and ` Left` 0.2 are left; `for` 0.15 is forward; right 0.05.
    assert got["direction"] == "left"
    assert got["probs"] == pytest.approx({"left": 0.8, "right": 0.05, "forward": 0.15})
    assert got["mass"] == pytest.approx(1.0) and got["p"] == pytest.approx(0.8)
    # Cancelled at the word: the runtime is free again at once.
    assert drv.lock.acquire(timeout=1.0)
    drv.lock.release()


@pytest.mark.parametrize("target,want", [("goright x", "right"), ("gofwd x", "forward"),
                                         ("gospace goright x", "right")])
def test_f_direction_each_word_and_a_leading_space(drv, target, want):
    got = drv.ask(JPEG, target, 5.0, question="direction")
    assert got["status"] == "answered" and got["direction"] == want


def test_f_direction_little_mass_on_the_words_is_unable(drv):
    got = drv.ask(JPEG, "gounsure x", 5.0, question="direction")
    assert got["status"] == "unparsed" and got["direction"] is None
    assert got["mass"] == pytest.approx(0.3)


def test_f_direction_deadline_and_identity_unchanged_after(drv):
    t0 = time.monotonic()
    got = drv.ask(JPEG, "goslow x", 0.5, question="direction")
    # Filler that never reaches a word: the deadline ends it.
    assert got["status"] == "timeout" and time.monotonic() - t0 < 0.8
    # Nothing at all before the deadline: timeout.
    assert drv.lock.acquire(timeout=2.0)
    drv.lock.release()
    got = drv.ask(JPEG, "goslow x", 0.15, question="direction")
    assert got["status"] == "timeout" and got["direction"] is None
    assert drv.lock.acquire(timeout=2.0)
    drv.lock.release()
    # The identity question still works on the same runtime after it.
    assert drv.ask(JPEG, "yes thing", 5.0)["score"] == pytest.approx(0.9)


def test_f_option_prefixes():
    from service.local_vlm.driver import _option
    assert [_option(t) for t in ("L", " for", "Forward.", "RIGHT", "r", "", "x", "fo",
                                 "Move", ":")] == \
        ["left", "forward", "forward", "right", "right", None, None, "forward", None, None]


def test_f_a_reply_that_ends_without_a_word_is_unparsed(drv):
    # "noparse": filler, then the end id -- no token was one of the words.
    got = drv.ask(JPEG, "noparse", 5.0, question="direction")
    assert got["status"] == "unparsed" and got["direction"] is None


def test_f_http_direction_route(tmp_path, monkeypatch):
    c, _ = app_client(tmp_path, monkeypatch, secret="s3cret")
    with c:
        wait_ready(c)
        body = {"image_b64": IMAGE, "target": "goright x"}
        assert c.post("/direction", json=body).status_code == 401
        r = c.post("/direction", json=body, headers={"x-app-secret": "s3cret"})
        assert r.status_code == 200 and r.json()["direction"] == "right"
