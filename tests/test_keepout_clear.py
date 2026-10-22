"""3.76 K5.3 (the user, 2026-10-21: "clear at mission end"): a mission's
keepout marks are cleared when it ends, whatever the end, so the next
mission starts on a clean map. The 300 s expiry (`SPOT_TTL_S`) still runs
within a mission. The robot server owns the marks (`robot/wedge_watch.py`'s
`SpotBook`), so it clears them (`DELETE /world/keepout`); the brain asks
through its navigator once the mission's own goal is ended."""

import logging

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from control.mission_runner import _GatedNavigator
from control.remote_navigator import RemoteNavigator
from tests.test_wedge_watch import drive_refused, wait_for, wedged_server  # noqa: F401


def _wedge_once(client, bridge, posts_before):
    drive_refused(client, 0.5)
    assert wait_for(lambda: bridge.keepout_posts > posts_before), "no mark sent"


def test_clearing_empties_the_bridge_and_the_book(wedged_server):
    client, bridge = wedged_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _wedge_once(client, bridge, 0)
    assert len(bridge.keepout) == 1
    r = client.delete("/world/keepout")
    assert r.status_code == 200, r.text
    assert r.json()["cleared"] == 1 and r.json()["sent"] is True, r.json()
    assert bridge.keepout == []
    # The book is empty too: the same spot marks again (3.83's dedupe saw a
    # live spot there before), and the set sent holds only the new one.
    posts = bridge.keepout_posts
    _wedge_once(client, bridge, posts)
    assert len(bridge.keepout) == 1


def test_the_clear_is_logged_where_live_runs_keep_the_log(wedged_server, caplog):
    """The live tours run the robot server at WARNING (`tests/demo_explore.py`),
    and K5.3's tour is judged on this line after every mission with live
    marks; at INFO it never reached the log (2026-10-22, 7 rooms, 0 lines)."""
    client, bridge = wedged_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _wedge_once(client, bridge, 0)
    with caplog.at_level(logging.WARNING):
        client.delete("/world/keepout")
    assert any("1 mark(s) cleared at a mission's end" in r.getMessage()
               and r.levelno >= logging.WARNING for r in caplog.records), caplog.text


def test_clearing_with_nothing_marked_sends_nothing(wedged_server):
    client, bridge = wedged_server
    r = client.delete("/world/keepout")
    assert r.status_code == 200 and r.json() == {"cleared": 0, "sent": False}, r.text
    assert bridge.keepout_posts == 0


def test_a_world_without_keepout_clears_nothing(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = TestClient(server.create_app())          # the plain simulator
    r = client.delete("/world/keepout")
    assert r.status_code == 200 and r.json() == {"cleared": 0, "sent": False}, r.text


def test_the_brain_clears_over_http(wedged_server):
    client, bridge = wedged_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _wedge_once(client, bridge, 0)
    nav = RemoteNavigator("http://testserver", client=client)
    assert nav.clear_keepout()["cleared"] == 1
    assert bridge.keepout == []


def test_an_older_server_without_the_route_is_not_an_error():
    import httpx
    old = httpx.Client(transport=httpx.MockTransport(lambda req: httpx.Response(404)),
                       base_url="http://old")
    assert RemoteNavigator("http://old", client=old).clear_keepout() == {
        "cleared": 0, "reason": "unsupported"}


class _Nav:
    def __init__(self, fail_clear=False):
        self.calls, self.fail_clear = [], fail_clear

    def set_goal(self, x, y):
        self.calls.append("set_goal")
        return {"accepted": True}

    def get_goal(self):
        self.calls.append("get_goal")
        return {"goal": {"state": "active", "x_m": 1.0, "y_m": 2.0}}

    def cancel_goal(self):
        self.calls.append("cancel_goal")
        return {"canceled": True}

    def clear_keepout(self):
        self.calls.append("clear_keepout")
        if self.fail_clear:
            raise RuntimeError("bridge down")
        return {"cleared": 1}

    def close(self):
        self.calls.append("close")


@pytest.mark.parametrize("fail_clear", [False, True])
def test_a_mission_end_clears_its_marks_after_its_goal(fail_clear):
    running = [True]
    nav = _Nav(fail_clear)
    gated = _GatedNavigator(nav, lambda: running[0])
    gated.set_goal(1.0, 2.0)
    running[0] = False
    gated.end()                       # never raises, even when the clear fails
    assert nav.calls == ["set_goal", "get_goal", "cancel_goal", "clear_keepout", "close"]


def test_a_mission_with_no_goal_of_its_own_still_clears():
    nav = _Nav()
    gated = _GatedNavigator(nav, lambda: False)
    gated.end()
    assert nav.calls == ["clear_keepout", "close"]


def test_a_navigator_that_cannot_clear_is_left_alone():
    class Plain:                      # tests/test_explore.py's FakeNav has no clear
        def set_goal(self, x, y): return {"accepted": True}
        def get_goal(self): return {"goal": None}
        def cancel_goal(self): return {}
    _GatedNavigator(Plain(), lambda: False).end()


def test_a_goal_on_its_way_at_the_end_is_cleared_after_it():
    """Thermos round 1: a goal accepted after end() had cleared could leave
    marks for 300 s. The clear runs when the last of end() and a goal in
    flight finishes, so it follows that goal's own cancel."""
    import threading
    running = [True]
    entered, release = threading.Event(), threading.Event()

    class Slow(_Nav):
        def set_goal(self, x, y):
            entered.set()
            release.wait(5)
            return super().set_goal(x, y)

    nav = Slow()
    gated = _GatedNavigator(nav, lambda: running[0])
    t = threading.Thread(target=gated.set_goal, args=(1.0, 2.0))
    t.start()
    assert entered.wait(5)
    running[0] = False
    gated.end()                               # the goal is still on its way
    assert "clear_keepout" not in nav.calls
    release.set()
    t.join(5)
    assert nav.calls[-2:] == ["clear_keepout", "close"], nav.calls
    assert nav.calls.index("cancel_goal") < nav.calls.index("clear_keepout"), nav.calls
