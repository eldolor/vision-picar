"""
3.105: the bridge holds goals until nav2's lifecycle manager answers active.

After a container start on a saved map the reload is judged (~1.7 s) long
before nav2's nodes are active (~13 s): `navigate_to_pose` has a server, so
a goal was answered 200 `pending` and then rejected by nav2 -- which
explore counts as a failed goal. A 503 is a fault explore waits on.
"""
import pathlib
import types

from tests.test_review_376h1 import _Future, _Handle, _Nav, bridge_mod  # noqa: F401
from world import bridge_faults


class _Manager:
    """nav2's lifecycle manager's `is_active` (std_srvs/Trigger) as a client."""

    def __init__(self, up=True):
        self.up = up
        self.calls = []

    def service_is_ready(self):
        return self.up

    def call_async(self, req):
        f = _Future()
        self.calls.append(f)
        return f

    def answer(self, active):
        self.calls[-1].complete(types.SimpleNamespace(success=active, message=""))


def _bridge(bridge_mod, manager=None):
    b = bridge_mod.Bridge()
    b.nav = _Nav()
    b.slam_state = "ready"
    if manager is not None:
        b.nav2_cli = manager
    return b


def _refused(b):
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503, (code, body)
    assert bridge_faults.retryable_status(code)
    assert "nav2 is not active" in body["error"]
    assert b.nav.futures == [], "nothing may be sent to nav2"
    return body


def test_a_fresh_bridge_holds_goals_until_nav2_answers(bridge_mod):
    """Red on dev: answered 200 and sent at once."""
    b = _bridge(bridge_mod, _Manager())
    _refused(b)


def test_goals_go_once_nav2_answers_active(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    _refused(b)                                  # asked, not yet answered
    m.answer(True)
    assert b.health()[1]["nav2"]["active"] is True
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 200 and body == {"state": "pending"}
    assert len(b.nav.futures) == 1


def test_an_inactive_answer_holds_goals(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(False)
    body = _refused(b)
    assert b.health()[1]["nav2"]["active"] is False
    assert "nav2" in body


def test_nav2_reset_after_active_holds_goals_again(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()
    m.answer(False)
    _refused(b)


def test_no_lifecycle_manager_holds_goals(bridge_mod):
    m = _Manager(up=False)
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    assert m.calls == [], "no service: not asked"
    body = _refused(b)
    assert "no lifecycle manager" in body["nav2"]["why"]


def test_an_unanswered_ask_reads_inactive_and_is_asked_again(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()                                # in flight, never answered
    b._poll_nav2()
    assert len(m.calls) == 2, "one ask in flight at a time"
    b._nav2_ask_since -= bridge_mod.NAV2_ASK_TIMEOUT_S + 0.1
    b._poll_nav2()
    _refused(b)
    assert len(m.calls) == 3, "a lost ask is given up and asked again"
    m.answer(True)
    assert b.send_goal({"x_m": 1.0, "y_m": 0.0})[0] == 200


def test_a_late_answer_to_a_given_up_ask_is_ignored(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    stale = m.calls[-1]
    b._nav2_ask_since -= bridge_mod.NAV2_ASK_TIMEOUT_S + 0.1
    b._poll_nav2()                               # gives it up, asks again
    m.answer(False)
    stale.complete(types.SimpleNamespace(success=True, message=""))
    _refused(b)


def test_a_failed_call_reads_inactive(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()

    def boom():
        raise RuntimeError("service call failed")
    m.calls[-1].result = boom
    m.calls[-1].complete(None)
    _refused(b)


def test_the_hold_cancels_nothing(bridge_mod):
    """A goal refused for nav2 leaves the live one alone, as the SLAM hold does."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    handle = _Handle()
    b.nav.futures[-1].complete(handle)
    b._poll_nav2()
    m.answer(False)
    code, _ = b.send_goal({"x_m": 2.0, "y_m": 0.0})
    assert code == 503 and handle.cancels == 0
    assert b.goal_state()[1]["goal"]["state"] == "active"


def test_health_says_how_long_ago_it_was_answered(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    nav2 = b.health()[1]["nav2"]
    assert nav2["active"] is False and nav2["why"] == "not asked yet"
    assert nav2["answer_age_s"] is None
    b._poll_nav2()
    m.answer(True)
    nav2 = b.health()[1]["nav2"]
    assert nav2["active"] is True and nav2["why"] is None
    assert 0 <= nav2["answer_age_s"] < 5 and nav2["inactive_s"] is None


def test_health_counts_how_long_nav2_has_been_inactive(bridge_mod):
    """A nav2 that never comes up holds explore until its time limit; /health
    shows for how long (nothing acts on it: review, round 4)."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._nav2_inactive_since -= 130
    assert b.health()[1]["nav2"]["inactive_s"] >= 130
    assert b.health()[1]["ok"] is True
    b._poll_nav2()
    m.answer(True)
    assert b.health()[1]["nav2"]["inactive_s"] is None


def test_a_nav2_reset_during_the_wait_leaves_the_live_goal_alone(bridge_mod):
    """Review: the first read is before wait_for_server (up to 1 s); it is
    read again after the wait and before the cancel."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    handle = _Handle()
    b.nav.futures[-1].complete(handle)
    wait = b.nav.wait_for_server

    def reset_meanwhile(timeout_sec):
        b._poll_nav2()
        m.answer(False)
        return wait(timeout_sec)
    b.nav.wait_for_server = reset_meanwhile
    code, body = b.send_goal({"x_m": 2.0, "y_m": 0.0})
    assert code == 503 and "nav2 is not active" in body["error"]
    assert body["nav2"]["active"] is False
    assert len(b.nav.futures) == 1 and handle.cancels == 0, "the live goal is left alone"


def test_a_change_during_the_cancel_sends_nothing(bridge_mod):
    """The last read, under the lock before the send (3.50.D4's guarantee
    for a re-map, and 3.105's for nav2): a change during cancel_goal()
    sends no goal."""
    for change in ("remap", "nav2"):
        m = _Manager()
        b = _bridge(bridge_mod, m)
        b._poll_nav2()
        m.answer(True)
        cancel = b.cancel_goal

        def change_meanwhile(*a, **kw):
            if change == "remap":
                b.slam_state = "remapping"
            else:
                b._poll_nav2()
                m.answer(False)
            return cancel(*a, **kw)
        b.cancel_goal = change_meanwhile
        code, _ = b.send_goal({"x_m": 2.0, "y_m": 0.0})
        assert code == 503 and b.nav.futures == [], change


def test_a_re_map_during_the_wait_leaves_the_live_goal_alone(bridge_mod):
    """Review: the SLAM re-read is before the cancel too (it was after)."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})
    handle = _Handle()
    b.nav.futures[-1].complete(handle)
    wait = b.nav.wait_for_server

    def remap_meanwhile(timeout_sec):
        b.slam_state = "remapping"
        return wait(timeout_sec)
    b.nav.wait_for_server = remap_meanwhile
    code, body = b.send_goal({"x_m": 2.0, "y_m": 0.0})
    assert code == 503 and body["slam"] == "remapping"
    assert handle.cancels == 0 and len(b.nav.futures) == 1


def test_a_given_up_ask_leaves_rclpys_pending_requests(bridge_mod):
    m = _Manager()
    removed = []
    m.remove_pending_request = removed.append
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    stale = m.calls[-1]
    b._nav2_ask_since -= bridge_mod.NAV2_ASK_TIMEOUT_S + 0.1
    b._poll_nav2()
    assert removed == [stale]
    assert "starting nav2, or hung" in b.health()[1]["nav2"]["why"]


def test_the_manager_asked_is_the_one_the_launch_file_starts(bridge_mod):
    """Review: a rename in picar.launch.py would hold every goal for good."""
    launch = (pathlib.Path(__file__).resolve().parents[1] / "service" / "slam" / "src"
              / "picar_bringup" / "launch" / "picar.launch.py").read_text()
    name = bridge_mod.NAV2_MANAGER.strip("/").split("/")[0]
    assert f'name="{name}"' in launch
    assert bridge_mod.NAV2_MANAGER == f"/{name}/is_active"
