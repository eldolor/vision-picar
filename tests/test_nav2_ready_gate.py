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


def test_a_nav2_that_never_activates_turns_health_not_ok(bridge_mod):
    """Review: the hold is a wait the brain never ends; past NAV2_STUCK_S
    of inactivity /health says so (ok false), as a failed SLAM start does."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(False)
    assert b.health()[1]["ok"] is True
    b._nav2_inactive_since -= bridge_mod.NAV2_STUCK_S + 1
    body = b.health()[1]
    assert body["ok"] is False and body["nav2"]["inactive_s"] > bridge_mod.NAV2_STUCK_S
    b._poll_nav2()
    m.answer(True)
    assert b.health()[1]["ok"] is True


def test_nav2_going_inactive_during_the_send_is_seen_under_the_lock(bridge_mod):
    """Review: the first read is before wait_for_server (up to 1 s); the
    send re-reads it under the lock, as it does slam.state."""
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


def test_the_stuck_count_runs_only_while_slam_is_ready(bridge_mod):
    """Review: a slow reload must not spend nav2's NAV2_STUCK_S."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b.slam_state = "checking"
    b._nav2_inactive_since -= bridge_mod.NAV2_STUCK_S + 1
    assert b.health()[1]["ok"] is True
    b.slam_state = "ready"
    assert b.health()[1]["ok"] is False


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
