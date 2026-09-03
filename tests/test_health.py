"""
tests/test_health.py

Phase M5 (PLAN-microduck-transplants.md) -- one health command.

Two things are under test, and the second is the one that rots.

**The verdict**, which has to be non-zero when either half is unreachable
or broken, because M11 gates a release rollback on this exit code and a
boot timer runs it with nobody reading the output.

**The rule about what may reach that verdict** (Microduck's
`robotd-design` section 3.4, invariant 5): only conditions a release can
be blamed for. A robot updated on a low battery must not roll its release
back and then judge the replacement on the same battery. The local
equivalents are a robot sitting against a wall, a robot nobody has
commanded for a while, and a safety veto on record -- all normal, all
reported, none of them a verdict input. A health check that goes red
because a robot is stationary is one everybody learns to ignore, so the
tests below pin the *silence* of the verdict as carefully as its noise.

Run with: pytest tests/test_health.py -v
"""

import httpx
import pytest

from control import health


def stub(robot_body=None, brain_body=None, robot_status=200, brain_status=200):
    """A transport answering both /health routes, so a test can state one
    field and nothing else. Distinguishing the two by port is what the real
    command does -- there is no shared client to confuse them."""

    def handler(request: httpx.Request) -> httpx.Response:
        if "8000" in str(request.url):
            if robot_body is None:
                raise httpx.ConnectError("connection refused")
            return httpx.Response(robot_status, json=robot_body)
        if brain_body is None:
            raise httpx.ConnectError("connection refused")
        return httpx.Response(brain_status, json=brain_body)

    return httpx.MockTransport(handler)


HEALTHY_ROBOT = {
    "status": "ok", "mode": "sim",
    "seconds_since_last_command": 0.2, "watchdog_timeout_s": 1.0,
    "seconds_since_watchdog_poll": 0.05, "watchdog_poll_interval_s": 0.1,
    "identity": {"git_revision": "abc1234", "executable": "/usr/bin/python3"},
}
HEALTHY_BRAIN = {
    "status": "ok", "robot_url": "http://127.0.0.1:8000",
    "mission_running": False, "tick_timeout_s": 30.0,
    "identity": {"git_revision": "abc1234", "executable": "/usr/bin/python3"},
}


@pytest.fixture
def patched(monkeypatch):
    """Point control.health's own httpx.get at a stub transport."""

    def install(**kwargs):
        transport = stub(**kwargs)
        client = httpx.Client(transport=transport)
        monkeypatch.setattr(health.httpx, "get", client.get)
        return client

    return install


# ---------- the verdict ----------


def test_both_halves_healthy_exits_zero(patched, capsys):
    patched(robot_body=HEALTHY_ROBOT, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0
    assert "OK" in capsys.readouterr().out


def test_an_unreachable_brain_is_non_zero_and_says_which_half(patched, capsys):
    """The press-this for this phase: kill the brain, and the command has
    to name the half that failed rather than just going red."""
    patched(robot_body=HEALTHY_ROBOT, brain_body=None)
    assert health.main([]) == 1
    out = capsys.readouterr().out
    assert "UNHEALTHY" in out and "brain" in out
    assert "robot  ok" in out, "the healthy half must still report as healthy"


def test_an_unreachable_robot_is_non_zero(patched):
    patched(robot_body=None, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 1


def test_a_stale_watchdog_poll_is_non_zero(patched, capsys):
    """The failure nothing else in this project can see. The server keeps
    answering every request and keeps reporting a growing silence, while
    the loop that is supposed to act on that silence is gone."""
    body = {**HEALTHY_ROBOT, "seconds_since_watchdog_poll": 4.0}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 1
    assert "the guard is not running" in capsys.readouterr().out


def test_a_slow_poll_is_not_a_stalled_one(patched):
    """A single late event-loop tick is not a fault. The failure being
    caught is a task that is gone, not one that is behind."""
    body = {**HEALTHY_ROBOT, "seconds_since_watchdog_poll": 0.3}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0


def test_a_mission_whose_loop_stopped_turning_is_non_zero(patched, capsys):
    """B3.3 structurally cannot report this: it aborts a hung tick from
    inside the very loop that is no longer running."""
    body = {**HEALTHY_BRAIN, "mission_running": True,
            "seconds_since_last_tick": 45.0, "tick_timeout_s": 30.0}
    patched(robot_body=HEALTHY_ROBOT, brain_body=body)
    assert health.main([]) == 1
    assert "the mission loop is not turning" in capsys.readouterr().out


def test_a_mission_ticking_normally_is_healthy(patched):
    body = {**HEALTHY_BRAIN, "mission_running": True,
            "seconds_since_last_tick": 0.4, "tick_timeout_s": 30.0,
            "tick_rate_hz": 3.9}
    patched(robot_body=HEALTHY_ROBOT, brain_body=body)
    assert health.main([]) == 0


def test_a_long_gap_with_no_mission_running_is_not_a_fault(patched):
    """An idle brain has no ticks and owes none. Judging it on a deadline
    that only applies to a running mission would make "nothing to do" look
    like "broken"."""
    body = {**HEALTHY_BRAIN, "mission_running": False,
            "seconds_since_last_tick": 999.0}
    patched(robot_body=HEALTHY_ROBOT, brain_body=body)
    assert health.main([]) == 0


# ---------- what may NOT reach the verdict ----------


def test_a_robot_against_a_wall_is_healthy(patched, capsys):
    """The plan's own test, and the rule in one line: a 0.0cm reading with
    everything else fine exits zero and is reported as description. This is
    the local version of Microduck's low battery -- a fact about the
    situation, not about the build."""
    body = {**HEALTHY_ROBOT, "distance_cm": 0.0,
            "last_refusal": {"reason": "safety_distance", "detail": "Blocked FORWARD",
                             "driver": "twin-dpad", "seconds_ago": 0.2}}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0
    assert "safety_distance" in capsys.readouterr().out, (
        "it must still be reported -- description means printed, not hidden")


def test_a_long_client_silence_is_not_a_fault(patched):
    """`seconds_since_last_command` measures whether anyone is driving. A
    parked robot is not a broken one, and gating a rollback on this would
    roll back every release deployed outside working hours."""
    body = {**HEALTHY_ROBOT, "seconds_since_last_command": 3600.0}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0


def test_a_preemption_on_record_is_not_a_fault(patched):
    """Someone took the robot. That is M4 working, not a release to blame."""
    body = {**HEALTHY_ROBOT,
            "last_refusal": {"reason": "preempted", "detail": "outranked",
                             "driver": "brain", "seconds_ago": 5.0}}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0


def test_a_server_older_than_m5_is_not_judged_on_fields_it_lacks(patched):
    """The stacks are redeployed one at a time. A server that publishes no
    watchdog poll age cannot be found unhealthy for it -- absent is not
    stale."""
    body = {k: v for k, v in HEALTHY_ROBOT.items()
            if k not in ("seconds_since_watchdog_poll", "watchdog_poll_interval_s")}
    patched(robot_body=body, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 0


def test_a_non_200_health_route_is_unreachable_not_ok(patched):
    """A 500 from the health route is not a healthy service that happens to
    be shouting."""
    patched(robot_body={"detail": "boom"}, robot_status=500, brain_body=HEALTHY_BRAIN)
    assert health.main([]) == 1


# ---------- the report itself ----------


def test_json_carries_the_same_verdict_as_the_text(patched, capsys):
    import json as jsonlib

    patched(robot_body={**HEALTHY_ROBOT, "seconds_since_watchdog_poll": 4.0},
            brain_body=HEALTHY_BRAIN)
    assert health.main(["--json"]) == 1
    report = jsonlib.loads(capsys.readouterr().out)
    assert report["status"] == "unhealthy"
    assert report["failed"] == ["robot"]


def test_the_report_names_the_build_that_answered(patched, capsys):
    """M11 rolls back on this command, so "which build said it was fine" is
    the first thing anyone reading the output afterwards needs. The
    executable path is the field that separates "the update worked" from
    "the symlink moved"."""
    patched(robot_body=HEALTHY_ROBOT, brain_body=HEALTHY_BRAIN)
    health.main([])
    out = capsys.readouterr().out
    assert "abc1234" in out and "/usr/bin/python3" in out
