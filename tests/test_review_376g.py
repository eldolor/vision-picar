"""
tests/test_review_376g.py

3.76 batch G: the whole-file review of the inventory and the late identity
check (`brain/inventory.py`, `control/inventory_store.py`,
`control/reconfirm.py`). One test per confirmed finding, each red on `dev`
before its fix; the ids are the findings table's in
`docs/plans/ros-alignment/3.76-legacy-review.md`.
"""

import threading
import time

from control.reconfirm import CloudWatch
from tests.test_arrival_reconfirm import (
    CLEAR_STARTS, FakeClockReconfirmer, SwitchableCloud, _ended_unconfirmed,
    _metrics_runner, _quiet_cloud, _wait_for)


# ---------- G1: a late call that outlives its guard ended the whole window ----------


class PacedReconfirmer(FakeClockReconfirmer):
    """A fake clock that also lets a little real time pass per probe, so a
    call on another thread can land inside the window. `on_probe(n)` runs
    before probe n (1-based)."""

    def __init__(self, *a, on_probe=None, on_wait=None, **k):
        super().__init__(*a, **k)
        self.on_probe = on_probe or (lambda n: None)
        self.on_wait = on_wait or (lambda n: None)
        self.n = 0
        inner = self.probe

        def probe():
            self.n += 1
            self.on_probe(self.n)
            return inner()
        self.probe = probe

    def _wait(self, seconds):
        self.on_wait(self.n)
        time.sleep(0.02)
        return super()._wait(seconds)


def test_g1_a_late_answer_that_outlived_its_guard_is_used_not_paid_twice():
    """Red on dev: the first call outlived its guard, the check was recorded
    `failed` ("hung") and returned -- and the round's own review: the first
    fix then paid a SECOND call for an answer already received."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    release = threading.Event()
    calls = {"n": 0}

    def gated(frame):
        calls["n"] += 1
        release.wait(5)
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = gated
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600,
                          on_probe=lambda n: release.set() if n == 3 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed", late
    assert calls["n"] == 1 and late["paid_calls"] == 1, (calls, late)
    # Taken as soon as it landed, not held to the half-window retry (probe 21).
    assert rc.n < 20, rc.n


def test_g1_the_last_allowed_call_is_still_answered_if_it_lands():
    """The round's own review: a timeout on the last attempt max_paid allows
    ended the check `failed`, though its paid reply landed a moment later."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    release = threading.Event()
    runner.vision_fn.cloud_vision_fn = lambda f: (release.wait(5), _quiet_cloud(f))[1]
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600, max_paid=1,
                          on_probe=lambda n: release.set() if n == 3 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 1, late


def test_g1_a_call_out_early_then_an_outage_reads_unreachable():
    """The round's own review: "still in flight" is about the end of the
    window; one that ended early, then an outage, is plainly unreachable."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    release = threading.Event()
    old = threading.Thread(target=release.wait, args=(5,), daemon=True)
    old.start()
    runner._confirm_thread = old
    rc = PacedReconfirmer(runner, lambda: rc.n <= 2, interval_s=15, window_s=600,
                          on_probe=lambda n: release.set() if n == 3 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "expired" and late["paid_calls"] == 0, late


def test_g1_an_earlier_confirmation_still_out_delays_the_ask_not_ends_it():
    """Red on dev: the mission's own arrival confirmation still out made the
    late check `failed` with no call sent, and the rest of the window was
    thrown away."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    cloud.up = True
    release = threading.Event()
    old = threading.Thread(target=release.wait, args=(5,), daemon=True)
    old.start()
    runner._confirm_thread = old
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600,
                          on_probe=lambda n: release.set() if n == 3 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 1, late
    # Not before the earlier call ended (released before probe 3).
    assert rc.n >= 3, "it asked before the earlier call ended"


def test_g1_a_probe_never_waits_on_a_call_still_out():
    """The round's own review: each probe joined the call still out for
    B3.2's timeout (20 s), so probes ran 35 s apart and a cancel waited."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    release = threading.Event()
    old = threading.Thread(target=release.wait, args=(5,), daemon=True)
    old.start()
    runner._confirm_thread = old
    runner.vision_timeout_s = 2.0
    from control.mission_runner import LateCallInFlight
    started = time.monotonic()
    try:
        try:
            runner.late_ask()
        except LateCallInFlight:
            pass
        assert time.monotonic() - started < 0.5
    finally:
        release.set()


def test_g1_blocked_all_window_is_not_unreachable():
    """The round's own review: a window spent behind a call that never ended
    read `expired`, "cloud still unreachable", with the cloud up throughout."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    release = threading.Event()
    old = threading.Thread(target=release.wait, args=(5,), daemon=True)
    old.start()
    runner._confirm_thread = old
    try:
        FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600).run()
    finally:
        release.set()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed" and "unreachable" not in late["reason"], late
    assert late["paid_calls"] == 0, late


def _gated(runner, outcome):
    """The cloud call blocks until released, then answers or raises."""
    release = threading.Event()

    def call(frame):
        release.wait(5)
        if isinstance(outcome, Exception):
            raise outcome
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = call

    def let_go():
        release.set()
        runner._confirm_thread.join(2)    # ended before the loop looks again
    return let_go


def test_g1_an_answer_landing_in_the_last_interval_is_kept():
    """Final review (both reviewers): the deadline was checked before a
    landed answer, so a paid `confirmed` arriving in the last interval was
    thrown away and the check recorded `failed`."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    let_go = _gated(runner, None)
    # Lands in the wait after the last probe (t=585), before the deadline.
    rc = PacedReconfirmer(runner, lambda: rc.n >= 39, interval_s=15, window_s=600,
                          max_paid=1, on_wait=lambda n: let_go() if n == 40 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 1, late


def test_g1_a_last_call_that_ends_in_an_error_ends_the_check_with_that_error():
    """Final review (both reviewers): the last allowed call timed out and
    then failed on its abandoned thread; the loop idled to the deadline with
    the record `waiting`, and ended quoting the timeout, not the error."""
    from brain.navigate import CloudUnavailable
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    let_go = _gated(runner, CloudUnavailable("HTTP 503: throttled"))
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600, max_paid=1,
                          on_probe=lambda n: let_go() if n == 3 else None)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "failed" and "503" in late["reason"], late
    assert rc.n < 10, rc.n


def test_g1_a_call_out_all_window_with_the_cloud_never_up_is_unreachable():
    """Final review (both reviewers): "the cloud answered" was recorded when
    no probe ever answered."""
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    release = threading.Event()
    old = threading.Thread(target=release.wait, args=(5,), daemon=True)
    old.start()
    runner._confirm_thread = old
    try:
        FakeClockReconfirmer(runner, lambda: False, interval_s=15, window_s=600).run()
    finally:
        release.set()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "expired", late


def test_g1_a_call_landing_after_a_cancel_ships_its_cost(monkeypatch):
    """Final review (both reviewers): a timed-out late call that ended after
    a cancel had made the record final never had its counters shipped."""
    import control.metrics_client as mc
    original, shipped = [], []
    monkeypatch.setattr(mc, "ship_run_async", lambda url, row, **k: original.append(row))
    monkeypatch.setattr(mc, "ship_run", lambda url, row, **k: shipped.append(row))
    cloud = SwitchableCloud()
    runner = _metrics_runner(cloud)
    runner.start()
    while runner.tick():
        pass
    cloud.up = True
    runner.vision_timeout_s = 0.05
    let_go = _gated(runner, None)
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600,
                          on_probe=lambda n: rc.cancel("a new mission") if n == 2 else None)
    rc.run()
    let_go()
    def timed(row):
        return (row["stats"].get("cloud_ms") or {}).get("n", 0)
    # cloud_calls counts a call when it starts; its time lands only when it ends.
    _wait_for(lambda: any(timed(r) > timed(original[0]) for r in shipped))
    assert shipped[-1]["stats"]["late_confirmation"]["state"] == "dropped"


def test_g1_an_answer_that_lands_at_the_guard_is_used_not_paid_again(monkeypatch):
    """The fixes' review (both reviewers): the guard saw the call alive, the
    call ended before late_ask() looked again, and its paid answer was
    dropped -- the attempt counted failed, and a second call paid for."""
    import control.mission_runner as mr
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    calls = {"n": 0}

    def counted(frame):
        calls["n"] += 1
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = counted

    def at_the_edge(fn, *args, timeout_s=None, on_start=None):
        t = threading.Thread(target=fn, args=args, daemon=True)
        t.start()
        if on_start is not None:
            on_start(t)
        t.join()        # ended just after the guard's own look
        raise TimeoutError(f"call exceeded {timeout_s}s")
    monkeypatch.setattr(mr, "call_with_timeout", at_the_edge)
    FakeClockReconfirmer(runner, lambda: True, interval_s=15, window_s=600, max_paid=2).run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed", late
    assert calls["n"] == 1 and late["paid_calls"] == 1, (calls, late)


def test_g1_an_error_landing_during_a_probe_is_not_a_second_attempt():
    """The fixes' fourth review (both reviewers): the earlier call's error,
    landing while the probe ran, reached the paid branch as LateCallFailed and
    was counted again -- "2 paid attempts failed" after one, and the second
    attempt max_paid allows never sent."""
    from brain.navigate import CloudUnavailable
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    release = threading.Event()
    calls = {"n": 0}

    def call(frame):
        calls["n"] += 1
        if calls["n"] == 1:
            release.wait(5)
            raise CloudUnavailable("HTTP 503: throttled")
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = call

    def at_the_retry(n):
        if n == 21:     # the half-window retry is due at this probe
            release.set()
            runner._confirm_thread.join(2)
    rc = PacedReconfirmer(runner, lambda: True, interval_s=15, window_s=600, max_paid=2,
                          on_probe=at_the_retry)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 2, late
    assert calls["n"] == 2, calls


def test_g1_an_error_landing_at_the_last_retry_still_gets_the_second_attempt():
    """The fixes' fifth review (/code-review): the landed error was swallowed
    and the loop waited an interval -- past the deadline when the retry had
    been clamped to its last interval -- so the second attempt never went."""
    from brain.navigate import CloudUnavailable
    cloud = SwitchableCloud()
    runner, _, _ = _ended_unconfirmed(cloud, CLEAR_STARTS[0])
    runner.vision_timeout_s = 0.05
    release = threading.Event()
    calls = {"n": 0}

    def call(frame):
        calls["n"] += 1
        if calls["n"] == 1:
            release.wait(5)
            raise CloudUnavailable("HTTP 503: throttled")
        return _quiet_cloud(frame)
    runner.vision_fn.cloud_vision_fn = call

    def at_the_last_probe(n):
        if n == 40:     # t=585: the retry clamped to deadline - interval
            release.set()
            runner._confirm_thread.join(2)
    rc = PacedReconfirmer(runner, lambda: rc.n >= 30, interval_s=15, window_s=600,
                          max_paid=2, on_probe=at_the_last_probe)
    rc.run()
    late = runner.status()["late_confirmation"]
    assert late["state"] == "confirmed" and late["paid_calls"] == 2, late


# ---------- G4: the cloud watch's age on the wall clock ----------


def test_g4_a_wall_clock_step_does_not_age_a_fresh_answer():
    """Red on dev: NTP setting the Jetson's clock forward three hours made a
    one-second-old answer read three hours old."""
    wall = {"t": 1000.0}
    mono = {"t": 50.0}
    watch = CloudWatch(lambda: True, interval_s=15, active=lambda: False,
                       wall=lambda: wall["t"], mono=lambda: mono["t"])
    watch.record(True)
    wall["t"] += 3 * 3600
    mono["t"] += 1.0
    assert watch.snapshot()["age_s"] == 1.0
