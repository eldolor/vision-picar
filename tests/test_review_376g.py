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
    CLEAR_STARTS, FakeClockReconfirmer, SwitchableCloud, _ended_unconfirmed, _quiet_cloud)


# ---------- G1: a late call that outlives its guard ended the whole window ----------


class PacedReconfirmer(FakeClockReconfirmer):
    """A fake clock that also lets a little real time pass per probe, so a
    call on another thread can land inside the window. `on_probe(n)` runs
    before probe n (1-based)."""

    def __init__(self, *a, on_probe=None, **k):
        super().__init__(*a, **k)
        self.on_probe = on_probe or (lambda n: None)
        self.n = 0
        inner = self.probe

        def probe():
            self.n += 1
            self.on_probe(self.n)
            return inner()
        self.probe = probe

    def _wait(self, seconds):
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
    assert rc.n < 12, rc.n


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
    assert rc.n > 3, "it asked before the earlier call ended"


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
