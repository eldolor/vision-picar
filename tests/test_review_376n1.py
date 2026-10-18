"""
tests/test_review_376n1.py

`PLAN-ros-alignment.md` 3.76, step N1: the bugs the review of the body and
world contracts, the factories, the brain's config and control's helpers
(`robot/interface.py`, `robot/factory.py`, `robot/identity.py`,
`world/interface.py`, `world/factory.py`, `world/bridge_faults.py`,
`config/robot.yaml`, `control/brain_config.py`, `control/health.py`,
`control/drills.py`, `control/identity_eval.py`, `control/target_probe.py`,
whole files) confirmed. Each test was written before its fix and seen to
fail on `dev` for the reason it names.
"""

import time

import pytest

from robot.interface import WheelFeedbackLost, carry_out_verb

R = 0.04


class WallClockBody:
    """A body whose wheels turn in real time at the standing command and
    whose feedback is sampled at `feedback_hz`, as the car's board reports
    (20 Hz). Thermos's stand-in, 3.76 N1."""

    stop_count = 0

    def stop(self):
        self.stop_count += 1
        self.set_wheel_velocity(0.0, 0.0)

    def __init__(self, feedback_hz):
        self.pos, self.w, self.t = 0.0, 0.0, time.monotonic()
        self.fb_period, self.fb_at, self.fb = 1 / feedback_hz, -1.0, 0.0
        self.fail_after = None

    def _integ(self):
        now = time.monotonic()
        self.pos += self.w * (now - self.t)
        self.t = now

    def set_wheel_velocity(self, left, right):
        self._integ()
        self.w = (left + right) / 2

    def get_wheel_state(self):
        self._integ()
        now = time.monotonic()
        if now - self.fb_at >= self.fb_period:
            self.fb_at, self.fb = now, self.pos
        if self.fail_after is not None:
            self.fail_after -= 1
            if self.fail_after < 0:
                return {"usable": False}
        return {"usable": True, "wheel_radius_m": R, "track_width_m": 0.172,
                "left": {"position_rad": self.fb}, "right": {"position_rad": self.fb}}

    def true_m(self):
        self._integ()
        return self.pos * R


# ---------- N1.1 (not a bug; a guard): a guarded verb on the wall clock ----------


def test_a_guarded_verb_on_the_wall_clock_stops_at_an_aged_line():
    """The claim: on the car a shortened last step leaves the wheels running
    while the loop waits on 20 Hz feedback, so a FORWARD ends ~3 cm past its
    line. Measured, the overshoot comes only from a STALE clearance; the
    verb's limit is `forward_clearance()`, which `SafetyController._aged`
    corrects by the reading's age at the measured speed -- and with that, the
    verb stops at the line."""
    body, line = WallClockBody(20), 0.30

    def limit(amount):
        seen = body.get_wheel_state()["left"]["position_rad"] * R   # 20 Hz feedback...
        return max(0.0, line - seen - 0.6 * body.fb_period), "line"  # ...aged, as _aged does

    w = 0.6 / R
    out = carry_out_verb(body, {"kind": "straight", "left_rad_s": w, "right_rad_s": w,
                                "target": 1.0, "wall_clock": True}, limit)
    assert out["ended"] == "clamped", out
    assert body.true_m() - line < 0.005, f"{100 * (body.true_m() - line):.1f} cm past the line"


# ---------- N1.2: the verb's last read could replace its outcome ----------


def test_feedback_lost_on_the_last_read_keeps_the_verbs_outcome():
    """The closing `achieved()` sat outside the try: feedback lost between
    the loop's end and that read turned a stopped verb into
    `WheelFeedbackLost` -- a mission that a Stop ended read `failed`."""
    body = WallClockBody(50)
    w = 0.6 / R
    n = {"reads": 0}
    real = body.get_wheel_state

    def reads():
        n["reads"] += 1
        if n["reads"] == 3:
            body.stop_count += 1                         # a Stop lands mid-verb
            body.fail_after = 1                          # and the read after this fails
        return real()

    body.get_wheel_state = reads
    out = carry_out_verb(body, {"kind": "straight", "left_rad_s": w, "right_rad_s": w,
                                "target": 1.0, "wall_clock": True})
    assert out["ended"] == "stopped", out


def test_a_short_end_with_its_last_read_lost_is_no_feedback():
    """Fixes' round 1 (Thermos): kept for every end, a clamped verb whose last
    read was lost came back clamped at 0 m -- `_guarded` read it as a safety
    block, which the brain steers around, instead of a body that cannot be
    measured."""
    body = WallClockBody(50)
    w = 0.6 / R

    def limit(amount):
        body.fail_after = 0                              # the very next read fails
        return 0.0, "line"

    out = carry_out_verb(body, {"kind": "straight", "left_rad_s": w, "right_rad_s": w,
                                "target": 1.0, "wall_clock": True}, limit)
    assert out["ended"] == "no_feedback", out


# ---------- N1.3: the tick-hang drill shared the runner's tick count ----------


def test_the_tick_hang_drill_centres_the_camera_and_counts_ticks_once():
    from control.drills import TickHangRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    robot = MockRobot(build_world("scaled_house"), render=False)
    robot.look_right()
    runner = TickHangRunner(robot, target_object="red backpack", hang_s=0.0)
    runner.start()
    runner.tick()
    log = list(robot.world.log)
    assert "LOOK_CENTER" in log[log.index("LOOK_RIGHT"):], "the drill's first tick did not centre the camera"
    assert runner.status()["ticks"] == 1, runner.status()["ticks"]


# ---------- N1.4: deadlines of 0 ----------


@pytest.mark.parametrize("key, value", [
    ("vision_timeout_s", 0), ("tick_timeout_s", 0), ("request_timeout_s", 0),
    # Fixes' round 1 (Thermos): the drills' own, and a NaN confirmation.
    ("drill_vision_timeout_s", 0), ("drill_tick_timeout_s", 0),
    ("arrival_confirm_timeout_s", ".nan")])
def test_a_deadline_of_zero_is_refused(tmp_path, key, value):
    """0 made every cloud call time out, or aborted every mission on its
    first tick as a hung loop -- and clamped the confirmation to 0 too."""
    from control.brain_config import load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(f"brain:\n  {key}: {value}\n")
    with pytest.raises(ValueError, match=key):
        load_brain_config(str(path))


# ---------- N1.5: health read a tick within its deadline as a dead loop ----------


def test_a_tick_inside_its_deadline_is_not_a_loop_not_turning(monkeypatch):
    import control.health as health
    monkeypatch.setattr(health, "_get", lambda *a, **k: {
        "mission_running": True, "seconds_since_last_tick": 30.1, "tick_timeout_s": 30.0})
    report = health.check_brain("http://brain")
    assert not any("not turning" in p for p in report["problems"]), report


# ---------- N1.6: the world's bridge did not follow the drive's ----------


def test_the_ros_world_uses_the_drives_bridge_url(tmp_path, monkeypatch):
    from world.factory import get_world
    monkeypatch.delenv("ROS_BRIDGE_URL", raising=False)
    path = tmp_path / "robot.yaml"
    path.write_text("world:\n  mode: ros\ndrive:\n  bridge_url: http://127.0.0.1:18990\n")
    world = get_world(str(path))
    assert str(world._http.base_url).rstrip("/") == "http://127.0.0.1:18990"


# ---------- N1.7: an empty env var stopped the robot server starting ----------


def test_an_empty_track_scrub_is_unset(monkeypatch):
    from robot.factory import _track_scrub
    monkeypatch.setenv("TRACK_SCRUB", "")
    assert _track_scrub({}) == 1.0
