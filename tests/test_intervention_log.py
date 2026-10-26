"""
tests/test_intervention_log.py

`PLAN-ros-alignment.md` 3.99: one durable JSON line per mission end, saying
whether a person stepped in (a Stop, a D-pad takeover) and the context it
landed in, and a report of interventions per hour of autonomy.

In process over the simulator. Criterion 5 (live) is in the section file.
"""

import json
import os
import time

import pytest

from brain import intervention_log as il
from brain.intervention_log import InterventionLog, read_missions, report, summarise
from control import intervention_report
from control.mission_runner import CANCELLED, FAILED, OUT_OF_BUDGET, MissionRunner
from robot.interface import StoppedByPerson
from tests.conftest import fresh_mock_robot, fresh_mock_runner, mock_world_for
from tests.test_mission_ends import _TakenOver


def _logged(runner, tmp_path):
    runner.intervention_log = InterventionLog(tmp_path / "interventions.jsonl")
    return tmp_path / "interventions.jsonl"


def _lines(path):
    return [json.loads(x) for x in path.read_text().splitlines()]


def _wait(runner):
    assert runner.intervention_done.wait(10), "the intervention line never landed"


def _drive(runner, max_ticks=5000):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"


# ---------- criteria 1-2: one line a mission, and who ended it ----------

def test_a_brain_stop_is_one_stop_intervention_line(tmp_path):
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    path = _logged(runner, tmp_path)
    runner.mission_id, runner.house = "m1", "starter_house"
    runner.start()
    for _ in range(5):
        runner.tick()
    runner.stop()
    _wait(runner)
    [line] = _lines(path)
    assert line["mission_id"] == "m1" and line["house"] == "starter_house"
    assert line["outcome"] == CANCELLED and line["policy"] == runner.policy
    assert line["target_object"] == "red backpack"
    assert line["autonomous_s"] >= 0 and line["started_at"] <= line["ended_at"]
    iv = line["intervention"]
    assert iv["kind"] == "stop" and iv["via"] == "brain_stop"
    assert iv["step"] == runner.status()["step"]
    assert iv["pose_after_stop"] is not None and set(iv["pose_after_stop"]) >= {"x_m", "y_m", "heading_deg"}
    assert line["person_picks"] == 0


def test_a_dpad_takeover_names_the_driver(tmp_path):
    inner = fresh_mock_robot()
    runner = MissionRunner(_TakenOver(inner), target_object="red backpack",
                           world=mock_world_for(inner))
    path = _logged(runner, tmp_path)
    runner.start()
    _drive(runner)
    _wait(runner)
    [line] = _lines(path)
    iv = line["intervention"]
    assert (iv["kind"], iv["via"], iv["by"]) == ("dpad", "preempted", "twin-dpad")
    assert "twin-dpad is driving" in iv["detail"]


class _Latched(_TakenOver):
    message = "twin-dpad pressed Stop -- autonomy stays stopped until a new mission starts"

    def _taken(self, *a, **k):
        raise StoppedByPerson(self.message)

    drive_forward = reverse = turn_left = turn_right = _taken
    look_left = look_right = set_wheel_velocity = verb_plan = _taken


def test_a_person_s_latched_stop_is_a_robot_stop(tmp_path):
    inner = fresh_mock_robot()
    runner = MissionRunner(_Latched(inner), target_object="red backpack",
                           world=mock_world_for(inner))
    path = _logged(runner, tmp_path)
    runner.start()
    _drive(runner)
    _wait(runner)
    [line] = _lines(path)
    assert line["outcome"] == CANCELLED
    assert (line["intervention"]["kind"], line["intervention"]["via"]) == ("stop", "robot_stop")


def test_an_autonomous_preemption_is_no_intervention(tmp_path):
    inner = fresh_mock_robot()
    robot = _TakenOver(inner)
    robot.message = ("ros is driving -- one autonomous driver at a time, so brain was "
                     "refused until it lapses")
    runner = MissionRunner(robot, target_object="red backpack", world=mock_world_for(inner))
    path = _logged(runner, tmp_path)
    runner.start()
    _drive(runner)
    _wait(runner)
    [line] = _lines(path)
    assert line["outcome"] == FAILED and line["intervention"] is None


def test_an_abort_and_a_time_limit_are_no_intervention(tmp_path):
    a = fresh_mock_runner(target_object="red backpack", max_steps=150)
    path = _logged(a, tmp_path)
    a.start()
    a.tick()
    a.abort("brain loop hung: tick exceeded 1s")
    _wait(a)
    t = {"now": 0.0}
    b = fresh_mock_runner(target_object="purple elephant", mission_time_limit_s=3.0,
                          clock=lambda: t["now"])
    b.intervention_log = InterventionLog(path)
    b.start()
    while b.tick():
        t["now"] += 1.0
    _wait(b)
    first, second = _lines(path)
    assert (first["outcome"], first["intervention"]) == (FAILED, None)
    assert (second["outcome"], second["intervention"]) == (OUT_OF_BUDGET, None)
    assert second["autonomous_s"] == pytest.approx(3.0, abs=1.0)


def test_a_second_end_writes_nothing_and_an_unstarted_mission_nothing(tmp_path):
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    path = _logged(runner, tmp_path)
    runner.start()
    runner.tick()
    runner.stop()
    runner.stop("again")
    runner.abort("and again")
    _wait(runner)
    assert len(_lines(path)) == 1
    never = fresh_mock_runner(target_object="red backpack")
    never.intervention_log = InterventionLog(tmp_path / "never.jsonl")
    never.stop()
    assert never.intervention_done.wait(5)
    assert not (tmp_path / "never.jsonl").exists()


def test_no_log_configured_writes_nothing_and_still_signals(tmp_path):
    runner = fresh_mock_runner(target_object="red backpack")
    runner.start()
    runner.stop()
    _wait(runner)


# ---------- criterion 3: a log never costs the mission ----------

class _PoseFails:
    def __init__(self, inner):
        self.inner = inner

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def get_pose(self):
        raise RuntimeError("bridge down")


def test_a_failed_pose_read_still_writes_the_line(tmp_path):
    inner = fresh_mock_robot()
    runner = MissionRunner(inner, target_object="red backpack",
                           world=_PoseFails(mock_world_for(inner)), max_steps=150)
    path = _logged(runner, tmp_path)
    runner.start()
    runner.stop()
    _wait(runner)
    [line] = _lines(path)
    assert line["intervention"]["pose_after_stop"] is None


def test_an_unwritable_log_never_changes_the_end_or_skips_the_stop(tmp_path, monkeypatch):
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    path = _logged(runner, tmp_path)

    def boom(fd):
        raise OSError("disk full")

    monkeypatch.setattr("brain.choice_log._fsync", boom)
    stops = []
    real_stop = runner.robot.stop
    runner.robot.stop = lambda: (stops.append(1), real_stop())[1]
    runner.start()
    runner.tick()
    runner.stop()
    _wait(runner)
    s = runner.status()
    assert s["outcome"] == CANCELLED and s["cancelled_by"] == "stop"
    assert stops


def test_the_stop_does_not_wait_on_the_pose_read(tmp_path):
    inner = fresh_mock_robot()

    class Slow(_PoseFails):
        armed = False

        def get_pose(self):
            if Slow.armed:
                time.sleep(1.5)
            return self.inner.get_pose()

    runner = MissionRunner(inner, target_object="red backpack",
                           world=Slow(mock_world_for(inner)), max_steps=150)
    _logged(runner, tmp_path)
    runner.start()
    runner.tick()
    Slow.armed = True
    t0 = time.monotonic()
    runner.stop()
    assert time.monotonic() - t0 < 0.5
    _wait(runner)


# ---------- criterion 4: the report ----------

FIXTURE = [
    {"mission_id": "a", "policy": "explore", "outcome": "cancelled", "autonomous_s": 1800.0,
     "intervention": {"kind": "dpad"}, "person_picks": 2},
    {"mission_id": "b", "policy": "explore", "outcome": "found", "autonomous_s": 1800.0,
     "intervention": None, "person_picks": 0},
    {"mission_id": "c", "policy": "tiered", "outcome": "cancelled", "autonomous_s": 900.0,
     "intervention": {"kind": "stop"}, "person_picks": 0},
    {"mission_id": "d", "policy": "tiered", "outcome": "exhausted", "autonomous_s": 2700.0,
     "intervention": None, "person_picks": 0},
]


def test_the_report_counts_interventions_per_hour(tmp_path):
    s = summarise(FIXTURE)
    assert s["all"] == {"missions": 4, "hours": 2.0, "interventions": 2, "stop": 1,
                        "dpad": 1, "per_hour": 1.0, "person_picks": 2}
    assert s["by_policy"]["explore"]["per_hour"] == 1.0
    assert s["by_policy"]["tiered"] == {"missions": 2, "hours": 1.0, "interventions": 1,
                                        "stop": 1, "dpad": 0, "per_hour": 1.0,
                                        "person_picks": 0}


def test_the_report_skips_a_cut_line_and_reads_no_file_as_none(tmp_path):
    path = tmp_path / "interventions.jsonl"
    path.write_text("\n".join(json.dumps(x) for x in FIXTURE) + '\n{"mission_id": "e", "pol')
    assert len(read_missions(path)) == 4
    assert "4 missions" in report(tmp_path)
    assert report(tmp_path / "nowhere") == "no missions recorded"
    assert summarise([])["all"]["per_hour"] is None


def test_the_report_runs_as_a_module(tmp_path, capsys):
    (tmp_path / "interventions.jsonl").write_text(
        "\n".join(json.dumps(x) for x in FIXTURE) + "\n")
    intervention_report.main([str(tmp_path)])
    out = capsys.readouterr().out
    assert "1.00 per hour" in out and "explore" in out and "tiered" in out


# ---------- the brain server wires it ----------

def test_the_brain_config_names_the_directory(monkeypatch, tmp_path):
    from control.brain_config import load_brain_config
    monkeypatch.delenv("INTERVENTION_DIR", raising=False)   # the suite's own (conftest)
    assert load_brain_config()["intervention_dir"] == "recordings/interventions"
    monkeypatch.setenv("INTERVENTION_DIR", str(tmp_path))
    assert load_brain_config()["intervention_dir"] == str(tmp_path)


def test_the_brain_server_writes_a_line_for_a_stopped_mission(monkeypatch, tmp_path):
    """Built as the brain server builds it: a mission started over HTTP and
    stopped by the brain's Stop leaves one `stop` line in INTERVENTION_DIR."""
    from fastapi.testclient import TestClient
    from control import brain_server
    from tests.conftest import fresh_mock_robot, mock_world_for

    monkeypatch.setenv("INTERVENTION_DIR", str(tmp_path))
    robot = fresh_mock_robot()
    app = brain_server.create_app(robot_factory=lambda: robot,
                                  world_factory=lambda _r: mock_world_for(robot))
    with TestClient(app) as client:
        r = client.post("/mission/start", json={"target_object": "purple elephant"})
        assert r.status_code == 200, r.text
        assert client.post("/mission/stop").status_code == 200
        runner = app.state.brain_state["runner"]
        _wait(runner)
    [line] = _lines(tmp_path / "interventions.jsonl")
    assert line["mission_id"] == runner.mission_id
    assert line["intervention"]["kind"] == "stop"
    assert line["intervention"]["via"] == "brain_stop"


# ---------- review round 1 ----------

def test_a_loop_cancelled_without_a_person_s_stop_is_no_intervention(tmp_path):
    """Both reviewers: the loop's cancellation handler calls `stop()`, and a
    cancel that no person's Stop asked for (the process shutting down) was
    logged as a person's `stop`."""
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    path = _logged(runner, tmp_path)
    runner.start()
    runner.tick()
    runner.loop_cancelled = True        # what the brain server sets
    runner.stop("mission loop cancelled")
    _wait(runner)
    [line] = _lines(path)
    assert line["outcome"] == CANCELLED and line["intervention"] is None
    assert line["end_via"] == il.VIA_LOOP_CANCELLED


def test_the_brain_server_s_shutdown_cancel_is_no_intervention(monkeypatch, tmp_path):
    import asyncio
    from control import brain_server

    monkeypatch.setenv("INTERVENTION_DIR", str(tmp_path))
    robot = fresh_mock_robot()
    app = brain_server.create_app(robot_factory=lambda: robot,
                                  world_factory=lambda _r: mock_world_for(robot))
    from fastapi.testclient import TestClient
    with TestClient(app) as client:
        assert client.post("/mission/start",
                           json={"target_object": "purple elephant"}).status_code == 200
        runner = app.state.brain_state["runner"]
        task = app.state.brain_state["task"]
        # What the event loop's teardown does: cancel the task, no route.
        client.portal.call(lambda: task.cancel())
        _wait(runner)
    [line] = _lines(tmp_path / "interventions.jsonl")
    assert line["intervention"] is None


def test_the_writer_is_not_a_daemon_thread(tmp_path):
    """Both reviewers: a daemon writer dies with the process, so a restart
    straight after a Stop lost the line. Python joins a non-daemon thread
    at exit."""
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    _logged(runner, tmp_path)
    runner.start()
    runner.stop()
    _wait(runner)
    assert runner._intervention_writer is not None
    assert not runner._intervention_writer.daemon


def test_a_hung_pose_read_holds_the_writer_only_its_own_deadline(tmp_path, monkeypatch):
    """Round 2, both reviewers: the writer is not a daemon, so a hung world
    held the process's exit for the client's whole timeout."""
    import threading
    from control import mission_runner as mr
    monkeypatch.setattr(mr, "POSE_READ_S", 0.3)
    inner = fresh_mock_robot()
    release = threading.Event()

    class Hung(_PoseFails):
        armed = False

        def get_pose(self):
            if Hung.armed:
                release.wait(10)
            return self.inner.get_pose()

    runner = MissionRunner(inner, target_object="red backpack",
                           world=Hung(mock_world_for(inner)), max_steps=150)
    path = _logged(runner, tmp_path)
    runner.start()
    runner.tick()
    Hung.armed = True
    try:
        t0 = time.monotonic()
        runner.stop()
        _wait(runner)
        runner._intervention_writer.join(5)
        assert not runner._intervention_writer.is_alive()
        assert time.monotonic() - t0 < 2.0
    finally:
        release.set()       # the abandoned reader, never left hanging in the suite
    iv = _lines(path)[0]["intervention"]
    assert iv["pose_after_stop"] is None and iv["pose_note"].startswith("read timed out")


def test_the_brain_server_shares_one_log_between_missions(monkeypatch, tmp_path):
    """Round 2: one log built with the app, so every mission's writer holds
    the same lock (round 1's registry of per-file locks, removed)."""
    from fastapi.testclient import TestClient
    from control import brain_server

    monkeypatch.setenv("INTERVENTION_DIR", str(tmp_path))
    robot = fresh_mock_robot()
    app = brain_server.create_app(robot_factory=lambda: robot,
                                  world_factory=lambda _r: mock_world_for(robot))
    logs = []
    with TestClient(app) as client:
        for _ in range(2):
            assert client.post("/mission/start",
                               json={"target_object": "purple elephant"}).status_code == 200
            assert client.post("/mission/stop").status_code == 200
            runner = app.state.brain_state["runner"]
            _wait(runner)
            logs.append(runner.intervention_log)
    assert logs[0] is logs[1]
    assert len(_lines(tmp_path / "interventions.jsonl")) == 2


def test_the_pose_says_why_it_is_missing(tmp_path):
    inner = fresh_mock_robot()

    class Unusable(_PoseFails):
        def get_pose(self):
            return {"usable": False, "reason": "no map yet"}

    for world, why in ((Unusable(mock_world_for(inner)), "unusable"),
                       (_PoseFails(mock_world_for(inner)), "read failed")):
        runner = MissionRunner(inner, target_object="red backpack", world=world,
                               max_steps=150)
        runner.intervention_log = InterventionLog(tmp_path / f"{why}.jsonl")
        runner.start()
        runner.stop()
        _wait(runner)
        iv = _lines(tmp_path / f"{why}.jsonl")[0]["intervention"]
        assert iv["pose_after_stop"] is None and iv["pose_note"].startswith(why)


def test_the_report_reads_the_brain_config_s_directory_by_default(tmp_path, monkeypatch,
                                                                 capsys):
    (tmp_path / "interventions.jsonl").write_text(
        "\n".join(json.dumps(x) for x in FIXTURE) + "\n")
    monkeypatch.setenv("INTERVENTION_DIR", str(tmp_path))
    intervention_report.main([])
    assert "4 missions" in capsys.readouterr().out
