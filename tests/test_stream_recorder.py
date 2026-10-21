"""3.90 E criterion 14: the brain records every frame the robot received,
marked perceived or not, and counts what it could not record."""

import base64
import json
import threading

from control.stream_recorder import StreamRecorder
from control.walk_store import LocalWalkStore
from sim.teleop_robot import FRAME_BUFFER, TeleopRobot

JPEG = base64.b64encode(b"\xff\xd8not-really-a-jpeg").decode()


class FakeRunner:
    """Decides on the frames listed in `decided`; running until stopped."""

    def __init__(self, decided=()):
        self.decided = set(decided)
        self.last_frame_seq = max(decided) if decided else None
        self.running = threading.Event()
        self.running.set()

    def is_running(self):
        return self.running.is_set()

    def decision_for(self, seq, boot_id=None):
        if seq not in self.decided:
            return None
        return {"action": "FORWARD", "_missionStatus": {"last_frame_seq": seq}}

    def decided_past(self, seq, boot_id=None):
        return self.last_frame_seq is not None and self.last_frame_seq >= seq


def _rows(store, walk):
    text = (store.root / walk / "walk.jsonl").read_text()
    return [json.loads(line) for line in text.splitlines()]


def _record(tmp_path, robot, runner, after=0, **kw):
    store = LocalWalkStore(tmp_path)
    rec = StreamRecorder(store, "w1", robot.frames_after, runner, poll_s=0.01,
                         meta={"target_object": "backpack"}, **kw).start(after=after)
    return store, rec


def test_every_received_frame_is_filed_and_marked(tmp_path):
    robot = TeleopRobot()
    for _ in range(6):
        robot.push_frame(JPEG)
    runner = FakeRunner(decided={2, 5})
    store, rec = _record(tmp_path, robot, runner)
    runner.running.clear()
    assert rec.finished.wait(5)
    rows = _rows(store, "w1")
    assert [r["teleop_seq"] for r in rows] == [1, 2, 3, 4, 5, 6]
    assert [r["seq"] for r in rows] == [0, 1, 2, 3, 4, 5]          # no duplicate seqs
    assert [r["perceived"] for r in rows] == [False, True, False, False, True, False]
    assert rows[1]["navigate"]["_missionStatus"]["last_frame_seq"] == 2
    assert rows[0]["navigate"] is None
    meta = json.loads((store.root / "w1" / "meta.json").read_text())
    assert meta["source"] == "stream" and meta["frames"] == 6
    assert meta["frames_perceived"] == 2 and meta["frames_lost"] == 0
    assert meta["target_object"] == "backpack"


def test_frames_before_the_mission_are_not_filed(tmp_path):
    robot = TeleopRobot()
    for _ in range(3):
        robot.push_frame(JPEG)
    runner = FakeRunner()
    store, rec = _record(tmp_path, robot, runner, after=3)
    robot.push_frame(JPEG)
    runner.running.clear()
    assert rec.finished.wait(5)
    assert [r["teleop_seq"] for r in _rows(store, "w1")] == [4]


def test_a_recorder_too_far_behind_counts_what_it_lost(tmp_path):
    robot = TeleopRobot()
    runner = FakeRunner()
    store = LocalWalkStore(tmp_path)
    rec = StreamRecorder(store, "w1", robot.frames_after, runner, poll_s=0.01)
    for _ in range(FRAME_BUFFER + 5):
        robot.push_frame(JPEG)
    runner.running.clear()
    rec.start(after=0)
    assert rec.finished.wait(5)
    meta = json.loads((store.root / "w1" / "meta.json").read_text())
    assert meta["frames_lost"] == 5 and meta["frames"] == FRAME_BUFFER


def test_a_frame_waits_for_the_mission_to_decide_past_it(tmp_path):
    robot = TeleopRobot()
    robot.push_frame(JPEG)
    runner = FakeRunner()                         # nothing decided yet
    store, rec = _record(tmp_path, robot, runner, settle_s=30)
    import time
    time.sleep(0.2)
    assert rec.saved == 0                         # held: the tick may yet take it
    runner.decided.add(1)
    runner.last_frame_seq = 1
    assert _wait(lambda: rec.saved == 1)
    runner.running.clear()
    assert rec.finished.wait(5)
    assert _rows(store, "w1")[0]["perceived"] is True


def test_the_cap_is_counted_not_silent(tmp_path):
    robot = TeleopRobot()
    for _ in range(5):
        robot.push_frame(JPEG)
    runner = FakeRunner()
    store, rec = _record(tmp_path, robot, runner, max_frames=3)
    runner.running.clear()
    assert rec.finished.wait(5)
    meta = json.loads((store.root / "w1" / "meta.json").read_text())
    assert meta["frames"] == 3 and meta["frames_over_cap"] == 2


def test_a_restarted_robot_starts_the_cursor_over(tmp_path):
    first, second = TeleopRobot(), TeleopRobot()
    for _ in range(5):
        first.push_frame(JPEG)
    body = {"robot": first}
    runner = FakeRunner()
    store = LocalWalkStore(tmp_path)
    rec = StreamRecorder(store, "w1", lambda after: body["robot"].frames_after(after),
                         runner, poll_s=0.01, settle_s=0.05).start(after=0)
    assert _wait(lambda: rec.saved == 5)
    body["robot"] = second                        # the robot server restarted...
    for _ in range(7):
        second.push_frame(JPEG)                   # ...and numbered past nothing yet
    assert _wait(lambda: rec.saved == 12)
    runner.running.clear()
    assert rec.finished.wait(5)
    assert [r["teleop_seq"] for r in _rows(store, "w1")] == [1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 6, 7]


def _wait(cond, timeout=3.0):
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


# ---------- through the brain's /mission/start ----------

def _brain(tmp_path, robot):
    from fastapi.testclient import TestClient

    from control.brain_server import create_app
    return TestClient(create_app(robot_factory=lambda: robot, store=LocalWalkStore(tmp_path)))


def test_record_walk_on_a_body_without_frames_is_refused(tmp_path):
    from tests.test_brain_server import RecordingRobot, fresh_mock_robot

    with _brain(tmp_path, RecordingRobot(fresh_mock_robot())) as client:
        r = client.post("/mission/start", json={"policy": "frontier", "target_object": "backpack",
                                                "record_walk": "w1"})
        assert r.status_code == 400
        assert "teleop" in r.json()["detail"]


def test_bad_names_and_votes_are_refused(tmp_path):
    robot = TeleopRobot()
    robot.push_frame(JPEG)
    with _brain(tmp_path, robot) as client:
        for body in ({"record_walk": "../x"}, {"record_walk": "metrics-x"},
                     {"frame_vote": "2of3"}, {"frame_vote": "9of9"}):
            r = client.post("/mission/start", json={"policy": "frontier",
                                                    "target_object": "backpack", **body})
            assert r.status_code in (400, 422), body


def test_a_teleop_mission_records_its_stream_and_reports_it(tmp_path):
    robot = TeleopRobot()
    robot.push_frame(JPEG)
    with _brain(tmp_path, robot) as client:
        r = client.post("/mission/start", json={"policy": "frontier", "target_object": "backpack",
                                                "record_walk": "w1"})
        assert r.status_code == 200, r.text
        for _ in range(4):
            robot.push_frame(JPEG)
        assert _wait(lambda: (client.get("/mission/status").json().get("recording") or {})
                     .get("saved", 0) >= 4, timeout=10)
        client.post("/mission/stop")
        assert _wait(lambda: (client.get("/mission/status").json()["recording"] or {})
                     .get("finished"), timeout=10)
    rows = _rows(LocalWalkStore(tmp_path), "w1")
    # From the newest frame at start (the one the opening tick takes) on.
    assert [r["teleop_seq"] for r in rows][:5] == [1, 2, 3, 4, 5]
    meta = json.loads((tmp_path / "w1" / "meta.json").read_text())
    assert meta["source"] == "stream" and meta["policy"] == "frontier"


def test_a_decision_slower_than_a_poll_still_lands_on_its_frame(tmp_path):
    # Review: a vision tick's cloud call takes 2-3 s; its frame must wait for
    # the decision, not be filed unperceived after a short settle.
    robot = TeleopRobot()
    runner = FakeRunner()
    store, rec = _record(tmp_path, robot, runner)
    robot.push_frame(JPEG)
    robot.push_frame(JPEG)
    import time
    time.sleep(2.5)                               # the tick is still thinking (old settle: 2 s)
    assert rec.saved == 0
    runner.decided.add(1)
    runner.last_frame_seq = 2                     # decided on 1; 2 never taken
    assert _wait(lambda: rec.saved == 2)
    runner.running.clear()
    assert rec.finished.wait(5)
    assert [r["perceived"] for r in _rows(store, "w1")] == [True, False]


def test_a_restarted_body_never_borrows_the_old_lifes_decisions():
    from control.mission_runner import MissionRunner

    class R:
        frame = {"metadata": {"seq": 7, "boot_id": "old"}}
        action = "LEFT"

    runner = MissionRunner.__new__(MissionRunner)
    import collections
    import threading as th
    runner._decisions, runner._decisions_lock = collections.OrderedDict(), th.Lock()
    runner._last_decided, runner._perception, runner._tier = None, None, None
    runner._last_reasoning = ""
    runner._note_decision(R())
    assert runner.decision_for(7, "old") is not None
    assert runner.decision_for(7, "new") is None
    assert runner.decided_past(3, "old") and not runner.decided_past(3, "new")


def test_a_stop_during_the_starts_frame_probe_is_not_lost(tmp_path, monkeypatch):
    # Review round 2: the probe after the mission was built ran outside the
    # build slot, so a Stop in it was lost and the mission started and drove.
    from control import mission_runner

    built = threading.Event()
    real_init = mission_runner.MissionRunner.__init__

    def init(self, *a, **kw):
        real_init(self, *a, **kw)
        built.set()

    monkeypatch.setattr(mission_runner.MissionRunner, "__init__", init)
    robot = TeleopRobot()
    robot.push_frame(JPEG)
    probing, release = threading.Event(), threading.Event()
    real_latest = robot.latest

    def slow_latest():
        if built.is_set():            # the probe once the mission exists
            probing.set()
            release.wait(5)
        return real_latest()

    robot.latest = slow_latest
    with _brain(tmp_path, robot) as client:
        out = {}
        t = threading.Thread(target=lambda: out.update(r=client.post(
            "/mission/start", json={"policy": "frontier", "target_object": "backpack",
                                    "record_walk": "w1"})))
        t.start()
        assert probing.wait(5)
        client.post("/mission/stop")
        release.set()
        t.join(10)
        assert out["r"].status_code == 409, out["r"].text
        assert client.get("/mission/status").json()["running"] is False
