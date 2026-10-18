"""3.81.E: an ended mission frees its perception models, so the next
mission's build never holds two pipelines at once (3.81 C2: 377 MB free at a
mission start with VE resident, floor 500)."""
import gc
import time
import threading
import types
import weakref

from brain import perceive
from brain.perceive import DETECTED, UNAVAILABLE, Perception
from brain.tiered import TieredVision
import pytest


@pytest.fixture(autouse=True)
def _no_abandoned_frames(monkeypatch):
    monkeypatch.setattr(perceive, "_abandoned", set())


class Pipe:
    def __init__(self):
        self.entered, self.go = threading.Event(), threading.Event()
        self.block = False

    def perceive(self, frame):
        if self.block:
            self.entered.set()
            self.go.wait(5)
        return Perception(status=DETECTED)


def test_closing_the_policy_frees_its_pipeline():
    pipe = Pipe()
    ref = weakref.ref(pipe)
    tier = TieredVision(pipe, lambda frame: {})
    del pipe
    tier.close()
    gc.collect()
    assert ref() is None, "the ended mission still holds its models"


def test_a_frame_after_close_is_unavailable_and_does_not_raise():
    tier = TieredVision(Pipe(), lambda frame: {})
    tier.close()
    assert tier.pipeline.perceive({"image_base64": "x"}).status == UNAVAILABLE
    tier({"image_base64": "x"})


def test_a_frame_inside_perception_at_close_returns_its_own_result():
    """Through the policy's own call: the frame finishes on the pipeline it
    started with, and the closed policy sends nothing to the cloud."""
    pipe = Pipe()
    pipe.block = True
    calls = []
    tier = TieredVision(pipe, lambda frame: calls.append(frame) or {})
    out = {}
    t = threading.Thread(target=lambda: out.update(r=tier({"image_base64": "x"})))
    t.start()
    assert pipe.entered.wait(5)
    tier.close()
    pipe.go.set()
    t.join(5)
    assert "r" in out
    assert tier.stats.perception == {DETECTED: 1}
    assert calls == []


def test_pipeline_for_frees_earlier_pipelines_before_it_loads(monkeypatch):
    order = []
    cuda = types.SimpleNamespace(is_available=lambda: True,
                                 empty_cache=lambda: order.append("empty_cache"))
    monkeypatch.setitem(__import__("sys").modules, "torch", types.SimpleNamespace(cuda=cuda))
    monkeypatch.setattr(perceive.gc, "collect", lambda: order.append("collect"))
    monkeypatch.setattr(perceive, "detector_for",
                        lambda *a, **k: order.append("detector") or object())
    monkeypatch.setattr(perceive, "ClipScorer",
                        lambda *a, **k: order.append("clip") or object())
    monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: k)
    perceive.pipeline_for("red backpack")
    assert order[:2] == ["collect", "empty_cache"]
    assert set(order[2:]) == {"detector", "clip"}


def test_pipeline_for_without_torch_loaded_only_collects(monkeypatch):
    order = []
    monkeypatch.delitem(__import__("sys").modules, "torch", raising=False)
    monkeypatch.setattr(perceive.gc, "collect", lambda: order.append("collect"))
    monkeypatch.setattr(perceive, "detector_for", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "ClipScorer", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: k)
    perceive.pipeline_for("red backpack")
    assert order == ["collect"]


# -- 3.81.E review: a start never builds beside a pipeline still in use --

class BlockingDetector:
    def __init__(self):
        self.entered, self.go = threading.Event(), threading.Event()

    def detect(self, image, confidence):
        self.entered.set()
        self.go.wait(5)
        return []


class NoScorer:
    def score(self, image, box, texts):
        return [0.0] * len(texts)


def test_pipeline_for_waits_for_a_frame_still_in_perception(monkeypatch):
    """A Stop cancels the loop, but its tick thread can still be inside the
    ended mission's perception; the next build waits for it."""
    import base64
    det = BlockingDetector()
    old = perceive.PerceptionPipeline(det, NoScorer(), "red backpack")
    frame = {"image_base64": base64.b64encode(b"x").decode(), "image_width": 640}
    t = threading.Thread(target=old.perceive, args=(frame,))
    t.start()
    assert det.entered.wait(5)
    built = threading.Event()
    monkeypatch.setattr(perceive, "detector_for", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "ClipScorer", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: built.set())
    b = threading.Thread(target=perceive.pipeline_for, args=("blue bottle",))
    b.start()
    assert not built.wait(0.3), "built beside a pipeline still perceiving"
    det.go.set()
    assert built.wait(5)
    t.join(5)
    b.join(5)


def test_pipeline_for_does_not_wait_forever_on_a_hung_frame(monkeypatch):
    monkeypatch.setattr(perceive, "PERCEIVE_WAIT_S", 0.2)
    det = BlockingDetector()
    old = perceive.PerceptionPipeline(det, NoScorer(), "red backpack")
    t = threading.Thread(target=old.perceive, args=({"image_base64": "eA=="},))
    t.start()
    assert det.entered.wait(5)
    monkeypatch.setattr(perceive, "detector_for", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "ClipScorer", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: "new")
    assert perceive.pipeline_for("blue bottle") == "new"
    det.go.set()
    t.join(5)


def test_a_start_waits_for_the_ended_mission_to_finish_closing(tmp_path):
    """`is_running()` is false before `_finish` has closed the policy (the
    stop's HTTP call comes first), so a start must wait for `finished`."""
    from fastapi.testclient import TestClient
    from control.brain_server import create_app
    from control.mission_runner import MissionRunner
    from tests.conftest import fresh_mock_robot

    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_interval_s: 0.0\n")
    robot = fresh_mock_robot()
    runners, built_at = [], []

    def factory(robot, req):
        built_at.append(time.monotonic())
        runners.append(MissionRunner(robot, target_object=req.target_object, max_steps=2))
        return runners[-1]

    app = create_app(config_path=str(config), robot_factory=lambda: robot,
                     runner_factory=factory)
    with TestClient(app) as client:
        assert client.post("/mission/start", json={"target_object": "red backpack"}).status_code == 200
        assert runners[0].finished.wait(10)
        runners[0].finished.clear()            # as if `_finish` were still closing
        released = []
        timer = threading.Timer(0.5, lambda: (released.append(time.monotonic()),
                                              runners[0].finished.set()))
        timer.start()
        assert client.post("/mission/start", json={"target_object": "blue bottle"}).status_code == 200
        timer.join()
        assert built_at[1] >= released[0], "built before the ended mission had closed"
        client.post("/mission/stop")


def _waited(call):
    """Did `call` wait out PERCEIVE_WAIT_S on a frame? A wait that times out
    gives the frames up (`_abandoned` grows) and takes the whole bound; one
    that needs no wait does neither. Judged by that state and, for a wait,
    a floor -- never a ceiling, which timed gc.collect() and CUDA's
    empty_cache() too (0.35-0.5 s in the whole suite's process: every
    fast-tier run failed on dev, 3.76, 2026-10-18)."""
    before, t0 = len(perceive._abandoned), time.monotonic()
    call()
    gave_up = len(perceive._abandoned) > before
    if gave_up:
        assert time.monotonic() - t0 >= 0.25, "gave up before the bound"
    return gave_up


def test_a_hung_frame_costs_one_wait_not_one_per_start(monkeypatch):
    monkeypatch.setattr(perceive, "PERCEIVE_WAIT_S", 0.3)
    det = BlockingDetector()
    old = perceive.PerceptionPipeline(det, NoScorer(), "red backpack")
    t = threading.Thread(target=old.perceive, args=({"image_base64": "eA=="},))
    t.start()
    try:
        assert det.entered.wait(5)
        monkeypatch.setattr(perceive, "detector_for", lambda *a, **k: object())
        monkeypatch.setattr(perceive, "ClipScorer", lambda *a, **k: object())
        monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: "new")
        assert _waited(lambda: perceive.pipeline_for("a"))      # waits once, gives up
        assert not _waited(lambda: perceive.pipeline_for("b")), "waited again on the same hung frame"
    finally:
        det.go.set()
        t.join(5)


def test_a_stop_during_the_wait_for_the_close_never_builds(tmp_path):
    from fastapi.testclient import TestClient
    from control.brain_server import create_app
    from control.mission_runner import MissionRunner
    from tests.conftest import fresh_mock_robot

    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  tick_interval_s: 0.0\n")
    robot = fresh_mock_robot()
    runners = []

    def factory(robot, req):
        runners.append(MissionRunner(robot, target_object=req.target_object, max_steps=2))
        return runners[-1]

    app = create_app(config_path=str(config), robot_factory=lambda: robot,
                     runner_factory=factory)
    with TestClient(app) as client:
        assert client.post("/mission/start", json={"target_object": "red backpack"}).status_code == 200
        assert runners[0].finished.wait(10)
        runners[0].finished.clear()
        stop = threading.Timer(0.3, lambda: client.post("/mission/stop"))
        release = threading.Timer(0.8, runners[0].finished.set)
        stop.start(); release.start()
        r = client.post("/mission/start", json={"target_object": "blue bottle"})
        stop.join(); release.join()
        assert r.status_code == 409
        assert len(runners) == 1, "built a mission the operator had stopped"


def test_a_hung_frame_that_returns_does_not_excuse_the_next_one(monkeypatch):
    monkeypatch.setattr(perceive, "PERCEIVE_WAIT_S", 0.3)
    monkeypatch.setattr(perceive, "detector_for", lambda *a, **k: object())
    monkeypatch.setattr(perceive, "ClipScorer", lambda *a, **k: object())
    real = perceive.PerceptionPipeline
    monkeypatch.setattr(perceive, "PerceptionPipeline", lambda **k: "new")
    hung, later = BlockingDetector(), BlockingDetector()
    a = threading.Thread(target=real(hung, NoScorer(), "x").perceive, args=({"image_base64": "eA=="},))
    b = threading.Thread(target=real(later, NoScorer(), "x").perceive, args=({"image_base64": "eA=="},))
    a.start()
    try:
        assert hung.entered.wait(5)
        perceive.pipeline_for("a")              # gives up on the hung frame
        hung.go.set()
        a.join(5)                               # ... which then returns
        b.start()
        assert later.entered.wait(5)
        assert _waited(lambda: perceive.pipeline_for("b")), "skipped the wait for a live frame"
    finally:
        hung.go.set()
        later.go.set()
        a.join(5)
        if b.is_alive():
            b.join(5)
