"""3.90 A: robot/camera_stream.py -- newest wins, never a stale frame, reopens
by itself (criteria 1-3 in docs/plans/ros-alignment/3.90-camera-stream.md)."""

import base64
import threading
import time

import pytest

from robot.camera_stream import CameraStale, CameraStream


class FakeSource:
    """Delivers numbered JPEG-looking bytes at `fps`, until told to stop."""

    def __init__(self, fps=30.0, fail_reads=False):
        self.period = 1.0 / fps
        self.n = 0
        self.closed = False
        self.fail_reads = fail_reads

    def read(self):
        if self.fail_reads:
            raise OSError("unplugged")
        time.sleep(self.period)
        self.n += 1
        return b"\xff\xd8frame-%d" % self.n

    def close(self):
        self.closed = True


def _wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def _get(stream):
    try:
        return stream.get_camera_frame()
    except CameraStale:
        return None


def test_serves_the_newest_frame_with_its_capture_stamp():
    source = FakeSource()
    stream = CameraStream(lambda: source)
    try:
        assert _wait(lambda: _get(stream) is not None)
        time.sleep(0.2)
        frame = stream.get_camera_frame()
        n = int(base64.b64decode(frame["image_base64"]).split(b"-")[1])
        assert n >= source.n - 1                 # newest, never a backlog
        assert frame["media_type"] == "image/jpeg"
        assert time.time() - frame["captured_at"] < 0.1
        assert frame["metadata"]["seq"] >= n
    finally:
        stream.close()


def test_no_frame_yet_raises_rather_than_answering():
    stream = CameraStream(lambda: FakeSource(fps=0.5))
    try:
        with pytest.raises(CameraStale):
            stream.get_camera_frame()
    finally:
        stream.close()


def test_a_source_that_stops_is_stale_within_one_second():
    gate = threading.Event()
    gate.set()

    class Stoppable(FakeSource):
        def read(self):
            if not gate.is_set():
                time.sleep(0.05)
                return None
            return super().read()

    stream = CameraStream(lambda: Stoppable())
    try:
        assert _wait(lambda: _get(stream) is not None)
        gate.clear()
        stopped = time.monotonic()
        assert _wait(lambda: _get(stream) is None, timeout=2.0)
        assert time.monotonic() - stopped <= 1.0
        with pytest.raises(CameraStale, match="no new frame"):
            stream.get_camera_frame()
    finally:
        stream.close()


def test_an_unplugged_camera_delivers_again_within_three_seconds_of_replug():
    plugged = threading.Event()
    plugged.set()
    opened = []

    def open_source():
        if not plugged.is_set():
            raise OSError("no such device")
        source = FakeSource()
        opened.append(source)
        return source

    stream = CameraStream(open_source)
    try:
        assert _wait(lambda: _get(stream) is not None)
        plugged.clear()
        opened[-1].fail_reads = True             # the cable comes out
        assert _wait(lambda: _get(stream) is None)
        assert opened[-1].closed
        plugged.set()
        replugged = time.monotonic()
        assert _wait(lambda: _get(stream) is not None, timeout=3.0)
        assert time.monotonic() - replugged <= 3.0
        assert stream.status()["opens"] == 2
        assert stream.status()["failures"] >= 1
    finally:
        stream.close()


def test_a_picture_is_encoded_once_and_only_when_read():
    encodes = []

    class PictureSource(FakeSource):
        def read(self):
            time.sleep(self.period)
            self.n += 1
            return ("picture", self.n)

    def encode(picture):
        encodes.append(picture)
        return b"\xff\xd8" + str(picture[1]).encode()

    stream = CameraStream(lambda: PictureSource(fps=100), encode=encode)
    try:
        assert _wait(lambda: _get(stream) is not None)
        time.sleep(0.3)                          # ~30 frames arrive, none read
        before = len(encodes)
        assert before <= 1
        stream.get_camera_frame()
        assert len(encodes) == before + 1
    finally:
        stream.close()


def test_the_same_frame_read_twice_is_encoded_once():
    encodes = []

    class OnePicture(FakeSource):
        def read(self):
            if self.n:
                time.sleep(0.01)
                return None
            self.n = 1
            return ("picture", 1)

    stream = CameraStream(lambda: OnePicture(), encode=lambda p: encodes.append(p) or b"\xff\xd8")
    try:
        assert _wait(lambda: _get(stream) is not None)
        stream.get_camera_frame()
        assert len(encodes) == 1
    finally:
        stream.close()


def test_close_stops_the_reader_and_closes_the_source():
    source = FakeSource()
    stream = CameraStream(lambda: source)
    assert _wait(lambda: _get(stream) is not None)
    stream.close()
    assert not stream._thread.is_alive()
    assert source.closed


def test_the_factory_builds_the_fake_camera_under_sim_camera_fake(monkeypatch):
    from robot.factory import _camera

    monkeypatch.setenv("SIM_CAMERA", "fake")
    monkeypatch.setenv("SIM_CAMERA_FPS", "30")
    stream = _camera({})
    try:
        assert _wait(lambda: _get(stream) is not None)
        time.sleep(0.5)
        frame = stream.get_camera_frame()
        assert base64.b64decode(frame["image_base64"])[:2] == b"\xff\xd8"
        assert 10 <= stream.status()["frames"] <= 25     # ~15 in 0.5 s at 30 fps, after open
    finally:
        stream.close()


def test_no_camera_configured_is_no_driver(monkeypatch):
    from robot.factory import _camera

    monkeypatch.delenv("SIM_CAMERA", raising=False)
    monkeypatch.delenv("ROBOT_CAMERA", raising=False)
    assert _camera({}) is None
    assert _camera({"hardware": {}}) is None
