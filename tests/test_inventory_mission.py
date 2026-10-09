"""
tests/test_inventory_mission.py -- the object inventory on the mission path
(PLAN-ros-alignment.md 3.46 criteria 1, 3-5 pinned, and D).

* Criterion 1: the search is identical with the inventory on and off --
  same actions, same outcome -- for the frontier and tiered policies in two
  houses. The inventory is recorded and reported only.
* Criteria 3-5: sweep 3's numbers (`tests/inventory_sweep.py --seed 3462`)
  pinned on a subset, so a regression fails the suite.
* D: the pose and scan are read with the frame; a frame without
  detections costs nothing; nothing in the inventory can fail a mission;
  the brain serves the report and hands it to the store once.
"""
import json
import logging
import math

import pytest
from fastapi.testclient import TestClient

from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.brain_server import create_app
from control.inventory_store import InventoryStore, inventory_store_from_config
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import inventory_sweep as sweep
from tests.conftest import mock_world_for

STEPS = 40


def _quiet_cloud(frame):
    return {"obstacles_ahead": [], "free_space": "unknown", "doorway_visible": False,
            "important_objects": [], "safest_direction": "RIGHT",
            "_navigate": {"target_visible": False}}


def _mission(house, policy, inventory, start):
    grid = build_world(house)
    (x, y), theta = start
    grid.x, grid.y, grid.theta = x, y, theta
    robot = MockRobot(grid, render=False)
    kwargs = {}
    if policy == "tiered":
        kwargs["vision_fn"] = TieredVision(FrameReportedPipeline("unicorn"), _quiet_cloud,
                                           steer_on_sight=True)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=STEPS, policy=policy,
                           world=mock_world_for(robot), inventory=inventory, **kwargs)
    runner.start()
    while runner.tick():
        pass
    return runner, grid


def _start(house):
    grid = build_world(house)
    return (grid.x, grid.y), grid.theta


@pytest.mark.parametrize("house", ["scaled_house", "home_first_floor", "complex_house"])
@pytest.mark.parametrize("policy", ["frontier", "tiered"])
def test_criterion_1_the_search_is_identical_with_and_without_it(house, policy):
    on, grid_on = _mission(house, policy, True, _start(house))
    off, grid_off = _mission(house, policy, False, _start(house))
    assert [a.action for a in on.memory.actions] == [a.action for a in off.memory.actions]
    assert on.status()["outcome"] == off.status()["outcome"]
    assert (round(grid_on.x, 6), round(grid_on.y, 6)) == (round(grid_off.x, 6),
                                                          round(grid_off.y, 6))
    assert on.inventory.frames > 0 and off.inventory is None


def test_no_policy_is_handed_the_inventory():
    """Recorded and reported only: the agent never holds a reference."""
    runner, _ = _mission("complex_house", "frontier", True, _start("complex_house"))
    held = [v for v in vars(runner.agent).values() if v is runner.inventory]
    assert not held


def test_criteria_3_to_5_hold_on_a_pinned_subset():
    """Sweep 3's fresh missions 1-4 (seed 3462, amendment 2's judged set),
    as the sweep runs them. The full sweep is
    `python -m tests.inventory_sweep --seed 3462`."""
    starts = sweep.starts(seed=3462)
    logging.disable(logging.WARNING)
    try:
        runs = [sweep.run_one(starts[k], seed=3462 * 100 + k) for k in range(4)]
    finally:
        logging.disable(logging.NOTSET)
    s = sweep.score(runs)
    assert s["recall"] >= 0.90 and s["recall_small"] >= 0.80
    assert s["placement_median_m"] <= 0.20 and s["placement_p90_m"] <= 0.50
    assert s["duplicates_per_instance"] <= 1.2
    assert s["precision_reported"] >= 0.95
    assert s["precision_reported"] - s["precision_all"] >= 0.05


class _CountingWorld:
    def __init__(self, inner):
        self.inner, self.poses = inner, 0

    def get_pose(self):
        self.poses += 1
        return self.inner.get_pose()

    def __getattr__(self, name):
        return getattr(self.inner, name)


def test_a_frame_without_detections_costs_no_reads():
    """A real camera frame carries no detections until 3.46 C: the hook
    must not spend a pose and a scan read on it."""
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    world = _CountingWorld(mock_world_for(robot))
    runner = MissionRunner(robot, target_object="unicorn", max_steps=3, policy="frontier",
                           world=world, detections_fn=lambda frame: None)
    before = world.poses
    runner._observe({"image_base64": "x"})
    assert world.poses == before and runner.inventory.frames == 0


def test_the_pose_is_read_with_the_frame_not_after_the_move():
    """The hook runs inside the agent's frame capture, so the pose an
    observation carries is the capture pose -- not the one after the step's
    move, which is what `MissionMemory`'s sighting pose is."""
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    world = mock_world_for(robot)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=3, policy="frontier",
                           world=world)
    at_capture = world.get_pose()
    frame = runner._gated.get_camera_frame()          # what the agent calls
    assert frame.get("detections"), "the start pose must see something"
    assert runner.inventory.frames == 1
    grid.x += 2.0                                      # the step's move
    lm = runner.inventory.landmarks[0]
    assert lm._hit_pose == at_capture


def test_an_inventory_failure_never_fails_the_mission():
    def broken(frame):
        raise RuntimeError("detector exploded")
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=5, policy="frontier",
                           world=mock_world_for(robot), detections_fn=broken)
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["outcome"] == "max_steps"


def test_the_sink_gets_the_report_once_and_cannot_fail_the_mission():
    got = []
    runner, _ = _mission("scaled_house", "frontier", True, _start("scaled_house"))
    # A fresh runner, so the sink is in place before the mission ends.
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=5, policy="frontier",
                           world=mock_world_for(robot))
    runner.inventory_sink = got.append
    runner.start()
    while runner.tick():
        pass
    runner.stop()
    assert len(got) == 1 and got[0]["outcome"] == "max_steps"
    assert {"reported", "candidates", "counts", "map_id"} <= set(got[0])
    tail = runner.status()["log_tail"]
    assert tail[-2].startswith("inventory:")
    # The end line stays last: the twin and the failsafe tests read the end
    # reason off log_tail[-1] (the first merge gate caught this).
    assert tail[-1].startswith("mission ended")

    def boom(report):
        raise RuntimeError("s3 down")
    robot = MockRobot(build_world("scaled_house"), render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=5, policy="frontier",
                           world=mock_world_for(robot))
    runner.inventory_sink = boom
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["outcome"] == "max_steps"


def _detecting_runner(max_steps=5):
    """A mission whose every frame carries a detection, so `_observe` runs
    on each tick."""
    robot = MockRobot(build_world("scaled_house"), render=False)
    return MissionRunner(robot, target_object="unicorn", max_steps=max_steps,
                         policy="frontier", world=mock_world_for(robot),
                         detections_fn=lambda f: [{"label": "mug", "bearing_deg": 0.0}])


def test_the_inventory_is_changed_only_under_the_runner_lock():
    """3.47, found by review: `_observe` runs on the tick thread while
    status(), inventory_report() and _finish read under `_lock`. Writing
    outside it let a reader see a landmark with no points yet
    (ZeroDivisionError in summary()). Red without the `with self._lock`."""
    runner = _detecting_runner()
    calls, real = [], runner.inventory.observe

    def checked(*a, **kw):
        calls.append(runner._lock._is_owned())
        return real(*a, **kw)
    runner.inventory.observe = checked
    runner.start()
    while runner.tick():
        pass
    assert calls and all(calls)


def test_a_frame_after_the_mission_ended_is_dropped():
    """3.47: the report is saved when the mission ends; a frame from a tick
    still in flight must not change it afterwards."""
    runner = _detecting_runner()
    saved = []
    runner.inventory_sink = saved.append
    runner.start()
    while runner.tick():
        pass
    frames = runner.inventory.frames
    runner._observe({"room": "hall"})
    assert runner.inventory.frames == frames
    assert runner.inventory_report()["counts"] == saved[0]["counts"]


def test_a_failing_summary_still_stops_the_robot():
    """3.47, found by review: summary() ran on the way to `_safe_stop()`,
    so an exception there skipped the stop, the policy's close and the end
    line. Red without the try around it."""
    runner = _detecting_runner()
    stops = []
    real_stop = runner.robot.stop
    runner.robot.stop = lambda: (stops.append(1), real_stop())[1]

    def broken():
        raise ZeroDivisionError("a landmark with no points")
    runner.inventory.summary = broken
    runner.start()
    while runner.tick():
        pass
    assert stops
    assert runner.status()["log_tail"][-1].startswith("mission ended")


def test_status_carries_counts_not_the_list():
    runner, _ = _mission("complex_house", "frontier", True, _start("complex_house"))
    counts = runner.status()["inventory"]
    assert set(counts) == {"frames", "observations", "unplaced", "no_pose",
                           "landmarks", "reported"}


# ---------- the store ----------

class _FakeS3:
    def __init__(self, fail=False):
        self.puts, self.fail = [], fail

    def put_object(self, **kw):
        if self.fail:
            raise RuntimeError("AccessDenied")
        self.puts.append(kw)


def test_the_store_writes_locally_and_to_the_private_bucket(tmp_path):
    s3 = _FakeS3()
    store = InventoryStore(str(tmp_path), bucket="b", prefix="inventory", robot="Jetson 1",
                           client=s3)
    out = store.save("20261008T010203Z-red backpack", {"reported": []})
    assert out["errors"] == []
    assert json.loads((tmp_path / "20261008t010203z-red-backpack.json").read_text()) == {"reported": []}
    (put,) = s3.puts
    assert put["Bucket"] == "b"
    assert put["Key"] == "inventory/jetson-1/20261008t010203z-red-backpack.json"
    assert put["ServerSideEncryption"] == "AES256"
    assert out["s3"] == "s3://b/" + put["Key"]


def test_no_bucket_means_local_only(tmp_path):
    out = InventoryStore(str(tmp_path), client=_FakeS3()).save("m", {})
    assert out["s3"] is None and out["local"]


def test_an_upload_failure_is_reported_not_raised(tmp_path):
    out = InventoryStore(str(tmp_path), bucket="b", client=_FakeS3(fail=True)).save("m", {})
    assert out["local"] and out["s3"] is None and "AccessDenied" in out["errors"][0]


def test_a_save_cut_off_midway_leaves_the_last_good_file(tmp_path, monkeypatch):
    """3.47, found by review: the save runs on a daemon thread, so a brain
    exiting mid-write left a truncated JSON file. Written to a temporary
    name and renamed, a cut-off write never touches the real one. Red with
    a direct write_text()."""
    from pathlib import Path
    store = InventoryStore(str(tmp_path), client=_FakeS3())
    store.save("m", {"reported": [1]})
    real = Path.write_text

    def cut_off(self, data, *a, **kw):
        real(self, data[: len(data) // 2], *a, **kw)
        raise OSError("process exited mid-write")
    monkeypatch.setattr(Path, "write_text", cut_off)
    out = store.save("m", {"reported": [2]})
    assert out["local"] is None and out["errors"]
    assert json.loads((tmp_path / "m.json").read_text()) == {"reported": [1]}


def _bucket_from_run_sh(tmp_path, env_bucket):
    """Run run.sh's own INVENTORY_BUCKET line, with a fake `aws` that
    reports a bucket."""
    import os
    import re
    import subprocess
    from pathlib import Path
    text = Path("service/tunnel/run.sh").read_text()
    line = re.search(r'^export INVENTORY_BUCKET=.*?\)\}"$', text, re.M | re.S).group(0)
    fake = tmp_path / "aws"
    fake.write_text("#!/bin/sh\necho found-bucket\n")
    fake.chmod(0o755)
    env = {"PATH": f"{tmp_path}:{os.environ['PATH']}"}
    if env_bucket is not None:
        env["INVENTORY_BUCKET"] = env_bucket
    return subprocess.run(["bash", "-c", line + '\nprintf "%s" "$INVENTORY_BUCKET"'],
                          env=env, capture_output=True, text=True, check=True).stdout


def test_run_sh_set_but_empty_bucket_means_local_only(tmp_path):
    """3.47, found by review: `${VAR:-lookup}` treats an empty value as
    unset, so `INVENTORY_BUCKET=` still uploaded. Red with `:-`."""
    assert _bucket_from_run_sh(tmp_path, "") == ""
    assert _bucket_from_run_sh(tmp_path, None) == "found-bucket"
    assert _bucket_from_run_sh(tmp_path, "mine") == "mine"


def test_the_store_reads_its_config(tmp_path):
    store = inventory_store_from_config({"inventory_dir": str(tmp_path),
                                         "inventory_bucket": "b", "inventory_prefix": "inv/"})
    assert (store.bucket, store.prefix, str(store.local_dir)) == ("b", "inv", str(tmp_path))


def test_the_bucket_comes_from_the_environment(monkeypatch):
    from control.brain_config import load_brain_config
    monkeypatch.setenv("INVENTORY_BUCKET", "from-env")
    assert load_brain_config()["inventory_bucket"] == "from-env"


# ---------- the route ----------

def test_the_brain_serves_the_inventory(monkeypatch, tmp_path):
    from tests.conftest import RecordingRobot, fresh_mock_robot
    monkeypatch.setenv("INVENTORY_BUCKET", "")
    app = create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        assert client.get("/mission/inventory").json() == {"inventory": None}
        client.post("/mission/start", json={"target_object": "unicorn", "max_steps": 3})
        body = client.get("/mission/inventory").json()["inventory"]
        assert {"reported", "candidates", "counts", "mission", "outcome"} <= set(body)
        client.post("/mission/stop")


def test_two_missions_in_one_second_get_their_own_file(monkeypatch):
    """3.47, found by review: a mission id to the second plus the target
    gave a quick restart the same file name and S3 key, so the second save
    overwrote the first. Red with the old `%H%M%SZ` id."""
    import time
    from control import brain_server
    from tests.conftest import RecordingRobot, fresh_mock_robot
    ids = []

    class Store:
        def save(self, mission_id, report):
            ids.append(mission_id)
    monkeypatch.setattr(brain_server, "inventory_store_from_config", lambda config: Store())
    app = create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        for _ in range(2):
            client.post("/mission/start", json={"target_object": "unicorn", "max_steps": 50})
            client.post("/mission/stop")
        deadline = time.monotonic() + 5
        while len(ids) < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
    assert len(ids) == 2 and ids[0] != ids[1]
