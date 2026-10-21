"""3.90 E: the teleop body keeps its last pushed frames for a recorder
(criteria 12 and 14), while decisions still take only the newest."""

from sim.teleop_robot import FRAME_BUFFER, TeleopRobot


def _push(robot, n):
    for i in range(n):
        robot.push_frame(f"img{i}")


def test_frames_after_returns_every_newer_frame_oldest_first():
    robot = TeleopRobot()
    _push(robot, 5)
    out = robot.frames_after(2)
    assert [f["metadata"]["seq"] for f in out["frames"]] == [3, 4, 5]
    assert out["latest_seq"] == 5 and out["lost"] == 0
    assert all("captured_at" in f for f in out["frames"])


def test_decisions_still_take_only_the_newest():
    robot = TeleopRobot()
    _push(robot, 5)
    assert robot.get_camera_frame()["metadata"]["seq"] == 5


def test_a_recorder_too_far_behind_is_told_how_many_it_lost():
    robot = TeleopRobot()
    _push(robot, FRAME_BUFFER + 4)
    out = robot.frames_after(0)
    assert len(out["frames"]) == FRAME_BUFFER
    assert out["frames"][0]["metadata"]["seq"] == 5
    assert out["lost"] == 4


def test_nothing_new_is_nothing_lost():
    robot = TeleopRobot()
    _push(robot, 3)
    out = robot.frames_after(3)
    assert out["frames"] == [] and out["lost"] == 0
    empty = TeleopRobot().frames_after(0)
    assert {k: v for k, v in empty.items() if k != "boot_id"} == {"frames": [], "latest_seq": 0, "lost": 0, "reset": False}


def test_the_route_serves_them_and_refuses_a_body_without_them(tmp_path):
    from tests.test_server import make_client, make_teleop_client

    config = tmp_path / "robot.yaml"
    config.write_text("mode: teleop\n")
    with make_teleop_client(str(config)) as client:
        for i in range(3):
            client.post("/teleop/frame", json={"image_base64": f"img{i}"})
        r = client.get("/teleop/frames", params={"after": 1})
        assert r.status_code == 200
        assert [f["metadata"]["seq"] for f in r.json()["frames"]] == [2, 3]
    with make_client() as client:
        assert client.get("/teleop/frames").status_code == 400


def test_a_restarted_robot_says_so_rather_than_nothing_new():
    robot = TeleopRobot()
    _push(robot, 3)
    out = robot.frames_after(500)              # a recorder from before a restart
    assert out["reset"] is True
    assert [f["metadata"]["seq"] for f in out["frames"]] == [1, 2, 3]
    assert out["lost"] == 0


def test_a_negative_after_is_not_a_loss():
    robot = TeleopRobot()
    _push(robot, 3)
    out = robot.frames_after(-5)
    assert out["lost"] == 0 and out["reset"] is False and len(out["frames"]) == 3


def test_walk_eval_scores_only_the_perceived_rows_of_a_stream_walk():
    from control.walk_eval import compute_metrics, decided_entries

    rows = [{"seq": 0, "navigate": {"action": "FORWARD"}, "perceived": True},
            {"seq": 1, "navigate": None, "perceived": False},
            {"seq": 2, "navigate": None, "perceived": False},
            {"seq": 3, "navigate": {"action": "FORWARD"}, "perceived": True}]
    assert compute_metrics(decided_entries(rows))["frames"] == 2
    old_walk = [{"seq": 0, "navigate": {"action": "LEFT"}}]
    assert decided_entries(old_walk) == old_walk
