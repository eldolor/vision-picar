"""3.76 batch H2: the cloud's templates and scripts. Read as text; nothing
here reaches AWS. The vision service's own fixes are tested in
service/vision_analyze/tests/test_review_376h2.py."""

import re

import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CFN = REPO_ROOT / "cloudformation"


def _code(path: Path) -> str:
    return "\n".join(ln for ln in path.read_text().splitlines()
                     if not ln.lstrip().startswith("#"))


def test_lambda_zips_outlive_a_month_between_deploys():
    """H2-7: serverless.yaml's VisionCodeKey/WalksCodeKey name zips in the
    deploy bucket, and a failed update rolls back to the previous ones. They
    expired after 30 days, so a failed deploy a month after the last build
    could not roll back (UPDATE_ROLLBACK_FAILED)."""
    text = (CFN / "deploy-bucket.yaml").read_text()
    m = re.search(r"ExpireAfterDays:\s*\n\s*Type: Number\s*\n\s*Default:\s*(\d+)", text)
    assert m, "ExpireAfterDays parameter not found"
    assert int(m.group(1)) >= 365
    # And the lambda/ rule still uses the parameter (not a literal).
    assert re.search(r"Prefix: lambda/\s*\n\s*ExpirationInDays: !Ref ExpireAfterDays", text)


def test_sync_waits_for_the_invalidation_to_complete():
    """H2-12: CLAUDE.md section 6 -- a twin change is not shipped until the
    invalidation reports Completed. The script printed InProgress and exited 0."""
    code = _code(REPO_ROOT / "service" / "static" / "sync.sh")
    assert "create-invalidation" in code
    assert "wait invalidation-completed" in code


def test_bedrock_timeouts_fit_inside_the_vision_lambda_timeout():
    """H2-11 and round 2 S1, the template half: every attempt's connect +
    read budget must stay under the function's own Timeout, or the function
    is killed first."""
    import sys
    vision_dir = str(REPO_ROOT / "service" / "vision_analyze")
    if vision_dir not in sys.path:
        sys.path.insert(0, vision_dir)
    import vision_core

    text = (CFN / "serverless.yaml").read_text()
    block = text.split("VisionFunction:", 1)[1].split("WalksFunction:", 1)[0]
    lambda_timeout = int(re.search(r"Timeout:\s*(\d+)", block).group(1))
    cfg = vision_core._BEDROCK_CONFIG
    attempts = (cfg.retries or {}).get("total_max_attempts")
    assert attempts, "retries must be bounded explicitly (botocore retries a read timeout)"
    assert attempts * (cfg.connect_timeout + cfg.read_timeout) < lambda_timeout


# ---------- round 2 ----------


def test_brain_rooms_mirror_ignores_empty_and_non_string_items():
    """S5: brain/rooms.py is the canonical copy of rooms_core and had the
    defects fixed there: "" matched every room, a non-string crashed."""
    from brain.rooms import identify_room

    assert identify_room([""]) == "unknown"
    assert identify_room("bed") == "unknown"
    assert identify_room(["Double bed", 3, "nightstand"]) == "bedroom"


def test_brain_vision_parser_mirror():
    """S5: brain/vision.py's scene parser had the TypeError on a non-object
    and lost JSON followed by prose."""
    from brain.vision import _parse_scene_json

    assert _parse_scene_json("[1, 2]")["_raw"] == "[1, 2]"
    out = _parse_scene_json('{"free_space": "clear"} That is all.')
    assert out["free_space"] == "clear" and "_raw" not in out


def _brain_client():
    from fastapi.testclient import TestClient

    from control.brain_server import create_app
    from tests.conftest import RecordingRobot, fresh_mock_robot

    return TestClient(create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot())))


@pytest.mark.parametrize("policy", ["vision", "tiered"])
@pytest.mark.parametrize("target", ["   ", "x" * 201])
def test_a_cloud_policy_refuses_a_blank_or_overlong_target_at_start(monkeypatch, policy, target):
    """S11 / round 3 P4: the vision service answers a blank target and one
    over 200 characters 400, which reaches a mission as a plain error on
    every call -- a mission that starts and then fails. Refused at start,
    with the reason, before anything moves."""
    monkeypatch.setenv("VISION_URL", "http://127.0.0.1:9")
    with _brain_client() as client:
        resp = client.post("/mission/start", json={"target_object": target, "policy": policy})
        assert resp.status_code == 400
        assert "target_object" in resp.json()["detail"]
        assert client.get("/mission/status").json()["running"] is False


def test_a_long_target_still_starts_a_frontier_mission():
    """P4: frontier/explore never call the cloud; round 2's 200-character cap
    refused a target that had always worked there."""
    with _brain_client() as client:
        resp = client.post("/mission/start", json={"target_object": "x" * 250, "policy": "frontier"})
        assert resp.status_code == 200
        client.post("/mission/stop")
