"""
tests/test_review_376p3.py

`PLAN-ros-alignment.md` 3.76, step P3: the bugs the review of the
remaining tools and hooks (`tools/gpu/`, `tools/trt/`, `tools/spec_lint.py`,
`tools/procthor_import.py`, `tools/steer_check.py`, `.claude/hooks/`,
`firmware/ugv_base_ros/`, whole files) confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names. No AWS,
no GPU: the scripts run against stubs on PATH.
"""

import os
import stat
import subprocess
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT
from tests.test_spec_lint import A, TODAY, codes, edit, repo  # noqa: F401  (fixture)
from tools import spec_lint

ROOT = Path(REPO_ROOT)


def _stub(bindir: Path, name: str, body: str) -> None:
    p = bindir / name
    p.write_text("#!/usr/bin/env bash\n" + body + "\n")
    p.chmod(p.stat().st_mode | stat.S_IEXEC)


# ---------- P3.1: run_matrix sent a script that could not run ----------


def test_the_matrix_script_reaches_the_box_with_its_arguments(tmp_path):
    """Escaped for two layers where `ec2.sh run` takes one: the box got a
    literal `$COMMON` and `$out`, every call failed behind `grep || true`,
    and the run printed MATRIX-DONE having scored nothing (both reviewers)."""
    sent = tmp_path / "sent.sh"
    fake_ec2 = tmp_path / "ec2.sh"
    fake_ec2.write_text(f'[ "$1" = run ] && printf %s "$2" > {sent}\n')
    script = (ROOT / "tools" / "gpu" / "run_matrix.sh").read_text().replace(
        "EC2=tools/hailo/ec2.sh", f"EC2={fake_ec2}")
    (tmp_path / "run_matrix.sh").write_text(script)
    subprocess.run(["bash", str(tmp_path / "run_matrix.sh")], check=True, cwd=tmp_path)
    remote = sent.read_text().replace(". /opt/pytorch/bin/activate", "true").replace(
        "cd /opt/eval", f"cd {tmp_path}")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    calls = tmp_path / "calls"
    _stub(bindir, "python", f'printf "%s\\n" "$*" >> {calls}')
    subprocess.run(["bash", "-c", remote], check=True, cwd=tmp_path,
                   env={**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}"})
    first = calls.read_text().splitlines()[0]
    assert "$COMMON" not in first and "--gate 0.8" in first, first
    assert "--save out/yoloe-11s-seg_none.json" in first, first


# ---------- P3.2: the pacing sweep's stub future had no exception() ----------


def test_the_pacing_stub_future_answers_exception():
    """`TieredVision._collect_inflight` calls `fut.exception()`; the stub had
    none, every landing raised and was swallowed, and every async row
    recorded zero calls (both reviewers)."""
    from tools.gpu.pacing_sweep import FrameClockExecutor
    ok = FrameClockExecutor._Future(lambda: 7, (), 0)
    assert ok.exception() is None and ok.result() == 7
    bad = FrameClockExecutor._Future(lambda: 1 / 0, (), 0)
    assert isinstance(bad.exception(), ZeroDivisionError)


# ---------- P3.3: an AWS failure read as "no instance" ----------


@pytest.mark.parametrize("script", ["tools/gpu/ec2.sh", "tools/hailo/ec2.sh"])
def test_down_with_an_expired_session_does_not_claim_no_instance(tmp_path, script):
    """`instance_id` dropped stderr and ended `|| true`: an expired session
    made `down` print "no instance to terminate" and exit 0 with a g5 still
    billing (Thermos reproduced), and deleted its role under it."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    _stub(bindir, "aws", 'echo "ExpiredToken: the security token has expired" >&2; exit 255')
    r = subprocess.run(["bash", str(ROOT / script), "down"], capture_output=True, text=True,
                       cwd=ROOT, env={**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}"})
    assert r.returncode != 0, r.stdout + r.stderr
    assert "no instance to terminate" not in r.stdout
    assert "could not list instances" in r.stderr


def test_a_failed_launch_names_the_instance_left_running():
    """`up` died after `run-instances` with "check the console" (/code-review)."""
    src = (ROOT / "tools" / "gpu" / "ec2.sh").read_text()
    up = src[src.index("cmd_up() {"):src.index("# ------------------------------------------------------------- status")]
    assert up.index("trap '") < up.index("aws_ ec2 wait instance-running")
    assert "STILL RUNNING" in up and "trap - EXIT" in up


def test_the_dead_man_switch_is_armed_before_the_corpus_step_and_said_only_if_armed():
    """It was armed after `ensure_corpus_read`, so a failure there left the
    box up with no switch, and "armed" printed even when it failed."""
    src = (ROOT / "tools" / "gpu" / "eval_host.sh").read_text()
    up = src[src.index("  up)"):src.index("  setup)")]
    assert up.index("shutdown -h +240") < up.index("ensure_corpus_read")
    assert 'if [ "$armed" = 1 ]' in up


# ---------- P3.4: steer_check scored on the wrong band ----------


def test_steer_check_scores_on_the_band_the_policy_steers_on():
    """A fixed 10 deg band counted every correct LEFT/RIGHT between 3 and
    10 deg as a disagreement under `steer_on_sight` (STEER_BAND_DEG 3)."""
    from tools.steer_check import wanted
    assert wanted(-6.0, True) == "left" and wanted(6.0, True) == "right"
    assert wanted(-6.0, False) == "center" and wanted(-12.0, False) == "left"
    assert wanted(None, True) == "unknown"


# ---------- P3.5: spec_lint ----------


def test_a_verified_datetime_does_not_crash_the_lint(repo):  # noqa: F811
    """YAML reads `2026-10-01 12:00:00` as a datetime; `check_stale` then
    subtracted it from a date and the whole lint raised (Thermos)."""
    edit(repo, A, "verified: 2026-10-01", "verified: 2026-10-01 12:00:00")
    spec_lint.lint(repo, today=TODAY)


def test_a_heading_inside_a_fence_is_not_a_section(repo):  # noqa: F811
    """A `## Purpose` inside a code block satisfied E7 (both reviewers)."""
    edit(repo, A, "## Purpose", "```\n## Purpose\n```\n\n## Not purpose")
    assert "E7" in codes(spec_lint.lint(repo, today=TODAY))


def test_a_link_with_a_title_is_still_checked(repo):  # noqa: F811
    """`[x](missing.md "title")` matched no link, so it went unchecked."""
    edit(repo, A, "(../engineering/brakes/ENGINEERING.md)", '(../engineering/brakes/MISSING.md "spec")')
    assert "E6" in codes(spec_lint.lint(repo, today=TODAY))
