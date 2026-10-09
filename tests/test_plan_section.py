"""tools/plan_section.py -- the parallel-sessions tooling
(docs/guides/PARALLEL-SESSIONS.md)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import plan_section as ps  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_every_section_has_a_file_and_the_index_is_fresh():
    secs = ps.sections(ROOT)
    nums = [n for n, _, _ in secs]
    assert len(nums) == len(set(nums)), "duplicate section numbers"
    index = open(os.path.join(ROOT, ps.INDEX)).read()
    for n, _, f in secs:
        assert f"[{n}]({f})" in index, f"{f} missing from the index: run plan_section.py index"


def test_the_main_plan_no_longer_holds_sections():
    text = open(os.path.join(ROOT, ps.PLAN)).read()
    assert not any(ps.HEAD_RE.match(l) for l in text.split("\n"))


def test_resources_never_collide_between_sections():
    seen = {}
    for minor in range(1, 100):
        r = ps.resources(f"3.{minor}")
        for k in ("PICAR_ROBOT_PORT", "PICAR_BRIDGE_PORT", "ROS_DOMAIN_ID", "PICAR_ROS_CONTAINER"):
            key = (k, r[k])
            assert key not in seen, f"3.{minor} and {seen[key]} share {k}"
            seen[key] = f"3.{minor}"
        assert r["ROS_IMAGE"] != "vision-picar-ros:latest"
        assert r["ROS_DOMAIN_ID"] not in (0, 73, 74) and r["ROS_DOMAIN_ID"] <= 101
        assert r["PICAR_ROBOT_PORT"] >= 9000


def test_slugs_are_file_safe():
    assert ps.slugify("The stop distance follows `speed` (2026-10-07): less room") \
        == "the-stop-distance-follows-speed-less"


def test_the_push_gate_skips_the_suite_only_for_docs():
    """A docs-only push runs the spec lint; anything that touches code,
    tests or config runs the fast tier (user, 2026-10-09)."""
    assert ps.docs_only(["docs/guides/INTRODUCTION.md", "CLAUDE.md"])
    assert ps.docs_only(["docs-review/REPORT.md"])
    assert not ps.docs_only([])
    assert not ps.docs_only(["docs/guides/INTRODUCTION.md", "robot/safety.py"])
    assert not ps.docs_only(["config/robot.yaml"])
    assert not ps.docs_only(["tests/test_ui.py"])


def test_the_fast_tier_leaves_out_only_the_slow_files():
    """The browser, live-stack and sweep files go to the nightly run; the
    quick safety, contract and failsafe files must stay in the tier, and
    every file named as slow must exist (a rename would otherwise quietly
    put it back in the push gate's tier)."""
    import fnmatch
    ignored = [a.split("=", 1)[1] for a in ps.SLOW_TESTS]
    tests = sorted(os.listdir(os.path.join(os.path.dirname(__file__))))
    left_out = {t for t in tests for g in ignored if fnmatch.fnmatch(f"tests/{t}", g)}
    assert {"test_ui.py", "test_slam_live.py", "test_arrival.py"} <= left_out
    for kept in ("test_robot_contract.py", "test_world_contract.py",
                 "test_failsafes.py", "test_depth_veto.py", "test_pivot_safety.py",
                 "test_authority.py", "test_spec_lint.py", "test_ros_containment.py"):
        assert kept not in left_out
    assert set(ps.SLOW_FILES) <= set(tests)


def test_a_clean_rebase_keeps_the_change_id(tmp_path, monkeypatch):
    """The gate keys a pass on the change, so a rebase onto another
    session's push does not rerun the tier; a different change does."""
    import subprocess as sp

    # The pre-push hook runs this suite with git's GIT_DIR (and friends) set.
    # Inherited, every `git` below -- config, commit, checkout -- acts on the
    # REAL repository, not tmp_path: on 2026-10-09 it set the repo's identity
    # to "t", moved a plan branch and created `main`/`mine` there.
    for k in [k for k in os.environ if k.startswith("GIT_")]:
        monkeypatch.delenv(k)

    def g(*a):
        return sp.run(["git", *a], cwd=tmp_path, check=True, capture_output=True,
                      text=True).stdout.strip()

    g("init", "-q", "-b", "main")
    g("config", "user.email", "t@example.com")
    g("config", "user.name", "t")
    (tmp_path / "a.txt").write_text("a\n")
    (tmp_path / "b.txt").write_text("b\n")
    g("add", "."); g("commit", "-qm", "base")
    base = g("rev-parse", "HEAD")
    g("checkout", "-qb", "mine")
    (tmp_path / "a.txt").write_text("a\nmine\n")
    g("commit", "-qam", "mine")
    before = ps.change_id(base, g("rev-parse", "HEAD"), str(tmp_path))
    g("checkout", "-q", "main")
    (tmp_path / "b.txt").write_text("b\ntheirs\n")
    g("commit", "-qam", "theirs")
    theirs = g("rev-parse", "HEAD")
    g("checkout", "-q", "mine"); g("rebase", "-q", "main")
    after = ps.change_id(theirs, g("rev-parse", "HEAD"), str(tmp_path))
    assert before and before == after
    (tmp_path / "a.txt").write_text("a\nmine, edited\n")
    g("commit", "-qam", "edit")
    assert ps.change_id(theirs, g("rev-parse", "HEAD"), str(tmp_path)) != before
