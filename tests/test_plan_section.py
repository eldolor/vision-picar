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
    for k in set(os.environ) - set(ps.scrubbed_env()):
        monkeypatch.delenv(k)      # the gate's own scrub: repo-local GIT_* only

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


def test_a_scratch_repo_never_touches_the_repo_a_hook_points_at(tmp_path, monkeypatch):
    """2026-10-09: the pre-push hook exports GIT_DIR, the gate ran the suite,
    and test_a_clean_rebase_keeps_the_change_id's scratch `git init` re-
    initialised the real repository (a "t" identity, a "base" commit on the
    branch being pushed, a stray branch). With GIT_DIR pointing at a
    sentinel repo, a git child given scrubbed_env() must leave it alone.
    Red with the environment passed through as it is."""
    import subprocess as sp
    sentinel = tmp_path / "sentinel"
    scratch = tmp_path / "scratch"
    scratch.mkdir()
    sp.run(["git", "init", "-q", "-b", "work", str(sentinel)], check=True)
    monkeypatch.setenv("GIT_DIR", str(sentinel / ".git"))
    monkeypatch.setenv("GIT_CONFIG_PARAMETERS", "'core.bare'='true'")
    env = ps.scrubbed_env()
    assert "GIT_DIR" not in env and "GIT_CONFIG_PARAMETERS" not in env
    sp.run(["git", "init", "-q", "-b", "main"], cwd=scratch, check=True, env=env)
    sp.run(["git", "config", "user.name", "t"], cwd=scratch, check=True, env=env)
    sp.run(["git", "checkout", "-qb", "mine"], cwd=scratch, check=True, env=env)
    monkeypatch.delenv("GIT_DIR")
    monkeypatch.delenv("GIT_CONFIG_PARAMETERS")

    def sentinel_git(*a):
        return sp.run(["git", "-C", str(sentinel), *a], capture_output=True,
                      text=True).stdout.strip()
    assert sentinel_git("config", "--local", "--get", "user.name") == ""
    assert sentinel_git("for-each-ref", "--format=%(refname:short)", "refs/heads") == ""
    assert sentinel_git("symbolic-ref", "HEAD") == "refs/heads/work"
    assert (scratch / ".git").is_dir()


def test_the_scrub_keeps_the_environment_s_own_git_config():
    """The cloud git proxy injects config via GIT_CONFIG_COUNT/KEY_n/VALUE_n
    and needs GIT_SSL_CAINFO; scrubbing them would break every git call."""
    drop = set(ps.repo_local_git_vars())
    assert "GIT_DIR" in drop and "GIT_WORK_TREE" in drop and "GIT_INDEX_FILE" in drop
    assert "GIT_CONFIG_COUNT" not in drop and "GIT_SSL_CAINFO" not in drop


def _scratch_repo(tmp_path, monkeypatch):
    """A throwaway repo, with no GIT_* variable able to point git anywhere
    else (2026-10-09), and the gate's records kept in tmp_path too."""
    import subprocess as sp
    for k in set(os.environ) - set(ps.scrubbed_env()):
        monkeypatch.delenv(k)      # the gate's own scrub: repo-local GIT_* only
    repo = tmp_path / "repo"
    repo.mkdir()
    records = tmp_path / "gate"
    records.mkdir()
    monkeypatch.setattr(ps, "gate_dir", lambda: str(records))
    monkeypatch.chdir(repo)

    def g(*a):
        return sp.run(["git", *a], cwd=repo, check=True, capture_output=True,
                      text=True).stdout.strip()
    g("init", "-q", "-b", "main")
    g("config", "user.email", "t@example.com")
    g("config", "user.name", "t")
    (repo / "README.md").write_text("r\n")
    g("add", "."); g("commit", "-qm", "base")
    return repo, g


def test_an_unreviewed_code_commit_is_warned_about_and_a_record_clears_it(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "tool.py").write_text("x = 1\n")
    g("add", "."); g("commit", "-qm", "a tool")
    head = g("rev-parse", "HEAD")
    assert [w for w in ps.review_warnings(base, head, str(repo)) if "no /code-review" in w]
    ps.cmd_reviewed([f"{base}..{head}"])
    assert ps.review_warnings(base, head, str(repo)) == []


def test_a_review_record_survives_a_rebase(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    g("checkout", "-qb", "mine")
    (repo / "tool.py").write_text("x = 1\n")
    g("add", "."); g("commit", "-qm", "a tool")
    ps.cmd_reviewed([f"{base}..HEAD"])
    g("checkout", "-q", "main")
    (repo / "other.py").write_text("y = 2\n")
    g("add", "."); g("commit", "-qm", "theirs")
    theirs = g("rev-parse", "HEAD")
    g("checkout", "-q", "mine"); g("rebase", "-q", "main")
    assert ps.review_warnings(theirs, g("rev-parse", "HEAD"), str(repo)) == []


def test_a_risky_path_asks_for_the_thermos_pass_too(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "robot").mkdir()
    (repo / "robot" / "safety.py").write_text("STOP = 20\n")
    g("add", "."); g("commit", "-qm", "safety")
    head = g("rev-parse", "HEAD")
    ps.cmd_reviewed([f"{base}..{head}"])
    assert [w for w in ps.review_warnings(base, head, str(repo)) if "Thermos" in w]
    ps.cmd_reviewed(["--thermos", f"{base}..{head}"])
    assert ps.review_warnings(base, head, str(repo)) == []


def test_docs_need_no_review_and_the_warning_never_stops_a_push(tmp_path, monkeypatch, capsys):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "notes.md").write_text("n\n")
    g("add", "."); g("commit", "-qm", "notes")
    assert ps.review_warnings(base, g("rev-parse", "HEAD"), str(repo)) == []
    ps.warn_reviews("not-a-commit", "HEAD", str(repo))        # must not raise
    assert "review check skipped" in capsys.readouterr().err


def test_reviewing_one_commit_records_that_commit_only(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "a.py").write_text("a = 1\n")
    g("add", "."); g("commit", "-qm", "a")
    (repo / "b.py").write_text("b = 1\n")
    g("add", "."); g("commit", "-qm", "b")
    ps.cmd_reviewed([g("rev-parse", "HEAD")])            # just b, not a and its history
    left = ps.review_warnings(base, g("rev-parse", "HEAD"), str(repo))
    assert len(left) == 1 and " a -- no /code-review" in left[0]


def test_the_risky_list_covers_what_claude_md_names():
    """CLAUDE.md section 6's risky kinds, one known module each: stored data,
    sidecars, paid cloud calls, motion and the gate itself."""
    for path in ("control/inventory_store.py", "control/reconfirm.py", "brain/tiered.py",
                 "control/perception_eval.py", "robot/safety.py", "control/mission_runner.py",
                 "brain/explore.py", "tools/hooks/pre-push", "cloudformation/serverless.yaml"):
        assert ps.risky([path]), path
    assert not ps.risky(["tests/test_explore.py", "docs/README.md"])


def test_a_closed_stderr_never_stops_a_push(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "tool.py").write_text("x = 1\n")
    g("add", "."); g("commit", "-qm", "a tool")

    class Closed:
        def write(self, *_a):
            raise BrokenPipeError("stderr closed")

        def flush(self):
            raise BrokenPipeError("stderr closed")
    monkeypatch.setattr(ps.sys, "stderr", Closed())
    ps.warn_reviews(base, g("rev-parse", "HEAD"), str(repo))      # must not raise


def test_a_coloured_git_config_still_gives_a_patch_id(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    g("config", "color.ui", "always")       # a user's setting must not empty the id
    (repo / "tool.py").write_text("x = 1\n")
    g("add", "."); g("commit", "-qm", "a tool")
    assert ps.commit_patch_id(g("rev-parse", "HEAD"), str(repo))


def test_moving_a_risky_file_away_still_asks_for_thermos(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    (repo / "robot").mkdir()
    (repo / "robot" / "safety.py").write_text("STOP = 20\n" * 20)
    g("add", "."); g("commit", "-qm", "safety")
    base = g("rev-parse", "HEAD")
    g("mv", "robot/safety.py", "robot/veto.py")
    g("commit", "-qm", "rename")
    ps.cmd_reviewed([f"{base}..HEAD"])
    assert [w for w in ps.review_warnings(base, g("rev-parse", "HEAD"), str(repo)) if "Thermos" in w]
