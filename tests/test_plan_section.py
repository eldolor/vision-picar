"""tools/plan_section.py -- the parallel-sessions tooling
(docs/guides/PARALLEL-SESSIONS.md)."""

import os
import sys
import pytest

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


def test_the_fast_tier_runs_on_half_the_cores_by_default(monkeypatch, capsys):
    """3.121: xdist workers are half the usable cores, fewer by
    `PICAR_TEST_WORKERS`, never more; each file whole on one worker."""
    monkeypatch.delenv("PICAR_TEST_WORKERS", raising=False)
    monkeypatch.delenv("PYTEST_DISABLE_PLUGIN_AUTOLOAD", raising=False)
    monkeypatch.setattr(ps, "usable_cores", lambda: 12)
    monkeypatch.setattr(ps, "xdist_starts", lambda py, root, load: True)
    assert ps.worker_count() == 6
    assert ps.parallel_args(sys.executable, ROOT) == ["-n", "6", "--dist", "loadfile"]
    monkeypatch.setenv("PICAR_TEST_WORKERS", "3")
    assert ps.parallel_args(sys.executable, ROOT) == ["-n", "3", "--dist", "loadfile"]
    monkeypatch.setenv("PICAR_TEST_WORKERS", "24")
    assert ps.worker_count() == 6 and "over half" in capsys.readouterr().out
    monkeypatch.setenv("PICAR_TEST_WORKERS", "x")
    assert ps.worker_count() == 6 and "not a number" in capsys.readouterr().out
    monkeypatch.setenv("PICAR_TEST_WORKERS", "")
    assert ps.worker_count() == 6 and capsys.readouterr().out == ""
    monkeypatch.setenv("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
    assert ps.parallel_args(sys.executable, ROOT)[:2] == ["-p", "xdist.plugin"]


def test_one_worker_or_no_xdist_runs_serially(monkeypatch, capsys):
    monkeypatch.setattr(ps, "usable_cores", lambda: 12)
    monkeypatch.setenv("PICAR_TEST_WORKERS", "1")
    assert ps.parallel_args(sys.executable, ROOT) == []
    monkeypatch.setenv("PICAR_TEST_WORKERS", "4")
    assert ps.parallel_args("/bin/false", ROOT) == []      # an interpreter that cannot
    assert "runs serially" in capsys.readouterr().out
    # The real check, in this venv: pytest starts one worker if xdist is
    # installed, and says no -- never raises -- if it is not.
    import importlib.util
    monkeypatch.delenv("PYTEST_ADDOPTS", raising=False)
    assert ps.xdist_starts(sys.executable, ROOT, []) == bool(importlib.util.find_spec("xdist"))


def test_the_core_count_without_an_affinity_call(monkeypatch):
    """macOS has no `sched_getaffinity`: the host's count, or 2 without one."""
    monkeypatch.delattr(ps.os, "sched_getaffinity", raising=False)
    monkeypatch.setattr(ps.os, "cpu_count", lambda: 10)
    assert ps.usable_cores() <= 10
    monkeypatch.setattr(ps.os, "cpu_count", lambda: None)
    assert 1 <= ps.usable_cores() <= 2


def _no_desktop_lock(monkeypatch, held):
    """Record the desktop lock's modes instead of taking it (3.128)."""
    import contextlib
    import desktop_lock

    @contextlib.contextmanager
    def fake(mode, why="", wait=True):
        held.append(mode)
        yield mode
    monkeypatch.setattr(desktop_lock, "held", fake)


def test_the_suite_runs_under_the_desktop_lock(monkeypatch):
    """3.128: the fast tier (`ready`, the push gate) holds the desktop lock
    shared, waiting for a live run but not for another suite; `ready
    --full` holds it exclusive. pytest sees the mode, so its own heavy runs
    pass through."""
    held, envs = [], []
    monkeypatch.setattr(ps, "parallel_args", lambda py, root: [])
    monkeypatch.setattr(ps, "venv_python", lambda root: sys.executable)
    monkeypatch.setattr(ps.subprocess, "run", lambda cmd, **kw: (cmd[1:3] == ["-m", "pytest"] and envs.append(kw["env"])) or
                        type("R", (), {"returncode": 0, "stdout": ""})())
    _no_desktop_lock(monkeypatch, held)
    assert ps.run_tests(ROOT, full=False) and ps.run_tests(ROOT, full=True)
    assert held == ["shared", "exclusive"]
    assert [e["PICAR_DESKTOP_LOCK"] for e in envs] == ["shared", "exclusive"]


def test_only_the_fast_tier_runs_in_parallel(monkeypatch):
    """`ready --full` (and so the nightly's browser and live-stack tests)
    stays one process; the fast tier gets the xdist arguments."""
    ran = []
    monkeypatch.setattr(ps, "usable_cores", lambda: 12)
    monkeypatch.setattr(ps, "xdist_starts", lambda py, root, load: True)
    monkeypatch.setenv("PICAR_TEST_WORKERS", "4")
    monkeypatch.delenv("PYTEST_DISABLE_PLUGIN_AUTOLOAD", raising=False)
    monkeypatch.setattr(ps, "venv_python", lambda root: sys.executable)
    monkeypatch.setattr(ps.subprocess, "run", lambda cmd, **kw: ran.append(cmd) or
                        type("R", (), {"returncode": 0, "stdout": ""})())
    _no_desktop_lock(monkeypatch, [])
    assert ps.run_tests(ROOT, full=True)
    assert ps.run_tests(ROOT, full=False)
    full, fast = [c for c in ran if c[1:3] == ["-m", "pytest"]]
    assert "-n" not in full and not any(a.startswith("--ignore") for a in full)
    assert fast[fast.index("-n"):][:4] == ["-n", "4", "--dist", "loadfile"]


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
    assert ps.commit_diff_id(g("rev-parse", "HEAD"), str(repo))


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


def test_ports_past_3_99_do_not_wrap():
    """3.107: 3.104 got 3.4's ports and 3.106 3.6's (minor % 100). Ports need
    not wrap: unique for any section, below the ephemeral range, and 3.1-3.99
    unchanged."""
    seen = {}
    for minor in range(1, 2001):
        r = ps.resources(f"3.{minor}")
        for k in ("PICAR_ROBOT_PORT", "PICAR_BRAIN_PORT", "PICAR_BRIDGE_PORT"):
            assert (k, r[k]) not in seen, f"3.{minor} and {seen[(k, r[k])]} share {k}"
            seen[(k, r[k])] = f"3.{minor}"
        assert r["PICAR_BRIDGE_PORT"] < 32768
        if minor < 100:
            assert r["PICAR_ROBOT_PORT"] == 9000 + 10 * minor
        assert 1 <= r["ROS_DOMAIN_ID"] <= 101 and r["ROS_DOMAIN_ID"] not in (73, 74)


def test_env_names_an_active_section_on_the_same_ros_domain():
    """3.107: domains must stay in 1-101, so past 3.99 two sections share one;
    `env` says so, as a comment `eval` ignores, when the other is active."""
    text = ps.env_text("3.106", others=["3.6", "3.50", "3.106"])
    warn = [l for l in text.splitlines() if "ROS domain" in l and "3.6" in l]
    assert warn and warn[0].startswith("#"), text
    assert "3.50" not in "".join(warn)
    assert all(l.startswith(("export ", "#")) for l in text.splitlines())
    assert not [l for l in ps.env_text("3.50", others=["3.6"]).splitlines() if "share" in l]


def test_env_for_a_named_section_still_works_outside_a_checkout(monkeypatch):
    """Thermos on a74d873: `env 3.106` needed no git before 3.107's check of
    active sections; outside a repo it now raised. No warnings then."""
    def no_repo(*a, **k):
        raise SystemExit("fatal: not a git repository")
    monkeypatch.setattr(ps, "top", no_repo)
    text = ps.env_text("3.106")
    assert "export PICAR_ROBOT_PORT=10060" in text and "WARNING" not in text


def test_env_header_shows_the_quoted_eval():
    """/code-review on a74d873: unquoted, `eval $(... env)` joins the lines
    into one comment and exports nothing."""
    assert 'eval "$(python tools/plan_section.py env)"' in ps.env_text("3.44", others=[])


def test_a_section_whose_ports_would_reach_the_ephemeral_range_is_refused():
    ps.resources("3.2376")
    with pytest.raises(SystemExit):
        ps.resources("3.2377")


def test_active_sections_skips_prunable_worktrees(monkeypatch):
    porcelain = ("worktree /a\nHEAD 1\nbranch refs/heads/plan/3.6-old\nprunable gitdir file points to non-existent location\n\n"
                 "worktree /b\nHEAD 2\nbranch refs/heads/plan/3.50-live\n")
    monkeypatch.setattr(ps, "git", lambda *a, **k: porcelain)
    assert ps.active_sections("/x") == ["3.50"]


def test_a_far_out_branch_elsewhere_does_not_break_env():
    """/code-review on f624674: the warning computed every other section's
    full resources, so a worktree on 3.2400 made `env` fail everywhere."""
    text = ps.env_text("3.50", others=["3.2400", "3.150"])
    assert "export PICAR_ROBOT_PORT=9500" in text and "3.150" in text
