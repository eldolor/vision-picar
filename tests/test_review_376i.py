"""
tests/test_review_376i.py

`PLAN-ros-alignment.md` 3.76, batch I: the bugs the whole-file review of
the parallel-sessions tooling (`tools/plan_section.py`, `tools/hooks/`)
confirmed. Each test was written before its fix and seen to fail on `dev`
(or on the previous round's fix) for the reason it names, except the ones
marked "a guard": those pin what the new id must keep doing.
"""

import os
import stat
import subprocess as sp
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import plan_section as ps  # noqa: E402
from tests.test_plan_section import _scratch_repo  # noqa: E402


def _with_origin(tmp_path, monkeypatch):
    """A scratch checkout with an `origin` holding `dev`, and the section
    directory and its index committed -- what `ready` and `new` expect."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    os.makedirs(repo / ps.DIR)
    ps.write_index(str(repo))
    g("add", "."); g("commit", "-qm", "index")
    bare = tmp_path / "origin.git"
    sp.run(["git", "init", "-q", "--bare", str(bare)], check=True, capture_output=True)
    g("remote", "add", "origin", str(bare))
    g("push", "-q", "origin", "main:dev")
    g("fetch", "-q", "origin")
    return repo, g


# ---------- I1: a pass names a tree the commit does not hold ----------


def test_the_gate_refuses_an_untracked_file_the_tests_would_import(tmp_path, monkeypatch):
    """The cleanliness check left untracked files out, and pytest runs on
    the working tree: a forgotten `git add brain/new_helper.py` passed the
    tier, a pass was recorded for the sha, and dev got the ImportError."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "tool.py").write_text("from helper import x\n")
    g("add", "."); g("commit", "-qm", "a tool")
    (repo / "helper.py").write_text("x = 1\n")           # never added
    head = g("rev-parse", "HEAD")
    monkeypatch.setattr(ps, "run_tests", lambda root, full: True)
    with pytest.raises(SystemExit) as exc:
        ps.cmd_gate(base, head)
    assert "helper.py" in str(exc.value)
    assert not os.path.exists(os.path.join(ps.gate_dir(), head)), "a pass was recorded"


def test_a_status_setting_does_not_hide_the_untracked_file(tmp_path, monkeypatch):
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    g("config", "status.showUntrackedFiles", "no")
    (repo / "helper.py").write_text("x = 1\n")
    assert [l for l in ps.uncommitted(str(repo)) if "helper.py" in l]


def test_ready_refuses_an_untracked_file_too(tmp_path, monkeypatch):
    repo, g = _with_origin(tmp_path, monkeypatch)
    (repo / "helper.py").write_text("x = 1\n")
    monkeypatch.setattr(ps, "run_tests", lambda root, full: True)
    with pytest.raises(SystemExit) as exc:
        ps.cmd_ready()
    assert "helper.py" in str(exc.value)
    assert not os.path.exists(os.path.join(ps.gate_dir(), g("rev-parse", "HEAD")))


# ---------- I2: one key for two programs ----------


def _two_indents(tmp_path, monkeypatch):
    """Two commits on one base that differ only in a line's indentation --
    inside the `if`, or after it: different Python."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    (repo / "m.py").write_text("if stop:\n    halt()\n")
    g("add", "."); g("commit", "-qm", "m")
    base = g("rev-parse", "HEAD")
    shas = []
    for name, line in (("inside", "    report()\n"), ("after", "report()\n")):
        g("checkout", "-q", "-b", name, base)
        (repo / "m.py").write_text("if stop:\n    halt()\n" + line)
        g("commit", "-qam", name)
        shas.append(g("rev-parse", "HEAD"))
    return repo, base, shas


def test_moving_a_line_out_of_a_block_is_a_different_change(tmp_path, monkeypatch):
    """`git patch-id` ignores whitespace, so a re-indent kept the earlier
    change's pass and the gate skipped the tier ("already passed")."""
    repo, base, (inside, after) = _two_indents(tmp_path, monkeypatch)
    assert ps.change_id(base, inside, str(repo)) != ps.change_id(base, after, str(repo))


def test_a_review_record_does_not_cover_a_re_indented_commit(tmp_path, monkeypatch):
    repo, _base, (inside, after) = _two_indents(tmp_path, monkeypatch)
    assert ps.commit_diff_id(inside, str(repo)) != ps.commit_diff_id(after, str(repo))


# ---------- I3: a docs-only push lints the working tree ----------


def test_a_docs_only_push_of_another_commit_is_refused(tmp_path, monkeypatch):
    """The docs-only branch returned before the HEAD-and-clean check, so the
    lint ran on whatever the checkout held, not on what was pushed."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    g("checkout", "-qb", "docs")
    (repo / "notes.md").write_text("[broken](nowhere.md)\n")
    g("add", "."); g("commit", "-qm", "notes")
    pushed = g("rev-parse", "HEAD")
    g("checkout", "-q", "main")                            # pushing docs:dev from main
    with pytest.raises(SystemExit) as exc:
        ps.cmd_gate(base, pushed)
    assert "push HEAD" in str(exc.value), exc.value


# ---------- I4: data under docs/ is read by tests ----------


def test_data_files_under_docs_are_not_docs_only():
    """tests/test_clip_rooms_eval.py pins numbers from
    docs/evaluations/*.json: a push of only that file ran the spec lint."""
    assert not ps.docs_only(["docs/evaluations/clip-rooms-3.51-cpu.json"])
    assert ps.docs_only(["docs/evaluations/REPORT.md", "CLAUDE.md"])
    assert ps.docs_only(["docs/guides/ui-390.png"]), "a screenshot is still docs"
    assert not ps.docs_only(["tools/ui-390.png"])


def test_a_code_file_renamed_to_markdown_runs_the_tier(tmp_path, monkeypatch):
    """The gate's file list detected renames and showed only the new path."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    (repo / "eval.json").write_text('{"bar": 0.8, "rows": [1, 2, 3, 4, 5, 6]}\n' * 5)
    g("add", "."); g("commit", "-qm", "data")
    base = g("rev-parse", "HEAD")
    g("mv", "eval.json", "eval.md")
    g("commit", "-qm", "rename")
    ran = []
    monkeypatch.setattr(ps, "run_tests", lambda root, full: ran.append(full) or True)
    ps.cmd_gate(base, g("rev-parse", "HEAD"))
    assert ran == [False], "only the spec lint ran"


# ---------- I5: `new` leaves a branch behind / nests worktrees ----------


def test_a_taken_worktree_path_leaves_no_branch_behind(tmp_path, monkeypatch):
    """`git worktree add -b` creates the branch before it finds the path
    taken; the retry then left `plan/3.1-<slug>` squatting on 3.1."""
    repo, g = _with_origin(tmp_path, monkeypatch)
    taken = repo / ".claude" / "worktrees" / "plan-3.1"
    taken.mkdir(parents=True)
    (taken / "x").write_text("another session's\n")
    ps.cmd_new("foo", "Foo")
    branches = g("branch", "--format=%(refname:short)").splitlines()
    assert "plan/3.2-foo" in branches
    assert "plan/3.1-foo" not in branches, branches


def test_another_sessions_branch_of_the_same_name_is_never_deleted(tmp_path, monkeypatch):
    """Same number, same slug: git refuses ("a branch named ... already
    exists") without creating anything, and the branch is the other
    session's."""
    repo, g = _with_origin(tmp_path, monkeypatch)
    g("branch", "plan/3.1-foo", "origin/dev")            # theirs, not yet checked out
    # ...created just after this session listed the numbers in use:
    monkeypatch.setattr(ps, "used_numbers", lambda root: set())
    taken = repo / ".claude" / "worktrees" / "plan-3.1"
    taken.mkdir(parents=True)
    (taken / "x").write_text("theirs\n")
    ps.cmd_new("foo", "Foo")
    assert "plan/3.1-foo" in g("branch", "--format=%(refname:short)").splitlines()


def test_a_stale_registered_worktree_leaves_no_branch_behind(tmp_path, monkeypatch):
    """A worktree deleted without `git worktree prune`: git creates the
    branch, then fails "missing but already registered", which is not
    "already exists" -- `new` stopped and left the branch."""
    repo, g = _with_origin(tmp_path, monkeypatch)
    stale = repo / ".claude" / "worktrees" / "plan-3.1"
    g("worktree", "add", "-q", "-b", "old", str(stale), "origin/dev")
    import shutil
    shutil.rmtree(stale)
    with pytest.raises(SystemExit):
        ps.cmd_new("foo", "Foo")
    assert "plan/3.1-foo" not in g("branch", "--format=%(refname:short)").splitlines()


def test_new_from_a_worktree_puts_the_new_one_beside_it(tmp_path, monkeypatch):
    """`top()` is the CURRENT worktree, so `new` run from one nested the new
    worktree inside it, where `git worktree remove` of the outer one would
    delete it."""
    repo, g = _with_origin(tmp_path, monkeypatch)
    ps.cmd_new("one", "One")
    first = repo / ".claude" / "worktrees" / "plan-3.1"
    monkeypatch.chdir(first)
    ps.cmd_new("two", "Two")
    assert (repo / ".claude" / "worktrees" / "plan-3.2").is_dir()
    assert not (first / ".claude" / "worktrees" / "plan-3.2").exists()


# ---------- I6: a commit with no text hunks ----------


def test_a_binary_only_commit_is_recorded_and_the_range_goes_on(tmp_path, monkeypatch):
    """A guard: the review's claim that an icon or a chmod gets no id did
    not hold for `git patch-id`; `diff_id` must keep giving one."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    base = g("rev-parse", "HEAD")
    (repo / "a.py").write_text("a = 1\n")
    g("add", "."); g("commit", "-qm", "a")
    (repo / "icon.png").write_bytes(b"\x89PNG\x00\x01\x02")
    g("add", "."); g("commit", "-qm", "icon")
    (repo / "b.py").write_text("b = 1\n")
    g("add", "."); g("commit", "-qm", "b")
    ps.cmd_reviewed([f"{base}..HEAD"])
    assert ps.review_warnings(base, g("rev-parse", "HEAD"), str(repo)) == []


def test_two_different_edits_of_one_image_get_different_ids(tmp_path, monkeypatch):
    """A guard: without the binary payload both read "Binary files differ"
    against one preimage, and (blob ids dropped) shared an id."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    (repo / "icon.png").write_bytes(b"\x00base")
    g("add", "."); g("commit", "-qm", "icon")
    base = g("rev-parse", "HEAD")
    ids = []
    for name, payload in (("one", b"\x00one"), ("two", b"\x00two")):
        g("checkout", "-q", "-b", name, base)
        (repo / "icon.png").write_bytes(payload)
        g("commit", "-qam", name)
        ids.append((ps.commit_diff_id(g("rev-parse", "HEAD"), str(repo)),
                    ps.change_id(base, g("rev-parse", "HEAD"), str(repo))))
    assert ids[0][0] and ids[0][0] != ids[1][0] and ids[0][1] != ids[1][1]


def test_one_edit_to_two_identical_blocks_is_two_changes(tmp_path, monkeypatch):
    """Dropping the whole hunk header made the same edit in two identical
    blocks one change; the function context tells them apart."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    block = "    radius = 0.35\n    cost = 3\n    keep = 1\n    more = 2\n    last = 4\n"
    pad = "".join(f"pad{i} = {i}\n" for i in range(8))
    text = "def local():\n" + block + pad + "def glob():\n" + block + pad
    (repo / "m.py").write_text(text)
    g("add", "."); g("commit", "-qm", "m")
    base = g("rev-parse", "HEAD")
    ids = []
    for name, which in (("local", 0), ("glob", 1)):
        g("checkout", "-q", "-b", name, base)
        parts = text.split("    keep = 1\n")
        parts[which] += "    keep = 1\n    extra = 9\n"
        parts[1 - which] += "    keep = 1\n"
        (repo / "m.py").write_text(parts[0] + parts[1] + parts[2])
        g("commit", "-qam", name)
        ids.append(ps.change_id(base, g("rev-parse", "HEAD"), str(repo)))
    assert ids[0] != ids[1]


def test_a_docs_only_push_runs_the_plan_s_own_tests():
    """test_plan_section.py reads the real section files, index and plan: a
    push of only a section file skipped the index-freshness check."""
    assert "tests/test_plan_section.py" in ps.DOCS_TESTS
    for t in ps.DOCS_TESTS:
        assert os.path.exists(os.path.join(ROOT, t)), t


def test_a_users_diff_config_never_merges_two_changes(tmp_path, monkeypatch):
    """`git diff` follows diff.context: at 0, with line numbers dropped, one
    line added at two places in a file read as one change."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    (repo / "m.py").write_text("".join(f"l{i} = {i}\n" for i in range(20)))
    g("add", "."); g("commit", "-qm", "m")
    base = g("rev-parse", "HEAD")
    g("config", "diff.context", "0")
    g("config", "diff.noprefix", "true")
    ids = []
    for name, at in (("top", 2), ("bottom", 15)):
        g("checkout", "-q", "-b", name, base)
        lines = [f"l{i} = {i}\n" for i in range(20)]
        lines.insert(at, "stop()\n")
        (repo / "m.py").write_text("".join(lines))
        g("commit", "-qam", name)
        ids.append(ps.change_id(base, g("rev-parse", "HEAD"), str(repo)))
    assert ids[0] != ids[1]


# ---------- I7: the venv found from a subdirectory ----------


def test_the_venv_is_found_from_a_subdirectory_of_the_main_checkout(tmp_path, monkeypatch):
    """`--git-common-dir` is a RELATIVE `.git` in the main checkout, and the
    existence test ran against the process's cwd: from `tools/` the suite
    ran on sys.executable (the system Python CLAUDE.md says not to trust)."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    py = repo / ".venv" / "bin" / "python"
    py.parent.mkdir(parents=True)
    py.write_text("#!/bin/sh\n")
    py.chmod(py.stat().st_mode | stat.S_IXUSR)
    (repo / "tools").mkdir()
    monkeypatch.chdir(repo / "tools")
    assert ps.venv_python(str(repo)) == str(py)


# ---------- I8: section 3.100 on ROS's default domain ----------


def test_section_100_is_not_on_the_default_ros_domain():
    """`minor % 100` put 3.100 on domain 0, which every stack started
    without a domain uses."""
    assert ps.resources("3.100")["ROS_DOMAIN_ID"] != 0


# ---------- I9: deleting dev ----------


def test_the_hook_refuses_to_delete_dev(tmp_path, monkeypatch):
    """A zero local sha was skipped before the override check, so
    `git push origin :dev` went through with no gate and no word."""
    repo, g = _scratch_repo(tmp_path, monkeypatch)
    monkeypatch.delenv("PICAR_SKIP_GATE", raising=False)
    zero = "0" * 40
    line = f"(delete) {zero} refs/heads/dev {g('rev-parse', 'HEAD')}\n"
    hook = os.path.join(ROOT, "tools", "hooks", "pre-push")
    r = sp.run(["sh", hook, "origin", "x"], input=line, text=True, cwd=repo,
               capture_output=True)
    assert r.returncode != 0 and "dev" in r.stderr, (r.returncode, r.stderr)
    r = sp.run(["sh", hook, "origin", "x"], input=line, text=True, cwd=repo,
               capture_output=True, env={**os.environ, "PICAR_SKIP_GATE": "1"})
    assert r.returncode == 0, r.stderr
