"""tools/plan_claims.py -- discrete plan steps and live claims (PLAN 3.75,
docs/guides/PARALLEL-SESSIONS.md rule 8). Each test builds a throwaway repo
with its own `origin/dev`; owners are real `sleep` processes, so liveness is
checked exactly as for a Claude Code process."""

import json
import os
import subprocess
import sys
import time

import pytest

TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools")
sys.path.insert(0, TOOLS)
import plan_claims as pc  # noqa: E402

SECTION = """# 3.90 Test section

## Criteria

1. none

## Steps

| Step | Title | Needs | Waits | Touches | State |
|---|---|---|---|---|---|
| 3.90.A | First | -- | -- | `brain/a.py`, `sim/` | open |
| 3.90.B | Second, after A | 3.90.A | -- | `brain/b.py` | open |
| 3.90.C | Waits on the user | -- | user | `brain/c.py` | open |
| 3.90.D | Done already | -- | -- | `brain/d.py` | done |
| 3.90.E | Shares sim/ with A | -- | -- | `sim/maps/*.py` | open |

## Records

| 3.90.Z | not a step: outside the Steps table | -- | -- | `x` | open |
"""

OTHER = """# 3.91 Other section

## Steps

| Step | Title | Needs | Waits | Touches | State |
|---|---|---|---|---|---|
| 3.91.A | Disjoint | -- | -- | `control/x.py` | open |
| 3.91.B | Overlaps 3.90.A's file | -- | -- | `brain/a.py` | open |
"""


def _git(cwd, *args):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                   env={**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
                        "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"})


@pytest.fixture
def repo(tmp_path):
    for k in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE"):
        os.environ.pop(k, None)
    r = tmp_path / "repo"
    (r / pc.DIR).mkdir(parents=True)
    (r / pc.DIR / "3.90-test.md").write_text(SECTION)
    (r / pc.DIR / "3.91-other.md").write_text(OTHER)
    _git(r, "init", "-q", "-b", "dev")
    _git(r, "add", "-A")
    _git(r, "commit", "-q", "-m", "base")
    _git(r, "update-ref", "refs/remotes/origin/dev", "HEAD")
    return str(r)


@pytest.fixture
def owners():
    """Real processes to own claims; killed at the end."""
    procs = []

    def make():
        p = subprocess.Popen(["sleep", "300"])
        procs.append(p)
        time.sleep(0.02)
        return p, pc.owner_of(p.pid)

    yield make
    for p in procs:
        p.kill()
        p.wait()


# ---------- parsing and paths ----------

def test_steps_table_is_parsed_and_only_the_steps_table():
    steps = pc.parse_steps(SECTION, "3.90")
    assert [s["id"] for s in steps] == ["3.90.A", "3.90.B", "3.90.C", "3.90.D", "3.90.E"]
    a, b, c, d, _ = steps
    assert a["touches"] == ["brain/a.py", "sim/"] and a["needs"] == [] and a["waits"] == ""
    assert b["needs"] == ["3.90.A"] and c["waits"] == "user" and d["state"] == "done"


@pytest.mark.parametrize("path,pattern,hit", [
    ("brain/a.py", "brain/a.py", True),
    ("brain/a.py", "brain/", True),
    ("brain/a.py", "brain", True),
    ("brainy/a.py", "brain", False),
    ("brain/ab.py", "brain/a.py", False),
    ("sim/maps/x.py", "sim/maps/*.py", True),
    ("sim/x.py", "sim/maps/*.py", False),
])
def test_matches(path, pattern, hit):
    assert pc.matches(path, pattern) is hit


def test_overlap_covers_files_directories_and_globs():
    assert pc.overlap(["sim/"], ["sim/maps/*.py"])
    assert pc.overlap(["sim/maps/a.py"], ["sim/"])
    assert pc.overlap(["brain/a.py"], ["brain/a.py"])
    assert not pc.overlap(["brain/a.py"], ["control/", "sim/maps/*.py"])


# ---------- criterion 1: one winner ----------

RACER = """
import os, sys, time
sys.path.insert(0, {tools!r})
import plan_claims as pc
while not os.path.exists({go!r}):
    time.sleep(0.001)
try:
    pc.claim("3.90.A", cwd={repo!r})
    print("WIN")
except SystemExit as e:
    print("REFUSED", e)
"""


def test_criterion_1_twenty_racers_one_winner(repo, owners, tmp_path):
    """50 trials were measured once (3.75 records, ~70 s); the suite runs 1 so
    the fast tier stays fast. PICAR_RACE_TRIALS=50 repeats the measurement."""
    racers, trials = 20, int(os.environ.get("PICAR_RACE_TRIALS", "1"))
    holders = [owners()[0] for _ in range(racers)]
    wins = []
    for trial in range(trials):
        go = str(tmp_path / f"go-{trial}")
        code = RACER.format(tools=TOOLS, go=go, repo=repo)
        ps = [subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True,
                               env={**os.environ, "PICAR_CLAIM_PID": str(h.pid)})
              for h in holders]
        time.sleep(0.3)                     # every racer is waiting on the go file
        open(go, "w").close()
        outs = [p.communicate(timeout=60)[0] for p in ps]
        wins.append(sum(o.startswith("WIN") for o in outs))
        pc.release("3.90.A", cwd=repo, me=pc.owner_of(holders[outs.index(
            next(o for o in outs if o.startswith("WIN")))].pid))
    assert wins == [1] * trials, wins


# ---------- criterion 2: liveness ----------

def test_criterion_2_a_live_claim_is_refused_to_everyone_else(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    for _ in range(5):
        with pytest.raises(SystemExit, match="claimed"):
            pc.claim("3.90.A", cwd=repo, me=b)
    assert pc.claim("3.90.A", cwd=repo, me=a)["pid"] == a["pid"]     # idempotent for the owner


def test_criterion_2_a_dead_owner_frees_the_claim(repo, owners):
    pa, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    pa.kill()
    pa.wait()
    assert pc.claim("3.90.A", cwd=repo, me=b)["pid"] == b["pid"]


def test_criterion_2_a_reused_pid_does_not_keep_the_claim(repo, owners):
    _, a = owners()
    _, b = owners()
    c = pc.claim("3.90.A", cwd=repo, me=a)
    path = os.path.join(pc.repo(repo)[1], "plan-claims", "3.90.A.json")
    c["start"] = "Thu Jan  1 00:00:00 1970"          # same PID, another process
    json.dump(c, open(path, "w"))
    assert pc.claim("3.90.A", cwd=repo, me=b)["pid"] == b["pid"]


# ---------- criterion 3: what is refused ----------

def test_criterion_3_overlap_needs_waits_and_section(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    with pytest.raises(SystemExit, match="conflict"):
        pc.claim("3.91.B", cwd=repo, me=b)          # the same file
    with pytest.raises(SystemExit, match="conflict"):
        pc.claim("3.91.B", cwd=repo, me=b, accept_overlap=True)   # never over a live claim
    with pytest.raises(SystemExit, match="claimed"):
        pc.claim("3.90.E", cwd=repo, me=b)          # same section, held by a
    assert pc.claim("3.91.A", cwd=repo, me=b)       # disjoint: granted
    with pytest.raises(SystemExit, match="blocked"):
        pc.claim("3.90.B", cwd=repo, me=a)          # needs 3.90.A, not done
    with pytest.raises(SystemExit, match="waits"):
        pc.claim("3.90.C", cwd=repo, me=a)
    with pytest.raises(SystemExit, match="done"):
        pc.claim("3.90.D", cwd=repo, me=a)
    with pytest.raises(SystemExit, match="no step"):
        pc.claim("3.90.Z", cwd=repo, me=a)


def test_criterion_3_overlap_by_directory_and_glob(repo, owners):
    _, a = owners()
    _, b = owners()
    (open(os.path.join(repo, pc.DIR, "3.92-g.md"), "w")
     .write(OTHER.replace("3.91", "3.92").replace("`control/x.py`", "`sim/maps/house.py`")))
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "g")
    _git(repo, "update-ref", "refs/remotes/origin/dev", "HEAD")
    pc.claim("3.90.A", cwd=repo, me=a)               # holds sim/
    with pytest.raises(SystemExit, match="conflict"):
        pc.claim("3.92.A", cwd=repo, me=b)           # a file inside sim/


def test_an_active_branch_counts_without_a_claim(repo, owners):
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.93-busy")
    os.makedirs(os.path.join(repo, "brain"), exist_ok=True)
    open(os.path.join(repo, "brain", "a.py"), "w").write("x = 1\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "busy")
    _git(repo, "checkout", "-q", "dev")
    with pytest.raises(SystemExit, match="active plan/3.93-busy"):
        pc.claim("3.90.A", cwd=repo, me=a)
    c = pc.claim("3.90.A", cwd=repo, me=a, accept_overlap=True)     # checked by hand
    assert c["accepted_overlap"] == ["plan/3.93-busy: brain/a.py"]
    pc.release(cwd=repo, me=a)
    board = pc.cmd_board(cwd=repo)
    assert "active    3.93" in board


def test_next_names_a_free_step_and_release_frees_it(repo, owners):
    _, a = owners()
    _, b = owners()
    assert pc.cmd_next(cwd=repo).startswith("3.90.A")
    pc.claim("3.90.A", cwd=repo, me=a)
    os.environ["PICAR_CLAIM_PID"] = str(b["pid"])
    try:
        assert pc.cmd_next(cwd=repo).startswith("3.91.A")
    finally:
        del os.environ["PICAR_CLAIM_PID"]
    assert pc.release(cwd=repo, me=a) == ["3.90.A"]
    assert pc.claim("3.90.A", cwd=repo, me=b)


# ---------- criterion 4: the edit guard ----------

def _hook(repo, path, owner_pid, mode="default"):
    env = {**os.environ, "PICAR_CLAIM_PID": str(owner_pid)}
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "plan_claims.py"), "hook-edit"],
                       input=json.dumps({"tool_name": "Edit", "cwd": repo, "permission_mode": mode,
                                         "tool_input": {"file_path": path}}),
                       capture_output=True, text=True, env=env)
    assert r.returncode == 0
    return r.stdout.strip()


def test_criterion_4_the_edit_guard(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    out = _hook(repo, os.path.join(repo, "sim", "new.py"), b["pid"])
    d = json.loads(out)["hookSpecificOutput"]
    assert d["permissionDecision"] == "ask" and "3.90.A" in d["permissionDecisionReason"]
    assert _hook(repo, os.path.join(repo, "sim", "new.py"), a["pid"]) == ""     # its own claim
    assert _hook(repo, os.path.join(repo, "control", "x.py"), b["pid"]) == ""   # unclaimed
    assert _hook("/", "/etc/hosts", b["pid"]) == ""                            # not a repo
    for mode in ("auto", "bypassPermissions", ""):     # no one would see an "ask"
        d = json.loads(_hook(repo, os.path.join(repo, "sim", "new.py"), b["pid"], mode))
        assert d["hookSpecificOutput"]["permissionDecision"] == "deny"
        assert "allow 3.90.A" in d["hookSpecificOutput"]["permissionDecisionReason"]
    pc.allow("3.90.A", cwd=repo, me=b)                 # the user agreed
    assert _hook(repo, os.path.join(repo, "sim", "new.py"), b["pid"], "auto") == ""
    _, c3 = owners()
    assert _hook(repo, os.path.join(repo, "sim", "new.py"), c3["pid"], "auto")  # only for b


@pytest.mark.parametrize("stdin", ["", "not json", "[]", '{"tool_input": null}', '{"tool_input": {}}'])
def test_criterion_4_malformed_input_says_nothing(stdin):
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "plan_claims.py"), "hook-edit"],
                       input=stdin, capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout == ""


# ---------- criterion 5: the commit guard ----------

def test_criterion_5_the_commit_guard_warns_and_never_blocks(repo, owners):
    _, a = owners()
    _, b = owners()
    hooks = os.path.join(repo, "hooks")
    os.makedirs(hooks)
    script = open(os.path.join(TOOLS, "hooks", "pre-commit")).read()
    open(os.path.join(hooks, "pre-commit"), "w").write(
        script.replace('"$root/tools/plan_claims.py"', repr(os.path.join(TOOLS, "plan_claims.py"))))
    os.chmod(os.path.join(hooks, "pre-commit"), 0o755)
    _git(repo, "config", "core.hooksPath", hooks)
    pc.claim("3.90.A", cwd=repo, me=a)
    os.makedirs(os.path.join(repo, "sim"))
    open(os.path.join(repo, "sim", "x.py"), "w").write("y = 2\n")
    _git(repo, "add", "-A")
    env = {**os.environ, "PICAR_CLAIM_PID": str(b["pid"]), "GIT_AUTHOR_NAME": "t",
           "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}
    r = subprocess.run(["git", "commit", "-q", "-m", "x"], cwd=repo, capture_output=True,
                       text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert "WARNING sim/x.py is inside step 3.90.A" in r.stderr
    open(os.path.join(repo, "sim", "x.py"), "a").write("z = 3\n")
    _git(repo, "add", "-A")
    env["PICAR_CLAIM_PID"] = str(a["pid"])              # the owner commits: no warning
    r = subprocess.run(["git", "commit", "-q", "-m", "y"], cwd=repo, capture_output=True,
                       text=True, env=env)
    assert r.returncode == 0 and "WARNING" not in r.stderr


# ---------- review fixes (2026-10-12) ----------

def test_review_a_leading_or_middle_glob_is_not_a_prefix_of_everything():
    assert not pc.overlap(["*.md"], ["robot/server.py"])
    assert not pc.overlap(["sim/*/maps"], ["sim/x.py"])
    assert pc.overlap(["*.md"], ["docs/x.md"])
    assert pc.overlap(["sim/maps/*.py"], ["sim/*/x.py"])          # two globs under one directory


def _busy_branch(repo, name, path):
    _git(repo, "checkout", "-q", "-b", name)
    os.makedirs(os.path.join(repo, os.path.dirname(path)), exist_ok=True)
    open(os.path.join(repo, path), "w").write("x = 1\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", name)


def test_review_a_session_never_conflicts_with_its_own_branch(repo, owners):
    _, a = owners()
    _busy_branch(repo, "plan/3.93-mine", "control/x.py")          # this checkout's branch
    assert pc.claim("3.91.A", cwd=repo, me=a)                       # touches control/x.py


def test_review_a_session_may_claim_its_own_idle_section(repo, owners):
    _, a = owners()
    _busy_branch(repo, "plan/3.93-mine", "control/y.py")
    old = str(int(time.time()) - 3 * 3600)
    _git(repo, "commit", "-q", "--amend", "--no-edit", f"--date=@{old}")
    subprocess.run(["git", "commit", "-q", "--amend", "--no-edit"], cwd=repo, check=True,
                   env={**os.environ, "GIT_COMMITTER_DATE": f"@{old}", "GIT_AUTHOR_NAME": "t",
                        "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
                        "GIT_COMMITTER_EMAIL": "t@t"})
    assert pc.claim("3.93", cwd=repo, me=a)
    _git(repo, "checkout", "-q", "dev")
    _, b = owners()
    pc.release(cwd=repo, me=a)
    with pytest.raises(SystemExit, match="idle"):
        pc.claim("3.93", cwd=repo, me=b)                            # someone else: still idle


def test_review_session_start_names_claims_this_process_already_holds(repo, owners):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    os.environ["PICAR_CLAIM_PID"] = str(a["pid"])
    try:
        out = pc.hook_start(cwd=repo)
    finally:
        del os.environ["PICAR_CLAIM_PID"]
    assert "THIS SESSION ALREADY HOLDS 3.90.A" in out and "release 3.90.A" in out


# ---------- Thermos bug pass fixes (2026-10-12) ----------

def test_thermos_owner_start_does_not_depend_on_the_callers_time_zone(owners):
    p, a = owners()
    os.environ["TZ"] = "Pacific/Kiritimati"                 # UTC+14: another rendering of lstart
    try:
        time.tzset()
        assert pc.alive(a, pc.processes())
    finally:
        del os.environ["TZ"]
        time.tzset()


def test_thermos_untracked_files_alone_do_not_make_a_branch_active(repo, owners):
    _, a = owners()
    _git(repo, "branch", "plan/3.94-merged")              # nothing beyond origin/dev
    wt = os.path.join(os.path.dirname(repo), "wt94")
    _git(repo, "worktree", "add", "-q", wt, "plan/3.94-merged")
    os.makedirs(os.path.join(wt, "brain"))
    open(os.path.join(wt, "brain", "a.py"), "w").write("log\n")   # an untracked log, just now
    assert "3.94" not in pc.section_branches(repo)
    assert pc.claim("3.90.A", cwd=repo, me=a)


def test_thermos_a_dirty_section_file_does_not_shadow_dev(repo):
    _git(repo, "branch", "plan/3.90-test")                # nothing committed beyond origin/dev
    wt = os.path.join(os.path.dirname(repo), "wt90")
    _git(repo, "worktree", "add", "-q", wt, "plan/3.90-test")
    open(os.path.join(repo, pc.DIR, "3.90-test.md"), "w").write(SECTION.replace(
        "| 3.90.A | First | -- | -- | `brain/a.py`, `sim/` | open |",
        "| 3.90.A | First | -- | -- | `brain/a.py`, `sim/` | done |"))
    _git(repo, "commit", "-qam", "A done on dev")
    _git(repo, "update-ref", "refs/remotes/origin/dev", "HEAD")
    open(os.path.join(wt, pc.DIR, "3.90-test.md"), "a").write("\nediting\n")   # dirty, old table
    states = {s["id"]: s["state"] for s in pc.load_steps(repo)}
    assert states["3.90.A"] == "done"


def test_thermos_allow_takes_only_a_step_id(repo, owners):
    _, a = owners()
    with pytest.raises(SystemExit, match="not a step id"):
        pc.allow("../../config", cwd=repo, me=a)
