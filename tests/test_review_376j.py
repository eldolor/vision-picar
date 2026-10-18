"""
tests/test_review_376j.py

`PLAN-ros-alignment.md` 3.76, batch J: the bugs the whole-file review of
`tools/plan_claims.py` confirmed (the nightly script's are in
`test_review_376j_nightly.py`). Each test was
written before its fix and seen to fail on `dev` for the reason it names,
except the ones marked "a guard": those pin what the fix must keep doing.
"""

import json
import os
import time

import pytest

from tests.test_plan_claims import OTHER, SECTION, TOOLS, _git, owners, repo  # noqa: F401
import plan_claims as pc  # noqa: E402  (tools/ is on the path via test_plan_claims)


def _commit_dev(repo, msg):
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", msg)
    _git(repo, "update-ref", "refs/remotes/origin/dev", "HEAD")


def _rewrite(repo, name, old, new):
    p = os.path.join(repo, pc.DIR, name)
    text = open(p).read()
    assert old in text
    open(p, "w").write(text.replace(old, new))


# ---------- J4: a State cell other than exactly open/done/dropped ----------

def test_j4_a_done_step_with_a_note_is_done(repo, owners):
    """3.70.M5 reads `done (sweep met; live FAILED 10/13)` on dev: the board
    listed it free, `claim` took it, and a step needing it stayed blocked."""
    _, a = owners()
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |",
             "| `brain/a.py`, `sim/` | done (sweep met; live FAILED 10/13) |")
    _commit_dev(repo, "A done with a note")
    with pytest.raises(SystemExit, match="done"):
        pc.claim("3.90.A", cwd=repo, me=a)
    assert pc.claim("3.90.B", cwd=repo, me=a)["step"] == "3.90.B"     # its need is met


def test_j4_an_unknown_state_is_never_claimable(repo, owners):
    """3.81.D reads `deferred`: anything but open, done or dropped must not be
    offered as free work."""
    _, a = owners()
    _rewrite(repo, "3.91-other.md", "| `control/x.py` | open |", "| `control/x.py` | deferred |")
    _commit_dev(repo, "deferred")
    with pytest.raises(SystemExit, match="deferred"):
        pc.claim("3.91.A", cwd=repo, me=a)
    assert "3.91.A" in pc.cmd_board(cwd=repo)                          # shown, not hidden
    assert not pc.cmd_next(cwd=repo).startswith("3.91.A")


# ---------- J5: a glob and a directory that name the same files ----------

def test_j5_a_glob_overlaps_a_directory_below_its_literal_part():
    """`fnmatch`'s `*` crosses `/`, so `brain/*.py` covers `brain/survey/x.py`
    (`matches` says so), but `overlap` missed it: two sessions could both
    claim, and the edit guard then fired on each."""
    assert pc.matches("brain/survey/x.py", "brain/*.py")
    assert pc.overlap(["brain/*.py"], ["brain/survey/"])
    assert pc.overlap(["brain/survey/"], ["brain/*.py"])


def test_j5_guard_a_file_is_not_a_directory():
    """A guard: a plain FILE entry overlaps a glob only if the glob matches it,
    an extensionless one too (round 1: `Dockerfile` read as a directory)."""
    assert not pc.overlap(["brain/*.json"], ["brain/survey.py"])
    assert not pc.overlap(["service/slam/*.yaml"], ["service/slam/Dockerfile"])
    assert not pc.overlap(["tools/*.py"], ["tools/hooks/pre-push"])
    assert not pc.overlap(["tools/inventory_*"], ["tools/jetson/"])
    assert not pc.overlap(["tools/inventory_*"], ["tools/jetson/*"])     # round 2: two globs
    assert pc.overlap(["sim/maps/*.py"], ["sim/*/x.py"])
    assert pc.overlap(["brain/*.py"], ["brain/survey.py"])


# ---------- J6: a stale branch's table over a newer dev ----------

def test_j6_a_branch_forked_before_devs_edit_does_not_shadow_it(repo, owners):
    """A branch whose commits reached dev under other shas (as
    plan/3.76-metrics-cancelled-by's did) stays "unmerged" for good; if it
    edited its section file, its old table replaced dev's newer one."""
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.90-old")
    _rewrite(repo, "3.90-test.md", "| First |", "| First (old title) |")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "old branch edit")
    _git(repo, "checkout", "-q", "dev")
    time.sleep(1.1)                               # dev's edit is newer by the clock too
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |", "| `brain/a.py`, `sim/` | done |")
    _commit_dev(repo, "A done on dev")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.A"]["state"] == "done"
    with pytest.raises(SystemExit, match="done"):
        pc.claim("3.90.A", cwd=repo, me=a)


def test_j6_a_branch_merged_into_dev_counts_when_it_reached_dev(repo, owners):
    """Round 7: dev's time was its newest commit touching the file, so a
    sibling branch merged with --no-ff counted at its own older commit's time,
    and a stale branch edited in between still shadowed dev."""
    _, a = owners()
    _git(repo, "branch", "plan/3.90-x")
    _git(repo, "checkout", "-q", "-b", "plan/3.90-y")
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |", "| `brain/a.py`, `sim/` | done |")
    _git(repo, "commit", "-q", "-am", "y: A done")
    time.sleep(1.1)
    _git(repo, "checkout", "-q", "plan/3.90-x")
    _rewrite(repo, "3.90-test.md", "| First |", "| First (x) |")
    _git(repo, "commit", "-q", "-am", "x: title")
    time.sleep(1.1)
    _git(repo, "checkout", "-q", "dev")
    _git(repo, "merge", "-q", "--no-ff", "-m", "merge y", "plan/3.90-y")
    _git(repo, "update-ref", "refs/remotes/origin/dev", "HEAD")
    _git(repo, "branch", "-D", "plan/3.90-y")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.A"]["state"] == "done"


def test_j6_a_renamed_section_file_keeps_its_table(repo):
    """Round 8: a branch that renamed its section file listed the old path
    too; read from there (gone on the branch), every step vanished."""
    _git(repo, "checkout", "-q", "-b", "plan/3.90-rename")
    _git(repo, "mv", f"{pc.DIR}/3.90-test.md", f"{pc.DIR}/3.90-a-new-name.md")
    _git(repo, "commit", "-q", "-m", "rename")
    _git(repo, "checkout", "-q", "dev")
    assert "3.90.A" in {s["id"] for s in pc.load_steps(repo)}


def test_j6_a_renamed_file_does_not_hide_devs_later_edit(repo):
    """Round 10: a branch that renamed its section file had no copy at the
    new name on dev to compare, so dev's later edit of the old name was
    hidden by the branch's older table."""
    _git(repo, "checkout", "-q", "-b", "plan/3.90-rename")
    _git(repo, "mv", f"{pc.DIR}/3.90-test.md", f"{pc.DIR}/3.90-a-new-name.md")
    _git(repo, "commit", "-q", "-m", "rename")
    _git(repo, "checkout", "-q", "dev")
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |", "| `brain/a.py`, `sim/` | done |")
    _commit_dev(repo, "A done on dev, old name")
    assert {s["id"]: s for s in pc.load_steps(repo)}["3.90.A"]["state"] == "done"


def test_j6_guard_a_branch_with_devs_copy_still_wins(repo):
    """A guard: a branch that holds dev's copy of the file and edits it on top
    (the live session's own table) is still read first."""
    _git(repo, "checkout", "-q", "-b", "plan/3.90-live")
    _rewrite(repo, "3.90-test.md", "| First |", "| First (live) |")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "live edit")
    _git(repo, "checkout", "-q", "dev")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.A"]["title"] == "First (live)"


def test_j6_a_live_branch_reads_devs_table_until_it_merges_dev(repo):
    """Round 1 let a branch's later edit win by commit time; round 9 showed
    no commit time says when a change reached dev (dev merged into the plan
    branch, then fast-forwarded). Now: dev's table until the branch merges
    dev, then the branch's."""
    _git(repo, "checkout", "-q", "-b", "plan/3.90-live")
    _git(repo, "checkout", "-q", "dev")
    _rewrite(repo, "3.90-test.md", "| Shares sim/ with A |", "| Shares (dev) |")
    _commit_dev(repo, "dev edit")
    _git(repo, "checkout", "-q", "plan/3.90-live")
    _rewrite(repo, "3.90-test.md", "| First |", "| First (live) |")
    _git(repo, "commit", "-q", "-am", "live edit after dev's")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.E"]["title"] == "Shares (dev)"
    _git(repo, "merge", "-q", "-m", "merge dev", "dev")
    _git(repo, "checkout", "-q", "dev")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.A"]["title"] == "First (live)" and steps["3.90.E"]["title"] == "Shares (dev)"


def test_j6_a_sibling_merged_by_fast_forward_is_not_shadowed(repo):
    """Round 9: the sibling merges dev into itself, dev fast-forwards to that
    merge; a stale branch edited in between still shadowed dev."""
    _git(repo, "branch", "plan/3.90-stale")
    _git(repo, "checkout", "-q", "-b", "plan/3.90-sib")
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |", "| `brain/a.py`, `sim/` | done |")
    _git(repo, "commit", "-q", "-am", "sib: A done")
    _git(repo, "checkout", "-q", "plan/3.90-stale")
    _rewrite(repo, "3.90-test.md", "| First |", "| First (stale) |")
    _git(repo, "commit", "-q", "-am", "stale: title")
    _git(repo, "checkout", "-q", "plan/3.90-sib")
    _git(repo, "merge", "-q", "-m", "merge dev into sib", "dev")
    _git(repo, "checkout", "-q", "dev")
    _git(repo, "merge", "-q", "--ff-only", "plan/3.90-sib")
    _git(repo, "update-ref", "refs/remotes/origin/dev", "HEAD")
    _git(repo, "branch", "-D", "plan/3.90-sib")
    steps = {s["id"]: s for s in pc.load_steps(repo)}
    assert steps["3.90.A"]["state"] == "done" and steps["3.90.A"]["title"] == "First"


# ---------- J7: a claim judged against a process list from before the lock ----------

def test_j7_a_claim_made_while_the_board_was_built_is_not_overwritten(repo, owners, monkeypatch):
    """claim() took `ps` once, before the lock; a session that started after
    that snapshot and claimed in the meantime read as dead, and was
    overwritten: two sessions on one step."""
    _, a = owners()
    _, b = owners()
    real = pc.processes
    calls = []

    def snapshot():
        calls.append(1)
        procs = real()
        if len(calls) == 1:                       # the board's snapshot: b had not started
            procs.pop(b["pid"], None)
        return procs

    monkeypatch.setattr(pc, "processes", snapshot)
    orig_board = pc.Board.__init__

    def board_then_b_claims(self, *args, **kw):
        orig_board(self, *args, **kw)
        path = os.path.join(pc.claims_dir(self.common), "3.90.A.json")
        json.dump({"step": "3.90.A", "section": "3.90", "title": "First", "touches": [],
                   "pid": b["pid"], "start": b["start"]}, open(path, "w"))

    monkeypatch.setattr(pc.Board, "__init__", board_then_b_claims)
    with pytest.raises(SystemExit, match="claimed"):
        pc.claim("3.90.A", cwd=repo, me=a)


# ---------- J8: ticked and plain Touches entries in one cell ----------

def test_j8_a_mixed_touches_cell_is_never_misread(repo, owners):
    """A plain entry beside ticked ones was dropped without a word, so a
    conflicting claim was granted. Rounds 1-3 guessed which words were paths;
    round 4 (the reviewers' altitude point) stops guessing: such a step is
    `unknown`, with the text named, until every path is ticked."""
    assert pc._items("`a.py`, `b/`") == (["a.py", "b/"], "")                 # a guard
    assert pc._items("--") == ([], "")                                       # a guard
    assert pc._items("brain/a.py, sim/") == (["brain/a.py", "sim/"], "")     # a guard
    assert pc._items("`brain/survey.py`, control/") == (["brain/survey.py"], "control/")
    assert pc._items("brain/a.py and sim/") == ([], "brain/a.py and sim/")     # round 8
    assert pc._items("brain/a.py; sim/, x.py") == (["x.py"], "brain/a.py; sim/")
    assert "unknown   3.91.A" not in pc.cmd_board(cwd=repo, short=True)     # before the edit
    _, a = owners()
    _rewrite(repo, "3.91-other.md", "| `control/x.py` | open |", "| `control/x.py`, brain/a.py | open |")
    _commit_dev(repo, "mixed cell")
    with pytest.raises(SystemExit, match="tick every path"):
        pc.claim("3.91.A", cwd=repo, me=a)
    assert "unknown   3.91.A" in pc.cmd_board(cwd=repo, short=True)  # round 5: the start-up board


def test_j4_guard_a_bold_done_is_done():
    assert pc._state("**done**") == "done" and pc._state("Done.") == "done"
    assert pc._state("") == "open" and pc._state("deferred") == "deferred"
    # round 2: a struck-out state, or a question, is not the word it holds
    assert pc._state("~~open~~ done") not in pc.STATES
    assert pc._state("done?") not in pc.STATES
    assert pc._state("`done`") == "done"                                  # round 3: ticked


def test_j4_round13_a_word_must_stand_alone():
    assert pc._state("done/dropped") not in pc.STATES and pc._state("done-ish") not in pc.STATES
    assert pc._state("done.") == "done" and pc._state("done (x)") == "done"
    assert pc._waits("`rover`") == "rover" and pc._waits("user (calipers)") == "user"
    assert pc._waits("--") == "" and pc._waits("jetson").startswith("?")
    assert pc._items("`brain/a.py, sim/`") == ([], "brain/a.py, sim/")


def test_j4_round13_an_annotated_wait_still_waits(repo, owners):
    """Round 13: Waits `` `rover` `` read as no wait, and the step as free."""
    _, a = owners()
    _rewrite(repo, "3.90-test.md", "| user | `brain/c.py` |", "| `rover` | `brain/c.py` |")
    _commit_dev(repo, "ticked wait")
    with pytest.raises(SystemExit, match="waits on the rover"):
        pc.claim("3.90.C", cwd=repo, me=a)


def test_j4_a_held_step_stays_its_holders_whatever_its_state(repo, owners):
    """Round 2: the State check ran before the claim check, so a session that
    wrote `in progress` on its own step lost it from `mine`."""
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    _rewrite(repo, "3.90-test.md", "| `brain/a.py`, `sim/` | open |",
             "| `brain/a.py`, `sim/` | in progress |")
    _commit_dev(repo, "in progress")
    assert pc.claim("3.90.A", cwd=repo, me=a)["pid"] == a["pid"]
    with pytest.raises(SystemExit, match="claimed by pid"):
        pc.claim("3.90.A", cwd=repo, me=b)


def test_r14_a_marked_up_no_wait_is_no_wait():
    """A ticked `--`, an em dash or "none" in Waits read as unknown text, and
    the step was never free; a ticked `--` in Touches became a path."""
    import plan_claims as pc
    for cell in ("`--`", "\u2014", "none", " - ", "**--**"):
        assert pc._waits(cell) == "", cell
    assert pc._items("`--`") == ([], "")
    assert pc._waits("`rover`") == "rover"


def test_r15_a_dash_beside_ticked_paths_is_not_stray():
    import plan_claims as pc
    assert pc._items("`a.py`, --") == (["a.py"], "")
    assert pc._items("`a.py` and b.py")[1]          # real stray text still shows


def test_r18_a_ticked_dash_does_not_change_how_the_rest_reads():
    import plan_claims as pc
    assert pc._items("`--`, b.py") == ([], "b.py")       # as beside any ticked entry


def test_r19_a_glob_is_an_entry_and_semicolons_separate():
    import plan_claims as pc
    assert pc._items("*") == (["*"], "")
    assert pc._items("`**`") == (["**"], "")
    assert pc._items("`a.py`; `b.py`") == (["a.py", "b.py"], "")
    assert pc._waits("**--**") == "" and pc._waits("*") != ""


def test_r20_dashes_beside_ticked_paths_and_unbalanced_markup():
    import plan_claims as pc
    assert pc._items("`a.py`, -, -") == (["a.py"], "")
    assert pc._no_entry("**--**") and not pc._no_entry("**none") and not pc._no_entry("*-")


def test_r21_no_beside_ticked_paths_is_text():
    import plan_claims as pc
    assert pc._items("`a.py` no `b.py`")[1], "the annotation vanished"

