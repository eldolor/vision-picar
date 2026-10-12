"""
tools/plan_claims.py -- discrete plan steps and live claims, so parallel
Claude Code sessions never work on the same thing (PLAN 3.75,
docs/guides/PARALLEL-SESSIONS.md rule 8).

    python tools/plan_section.py board [--all]     every open step: claimed, active, waiting, blocked or free
    python tools/plan_section.py next              the first step this session may claim
    python tools/plan_section.py claim <step> [--accept-overlap]
                                                   take a step (refused if it would collide);
                                                   --accept-overlap: a file shared with an active
                                                   branch holding no claim was checked by hand
    python tools/plan_section.py release [<step>]  give back this session's claims (or one)

    python tools/plan_claims.py allow <step>       the USER agreed this session may edit inside that claim
    python tools/plan_claims.py hook-edit          PreToolUse hook: ask (or deny, in auto mode) before
                                                   editing another session's claim
    python tools/plan_claims.py hook-commit        pre-commit hook: warn (never block) on the same
    python tools/plan_claims.py hook-start         SessionStart hook: the board's short form

Steps are rows of a `## Steps` table in their section's file:

    | Step | Title | Needs | Waits | Touches | State |
    | 3.70.S | Survey mode | 3.50.D | -- | `brain/survey.py`, `control/` | open |

A claim is a file in the shared .git (`plan-claims/<step>.json`), so every
worktree on this machine sees it. It names the owning Claude Code process
(the nearest `claude` ancestor, with its start time) and is FREE once that
process has exited: nobody has to remember to release it.

Standard library only: the hooks run under whatever `python3` is on PATH.
"""

import fcntl
import fnmatch
import json
import os
import re
import subprocess
import sys
import time

DIR = "docs/plans/ros-alignment"
INDEX = f"{DIR}/README.md"
FILE_RE = re.compile(r"^(3\.(\d+)([a-z]?))-(.+)\.md$")
BRANCH_RE = re.compile(r"^(?:origin/)?plan/(3\.\d+[a-z]?)-")
STEP_ID_RE = re.compile(r"3\.\d+[a-z]?\.[A-Za-z0-9_-]+")
WAITS = ("user", "rover")
CLOSED = ("done", "dropped")
ACTIVE_S = 2 * 3600          # a section branch committed within this is "active"


# ---------- git and processes ----------

def git(*args, cwd=None, check=True) -> str:
    r = subprocess.run(["git", *args], capture_output=True, text=True, cwd=cwd)
    if check and r.returncode != 0:
        raise SystemExit(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


def repo(cwd=None):
    """(worktree top, shared .git dir), both absolute; None outside a repo."""
    r = subprocess.run(["git", "rev-parse", "--show-toplevel", "--path-format=absolute",
                        "--git-common-dir"], capture_output=True, text=True, cwd=cwd)
    if r.returncode != 0:
        return None
    lines = r.stdout.strip().split("\n")
    return (lines[0], lines[1]) if len(lines) == 2 else None


def processes() -> dict:
    """{pid: (ppid, start, command basename)} from one `ps` call."""
    out = subprocess.run(["ps", "-A", "-o", "pid=,ppid=,lstart=,comm="],
                         capture_output=True, text=True).stdout
    procs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        procs[int(parts[0])] = (int(parts[1]), " ".join(parts[2:7]),
                                os.path.basename(" ".join(parts[7:])))
    return procs


def owner(procs=None):
    """{"pid", "start"} of the Claude Code process this one runs under.
    PICAR_CLAIM_PID names another owner (tests; a person in a terminal)."""
    procs = procs if procs is not None else processes()
    forced = os.environ.get("PICAR_CLAIM_PID")
    if forced:
        return owner_of(int(forced), procs)
    pid = os.getpid()
    for _ in range(64):
        if pid not in procs or pid <= 1:
            return None
        ppid, start, name = procs[pid]
        if name == "claude":
            return {"pid": pid, "start": start}
        pid = ppid
    return None


def owner_of(pid: int, procs=None):
    """{"pid", "start"} for a given process, or None if it is not running."""
    procs = procs if procs is not None else processes()
    return {"pid": pid, "start": procs[pid][1]} if pid in procs else None


def alive(claim: dict, procs: dict) -> bool:
    """The claim's owner still runs -- the same process, not a reused PID."""
    p = procs.get(claim.get("pid"))
    return bool(p) and p[1] == claim.get("start")


def same_owner(claim: dict, me) -> bool:
    return bool(me) and claim.get("pid") == me["pid"] and claim.get("start") == me["start"]


# ---------- paths ----------

def _is_glob(p: str) -> bool:
    return any(c in p for c in "*?[")


def matches(path: str, pattern: str) -> bool:
    """A repo path falls under a Touches entry: the same file, a file inside a
    directory entry (trailing `/`, or a plain path with files under it), or a
    glob match."""
    path = path.strip("/")
    if _is_glob(pattern):
        return fnmatch.fnmatch(path, pattern) or fnmatch.fnmatch(path, pattern.rstrip("/") + "/*")
    p = pattern.strip("/")
    return path == p or path.startswith(p + "/")


def overlap(a: list, b: list) -> list:
    """The pairs of Touches entries that can name the same file."""
    def base(p):                   # a glob's literal directory: "sim/maps/*.py" -> "sim/maps/"
        lit = re.split(r"[*?\[]", p, maxsplit=1)[0]
        return lit[:lit.rfind("/") + 1]

    hits = []
    for x in a:
        for y in b:
            if (matches(x, y) or matches(y, x)
                    or _is_glob(x) and _is_glob(y)
                    and (base(x).startswith(base(y)) or base(y).startswith(base(x)))):
                hits.append((x, y))
    return hits


# ---------- the steps ----------

def _cells(line: str) -> list:
    return [c.strip() for c in line.strip().strip("|").split("|")]


def _items(cell: str) -> list:
    ticked = re.findall(r"`([^`]+)`", cell)
    if ticked:
        return ticked
    return [x.strip() for x in cell.split(",") if x.strip() and x.strip() not in ("--", "-")]


def parse_steps(text: str, section: str) -> list:
    """The rows of a section file's `## Steps` table."""
    steps, inside = [], False
    for line in text.split("\n"):
        if line.startswith("## "):
            inside = line.strip().lower() == "## steps"
            continue
        if not inside or not line.lstrip().startswith("|"):
            continue
        c = _cells(line)
        if len(c) < 6 or not STEP_ID_RE.fullmatch(c[0]):
            continue
        waits = c[3].strip().lower()
        steps.append({
            "id": c[0], "section": section, "title": c[1],
            "needs": STEP_ID_RE.findall(c[2]),
            "waits": waits if waits in WAITS else "",
            "touches": _items(c[4]),
            "state": c[5].strip().lower() or "open",
        })
    return steps


def _cat(root: str, specs: list) -> dict:
    """{"ref:path": text} for many blobs in one `git cat-file --batch`."""
    if not specs:
        return {}
    r = subprocess.run(["git", "cat-file", "--batch"], input=("\n".join(specs) + "\n").encode(),
                       capture_output=True, cwd=root)
    out, data, i = {}, r.stdout, 0
    for spec in specs:
        nl = data.index(b"\n", i)
        head = data[i:nl].decode()
        i = nl + 1
        if head.endswith("missing"):
            continue
        size = int(head.split()[2])
        out[spec] = data[i:i + size].decode("utf-8", "replace")
        i += size + 1
    return out


def worktrees(root: str) -> dict:
    """{branch: worktree path} for every checked-out branch."""
    out, path = {}, None
    for line in git("worktree", "list", "--porcelain", cwd=root).splitlines():
        if line.startswith("worktree "):
            path = line[9:]
        elif line.startswith("branch refs/heads/") and path:
            out[line[18:]] = path
    return out


def dirty(wt: str):
    """(newest mtime, paths) of a worktree's uncommitted changes."""
    r = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, cwd=wt)
    newest, paths = 0.0, []
    for line in r.stdout.splitlines():
        rel = line[3:].split(" -> ")[-1].strip('"')
        paths.append(rel.rstrip("/") + ("/" if rel.endswith("/") else ""))
        try:
            newest = max(newest, os.stat(os.path.join(wt, rel)).st_mtime)
        except OSError:
            pass
    return newest, paths


def section_branches(root: str) -> dict:
    """{section: [(branch, last activity, files changed)]} for every plan/*
    branch with commits not on origin/dev or uncommitted work in its worktree.
    Activity is the newer of its last commit and its newest uncommitted edit,
    so a session that has not committed yet still shows."""
    out = {}
    wts = worktrees(root)
    refs = git("for-each-ref", "--format=%(refname:short) %(committerdate:unix)",
               "refs/heads/plan/", cwd=root).splitlines()
    for line in refs:
        b, ts = line.rsplit(" ", 1)
        m = BRANCH_RE.match(b)
        if not m:
            continue
        files, last = [], 0
        base = git("merge-base", "origin/dev", b, cwd=root, check=False)
        if base and git("rev-parse", b, cwd=root) != base:
            files = git("diff", "--name-only", base, b, cwd=root).splitlines()
            last = int(ts)
        if b in wts and os.path.isdir(wts[b]):
            mt, paths = dirty(wts[b])
            files += [f for f in paths if f not in files]
            last = max(last, mt)
        if files:
            out.setdefault(m.group(1), []).append((b, last, [f for f in files if f != INDEX]))
    return out


def load_steps(root: str, branches=None) -> list:
    """Every step in every section, each section read from its newest ref:
    the newest of its plan/* branches if one changed the file, else origin/dev."""
    branches = branches if branches is not None else section_branches(root)
    files = {}                                         # section -> (ref, path)
    for f in git("ls-tree", "--name-only", f"origin/dev:{DIR}", cwd=root, check=False).splitlines():
        m = FILE_RE.match(f)
        if m:
            files[m.group(1)] = ("origin/dev", f"{DIR}/{f}")
    for sec, bs in branches.items():
        for b, ts, changed in sorted(bs, key=lambda x: x[1]):     # the newest wins
            for f in changed:
                m = FILE_RE.match(os.path.basename(f))
                if f.startswith(DIR + "/") and m and m.group(1) == sec:
                    files[sec] = (b, f)
    blobs = _cat(root, [f"{ref}:{path}" for ref, path in files.values()])
    for sec, bs in branches.items():                   # a branch with no section file yet
        if sec not in files:
            files[sec] = (bs[0][0], "")
    steps = []
    for sec in sorted(files, key=lambda s: tuple(int(x) for x in re.findall(r"\d+", s))):
        ref, path = files[sec]
        text = blobs.get(f"{ref}:{path}", "")
        found = parse_steps(text, sec)
        if found:
            steps += found
        elif sec in branches:                          # a section with no table yet
            bs = branches[sec]
            steps.append({
                "id": sec, "section": sec, "implicit": True,
                "title": (re.sub(r"^#+\s*3\.\d+[a-z]?\s*", "", text.split("\n", 1)[0])
                          or bs[0][0])[:70],
                "needs": [], "waits": "", "state": "open",
                "touches": sorted({f for _, _, ch in bs for f in ch}),
            })
    return steps


# ---------- claims ----------

def claims_dir(common: str) -> str:
    d = os.path.join(common, "plan-claims")
    os.makedirs(d, exist_ok=True)
    return d


def read_claims(common: str) -> list:
    d = os.path.join(common, "plan-claims")
    out = []
    for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        if f.endswith(".json"):
            try:
                with open(os.path.join(d, f)) as fh:
                    out.append(json.load(fh))
            except (OSError, ValueError):
                pass
    return out


class _Lock:
    """One flock over every claim change, so check-then-write is atomic."""

    def __init__(self, common: str):
        self.path = os.path.join(claims_dir(common), ".lock")

    def __enter__(self):
        self.fh = open(self.path, "a")
        fcntl.flock(self.fh, fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        fcntl.flock(self.fh, fcntl.LOCK_UN)
        self.fh.close()


def active_branches(branches: dict, now: float) -> dict:
    """{section: [branch, ...]} whose last commit is within ACTIVE_S."""
    return {sec: [b for b, ts, _ in bs if now - ts < ACTIVE_S]
            for sec, bs in branches.items() if any(now - ts < ACTIVE_S for _, ts, _ in bs)}


def assess(step: dict, steps: list, live: list, me, branches: dict, here_branch: str, now: float,
           accepted=None):
    """(status, reason) for one step from this session's point of view.
    status: done | mine | claimed | active | waits | blocked | conflict | free."""
    if step["state"] in CLOSED:
        return step["state"], ""
    held = [c for c in live if c["step"] == step["id"]]
    if held:
        c = held[0]
        if same_owner(c, me):
            return "mine", f"held by this session ({c.get('worktree', '')})"
        return "claimed", f"claimed by pid {c['pid']} in {c.get('worktree', '?')}"
    other = [c for c in live if c.get("section") == step["section"] and not same_owner(c, me)]
    if other:
        c = other[0]
        return "claimed", (f"section {step['section']} is held by pid {c['pid']} ({c['step']}): one "
                           "session per section, which owns its worktree, ports and ROS domain")
    act = active_branches(branches, now)
    mine_here = any(c.get("section") == step["section"] for c in live if same_owner(c, me))
    if step["section"] in act and not mine_here and here_branch not in act[step["section"]]:
        return "active", f"{', '.join(act[step['section']])} worked on in the last 2 h (no claim)"
    own = here_branch in [b for b, _, _ in branches.get(step["section"], [])]
    if step.get("implicit") and step["section"] not in act and not own:
        return "idle", "unmerged branch, idle over 2 h, no step table"
    if step["waits"]:
        return "waits", f"waits on the {step['waits']}"
    state = {s["id"]: s["state"] for s in steps}
    unmet = [n for n in step["needs"] if state.get(n) != "done"]
    if unmet:
        return "blocked", "needs " + ", ".join(unmet)
    for c in live:
        if same_owner(c, me):
            continue
        hits = overlap(step["touches"], c.get("touches", []))
        if hits:
            return "conflict", f"touches {hits[0][0]} -- claimed by {c['step']} (pid {c['pid']})"
    for sec, bs in act.items():
        if (sec == step["section"] or here_branch in bs
                or any(c.get("section") == sec for c in live)):
            continue                               # its own work, or covered by a claim
        files = sorted({f for b, ts, ch in branches[sec] if b in bs for f in ch})
        hits = overlap(step["touches"], files)
        if hits and accepted is not None:
            accepted += [f"{bs[0]}: {y}" for _, y in hits]
        elif hits:
            return "conflict", f"touches {hits[0][1]} -- changed on active {bs[0]} " \
                               "(no claim there: checked by hand? claim --accept-overlap)"
    return "free", ""


class Board:
    def __init__(self, cwd=None, me=None):
        rp = repo(cwd)
        if not rp:
            raise SystemExit("not in a git repository")
        self.top, self.common = rp
        self.now = time.time()
        self.procs = processes()
        self.me = me or owner(self.procs)
        self.here = git("rev-parse", "--abbrev-ref", "HEAD", cwd=self.top, check=False)
        self.branches = section_branches(self.top)
        self.steps = load_steps(self.top, self.branches)
        self.claims = read_claims(self.common)
        self.live = [c for c in self.claims if alive(c, self.procs)]

    def status(self, step, accepted=None):
        return assess(step, self.steps, self.live, self.me, self.branches, self.here, self.now,
                      accepted)

    def find(self, sid):
        return next((s for s in self.steps if s["id"] == sid), None)


# ---------- commands ----------

ORDER = ["mine", "claimed", "active", "free", "conflict", "blocked", "waits", "idle"]


def cmd_board(show_all=False, short=False, cwd=None) -> str:
    b = Board(cwd)
    rows = []
    for s in b.steps:
        st, why = b.status(s)
        if st in CLOSED and not show_all:
            continue
        rows.append((st, s, why))
    rows.sort(key=lambda r: ORDER.index(r[0]) if r[0] in ORDER else 99)
    if short:
        rows = [r for r in rows if r[0] in ("mine", "claimed", "active", "free")]
    lines = []
    for st, s, why in rows:
        lines.append(f"{st:9} {s['id']:10} {s['title'][:60]}" + (f"\n{'':20} {why}" if why else ""))
    if not lines:
        lines = ["no open steps"]
    return "\n".join(lines)


def cmd_next(cwd=None) -> str:
    b = Board(cwd)
    for s in b.steps:
        st, _ = b.status(s)
        if st == "mine":
            return f"this session already holds {s['id']}: {s['title']}"
    for s in b.steps:
        if not s.get("implicit") and b.status(s)[0] == "free":
            return (f"{s['id']}: {s['title']}\n  touches: {', '.join(s['touches']) or '--'}\n"
                    f"  take it: python tools/plan_section.py claim {s['id']}")
    return "no free step: every open step is claimed, active, blocked or waiting (see `board`)"


def claim(sid: str, cwd=None, me=None, accept_overlap=False) -> dict:
    """Take a step for this session, or SystemExit with the reason.
    accept_overlap: a file shared with an ACTIVE BRANCH THAT HOLDS NO CLAIM was
    checked by hand and is fine; it is recorded in the claim. It never
    overrides another session's claim."""
    b = Board(cwd, me)
    if not b.me:
        raise SystemExit("claim: no Claude Code process above this one (set PICAR_CLAIM_PID)")
    s = b.find(sid)
    if not s:
        raise SystemExit(f"claim: no step {sid} (steps are rows of a section's `## Steps` table, "
                         f"committed on origin/dev or its plan/* branch)")
    with _Lock(b.common):
        b.claims = read_claims(b.common)              # re-read under the lock
        b.live = [c for c in b.claims if alive(c, b.procs)]
        accepted = [] if accept_overlap else None
        st, why = b.status(s, accepted)
        if st == "mine":
            return next(c for c in b.live if c["step"] == sid)
        if st != "free":
            raise SystemExit(f"claim refused: {sid} is {st}" + (f" -- {why}" if why else ""))
        c = {"step": sid, "section": s["section"], "title": s["title"],
             "touches": s["touches"], "pid": b.me["pid"], "start": b.me["start"],
             "session": os.environ.get("CLAUDE_SESSION_ID", ""),
             "worktree": b.top, "branch": b.here, "accepted_overlap": accepted or [],
             "claimed_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        path = os.path.join(claims_dir(b.common), f"{sid}.json")
        tmp = f"{path}.{os.getpid()}.tmp"
        with open(tmp, "w") as fh:
            json.dump(c, fh, indent=1)
        os.replace(tmp, path)
    return c


def release(sid=None, cwd=None, me=None) -> list:
    """Drop this session's claims (or one); a dead owner's claim may be dropped by anyone."""
    rp = repo(cwd)
    if not rp:
        raise SystemExit("not in a git repository")
    procs = processes()
    me = me or owner(procs)
    gone = []
    with _Lock(rp[1]):
        for c in read_claims(rp[1]):
            if sid and c["step"] != sid:
                continue
            if same_owner(c, me) or not alive(c, procs):
                os.remove(os.path.join(claims_dir(rp[1]), f"{c['step']}.json"))
                gone.append(c["step"])
            elif sid:
                raise SystemExit(f"release refused: {sid} belongs to live pid {c['pid']}")
    return gone


# ---------- hooks (never fail the tool they guard) ----------

def foreign_claims_on(rel: str, common: str, procs: dict, me) -> list:
    """Live claims of other sessions covering rel, minus those whose owner the
    user let this session edit (`allow`)."""
    return [c for c in read_claims(common)
            if alive(c, procs) and not same_owner(c, me)
            and not any(same_owner(a, me) for a in c.get("allowed", []))
            and any(matches(rel, t) for t in c.get("touches", []))]


def allow(sid: str, cwd=None, me=None) -> dict:
    """The user agreed this session may edit inside another session's claim:
    recorded in that claim, for as long as it lives."""
    rp = repo(cwd)
    if not rp:
        raise SystemExit("not in a git repository")
    procs = processes()
    me = me or owner(procs)
    if not me:
        raise SystemExit("allow: no Claude Code process above this one")
    path = os.path.join(claims_dir(rp[1]), f"{sid}.json")
    with _Lock(rp[1]):
        try:
            c = json.load(open(path))
        except (OSError, ValueError):
            raise SystemExit(f"allow: no claim on {sid}")
        if not alive(c, procs):
            raise SystemExit(f"allow: {sid}'s owner has exited; the claim is free")
        if not any(same_owner(a, me) for a in c.get("allowed", [])):
            c.setdefault("allowed", []).append({**me, "at": time.strftime("%Y-%m-%dT%H:%M:%S")})
        tmp = f"{path}.{os.getpid()}.tmp"
        json.dump(c, open(tmp, "w"), indent=1)
        os.replace(tmp, path)
    return c


# Modes in which Claude Code shows the user an "ask"; in the others (auto,
# bypassPermissions, ...) an "ask" never reaches a person, so the guard denies.
ASK_MODES = ("default", "plan", "acceptEdits")


def hook_edit(stdin_text: str) -> str:
    """PreToolUse on Edit/Write/NotebookEdit: before this session edits a file
    inside another live session's claim, ask the user -- or, in a mode where
    no one would see the question, deny with the reason. '' means no opinion."""
    try:
        data = json.loads(stdin_text)
        ti = data.get("tool_input") or {}
        path = ti.get("file_path") or ti.get("notebook_path")
        if not path:
            return ""
        path = os.path.abspath(os.path.join(data.get("cwd") or os.getcwd(), path))
        d = os.path.dirname(path)
        while d and not os.path.isdir(d):
            d = os.path.dirname(d)
        rp = repo(d)
        if not rp or not os.path.isdir(os.path.join(rp[1], "plan-claims")):
            return ""
        rel = os.path.relpath(os.path.realpath(path), os.path.realpath(rp[0]))
        if rel.startswith(".."):
            return ""
        procs = processes()
        hits = foreign_claims_on(rel, rp[1], procs, owner(procs))
        if not hits:
            return ""
        c = hits[0]
        reason = (f"{rel} is inside step {c['step']} ({c.get('title', '')[:60]}), claimed by "
                  f"another live Claude Code session (pid {c['pid']}, {c.get('worktree', '?')}). "
                  "Editing it here may collide with that session (PARALLEL-SESSIONS.md rule 8).")
        mode = data.get("permission_mode", "")
        if mode in ASK_MODES:
            decision = "ask"
        else:
            decision = "deny"
            reason += (" Ask the user before going on; only if they agree, run "
                       f"`python3 tools/plan_claims.py allow {c['step']}` and retry.")
        return json.dumps({"hookSpecificOutput": {
            "hookEventName": "PreToolUse", "permissionDecision": decision,
            "permissionDecisionReason": reason}})
    except Exception:                                  # a guard must never break editing
        return ""


def hook_commit(cwd=None) -> str:
    """pre-commit: a warning for each staged file inside another live claim."""
    try:
        rp = repo(cwd)
        if not rp or not os.path.isdir(os.path.join(rp[1], "plan-claims")):
            return ""
        staged = git("diff", "--cached", "--name-only", cwd=rp[0], check=False).splitlines()
        procs = processes()
        me = owner(procs)
        out = []
        for f in staged:
            for c in foreign_claims_on(f, rp[1], procs, me):
                out.append(f"pre-commit: WARNING {f} is inside step {c['step']}, claimed by "
                           f"live pid {c['pid']} ({c.get('worktree', '?')})")
        return "\n".join(out)
    except Exception:
        return ""


def hook_start(cwd=None) -> str:
    """The board for a new conversation. A claim belongs to the Claude Code
    PROCESS, so after /clear or a resume in the same terminal the new
    conversation inherits it; say so plainly rather than let it sit unseen."""
    try:
        board = cmd_board(short=True, cwd=cwd)
        b = Board(cwd)
        held = [c for c in b.live if same_owner(c, b.me)]
    except BaseException:
        return ""
    head = "".join(
        f"THIS SESSION ALREADY HOLDS {c['step']} ({c.get('title', '')[:60]}; worktree "
        f"{c.get('worktree', '?')}), claimed by an earlier conversation in this terminal. "
        f"Continue it there, or give it back: `python3 tools/plan_claims.py release {c['step']}`.\n"
        for c in held)
    return (head + "Plan steps (PARALLEL-SESSIONS.md rule 8). Before working on the plan, take a "
            "step: `python tools/plan_section.py next`, then `claim <step>`. Board:\n" + board)


def main(argv):
    if not argv:
        raise SystemExit(__doc__)
    cmd, args = argv[0], argv[1:]
    if cmd == "board":
        print(cmd_board(show_all="--all" in args, short="--short" in args))
    elif cmd == "next":
        print(cmd_next())
    elif cmd == "claim" and len([a for a in args if a != "--accept-overlap"]) == 1:
        c = claim([a for a in args if a != "--accept-overlap"][0],
                  accept_overlap="--accept-overlap" in args)
        print(f"claimed {c['step']} for pid {c['pid']}: {c['title']}\n"
              f"  touches: {', '.join(c['touches']) or '--'}"
              + "".join(f"\n  accepted overlap: {x}" for x in c.get("accepted_overlap", [])))
    elif cmd == "release":
        gone = release(args[0] if args else None)
        print("released " + (", ".join(gone) if gone else "nothing"))
    elif cmd == "allow" and len(args) == 1:
        allow(args[0])
        print(f"this session may now edit inside {args[0]} (recorded in its claim)")
    elif cmd == "hook-edit":
        out = hook_edit(sys.stdin.read())
        if out:
            print(out)
    elif cmd == "hook-commit":
        out = hook_commit()
        if out:
            print(out, file=sys.stderr)
    elif cmd == "hook-start":
        print(hook_start())
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
