# Thermos bug pass

The second reviewer for risky changes. CLAUDE.md section 6 ("Code review")
says when to run it; this file is what to run.

Adapted from the `thermo-nuclear-review` skill in Cursor's Thermos plugin
(github.com/cursor/plugins, `thermos/skills/thermo-nuclear-review`, MIT,
copyright 2026 Cursor). The rubric below is theirs; the "how to run it"
and "trace these here" parts are this project's. Thermos also ships a code
quality pass; it is deliberately not used here (see "Why this and not the
rest of Thermos").

## How to run it

Run it next to `/code-review`, not instead of it, and on the same target.
Launch one background subagent (`Agent`, `subagent_type:
general-purpose`, `run_in_background: true`) with this prompt, filling in
the target:

> You are a read-only reviewer: never edit, commit, push or post anything.
> Read `docs-review/THERMOS-BUG-PASS.md` and follow its "Rubric" and
> "Trace these here" sections exactly. Target: `<commit, range, or
> "the working-tree diff">` in `/home/user/vision-picar`. Get the diff with
> `git show <commit>` / `git diff <range>` / `git diff HEAD`, and read the
> changed files at that revision. Return a numbered list of findings,
> highest priority first, each with severity, file:line evidence and a
> concrete failure scenario, then what you could not verify. Under ~700
> words.

Then merge its findings with `/code-review`'s: dedupe, and treat a bug
both found as confirmed. A finding only one of them raised is a claim to
check before acting on it, not an order.

## Rubric

You are a security expert performing a comprehensive review of a change.
Audit it extremely thoroughly for bugs, changes that break existing
features or functionality, and security vulnerabilities. Be extremely
thorough, rigorous, careful, ambitious and attentive. Nothing can slip
through.

**Scope.** Only report issues in code the change ADDS or MODIFIES. Do not
report problems in existing code the change does not touch.

**Breaking functionality.** This codebase has many cross-module
dependencies; a simple change in one place often breaks something
elsewhere. Trace the side effects of every change thoroughly.

**Breaking devex.** Catch changes that alter how developers run or build
the code: how or where secrets are read, environment variable names or new
required variables, ports and networking, new scripts that must be run for
existing functionality to keep working. A new optional way to run
something, or an ordinary new dependency, does not count.

**Feature leaks.** Features gated behind a flag or an internal-only check
must not leak. These leaks are often subtle.

**Intended breakage.** If a high-risk finding is the evident purpose of the
change and its scope is well constrained, do not report it, unless the
author seems unaware of the full implications or the change looks
malicious.

**Over-reporting.** Never mark an issue higher priority than it is;
reviewers who cry wolf stop being read. Trace each issue end to end until
you are confident before reporting it.

**Critical rules.**
- Never present an issue with unfinished research ("X breaks unless the
  server handles it" when the server code is right there). Go and check.
- If the change has a pull request and you have medium-or-higher findings,
  read its review discussion (GitHub MCP tools; `gh` is not available
  here) only AFTER your own audit, so you review with fresh eyes. Validate
  and dedupe what others found, and say which findings came from them.

## Trace these here

Where this project's real bugs have come from, so where to look first:

- **Stop, restart and late replies.** A call still in flight when a loop
  is stopped, paused or restarted: counters reset under it, a stale reply
  rendered or recorded into the next session, a timer nobody reschedules.
  (`web-twin/app.js`'s guidance loop, `MissionRunner`, `brain_server`.)
- **Anything that writes recorded data.** Frame numbering, overwrites,
  per-key files that a re-run replaces, schema bumps that leave stored
  results unrescored. A wrong recording is worse than a missing one: it is
  scored as if it were real.
- **What the person actually sees.** Follow a result to the screen: a
  status that is set but never drawn, an action shown that was vetoed, a
  toast that contradicts the status line.
- **Routing.** Every public path needs an exact ALB pattern
  (`tests/test_alb_routes.py`); a relative fetch from one service can land
  on another service's route.
- **Retries and timeouts.** A retried paid call the server is still
  billing; a client deadline equal to the load balancer's; a retry that
  never fires because the client raises instead of returning a status.
- **The safety path.** Anything that can move the robot must still pass
  `robot/safety.py`; anything that stops it must be fail-safe.

## Why this and not the rest of Thermos

Measured on six commits (`501d874`, `f5d92c8`, `c43a1e0`, `69c773c`,
`6f37f83`, `7a00165`), with every finding judged blind against the code.
There were 18 real bugs. `/code-review` found 17, in 52 findings, 81% of
which held up. This bug pass found 15, in 32 findings, 91% of which held
up, including the one bug `/code-review` missed: duplicate recorded-frame
numbers on `f5d92c8`, which shipped and was fixed later in `dbfd34b`.
Together they found all 18. Thermos's code-quality pass added no bug
`/code-review` had not found, and about half of its unique findings held
up, so it is not part of the process.
