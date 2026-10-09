---
name: thermos-bug-pass
description: "Run the Thermos bug pass (a diff-scoped bug, breakage and security audit) on a commit, range or the working tree, next to /code-review. Use on any risky change as CLAUDE.md section 6 'Code review' defines it: stoppable loops with calls in flight, anything that writes recorded or stored data, the safety or motion path, retries/timeouts/paid cloud calls, routes or CloudFormation."
---

# Thermos bug pass

The rubric, the project-specific places to trace, and the evidence for
using it all live in `docs-review/THERMOS-BUG-PASS.md`. That file is the
single source; this skill only runs it.

1. Work out the target from the request: a commit, a range, a branch
   (`git diff <base>...<branch>`), or the working-tree diff (`git diff
   HEAD`). Default to the working-tree diff.
2. Start `/code-review` on the same target if it is not already running.
   The bug pass is the second reviewer, never a replacement.
3. Launch one background subagent (`Agent`, `subagent_type:
   general-purpose`, `run_in_background: true`) with the prompt given in
   that file's "How to run it" section, with the target filled in.
4. When both have reported, merge their findings: dedupe, mark a bug both
   found as confirmed, and check any finding only one of them raised
   against the code before acting on it. Lead with the confirmed ones.
