# Spec review prompt

The judgement half of
[decision 0001](decisions/0001-architecture-vs-engineering-specs.md).
`tools/spec_lint.py` already checks everything a script can check: front
matter, location, tagged fences, the parent link, pairing, that paths
exist, required sections and the index. Do not repeat that work. Run the
linter first, then review only what it cannot see.

Run it as `/spec-review [domain ...]` in Claude Code
(`.claude/commands/spec-review.md`), or paste everything below the line
into any session.

---

Review the specifications under `docs/` for the domains given (all of
them if none are given). Do not edit any spec. Read, verify and report
only. Do not run anything that moves hardware, deploys, or spends money.

## Step 1 -- Run the linter

Run `python tools/spec_lint.py`. List its errors and warnings for the
domains in scope. Errors are defects already. Warnings are leads for
Step 2. Read each warning in context and say whether it is a real leak or
a legitimate exception, giving the reason.

## Step 2 -- Apply the WHAT+WHY test in both directions

For each **architecture spec**, read every sentence and ask: *could a
competent engineer build this more than one reasonable way while this
sentence stays true?* If not, the sentence commits to a HOW and belongs in
the engineering spec. Things the linter cannot catch:

- **Numbers in prose.** A threshold, period, margin, size or count stated
  as tuning ("re-vets at 20 Hz") is HOW. A number that IS the commitment,
  such as a resilience target or the bar a phase was accepted against, may
  stay. Name which kind each number is.
- **Technology choices stated as facts with no decision around them.** "It
  runs on Fargate" is HOW. "It runs on the robot, not in the cloud,
  because..." is a decision. A technology may appear in an architecture
  spec only as the subject of a decision with its alternatives.
- **Module, class, file and function names used as the description**
  rather than as a pointer. The architecture should survive a rename.
- **Procedures,** meaning any ordered list of steps a person performs.

For each **engineering spec**, ask the reverse: *does it supply the HOW,
or only restate the decision?* Flag:

- Sections that paraphrase the parent spec instead of linking to it.
- Parameters given without their value, unit, or the place they are read.
- Procedures with no command, no expected output, or no failure signature.
- Rationale that is a decision rather than a tuning reason. "We chose
  Cyclone DDS because Humble's tf2 deadlocked" is a decision; move it to
  the architecture spec. "The margin is 1.3 cm because one guarded turn
  read 0.96 cm" is a tuning reason; it stays.

## Step 3 -- Verify against the code

A spec that reads well but is wrong is worse than a missing one. For every
claim of fact in the engineering specs, and every contract in the
architecture specs, check the source:

- Interfaces, signatures, routes, request and response fields: against
  the code that defines them.
- Config keys and defaults: against `config/robot.yaml` and the code that
  reads them (`control/brain_config.py`, `robot/factory.py`, the
  environment variables each server reads).
- Constants and thresholds: against the module that defines them, and
  against `tests/test_wall_linters.py`'s duplicate registry where they
  cross the ROS wall.
- Measured numbers: against the plan section or evaluation record they
  cite.
- Status words (built, off by default, not deployed, planned): against
  the code and the status table in `CLAUDE.md`.

Give every mismatch a file:line reference on both sides. Mark anything you
established only by reading, and not by a safe command such as `grep`,
`pytest --collect-only` or a dry import, as UNCONFIRMED.

## Step 4 -- Check the pair and the set

- **Pair agreement.** Every decision in the architecture spec that has an
  implementation is reflected in the engineering spec, and nothing in the
  engineering spec contradicts its parent.
- **Boundaries.** Two domains claiming the same state or the same
  decision. Name the owner the architecture specs imply.
- **Coverage.** Code under `robot/`, `world/`, `brain/`, `control/`,
  `sim/`, `service/`, `firmware/`, `tools/` or `web-twin/` that belongs to
  no domain.
- **Duplication.** The same HOW fact maintained in two engineering specs,
  or in a spec and a `PLAN-*.md` as if both were current. Say which copy
  is canonical.

## Output

Write `docs-review/SPEC-REVIEW.md` with these sections:

1. **Summary.** Three sentences, and the single most important defect.
2. **Linter results,** with your verdict on each warning.
3. **HOW in architecture specs.** Quote each line briefly, say why it is
   HOW, and say where it should go.
4. **Restatement in engineering specs,** quoted the same way.
5. **Verification failures:** the spec's claim against the code, with
   file:line on both sides.
6. **Pair, boundary, coverage and duplication findings.**
7. **Fix list.** Order by how badly each defect would mislead someone
   building or operating the robot. Tag each fix S, M or L for effort.
8. **What was checked and found correct,** so nobody re-checks it.

Be specific. "The safety spec is too detailed" is useless. "safety
ARCHITECTURE.md:42 states the 3 cm corridor margin. That is tuning (3.18
chose it, and a depth camera would change it), so move it to engineering
§Parameters" is useful. End with a short summary in chat and the path to
the report.
