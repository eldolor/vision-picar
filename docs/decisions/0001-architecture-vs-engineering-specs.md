---
kind: decision
domain: specs
status: accepted
verified: 2026-10-02
---

# 0001 -- Architecture specs and engineering specs are separate documents

**Decided 2026-10-02 by the user.** Every component of the robot has two
specifications: one that says WHAT it is and WHY, and one that says HOW it
is built today. They live in different directories, follow different rules,
and are checked by a linter (`tools/spec_lint.py`) and a review prompt
(`docs/SPEC_REVIEW_PROMPT.md`).

## Context

The project's documentation grew the way most project documentation does.
Each document started with a purpose and then absorbed every fact that
touched it. `PLAN-onboard-perception.md` is 9,140 lines. `CLAUDE.md`'s
status table puts decisions, part numbers, thresholds, test file names and
measured results in the same cell. A reader cannot tell which sentences are
commitments and which are today's tuning. The cost shows up in
`docs-review/REPORT.md`: most of its verified mismatches are tuning facts
that changed in code while the prose that carried them did not. Some of
them sat in paragraphs whose decision was still true.

The car is about to change underneath every document at once. The Jetson
arrived on 2026-09-30 and the UGV Rover was ordered the same day. Most HOW
facts will move on hardware day: ports, calibration constants, power
figures, which process starts what. The decisions should not move:
safety stays local, the brain is a separate process, ROS stays behind one
wall.

## Decision

| | Architecture spec | Engineering spec |
|---|---|---|
| Lives in | `docs/<domain>/ARCHITECTURE.md` | `docs/engineering/<domain>/ENGINEERING.md` |
| Answers | WHAT and WHY | HOW |
| Content | Components and their boundaries; the decision and the alternatives rejected, with the trade-off; contracts between components (direction, protocol category, ownership); failure modes and resilience targets | Specific parameters, thresholds, sizing and tool configuration; step-by-step procedures, runbooks and validation checklists; interface signatures, schemas, routes and config keys |
| Stays true | Even if the implementation is rewritten | Only until the implementation changes |
| Obligation | None | Must name its parent architecture spec for the what and why |

**The one-line heuristic.** If a competent engineer could implement it in
more than one reasonable way and the document does not care which, it is
architecture. Once the document commits to one specific way, that content
belongs in an engineering spec.

**A worked pair from this repo.** The safety architecture says that every
movement is vetted by a layer on the robot that no driver can bypass. It
also says that a sensor that cannot see never counts as clear and never
counts as blocked, and that a person outranks the brain. It does not say
how. The safety engineering spec says the corridor margin is 3 cm and the
pivot margin is 1.3 cm. It says the wheel loop re-vets at 20 Hz, gives the
`min_distance_cm` key, and names the test that sweeps 5,760 runs against
ground truth. Replace the scan-based corridor with a depth camera and the
first document does not change. That is the test passing.

**Two more, from the status table:**

- "The brain and the robot are two processes" is architecture. Changing it
  changes which failures the watchdog can see.
- "The watchdog timeout is 1.0 s" is engineering. Changing it changes a
  config value and a runbook, not the model.

**Resilience targets are architecture, tuning is not.** "A dead link stops
the motors within one watchdog period" is a target the design commits to.
"The period is 1.0 s" is the tuning that meets it. When a number is the
commitment itself (a bar a phase was accepted against), the architecture
spec may state it. A number that someone is expected to adjust belongs in
the engineering spec.

## Alternatives rejected

- **One document per component, with an "implementation" section at the
  bottom.** This is what the PLAN documents became. The section grows, the
  decisions above it are edited in place when the code changes, and the
  record of why stops being separable from the record of what. Rejected
  because the drift it causes is the failure this decision exists to stop.
- **Generate the engineering half from code (docstrings, OpenAPI).** This
  covers signatures and misses procedures, calibration, runbooks and the
  reasons a threshold has its value. Generated reference is still welcome,
  and an engineering spec may point to it.
- **Keep the PLAN documents as the specification.** They are dated
  experiment logs: criteria written first, then results, then corrections.
  That record is valuable and stays. But a plan describes a phase, not a
  component. The current shape of the safety layer is spread across ten
  phases in `PLAN-ros-alignment.md`. Specs describe the component as it is
  and cite the plan sections as the record of how it got there.

## Consequences

- The `PLAN-*.md` documents, `CLAUDE.md` and the handoffs are unchanged and
  remain the dated history. A spec may cite them. A plan does not have to
  cite a spec.
- A change to code updates the engineering spec in the same commit. A
  change to a decision updates the architecture spec and, if the decision
  is significant, adds a record in `docs/decisions/`.
- The index of domains and the reading path is `docs/README.md`.

## How it is enforced

**Deterministically,** by `tools/spec_lint.py`, run in the suite by
`tests/test_spec_lint.py`. Errors fail the build; warnings are printed.

- **Errors:**
  - Every spec carries front matter naming its kind, domain, status and
    verification date, and lives in the directory its kind requires.
  - An architecture spec or decision record may not contain a code fence
    tagged with a language. The only tags allowed are the diagram tags
    `text` and `mermaid`.
  - An engineering spec must name a parent architecture spec that exists
    and has the same domain. Each domain must have both halves.
  - A repo path written in backticks, and every relative link, must exist.
    This is the mismatch class `docs-review/REPORT.md` found most often.
  - Each spec has its kind's required sections, and every domain is listed
    in `docs/README.md`.
- **Warnings:**
  - An untagged fence in an architecture spec that holds commands or
    config.
  - A config key from `config/robot.yaml`, an environment variable, a CLI
    flag or a call signature with arguments in an architecture spec.
  - An engineering spec too thin to be one.
  - A verification date older than 120 days.

**By judgement,** in `docs/SPEC_REVIEW_PROMPT.md`. It applies the WHAT+WHY
test in both directions: an architecture spec that leaks HOW, and an
engineering spec that only restates the decision without supplying
concrete parameters. It also checks claims against the code, which no
linter can do in general.
