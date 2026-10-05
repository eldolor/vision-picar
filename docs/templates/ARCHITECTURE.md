---
kind: architecture
domain: <domain>
status: current            # current | draft | planned | superseded
verified: YYYY-MM-DD       # the date the claims below were checked against the code
---

# <Component> -- architecture

One paragraph: what this component is, the problem it solves, and when to
read this rather than the engineering spec. Link the engineering spec:
[engineering spec](../engineering/<domain>/ENGINEERING.md).

## Purpose

What it is for, who depends on it, and what would break without it.

## Components and boundaries

The parts, what each owns, and the line it may not cross (what it must
never import, call, or know about). A diagram in an untagged or `text`
fence is welcome; code and config are not.

## Decisions

One subsection per decision. For each: the decision, the alternatives
rejected, and the trade-off accepted. Cite the dated record (a PLAN-*.md
section, a docs/decisions/ entry) as the history.

## Contracts

Between this component and each neighbour: direction (who calls whom),
protocol category (in-process interface, HTTP/JSON, serial, ROS topic),
and ownership (who owns the state, who may write it). Name the contract,
not its signature.

## Failure modes and resilience targets

What fails, what the component does about it, and the target it commits
to ("a dead link stops the motors within one watchdog period"). The number
that tunes it belongs in the engineering spec unless the number IS the
commitment.

## Open questions

Undecided points, with who decides and what evidence would settle them.
