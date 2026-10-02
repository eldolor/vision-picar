---
kind: engineering
domain: <domain>
status: current
verified: YYYY-MM-DD
parent: docs/<domain>/ARCHITECTURE.md
---

# <Component> -- engineering

How <component> is built today. The what and why are in the
[architecture spec](../../<domain>/ARCHITECTURE.md); this document is true
only until the implementation changes, and is updated in the same commit
that changes it.

## Implementation

Where it lives: modules, classes, processes, containers, and what each
file does.

## Interfaces

Signatures, routes (method, path, request and response shape, status
codes), message schemas, serial frames, topics.

## Parameters and configuration

A table: config key or constant, default, unit, where it is read, and why
it has that value (with the measurement that set it, if one did).

## Procedures

How to run, operate, calibrate, and recover it: exact commands, expected
output, and what a failure looks like.

## Verification

The tests that pin it (file and what each proves), the ground-truth
sweeps and their recorded numbers, and a checklist for a change.

## Known gaps

What is not built, not measured, or known wrong.
