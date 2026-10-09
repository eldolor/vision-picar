---
name: benchmark-checklist
description: "Vet a measured number from this project (a replay matrix cell, a walk score, a FORWARD or field rate, a latency, a mission's cost or wall-clock) before you report it, write it into CLAUDE.md or a plan doc, or act on it. Use when you run a replay, score a walk, compare models or prompt variants, or report a speedup, regression or rate you measured."
disable-model-invocation: true
---

# Benchmark checklist (vision-picar)

Adapted from the `benchmark-checklist` skill in Cursor's pStack pack
(github.com/cursor/plugins, `pstack/skills/benchmark-checklist`, MIT,
copyright 2026 Lauren Tan). The seven questions are the original's; the
examples, tools and failure history are this project's.

Use this whenever you produce a number someone will act on: a cell in a
model x prompt-variant replay matrix, a walk's eval score, a rate for a
`/navigate` field, a latency or call rate, a mission's cost or wall-clock,
or a sim-vs-hardware claim. Answer each question with evidence from a run,
not from reading the code.

This project has already shipped wrong numbers in exactly these ways. The
33/33/33 replay matrix "showed the wording makes no difference"; the cells
had completed 3, 9 and 2 of 22 frames because the vision service timed out
(CLAUDE.md, Stage 0). Walks were diagnosed for a day as Nova Lite that had
run on Sonnet 4.5, because the template's env var overrode the code
default. Each question below names the check that would have caught it.

For a quick ballpark the user asked for, one run is enough. Still check
questions 4 and 7, and say it is one run. A choice between models, prompt
variants or settings is never a ballpark.

## Before you run anything

- Write the claim you expect to make, in the words you would ship ("Opus
  4.5 / `default` reaches the target in 6-14 frames on walk
  `red-backpack-20260829-195904`"). The questions test that sentence.
- Read the measuring code. For replays that is `control/walk_replay.py` and
  the replay route in `control/admin_server.py`; for scores,
  `control/walk_eval.py`; for missions, `tests/demo_replay_mission.py`;
  for frame-folder spreads, `tests/manual_replay_navigate.py`. Note what
  each counts, what it drops, and what it does on a timeout.
- Note what the vision service is doing. Replays stacked on one vision
  task lose frames; never start a replay while another is still running
  (poll `GET /recording/walks/{walk}/replays` for a fresh `replayed_at`,
  as `control/admin.js` does), and say if anything else was using it.

## The questions

1. **Why not double?** Name the limiter before calling a change useless or
   a win. For a `/navigate` loop it is almost always the model call
   (seconds) rather than anything local; for Robot view it is
   `GUIDANCE_MAX_IN_FLIGHT` (2) times the call latency; for a mission it is
   the step or cost cap. If you overlapped calls and decisions now land
   every ~500 ms, the number of paid calls did not change, only how fast
   the budget is spent: say which one you measured.
2. **Was it tuned?** Run every side the way production runs it. The model
   that ran is whatever the deployed `NavigateModelId` parameter says, not
   the code default, so check the recorded `model_id` on every reply (and
   `walk_eval`'s `model_ids`) rather than any prose. Use the same prompt
   variant, the same `replay_timeout_s`, the same environment (Lab vs
   production) on every side. A comparison where one side hit a cold or
   throttled vision task compared deployments, not models.
3. **Did it break limits?** Do the arithmetic. A walk has a fixed number of
   frames; a rate's denominator is the frames actually answered, not the
   walk length. A 120-call Robot-view cap at two calls in flight is about a
   minute of walking; a result implying more calls than that is counting
   something else. A sim latency far below a real network round trip means
   `sim.realtime` was off and no time passed.
4. **Did it error?** Read a replay's `coverage` first: below
   `REPLAY_MIN_COVERAGE` (0.8) it is stored but deliberately unscored, and
   a score over a third of a walk is a different measurement, not a weaker
   one. Count timeouts, 4xx/5xx and `unknown` field values separately from
   real answers. httpx *raises* on a timeout rather than returning a status
   code, so a retry that only branches on status codes never runs: check
   the error count is not zero by construction.
5. **Does it reproduce?** Model answers vary run to run. For a
   model or variant comparison, replay the same frames, alternate the
   sides, and report the spread. Two live walks are not a controlled
   comparison: they vary the operator's path as well as the model, so only
   a replay can attribute a difference to the model or prompt. Treat a gap
   smaller than the run-to-run variation as no measurable difference.
6. **Does it matter?** Tie the number to the thing the robot has to do.
   A FORWARD rate is only good in a band: 0% is the never-FORWARD stall and
   100% is the drive-into-the-wall mode, so report the collision check next
   to it. A field rate must be physically plausible: someone walking a
   house is not one step from a collision on 60% of frames, so
   `within_one_step` at 60% is miscalibration, not caution. Check the skew
   holds on the frames where the target is not in view, and whether two
   fields are wrong together rather than independently. And a result on
   raycaster frames is a statement about flat-shaded frames, not rooms
   (`sim/renderer.py`'s fidelity note): say which you measured.
7. **Did it even happen?** Confirm the work ran: the reply carried the
   `model_id` and `prompt_variant` you meant (an unknown variant or a
   silently unchanged prompt answers like `default`), every frame you
   count has a reply, and a field you rate was actually asked for (the
   default prompt never asks for `distance_estimate`, so every value is
   `unknown`). For timing, confirm the sim was not sharing a process with
   the thing under test: measure the thing, not its test rig.

## Report

- Lead with the verdict: better, worse, no measurable difference, or
  inconclusive.
- Give the number with its unit, its denominator, the coverage, the run
  count, the spread, the model id and prompt variant, and the limiter. For
  example: "FORWARD rate 0.591 (13/22 frames), coverage 1.00, Opus 4.5 /
  `default`, one replay of walk red-backpack-20260829-195904; Lab vision
  service."
- Call it inconclusive when coverage is under 0.8, when a side ran a
  different model, variant, timeout or environment, when you cannot name
  the limiter, or when you could not check questions 4 and 7. Name the gap.
- Before a number goes into CLAUDE.md or a PLAN-*.md file, it must pass
  questions 4 and 7 at least, and carry its walk name and model id so the
  next session can rerun it.
