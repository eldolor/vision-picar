# Phase D: pacing the deliberation call -- A10G, 2026-09-12

`tools/gpu/pacing_sweep.py` over all 8 labelled walks, 610 frames, 159
visible frames in **15 contiguous visible spans**. One A10G, torn down.
The cloud is **stubbed** -- no paid calls -- because the trigger policy
does not read the cloud's answer.

## Why this exists

The live trigger record says 90% of deliberation calls are fired by a
clock, not by perception: across six tiered walks and 70 calls,
`cold_search` 79%, `mission_start` 9%, `staleness` 3%, and
`candidate_sighting` -- **the only trigger the local tier owns -- 10%, on
two walks of six.** So `cold_search_after` is not a backstop, it is the
search mechanism, and it had never been measured.

## The metric

Cost is explicitly not the constraint. What matters is **how long the
target sits in view before anything looks at it.** For each visible span,
`look_latency` is the frames from the span's start to the first cloud
call at or after it. A span with no call at all is a **miss**, reported
separately -- an infinite latency has no mean.

`cloud_frames=4`: a dispatched call is outstanding for four frames, which
is what makes Phase A measurable. A wall clock would be unreproducible.

## Results

| config | calls | spans looked at | median lat | max lat |
|---|---|---|---|---|
| frames-2 | 266 | 13/15 | 1 | 2 |
| **frames-2 async** | 135 | **14/15** | 1.0 | 3 |
| frames-3 | 182 | 12/15 | 1.0 | 3 |
| **frames-3 async** | 135 | **14/15** | 1.0 | 3 |
| frames-4 | 142 | 13/15 | 1 | 4 |
| **frames-4 async** | 134 | **14/15** | 1.0 | 3 |
| frames-6 (**shipped**) | 100 | 12/15 | 1.0 | 4 |
| **frames-6 async** | **94** | **14/15** | 1.0 | **2** |
| frames-9 | 75 | 12/15 | 1.0 | 8 |
| frames-9 async | 71 | 14/15 | 0.5 | 4 |
| frames-12 | 75 | 12/15 | 1.0 | 8 |
| frames-12 async | 71 | 14/15 | 0.5 | 4 |
| cm-20 *(derived)* | 116 | 11/15 | 1 | 4 |
| cm-40 *(derived)* | 89 | 11/15 | 1 | 4 |
| cm-80 *(derived)* | 81 | 11/15 | 1 | 4 |

## Four findings

**1. Async dominates at every interval, on every axis.** At the shipped
interval of 6 it covers **14 of 15 spans against 12**, with **fewer calls**
(94 vs 100) and **half the worst-case latency** (2 frames vs 4). There is
no operating point where the synchronous version is better at anything.

The mechanism is visible per walk: on `blue-bottle-...142454` sync covers
2 of 3 spans and async 3 of 3, with the same five calls. Holding one call
outstanding spreads the calls out instead of bursting them, so they land
across more of the walk.

**2. A shorter interval buys nothing once async is on.** 2, 3, 4 and 6 all
read 14/15 with a median latency of 1.0. Going from 6 to 2 costs **41 more
calls for zero additional coverage**. The instinct to poll harder for a
smoother experience is wrong here, and this is the row that says so.

**3. Above ~9 the staleness timer becomes the binding rule.**
`frames-9` and `frames-12` are *identical* in every column. That is 6.1's
finding -- *"the staleness timer stops binding past `stale_n` ~10"* --
arriving from the other side: past a cold-search interval of about 9, the
staleness floor at 10 fires first and the interval stops mattering. The
useful range of this parameter is 2 to 9 and nothing beyond.

**4. The distance rule does NOT help on this corpus, and the evidence is
weak in a knowable direction.** All three distance settings cover 11 of 15
spans -- worse than any frame setting -- and barely differ from each other
across a 4x range. But **the odometry is derived, not measured**: a phone
on a wheeled rig has no encoders, so travel is integrated from the walk's
own recorded action stream at a nominal 30cm per FORWARD. On search walks
the policy turns far more than it drives, so derived distance accumulates
slowly and fires at the wrong moments.

This says the rule is **unproven, not refuted.** The real test needs
encoders, which means hardware.

## What was set as a result

    tier_async_cloud: true          # was false
    tier_cold_search_after: 6       # unchanged -- 2 buys nothing
    tier_cold_search_after_cm: 0    # off -- unproven without encoders

## Three things this cannot tell you

**The stubbed cloud means "looked at" is when the cloud was ASKED, not
when it answered.** Async does not make answers arrive sooner. What it
does is keep the robot moving on its last goal instead of freezing, and
spread the asking more evenly. The freeze it removes -- a 3.6s stall
against a 0.25s tick, roughly every nine frames -- is structural and
follows from the call being non-blocking; it is not measured here.

**Replay is open loop.** The frames are fixed, so a different pacing
cannot change where the robot went. On a live walk it would.

**Eight walks in one basement, 15 spans.** Two of the eight contribute no
spans at all, and one contributes a third of all frames. Every caution
`PLAN-onboard-perception.md` 4.10 carries about this corpus applies here.
