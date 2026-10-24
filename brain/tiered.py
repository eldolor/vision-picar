"""
tiered.py

The trigger discipline, as a `vision_fn`. Phase **P2**.

`brain/perceive.py` (P1) answers *is the target in this frame*. This
decides **whether that is worth a cloud call**, which is where
`PLAN-onboard-perception.md` section 2.4 says the entire cost saving
lives: deliberation must be *event-driven, never periodic*.

The result is 2.8's mission, minus the wheels: the detector and CLIP look
at every frame for free, and Opus 4.5 is asked only at the moments a
person would ask a colleague. It plugs into `MissionRunner`'s existing
`vision_fn(frame) -> scene` seam, so nothing in `control/` learns that
perception got a new tier -- 2.6's first invariant, kept.

## The triggers this can actually fire

2.4 lists seven. Three of them belong to a reactive tier that holds and
executes a goal (C6) and to a planner that issues one (C8), neither of
which exists, so firing them here would be theatre. The four that are
real without either -- the three below, and `staleness` (added 2026-09-07,
see `DEFAULT_STALE_AFTER`) -- plus one paid call that is not a trigger:
`arrival_confirmation`, the identity check at arrival (handoff 1a,
`confirm_arrival()`).

| Trigger | When | Why it is honest here |
|---|---|---|
| `mission_start` | first frame | 2.5: blocking only under a synchronous tier; under `async_cloud` the one blocking call is the arrival confirmation |
| `candidate_sighting` | perception says `detected` | 2.4's *"the one that does real work"* -- on-board proposes, the cloud confirms identity and **reachability** |
| `cold_search` | `cold_search_after` `absent` frames since the last answer landed (a `detected` frame does not reset the count) | 2.4's inverse: the cloud proposes, on-board tracks. Without it a target outside COCO's 80 never fires anything and the robot can drive past it indefinitely |

`goal_achieved`, `goal_impossible` and `room_change` are
deliberately absent. They are listed in `UNAVAILABLE_TRIGGERS` so the gap
is legible rather than looking like an oversight.

## Hysteresis is not tuning -- it is what makes the discipline work

Section 6.1 measured this and it is the finding most likely to be skipped:
firing on every raw edge gives a **2.8x** saving; requiring two
consecutive frames of agreement first gives **4.1x**. `target_visible`
flips on 24.1% of frames. So roughly **40% of a naive trigger count is
field noise rather than events**, and a trigger policy is only as stable
as the fields it triggers on.

`consecutive_frames` is that requirement, defaulting to 2. Setting it to 1
is how you reproduce the naive count, which is worth doing once on real
frames to see the difference for yourself.

## What happens on the frames that do not fire

Something has to come back, because `MissionRunner` acts on every tick.
**This is a stand-in for the reactive tier, and it is labelled as one** --
a scan step, the same thing 2.4's cold-search description assumes the
robot is doing while it finds nothing. It is not a policy and must not
grow into one: holding and executing a goal is C6's job, and doing it here
would be building the reactive tier in the wrong module.

The stand-in exists so the loop closes and so the **call counter** means
something. That counter is 6.3's own definition of what makes this
architecture watchable: *"a deliberation-call counter that visibly does
not climb every step. That single number makes the whole architecture
watchable."*

## What it costs to run

Perception is free and local. Every cloud call is real money against the
deployed `/navigate`, so `stats()` reports frames, calls and the ratio,
and `max_calls` is a hard stop -- the same shape as Robot view's 120-call
cap, for the same reason.
"""

from __future__ import annotations

import functools
import logging
import threading
import math
import numbers
import random
import re
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Callable, Optional

from brain.arrival import run_spans
from brain.goal_pose import GoalPose, OdomTracker, Pose, _wrap
from brain.tree import SUCCESS, Action, Condition, Decorator, Fallback, Node, Sequence
from brain.perceive import (ABSENT, DEFAULT_MATCH_PROBABILITY, DETECTED, UNAVAILABLE,
                            Perception)

logger = logging.getLogger("tiered")


class _ReleasedPipeline:
    """What an ended mission's policy holds instead of its pipeline (3.81.E),
    so the mission's YOLOE and CLIP are freed before the next mission builds
    its own. A frame that still arrives sees nothing, honestly."""

    def perceive(self, frame: dict) -> Perception:
        return Perception(status=UNAVAILABLE, reason="the mission has ended")


RELEASED_PIPELINE = _ReleasedPipeline()

TRIGGER_START = "mission_start"
TRIGGER_CANDIDATE = "candidate_sighting"
TRIGGER_COLD_SEARCH = "cold_search"
# Handoff 2026-10-02 1a: not a trigger the policy fires on its own, but a
# paid call all the same, so it is counted beside them. `brain/agent.py`
# asks for it once per arrival, through `confirm_arrival()`.
TRIGGER_ARRIVAL = "arrival_confirmation"
# 3.80 (approved by the user 2026-10-14): a local score at or above YES_AT
# confirms; at or below `tier_local_no_below` it refuses; between, or no
# score at all, is unable, and `tier_local_max_unable` of those in one
# arrival hand it to the cloud.
LOCAL_YES_AT = 0.5
DEFAULT_LOCAL_NO_BELOW = 0.1
DEFAULT_LOCAL_MAX_UNABLE = 2
# Frames off the arrival rule, in a row, that end an arrival for the local
# tier's count: ArrivalCheck's own 2 frames plus one.
LOCAL_RESET_FRAMES = 3

# 3.80.D: the two questions only the local tier is asked first. A candidate
# whose CLIP P is in [`tier_local_band_low`, the match gate) on
# `consecutive_frames` frames in a row; and an out-of-vocabulary target
# after `consecutive_frames` absent frames (a local question costs no money,
# so it is asked about as often as the VLM can answer).
TRIGGER_BAND = "uncertain_band"
TRIGGER_OOV = "out_of_vocabulary"
# 3.80.O: explore is closing in on a target only the local VLM has seen, so
# the local VLM is asked on every frame it is free (its box steers the last
# half metre and judges the arrival). Local only: never handed to the cloud.
TRIGGER_TRACK = "local_track"
# 3.91.C (the user, 2026-10-23/24): the tiers disagree -- the local VLM said
# no where it cannot tell (`no_box_range`) and perception still passes the
# candidate, or said yes about a candidate the cloud refused. Asked of the
# cloud, once per candidate run.
TRIGGER_DISAGREE = "disagreement"

# 3.91.C, the user's ordering (2026-10-24): each tier outranks the one below
# -- YOLOE + CLIP < the local VLM < the cloud -- and only the robot server's
# safety layer overrides them all. An answer about a candidate is never undone
# by a lower tier's: a cloud no is lifted only by a cloud yes (or the
# candidate changing), a cloud yes stands against a local no, and where the
# two VLMs disagree the cloud is asked. Perception acts only where neither
# VLM has spoken about the candidate.
TIER_RANK = {"local": 1, "cloud": 2}

# Outside the trees, by design: no tree node may reorder or tune them away.
HARD_RULES = (
    "safety: robot/safety.py vetoes every move; only it overrides the tiers",
    "a person's Stop is the last word over autonomy (3.84)",
    "the tiers' order: perception < local VLM < cloud -- a lower tier never "
    "undoes a higher tier's answer about a candidate (3.91.C)",
    "an arrival the local VLM's box led to is confirmed only by the cloud (3.80.O)",
    "arrival is the lidar's rule, then the identity check",
    "B3: the watchdog, the runner's vision timeout and failure budget, the tick guard",
    "who to ask: the guards' order (call cap, one call at a time)",
)

# 3.91.C: a "no" about a DETECTED candidate is remembered (B6: Qwen said no
# on 18 of 18 taupe sightings and perception still steered at it on every
# later frame). While remembered, a frame showing the candidate neither
# steers at it nor asks about it, until the candidate changes: on a body
# with odometry it moved `reject_move_m` or turned `reject_turn_deg` since
# the asked frame; without (a phone) the candidate was lost for
# REJECT_LOST_FRAMES frames and found again, or `reject_hold_s` passed. Any
# yes releases it. A local no counts only when perception's box covered
# `no_box_range` of the frame: under it is the far view (B5's 1.0 m hold,
# 11%; B6's false no at seq 19), over it fabric only (~0.3 m). The numbers
# are the proposal's, approved as written.
#
# Amendment 3 (the user, 2026-10-23): a no released by loss or time puts the
# candidate on PROBATION -- asked about again, not steered at until a yes (or,
# amendment 4, a no about the far view, which rule 2 does not remember).
# Without it the replay of the taupe walk reversed 7 times: perception's
# close-range blind spot loses the same bag for 3+ frames, and every re-found
# bag was steered at for ~2 frames before the new no landed.
PROBATION_RELEASES = ("lost", "timeout")
DEFAULT_REMEMBER_NO = True
DEFAULT_REJECT_HOLD_S = 20.0
DEFAULT_REJECT_MOVE_M = 0.5
DEFAULT_REJECT_TURN_DEG = 30.0
REJECT_LOST_FRAMES = 3
DEFAULT_NO_BOX_RANGE = (0.15, 0.95)


def candidate_refused(scene: dict) -> bool:
    """3.91.C, the tiers' order: the scene's detected candidate is one a VLM
    refused (a held no, or probation). A reader of `_perception` -- explore's
    sightings and close-in -- treats it as no detection."""
    return bool(((scene or {}).get("_tier") or {}).get("candidate_refused"))


def located_local_bearing(scene: dict):
    """3.80.O: the body bearing of the local VLM's located yes landed on this
    frame (`_tier.local_landed`), else None. One reader for the arrival rule
    and explore."""
    landed = ((scene or {}).get("_tier") or {}).get("local_landed") or {}
    if landed.get("answer") == "yes" and landed.get("bearing_deg") is not None:
        return float(landed["bearing_deg"])
    return None


def local_said_no(scene: dict) -> bool:
    landed = ((scene or {}).get("_tier") or {}).get("local_landed") or {}
    return landed.get("answer") == "no"
DEFAULT_LOCAL_BAND_LOW = 0.5
# The field of view a box's bearing is read in, when the pipeline names none
# (perceive.py's Camera Module 3 default).
DEFAULT_HFOV_DEG = 66.0

# 2.4's other four. Named, not implemented -- each needs a tier that does
# not exist yet, and a trigger that cannot fire is worse than one that is
# absent, because it looks like coverage.
TRIGGER_STALE = "staleness"
# 3.80.F: the questions that ask WHERE TO LOOK, not whether a candidate is
# the target. Put to the local tier, a reply that is not a yes is followed by
# the direction question about the same frame.
SEARCH_TRIGGERS = (TRIGGER_START, TRIGGER_COLD_SEARCH, TRIGGER_STALE)
# The local VLM's one-word answer, as the action the policy holds.
DIRECTION_ACTIONS = {"left": "LEFT", "right": "RIGHT", "forward": "FORWARD"}
# A direction reply that says there is no direction to be had -- the service
# predates 3.80.F (404), or went away between the two questions. Neither
# answers nor fails the search question: the policy behaves as 3.80.D.
DIRECTION_NONE = ("absent", "unsupported")

UNAVAILABLE_TRIGGERS = {
    "goal_achieved": "needs C6 -- nothing holds a goal to achieve",
    "goal_impossible": "needs C6, and the lidar for the boxed-in case",
    "room_change": "needs the lidar -- 1.13 makes the transition geometric",
}

# 2.4's staleness trigger. This module used to list it as unavailable --
# "needs a goal with an age (C4/C6)" -- and that reading was too strict:
# the CALL has an age, and that is enough.
#
# **Implemented 2026-09-07 because a real run found the hole.** Once
# perception got good (the open-vocabulary crop path, two rig walks), it
# reported `detected` on 23 of 38 and 32 of 43 frames -- and the mission
# then never finished. The reason is worth stating plainly, because it is
# a property of the whole architecture and not of this file:
#
#   * `candidate_sighting` fires on the EDGE into `detected`, once.
#   * `cold_search` fires only after a run of `absent`.
#   * So a robot that can see its target continuously fires nothing after
#     the first frame -- and **arrival is the VLM's call** (1.11: identity
#     and reachability belong to the cloud), so it is never made.
#
# Better perception starved the trigger policy. Both walks went from
# `found` to `max_steps` on exactly this. A deliberation tier that can be
# silenced by things going well is not event-driven, it is edge-driven,
# and 2.4's own list already had the fix in it.
#
# 6.1 measured that this timer is NOT the cost driver -- past stale_n ~10
# it stops binding entirely, contributing 5.7% of triggers at 8 -- so the
# floor it puts under the call rate is cheap. 8 is 6.1's own figure.
DEFAULT_STALE_AFTER = 8

# Section 6.1: two consecutive frames of agreement before an edge is
# believed. One reproduces the naive count; three buys 4.5x against 4.1x,
# at the cost of a frame's lag on a real sighting.
DEFAULT_CONSECUTIVE = 2
# 3.90 A: the span in seconds a `consecutive_frames` run must also cover when
# its frames carry capture stamps (`captured_at`): today's two frames at the
# brain's 0.25 s tick. Counted in frames alone, the rule would shrink as the
# camera got faster (R1c). Unstamped frames (the simulator's) count as before.
DEFAULT_SIGHTING_SPAN_S = 0.25
# 3.90 E: "combine frames" -- `candidate_sighting` fires on k detected of the
# last n frames perception took, inside n ticks of time, instead of
# `consecutive_frames` in a row. Off by default: a switch for walks to
# compare, adopted only on the Rover's data (3.90 C, D).
FRAME_VOTES = {"off": None, "2of3": (2, 3), "3of5": (3, 5)}
VOTE_TICK_S = 0.25


def parse_frame_vote(value) -> Optional[tuple]:
    """"off" / "2of3" / "3of5" (or None, "") -> None or (k, n)."""
    if value in (None, ""):
        return None
    if value not in FRAME_VOTES:
        raise ValueError(f"frame_vote must be one of {sorted(FRAME_VOTES)}, got {value!r}")
    return FRAME_VOTES[value]

# How many consecutive `absent` frames before asking the cloud whether the
# thing is even in this room. 2.4 gives no number; this one is a guess and
# is the first thing to tune against a real walk.
DEFAULT_COLD_SEARCH_AFTER = 6

# Phase C. How much NEW GROUND may be covered, in centimetres, before the
# cloud is asked to look again while the local tier is seeing nothing.
#
# **Why distance rather than frames.** Frame count is a proxy for ground
# covered, and the robot now has the real quantity (`get_odometry()`).
# The proxy breaks at both ends: a robot stopped while the operator reads
# the log burns calls for no new information, and a robot at 0.5 m/s under
# 1.14's continuous motion travels a metre between looks at the same frame
# count. Distance couples the poll interval to the thing that actually
# changes the view.
#
# **Turning is not travel**, and that is deliberate rather than an
# oversight -- `get_odometry()` reports a pivot as zero path. A robot
# scanning in place reveals new *view* without new *ground*, so on
# distance alone a scan would never re-trigger. The frame floor below is
# what covers that case, and it is why the two rules are an OR rather than
# a replacement.
#
# **None disables it**, which is the shipped default until Phase D
# measures a number. `PLAN-onboard-perception.md`'s own lesson, recorded
# twice: do not move a default on one walk.
DEFAULT_COLD_SEARCH_AFTER_CM = None

# The scan step the stand-in emits. RIGHT rather than LEFT for no better
# reason than that a consistent direction makes an oscillation obvious in
# the log, which is what `control/walk_eval.py` looks for.
SCAN_ACTION = "RIGHT"

# Phase F (2026-09-12), from five real rig walks. Every one of them was
# dominated by RIGHT -- 69 to 107 turns out of ~120 actions -- and on the
# one walk where BOTH tiers saw the target (perception 15 frames, cloud 30)
# the mission still did not converge. The cause is not arbitration and not
# recognition: `_local_scene` emitted SCAN_ACTION on every free frame, and
# with `cold_search_after` at 6 that is five frames in six. **The stand-in
# outvoted the cloud 5:1**, whatever either tier had just seen.
#
# So the stand-in stops scanning by default and keeps doing what the cloud
# last said. This is 2.5's own text -- *"the reactive tier ALWAYS holds a
# current goal"* -- which Phase A scoped to in-flight calls only because
# nothing had measured it. These walks are that measurement.
DEFAULT_HOLD_GOAL = True

# And a floor under it: after this many QUARTER TURNS' WORTH of consecutive
# turning (8 = 720 degrees, two rotations) with the target not detected,
# emit one FORWARD instead. Counted in DEGREES since R1c: it counted TURNS,
# and when R1b halved the search step to 45 degrees the same 8 silently
# became ONE rotation -- so a single missed frame as the sweep passed the
# target forced a FORWARD off the doorway's line, and a detector that sees
# the target 90% of the time lost one mission in six (84.5% arrived; 95.2%
# with two rotations restored). Degrees keep the meaning fixed whatever the
# step size. A held goal of RIGHT repeated
# forever is the same spin by another route, and every one of the five
# walks would have tripped this. 0 disables.
#
# The collar still owns safety -- robot/safety.py re-checks clearance
# before any FORWARD -- so this can propose a move it is not allowed to
# make, and that refusal is the correct outcome rather than a bug.
DEFAULT_SPIN_GUARD_AFTER = 8

TURN_ACTIONS = ("LEFT", "RIGHT")

# Phase G (2026-09-12). **Whoever can see the target, steers.**
#
# Measured on the newest rig walk: on the 37 frames where the local tier
# reported the basket DETECTED, the robot went RIGHT 28 times and FORWARD
# 4. Detection had no effect on motion. `_direction_for()` computes a
# bearing off the box and writes it to `_navigate.target_direction`, and
# `safest_direction` was still the held cloud goal or a scan -- so the
# bearing was reported and never steered. `target_reached` fired 0 times
# in any walk.
#
# This is 1.11's arbitration split finally honoured rather than argued:
# the on-board tier says WHERE (it has a bearing and the cloud does not),
# the cloud says WHAT. Until now the cloud did both and perception did
# neither.
#
# It also dissolves the "give up on local after N frames and switch to the
# cloud" question, which has no good threshold: when perception cannot see
# the target the cloud is already driving, because there is no local
# bearing to steer on. No mode, nothing to get stuck in.
#
# **The risk is a local false positive steering at the wrong object.**
# Bounded only by the P>=0.8 gate: ONE frame that passes it steers. The
# two-frame hysteresis gates the cloud triggers, not steering, and under
# `async_cloud` a landed cloud answer never overrides a local sighting, so a
# wrong lock-on is not corrected by the next paid call either. On this
# corpus the local tier reads 26% and 68% at zero false positives on the
# basket walks, so it WILL sometimes steer at a handbag. That is
# recoverable; what must not happen is ending `found` there, and that is
# guarded at ARRIVAL instead (handoff 2026-10-02 1a): `confirm_arrival()`
# puts the arrival frame to the cloud, and without its yes there is no
# `found`. Not steering at all is not recoverable.
DEFAULT_STEER_ON_SIGHT = True

# P25 / P7c item 2 -- dead-reckon the bearing to a sighting the detector is
# no longer seeing, instead of falling back to a cloud goal several seconds
# old.
#
# **Default OFF, and that is deliberate.** It changes what the robot does on
# every frame the detector misses, and there is no live evidence yet that it
# helps -- only the measurement that the thing it targets is real
# (`walk_eval.median_command_run` is 1 on three of six rig walks). Flipping a
# default on an argument rather than a number is the NavigateModelId mistake
# this project has already made once. The way to earn the flip is to run it
# both ways and show the run length rise.
DEFAULT_HOLD_BEARING = False
# How far the robot may travel after a sighting before the anchor is
# dropped. A DIRECTION-only anchor (no range, which is every monocular
# frame) is exact under rotation and wrong under translation, so the bound
# is really about translation rather than time. One metre is roughly a
# room-crossing third and well inside where a direction stays useful; it is
# a starting value to be measured, not a derived one.
DEFAULT_HOLD_BEARING_MAX_M = 1.0


# -- 1.11a: corroborated identity, REPORTED ONLY -------------------------
#
# `PLAN-onboard-perception.md` 1.11a is **proposed and not decided**, and
# nothing below changes a single decision. It computes the verdict, counts
# it and publishes it, so the next rig walks measure it live instead of by
# replay -- which is the one thing a replay cannot give it, because 1.11a's
# own falsifier is a walk that does not exist yet.
#
# The rule it would apply, if it were applied:
#
#   * the VLM's **negative** is trusted (10/10 recall across the corpus,
#     never a missed sighting), so only a positive claim needs support;
#   * corroboration uses a LOWER bar than detection -- `P >= 0.8` asks *is
#     this a sighting on its own evidence*, corroboration asks *is the
#     local tier seeing anything consistent with a claim the cloud already
#     made*, and the second deserves the lower bar because the VLM has
#     already contributed evidence. At 0.8 corroboration keeps 2 of 10 true
#     sightings and is unusable; at 0.5 it keeps 8 and still rejects 34 of
#     34 storage-bin claims;
#   * disagreement is `unclear` -- M3's tri-state argument one tier up. Read
#     as absent it discards a real sighting; read as present it keeps the
#     confabulation. An `unclear` sighting may steer and may not commit.
#
# **The third bullet is the behaviour, and it is NOT implemented.** No
# `target_visible` is rewritten, no `target_reached` is suppressed, no
# sighting is withheld from `MissionMemory`. 1.11a asks for two more
# searches on out-of-vocabulary targets first, because a target the local
# tier cannot see at all would make every sighting `unclear` and leave the
# robot steering forever without committing -- strictly worse than
# believing a VLM that is right most of the time.
#
# Costs nothing: `_annotate()` already holds the `Perception` beside the
# cloud's answer, so the verdict is a comparison between two numbers that
# have both already been computed.
DEFAULT_CORROBORATION_P = 0.5

# The four verdicts. `UNCLEAR` is the one 1.11a is about -- and note it is
# a **relationship between two tiers**, not a fourth perception state, which
# is why it lives on `_tier` and not on `Perception.status`.
CORROBORATED = "corroborated"
UNCLEAR = "unclear"
NO_CLAIM = "no_claim"            # the cloud says the target is not visible
CORROBORATION_UNAVAILABLE = "unavailable"   # the local tier could not tell
CORROBORATION_VERDICTS = (CORROBORATED, UNCLEAR, NO_CLAIM,
                          CORROBORATION_UNAVAILABLE)


def corroboration_for(scene: dict, perception: Perception,
                      bar: float = DEFAULT_CORROBORATION_P) -> dict:
    """Does the local tier see anything consistent with the cloud's claim?

    Pure, and reporting-only: it reads both answers and returns a verdict.
    Nothing acts on the result today -- see the block above.

    The local number is the **best candidate's probability**, taken over
    every candidate rather than off `perception.best`: `best` is the argmax
    under the *detection* gate, and corroboration is a different question
    asked at a different bar. On the frames that matter this is the whole
    measurement -- the storage-bin frames score 0.00-0.44 locally and the
    true sightings 0.25-0.98.
    """
    nav = scene.get("_navigate") or {}
    claimed = bool(nav.get("target_visible"))
    local = None
    if perception.status != UNAVAILABLE and perception.candidates:
        local = max(c.probability for c in perception.candidates)

    if not claimed:
        verdict = NO_CLAIM
    elif perception.status == UNAVAILABLE:
        verdict = CORROBORATION_UNAVAILABLE
    elif local is not None and local >= bar:
        verdict = CORROBORATED
    else:
        verdict = UNCLEAR
    return {
        "verdict": verdict,
        "claimed": claimed,
        "bar": bar,
        "local_probability": round(local, 4) if local is not None else None,
        # Says plainly that the verdict changed nothing, so no reader of a
        # walk or a status payload can mistake a measurement for a policy.
        "enforced": False,
    }


# How far off centre a target may be and still be called "center". Matches
# the vocabulary /navigate answers in, so a consumer cannot tell a local
# bearing from a cloud one by its shape -- only by `_tier.cloud_called`,
# which is the field that actually means it.
CENTER_BAND_DEG = 10.0

# How far a bearing-steered turn may go, in degrees (R1). A turn used to be
# a verb with no size -- every LEFT/RIGHT was the server's default 90 -- and
# against a 10-degree centre band that overshoots every target less than 80
# degrees off-axis, so the tier flipped LEFT/RIGHT on 52 of 60 steps and
# closed ZERO distance (measured 2026-09-25, PLAN-ros-alignment.md R1). Now a
# turn chosen from a measured bearing turns BY that bearing: one correction,
# then FORWARD. The floor exists because a bearing just past the band is
# still a real error and a 0-degree turn would spin in place for ever; the
# ceiling because a target behind the robot is better approached as two
# quarter turns re-measured in between than one blind half turn.
MIN_TURN_DEG = 5
MAX_TURN_DEG = 90

# How far off centre a MEASURED bearing may be before the tier corrects it
# (R1b). Deliberately separate from CENTER_BAND_DEG, which is /navigate's
# reporting vocabulary and stays at 10. The 10-degree band made sense while
# every turn was a quarter turn -- correcting 7 degrees with 90 was worse than
# not correcting. With sized turns it is just drift: every one of 28 search
# runs that ended blocked had driven FORWARD with the target 5-6 degrees off,
# drifted 0.2-0.26 cells off the doorway's centre line over three cells, and
# put a jamb in its own path. A 3-degree band with MIN_TURN_DEG's 5-degree
# floor corrects anything that matters and settles in one step.
STEER_BAND_DEG = 3.0


# The size of a SEARCH turn -- any tier turn with no measured bearing behind
# it: a scan, a held cloud goal, a cloud turn the local tier could not size
# (R1b, PLAN-ros-alignment.md 3.5). Those used to go out with no size, so the
# executor's default 90 degrees applied -- and against a 60-degree simulated
# field of view (66 for the Camera Module 3, brain/perceive.py's hfov_deg) a
# 90-degree step leaves a blind gap between consecutive views. A target
# sitting in the gap is never seen by turning: live, a robot starting 45
# degrees off the backpack turned eight times without once facing it.
# Forty-five is under both fields of view with 15-21 degrees of overlap, so
# one rotation covers the whole circle.
SCAN_TURN_DEG = 45


def turn_for(bearing_deg: float) -> int:
    """The size of a turn that centres a target at `bearing_deg`.

    An int because `POST /action`'s `angle` is one. Sign is the caller's
    business -- LEFT or RIGHT already says which way.
    """
    return int(round(min(MAX_TURN_DEG, max(MIN_TURN_DEG, abs(bearing_deg)))))


def percentiles(samples) -> Optional[dict]:
    """n / p50 / p90 / p99 / max over a list of millisecond samples.

    `None` when there are none, rather than a dict of zeroes: "no calls
    were made" and "every call took 0ms" are different facts, and a
    dashboard that draws the second when the first is true is the stale-
    readout failure this project keeps meeting.

    **A percentile over a handful of samples is one sample with a fancy
    name.** `n` travels with every one of these so a reader can see that,
    and the dashboard prints it.
    """
    if not samples:
        return None
    xs = sorted(samples)

    def at(p):
        return xs[min(len(xs) - 1, max(0, int(round(p / 100 * len(xs))) - 1))]

    return {"n": len(xs), "p50": round(at(50)), "p90": round(at(90)),
            "p99": round(at(99)), "max": round(xs[-1]),
            "mean": round(sum(xs) / len(xs))}


def _direction_for(perception) -> str:
    """`left` / `center` / `right`, or an honest non-answer.

    Three outcomes, and the third is the one that matters: a target that
    is detected but whose bearing could not be measured is `unknown`, not
    `not_visible`. The two mean opposite things to anything reading the
    field, and "not_visible" beside `target_visible: True` is a straight
    contradiction.
    """
    if perception.status != DETECTED:
        return "not_visible"
    bearing = perception.bearing_deg
    if bearing is None:
        return "unknown"
    if bearing < -CENTER_BAND_DEG:
        return "left"
    if bearing > CENTER_BAND_DEG:
        return "right"
    return "center"


def _name_of(backend, attr: str) -> Optional[str]:
    """What to call a detector or a scorer on screen.

    Its own declared name if it has one -- `yolo11s.pt` today, a `.hef`
    after P4 -- and its class name otherwise, so a fake reads as a fake
    rather than as a blank. Never None for a backend that exists: a blank
    name in 6.3's readout would say "no detector", which is a different
    and much more alarming claim.
    """
    if backend is None:
        return None
    return getattr(backend, attr, None) or type(backend).__name__


@dataclass
class TierStats:
    """Counters, for 6.3's readout and for measuring 2.4's claim live."""

    frames: int = 0
    cloud_calls: int = 0
    # 3.90 E: frames the robot had that perception never took (gaps in the
    # robot's frame seq: it was busy with an earlier one), and the vote's
    # last tally, when one is set.
    frames_skipped: int = 0
    vote: Optional[dict] = None
    triggers: dict = field(default_factory=dict)
    perception: dict = field(default_factory=dict)
    # 1.11a's own list of what it still needs: *"the counter in 6.3 should
    # show corroborated-versus-claimed, or the twin cannot show this
    # working."* Two counters rather than a rate, for the same reason the
    # deliberation counter is calls AND frames -- "80% corroborated" over
    # five claims is a different statement from the same figure over fifty.
    claims: int = 0
    corroborated: int = 0
    verdicts: dict = field(default_factory=dict)
    # Per-call wall clock, milliseconds. Kept as raw SAMPLES rather than a
    # running mean because the question asked of them is about the tail --
    # a mean of 3090ms and a p99 of 3875ms are different facts, and you
    # cannot recover the second from the first. Bounded below so a long
    # mission cannot grow this without limit.
    cloud_ms: list = field(default_factory=list)
    perception_ms: list = field(default_factory=list)
    # 3.80.C: the local VLM tier at arrival -- questions asked, how each
    # ended (`yes`, `no`, `absent`, or `unable_<status>`), its wall clock,
    # and how many arrivals it handed to the cloud after max_unable.
    local_calls: int = 0
    local: dict = field(default_factory=dict)
    local_ms: list = field(default_factory=list)
    escalations: int = 0
    # 3.80.O: arrivals on the local VLM's box, confirmed by the cloud alone.
    guarded: int = 0
    # 3.80.D: the per-frame questions put to the local tier, by trigger;
    # how they ended (`yes`, `no`, `unable_<status>`, `absent`); and the
    # questions handed to the cloud, by why (`unable`, `absent`).
    local_triggers: dict = field(default_factory=dict)
    local_frame: dict = field(default_factory=dict)
    local_frame_calls: int = 0
    frame_escalations: dict = field(default_factory=dict)
    local_frame_ms: list = field(default_factory=list)
    # 3.80.F: the direction questions, by how they ended (`left`, `right`,
    # `forward`, `unable_<status>`, `absent`), and their wall clock; and the
    # frames YOLOE + CLIP's candidate steered.
    local_direction: dict = field(default_factory=dict)
    local_direction_ms: list = field(default_factory=list)
    candidate_steers: int = 0
    # 3.80.G: cloud failures logged rather than raised (cloud_errors_fatal off).
    cloud_errors: int = 0
    # 3.91 A: cloud answers dropped from the walk record's queue, untaken.
    cloud_answers_dropped: int = 0
    # 3.91.C: nos remembered (by `local` / `cloud`), how each was released
    # (`moved`, `turned`, `lost`, `timeout`, `yes`), local nos that could not
    # tell (the box out of range), and frames that showed a rejected
    # candidate.
    rejections: dict = field(default_factory=dict)
    releases: dict = field(default_factory=dict)
    cannot_tell: int = 0
    masked_frames: int = 0
    # How many samples each bucket has been offered, and the reservoir's
    # draw (seeded, so a replay samples the same way) -- 3.76.
    _seen: dict = field(default_factory=dict, repr=False, compare=False)
    _max: dict = field(default_factory=dict, repr=False, compare=False)
    _rng: random.Random = field(default_factory=lambda: random.Random(0),
                                repr=False, compare=False)

    # Enough to characterise a walk (a mission is ~120 steps and ~20 calls)
    # and small enough that the status payload stays a status payload.
    MAX_SAMPLES = 500

    def record(self, bucket: str, ms: float) -> None:
        """3.76: a uniform sample of the whole mission (reservoir sampling),
        not its first MAX_SAMPLES -- at 4 frames/s that was the first two
        minutes of a 20-minute mission, so a later slowdown never showed."""
        samples = getattr(self, bucket)
        seen = self._seen[bucket] = self._seen.get(bucket, 0) + 1
        value = round(float(ms), 1)
        self._max[bucket] = max(self._max.get(bucket, value), value)
        if len(samples) < self.MAX_SAMPLES:
            samples.append(value)
        else:
            j = self._rng.randrange(seen)
            if j < self.MAX_SAMPLES:
                samples[j] = value

    def _percentiles(self, bucket: str) -> Optional[dict]:
        """The reservoir's percentiles with the TRUE count and maximum: `n`
        is how many samples back the figures, and one slow call must never
        be evicted from `max` (3.76)."""
        p = percentiles(getattr(self, bucket))
        if p is not None:
            p["n"] = self._seen.get(bucket, p["n"])
            p["max"] = round(self._max.get(bucket, p["max"]))
        return p

    def as_dict(self) -> dict:
        saving = (self.frames / self.cloud_calls) if self.cloud_calls else None
        return {
            "cloud_ms": self._percentiles("cloud_ms"),
            "perception_ms": self._percentiles("perception_ms"),
            "local_ms": self._percentiles("local_ms"),
            "local_calls": self.local_calls,
            "local": dict(self.local),
            "escalations": self.escalations,
            "guarded": self.guarded,
            "local_triggers": dict(self.local_triggers),
            "local_frame": dict(self.local_frame),
            "local_frame_calls": self.local_frame_calls,
            "local_frame_ms": self._percentiles("local_frame_ms"),
            "frame_escalations": dict(self.frame_escalations),
            "local_direction": dict(self.local_direction),
            "local_direction_ms": self._percentiles("local_direction_ms"),
            "candidate_steers": self.candidate_steers,
            "cloud_errors": self.cloud_errors,
            "cloud_answers_dropped": self.cloud_answers_dropped,
            "rejections": dict(self.rejections),
            "releases": dict(self.releases),
            "cannot_tell": self.cannot_tell,
            "masked_frames": self.masked_frames,
            "frames": self.frames,
            "frames_skipped": self.frames_skipped,
            "vote": dict(self.vote) if self.vote else None,
            "cloud_calls": self.cloud_calls,
            "claims": self.claims,
            "corroborated": self.corroborated,
            "verdicts": dict(self.verdicts),
            # The number 6.1 puts at 4-6x. Measured live here rather than
            # replayed, and directly comparable.
            "frames_per_call": round(saving, 2) if saving else None,
            "triggers": dict(self.triggers),
            "perception": dict(self.perception),
        }


def _local_kind(got: dict, no_below: float, yes_at: float = LOCAL_YES_AT) -> str:
    """A sighting reply as `yes`, `no` or `unable_<why>` (3.80.D's rule)."""
    status, score = got.get("status"), got.get("score")
    numeric = isinstance(score, (int, float))
    if status == "answered" and numeric and score >= yes_at:
        return "yes"
    if status == "answered" and numeric and score <= no_below:
        return "no"
    return (f"unable_{status}" if status != "answered"
            else "unable_band" if numeric else "unable_noscore")


@dataclass
class _Question:
    """3.80.D: one frame's question, as it was asked -- for reading a box's
    bearing when the answer lands, perhaps several frames and a turn later."""
    trigger: str
    epoch: int
    pose: Optional[Pose]
    pan_deg: float
    hfov_deg: float
    cloud_ok: bool = True
    # 3.80.F: what perception saw on the asked frame -- the target (no
    # direction is asked: it is in view), and a crop in the uncertain band
    # (a no on this frame rejects it).
    detected: bool = False
    band: bool = False
    # 3.91.C: the candidate run the asked frame was in, and its box's share
    # of the frame (None: unknown, so a no about it counts).
    episode: int = 0
    box_share: Optional[float] = None
    # Set when the local tier has answered (or failed): the arrival's own
    # local check waits for this, not for the whole question.
    local_done: threading.Event = field(default_factory=threading.Event)


# 3.91 A: how much of a cloud answer's text a walk row keeps. A cloud scene
# (`brain/navigate.py` `to_scene`) carries the action it acts on as
# `safest_direction` and the rest of the reply under `_navigate`, all of
# which is kept (a hand-kept list of its fields drifted once already).
CLOUD_TEXT_CHARS = 500
CLOUD_FIELDS_KEPT = 16


# Per answer: every string in the stored reply together.
CLOUD_REPLY_CHARS = 4000
# Kept first when a reply has more fields than CLOUD_FIELDS_KEPT.
_REPLY_FIRST = ("reply_action", "model_id", "target_visible", "target_reached",
                "target_direction", "reasoning")


def _json_safe(value, depth: int = 0, budget: Optional[list] = None):
    """A copy that stores as strict JSON and stays small: strings cut to
    CLOUD_TEXT_CHARS and, together, to CLOUD_REPLY_CHARS; at most
    CLOUD_FIELDS_KEPT items a level (`_REPLY_FIRST` first), three levels;
    non-finite numbers as None; numpy scalars by value; anything else by its
    type name."""
    if budget is None:
        budget = [CLOUD_REPLY_CHARS]
    if hasattr(value, "item") and not isinstance(value, (list, tuple, dict, str)):
        try:
            value = value.item()          # a numpy scalar
        except Exception:  # noqa: BLE001 -- not one; falls through to its name
            pass
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        cut = value[:min(CLOUD_TEXT_CHARS, max(budget[0], 0))]
        budget[0] -= len(cut)
        return cut
    if depth >= 3:
        return type(value).__name__
    if isinstance(value, dict):
        keys = ([k for k in _REPLY_FIRST if k in value] if depth == 0 else []) + \
               [k for k in value if not (depth == 0 and k in _REPLY_FIRST)]
        return {str(k)[:64]: _json_safe(value[k], depth + 1, budget)
                for k in keys[:CLOUD_FIELDS_KEPT]}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v, depth + 1, budget) for v in list(value)[:CLOUD_FIELDS_KEPT]]
    return type(value).__name__
# Held for the runner to take; more than this untaken (no one filing) and the
# oldest go.
CLOUD_ANSWERS_KEPT = 64
_HTTP_STATUS = re.compile(r"\bHTTP (\d{3})\b")


def _http_status(error: BaseException) -> Optional[int]:
    """The HTTP status a failed call carries, from the error or its text."""
    for value in (getattr(error, "status_code", None),
                  getattr(getattr(error, "response", None), "status_code", None)):
        if isinstance(value, int):
            return value
    found = _HTTP_STATUS.search(str(error)[:200])
    return int(found.group(1)) if found else None


def _frame_seq(frame) -> Optional[int]:
    """The robot's own number for a frame (3.90's `metadata.seq`), or None."""
    seq = ((frame or {}).get("metadata") or {}).get("seq") if isinstance(frame, dict) else None
    return seq if isinstance(seq, int) else None


@dataclass
class _LocalAnswer:
    q: _Question
    got: dict
    absent: bool = False   # absent, and the call cap kept the cloud out
    # 3.80.F: the direction question's reply, when one was asked after a
    # search question's sighting answer was not a yes.
    direction: Optional[dict] = None


# 3.94: the rungs that act on a bearing measured or located on THIS frame
# are exempt from the spin guard; those and dead reckoning size their turn.
RUNG_STEER, RUNG_LOCATED, RUNG_RECKONED = "steer", "located", "reckoned"
RUNG_CANDIDATE, RUNG_HELD, RUNG_SCAN, RUNG_SPIN_GUARD = "candidate", "held", "scan", "spin_guard"
RUNGS_THIS_FRAME = (RUNG_STEER, RUNG_LOCATED, RUNG_CANDIDATE)
# 3.94.D: the tree's defaults -- the code's values before the tree. The same
# table is `control/brain_config.py`'s TIER_TREE_DEFAULTS (checked by a test).
TREE_DEFAULTS = {
    "what_rungs": [RUNG_STEER, RUNG_LOCATED, RUNG_RECKONED, RUNG_CANDIDATE, RUNG_HELD, RUNG_SCAN],
    "absent_events": ["band", "oov", "cold_frames", "cold_metres"],
    "steer_band_deg": STEER_BAND_DEG,
    "scan_turn_deg": SCAN_TURN_DEG,
    "min_turn_deg": MIN_TURN_DEG,
    "max_turn_deg": MAX_TURN_DEG,
    "local_yes_at": LOCAL_YES_AT,
}


@dataclass
class _RouteDecision:
    """3.94.C: the blackboard of the "who to ask" tree."""
    frame: dict
    perception: Perception
    trigger: Optional[str]
    epoch: int
    local_fn: Optional[Callable] = None
    scene: Optional[dict] = None


@dataclass
class _TriggerDecision:
    """3.94.B: the blackboard of the "when to ask" tree."""
    perception: Perception
    trigger: Optional[str] = None
    band: bool = False
    confirmed: Optional[str] = None


@dataclass
class _FrameDecision:
    """3.94: the blackboard of the "what to do this frame" tree."""
    perception: Perception
    why: str
    direction: Optional[str] = None
    rung: Optional[str] = None
    rung_turn_deg: Optional[int] = None
    turn_deg: Optional[int] = None
    located_turn: Optional[tuple] = None
    held: Optional[str] = None


class TieredVision:
    """Perception locally, deliberation on a trigger.

    Callable as a `vision_fn(frame) -> scene`, so `MissionRunner` takes it
    exactly where it takes `brain/navigate.py`'s. The per-call timeout and
    the consecutive-failure budget (B3.2) still wrap it, unchanged --
    **and they now wrap only the frames that actually call out**, which is
    a second and unadvertised benefit of the trigger discipline.
    """

    def __init__(
        self,
        pipeline,
        cloud_vision_fn: Callable[[dict], dict],
        *,
        consecutive_frames: int = DEFAULT_CONSECUTIVE,
        sighting_span_s: Optional[float] = DEFAULT_SIGHTING_SPAN_S,
        frame_vote: Optional[tuple] = None,
        cold_search_after: int = DEFAULT_COLD_SEARCH_AFTER,
        cold_search_after_cm: Optional[float] = DEFAULT_COLD_SEARCH_AFTER_CM,
        stale_after: int = DEFAULT_STALE_AFTER,
        max_calls: Optional[int] = None,
        corroboration_bar: float = DEFAULT_CORROBORATION_P,
        oov_cold_search_after: Optional[int] = None,
        local_cold_search_after: Optional[int] = None,
        async_cloud: bool = False,
        hold_goal: bool = DEFAULT_HOLD_GOAL,
        steer_on_sight: bool = DEFAULT_STEER_ON_SIGHT,
        spin_guard_after: int = DEFAULT_SPIN_GUARD_AFTER,
        hold_bearing: bool = DEFAULT_HOLD_BEARING,
        hold_bearing_max_m: float = DEFAULT_HOLD_BEARING_MAX_M,
        confirm_vision_fn: Optional[Callable[[dict], dict]] = None,
        local_identity_fn: Optional[Callable[[dict], dict]] = None,
        local_no_below: float = DEFAULT_LOCAL_NO_BELOW,
        local_max_unable: int = DEFAULT_LOCAL_MAX_UNABLE,
        local_sighting_fn: Optional[Callable[..., dict]] = None,
        local_band_low: float = DEFAULT_LOCAL_BAND_LOW,
        local_direction_fn: Optional[Callable[[dict], dict]] = None,
        steer_on_candidate: bool = False,
        cloud_errors_fatal: bool = True,
        tree: Optional[dict] = None,
        remember_no: bool = DEFAULT_REMEMBER_NO,
        reject_hold_s: float = DEFAULT_REJECT_HOLD_S,
        reject_move_m: float = DEFAULT_REJECT_MOVE_M,
        reject_turn_deg: float = DEFAULT_REJECT_TURN_DEG,
        no_box_range: tuple = DEFAULT_NO_BOX_RANGE,
    ):
        self.pipeline = pipeline
        # 3.91.C: a no about a candidate, remembered (see DEFAULT_REMEMBER_NO).
        # `_rejection` is {"by", "episode", "at_s", "pose"} or None; an
        # episode is a run of detections broken by REJECT_LOST_FRAMES absent
        # frames. `_masked`: this frame shows the rejected candidate.
        self.remember_no = bool(remember_no)
        self.reject_hold_s = max(0.0, float(reject_hold_s))
        self.reject_move_m = max(0.0, float(reject_move_m))
        self.reject_turn_deg = max(0.0, float(reject_turn_deg))
        self.no_box_range = (float(no_box_range[0]), float(no_box_range[1]))
        self._rejection: Optional[dict] = None
        # Review: `wait_inflight()` can apply an answer on the runner's
        # abandoned thread while a tick tracks the candidate.
        self._reject_lock = threading.RLock()
        self._episode = 0
        self._lost = REJECT_LOST_FRAMES
        self._masked = False
        self._probation = False
        # The candidate run a probation is about (the user, 2026-10-24: that
        # run only -- lost again with no answer, it ends).
        self._probation_episode: Optional[int] = None
        # The highest tier behind a probation (TIER_RANK's names).
        self._probation_by: Optional[str] = None
        # A candidate run the cloud should settle (a local no that cannot
        # tell, or a local yes against the cloud's no), and the run the cloud
        # last said yes about (a local no does not undo it).
        self._escalate_episode: Optional[int] = None
        self._cloud_yes_episode: Optional[int] = None
        self._release_gen = 0
        self._disagree_key: Optional[tuple] = None   # (episode, release_gen) asked
        self._inflight_ask: Optional[tuple] = None   # (pose, episode) of a cloud call out
        # 3.80.F: the local tier's search question (brain/local_vlm.py's
        # `direction`), asked after a search trigger's sighting answer that
        # is not a yes; its answer is held as a cloud goal is. None: the
        # local tier gives no direction (3.80.D).
        self.local_direction_fn = local_direction_fn
        # 3.80.G: under an explore search the cloud only ever answers an
        # escalated question whose direction is unused, and explore needs no
        # cloud to search: its failures are logged and counted
        # (`stats.cloud_errors`), not raised into B3.2's budget.
        self.cloud_errors_fatal = bool(cloud_errors_fatal)
        # 3.80.F: YOLOE + CLIP steer toward a best crop in the uncertain band
        # (not a detection) -- not after a local no about a frame with such
        # a crop, until the band run breaks (`_candidate_rejected`).
        self.steer_on_candidate = bool(steer_on_candidate)
        self._candidate_rejected = False
        self._local_held_why: Optional[str] = None   # "located" or "direction"
        # 3.80.D: the local tier for every question a FRAME raises, called
        # with `box=True` (brain/local_vlm.py's `ask`). None: the triggers go
        # to the cloud, as before 3.80.D, and the band and out-of-vocabulary
        # questions do not exist.
        self.local_sighting_fn = local_sighting_fn
        self.local_band_low = float(local_band_low)
        self.match_probability = float(getattr(pipeline, "match_probability", None)
                                        or DEFAULT_MATCH_PROBABILITY)
        self._frame_unable = 0           # unable answers in a row, any question
        self._band_run = 0
        self._band_asked = False
        self._retry_start = False
        self._local_turn: Optional[tuple] = None   # (direction, turn_deg), used once
        self._local_held: Optional[str] = None     # FORWARD after a located yes
        self._inflight_tier: Optional[str] = None
        self._inflight_question: Optional["_Question"] = None
        self._local_landed: Optional[dict] = None  # shown on the frame it landed
        self._local_track = False   # 3.80.O: set by explore (`set_local_track`)
        # Counts the worker thread changes too (an absent local tier's
        # cloud call).
        self._stats_lock = threading.Lock()
        # 3.80.C: the local VLM tier at arrival (brain/local_vlm.py's
        # client). None: the cloud alone confirms, as before 3.80.
        self.local_identity_fn = local_identity_fn
        self.local_no_below = float(local_no_below)
        self.local_max_unable = max(1, int(local_max_unable))
        self._local_unable = 0
        # Guards the local tier's counters against a call the runner
        # abandoned (abandon_local) returning on its own thread later.
        self._local_lock = threading.Lock()
        self._local_started = 0          # ids of local calls, in order
        self._local_inflight: Optional[int] = None
        self._local_done: Optional[tuple] = None   # (id, verdict) of the last one ended
        self._local_abandoned: set = set()
        self._frames_off_arrival = 0
        self.cloud_vision_fn = cloud_vision_fn
        # 3.56: the arrival confirmation's own client, built with the
        # confirmation's shorter deadline so a hang ends in httpx (as
        # CloudUnavailable) rather than in an abandoned thread. None: the
        # triggers' client, resolved at call time.
        self.confirm_vision_fn = confirm_vision_fn
        self.consecutive_frames = max(1, int(consecutive_frames))
        # 0 (or None) is counted only; a negative span would time nothing.
        self.sighting_span_s = max(0.0, float(sighting_span_s or 0)) or None
        self.frame_vote = tuple(frame_vote) if frame_vote else None
        self.cold_search_after = max(1, int(cold_search_after))
        # The distance rule, and the state it needs. `_odometry_at_call` is
        # the reading when the cloud last looked; None means "never looked
        # or the backend has no encoders", and both resolve to the frame
        # rule rather than to a fabricated zero.
        self.cold_search_after_cm = (
            None if cold_search_after_cm is None
            else max(1.0, float(cold_search_after_cm)))
        self._odometry_at_call: Optional[float] = None
        self._odometry_usable: Optional[bool] = None
        self._distance_since_call: Optional[float] = None
        # 0 disables the floor entirely, which reproduces the pre-2026-09-07
        # edge-only behaviour -- worth having for measuring the difference,
        # and worth NOT having as the default.
        self.stale_after = max(0, int(stale_after))
        self.max_calls = max_calls
        # 1.11a asks that the bar get the same treatment
        # DEFAULT_MATCH_PROBABILITY got -- measured, named, and settable in
        # config rather than compiled in. It is settable here and in
        # config/robot.yaml, and it gates nothing.
        self.corroboration_bar = float(corroboration_bar)

        # 4.2's quiet failure, made explicit at mission start (2026-09-12).
        #
        # `absent` from a class-gated pipeline and `absent` from a clear
        # room are the same string. There is no `unknown` class and no
        # error: for a target COCO has no word for, the detector reports
        # nothing, and nothing is indistinguishable from an empty room.
        # Today that is discovered three frames in, when `cold_search`
        # happens to fire. The pipeline can say it at step zero.
        #
        # **Reported, and NOT enforced by default** -- deliberately, and
        # for the reason 1.11a is: `in_vocabulary` is a measured-bad
        # predictor of local visibility. `bottle` is one of COCO's 80 and
        # the local tier still lost 11 of 18 sightings on the bottle walk;
        # the shoes walk has no COCO word at all and still detected 7 of
        # 13. So the flag is evidence about the *detector's vocabulary*,
        # never a capability claim, and shortening the cold-search wait on
        # it is an experiment rather than a fix. Set
        # `oov_cold_search_after` to run that experiment; leave it None and
        # the trigger policy is bit-for-bit what it was.
        self.vocabulary = getattr(pipeline, "vocabulary", None)
        self.oov_cold_search_after = (
            None if oov_cold_search_after is None
            else max(1, int(oov_cold_search_after)))
        # 3.80.P (the user's option a): the cold-search pace while the
        # question goes to the local tier, whose answers are free. A
        # question the cloud would answer -- the local tier absent, or
        # unable `local_max_unable` times -- still waits the cloud's bar,
        # so a dead VLM never brings the paid pace to this one. None: one
        # pace, as before.
        self.local_cold_search_after = (
            None if local_cold_search_after is None
            else max(1, int(local_cold_search_after)))
        # -------- Phase A: the cloud call stops blocking the tick --------
        #
        # 2.5 has specified this since the tiered architecture was written
        # -- *"the reactive tier always holds a current goal; a new one
        # arrives asynchronously and replaces it"* -- and the code did the
        # opposite. `control/mission_runner.py` calls the policy through
        # `call_with_timeout`, so a paid step froze the mission for the
        # round trip (3.6s measured, `vision_timeout_s` 20.0) against a
        # 0.25s tick. With 79% of calls fired by the cold-search counter,
        # that is a stall roughly every nine frames.
        #
        # Two halves, and they only work together:
        #
        #   1. **Dispatch, don't wait.** The call runs on one worker
        #      thread and the tick returns now.
        #   2. **Hold the goal while it is in flight.** Without this the
        #      stand-in emits SCAN_ACTION, so the robot spins through
        #      every frame it is waiting on -- which is the degenerate
        #      RIGHT-on-every-frame mode this project has already recorded
        #      once, arrived at from a different direction.
        #
        # **Off by default.** It changes what the robot does on a free
        # frame, and nothing has measured that yet. Phase D is where a
        # number gets chosen and this gets turned on deliberately.
        self.async_cloud = bool(async_cloud)
        self.hold_goal = bool(hold_goal)
        self.steer_on_sight = bool(steer_on_sight)
        # P25. The tracker integrates every odometry reading whether or not
        # the feature is on, so the readouts show what it WOULD have said --
        # which is how the default gets earned rather than argued.
        self.hold_bearing = bool(hold_bearing)
        self.hold_bearing_max_m = max(0.0, float(hold_bearing_max_m))
        self._odom = OdomTracker()
        self._goal = GoalPose()
        self._dead_reckoned = 0
        self._steer_turn_deg = None
        self._reckoned_turn_deg = None
        self.spin_guard_after = max(0, int(spin_guard_after))
        self._consecutive_turn_deg = 0.0
        self._executor = None
        self._collect_lock = threading.Lock()
        # 3.56 follow-up: the last call taken by a collector, and the one the
        # runner charged to B3.2 when its wait timed out (never re-counted).
        self._last_taken = None
        self._charged = None
        self._inflight = None
        self._inflight_trigger: Optional[str] = None
        self._inflight_perception: Optional[Perception] = None
        self._perception_this_frame: Optional[Perception] = None
        # The epoch is the `guidanceEpoch` trick one layer down. A call
        # still in flight when the mission ends must not have its answer
        # applied -- that is the orphaned-in-flight-call class that put a
        # dead session's decision over a live camera view in Robot view,
        # and that filed one walk's frame into the next walk's directory.
        self._epoch = 0
        self._closed = False
        self._executor_lock = threading.Lock()
        self._last_cloud_scene: Optional[dict] = None
        # A verdict from an async call, held for exactly ONE frame -- the
        # one it landed on. Without this the corroboration row reads "no
        # claim this step" on every frame of an async mission while the
        # tally climbs behind it, so 1.11a's per-frame measurement is lost
        # from walk.jsonl entirely. Cleared after it is shown, because the
        # rule `_local_scene` already enforces is that a stale verdict must
        # never read as this frame's.
        self._landed_corroboration: Optional[dict] = None
        # 3.91 A: every cloud answer (or failure) that landed, as a summary,
        # until `take_cloud_answers()`. Appended and taken under
        # `_collect_lock`, so an answer collected by the runner's wait thread
        # is neither lost nor shown twice.
        self._cloud_answers: list = []
        self._unapplied_noted = None
        self._inflight_seq = None
        self._pending_error: Optional[BaseException] = None
        self.stats = TierStats()
        # 6.3 asks for **the detector's own name** on screen, not just its
        # output: *"swap the HEF and the name on screen changes; that is
        # the experiment loop made watchable."* Read off the backends
        # rather than passed in, so a fake, a `.pt` file and a `.hef`
        # each report themselves and none of them can be misdescribed by
        # a caller. Falling back to the class name keeps a fake legible
        # instead of blank -- a blank name would read as "no detector".
        # `getattr` twice over, because a pipeline is only a duck here --
        # the fakes in tests/test_tiered.py are a `perceive()` method and
        # nothing else, and a readout that crashed the mission loop
        # because a stand-in had no detector attribute would be a poor
        # trade for a label.
        self.models = {
            "detector": _name_of(getattr(pipeline, "detector", None), "weights"),
            "scorer": _name_of(getattr(pipeline, "scorer", None), "model_name"),
            # None when the floor mask is off, which is the default -- and
            # the readout has to show that difference, because "which crop
            # sources were running" is the first thing you need to know
            # when reading a walk back (detector alone measured 86% recall
            # on the corpus, detector + mask 94%).
            "proposer": _name_of(getattr(pipeline, "proposer", None), "model_name"),
            "target": getattr(pipeline, "target", None),
            "crop_source": getattr(pipeline, "crop_source", None),
        }

        self._started = False
        self._run: list = []          # the recent status history
        # 3.90 A: the capture stamp of the first frame of the current
        # same-status run (None: unstamped, so the run is counted only), and
        # of the frame being judged.
        self._run_since: Optional[float] = None
        self._frame_stamp: Optional[float] = None
        # 3.90 E: the vote's window -- (stamp, detected) of the last n frames
        # perception took -- and the robot's seq of the last frame taken.
        self._votes: list = []
        self._last_seq: Optional[int] = None
        self._repeat_frame = False
        self._vote_ok = False
        if self.frame_vote:
            self.stats.vote = {"k": self.frame_vote[0], "n": self.frame_vote[1],
                               "detected": 0, "fired": 0}
        self._absent_streak = 0
        self._last_confirmed = ABSENT
        self._since_call = 0
        # 3.94.D: the tree's order and numbers (`brain.tier_tree`, validated
        # by `load_brain_config`; the defaults are the module's constants, so
        # a policy built without one behaves as before).
        t = {**TREE_DEFAULTS, **(tree or {})}
        for key, names in (("what_rungs", TREE_DEFAULTS["what_rungs"]),
                           ("absent_events", TREE_DEFAULTS["absent_events"])):
            # The loader validates in full; a policy built directly (a test, a
            # tool) still fails here, by name, not as a KeyError mid-build.
            if (not isinstance(t[key], (list, tuple)) or len(set(t[key])) != len(t[key])
                    or any(n not in names for n in t[key])):
                raise ValueError(f"tree {key} must name each of {names} at most once; "
                                 f"got {t[key]!r}")
        self.what_rungs = list(t["what_rungs"])
        self.absent_events = list(t["absent_events"])
        self.steer_band_deg = float(t["steer_band_deg"])
        self.scan_turn_deg = int(t["scan_turn_deg"])
        self.min_turn_deg, self.max_turn_deg = t["min_turn_deg"], t["max_turn_deg"]
        self.local_yes_at = float(t["local_yes_at"])
        self._what_tree = self._build_what_tree()
        self._when_tree = self._build_when_tree()
        self._who_tree = self._build_who_tree()

    # -- the vision_fn contract ------------------------------------------

    def take_cloud_answers(self, final: bool = False) -> list:
        """3.91 A: every cloud answer (or failure) that landed since the
        last take, oldest first, each handed out once. Taken by the one
        reader that files rows (`MissionRunner._note_decision`) when it files
        a decision, not by `__call__`: a frame's scene can be thrown away (a
        guard that gave up on it, a halt before the decision is noted), and
        an answer taken into it would be lost.

        `final`: the runner's last take, at the mission's end, before
        `close()` drops what is out. A call that has landed but that no
        frame has collected is NOTED, never applied: this runs on the Stop's
        thread, beside a tick that may be mid-frame, before the motors stop
        (review round 3), so it reads the finished future and touches
        nothing a decision reads."""
        if final:
            try:
                self._note_landed_unapplied()
            except Exception as e:  # noqa: BLE001 -- the queue is taken regardless
                logger.warning("noting the last cloud answer failed: %s", e)
        with self._collect_lock:
            answers, self._cloud_answers = self._cloud_answers, []
        return answers

    def _note_landed_unapplied(self) -> None:
        with self._collect_lock:
            fut = self._inflight
            if fut is None or not fut.done() or fut.cancelled():
                return
            self._unapplied_noted = fut
            trigger, asked_seq = self._inflight_trigger, self._inflight_seq
            via_local = self._inflight_tier == "local"
            perception = self._inflight_perception
        error = fut.exception()
        if error is not None:
            self._note_cloud_answer(trigger, asked_seq, error=error, via_local=via_local)
        elif isinstance(fut.result(), dict):      # not a local answer, not superseded
            scene = fut.result()
            # The verdict as a collect would read it, without its counting.
            verdict = (corroboration_for(scene, perception, self.corroboration_bar)["verdict"]
                       if perception is not None else None)
            self._note_cloud_answer(trigger, asked_seq, scene=scene, verdict=verdict,
                                    via_local=via_local)

    def __call__(self, frame: dict) -> dict:
        # 3.76: the mission this frame belongs to. A Stop closes the policy
        # from another thread, often while this frame is in perception; the
        # call this frame would fire must then not go out (it was billed,
        # on a worker pool created again and never shut down).
        frame_epoch = self._epoch
        self._read_odometry(frame)
        # 3.91.C (review): before a landed answer is applied, so a no that
        # lands now is held from this frame's stamp, not the one before.
        self._frame_stamp = frame.get("captured_at")
        # An answer that landed since the last frame becomes the current
        # goal now, BEFORE the trigger policy runs -- so a call that has
        # already returned is never counted as still in flight.
        self._collect_inflight()
        _t0 = time.perf_counter()
        perception = self.pipeline.perceive(frame)
        self.stats.record("perception_ms", (time.perf_counter() - _t0) * 1000)
        self._perception_this_frame = perception
        self.stats.frames += 1
        self.stats.perception[perception.status] = (
            self.stats.perception.get(perception.status, 0) + 1)

        # B3.2 still owns the failure budget. An async call that failed is
        # raised on the first frame after it lands rather than swallowed,
        # so `MissionRunner._guarded_vision()` converts it to
        # VisionUnavailable exactly as it does a synchronous failure and
        # the budget counts the same events it always did.
        if self._pending_error is not None:
            err, self._pending_error = self._pending_error, None
            if self.cloud_errors_fatal:
                raise err
            self._count_cloud_error(err)

        self._count_skipped(frame)
        self._track_candidate(perception)
        # 3.94.C: who to ask (if anyone) is a tree (`_build_who_tree`); the
        # code it replaced is in `tests/tiered_reference.py`.
        bb = _RouteDecision(frame=frame, perception=perception,
                            trigger=self._trigger_for(perception), epoch=frame_epoch)
        if self._who_tree.tick(bb) != SUCCESS:
            # Every route ends in an answer; a tree that ends without one
            # would hand the runner nothing to act on.
            raise RuntimeError(f"who to ask: no route answered {bb.trigger}")
        return bb.scene

    # -- 3.91.C: a no about a candidate, remembered ------------------------

    def _stamp(self) -> Optional[float]:
        stamp = self._frame_stamp
        if isinstance(stamp, numbers.Real) and math.isfinite(stamp):
            return float(stamp)
        return None

    def _held_s(self, r: dict) -> float:
        """How long a no has been held: on the frames' capture stamps when
        both ends have one (a replay keeps its time), else on the monotonic
        clock -- never one against the other (review: wall seconds against
        monotonic ones)."""
        now = self._stamp()
        if now is not None and r["at_stamp"] is not None:
            return now - r["at_stamp"]
        return time.monotonic() - r["at_mono"]

    def _band_crop(self, perception: Perception) -> bool:
        """An absent frame that still holds a crop at the band's P: the
        candidate is in view, under the gate (review: such frames counted as
        lost, and three of them released the no)."""
        return bool(perception.candidates) and max(
            float(c.probability) for c in perception.candidates) >= self.local_band_low

    def _pose_now(self) -> Optional[Pose]:
        if self._odometry_usable and self._odom.samples:
            p = self._odom.pose
            return Pose(p.x, p.y, p.heading_deg)
        return None

    def _track_candidate(self, perception: Perception) -> None:
        """Before the trees: number the candidate run this frame is in,
        release a remembered no whose candidate has changed, and say whether
        this frame shows the rejected one (`_masked`)."""
        with self._reject_lock:
            self._track_candidate_locked(perception)

    def _track_candidate_locked(self, perception: Perception) -> None:
        if perception.status == DETECTED:
            if self._lost >= REJECT_LOST_FRAMES:
                self._episode += 1
                if self._probation and self._probation_episode != self._episode:
                    # The user (2026-10-24): probation is about the run it
                    # began in; lost again with no answer, it ends -- else it
                    # outlives its object and the real target is never
                    # steered at (review: nothing may be left to clear it).
                    self._end_probation()
            self._lost = 0
        elif perception.status == ABSENT and not self._band_crop(perception):
            # A frame with no picture is not a look away, nor one with the
            # candidate just under the gate.
            self._lost += 1
            if (self._lost == REJECT_LOST_FRAMES and self._probation
                    and self._probation_episode == self._episode):
                # Its run is lost: probation ends here, not at the next
                # detection (Thermos round 3: the readout stayed true).
                self._end_probation()
        r = self._rejection
        if r is not None:
            pose = self._pose_now()
            why = None
            if pose is not None and r["pose"] is not None:
                moved = math.hypot(pose.x - r["pose"].x, pose.y - r["pose"].y)
                turned = abs(_wrap(pose.heading_deg - r["pose"].heading_deg))
                if moved >= self.reject_move_m:
                    why = "moved"
                elif turned >= self.reject_turn_deg:
                    why = "turned"
            elif self._episode != r["episode"]:
                why = "lost"
            elif self._held_s(r) >= self.reject_hold_s:
                why = "timeout"
            if why:
                self._release(why)
        self._masked = False
        self._remask(perception)

    def _remask(self, perception: Perception) -> None:
        """Does this frame show the rejected candidate. Again after an answer
        applied on this frame's own tick (sync), so the frame it lands on does
        not steer at it (review)."""
        with self._reject_lock:
            masked = self._rejection is not None and perception.status == DETECTED
            if masked and not self._masked:
                with self._stats_lock:
                    self.stats.masked_frames += 1
            self._masked = masked

    def _reject(self, by: str, pose: Optional[Pose], episode: int) -> None:
        if not self.remember_no:
            return
        with self._reject_lock:
            if episode != self._episode or self._lost >= REJECT_LOST_FRAMES:
                # A no about a run that has ended, or is being lost (an answer
                # landing late), is about a candidate no longer in view: kept,
                # it would mask whatever is in view now, or be released at
                # once into probation over it (review, both bodies).
                return
            if by == "local" and (self._cloud_yes_episode == episode or (
                    self._rejection is not None and self._rejection["rank"] == "cloud")):
                # The tier ordering: the cloud said yes about this run, or
                # holds its own no already.
                return
            # Moves are measured from where the body is as the no LANDS
            # (review: an approach during an async call released it at once).
            # `pose` None: the asked frame had no odometry.
            if pose is not None:
                pose = self._pose_now() or pose
            # `rank`: the highest tier behind this no -- a cloud probation it
            # replaces keeps the cloud's rank, or a local yes could lift it.
            rank = by
            if self._probation and TIER_RANK[self._probation_by] > TIER_RANK[by]:
                rank = self._probation_by
            self._rejection = {"by": by, "rank": rank, "episode": episode, "pose": pose,
                               "at_stamp": self._stamp(), "at_mono": time.monotonic()}
            if by == "cloud":
                # The cloud's latest answer about this run is no (review
                # round 4: an older yes kept refusing local nos after it).
                self._cloud_yes_episode = None
            # A decided no ends probation and settles a disagreement.
            self._end_probation()
            self._escalate_episode = None
            # A sighting anchored on it is the same candidate (P25's rung).
            self._goal.clear()
        self._count("rejections", by)

    def _release(self, why: str) -> None:
        with self._reject_lock:
            if self._rejection is None:
                return
            rank, self._rejection = self._rejection["rank"], None
            # Each release is a new chance for the disagreement question
            # (Thermos round 4: a local yes on the cloud's probation in a run
            # whose question was spent was never asked).
            self._release_gen += 1
            if why in PROBATION_RELEASES:
                self._probation = True
                self._probation_by = rank
                # A timeout while the candidate is out of view is about the
                # run that comes next, which numbers one up (review round 3:
                # it ended probation the moment the refused bag came back).
                self._probation_episode = self._episode + (
                    why == "timeout" and self._lost >= REJECT_LOST_FRAMES)
        self._count("releases", why)
        if why != "yes":
            # Asked about again once it is confirmed in view: the edge into
            # `detected` was spent while it was remembered. Not after a yes,
            # which has answered it (review: a second paid call).
            self._rearm(TRIGGER_CANDIDATE)

    def _end_probation(self) -> None:
        with self._reject_lock:
            self._probation = False
            self._probation_episode = None
            self._probation_by = None

    def _on_yes(self, by: str, episode: int) -> bool:
        """A yes (a local yes, or a cloud claim) releases a no and ends a
        probation that no higher tier stands behind, and settles a
        disagreement. Outranked (False), a local yes steers nothing, its box
        included: with perception silent nothing says the box is not the
        object the cloud refused (round 4 let it steer: the red replay's
        reversals 1 -> 3). On the cloud's probation it is a disagreement,
        and the cloud is asked."""
        with self._reject_lock:
            r = self._rejection
            if by == "cloud":
                self._cloud_yes_episode = episode
            no_outranks = r is not None and TIER_RANK[r["rank"]] > TIER_RANK[by]
            probation_outranks = (self._probation
                                  and TIER_RANK[self._probation_by] > TIER_RANK[by])
            if no_outranks or probation_outranks:
                if probation_outranks and episode == self._probation_episode:
                    self._escalate_episode = episode
                return False      # a held no: masked frames ask nothing
            self._release("yes")
            self._end_probation()
            self._escalate_episode = None
            return True

    def _learn_from_cloud(self, scene: dict, perception: Optional[Perception],
                          pose: Optional[Pose], episode: int) -> None:
        """A cloud answer that landed: "not visible" about a frame
        perception called DETECTED is remembered (whatever the box: rule 2
        is about where Qwen can tell); a claim releases any no."""
        nav = (scene or {}).get("_navigate") or {}
        if "target_visible" not in nav or nav.get("unparsed"):
            # Review: a reply the service could not parse reads "not
            # visible" by default; it is no answer, not a decided no.
            return
        if nav.get("target_visible"):
            self._on_yes("cloud", episode)
        elif perception is not None and perception.status == DETECTED:
            self._reject("cloud", pose, episode)

    def _rejection_readout(self) -> Optional[dict]:
        r = self._rejection
        if r is None:
            return None
        return {"by": r["by"], "episode": r["episode"],
                "age_s": round(max(0.0, self._held_s(r)), 1)}

    def _refused_in_view(self, perception: Perception) -> bool:
        return perception.status == DETECTED and (
            self._rejection is not None or self._probation)

    def cloud_spoke(self) -> bool:
        """The tiers' order at arrival (the user, 2026-10-24): the cloud has
        answered about the candidate in view -- a yes about this run, a no
        it holds, or its probation -- so the identity check is the cloud's,
        not the local VLM's."""
        with self._reject_lock:
            r = self._rejection
            return (self._cloud_yes_episode == self._episode
                    or (r is not None and r["rank"] == "cloud")
                    or (self._probation and self._probation_by == "cloud"))

    def _on_probation(self, perception: Perception) -> bool:
        """Amendment 3: a candidate in view whose no was released by loss or
        time, with no yes about it since and no far-view no (amendment 4)."""
        return self._probation and perception.status == DETECTED

    # -- 3.94.C: who to ask, as a tree --------------------------------------

    def _build_who_tree(self) -> Node:
        """Who answers this frame's question, drawn as a tree. No trigger: no
        question. Else an edge held back by a call in flight is re-armed, the
        local tier is chosen first (unless it has been unable
        `local_max_unable` times in a row), and the first of these answers:
        the call cap (no question for the cloud), one call at a time, the
        mission ended, the local tier, the cloud."""
        return Fallback("who to ask", [
            Action("no trigger, no question", self._who_no_trigger),
            Sequence("a question", [
                Action("re-arm an edge held back by a call in flight", self._who_rearm),
                Action("the local tier first", self._who_choose_local,
                       lambda: {"local_max_unable": self.local_max_unable}),
                Fallback("the first that answers", [
                    Action("the call cap", self._who_call_cap, lambda: {"max_calls": self.max_calls}),
                    Action("one call at a time", self._who_one_at_a_time,
                           lambda: {"async_cloud": self.async_cloud}),
                    Action("the mission ended", self._who_mission_ended),
                    Action("ask the local tier", self._who_ask_local),
                    Action("ask the cloud", self._who_ask_cloud),
                ]),
            ]),
        ])

    def describe_who_tree(self) -> str:
        return "\n".join(self._who_tree.describe())

    def _answer(self, bb: "_RouteDecision", scene: dict) -> bool:
        bb.scene = scene
        return True

    def _who_no_trigger(self, bb: "_RouteDecision") -> bool:
        if bb.trigger is not None:
            return False
        if not self._masked:
            # 3.91.C: a frame of a rejected candidate is no step toward a
            # question about it (the floor would ask the same thing).
            self._since_call += 1
        return self._answer(bb, self._local_scene(bb.perception))

    def _who_rearm(self, bb: "_RouteDecision") -> bool:
        # 3.76: the edge into `detected` was spent in _trigger_for; held back
        # by the call in flight, it must fire once that lands, not wait for the
        # staleness floor. 3.80.D: the same for the band.
        if (bb.trigger in (TRIGGER_CANDIDATE, TRIGGER_BAND, TRIGGER_DISAGREE)
                and self.async_cloud and self._inflight is not None):
            self._rearm(bb.trigger)
        return True

    def _who_choose_local(self, bb: "_RouteDecision") -> bool:
        # 3.80.D: the local tier first, unless it has been unable
        # `local_max_unable` times in a row (then this one is the cloud's).
        # 3.91.C: the disagreement question is the cloud's: the local tier
        # has answered it.
        bb.local_fn = None if bb.trigger == TRIGGER_DISAGREE else self._local_first()
        return True

    def _who_call_cap(self, bb: "_RouteDecision") -> bool:
        if not (bb.local_fn is None and self.max_calls is not None
                and self.stats.cloud_calls >= self.max_calls):
            return False
        logger.warning("call cap %d reached -- not firing %s", self.max_calls, bb.trigger)
        if self.local_sighting_fn is None or self._probation_by == "cloud":
            # 3.91.C amendment 3: no question can clear probation now (the
            # local tier cannot lift the cloud's).
            self._end_probation()
        return self._answer(bb, self._local_scene(bb.perception, note="call cap reached"))

    def _who_one_at_a_time(self, bb: "_RouteDecision") -> bool:
        # One call at a time. A second dispatch while the first is outstanding
        # is how replays used to lose frames, and it would also spend twice
        # for one answer.
        if not (self.async_cloud and self._inflight is not None):
            return False
        return self._answer(bb, self._local_scene(
            bb.perception, note=f"{self._inflight_trigger} call still in flight"))

    def _who_mission_ended(self, bb: "_RouteDecision") -> bool:
        if not (self._closed or self._epoch != bb.epoch):
            return False
        return self._answer(bb, self._local_scene(bb.perception, note="mission ended"))

    def _who_ask_local(self, bb: "_RouteDecision") -> bool:
        if bb.local_fn is None:
            return False
        return self._answer(bb, self._ask_local_first(
            bb.frame, bb.perception, bb.trigger, bb.epoch, bb.local_fn))

    def _who_ask_cloud(self, bb: "_RouteDecision") -> bool:
        frame, perception, trigger, frame_epoch = bb.frame, bb.perception, bb.trigger, bb.epoch
        escalated = ("disagreement" if trigger == TRIGGER_DISAGREE
                     else "unable" if self.local_sighting_fn is not None else None)

        self.stats.cloud_calls += 1
        self.stats.triggers[trigger] = self.stats.triggers.get(trigger, 0) + 1
        self._absent_streak = 0
        self._since_call = 0
        self._mark_odometry_call()

        if self.async_cloud:
            if not self._dispatch(frame, trigger, frame_epoch):
                # 3.76: close() won the race: nothing went out, so nothing
                # is counted as a call.
                self.stats.cloud_calls -= 1
                self.stats.triggers[trigger] -= 1
                return self._answer(bb, self._local_scene(perception, note="mission ended"))
            if escalated:
                # 3.80.D: handed up after `local_max_unable` unable answers;
                # 3.91.C: or the tiers disagree.
                self._count("frame_escalations", escalated)
            return self._answer(bb, self._local_scene(perception, note="dispatched",
                                                      dispatched=trigger))

        if escalated:
            self._count("frame_escalations", escalated)
        _t0 = time.perf_counter()
        try:
            scene = self.cloud_vision_fn(frame)
        except Exception as exc:  # noqa: BLE001 -- raised for B3.2 unless 3.80.G says not
            self._note_cloud_answer(trigger, _frame_seq(frame), error=exc)
            if self.cloud_errors_fatal and trigger != TRIGGER_DISAGREE:
                # 3.91.C (review): the disagreement question is extra to dev's
                # ladder; offline it must not spend B3.2's budget on a mission
                # the local tier would carry.
                raise
            self._count_cloud_error(exc)
            return self._answer(bb, self._local_scene(perception, note="cloud call failed"))
        self.stats.record("cloud_ms", (time.perf_counter() - _t0) * 1000)
        # The goal the stand-in holds on later free frames. Set on BOTH
        # paths: it lived only in _collect_inflight() at first, so a
        # synchronous mission had nothing to hold and silently kept
        # scanning -- the exact behaviour Phase F exists to remove.
        self._last_cloud_scene = scene
        self._local_held = None
        self._local_held_why = None
        self._local_turn = None     # 3.80.D: this answer is newer
        self._frame_unable = 0
        self._learn_from_cloud(scene, perception, self._pose_now(), self._episode)
        self._remask(perception)
        return self._answer(bb, self._annotate_noted(scene, perception, trigger, frame))

    # -- 3.80.D: a frame's question, to the local tier first --------------

    def _local_first(self):
        """The local tier's sighting call, or None when this question is
        the cloud's: no local tier, or it was unable `local_max_unable`
        times in a row."""
        fn = self.local_sighting_fn
        if fn is None:
            return None
        if self._frame_unable >= self.local_max_unable:
            # With the call cap spent there is no cloud to hand it to: the
            # free local tier keeps the question (review, 2026-10-14).
            spent = self.max_calls is not None and self.stats.cloud_calls >= self.max_calls
            return fn if spent else None
        return fn

    def _rearm(self, trigger: str) -> None:
        """A one-shot trigger (candidate, band, start) whose question was
        not answered fires again on the next frame that raises it."""
        if trigger == TRIGGER_CANDIDATE:
            self._last_confirmed = None
        elif trigger == TRIGGER_BAND:
            self._band_asked = False
        elif trigger == TRIGGER_START:
            self._retry_start = True
        elif trigger == TRIGGER_DISAGREE:
            self._disagree_key = None

    def _count(self, bucket: str, key: str) -> None:
        with self._stats_lock:
            d = getattr(self.stats, bucket)
            d[key] = d.get(key, 0) + 1

    def _ask_local_first(self, frame: dict, perception: Perception, trigger: str,
                         epoch: int, fn) -> dict:
        """Put this frame's question to the local tier (and, if it is
        absent, to the cloud about the same frame). The pacing counts are
        reset when a DECIDED answer lands (`_apply_local`), not here: an
        unable answer leaves the question open."""
        with self._local_lock:
            arrival_out = self._local_inflight is not None
        if arrival_out:
            # The arrival's identity call (one the runner gave up on, still
            # out) holds the VLM: a question now comes back `busy` and is
            # counted unable, and two of those bought a paid cloud call
            # (3.76 K3). Not asked; still open, so asked once it ends -- a
            # one-shot trigger's edge, spent in _trigger_for, armed again.
            self._rearm(trigger)
            return self._local_scene(perception, note="local VLM busy with the arrival check")
        self._count("local_triggers", trigger)
        q = _Question(
            trigger=trigger, epoch=epoch,
            pose=(Pose(self._odom.pose.x, self._odom.pose.y, self._odom.pose.heading_deg)
                  if (self._odometry_usable and self._odom.samples) else None),
            pan_deg=float(frame.get("pan_deg") or 0.0),
            hfov_deg=float(getattr(self.pipeline, "hfov_deg", None) or DEFAULT_HFOV_DEG),
            detected=perception.status == DETECTED,
            band=self._band_run > 0,
            episode=self._episode,
            box_share=perception.box_share if perception.status == DETECTED else None,
            # An absent local tier hands this to the cloud, which an
            # out-of-vocabulary question earns only at cold_search's bar.
            # 3.80.P: a cold search asked at the local pace reaches the
            # cloud only at the cloud's own bar.
            cloud_ok=(trigger != TRIGGER_TRACK
                      and (trigger not in (TRIGGER_OOV, TRIGGER_COLD_SEARCH)
                           or self._absent_streak >= self._cloud_cold_search_bar()
                           or self._distance_bar_reached())))
        if self.async_cloud:
            self._inflight_question = q
            if not self._dispatch(frame, trigger, epoch, tier="local",
                                  job=lambda: self._ask_tiers(frame, q, fn)):
                self._inflight_question = None
                with self._stats_lock:   # close() won: nothing was asked
                    self.stats.local_triggers[trigger] -= 1
                return self._local_scene(perception, note="mission ended")
            return self._local_scene(perception, note="asked the local tier",
                                     local_asked=trigger)
        try:
            got = self._ask_tiers(frame, q, fn)
        except Exception as exc:  # noqa: BLE001 -- the absent tier's cloud call
            # Only the cloud's call raises out of _ask_tiers: it was made, so
            # its pacing starts again, as the main path's does before its
            # call -- else a failing cloud is retried on every frame (review
            # of 3.80.P: 55 calls in 60 frames against 10).
            self._absent_streak = 0
            self._since_call = 0
            self._mark_odometry_call()
            self._note_cloud_answer(trigger, _frame_seq(frame), error=exc, via_local=True)
            if self.cloud_errors_fatal:
                raise
            self._count_cloud_error(exc)
            return self._local_scene(perception, note="cloud call failed")
        if isinstance(got, _LocalAnswer) or got is None:
            if got is not None:
                self._apply_local(got)
                self._remask(perception)
            return self._local_scene(perception, note="asked the local tier",
                                     local_asked=trigger)
        # The local tier was absent and the cloud answered about this frame.
        self._absent_streak = 0
        self._since_call = 0
        self._mark_odometry_call()
        self._last_cloud_scene = got
        self._local_held = None
        self._local_held_why = None
        self._local_turn = None
        self._frame_unable = 0
        self._learn_from_cloud(got, perception, q.pose, q.episode)
        self._remask(perception)
        return self._annotate_noted(got, perception, trigger, frame, via_local=True)

    def _ask_tiers(self, frame: dict, q: "_Question", fn):
        """Runs on the worker (or inline, synchronously). The local answer,
        or -- the local tier absent -- the cloud's scene about the same
        frame; None if the mission moved on. A cloud failure raises, for
        B3.2, as a trigger's always has."""
        if q.epoch != self._epoch:
            q.local_done.set()
            return None
        started = time.perf_counter()
        direction = None
        try:
            got, error = (fn(frame, box=True) or {}), None
            with self._stats_lock:
                self.stats.record("local_frame_ms", (time.perf_counter() - started) * 1000)
            got = got if isinstance(got, dict) else {}
            # 3.80.F: still the LOCAL phase -- the VLM is not free for the
            # arrival's question until the direction is answered too.
            direction = self._ask_direction(frame, q, got)
        except Exception as e:  # noqa: BLE001 -- LocalVLMAbsent, or a fault in the client
            got, error = None, e
            with self._stats_lock:
                self.stats.record("local_frame_ms", (time.perf_counter() - started) * 1000)
        finally:
            q.local_done.set()
        if error is None:
            return _LocalAnswer(q, got, direction=direction)
        logger.warning("local VLM absent for %s (%s); asking the cloud", q.trigger, error)
        with self._stats_lock:
            if q.epoch != self._epoch or self._closed:
                return None   # a Stop while the local tier was asked: nothing paid
            s = self.stats
            s.local_frame["absent"] = s.local_frame.get("absent", 0) + 1
            if (not q.cloud_ok
                    or (self.max_calls is not None and s.cloud_calls >= self.max_calls)):
                return _LocalAnswer(q, {}, absent=True)
            s.cloud_calls += 1
            s.triggers[q.trigger] = s.triggers.get(q.trigger, 0) + 1
            s.frame_escalations["absent"] = s.frame_escalations.get("absent", 0) + 1
        started = time.perf_counter()
        try:
            return self.cloud_vision_fn(frame)
        finally:
            with self._stats_lock:
                self.stats.record("cloud_ms", (time.perf_counter() - started) * 1000)

    def _count_cloud_error(self, exc: BaseException) -> None:
        logger.warning("cloud call failed (not fatal under an explore search): %s", exc)
        with self._stats_lock:
            self.stats.cloud_errors += 1
        # A cloud that gave no answer is no better than a spent cap: the
        # free local tier gets the question back (3.76 K3). Only an answer
        # restarted the count, so offline the local VLM was never asked again.
        self._frame_unable = 0

    def _ask_direction(self, frame: dict, q: "_Question", got: dict) -> Optional[dict]:
        """3.80.F, on the worker: the direction question about the same
        frame, when this is a search question whose sighting answer is not
        a yes. None when not asked."""
        # Only after a reply the VLM finished: a `timeout` or `busy` one is
        # still draining on the driver, and the direction would wait out its
        # own deadline behind it (review, 2026-10-16).
        if (self.local_direction_fn is None or q.trigger not in SEARCH_TRIGGERS
                or _local_kind(got, self.local_no_below, self.local_yes_at) == "yes"
                or got.get("status") not in ("answered", "unparsed")
                or q.detected or q.epoch != self._epoch or self._closed):
            return None
        started = time.perf_counter()
        try:
            reply = self.local_direction_fn(frame)
            reply = reply if isinstance(reply, dict) else {}
        except Exception as e:  # noqa: BLE001 -- LocalVLMAbsent, or a fault in the client
            logger.warning("local VLM absent for the direction (%s)", e)
            reply = {"status": "absent"}
        with self._stats_lock:
            self.stats.record("local_direction_ms", (time.perf_counter() - started) * 1000)
        return reply

    def _apply_direction(self, reply: dict, unable_before: int) -> Optional[str]:
        """3.80.F: a direction reply that landed. Returns the held action,
        or None. An unable reply counts the search question once as unable
        for the escalation rule (`unable_before` is the count before its
        sighting answer was applied); the next search question comes at the
        pace that answer set. A reply with no direction to give
        (DIRECTION_NONE) changes nothing."""
        word = reply.get("direction")
        action = (DIRECTION_ACTIONS.get(word) if reply.get("status") == "answered"
                  else None)
        status = reply.get("status")
        kind = (word if action else status if status in DIRECTION_NONE
                else f"unable_{status}")
        self._count("local_direction", kind)
        if action is None and status in DIRECTION_NONE:
            return None
        if action is None:
            self._frame_unable = unable_before + 1
            return None
        # The search question is answered: its pacing starts again, as on a
        # decided sighting answer (bug pass, 2026-10-16: an answered direction
        # after an unable sighting left the question open and re-asked on the
        # next frame, and the unable count never reached the cloud).
        self._frame_unable = 0
        self._absent_streak = 0
        self._since_call = 0
        self._retry_start = False
        self._mark_odometry_call()
        self._local_held = action
        self._local_held_why = "direction"
        self._local_turn = None
        return action

    def _apply_local(self, ans: "_LocalAnswer") -> None:
        """A local answer that landed: count it, and act on it."""
        if ans.q.epoch != self._epoch:
            return
        trigger = ans.q.trigger
        if ans.absent:
            # Absent, and the cloud not to be asked (the call cap spent, or an
            # out-of-vocabulary question before cold_search's bar): nothing
            # answered, nothing paid.
            self._local_landed = {"trigger": trigger, "answer": "absent", "score": None}
            if self.max_calls is not None and self.stats.cloud_calls >= self.max_calls:
                # 3.91.C (review): the local tier gone and the cap spent -- no
                # question can clear probation now, as with no local tier.
                self._end_probation()
            return
        got = ans.got
        unable_before = self._frame_unable
        status, score = got.get("status"), got.get("score")
        numeric = isinstance(score, (int, float))
        kind = _local_kind(got, self.local_no_below, self.local_yes_at)
        with self._stats_lock:
            self.stats.local_frame_calls += 1
            self.stats.local_frame[kind] = self.stats.local_frame.get(kind, 0) + 1
        bearing = None
        if kind in ("yes", "no"):
            self._frame_unable = 0
            self._absent_streak = 0
            self._since_call = 0
            self._mark_odometry_call()
            if kind == "yes":
                # Outranked (a cloud no or probation stands): its box steers
                # nothing either; the cloud is asked when it is a probation.
                accepted = self._on_yes("local", ans.q.episode)
                bearing = self._bearing_of(ans) if accepted else None
                if bearing is not None:
                    self._local_turn = (
                        ("LEFT" if bearing < 0 else "RIGHT", self._turn_for(bearing))
                        if abs(bearing) > self.steer_band_deg else ("FORWARD", None))
                    self._local_held = "FORWARD"
                    self._local_held_why = "located"
                elif self._local_held_why == "direction":
                    # In view somewhere: an older search direction is moot.
                    self._local_held = self._local_held_why = None
            else:
                self._local_turn = None
                if self._local_held_why != "direction":
                    # A no drops the FORWARD a located yes held; a direction
                    # (3.80.F) is about where to look, and a no does not
                    # answer that.
                    self._local_held = self._local_held_why = None
                if ans.q.detected and self.remember_no:
                    # 3.91.C: about the candidate perception called the
                    # target -- remembered where Qwen can tell, else a
                    # disagreement the cloud may settle. Off, none of it
                    # (review: off is dev's behaviour, its stats too).
                    lo, hi = self.no_box_range
                    share = ans.q.box_share
                    if share is None or lo <= share <= hi:
                        self._reject("local", ans.q.pose, ans.q.episode)
                    elif self._cloud_yes_episode != ans.q.episode:
                        with self._reject_lock:
                            # Option (a) (the user, 2026-10-24): a no Qwen
                            # cannot vouch for goes up to the cloud, once per
                            # run; perception steers until it answers. Also on
                            # the cloud's probation, where the local no agrees:
                            # the cloud's renewed no is HELD, and outlives the
                            # next loss of the object, where a probation ends
                            # (round 4 skipped the call: the taupe replay then
                            # steered at the re-found bag, 2 reversals).
                            self._escalate_episode = ans.q.episode
                            if share < lo and self._probation_by == "local":
                                # Amendment 4 (the user, 2026-10-23/24): a far
                                # no ends a local probation -- the far view is
                                # where a real target would otherwise never be
                                # closed on. A fabric-only no keeps it: the
                                # robot is at that object already (the taupe
                                # replay: 2 reversals). The cloud's stands.
                                self._end_probation()
                        with self._stats_lock:
                            self.stats.cannot_tell += 1
                if ans.q.band:
                    # 3.80.F: the crop in view when it was asked is not the
                    # target. Rejected until the band run breaks (the crop
                    # leaves view or crosses the gate): the band question is
                    # not asked again in that run either.
                    self._candidate_rejected = True
        else:
            self._frame_unable += 1
            # The question stays open: asked again on the next frame that
            # raises it (pacing counts untouched, edges armed again).
            self._rearm(trigger)
        self._local_landed = {"trigger": trigger, "answer": kind,
                              "score": score if numeric else None,
                              "bearing_deg": None if bearing is None else round(bearing, 1)}
        if ans.direction is not None and kind != "yes":
            # A search question is answered only by a direction: an unable
            # one counts toward the cloud, whatever the sighting answer reset.
            self._local_landed["direction"] = self._apply_direction(
                ans.direction, unable_before if kind == "no" else self._frame_unable - 1)

    def _bearing_of(self, ans: "_LocalAnswer") -> Optional[float]:
        """Body bearing to the box's centre NOW: the bearing in the frame it
        was asked about, less the turn odometry measured since."""
        frac = ans.got.get("box_frac")
        try:
            x1, _, x2, _ = (float(v) for v in frac)
        except (TypeError, ValueError):
            return None
        bearing = ((x1 + x2) / 2 - 0.5) * ans.q.hfov_deg + ans.q.pan_deg
        if ans.q.pose is not None and self._odometry_usable and self._odom.samples:
            bearing = _wrap(ans.q.pose.heading_deg + bearing - self._odom.pose.heading_deg)
        return bearing

    def wait_inflight(self) -> None:
        """Wait for an async trigger call still out, and apply it. 3.56:
        MissionRunner calls this BEFORE the arrival confirmation's own (shorter)
        deadline starts, under B3.2's timeout -- the trigger's call has the
        triggers' deadline, and a healthy but slow one must not be charged to
        the confirmation. `confirm_arrival()` keeps the same wait for callers
        that do not do this first. With the call cap already spent there is
        nothing to wait for: `confirm_arrival()` refuses locally at once."""
        if self.max_calls is not None and self.stats.cloud_calls >= self.max_calls:
            return
        fut = self._inflight
        if fut is not None:
            try:
                fut.result()
            except BaseException:  # noqa: BLE001 -- _collect_inflight holds it
                pass
            self._collect_inflight()

    def abandon_inflight(self) -> None:
        """The runner gave up waiting for the call still out and has charged
        it to B3.2 already (3.56 follow-up). Mark it charged -- it stays in
        `_inflight`, so the one-call gate stays closed while it runs -- and
        its failure, whenever it lands, is never held for the next tick.

        If the abandoned wait already collected it (the call failed in the
        moment between the timeout and this), its error is dropped here: in
        this tick `__call__` has already raised anything collected before
        it, so a pending error now can only be that call's."""
        with self._collect_lock:
            self._charged = self._inflight or self._last_taken
            self._pending_error = None

    def cloud_only_confirmer(self):
        """3.80.O: the confirmer for an arrival the local VLM's box led to:
        the cloud alone, counted as guarded."""
        return functools.partial(self.confirm_cloud, guarded=True)

    def wait_local_free(self, timeout_s: float) -> bool:
        """3.80.O: wait up to `timeout_s` for a frame's question still holding
        the local VLM to finish its local phase. True when it is free."""
        q = self._inflight_question
        return q is None or q.local_done.wait(max(0.0, timeout_s))

    def set_local_track(self, on: bool) -> None:
        """3.80.O: explore says whether it is closing in on a target only the
        local VLM has seen; while it is, every frame perception does not
        detect the target asks the local VLM (`local_track`)."""
        self._local_track = bool(on)

    # -- 1a: the cloud confirms identity at arrival -----------------------

    def confirm_arrival(self, frame: dict, local_ok: bool = True) -> dict:
        """The arrival's identity question, through the tiers (3.80.C): the
        local VLM first, the cloud only when it is absent or has been unable
        `local_max_unable` times in this arrival. `MissionRunner` calls the
        two halves itself (`confirm_local`, then `confirm_cloud`), so each
        gets its own deadline."""
        if not local_ok:
            return self.confirm_cloud(frame, guarded=True)
        verdict = self.confirm_local(frame)
        return verdict if verdict is not None else self.confirm_cloud(frame)

    def next_confirm_is_local(self) -> bool:
        return self.local_identity_fn is not None and self._local_unable < self.local_max_unable

    def arrival_ended(self) -> None:
        """A frame on which the arrival rule does not hold. After
        `LOCAL_RESET_FRAMES` of them in a row the next arrival is a new
        question and the local tier gets its tries again; fewer is the same
        arrival flickering (a missed detection, a panned camera), which must
        not keep the question from ever reaching the cloud (bug pass,
        2026-10-14)."""
        with self._local_lock:
            self._frames_off_arrival += 1
            if self._frames_off_arrival >= LOCAL_RESET_FRAMES:
                self._local_unable = 0

    def begin_local(self) -> int:
        """Reserve the id of the next local call, so a caller that may give
        up on it (the runner's `abandon_local(call_id)`) names THAT call even
        if its thread has not started yet."""
        with self._local_lock:
            self._local_started += 1
            return self._local_started

    def confirm_local(self, frame: dict, call_id: Optional[int] = None) -> Optional[dict]:
        """The local VLM's verdict on the arrival frame, or None when the
        cloud must be asked instead (no local tier, it is absent, or it has
        already been unable `local_max_unable` times). Never raises: the
        local tier failing is never a mission failure.

        An unable answer is `undecided`: neither confirmed nor refused, and
        the agent asks again on the next arrival frame."""
        s = self.stats
        q = self._inflight_question
        if q is not None and self.next_confirm_is_local():
            # 3.80.D: a frame's question still holds the local VLM, which
            # serves one request at a time: asked now, the arrival would be
            # `busy` (review, 2026-10-14). Only on the local tier's turn, and
            # only for the question's LOCAL phase (bounded by the client's
            # timeout, and by the runner's budget); a cloud call it fell
            # through to is not waited for. The answer is left for the tick
            # thread to apply (review round 2).
            q.local_done.wait()
        with self._local_lock:
            # An arrival frame: the arrival is still on (arrival_ended's run
            # of frames off it starts again), whichever tier is asked.
            self._frames_off_arrival = 0
            if call_id is None:
                self._local_started += 1
                call_id = self._local_started
            if call_id in self._local_abandoned:
                # Given up on before its thread got here: never sent.
                self._local_abandoned.discard(call_id)
                return None
            if not self.next_confirm_is_local():
                self._local_done = (call_id, None)
                return None
            if self._local_inflight is not None:
                # The last local check outlived the runner's deadline and is
                # still out: one request at a time, so this frame sends none,
                # and it counts as unable (bug pass review, 2026-10-14).
                verdict = {"cloud_called": False, "local_called": False, "tier": "local",
                           "local_status": "busy", "local_score": None,
                           **self._count_unable("unable_inflight",
                                                "the previous local check is still running")}
                self._local_done = (call_id, verdict)
                return verdict
            self._local_inflight = call_id
        started = time.perf_counter()
        try:
            got = self.local_identity_fn(frame) or {}
            error = None
        except Exception as e:  # noqa: BLE001 -- LocalVLMAbsent, or a fault in the client
            got, error = None, e
        with self._local_lock:
            if self._local_inflight == call_id:
                self._local_inflight = None
            if call_id in self._local_abandoned:
                # The runner gave up on this call and already counted it
                # (abandon_local); a late answer changes nothing.
                self._local_abandoned.discard(call_id)
                return None
            s.record("local_ms", (time.perf_counter() - started) * 1000)
            verdict = self._local_verdict(got, error)
            self._local_done = (call_id, verdict)
            return verdict

    def _local_verdict(self, got: Optional[dict], error: Optional[BaseException]):
        s = self.stats
        if error is not None:
            s.local["absent"] = s.local.get("absent", 0) + 1
            logger.warning("local VLM absent at arrival (%s); asking the cloud", error)
            return None
        s.local_calls += 1
        status, score = got.get("status"), got.get("score")
        base = {"cloud_called": False, "local_called": True, "tier": "local",
                "local_status": status, "local_score": score}
        if status == "answered" and isinstance(score, (int, float)):
            if score >= LOCAL_YES_AT:
                s.local["yes"] = s.local.get("yes", 0) + 1
                return {**base, "confirmed": True, "stats": s.as_dict(),
                        "reason": f"the local model sees the target (score {score:.2f})"}
            if score <= self.local_no_below:
                s.local["no"] = s.local.get("no", 0) + 1
                return {**base, "confirmed": False, "stats": s.as_dict(),
                        "reason": f"the local model does not see the target "
                                  f"(score {score:.2f})"}
        kind = (f"unable_{status}" if status != "answered"
                else "unable_band" if isinstance(score, (int, float)) else "unable_noscore")
        what = (f"score {score:.2f}" if isinstance(score, (int, float))
                else (got.get("reason") or status or "no reply"))
        return {**base, **self._count_unable(kind, what)}

    def abandon_local(self, call_id: Optional[int] = None) -> Optional[dict]:
        """The runner stopped waiting for `confirm_local` (its share of
        B3.2's deadline ran out): counted here as one unable answer, and the
        call's late answer, whenever it returns, is ignored. A call that
        finished in the moment between the timeout and this returns its own
        verdict instead, counted once."""
        with self._local_lock:
            cid = self._local_started if call_id is None else call_id
            if self._local_done is not None and self._local_done[0] == cid:
                return self._local_done[1]
            self._local_abandoned.add(cid)
            if self._local_inflight != cid:
                # Its thread never got as far as sending: not a local call,
                # but the arrival waited its share all the same.
                return {"cloud_called": False, "local_called": False, "tier": "local",
                        "local_status": "timeout", "local_score": None,
                        **self._count_unable("unable_not_started",
                                             "the local check did not start in time")}
            self.stats.local_calls += 1
            return {"cloud_called": False, "local_called": True, "tier": "local",
                    "local_status": "timeout", "local_score": None,
                    **self._count_unable("unable_runner_timeout",
                                         "no answer inside the mission's deadline")}

    def _count_unable(self, kind: str, what: str) -> dict:
        s = self.stats
        s.local[kind] = s.local.get(kind, 0) + 1
        self._local_unable += 1
        last = self._local_unable >= self.local_max_unable
        return {"confirmed": False, "undecided": True, "stats": s.as_dict(),
                "reason": (f"the local model could not decide ({what}); "
                           + ("asking the cloud next" if last else "asking again"))}

    def confirm_cloud(self, frame: dict, guarded: bool = False) -> dict:
        """One paid, synchronous cloud call on the ARRIVAL frame: is the
        target in it? (Handoff 2026-10-02 1a, decided by the user.)

        `brain/arrival.py` decides DISTANCE from the lidar; this decides
        IDENTITY, which is the cloud's question by 1.11's split. Only an
        explicit `target_visible: true` confirms -- a scene that does not
        say, or a call cap already spent, is a refusal, because the cost of
        a missed `found` is a few more steps and the cost of a wrong one is
        the mission. Synchronous even under `async_cloud`: the mission is
        about to end on the answer, so there is nothing to hold meanwhile.
        A failure raises, and the runner counts it against B3.2's budget.
        """
        if self.max_calls is not None and self.stats.cloud_calls >= self.max_calls:
            return {"confirmed": False, "cloud_called": False,
                    "reason": "call cap reached -- identity not confirmed",
                    "stats": self.stats.as_dict()}
        # One call in flight at a time, this one included (spec review 3,
        # fix 8): under `async_cloud` a trigger call may still be out. Wait
        # for it and apply it first -- the confirmation already blocks the
        # tick, and the runner's per-call timeout bounds the whole wait.
        fut = self._inflight
        if fut is not None:
            try:
                fut.result()
            except BaseException:  # noqa: BLE001 -- _collect_inflight holds it
                pass
            self._collect_inflight()
        self.stats.cloud_calls += 1
        self.stats.triggers[TRIGGER_ARRIVAL] = self.stats.triggers.get(TRIGGER_ARRIVAL, 0) + 1
        with self._local_lock:
            self._frames_off_arrival = 0
        if guarded:
            # 3.80.O: an arrival the local VLM's box led to, never put to it.
            self.stats.guarded += 1
        elif self.local_identity_fn is not None and not self.next_confirm_is_local():
            # 3.80.C: an arrival the local tier could not decide, handed up.
            self.stats.escalations += 1
        started = time.perf_counter()
        try:
            scene = (self.confirm_vision_fn or self.cloud_vision_fn)(frame) or {}
        finally:
            self.stats.record("cloud_ms", (time.perf_counter() - started) * 1000)
        nav = scene.get("_navigate") or {}
        confirmed = nav.get("target_visible") is True
        return {"confirmed": confirmed, "cloud_called": True, "tier": "cloud",
                "reason": ("the cloud sees the target in the arrival frame" if confirmed
                           else "the cloud does not see the target in the arrival frame"),
                "cloud_reasoning": nav.get("reasoning"),
                # The frame's `_tier` snapshot was taken before this call; the
                # agent refreshes it from here so status and metrics count it.
                "stats": self.stats.as_dict()}

    # -- Phase A: dispatch, collect, and hold the goal --------------------

    def _dispatch(self, frame: dict, trigger: str, epoch: Optional[int] = None,
                  tier: str = "cloud", job: Optional[Callable] = None) -> bool:
        """Start the cloud call on a worker thread and return immediately.

        3.76: `epoch` is the frame's, taken when the frame began, so a
        close() landing at any point after that makes the worker skip the
        call; and the pool is created under the lock close() takes, so a
        close() is never followed by a new pool nothing shuts down."""
        if epoch is None:
            epoch = self._epoch
        with self._executor_lock:
            if self._closed:
                return False
            if self._executor is None:
                # One worker, created lazily so a synchronous mission (the
                # default) never starts a thread at all.
                self._executor = ThreadPoolExecutor(
                    max_workers=1, thread_name_prefix="tiered-cloud")
            self._inflight_trigger = trigger
            self._inflight_tier = tier
            # Corroboration (1.11a) compares the cloud's claim against the
            # LOCAL evidence from the same frame. Async breaks that pairing
            # unless the perception is carried along with the call, so it is.
            self._inflight_perception = self._perception_this_frame
            self._inflight_seq = _frame_seq(frame)
            # 3.91.C: where the body was and which candidate was in view.
            self._inflight_ask = (self._pose_now(), self._episode)
            self._inflight = (self._executor.submit(job) if job is not None
                              else self._executor.submit(self._call_cloud, frame, epoch))
        return True

    def _call_cloud(self, frame: dict, epoch: int) -> Optional[dict]:
        """Runs on the worker. Returns None if the mission moved on.

        The epoch check is here as well as at collection because the two
        answer different questions: this one avoids doing work nobody
        wants, and the one in `_collect_inflight` is the guard that
        actually matters -- it is what stops a superseded answer being
        applied.
        """
        if epoch != self._epoch:
            return None
        # Timed here, on the worker, so this is the CALL's duration and not
        # how long the collector happened to take to notice. Under async the
        # two differ by up to a frame, and the number people ask for is the
        # first one.
        started = time.perf_counter()
        try:
            return self.cloud_vision_fn(frame)
        finally:
            self.stats.record("cloud_ms", (time.perf_counter() - started) * 1000)

    def _collect_inflight(self) -> None:
        """Apply a landed answer, or note that it failed. Never waits.

        The take is atomic (3.56 follow-up): a `wait_inflight()` thread the
        runner abandoned at its deadline can call this at the same moment as
        the tick thread, and only one of them may own the landed call --
        else an answer is applied twice, or one failure counted twice."""
        with self._collect_lock:
            fut = self._inflight
            if fut is None or not fut.done():
                return
            self._inflight = None
            self._last_taken = fut
            # 3.91 A: noted at the mission's end already (`take_cloud_answers`).
            already_noted = fut is self._unapplied_noted
            trigger, self._inflight_trigger = self._inflight_trigger, None
            # 3.91 A: a local-first job hands back the local answer (applied
            # below, never noted) or the cloud's, asked when the local tier was
            # absent; only the cloud's call raises out of it (`_ask_tiers`).
            via_local = self._inflight_tier == "local"
            self._inflight_tier = None
            self._inflight_question = None
            perception, self._inflight_perception = self._inflight_perception, None
            asked_seq, self._inflight_seq = self._inflight_seq, None
            ask, self._inflight_ask = self._inflight_ask, None
        if fut.exception() is None and isinstance(fut.result(), _LocalAnswer):
            # 3.80.D: the local tier's answer; its own rules reset the pacing.
            self._apply_local(fut.result())
            return
        # 3.76 (the user, 2026-10-13): pacing counts from when the cloud
        # ANSWERED, not from when it was asked. Every count kept running while
        # a slow call was out -- absent frames, frames since the call, distance
        # -- so the frame after it landed was already past a bar and a second
        # paid call went out back to back.
        self._absent_streak = 0
        self._since_call = 0
        self._mark_odometry_call()
        try:
            scene = fut.result()
        except BaseException as exc:  # noqa: BLE001
            # Held, not raised here: this runs before perception, and a
            # failure surfaced mid-frame would skip the local tier's work
            # for that frame. Raised at the top of the NEXT call, where
            # B3.2's budget sees it as it always did -- unless the runner
            # already charged this call when its wait timed out
            # (`abandon_inflight`). Decided under the lock, against it.
            logger.warning("async cloud call failed: %s", exc)
            if not already_noted:
                self._note_cloud_answer(trigger, asked_seq, error=exc, via_local=via_local)
            if trigger == TRIGGER_DISAGREE:
                # 3.91.C (review): never B3.2's, as on the synchronous path --
                # unless the runner charged it already (a hang at arrival),
                # then not counted twice.
                with self._collect_lock:
                    charged = fut is self._charged
                if not charged:
                    self._count_cloud_error(exc)
                return
            with self._collect_lock:
                if fut is not self._charged:
                    self._pending_error = exc
            return
        if scene is None:
            # Superseded: the mission ended or reset while this was out.
            # Dropping it silently is the point -- applying it is the
            # orphaned-call bug.
            return
        self._last_cloud_scene = scene
        self._local_held = None
        self._local_held_why = None
        self._frame_unable = 0      # 3.80.D: a cloud answer starts the count again
        pose, episode = ask or (None, self._episode)
        self._learn_from_cloud(scene, perception, pose, episode)
        # Run the annotation for its SIDE EFFECTS -- corroboration and the
        # stats 6.3 puts on the panel -- against the perception this call
        # was actually made on, not whatever is in front of the camera now.
        # The annotated scene itself is not returned: the decision for this
        # frame belongs to this frame's pixels, and what carries forward is
        # the goal, not the whole answer.
        verdict = None
        try:
            if perception is not None:
                annotated = self._annotate(scene, perception, trigger or "", executed=False)
                verdict = (annotated.get("_tier") or {}).get("corroboration")
        finally:
            # Noted even if the annotation raised: the answer did land.
            if not already_noted:
                self._note_cloud_answer(trigger, asked_seq, scene=scene,
                                        verdict=(verdict or {}).get("verdict"),
                                        via_local=via_local)
        if perception is not None:
            if verdict:
                # Marked `landed_late`, and carrying the trigger it belongs
                # to, so a reader can never take it for a verdict about the
                # frame in front of the camera now.
                self._landed_corroboration = {**verdict,
                                              "landed_late": True,
                                              "for_trigger": trigger}

    def reset_epoch(self) -> None:
        """Invalidate anything in flight. Called when a mission ends, so a
        late answer cannot be applied to a robot that has stopped."""
        self._epoch += 1
        q = self._inflight_question
        if q is not None:
            # A question cancelled before it ran never sets this itself; an
            # arrival waiting on it must not wait for ever (review round 3).
            q.local_done.set()
        with self._collect_lock:
            self._inflight = None
            self._inflight_trigger = None
            self._inflight_tier = None
            self._inflight_question = None
            self._inflight_perception = None
            self._inflight_seq = None
            self._inflight_ask = None
            # 3.91 A: the runner takes what landed before it closes the policy;
            # nothing of this mission may ride out on another.
            self._cloud_answers = []
            self._unapplied_noted = None
        self._landed_corroboration = None
        self._local_turn = None
        self._local_landed = None
        self._pending_error = None
        with self._reject_lock:
            self._rejection = None
            self._end_probation()
            self._masked = False
            self._escalate_episode = None
            self._cloud_yes_episode = None
        self._disagree_key = None

    def close(self) -> None:
        """Release the worker thread. Safe to call more than once.

        3.56: also drops the arrival confirmation's short-deadline client.
        It exists for the arrival TICK, with a robot waiting; once the mission
        has ended (MissionRunner closes the policy in `_finish`), 3.53's late
        check asks through the triggers' client and B3.2's full deadline."""
        with self._executor_lock:
            self._closed = True
            self.reset_epoch()
            self.confirm_vision_fn = None
            # 3.81.E: the models go with the mission. A frame already in
            # perception finishes on the pipeline it started with.
            self.pipeline = RELEASED_PIPELINE
            # 3.80.C: the local tier answers only a live arrival; the late
            # check (3.53) is the cloud's.
            local, self.local_identity_fn = self.local_identity_fn, None
            self.local_sighting_fn = None
            self.local_direction_fn = None
            if local is not None and hasattr(local, "close"):
                try:
                    local.close()
                except Exception as e:  # noqa: BLE001
                    logger.debug("closing the local VLM client: %s", e)
            if self._executor is not None:
                self._executor.shutdown(wait=False, cancel_futures=True)
                self._executor = None

    def _annotate_noted(self, scene: dict, perception: Perception, trigger: str,
                        frame: dict, via_local: bool = False) -> dict:
        """A synchronous answer, annotated for this frame and noted even if
        the annotation raises -- the answer landed either way."""
        verdict = None
        try:
            out = self._annotate(scene, perception, trigger)
            verdict = (out["_tier"].get("corroboration") or {}).get("verdict")
            return out
        finally:
            self._note_cloud_answer(trigger, _frame_seq(frame), scene=scene, verdict=verdict,
                                    via_local=via_local)

    def _note_cloud_answer(self, trigger, asked_seq, scene: Optional[dict] = None,
                           verdict: Optional[str] = None,
                           error: Optional[BaseException] = None,
                           via_local: bool = False) -> None:
        """3.91 A: what the cloud said, kept for the walk's rows. Recorded
        whatever the answer does next, so a reader can tell a held REVERSE
        that meant "too close" from one that meant "not the target"."""
        if error is not None:
            # The type and an HTTP status only, never the message: walk rows
            # go to S3, and an error's text can quote the endpoint, a body or
            # a host in forms no scrub keeps up with (review rounds 2-3).
            entry = {"trigger": trigger, "asked_seq": asked_seq,
                     "error": type(error).__name__}
            status = _http_status(error)
            if status is not None:
                entry["status"] = status
        else:
            # Shapes are checked, not assumed: this runs inside `finally`
            # blocks, where raising would replace the call's own outcome.
            scene = scene if isinstance(scene, dict) else {}
            # `action` is what the brain acts on; `reply` is the rest of the
            # answer as the scene carries it, made safe to store whole: under
            # its own key so it can never write the entry's fields, and kept
            # whole so no field is lost to a hand-kept list (review).
            entry = {"trigger": trigger, "asked_seq": asked_seq, "verdict": verdict,
                     "action": scene.get("safest_direction"),
                     "reply": _json_safe(scene.get("_navigate"))}
        if via_local:
            entry["via_local_tier"] = True     # asked because the local tier was absent
        with self._collect_lock:
            self._cloud_answers.append(entry)
            if len(self._cloud_answers) > CLOUD_ANSWERS_KEPT:
                # Bounded whoever takes them: the oldest go, counted.
                del self._cloud_answers[0]
                self.stats.cloud_answers_dropped += 1

    def _held_direction(self) -> Optional[str]:
        """The last direction the cloud gave, if there is one.

        2.5's *"the reactive tier always holds a current goal"*. Used while
        a call is in flight and, under `hold_goal` (shipped `true`), on
        every free frame -- see `_local_scene()`.
        """
        if self._local_held:
            # 3.80.D: a local yes that said where, newer than the cloud's goal.
            return self._local_held
        if not self._last_cloud_scene:
            return None
        return self._last_cloud_scene.get("safest_direction")

    # -- the trigger policy ----------------------------------------------

    def _count_skipped(self, frame: dict) -> None:
        """3.90 E: frames the robot had between two that perception took
        (its seq jumped) were skipped -- perception was busy. Decisions take
        the newest frame on purpose; this says how many it passed over."""
        seq = (frame.get("metadata") or {}).get("seq")
        self._repeat_frame = isinstance(seq, int) and seq == self._last_seq
        if not isinstance(seq, int):
            return
        if self._last_seq is not None and seq > self._last_seq + 1:
            self.stats.frames_skipped += seq - self._last_seq - 1
        self._last_seq = seq

    def _note_vote(self, status: str) -> None:
        """3.90 E: every frame perception took enters the vote's window --
        tracking and start frames too (review) -- except the same frame read
        a second time, which is one look, not two."""
        if not self.frame_vote or getattr(self, "_repeat_frame", False):
            return
        self._votes.append((self._frame_stamp, status == DETECTED))
        del self._votes[:-self.frame_vote[1]]

    def _voted(self) -> bool:
        """3.90 E: k detected of the last n frames perception took, all of
        them inside n ticks of time when stamped (a skipped frame neither
        votes nor breaks the window)."""
        k, n = self.frame_vote
        window = self._votes
        if self._frame_stamp is not None:
            window = [v for v in window
                      if v[0] is not None and 0 <= self._frame_stamp - v[0] <= n * VOTE_TICK_S]
        detected = sum(1 for _, d in window if d)
        self.stats.vote["detected"] = detected
        return detected >= k

    def _note_run(self, status: str) -> None:
        """One frame into the run of recent statuses (`consecutive_frames`
        long), and when the current same-status run began (3.90 A)."""
        if not self._run or self._run[-1] != status or self._frame_stamp is None:
            self._run_since = self._frame_stamp
        self._run.append(status)
        if len(self._run) > self.consecutive_frames:
            self._run.pop(0)

    def _run_spans(self) -> bool:
        """3.90 A: a run of stamped frames must also cover
        `sighting_span_s`, so the rule means the same time at any frame
        rate. Unstamped frames, or no span set: counted only, as before."""
        return run_spans(self._run_since, self._frame_stamp, self.sighting_span_s)

    def _trigger_for(self, perception: Perception) -> Optional[str]:
        # 3.94.B: when to ask is a tree (`_build_when_tree`); the code it
        # replaced is in `tests/tiered_reference.py`.
        bb = _TriggerDecision(perception=perception)
        self._when_tree.tick(bb)
        return bb.trigger

    # -- 3.94.B: when to ask, as a tree -----------------------------------

    def _build_when_tree(self) -> Node:
        """The trigger policy, drawn as a tree:

        * the frame vote tallied, on every frame before anything returns;
        * then the first of: the opening question (the first frame); a frame
          with no picture asking nothing; the opening question again (it went
          unanswered); and this frame's evidence followed by the events --
          local tracking, the candidate edge, then for an absent frame the
          uncertain band, out of vocabulary and cold search (frames or
          metres), then staleness, the floor that never pre-empts an event
          that says more.
        """
        events_by_name = {
            "band": Action("the uncertain band", self._tr_band,
                           lambda: {"band_low": self.local_band_low,
                                    "gate": self.match_probability,
                                    "frames": self.consecutive_frames}),
            "oov": Action("out of vocabulary", self._tr_oov),
            "cold_frames": Action("cold search, frames", self._tr_cold_frames,
                                  lambda: {"after": self._cold_search_bar()}),
            "cold_metres": Action("cold search, metres", self._tr_cold_metres,
                                  lambda: {"after_cm": self.cold_search_after_cm}),
        }
        absent = Sequence("an absent frame", [
            Condition("the target is absent", lambda bb: bb.perception.status == ABSENT),
            Action("count the absent frame", self._tr_count_absent),
            # 3.94.D: the order is `tier_tree.absent_events`; one left out
            # never fires. With none, an absent frame goes on to staleness.
            Fallback("absent events", [events_by_name[n] for n in self.absent_events]
                     or [Condition("none configured", lambda bb: False)]),
        ])
        events = Fallback("events", [
            Action("local tracking", self._tr_track),
            Action("candidate sighting", self._tr_candidate,
                   lambda: {"frames": self.consecutive_frames, "vote": self.frame_vote}),
            Action("the tiers disagree", self._tr_disagree),
            Sequence("after the edges", [
                Action("the absent edge", self._tr_absent_edge),
                Fallback("absent, then the floor", [
                    absent,
                    Action("staleness", self._tr_stale, lambda: {"after": self.stale_after}),
                    Action("no trigger", self._tr_none),
                ]),
            ]),
        ])
        return Sequence("when to ask", [
            Action("tally the vote", self._tr_vote),
            Fallback("the first trigger that fires", [
                Action("the opening question", self._tr_start),
                Action("no picture, no question", self._tr_unavailable),
                Action("the opening question again", self._tr_retry_start),
                Action("a rejected candidate asks nothing", self._tr_rejected,
                       lambda: {"remember_no": self.remember_no}),
                Sequence("this frame's evidence, then the events", [
                    Action("track the band", self._tr_track_band),
                    Action("note the run", self._tr_note_run),
                    Action("confirm the run", self._tr_confirm),
                    events,
                ]),
            ]),
        ])

    def describe_when_tree(self) -> str:
        return "\n".join(self._when_tree.describe())

    def _fire(self, bb: "_TriggerDecision", trigger: Optional[str]) -> bool:
        bb.trigger = trigger
        return True

    def _tr_vote(self, bb: "_TriggerDecision") -> bool:
        if bb.perception.status != UNAVAILABLE:
            self._note_vote(bb.perception.status)
            # Tallied on every frame, before any early return (review: the
            # tracking and start returns left the HUD's count stale).
            self._vote_ok = self._voted() if self.frame_vote else False
        return True

    def _tr_start(self, bb: "_TriggerDecision") -> bool:
        if self._started:
            return False
        self._started = True
        # **The opening call is also the first frame's confirmation.** Seeding
        # the edge detector from the start frame is what stops a
        # `candidate_sighting` a frame later buying a second answer to the
        # question just asked about the same view.
        if bb.perception.status in (DETECTED, ABSENT):
            self._note_run(bb.perception.status)
            self._last_confirmed = bb.perception.status
        return self._fire(bb, TRIGGER_START)

    def _tr_unavailable(self, bb: "_TriggerDecision") -> bool:
        # `unavailable` is never an event: no trigger, and no cold-search
        # streak -- a wedged camera would otherwise buy a cloud call every
        # few frames and look like a search.
        if bb.perception.status != UNAVAILABLE:
            return False
        self._run.clear()
        self._run_since = None
        self._votes.clear()
        self._vote_ok = False
        if self.stats.vote:
            self.stats.vote["detected"] = 0
        self._band_run = 0          # 3.80.D: any other frame breaks a band run
        self._band_asked = False
        self._candidate_rejected = False
        return self._fire(bb, None)

    def _tr_retry_start(self, bb: "_TriggerDecision") -> bool:
        if not self._retry_start:   # 3.80.D: the opening question went unanswered
            return False
        self._retry_start = False
        return self._fire(bb, TRIGGER_START)

    def _tr_track_band(self, bb: "_TriggerDecision") -> bool:
        # 3.80.D: the uncertain band, tracked only with a local tier.
        p = bb.perception
        bb.band = (self.local_sighting_fn is not None and p.status == ABSENT
                   and bool(p.candidates)
                   and self.local_band_low
                   <= max(c.probability for c in p.candidates)
                   < self.match_probability)
        if bb.band:
            self._band_run += 1
        else:
            self._band_run = 0
            self._band_asked = False
            self._candidate_rejected = False
        return True

    def _tr_note_run(self, bb: "_TriggerDecision") -> bool:
        self._note_run(bb.perception.status)
        return True

    def _tr_confirm(self, bb: "_TriggerDecision") -> bool:
        bb.confirmed = (
            bb.perception.status
            if (len(self._run) >= self.consecutive_frames
                and len(set(self._run)) == 1
                and self._run_spans())
            else None
        )
        return True

    def _tr_track(self, bb: "_TriggerDecision") -> bool:
        # Only while the local tier takes the question (review): after
        # `local_max_unable` unable answers the usual triggers run.
        if not (self._local_track and self._local_first() is not None
                and bb.perception.status != DETECTED):
            return False
        if bb.confirmed == ABSENT:
            # The absent edge still moves (review), or a later detection would
            # never fire `candidate_sighting`.
            self._last_confirmed = ABSENT
        # The absent streak does not grow on tracking frames (review): a cold
        # search must not fire, paid, the moment tracking ends.
        return self._fire(bb, TRIGGER_TRACK)

    def _tr_candidate(self, bb: "_TriggerDecision") -> bool:
        # The vote never fires on a frame that does not itself show the target
        # (review): the question and any steering need this frame's box.
        if self.frame_vote:
            sighted = bb.perception.status == DETECTED and self._vote_ok
        else:
            sighted = bb.confirmed == DETECTED
        if not (sighted and self._last_confirmed != DETECTED):
            return False
        self._last_confirmed = DETECTED
        if self.frame_vote:
            self.stats.vote["fired"] += 1
        return self._fire(bb, TRIGGER_CANDIDATE)

    def _tr_rejected(self, bb: "_TriggerDecision") -> bool:
        # 3.91.C: the candidate a no is remembered about asks nothing and
        # counts toward nothing; its run starts again once it is released.
        if not self._masked:
            return False
        self._run.clear()
        self._run_since = None
        # Review: nor toward the vote, or the frame it is released on fires
        # the candidate edge alone on the masked frames' votes.
        self._votes.clear()
        self._vote_ok = False
        if self.stats.vote:
            self.stats.vote["detected"] = 0
        self._band_run = 0
        self._band_asked = False
        return self._fire(bb, None)

    def _tr_disagree(self, bb: "_TriggerDecision") -> bool:
        # 3.91.C (3.92: escalate on disagreement, not on Qwen's own doubt):
        # the local VLM said no where it cannot tell, or yes against the
        # cloud's no, and perception still passes the candidate -- the cloud,
        # once a run (the tier ordering; option (a), the user, 2026-10-24).
        if (not self.remember_no or self.local_sighting_fn is None
                or bb.confirmed != DETECTED
                or self._escalate_episode != self._episode
                or self._disagree_key == (self._episode, self._release_gen)):
            return False
        self._disagree_key = (self._episode, self._release_gen)
        return self._fire(bb, TRIGGER_DISAGREE)

    def _tr_absent_edge(self, bb: "_TriggerDecision") -> bool:
        if bb.confirmed == ABSENT:
            self._last_confirmed = ABSENT
        return True

    def _tr_count_absent(self, bb: "_TriggerDecision") -> bool:
        self._absent_streak += 1
        return True

    def _tr_band(self, bb: "_TriggerDecision") -> bool:
        if not (bb.band and self._band_run >= self.consecutive_frames and not self._band_asked):
            return False
        self._band_asked = True
        return self._fire(bb, TRIGGER_BAND)

    def _tr_oov(self, bb: "_TriggerDecision") -> bool:
        # Only while the local tier takes it: a question due for the cloud
        # waits for cold_search's bar (review, 2026-10-14).
        if (self._local_first() is not None and self.vocabulary is not None
                and not self.vocabulary.in_vocabulary
                and self._absent_streak >= self.consecutive_frames):
            return self._fire(bb, TRIGGER_OOV)
        return False

    def _tr_cold_frames(self, bb: "_TriggerDecision") -> bool:
        # An OR with the distance bar, not a replacement: distance measures
        # new view while driving; the frame floor still fires when scanning
        # in place, which covers new view at zero path.
        if not self._absent_streak >= self._cold_search_bar():
            return False
        return self._fire(bb, TRIGGER_COLD_SEARCH)

    def _tr_cold_metres(self, bb: "_TriggerDecision") -> bool:
        if not self._distance_bar_reached():
            return False
        return self._fire(bb, TRIGGER_COLD_SEARCH)

    def _tr_stale(self, bb: "_TriggerDecision") -> bool:
        # The floor. Last, so it never pre-empts an event that says something
        # more specific about why we are calling.
        if not (self.stale_after and self._since_call >= self.stale_after):
            return False
        return self._fire(bb, TRIGGER_STALE)

    def _tr_none(self, bb: "_TriggerDecision") -> bool:
        return self._fire(bb, None)

    def _read_odometry(self, frame: dict) -> None:
        """Pull this frame's odometry reading, if the backend has any.

        `MissionRunner._guarded_vision()` attaches it. A frame with no
        `odometry` key at all is a caller that predates Phase C -- a
        replay harness, a test, a policy driven directly -- and is treated
        exactly like a backend with no encoders, which is the honest
        reading of "nobody told me how far this thing went."
        """
        odo = frame.get("odometry") or {}
        # P25: integrate unconditionally. An unusable reading is counted by
        # the tracker rather than treated as standing still, which is the
        # case every real-pixels walk takes -- a phone has no encoders.
        self._odom.update(odo)
        usable = bool(odo.get("usable")) and odo.get("distance_m") is not None
        self._odometry_usable = usable
        if not usable:
            self._distance_since_call = None
            return
        travelled_cm = float(odo["distance_m"]) * 100.0
        if self._odometry_at_call is None:
            self._odometry_at_call = travelled_cm
        self._distance_since_call = max(0.0, travelled_cm - self._odometry_at_call)

    def _mark_odometry_call(self) -> None:
        """Reset the distance baseline to here, because the cloud just
        looked. Called on EVERY trigger rather than only on cold_search:
        a candidate_sighting call has looked at this view too, and
        counting the ground it covered again would fire a redundant
        cold_search moments later."""
        if self._distance_since_call is not None:
            self._odometry_at_call = (
                (self._odometry_at_call or 0.0) + self._distance_since_call)
            self._distance_since_call = 0.0

    def _distance_bar_reached(self) -> bool:
        if self.cold_search_after_cm is None:
            return False
        if self._distance_since_call is None:
            return False
        return self._distance_since_call >= self.cold_search_after_cm

    def _pacing_readout(self) -> dict:
        """What is pacing the cloud right now, for 6.3's panel.

        Names the rule in force rather than leaving it to be inferred. A
        silent fallback from distance to frames is how a walk becomes
        unattributable -- the same failure mode `crop_source` is reported
        for, one tier up.
        """
        distance_armed = (self.cold_search_after_cm is not None
                          and self._odometry_usable
                          and self._distance_since_call is not None)
        return {
            "rule": "distance" if distance_armed else "frames",
            "frames_absent": self._absent_streak,
            "frames_bar": self._cold_search_bar(),
            # 3.80.P: the bar a question handed to the cloud waits, which
            # differs from frames_bar while the local tier takes them.
            "cloud_frames_bar": self._cloud_cold_search_bar(),
            "cm_since_call": (None if self._distance_since_call is None
                              else round(self._distance_since_call, 1)),
            "cm_bar": self.cold_search_after_cm,
            "odometry_usable": self._odometry_usable,
            # Why the distance rule is not in force, when it is not. Three
            # different reasons, and they want different fixes: not
            # configured, no encoders on this backend, or nothing measured
            # yet this mission.
            "reason": (
                None if distance_armed
                else "cold_search_after_cm is not set" if self.cold_search_after_cm is None
                else "this backend reports no odometry"),
        }

    def _cold_search_bar(self) -> int:
        """How many `absent` frames before a cold search is asked: the local
        pace while the local tier takes the question (3.80.P), else the
        cloud's (`_cloud_cold_search_bar`)."""
        if self.local_cold_search_after is not None and self._local_first() is not None:
            return self.local_cold_search_after
        return self._cloud_cold_search_bar()

    def _cloud_cold_search_bar(self) -> int:
        """How many `absent` frames before the cloud is asked to propose.

        One number unless the experiment above is switched on, in which
        case an out-of-vocabulary target gets the shorter wait -- because
        its `absent` carries less information, not because it is more
        likely to be present.
        """
        if (self.oov_cold_search_after is not None
                and self.vocabulary is not None
                and not self.vocabulary.in_vocabulary):
            return self.oov_cold_search_after
        return self.cold_search_after

    # -- what comes back -------------------------------------------------

    def _local_scene(self, perception: Perception, note: str = "",
                     dispatched: Optional[str] = None,
                     local_asked: Optional[str] = None) -> dict:
        """The stand-in. Same schema `brain/navigate.py` produces, so every
        consumer is unchanged -- but `_navigate.reasoning` says plainly
        that no model was asked, because a log line that reads like a
        model's reasoning when nothing was called is exactly the kind of
        thing this project has been burned by.

        `dispatched` is the trigger of a call STARTED on this frame (Phase
        A's async path). Such a frame spent money, so it must not report
        `cloud_called: False` -- see the note on that key below.
        """
        why = note or f"no trigger ({perception.status})"
        # 2.5's held goal, and ONLY while a call is outstanding. Without
        # this the stand-in scans through every frame it is waiting on,
        # which turns a 3.6s round trip into a visible spin -- the
        # stutter Phase A exists to remove. On an ordinary free frame the
        # scan stays, because holding a goal with nothing confirming it is
        # a different design and an unmeasured one.
        landed, self._landed_corroboration = self._landed_corroboration, None
        local_landed, self._local_landed = self._local_landed, None
        # Set by the two rungs that have a real bearing in hand (R1), read
        # once the final direction is known.
        self._steer_turn_deg = self._reckoned_turn_deg = None
        # 3.94: what to do this frame is a tree (`_build_what_tree`): the
        # rungs in order of evidence, the spin guard over them, then the
        # turn's size. `tests/tiered_reference.py` holds the code it replaced.
        bb = _FrameDecision(perception=perception, why=why)
        self._what_tree.tick(bb)
        direction, turn_deg, why, held = bb.direction, bb.turn_deg, bb.why, bb.held
        return {
            "obstacles_ahead": [],
            "turn_deg": turn_deg,
            # Nothing local measures depth. M1's argument exactly: the
            # collar's get_distance() re-check is the only obstacle logic
            # on this path, and "unknown" is what stops a consumer acting
            # on a number nobody produced.
            "free_space": "unknown",
            "doorway_visible": False,
            "important_objects": [],
            "safest_direction": direction,
            "_navigate": {
                "target_visible": perception.status == DETECTED,
                # "not_visible" would contradict target_visible above on
                # exactly the frames perception did its job on -- the log
                # read "target not_visible" under a +0.068 match. The
                # honest answer when the target IS seen is the bearing
                # perception already measured, or "unknown" when the frame
                # gave nothing to measure it from. This is perception's own
                # output (1.11: bearing from the box), not a decision --
                # the stand-in still must not grow into a policy.
                "target_direction": _direction_for(perception),
                "target_reached": False,
                "obstacle_ahead": None,
                "room_guess": "unclear",
                "distance_estimate": "unknown",
                # A dispatch frame DID call out; only the answer is
                # missing. It used to read "[reactive tier, no cloud call]
                # [cloud: cold_search] dispatched" -- a line that denies
                # and admits the same call in eleven words.
                "reasoning": (f"[reactive tier, cloud call dispatched] {why}"
                              if dispatched
                              else f"[reactive tier, local question: {local_asked}] {why}"
                              if local_asked
                              else f"[reactive tier, no cloud call] {why}"),
            },
            "_perception": perception.as_dict(),
            # Phase A split "the cloud was asked" from "the cloud
            # answered", and this readout only ever expressed the second.
            # Under `async_cloud` the dispatch branch returns here, so
            # `cloud_called` was False on EVERY frame of every async
            # mission. The twin's calls-and-frames counter was unaffected
            # (it reads `stats.cloud_calls`, incremented before dispatch);
            # what broke is the per-frame fact -- `[cloud: <trigger>]`
            # never appeared in a mission log, and `_tier` is what a walk
            # stores, so no recording could say which frame the cloud saw.
            # `cloud_called` now means money was spent on THIS step, which
            # is what it meant before Phase A and what every consumer
            # already assumes. `cloud_landed` is the new, separate fact.
            "_tier": {"cloud_called": dispatched is not None,
                      "trigger": dispatched,
                      "cloud_landed": landed is not None,
                      "models": dict(self.models),
                      # No cloud call means no claim to corroborate. Stated
                      # rather than omitted: a missing key would leave the
                      # panel showing the previous call's verdict for however
                      # many free frames follow it, which is exactly how a
                      # stale readout becomes a believed one.
                      # Normally None -- a free frame made no claim. The
                      # exception is a verdict that LANDED this frame from
                      # an async call, shown once and then cleared.
                      "corroboration": landed,
                      # What the robot is doing while it waits, named so a
                      # held goal can never be mistaken for a fresh answer.
                      "in_flight": self._inflight_trigger,
                      # 3.80.D: which tier a question still out was put to,
                      # the question asked on this frame, and an answer from
                      # the local tier that landed on it.
                      "in_flight_tier": self._inflight_tier,
                      "local_called": local_asked,
                      "local_landed": local_landed,
                      "holding": held,
                      # 3.91.C: a no remembered about a candidate, if any.
                      "rejected": self._rejection_readout(),
                      "probation": self._probation,
                      # The tiers' order: this frame's candidate is one a
                      # VLM refused (held or on probation) -- explore and any
                      # other reader of `_perception` must not act on it.
                      "candidate_refused": self._refused_in_view(perception),
                      "vocabulary": self._vocabulary_readout(),
                      "pacing": self._pacing_readout(),
                      "stats": self.stats.as_dict()},
        }

    # -- 3.94: what to do this frame, as a tree ---------------------------

    def _build_what_tree(self) -> Node:
        """The precedence ladder of `_local_scene`, drawn as a tree:

        * this frame's located yes is taken (used at most once, even when a
          better rung wins), and the held goal read;
        * the rungs, best evidence first -- a bearing measured on THIS frame,
          the local VLM's located yes, a dead-reckoned sighting, YOLOE +
          CLIP's uncertain candidate, the held goal, and the scan;
        * the spin guard over them;
        * the turn's size, and the turn counted for the guard.
        """
        nodes = {
            RUNG_STEER: Action("steer on this frame's bearing", self._rung_steer,
                               lambda: {"steer_on_sight": self.steer_on_sight,
                                        "band_deg": self.steer_band_deg}),
            RUNG_LOCATED: Action("turn to the local VLM's located yes", self._rung_located),
            RUNG_RECKONED: Action("dead-reckon to a sighting", self._rung_reckoned,
                                  lambda: {"hold_bearing": self.hold_bearing,
                                           "max_m": self.hold_bearing_max_m}),
            RUNG_CANDIDATE: Action("steer on an uncertain candidate", self._rung_candidate,
                                   lambda: {"steer_on_candidate": self.steer_on_candidate,
                                            "band_low": self.local_band_low,
                                            "gate": self.match_probability}),
            RUNG_HELD: Action("hold the last goal", self._rung_held,
                              lambda: {"hold_goal": self.hold_goal}),
            RUNG_SCAN: Action("scan", self._rung_scan, {"action": SCAN_ACTION}),
        }
        if not self.what_rungs or self.what_rungs[-1] != RUNG_SCAN:
            raise ValueError(f"what_rungs must end with {RUNG_SCAN!r}; got {self.what_rungs}")
        # 3.94.D: the order is `tier_tree.what_rungs`; a rung left out never runs.
        rungs = Fallback("rungs, best evidence first", [nodes[n] for n in self.what_rungs])
        # Settings given as a function are read when printed, so `describe`
        # shows the values the rungs act on now, not those at construction.
        return Sequence("what to do this frame", [
            Action("take this frame's evidence", self._take_evidence),
            Decorator("spin guard", rungs, self._spin_guard,
                      lambda: {"after_quarter_turns": self.spin_guard_after}),
            Action("size the turn", self._size_turn,
                   lambda: {"min_deg": self.min_turn_deg, "max_deg": self.max_turn_deg,
                            "scan_deg": self.scan_turn_deg}),
            Action("count the turn", self._count_turn),
        ])

    def describe_tree(self) -> str:
        return "\n".join(self._what_tree.describe())

    def describe_hard_rules(self) -> str:
        """The rules the three trees work inside and no tree may reorder
        (3.94; the tiers' order, 3.91.C)."""
        return "\n".join(HARD_RULES)

    def _take_evidence(self, bb: "_FrameDecision") -> bool:
        # A located yes is used once, after this frame's own bearing and
        # before anything older -- and spent even when a better rung wins.
        bb.located_turn, self._local_turn = self._local_turn, None
        bb.held = (self._held_direction()
                   if (self.hold_goal or self._inflight is not None) else None)
        return True

    def _choose(self, bb: "_FrameDecision", rung: str, direction: str, why: str,
                turn_deg=None) -> bool:
        bb.rung, bb.direction, bb.rung_turn_deg = rung, direction, turn_deg
        if why:
            bb.why = f"{bb.why} -- {why}"
        return True

    def _rung_steer(self, bb: "_FrameDecision") -> bool:
        if self._masked:
            r = self._rejection or {}
            bb.why = (f"{bb.why} -- not steering at a candidate the {r.get('by', '?')} "
                      "said is not the target")
            return False
        if self._on_probation(bb.perception):
            bb.why = (f"{bb.why} -- not steering at a candidate once refused, "
                      "until a yes (probation)")
            return False
        steer = self._steer_to(bb.perception)
        return bool(steer) and self._choose(
            bb, RUNG_STEER, steer, f"steering on local sighting ({_direction_for(bb.perception)})",
            self._steer_turn_deg)

    def _rung_located(self, bb: "_FrameDecision") -> bool:
        located = bb.located_turn[0] if bb.located_turn else None
        return bool(located) and self._choose(
            bb, RUNG_LOCATED, located, "turning to where the local VLM saw the target",
            bb.located_turn[1])

    def _rung_reckoned(self, bb: "_FrameDecision") -> bool:
        reckoned = self._dead_reckoned_direction()
        return bool(reckoned) and self._choose(
            bb, RUNG_RECKONED, reckoned,
            f"dead-reckoning to a sighting {self._goal.sightings} detection(s) ago",
            self._reckoned_turn_deg)

    def _rung_candidate(self, bb: "_FrameDecision") -> bool:
        candidate = self._steer_to_candidate(bb.perception)
        if not candidate:
            return False
        with self._stats_lock:
            self.stats.candidate_steers += 1
        best = max(float(c.probability) for c in bb.perception.candidates)
        return self._choose(bb, RUNG_CANDIDATE, candidate,
                            f"steering on an uncertain candidate (P {best:.2f})",
                            self._steer_turn_deg)

    def _rung_held(self, bb: "_FrameDecision") -> bool:
        if not bb.held:
            return False
        why = (f"holding the local VLM's {self._local_held_why or 'answer'} {bb.held}"
               if self._local_held else f"holding last cloud goal {bb.held}")
        return self._choose(bb, RUNG_HELD, bb.held, why)

    def _rung_scan(self, bb: "_FrameDecision") -> bool:
        return self._choose(bb, RUNG_SCAN, SCAN_ACTION, "")

    def _spin_guard(self, bb: "_FrameDecision", rungs_status) -> str:
        """The spin floor, decided after the rungs have run (py_trees ticks
        the decorator's child first). Counted over what the robot was
        actually TOLD to do, cloud steps included, because a spin does not
        care which tier caused it. Never over a bearing measured or located
        on this frame."""
        if bb.direction is None:
            # The scan rung always chooses; a ladder that ends without one
            # would skip the turn's size and the guard's count below.
            raise RuntimeError("the rungs chose no direction")
        if (self.spin_guard_after
                and bb.direction in TURN_ACTIONS
                and bb.rung not in RUNGS_THIS_FRAME
                and (bb.perception.status != DETECTED or self._masked
                     or self._on_probation(bb.perception))
                and self._consecutive_turn_deg >= self.spin_guard_after * 90):
            bb.direction, bb.rung = "FORWARD", RUNG_SPIN_GUARD
            self._consecutive_turn_deg = 0.0
            bb.why = (f"{bb.why} -- spin guard: {self.spin_guard_after * 90} degrees "
                      "of turning without a detection, forcing FORWARD")
            bb.rung_turn_deg = None
        return SUCCESS

    def _size_turn(self, bb: "_FrameDecision") -> bool:
        """R1: how FAR to turn, when a measured or dead-reckoned bearing
        chose the turn. A scan, a held goal or a spin-guard override has no
        bearing behind it; R1b: such a turn is a SEARCH step, smaller than
        the field of view (SCAN_TURN_DEG)."""
        bb.turn_deg = None
        if bb.direction in TURN_ACTIONS:
            # Only a bearing rung hands one over (`_choose`); a held goal, a
            # scan or a spin-guard FORWARD leave it None.
            bb.turn_deg = self.scan_turn_deg if bb.rung_turn_deg is None else bb.rung_turn_deg
        return True

    def _turn_for(self, bearing_deg: float) -> int:
        """`turn_for` with this policy's bounds (`tier_tree`)."""
        return int(round(min(self.max_turn_deg, max(self.min_turn_deg, abs(bearing_deg)))))

    def _count_turn(self, bb: "_FrameDecision") -> bool:
        # Counted AFTER the size is known -- the guard is in degrees (R1c).
        self._note_action(bb.direction, bb.turn_deg)
        return True

    def _steer_to(self, perception) -> Optional[str]:
        """An action from the local tier's own bearing, or None.

        `None` on every path that is not a measured sighting -- not
        detected, no bearing measurable, or the feature switched off --
        because the caller falls back to the cloud's goal and a wrong
        default here would silently become the policy.
        """
        if not self.steer_on_sight or perception.status != DETECTED:
            return None
        # A measured sighting anchors (or re-anchors) the goal pose, which
        # is what lets the NEXT frame answer without a detection. Only a
        # frame that actually measured a bearing may do this -- anchoring on
        # `unknown` would store a direction nobody observed.
        if perception.bearing_deg is not None and self._odom.samples:
            self._goal.sight(self._odom.pose, float(perception.bearing_deg))
        where = _direction_for(perception)
        bearing = perception.bearing_deg
        # Steer on the finer band (STEER_BAND_DEG); `where` stays the
        # reporting vocabulary for everything else.
        if bearing is not None and abs(bearing) > self.steer_band_deg:
            self._steer_turn_deg = self._turn_for(bearing)
            return "LEFT" if bearing < 0 else "RIGHT"
        if where in ("left", "right", "center"):
            # Centred: close the distance. The collar re-checks clearance
            # before any FORWARD, so this proposes and never commits.
            return "FORWARD"
        # "unknown" -- detected, but no bearing. Steering on a direction
        # nobody measured is exactly the fabrication `unusable_grid()` and
        # NO_SENSOR_CM refuse elsewhere.
        return None

    def _steer_to_candidate(self, perception) -> Optional[str]:
        """3.80.F: an action toward the best crop when it is in the
        uncertain band -- P in [local_band_low, the gate) -- or None. Not
        after a local no on this band run (`_candidate_rejected`), not for a
        panned camera, and never an anchor for dead reckoning: the crop may
        not be the target."""
        if (not self.steer_on_candidate or self._candidate_rejected
                # 3.91.C (review): a crop under the gate while a no or a
                # probation is held is the same object, most likely.
                or self._rejection is not None or self._probation
                or perception.status != ABSENT or not perception.candidates
                or getattr(perception, "pan_deg", 0.0)):
            return None
        # An ABSENT perception carries its crops but no `best` (perceive.py):
        # the strongest one, as the band trigger reads it.
        try:
            best = max(perception.candidates, key=lambda c: float(c.probability))
            p = float(best.probability)
        except (AttributeError, TypeError, ValueError):
            return None
        if best.bearing_deg is None:
            return None
        if not (self.local_band_low <= p < self.match_probability):
            return None
        bearing = float(best.bearing_deg)
        if abs(bearing) > self.steer_band_deg:
            self._steer_turn_deg = self._turn_for(bearing)
            return "LEFT" if bearing < 0 else "RIGHT"
        return "FORWARD"

    def _dead_reckoned_direction(self) -> Optional[str]:
        """An action from an ANCHORED sighting the detector is not seeing.

        P25's repair (P7c item 2). The target is static, so a sighting stays
        true while the robot moves; what changes is the robot, and odometry
        measures that far better than a detector recognises objects. So a
        frame with no detection need not re-decide blind.

        `None` on every path that is not a live anchor plus real odometry:
        the feature off, no odometry (a phone has no encoders), nothing
        anchored, or the robot has travelled past the bound. The caller then
        falls back to the cloud goal exactly as before, so this ADDS a rung
        to the precedence ladder and removes none.
        """
        if not self.hold_bearing or not self._goal.held:
            return None
        if not self._odometry_usable or not self._odom.samples:
            return None
        seen_at = self._goal._seen_at
        if seen_at is not None and self.hold_bearing_max_m:
            moved = math.hypot(self._odom.pose.x - seen_at.x,
                               self._odom.pose.y - seen_at.y)
            # A direction-only anchor is wrong under translation, so the
            # bound is a distance and not a timeout. Dropping it is the
            # honest outcome -- a stale anchor steered on confidently is
            # worse than scanning.
            if moved > self.hold_bearing_max_m:
                self._goal.clear()
                return None
        bearing = self._goal.bearing_from(self._odom.pose)
        if bearing is None:
            return None
        self._dead_reckoned += 1
        if abs(bearing) > self.steer_band_deg:
            self._reckoned_turn_deg = self._turn_for(bearing)
        if bearing < -self.steer_band_deg:
            return "LEFT"
        if bearing > self.steer_band_deg:
            return "RIGHT"
        return "FORWARD"

    def _note_action(self, direction, turn_deg=None) -> None:
        """Track consecutive TURNING, in degrees, over cloud and local steps
        alike. A turn with no size is counted at the executor's default
        quarter turn, which is what it will actually do."""
        if direction in TURN_ACTIONS:
            self._consecutive_turn_deg += float(turn_deg or 90)
        else:
            self._consecutive_turn_deg = 0.0

    def _annotate(self, scene: dict, perception: Perception, trigger: str,
                  executed: bool = True) -> dict:
        """A real cloud scene, with the local evidence attached beside it.

        The cloud's answer is **not** overridden by the local one, in
        either direction. 1.11's arbitration is split by question: identity
        belongs to the VLM, and a high CLIP margin raises a candidate but
        never confirms one. Both are recorded so a walk can be scored on
        how often they agreed.
        """
        corroboration = corroboration_for(scene, perception,
                                          self.corroboration_bar)
        verdict = corroboration["verdict"]
        self.stats.verdicts[verdict] = self.stats.verdicts.get(verdict, 0) + 1
        if corroboration["claimed"]:
            self.stats.claims += 1
            if verdict == CORROBORATED:
                self.stats.corroborated += 1

        out = dict(scene)
        # R1: SIZE the cloud's turn from the local bearing, when the two
        # agree on which way. The cloud still owns the direction -- this
        # never overrides it, in keeping with 1.11's split -- but /navigate
        # answers LEFT/RIGHT with no magnitude, so on every trigger frame a
        # correct "it's to the left" went out as a blind quarter turn and
        # swung the robot straight past the target the tier had just
        # measured. When they disagree, or nothing local was measured, the
        # turn keeps the executor's default: no size nobody measured.
        direction = scene.get("safest_direction")
        bearing = perception.bearing_deg if perception.status == DETECTED else None
        if (direction in TURN_ACTIONS and bearing is not None
                and abs(bearing) > self.steer_band_deg
                and (direction == "LEFT") == (bearing < 0)):
            out["turn_deg"] = self._turn_for(bearing)
        elif direction in TURN_ACTIONS and not out.get("turn_deg"):
            # The cloud said turn and nothing local can size it: a search
            # step, for the reason SCAN_TURN_DEG gives.
            out["turn_deg"] = self.scan_turn_deg
        if executed:
            # 3.76: not for a landed async answer annotated for its side
            # effects -- that turn is never executed, and counting it fired
            # the spin guard early (or a FORWARD reset it mid-spin).
            self._note_action(direction, out.get("turn_deg"))
        out["_perception"] = perception.as_dict()
        out["_tier"] = {"cloud_called": True, "trigger": trigger,
                        # Synchronous: asked and answered on one frame.
                        "cloud_landed": True,
                        "models": dict(self.models),
                        # 1.11a, reported and NOT enforced. `scene` above is
                        # passed through untouched -- deliberately, and it is
                        # the whole difference between measuring the
                        # amendment and shipping it.
                        "corroboration": corroboration,
                        "vocabulary": self._vocabulary_readout(),
                        "pacing": self._pacing_readout(),
                        "stats": self.stats.as_dict()}
        return out

    def _vocabulary_readout(self) -> Optional[dict]:
        """The verdict, for 6.3's panel. Carries `enforced` explicitly for
        the same reason every corroboration line carries "not enforced":
        a measurement that a reader can mistake for a decision will corrupt
        the walks meant to decide it."""
        if self.vocabulary is None:
            return None
        out = self.vocabulary.as_dict()
        # The cloud's bar: this readout is about the OOV experiment, which
        # paces the cloud (3.80.P: the local pace is `pacing.frames_bar`).
        out["cold_search_after"] = self._cloud_cold_search_bar()
        out["enforced"] = self.oov_cold_search_after is not None
        return out

    # -- passthrough -----------------------------------------------------

    def set_searched_rooms(self, rooms) -> None:
        """`MissionRunner._guarded_vision()` calls this when present. Pass
        it through so room-level step memory (S2b) keeps working -- it is
        the cloud call's business, not perception's."""
        for fn in {id(f): f for f in (self.cloud_vision_fn, self.confirm_vision_fn)
                   if f is not None}.values():
            # 3.56: the confirmation's own client too -- its /navigate call
            # carried the searched rooms before it had a client of its own.
            setter = getattr(fn, "set_searched_rooms", None)
            if setter is not None:
                setter(rooms)


def tiered_vision_fn_for(target: str, cloud_vision_fn: Callable[[dict], dict],
                         *, pipeline=None, **kwargs) -> TieredVision:
    """Bind a target and a cloud `vision_fn` into a tiered one.

    `pipeline` is injectable so a test can supply fakes and a real run can
    supply `brain.perceive.pipeline_for(target)` -- which is the only line
    that needs the optional heavy dependencies.
    """
    if pipeline is None:
        from brain.perceive import pipeline_for
        pipeline = pipeline_for(target)
    return TieredVision(pipeline, cloud_vision_fn, **kwargs)


__all__ = [
    "TieredVision", "TierStats", "tiered_vision_fn_for", "CENTER_BAND_DEG", "HARD_RULES",
    "candidate_refused",
    "TIER_RANK",
    "MAX_TURN_DEG", "MIN_TURN_DEG", "SCAN_TURN_DEG", "STEER_BAND_DEG", "turn_for",
    "TRIGGER_START", "TRIGGER_CANDIDATE", "TRIGGER_COLD_SEARCH",
    "TRIGGER_STALE", "DEFAULT_STALE_AFTER", "TRIGGER_BAND", "TRIGGER_OOV", "TRIGGER_DISAGREE",
    "UNAVAILABLE_TRIGGERS", "SCAN_ACTION",
    "DEFAULT_CONSECUTIVE", "DEFAULT_COLD_SEARCH_AFTER",
    "DEFAULT_CORROBORATION_P", "corroboration_for",
    "CORROBORATED", "UNCLEAR", "NO_CLAIM", "CORROBORATION_UNAVAILABLE",
    "CORROBORATION_VERDICTS",
]
