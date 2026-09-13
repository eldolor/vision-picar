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

## The three triggers this can actually fire

2.4 lists seven. Four of them belong to a reactive tier that holds and
executes a goal (C6) and to a planner that issues one (C8), neither of
which exists, so firing them here would be theatre. The three that are
real without either:

| Trigger | When | Why it is honest here |
|---|---|---|
| `mission_start` | first frame | 2.5: the one genuinely blocking call |
| `candidate_sighting` | perception says `detected` | 2.4's *"the one that does real work"* -- on-board proposes, the cloud confirms identity and **reachability** |
| `cold_search` | `absent` for `cold_search_after` consecutive frames | 2.4's inverse: the cloud proposes, on-board tracks. Without it a target outside COCO's 80 never fires anything and the robot can drive past it indefinitely |

`goal_achieved`, `goal_impossible`, `room_change` and `staleness` are
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

import logging
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Callable, Optional

from brain.perceive import ABSENT, DETECTED, UNAVAILABLE, Perception

logger = logging.getLogger("tiered")

TRIGGER_START = "mission_start"
TRIGGER_CANDIDATE = "candidate_sighting"
TRIGGER_COLD_SEARCH = "cold_search"

# 2.4's other four. Named, not implemented -- each needs a tier that does
# not exist yet, and a trigger that cannot fire is worse than one that is
# absent, because it looks like coverage.
TRIGGER_STALE = "staleness"

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

    def as_dict(self) -> dict:
        saving = (self.frames / self.cloud_calls) if self.cloud_calls else None
        return {
            "frames": self.frames,
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
        cold_search_after: int = DEFAULT_COLD_SEARCH_AFTER,
        cold_search_after_cm: Optional[float] = DEFAULT_COLD_SEARCH_AFTER_CM,
        stale_after: int = DEFAULT_STALE_AFTER,
        max_calls: Optional[int] = None,
        corroboration_bar: float = DEFAULT_CORROBORATION_P,
        oov_cold_search_after: Optional[int] = None,
        async_cloud: bool = False,
    ):
        self.pipeline = pipeline
        self.cloud_vision_fn = cloud_vision_fn
        self.consecutive_frames = max(1, int(consecutive_frames))
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
        self._executor = None
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
        self._last_cloud_scene: Optional[dict] = None
        # A verdict from an async call, held for exactly ONE frame -- the
        # one it landed on. Without this the corroboration row reads "no
        # claim this step" on every frame of an async mission while the
        # tally climbs behind it, so 1.11a's per-frame measurement is lost
        # from walk.jsonl entirely. Cleared after it is shown, because the
        # rule `_local_scene` already enforces is that a stale verdict must
        # never read as this frame's.
        self._landed_corroboration: Optional[dict] = None
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
        self._absent_streak = 0
        self._last_confirmed = ABSENT
        self._since_call = 0

    # -- the vision_fn contract ------------------------------------------

    def __call__(self, frame: dict) -> dict:
        self._read_odometry(frame)
        # An answer that landed since the last frame becomes the current
        # goal now, BEFORE the trigger policy runs -- so a call that has
        # already returned is never counted as still in flight.
        self._collect_inflight()
        perception = self.pipeline.perceive(frame)
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
            raise err

        trigger = self._trigger_for(perception)
        if trigger is None:
            self._since_call += 1
            return self._local_scene(perception)

        if self.max_calls is not None and self.stats.cloud_calls >= self.max_calls:
            logger.warning("call cap %d reached -- not firing %s",
                           self.max_calls, trigger)
            return self._local_scene(perception, note="call cap reached")

        if self.async_cloud and self._inflight is not None:
            # One call at a time. A second dispatch while the first is
            # outstanding is how replays used to lose frames (CLAUDE.md's
            # note on stacking replays on one vision task), and it would
            # also spend twice for one answer.
            return self._local_scene(
                perception,
                note=f"{self._inflight_trigger} call still in flight")

        self.stats.cloud_calls += 1
        self.stats.triggers[trigger] = self.stats.triggers.get(trigger, 0) + 1
        self._absent_streak = 0
        self._since_call = 0
        self._mark_odometry_call()

        if self.async_cloud:
            self._dispatch(frame, trigger)
            return self._local_scene(perception,
                                     note=f"[cloud: {trigger}] dispatched")

        scene = self.cloud_vision_fn(frame)
        return self._annotate(scene, perception, trigger)

    # -- Phase A: dispatch, collect, and hold the goal --------------------

    def _dispatch(self, frame: dict, trigger: str) -> None:
        """Start the cloud call on a worker thread and return immediately."""
        if self._executor is None:
            # One worker, created lazily so a synchronous mission (the
            # default) never starts a thread at all.
            self._executor = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="tiered-cloud")
        epoch = self._epoch
        self._inflight_trigger = trigger
        # Corroboration (1.11a) compares the cloud's claim against the
        # LOCAL evidence from the same frame. Async breaks that pairing
        # unless the perception is carried along with the call, so it is.
        self._inflight_perception = self._perception_this_frame
        self._inflight = self._executor.submit(
            self._call_cloud, frame, epoch)

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
        return self.cloud_vision_fn(frame)

    def _collect_inflight(self) -> None:
        """Apply a landed answer, or note that it failed. Never waits."""
        fut = self._inflight
        if fut is None or not fut.done():
            return
        self._inflight = None
        trigger, self._inflight_trigger = self._inflight_trigger, None
        try:
            scene = fut.result()
        except BaseException as exc:  # noqa: BLE001
            # Held, not raised here: this runs before perception, and a
            # failure surfaced mid-frame would skip the local tier's work
            # for that frame. Raised at the top of the NEXT call, where
            # B3.2's budget sees it as it always did.
            logger.warning("async cloud call failed: %s", exc)
            self._pending_error = exc
            return
        if scene is None:
            # Superseded: the mission ended or reset while this was out.
            # Dropping it silently is the point -- applying it is the
            # orphaned-call bug.
            return
        self._last_cloud_scene = scene
        # Run the annotation for its SIDE EFFECTS -- corroboration and the
        # stats 6.3 puts on the panel -- against the perception this call
        # was actually made on, not whatever is in front of the camera now.
        # The annotated scene itself is not returned: the decision for this
        # frame belongs to this frame's pixels, and what carries forward is
        # the goal, not the whole answer.
        if self._inflight_perception is not None:
            annotated = self._annotate(scene, self._inflight_perception,
                                       trigger or "")
            verdict = (annotated.get("_tier") or {}).get("corroboration")
            if verdict:
                # Marked `landed_late`, and carrying the trigger it belongs
                # to, so a reader can never take it for a verdict about the
                # frame in front of the camera now.
                self._landed_corroboration = {**verdict,
                                              "landed_late": True,
                                              "for_trigger": trigger}
        self._inflight_perception = None

    def reset_epoch(self) -> None:
        """Invalidate anything in flight. Called when a mission ends, so a
        late answer cannot be applied to a robot that has stopped."""
        self._epoch += 1
        self._inflight = None
        self._inflight_trigger = None
        self._inflight_perception = None
        self._landed_corroboration = None
        self._pending_error = None

    def close(self) -> None:
        """Release the worker thread. Safe to call more than once."""
        self.reset_epoch()
        if self._executor is not None:
            self._executor.shutdown(wait=False, cancel_futures=True)
            self._executor = None

    def _held_direction(self) -> Optional[str]:
        """The last direction the cloud gave, if there is one.

        2.5's *"the reactive tier always holds a current goal"*. Used only
        while a call is in flight: on an ordinary free frame the stand-in
        keeps its scan, because holding a goal indefinitely with nothing
        confirming it is a different design and an unmeasured one.
        """
        if not self._last_cloud_scene:
            return None
        return self._last_cloud_scene.get("safest_direction")

    # -- the trigger policy ----------------------------------------------

    def _trigger_for(self, perception: Perception) -> Optional[str]:
        if not self._started:
            self._started = True
            # **The opening call is also the first frame's confirmation.**
            # It goes out while looking at whatever is in front of the
            # robot, so if that is already the target, firing
            # `candidate_sighting` a frame later would buy a second answer
            # to a question just asked about the same view. Seeding the
            # edge detector from the start frame is what prevents that,
            # and it is the difference between 2 calls and 1 on a mission
            # that begins pointed at the backpack.
            if perception.status in (DETECTED, ABSENT):
                self._run.append(perception.status)
                self._last_confirmed = perception.status
            return TRIGGER_START

        # `unavailable` is never an event. It is the absence of
        # information, so it must not fire a trigger and must not advance
        # the cold-search streak -- a wedged camera would otherwise buy a
        # cloud call every few frames and look like a search.
        if perception.status == UNAVAILABLE:
            self._run.clear()
            return None

        self._run.append(perception.status)
        if len(self._run) > self.consecutive_frames:
            self._run.pop(0)

        confirmed = (
            perception.status
            if (len(self._run) >= self.consecutive_frames
                and len(set(self._run)) == 1)
            else None
        )

        if confirmed == DETECTED and self._last_confirmed != DETECTED:
            self._last_confirmed = DETECTED
            return TRIGGER_CANDIDATE
        if confirmed == ABSENT:
            self._last_confirmed = ABSENT

        if perception.status == ABSENT:
            self._absent_streak += 1
            # An OR, not a replacement. Distance is the better measure of
            # "how much new view is there to look at" while the robot is
            # driving; the frame floor is what still fires when it is
            # scanning in place, which covers new view at zero path.
            if self._absent_streak >= self._cold_search_bar():
                return TRIGGER_COLD_SEARCH
            if self._distance_bar_reached():
                return TRIGGER_COLD_SEARCH

        # The floor. Last, so it never pre-empts an event that says
        # something more specific about why we are calling.
        if self.stale_after and self._since_call >= self.stale_after:
            return TRIGGER_STALE
        return None

    def _read_odometry(self, frame: dict) -> None:
        """Pull this frame's odometry reading, if the backend has any.

        `MissionRunner._guarded_vision()` attaches it. A frame with no
        `odometry` key at all is a caller that predates Phase C -- a
        replay harness, a test, a policy driven directly -- and is treated
        exactly like a backend with no encoders, which is the honest
        reading of "nobody told me how far this thing went."
        """
        odo = frame.get("odometry") or {}
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

    def _local_scene(self, perception: Perception, note: str = "") -> dict:
        """The stand-in. Same schema `brain/navigate.py` produces, so every
        consumer is unchanged -- but `_navigate.reasoning` says plainly
        that no model was asked, because a log line that reads like a
        model's reasoning when nothing was called is exactly the kind of
        thing this project has been burned by."""
        why = note or f"no trigger ({perception.status})"
        # 2.5's held goal, and ONLY while a call is outstanding. Without
        # this the stand-in scans through every frame it is waiting on,
        # which turns a 3.6s round trip into a visible spin -- the
        # stutter Phase A exists to remove. On an ordinary free frame the
        # scan stays, because holding a goal with nothing confirming it is
        # a different design and an unmeasured one.
        landed, self._landed_corroboration = self._landed_corroboration, None
        held = self._held_direction() if self._inflight is not None else None
        direction = held or SCAN_ACTION
        if held:
            why = f"{why} -- holding last cloud goal {held}"
        return {
            "obstacles_ahead": [],
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
                "reasoning": f"[reactive tier, no cloud call] {why}",
            },
            "_perception": perception.as_dict(),
            "_tier": {"cloud_called": False, "trigger": None,
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
                      "holding": held,
                      "vocabulary": self._vocabulary_readout(),
                      "pacing": self._pacing_readout(),
                      "stats": self.stats.as_dict()},
        }

    def _annotate(self, scene: dict, perception: Perception, trigger: str) -> dict:
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
        out["_perception"] = perception.as_dict()
        out["_tier"] = {"cloud_called": True, "trigger": trigger,
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
        out["cold_search_after"] = self._cold_search_bar()
        out["enforced"] = self.oov_cold_search_after is not None
        return out

    # -- passthrough -----------------------------------------------------

    def set_searched_rooms(self, rooms) -> None:
        """`MissionRunner._guarded_vision()` calls this when present. Pass
        it through so room-level step memory (S2b) keeps working -- it is
        the cloud call's business, not perception's."""
        setter = getattr(self.cloud_vision_fn, "set_searched_rooms", None)
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
    "TieredVision", "TierStats", "tiered_vision_fn_for", "CENTER_BAND_DEG",
    "TRIGGER_START", "TRIGGER_CANDIDATE", "TRIGGER_COLD_SEARCH",
    "TRIGGER_STALE", "DEFAULT_STALE_AFTER",
    "UNAVAILABLE_TRIGGERS", "SCAN_ACTION",
    "DEFAULT_CONSECUTIVE", "DEFAULT_COLD_SEARCH_AFTER",
    "DEFAULT_CORROBORATION_P", "corroboration_for",
    "CORROBORATED", "UNCLEAR", "NO_CLAIM", "CORROBORATION_UNAVAILABLE",
    "CORROBORATION_VERDICTS",
]
