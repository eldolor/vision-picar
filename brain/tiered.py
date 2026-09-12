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
        stale_after: int = DEFAULT_STALE_AFTER,
        max_calls: Optional[int] = None,
        corroboration_bar: float = DEFAULT_CORROBORATION_P,
        oov_cold_search_after: Optional[int] = None,
    ):
        self.pipeline = pipeline
        self.cloud_vision_fn = cloud_vision_fn
        self.consecutive_frames = max(1, int(consecutive_frames))
        self.cold_search_after = max(1, int(cold_search_after))
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
        perception = self.pipeline.perceive(frame)
        self.stats.frames += 1
        self.stats.perception[perception.status] = (
            self.stats.perception.get(perception.status, 0) + 1)

        trigger = self._trigger_for(perception)
        if trigger is None:
            self._since_call += 1
            return self._local_scene(perception)

        if self.max_calls is not None and self.stats.cloud_calls >= self.max_calls:
            logger.warning("call cap %d reached -- not firing %s",
                           self.max_calls, trigger)
            return self._local_scene(perception, note="call cap reached")

        self.stats.cloud_calls += 1
        self.stats.triggers[trigger] = self.stats.triggers.get(trigger, 0) + 1
        self._absent_streak = 0
        self._since_call = 0

        scene = self.cloud_vision_fn(frame)
        return self._annotate(scene, perception, trigger)

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
            if self._absent_streak >= self._cold_search_bar():
                return TRIGGER_COLD_SEARCH

        # The floor. Last, so it never pre-empts an event that says
        # something more specific about why we are calling.
        if self.stale_after and self._since_call >= self.stale_after:
            return TRIGGER_STALE
        return None

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
        return {
            "obstacles_ahead": [],
            # Nothing local measures depth. M1's argument exactly: the
            # collar's get_distance() re-check is the only obstacle logic
            # on this path, and "unknown" is what stops a consumer acting
            # on a number nobody produced.
            "free_space": "unknown",
            "doorway_visible": False,
            "important_objects": [],
            "safest_direction": SCAN_ACTION,
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
                      "corroboration": None,
                      "vocabulary": self._vocabulary_readout(),
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
