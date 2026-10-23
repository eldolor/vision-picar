"""3.94: the tiered policy's decisions as they were before the behavior tree,
frozen, so the tree can be checked against them decision for decision
(as `frontier_reference.py` is for 3.77's numpy search).

DO NOT EDIT to follow `brain/tiered.py`: this is the reference. A
deliberate change of behaviour in the policy is a change to its tests, and
this file stays the record of what the code did on 2026-10-22 (`dev` at the
branch point of 3.94).

`ReferenceTieredVision` is `TieredVision` with the old bodies of the methods
the tree replaced, copied verbatim: `_local_scene` (3.94.A),
`_trigger_for` (3.94.B) and `__call__` (3.94.C).
"""

from typing import Optional  # noqa: F401

from brain.tiered import (  # noqa: F401 -- the names the frozen bodies use
    SCAN_ACTION, SCAN_TURN_DEG, TRIGGER_BAND, TRIGGER_CANDIDATE, TRIGGER_COLD_SEARCH,
    TRIGGER_OOV, TRIGGER_STALE, TRIGGER_START, TRIGGER_TRACK, TURN_ACTIONS, TieredVision,
    _direction_for, _frame_seq, logger)
import time  # noqa: F401
from brain.perceive import ABSENT, DETECTED, UNAVAILABLE, Perception  # noqa: F401


class ReferenceTieredVision(TieredVision):
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
        # below once the final direction is known.
        self._steer_turn_deg = self._reckoned_turn_deg = None
        # Phase G, and it takes precedence over the held goal: a bearing
        # measured on THIS frame is better evidence about where to go than
        # a direction the cloud gave several seconds ago.
        steer = self._steer_to(perception)
        # 3.80.D: a local VLM's located yes, used once, after perception's
        # own bearing (measured on THIS frame) and before anything older.
        located_turn, self._local_turn = self._local_turn, None
        if steer:
            located_turn = None
        located = located_turn[0] if located_turn else None
        # P25 sits BETWEEN the two: a bearing dead-reckoned from a sighting
        # this mission actually made is worse evidence than one measured on
        # this frame, and better than a direction the cloud gave several
        # seconds and several turns ago.
        reckoned = None if (steer or located) else self._dead_reckoned_direction()
        # 3.80.F: YOLOE + CLIP's own weaker evidence -- a crop in the
        # uncertain band, measured on this frame -- after any sighting.
        candidate = (None if (steer or located or reckoned)
                     else self._steer_to_candidate(perception))
        held = (self._held_direction()
                if (self.hold_goal or self._inflight is not None) else None)
        direction = steer or located or reckoned or candidate or held or SCAN_ACTION
        if steer:
            why = (f"{why} -- steering on local sighting "
                   f"({_direction_for(perception)})")
        elif located:
            why = f"{why} -- turning to where the local VLM saw the target"
        elif reckoned:
            why = (f"{why} -- dead-reckoning to a sighting "
                   f"{self._goal.sightings} detection(s) ago")
        elif candidate:
            why = (f"{why} -- steering on an uncertain candidate (P "
                   f"{max(float(c.probability) for c in perception.candidates):.2f})")
            with self._stats_lock:
                self.stats.candidate_steers += 1
        elif held and self._local_held:
            why = (f"{why} -- holding the local VLM's "
                   f"{self._local_held_why or 'answer'} {held}")
        elif held:
            why = f"{why} -- holding last cloud goal {held}"
        # The spin floor. Counted over what the robot was actually TOLD to
        # do, cloud steps included, because a spin does not care which tier
        # caused it.
        if (self.spin_guard_after
                and direction in TURN_ACTIONS
                and steer is None
                and located is None
                and candidate is None
                and perception.status != DETECTED
                and self._consecutive_turn_deg >= self.spin_guard_after * 90):
            direction = "FORWARD"
            self._consecutive_turn_deg = 0.0
            why = (f"{why} -- spin guard: {self.spin_guard_after * 90} degrees "
                   "of turning without a detection, forcing FORWARD")
        # R1: how FAR to turn, when a measured or dead-reckoned bearing chose
        # the turn. Absent otherwise -- a scan, a held cloud goal or a
        # spin-guard override has no bearing behind it, and the executor's
        # default quarter turn is the honest answer for those.
        turn_deg = None
        if direction in TURN_ACTIONS:
            if steer == direction:
                turn_deg = self._steer_turn_deg
            elif located == direction:
                turn_deg = located_turn[1]
            elif reckoned == direction:
                turn_deg = self._reckoned_turn_deg
            elif candidate == direction:
                turn_deg = self._steer_turn_deg
            # R1b: a turn with no bearing behind it is a SEARCH, and a search
            # step must be smaller than the field of view or it skips part of
            # the circle -- see SCAN_TURN_DEG.
            if turn_deg is None:
                turn_deg = SCAN_TURN_DEG
        # Counted AFTER the size is known -- the guard is in degrees (R1c).
        self._note_action(direction, turn_deg)
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
                      "vocabulary": self._vocabulary_readout(),
                      "pacing": self._pacing_readout(),
                      "stats": self.stats.as_dict()},
        }

    def _trigger_for(self, perception: Perception) -> Optional[str]:
        if perception.status != UNAVAILABLE:
            self._note_vote(perception.status)
            # Tallied on every frame, before any early return (review: the
            # tracking and start returns left the HUD's count stale).
            self._vote_ok = self._voted() if self.frame_vote else False
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
                self._note_run(perception.status)
                self._last_confirmed = perception.status
            return TRIGGER_START

        # `unavailable` is never an event. It is the absence of
        # information, so it must not fire a trigger and must not advance
        # the cold-search streak -- a wedged camera would otherwise buy a
        # cloud call every few frames and look like a search.
        if perception.status == UNAVAILABLE:
            self._run.clear()
            self._run_since = None
            self._votes.clear()
            self._vote_ok = False
            if self.stats.vote:
                self.stats.vote["detected"] = 0
            self._band_run = 0          # 3.80.D: any other frame breaks a band run
            self._band_asked = False
            self._candidate_rejected = False
            return None

        if self._retry_start:
            # 3.80.D: the opening question went unanswered (unable).
            self._retry_start = False
            return TRIGGER_START
        # 3.80.D: the uncertain band, tracked only with a local tier.
        band = (self.local_sighting_fn is not None and perception.status == ABSENT
                and bool(perception.candidates)
                and self.local_band_low
                <= max(c.probability for c in perception.candidates)
                < self.match_probability)
        if band:
            self._band_run += 1
        else:
            self._band_run = 0
            self._band_asked = False
            self._candidate_rejected = False

        self._note_run(perception.status)

        confirmed = (
            perception.status
            if (len(self._run) >= self.consecutive_frames
                and len(set(self._run)) == 1
                and self._run_spans())
            else None
        )

        if (self._local_track and self._local_first() is not None
                and perception.status != DETECTED):
            # Only while the local tier takes the question (review): after
            # `local_max_unable` unable answers the usual triggers run, and
            # the cloud may take one of them.
            if confirmed == ABSENT:
                # The absent edge still moves (review), or a later detection
                # would never fire `candidate_sighting`.
                self._last_confirmed = ABSENT
            # The absent streak does not grow on tracking frames (review): a
            # cold search must not fire, paid, the moment tracking ends.
            return TRIGGER_TRACK
        # The vote never fires on a frame that does not itself show the target
        # (review): the question and any steering need this frame's box.
        if self.frame_vote:
            sighted = perception.status == DETECTED and self._vote_ok
        else:
            sighted = confirmed == DETECTED
        if sighted and self._last_confirmed != DETECTED:
            self._last_confirmed = DETECTED
            if self.frame_vote:
                self.stats.vote["fired"] += 1
            return TRIGGER_CANDIDATE
        if confirmed == ABSENT:
            self._last_confirmed = ABSENT

        if perception.status == ABSENT:
            self._absent_streak += 1
            if band and self._band_run >= self.consecutive_frames and not self._band_asked:
                self._band_asked = True
                return TRIGGER_BAND
            if (self._local_first() is not None and self.vocabulary is not None
                    and not self.vocabulary.in_vocabulary
                    and self._absent_streak >= self.consecutive_frames):
                # Only while the local tier takes it: a question due for the
                # cloud waits for cold_search's bar (review, 2026-10-14).
                return TRIGGER_OOV
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

    def __call__(self, frame: dict) -> dict:
        # 3.76: the mission this frame belongs to. A Stop closes the policy
        # from another thread, often while this frame is in perception; the
        # call this frame would fire must then not go out (it was billed,
        # on a worker pool created again and never shut down).
        frame_epoch = self._epoch
        self._read_odometry(frame)
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

        self._frame_stamp = frame.get("captured_at")
        self._count_skipped(frame)
        trigger = self._trigger_for(perception)
        if trigger is None:
            self._since_call += 1
            return self._local_scene(perception)

        if (trigger == TRIGGER_CANDIDATE and self.async_cloud
                and self._inflight is not None):
            # 3.76: the edge into `detected` was spent in _trigger_for; held
            # back by the call in flight, it must fire once that lands, not
            # wait for the staleness floor.
            self._last_confirmed = None
        if trigger == TRIGGER_BAND and self.async_cloud and self._inflight is not None:
            self._band_asked = False   # 3.80.D: the same, for the band
        # 3.80.D: the local tier first, unless it has been unable
        # `local_max_unable` times in a row (then this one is the cloud's).
        local_fn = self._local_first()
        if (local_fn is None and self.max_calls is not None
                and self.stats.cloud_calls >= self.max_calls):
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

        if self._closed or self._epoch != frame_epoch:
            return self._local_scene(perception, note="mission ended")

        if local_fn is not None:
            return self._ask_local_first(frame, perception, trigger, frame_epoch, local_fn)
        escalated = self.local_sighting_fn is not None

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
                return self._local_scene(perception, note="mission ended")
            if escalated:
                # 3.80.D: handed up after `local_max_unable` unable answers.
                self._count("frame_escalations", "unable")
            return self._local_scene(perception, note="dispatched",
                                     dispatched=trigger)

        if escalated:
            self._count("frame_escalations", "unable")
        _t0 = time.perf_counter()
        try:
            scene = self.cloud_vision_fn(frame)
        except Exception as exc:  # noqa: BLE001 -- raised for B3.2 unless 3.80.G says not
            self._note_cloud_answer(trigger, _frame_seq(frame), error=exc)
            if self.cloud_errors_fatal:
                raise
            self._count_cloud_error(exc)
            return self._local_scene(perception, note="cloud call failed")
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
        return self._annotate_noted(scene, perception, trigger, frame)
