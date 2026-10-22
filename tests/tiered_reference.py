"""3.94: the tiered policy's decisions as they were before the behavior tree,
frozen, so the tree can be checked against them decision for decision
(as `frontier_reference.py` is for 3.77's numpy search).

DO NOT EDIT to follow `brain/tiered.py`: this is the reference. A
deliberate change of behaviour in the policy is a change to its tests, and
this file stays the record of what the code did on 2026-10-22 (`dev` at the
branch point of 3.94).

`ReferenceTieredVision` is `TieredVision` with the old bodies of the methods
the tree replaced, copied verbatim.
"""

from typing import Optional  # noqa: F401

from brain.tiered import (  # noqa: F401 -- the names the frozen bodies use
    SCAN_ACTION, SCAN_TURN_DEG, TURN_ACTIONS, TieredVision, _direction_for)
from brain.perceive import DETECTED, Perception  # noqa: F401


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
