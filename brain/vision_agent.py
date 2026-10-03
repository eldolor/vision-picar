"""
vision_agent.py

Phase S2b (partial) -- the vision policy, in Python at last.

`MissionAgent` peeks left, right and forward with the ultrasonic sensor
and prefers whichever clear direction leads somewhere it has not been.
That policy is rule-based, deterministic, free, and **not on the hardware
path** (`PLAN-sim-hardening.md` 2.2). `VisionAgent` is the one that is:
each step, whatever the camera sees goes to a vision model, and the model
picks the move.

The class is deliberately tiny, because the harness already does
everything else:

- `MissionAgent.step()` records every step into `MissionMemory` -- rooms,
  sightings, action history -- so mission memory is inherited unchanged.
- `ConstrainedAgent.decide()` already validates an action against the
  allowed set and breaks a deadlock (three `STOP`s in a row forces a
  turn), so this class reuses it rather than repeating it.
- `MissionRunner` supplies the per-call timeout and the
  consecutive-failure budget around the vision call itself.

What is left for this class is one sentence: **trust the model's action,
unless the mission is already over.**

## What it does NOT do

**No peeking.** The rule-based policy spends three pan actions and three
distance reads per decision. Against a camera, a peek costs a real move
and buys nothing the photograph does not already show -- the model is
looking at the scene, not at a distance number. Removing the peek also
removes six HTTP round trips per step.

**Room memory comes from the cloud.** A photograph carries no room label;
`/navigate` returns a `room_guess` that `MissionAgent.step()` backfills
into `frame["room"]`, and the searched rooms go back to it on every call
(`brain/navigate.py`, "Room-level step memory";
`docs/guides/AGENT-HARNESS.md` section 10).

**No safety, of its own.** Same as every other policy: the action goes
through `robot/safety.py`, which re-reads the distance sensor and can veto
it. On a backend with no real sensor (a replayed walk), that check is
inert -- see `sim/replay_robot.py`.
"""

import logging
from typing import Optional

from brain.agent import ConstrainedAgent, MissionAgent

logger = logging.getLogger("vision_agent")


class VisionAgent(MissionAgent):
    """Mission agent whose decisions come from the vision model."""

    def decide(self, scene: dict, frame: Optional[dict] = None) -> str:
        if self.memory.is_complete():
            return "STOP"

        # ConstrainedAgent.decide() reads scene["safest_direction"] -- which
        # for this policy is the model's chosen action, mapped in
        # brain/navigate.py -- validates it, and applies the stuck-breaker.
        # Skipping MissionAgent.decide() in between is the whole difference
        # between this policy and the rule-based one.
        action = ConstrainedAgent.decide(self, scene, frame)

        reasoning = (scene.get("_navigate") or {}).get("reasoning", "")
        if reasoning:
            logger.info(f"vision policy chose {action}: {reasoning}")
        return action
