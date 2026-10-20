"""
choice_log.py -- 3.88 A: explore asks a person which goal to send next, and
every choice it then makes is recorded.

**The ask** (`GoalAsk`). Under ask mode (`explore_ask`, default off) explore
pauses at each goal choice and publishes the candidates the rules allow --
the same pool 3.65's learned ranking scores (`ExploreAgent._pool`): frontiers
not reached twice and not set aside, then view gaps not already looked at.
A pick names one candidate by id; an id from an earlier choice, or one never
offered, is refused (the brain answers 409) and nothing is sent. No pick
within the timeout (60 s) takes the rule's own choice, recorded as
`rule_timeout`, never as a person's label. The robot is stopped throughout:
a pause is a mission sending nothing. The time paused is kept here and left
out of the mission's time limit (`MissionRunner._out_of_budget`).

**Corrections are picks, never takeovers** (the user, 2026-10-20). This is
its own message (`POST /mission/explore/pick`), not the person's map goal
(`/world/goal`, driver `twin-dpad`), which explore deliberately neither
adopts nor cancels (3.76 E); a person's Stop or D-pad still ends explore and
latches the brain out (3.84), unchanged.

**The log** (`ChoiceLog`). One JSON line a choice explore carried out (the
goal sent: a pick withdrawn before it went is no record), one file a
mission, each line flushed and fsynced: the candidates' features,
the pick and who made it (`person`, `rule_timeout`; 3.88 B adds `oracle`).
Positions only in the house frame, and never an image. Written only under
ask mode: a mission without it records nothing new.

The lock here is the one place the brain's HTTP thread and the mission's
tick thread meet over an ask; nothing in it calls out.
"""

import json
import logging
import os
import threading
from pathlib import Path
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# 3.88 criterion 1: a pause this long ends in the rule's own pick.
ASK_TIMEOUT_S = 60.0
# Who made a recorded choice.
BY_PERSON = "person"
BY_RULE_TIMEOUT = "rule_timeout"


class GoalAsk:
    """One open question at a time: which of these candidates next?"""

    def __init__(self, clock: Callable[[], float], timeout_s: float = ASK_TIMEOUT_S):
        if not timeout_s or timeout_s <= 0:
            raise ValueError(f"the ask's timeout must be > 0 s; got {timeout_s!r}")
        self.clock = clock
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self.choice = 0                 # the last choice number opened
        self._open: Optional[dict] = None
        self._pick: Optional[str] = None
        self._paused_s = 0.0            # every closed pause, summed
        self._ended = False

    def open(self, candidates: list, default: int) -> Optional[tuple]:
        """Open choice N+1 over `candidates` (dicts), whose rule pick is the
        one at index `default`. Each is copied with its id, `"<N+1>.<index>"`,
        so the number and the ids cannot disagree. Returns (the choice number,
        the copies), or None once the mission has ended (a Stop landed while
        it was chosen)."""
        with self._lock:
            if self._ended:
                return None
            self._close_locked()
            self.choice += 1
            candidates = [dict(c, id=f"{self.choice}.{i}") for i, c in enumerate(candidates)]
            self._open = {"choice": self.choice, "candidates": candidates,
                          "default": candidates[default]["id"], "opened_at": self.clock()}
            self._pick = None
            return self.choice, candidates

    def pick(self, choice, candidate_id) -> tuple:
        """A person's pick, from the brain's HTTP thread: (True, "") once
        taken, else (False, why). Stale, unknown or second picks are refused."""
        with self._lock:
            if self._open is None:
                return False, "no goal choice is open"
            if choice != self._open["choice"]:
                return False, (f"choice {choice} is not the open one "
                               f"(choice {self._open['choice']})")
            if not any(c["id"] == candidate_id for c in self._open["candidates"]):
                return False, f"candidate {candidate_id!r} is not offered in this choice"
            if self._pick is not None:
                return False, f"choice {choice} was already answered"
            if self.clock() - self._open["opened_at"] >= self.timeout_s:
                return False, f"choice {choice} timed out; the rule's pick stands"
            self._pick = candidate_id
            return True, ""

    def poll(self) -> Optional[tuple]:
        """From the tick thread: (by, candidate id) once the open choice is
        decided -- a person's pick, or the rule's at the timeout -- else None."""
        with self._lock:
            if self._open is None:
                return None
            if self._pick is not None:
                return BY_PERSON, self._pick
            if self.clock() - self._open["opened_at"] >= self.timeout_s:
                return BY_RULE_TIMEOUT, self._open["default"]
            return None

    def close(self) -> Optional[float]:
        """End the open pause (decided or withdrawn). Its length, or None."""
        with self._lock:
            return self._close_locked()

    def end(self) -> None:
        """The mission ended: close any pause and refuse every later pick."""
        with self._lock:
            self._close_locked()
            self._ended = True

    def _close_locked(self) -> Optional[float]:
        if self._open is None:
            return None
        waited = max(0.0, self.clock() - self._open["opened_at"])
        self._paused_s += waited
        self._open = None
        self._pick = None
        return waited

    def paused_s(self) -> float:
        """Every pause so far, the open one included."""
        with self._lock:
            open_s = (max(0.0, self.clock() - self._open["opened_at"])
                      if self._open is not None else 0.0)
            return self._paused_s + open_s

    def snapshot(self) -> Optional[dict]:
        """The open choice as `/mission/status` publishes it, or None."""
        with self._lock:
            if self._open is None:
                return None
            waited = max(0.0, self.clock() - self._open["opened_at"])
            return {"choice": self._open["choice"],
                    "candidates": self._open["candidates"],
                    "default": self._open["default"],
                    "picked": self._pick,
                    "waited_s": round(waited, 1),
                    "timeout_s": self.timeout_s,
                    "remaining_s": round(max(0.0, self.timeout_s - waited), 1)}


def _fsync(fd: int) -> None:
    """`os.fsync`, named here so a test can fail it for this module only."""
    os.fsync(fd)


class ChoiceLog:
    """One mission's choices, a JSON line each, appended durably. The file
    is created on the first record, so a mission that never chose leaves
    none behind."""

    def __init__(self, path):
        self.path = Path(path)
        self._lock = threading.Lock()

    def append(self, record: dict) -> None:
        line = json.dumps(record, sort_keys=True, default=str) + "\n"
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fresh = not self.path.exists()
            with open(self.path, "a") as f:
                f.write(line)
                f.flush()
                # Durable, not only written: the Jetson can lose power right
                # after a choice, and the line is a person's label.
                _fsync(f.fileno())
            if fresh:
                # ...and the new file's name, once.
                try:
                    fd = os.open(self.path.parent, os.O_RDONLY)
                except OSError:
                    return
                try:
                    _fsync(fd)
                except OSError:
                    pass
                finally:
                    os.close(fd)


def read_choices(path) -> list:
    """Every whole record in a mission's file. A last line cut off by a power
    loss is skipped, never an error."""
    out = []
    for line in Path(path).read_text().splitlines():
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            logger.warning("%s: skipped a partial line", path)
    return out
