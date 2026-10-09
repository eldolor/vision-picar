"""
control/reconfirm.py

3.53 (`docs/plans/ros-alignment/3.53-arrival-reconfirm.md`): **after an
`arrived_unconfirmed` ending, ask the identity question again once the
cloud is back.**

3.47 ends a tiered mission `arrived_unconfirmed` when its arrival holds but
the cloud that must confirm identity cannot be reached. This waits for the
cloud to come back and then puts the question ONCE, to the frame the arrival
was judged on, recording the answer beside the outcome
(`status.late_confirmation`) -- never instead of it: the outcome is not
rewritten and `memory.found` stays false.

* **Probing is free:** `GET <vision_url>/health` does no Bedrock call. Only a
  probe that answers 200 is followed by a paid call.
* **Bounded:** one probe per `interval_s`, for at most `window_s`; at most
  `max_paid` paid attempts.
* **Never touches the robot.** The mission is over and its car stopped; a
  stop from here would halt whoever drives next.
* **Cancelled** by a new mission or a Stop (`dropped`); lost on a brain
  restart (it lives in memory only).
"""

import logging
import threading
import time
from typing import Callable, Optional

import httpx

logger = logging.getLogger("reconfirm")

DEFAULT_MAX_PAID = 2


def health_probe(vision_url: str, secret: str = "", timeout_s: float = 3.0
                 ) -> Callable[[], bool]:
    """A free reachability check against the vision service's /health."""
    url = vision_url.rstrip("/") + "/health"
    headers = {"x-app-secret": secret} if secret else {}

    def probe() -> bool:
        try:
            return httpx.get(url, headers=headers, timeout=timeout_s).status_code == 200
        except httpx.HTTPError:
            return False
    return probe


class Reconfirmer:
    """One late confirmation for one finished runner, on a daemon thread."""

    def __init__(self, runner, probe: Callable[[], bool], *, interval_s: float,
                 window_s: float, max_paid: int = DEFAULT_MAX_PAID,
                 clock: Callable[[], float] = time.monotonic):
        self.runner = runner
        self.probe = probe
        self.interval_s = interval_s
        self.window_s = window_s
        self.max_paid = max_paid
        self.clock = clock
        self._cancelled = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "Reconfirmer":
        self._thread = threading.Thread(target=self.run, name="reconfirm", daemon=True)
        self._thread.start()
        return self

    def cancel(self, reason: str) -> None:
        """Stop asking. Recorded `dropped` if no answer has landed yet; an
        answer that lands after this is ignored (the first state wins).
        Recorded BEFORE the flag is set, so `late_ask()` -- which checks the
        record under the runner's lock -- refuses from this moment on."""
        self.runner.late_update(state="dropped", reason=reason)
        self._cancelled.set()

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def run(self) -> None:
        if not self.runner.late_confirmation_pending():
            return
        self.runner.late_update(state="waiting", reason="cloud unreachable; probing")
        deadline = self.clock() + self.window_s
        probes = failed_asks = 0
        next_paid_at = None
        last_error = None
        while not self._cancelled.is_set():
            if self.clock() >= deadline:
                if failed_asks:
                    # The cloud WAS reached and a paid call failed; saying
                    # "unreachable" would be the opposite of what happened.
                    self.runner.late_update(
                        state="failed",
                        reason=f"{failed_asks} paid attempt(s) failed and the "
                               f"window ran out; last: {last_error}")
                else:
                    self.runner.late_update(
                        state="expired",
                        reason=f"cloud still unreachable after {self.window_s:g}s")
                return
            probes += 1
            try:
                up = bool(self.probe())
            except Exception:  # noqa: BLE001 -- a probe never ends the loop
                up = False
            self.runner.late_update(probes=probes)
            # /health is shallow: it answers while Bedrock is down or
            # throttling (5xx/429), which only a paid call can see. So after
            # a paid call fails, the next one waits half the window -- the two
            # attempts span the outage the window exists for, not 15 s of it.
            ready = next_paid_at is None or self.clock() >= next_paid_at
            if up and ready and not self._cancelled.is_set():
                try:
                    verdict = self.runner.late_ask()
                except TimeoutError as e:
                    # The call is still in flight on its abandoned thread; a
                    # second one beside it would break "one call at a time".
                    self.runner.late_update(
                        state="failed", reason=f"the late confirmation hung: {e}")
                    return
                except Exception as e:  # noqa: BLE001
                    failed_asks += 1
                    logger.warning("late confirmation attempt %d failed: %s", failed_asks, e)
                    if failed_asks >= self.max_paid:
                        self.runner.late_update(
                            state="failed",
                            reason=f"{failed_asks} paid attempts failed; last: {e}")
                        return
                    last_error = e
                    # Half a window on, but inside it: a first failure late
                    # in the window still gets its second attempt.
                    now = self.clock()
                    next_paid_at = min(now + self.window_s / 2,
                                       max(now, deadline - self.interval_s))
                else:
                    if verdict is not None:  # None: a cancel landed first
                        self.runner.late_update(**_state_for(verdict))
                    return
            if self._wait(self.interval_s):
                return

    def _wait(self, seconds: float) -> bool:
        """Sleep between probes; True if cancelled meanwhile. A seam for
        tests that drive the clock."""
        return self._cancelled.wait(seconds)


def _state_for(verdict: dict) -> dict:
    if not verdict.get("cloud_called"):
        return {"state": "not_asked", "confirmed": None,
                "reason": verdict.get("reason") or "no call was made"}
    if verdict.get("confirmed"):
        return {"state": "confirmed", "confirmed": True,
                "reason": verdict.get("reason") or "the cloud sees the target"}
    return {"state": "refused", "confirmed": False,
            "reason": verdict.get("reason") or "the cloud does not see the target"}
