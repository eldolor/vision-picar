"""
tests/test_stop_task_race.py

3.61 (`docs/plans/ros-alignment/3.61-stop-task-race.md`): **a Stop never
forgets a mission that a concurrent Start began.**

`stop_mission()` awaited, then set `state["task"] = None` unconditionally,
erasing the loop task of a Start that landed during its awaits: the brain
lost hold of the loop it was running, and a later Stop could not cancel it.
Driven concurrently over an async client, through brain_server's own
handlers, with 3.53's late-check fixture (a tiered mission that parks
`arrived_unconfirmed`, so a Start is allowed while the Stop is parked).
"""

import asyncio
import logging
import time

import httpx
import pytest

from tests.test_arrival_reconfirm import (  # noqa: F401 -- the fixture
    _ended_over_asgi, reconfirm_app)
from tests.test_bearing_turns import TARGET


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


class SlowFinished:
    """Parks the Stop inside its awaits (the old runner's `finished.wait`)
    long enough for a Start to land."""

    def wait(self, timeout=None):
        time.sleep(1.0)
        return True

    def is_set(self):
        return True


class StopSpy:
    def __init__(self, robot):
        self.robot, self.stops = robot, 0
        real = robot.stop

        def stop(*a, **k):
            self.stops += 1
            return real(*a, **k)
        robot.stop = stop


@pytest.mark.parametrize("reconfirm_app", [1.0], indirect=True)
def test_a_stop_never_forgets_the_task_a_concurrent_start_began(reconfirm_app):
    """Criteria 1-3."""
    app, cloud, runners = reconfirm_app
    brain = app.state.brain_state
    seen = {}

    async def go():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),
                                     base_url="http://brain") as client:
            await _ended_over_asgi(client)
            runners[0].finished = SlowFinished()

            # The window is the Stop's `await task`: an old loop whose
            # cancellation takes a moment (asynchronous cleanup, as a real
            # loop's can) holds the Stop there while the Start lands.
            async def slow_to_cancel():
                try:
                    await asyncio.sleep(3600)
                except asyncio.CancelledError:
                    await asyncio.sleep(1.0)
                    raise
            brain["task"] = asyncio.create_task(slow_to_cancel())
            await asyncio.sleep(0)

            async def start_later():
                await asyncio.sleep(0.3)
                r = await client.post("/mission/start",
                                      json={"target_object": TARGET, "policy": "tiered"})
                seen["new_task"] = brain["task"]
                return r
            # Criterion 3: the first Stop's runner (captured at entry, already
            # finished) must send nothing once a new mission may be driving --
            # on the car both share one robot. Its own stops are counted from
            # here; the new mission's are its own business.
            seen["spy"] = StopSpy(runners[0].robot)
            stop, start = await asyncio.gather(client.post("/mission/stop"), start_later())
            assert stop.status_code == 200 and start.status_code == 200, (stop.text, start.text)
            # Criterion 1: the first Stop finished AFTER the Start.
            seen["task_after_stop"] = brain["task"]
            seen["stops_from_first"] = seen["spy"].stops
            # Criterion 2: a later Stop reaches the new loop.
            steps_before = (await client.get("/mission/status")).json()["step"]
            second = await client.post("/mission/stop")
            assert second.status_code == 200
            seen["steps_before"] = steps_before
            seen["outcome"] = runners[1].status()["outcome"]
    asyncio.run(go())

    new_task = seen["new_task"]
    assert new_task is not None
    assert seen["task_after_stop"] is new_task, "the Stop erased the new mission's loop task"
    assert new_task.done(), "the second Stop never reached the new loop"
    assert seen["stops_from_first"] == 0, "the first Stop sent a stop to the new driver"
    assert seen["outcome"] in ("stopped", "arrived_unconfirmed", "found"), seen["outcome"]
