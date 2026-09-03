"""
tests/test_authority.py

Phase M4 (PLAN-microduck-transplants.md) -- refusals are state, and manual
preempts autonomous.

Two gaps, both real before this file existed.

**Nothing on the robot server knew that two drivers existed.** The twin's
D-pad and a remote mission both posted to `/action`, and the later one
simply won -- last-writer-wins between two loops that each read the
other's moves as the world changing under them. The only guards anywhere
were the twin refusing to start its local loop during a remote mission,
and the brain's 409 on a second `/mission/start`; neither is on the robot,
and neither sees the D-pad at all. Microduck lists authority priority as
something to *decide* rather than let emerge (`architecture` section 6),
and the decision is written in `AGENT-HARNESS.md`:

    stop  >  manual D-pad  >  remote mission  >  local brain

**And a refusal carried no reason.** `{"executed": false}` plus prose was
all a client got, so a move refused because someone else had taken the
robot was indistinguishable from one refused for being about to hit a
wall -- two situations whose correct responses are opposites (retry later
vs. stop, you are not driving). Microduck's `robotd-design` section 3.2:
a teleop UI showing the stick forward and the robot still is unusable.

Run with: pytest tests/test_authority.py -v
"""

import time

import pytest

from control.mission_runner import PREEMPTED
from control.remote_robot import RemoteRobot
from robot.interface import Preempted
from robot.safety import SafetyViolation
from robot.server import create_app
from tests.conftest import ASGI_BASE_URL, asgi_client


@pytest.fixture
def server():
    """One robot server app, plus a helper to post as any driver.

    A shared client on purpose: the whole subject here is two drivers
    reaching the *same* robot, which is exactly what a per-driver fixture
    would hide.
    """
    client = asgi_client(create_app())

    def act(action, driver=None, path="/action"):
        headers = {"x-driver": driver} if driver else {}
        body = {"action": action} if path == "/action" else {}
        return client.post(ASGI_BASE_URL + path, json=body, headers=headers).json()

    def health():
        return client.get(ASGI_BASE_URL + "/health").json()

    yield act, health, client
    client.close()


# ---------- the order, on the server ----------


def test_a_lower_priority_driver_is_refused_while_a_higher_one_drives(server):
    """The headline. Re-introduce last-writer-wins -- let /action accept
    whoever asked most recently -- and this goes red."""
    act, _, _ = server
    assert act("LOOK_LEFT", "twin-dpad")["executed"] is True

    refused = act("LOOK_RIGHT", "brain")
    assert refused["executed"] is False
    assert refused["reason"] == "preempted"
    assert "twin-dpad" in refused["detail"], refused["detail"]


def test_a_higher_priority_driver_takes_the_robot_from_a_lower_one(server):
    """The direction that must always work: a person interrupting a
    mission. An arbitration that refused this would be worse than none."""
    act, health, _ = server
    assert act("LOOK_LEFT", "brain")["executed"] is True
    assert act("LOOK_RIGHT", "twin-dpad")["executed"] is True
    assert health()["authority_holder"] == "twin-dpad"


def test_the_same_driver_never_fights_itself(server):
    """Equal rank passes. Two D-pad taps, or a mission's own successive
    ticks, must not arbitrate against each other."""
    act, _, _ = server
    for _ in range(5):
        assert act("LOOK_CENTER", "twin-dpad")["executed"] is True
    for _ in range(5):
        assert act("LOOK_CENTER", "brain")["executed"] is False


def test_the_local_brain_ranks_below_the_remote_one(server):
    """The bottom of the order: a loop in a browser tab yields to a loop on
    the robot's own network, which yields to a person."""
    act, _, _ = server
    assert act("LOOK_LEFT", "brain")["executed"] is True
    assert act("LOOK_RIGHT", "twin-local-brain")["reason"] == "preempted"


def test_an_unnamed_command_ranks_as_a_person(server):
    """Deliberate, and the reasoning is in robot/interface.py: the callers
    that do not name themselves are a person with curl, a script run by
    hand, or a test. Ranking them low would mean a running mission ignores
    a human's direct command, which is what the order forbids."""
    act, _, _ = server
    assert act("LOOK_LEFT", "brain")["executed"] is True
    assert act("LOOK_RIGHT")["executed"] is True


# ---------- authority lapses on silence ----------


def test_authority_lapses_so_the_next_driver_starts_clean(server):
    """Otherwise one D-pad tap locks the brain out forever and there is no
    release call to forget. Authority rides the deadman the server already
    keeps: `watchdog_timeout_s` after the last command the motors stop and
    the claim goes with them."""
    act, health, client = server
    act("LOOK_LEFT", "twin-dpad")
    assert health()["authority_holder"] == "twin-dpad"
    assert act("LOOK_RIGHT", "brain")["executed"] is False

    # The app under test is built from config/robot.yaml, whose
    # watchdog_timeout_s is 1.0s. Waiting it out is the behaviour, not an
    # implementation detail -- there is nothing else to call.
    timeout = health()["watchdog_timeout_s"]
    time.sleep(timeout + 0.2)

    assert health()["authority_holder"] is None
    assert health()["driver"] == "twin-dpad", "who last drove is still known"
    assert act("LOOK_RIGHT", "brain")["executed"] is True


def test_stop_is_allowed_from_anyone_and_claims_nothing(server):
    """Top of the order. A stop that could be refused because someone else
    is driving is not a stop -- and stopping is not a bid to drive, so the
    holder keeps its claim."""
    act, health, _ = server
    act("LOOK_LEFT", "twin-dpad")
    assert act("STOP", "brain", path="/stop")["executed"] is True
    assert health()["authority_holder"] == "twin-dpad", (
        "a stop must not hand the robot to whoever sent it"
    )
    assert act("LOOK_RIGHT", "brain")["executed"] is False


# ---------- refusals carry a reason ----------


def test_a_safety_veto_and_a_preemption_are_told_apart_on_the_wire(server):
    """The two refusals have opposite correct responses, so a client must
    not have to read prose to tell them apart."""
    act, _, _ = server
    # Drive down the hallway into its far wall as the only driver. Driving
    # until it refuses rather than counting moves: the start pose is the
    # map's business, not this test's.
    vetoed = None
    for _ in range(30):
        result = act("FORWARD", "twin-dpad")
        if result["executed"] is False:
            vetoed = result
            break
    assert vetoed is not None, "never reached a wall in 30 moves"
    assert vetoed["reason"] == "safety_distance", vetoed

    preempted = act("FORWARD", "brain")
    assert preempted["reason"] == "preempted"


def test_health_reports_who_is_driving_and_what_was_last_refused(server):
    """"The robot is not moving -- what stopped it?" has to be answerable
    from one place, which is what the twin's readout reads."""
    act, health, _ = server
    act("LOOK_LEFT", "twin-dpad")
    act("LOOK_RIGHT", "brain")

    body = health()
    assert body["driver"] == "twin-dpad"
    assert body["last_refusal"]["reason"] == "preempted"
    assert body["last_refusal"]["driver"] == "brain", "the refused party, not the holder"
    assert body["last_refusal"]["seconds_ago"] >= 0


# ---------- the client half ----------


def test_remote_robot_raises_preempted_not_a_safety_violation():
    """Retrying is the right response to a safety veto and exactly the
    wrong one to a preemption, so these must not be the same exception."""
    client = asgi_client(create_app())
    try:
        client.post(ASGI_BASE_URL + "/action", json={"action": "LOOK_LEFT"},
                    headers={"x-driver": "twin-dpad"})
        brain = RemoteRobot(ASGI_BASE_URL, client=client)  # driver: "brain"
        with pytest.raises(Preempted):
            brain.turn_left()
    finally:
        client.close()


def test_a_server_older_than_m4_still_reads_as_a_safety_veto():
    """A refusal with no `reason` is a pre-M4 server, where the only thing
    that ever refused was the distance check. Branching on the reason
    rather than the prose is what makes that fallback exact."""
    import httpx

    def handler(request):
        return httpx.Response(200, json={"executed": False, "detail": "too close"})

    bot = RemoteRobot("http://robot.test",
                      client=httpx.Client(transport=httpx.MockTransport(handler)))
    with pytest.raises(SafetyViolation):
        bot.drive_forward()


# ---------- a mission, preempted ----------


def test_a_foreign_driver_between_ticks_ends_the_mission_preempted():
    """M4's own test, end to end: a person taps the D-pad while a mission
    is running.

    The mission must end `preempted` -- not `failed`, which is where a dead
    AWS link goes; nothing went wrong here, the brain was outranked. And
    the robot must be stopped, because a brain that has lost an arbitration
    does not get to keep driving.

    Remove the arbitration from `/action` (let the last writer win) and the
    brain's next tick succeeds instead, the mission runs to completion, and
    this goes red on the outcome.
    """
    from control.mission_runner import MissionRunner

    client = asgi_client(create_app())
    try:
        robot = RemoteRobot(ASGI_BASE_URL, client=client)  # driver: "brain"
        runner = MissionRunner(robot, target_object="red backpack", max_steps=50)
        runner.start()
        assert runner.tick() is True, "the mission should get going first"

        # The person, arriving between two ticks.
        taken = client.post(ASGI_BASE_URL + "/action", json={"action": "LOOK_LEFT"},
                            headers={"x-driver": "twin-dpad"}).json()
        assert taken["executed"] is True

        while runner.tick():
            pass

        status = runner.status()
        assert status["outcome"] == PREEMPTED, status
        assert status["running"] is False
        assert "twin-dpad" in status["log_tail"][-1], status["log_tail"][-1]
    finally:
        client.close()


def test_a_preemption_is_not_counted_against_the_vision_budget():
    """B3.2's budget exists for a flaky link. A person taking the robot is
    not a flaky link, and burning a retry on it would both mislabel the
    outcome and give the brain two more chances to fight a human for the
    car."""
    from control.mission_runner import MissionRunner

    client = asgi_client(create_app())
    try:
        robot = RemoteRobot(ASGI_BASE_URL, client=client)
        runner = MissionRunner(robot, target_object="red backpack", max_steps=50)
        runner.start()
        runner.tick()
        client.post(ASGI_BASE_URL + "/action", json={"action": "LOOK_LEFT"},
                    headers={"x-driver": "twin-dpad"})
        while runner.tick():
            pass
        assert runner.status()["vision_failures"] == 0
    finally:
        client.close()
