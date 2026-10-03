"""
health.py

One command that answers "what is wrong with this robot" -- phase M5
(PLAN-microduck-transplants.md).

    python -m control.health
    python -m control.health --json
    python -m control.health --robot-url http://pi.local:8000 --brain-url ...

Exits **non-zero** when either half is unhealthy or unreachable, so it can
gate a deploy (M11 rolls a release back on it) and be run from a timer
after boot without anyone reading the output.

## Why one command

Microduck's argument, and it is about the question rather than the code:
"what is wrong with this robot" does not divide into hardware and software
until *after* it is answered (`architecture` section 8.4). Two health
routes that each report themselves fine, and a person left to work out
that the brain is talking to the wrong robot, is not an answer.

## The rule this file exists to hold: what may reach the verdict

Microduck's `robotd-design` section 3.4, invariant 5: only conditions a
release can be **blamed for** reach the verdict. A robot updated on a low
battery must not roll its release back and then judge the replacement on
the same battery.

So the verdict is built from exactly three kinds of input:

1. **Each process is reachable** and answers its health route.
2. **The robot's watchdog loop is still running** -- not "the robot has
   been quiet", which is a fact about clients, but "the guard that is
   supposed to act on that silence ran recently".
3. **The brain's mission loop is still turning**, when a mission is
   running: no tick has completed within the mission's own
   `tick_timeout_s`. A loop that is alive, answers every request and has
   stopped ticking is the failure this catches, and B3.3 structurally
   cannot -- it aborts a hung tick from *inside* the loop that is gone.

Everything else is **description**: the distance reading, the watchdog's
measured silence, who is driving, the last refusal, the tick rate, and
later the battery and motor temperature. They are printed, they are in
`--json`, and they never change the exit code. When you add a field, put
it in one of those two lists deliberately -- the drift this rule prevents
is a health check that goes red because a robot is sitting still, which
teaches everyone to ignore it.

A rate is the one specified input that is deliberately description here:
there is no knowable target for it. See
`control/mission_runner.py:_tick_rate_hz()`.
"""

import argparse
import json
import sys
from typing import Optional

import httpx

DEFAULT_ROBOT_URL = "http://127.0.0.1:8000"
DEFAULT_BRAIN_URL = "http://127.0.0.1:8001"
DEFAULT_TIMEOUT_S = 5.0

# How many watchdog poll intervals may pass before the loop counts as
# stalled. Ten, because the loop wakes every 0.1s and a single slow event
# loop tick is not a fault -- the failure being caught is a task that is
# gone, not one that is late.
WATCHDOG_POLL_SLACK = 10

OK = "ok"
UNHEALTHY = "unhealthy"
UNREACHABLE = "unreachable"


def _get(url: str, path: str, secret: Optional[str], timeout: float) -> dict:
    headers = {"x-app-secret": secret} if secret else {}
    response = httpx.get(f"{url.rstrip('/')}{path}", headers=headers, timeout=timeout)
    response.raise_for_status()
    return response.json()


def check_robot(url: str, secret=None, timeout=DEFAULT_TIMEOUT_S) -> dict:
    try:
        body = _get(url, "/health", secret, timeout)
    except Exception as e:  # noqa: BLE001 -- every failure is the same answer
        return {"name": "robot", "url": url, "status": UNREACHABLE, "problems": [str(e)]}

    problems = []

    # Verdict input 2. Absent on a server older than M5, which cannot be
    # judged on a field it does not publish -- reported as description
    # instead of failing the whole check.
    poll_age = body.get("seconds_since_watchdog_poll")
    interval = body.get("watchdog_poll_interval_s")
    if poll_age is not None and interval:
        if poll_age > interval * WATCHDOG_POLL_SLACK:
            problems.append(
                f"watchdog loop has not polled for {poll_age}s "
                f"(wakes every {interval}s) -- the guard is not running"
            )

    return {
        "name": "robot",
        "url": url,
        "status": UNHEALTHY if problems else OK,
        "problems": problems,
        "identity": body.get("identity"),
        # Description only. Every one of these is a fact about the robot's
        # situation, not about whether this build works.
        "description": {
            "mode": body.get("mode"),
            "seconds_since_last_command": body.get("seconds_since_last_command"),
            "watchdog_timeout_s": body.get("watchdog_timeout_s"),
            "watchdog_poll_age_s": poll_age,
            "driver": body.get("driver"),
            "authority_holder": body.get("authority_holder"),
            "last_refusal": body.get("last_refusal"),
            "env_label": body.get("env_label") or None,
            # 3.34: the motor board's link. Description, not a verdict input:
            # a board goes quiet for reasons no release is to blame for (a
            # cable, a flat battery), and the body already refuses to drive
            # without it -- `no_feedback` in last_refusal says so.
            "motor_board": body.get("motor_board"),
        },
    }


def check_brain(url: str, secret=None, timeout=DEFAULT_TIMEOUT_S) -> dict:
    try:
        body = _get(url, "/health", secret, timeout)
    except Exception as e:  # noqa: BLE001
        return {"name": "brain", "url": url, "status": UNREACHABLE, "problems": [str(e)]}

    problems = []

    # Verdict input 3.
    since_tick = body.get("seconds_since_last_tick")
    deadline = body.get("tick_timeout_s")
    if body.get("mission_running") and since_tick is not None and deadline:
        if since_tick > deadline:
            problems.append(
                f"a mission is running but no tick has completed for {since_tick}s "
                f"(deadline {deadline}s) -- the mission loop is not turning"
            )

    return {
        "name": "brain",
        "url": url,
        "status": UNHEALTHY if problems else OK,
        "problems": problems,
        "identity": body.get("identity"),
        "description": {
            "robot_url": body.get("robot_url"),
            "mission_running": body.get("mission_running"),
            "seconds_since_last_tick": since_tick,
            "tick_rate_hz": body.get("tick_rate_hz"),
            "navigate_model_id": body.get("navigate_model_id"),
            "navigate_prompt_variant": body.get("navigate_prompt_variant"),
        },
    }


def check(robot_url: str, brain_url: str, secret=None,
          timeout=DEFAULT_TIMEOUT_S) -> dict:
    parts = [check_robot(robot_url, secret, timeout),
             check_brain(brain_url, secret, timeout)]
    unhealthy = [p for p in parts if p["status"] != OK]
    return {
        "status": OK if not unhealthy else UNHEALTHY,
        "failed": [p["name"] for p in unhealthy],
        "parts": parts,
    }


def render(report: dict) -> str:
    lines = []
    verdict = report["status"].upper()
    if report["failed"]:
        verdict += "  (" + ", ".join(report["failed"]) + ")"
    lines.append(verdict)
    for part in report["parts"]:
        lines.append("")
        lines.append(f"{part['name']}  {part['status']}  {part['url']}")
        ident = part.get("identity")
        if ident:
            lines.append(f"  build     git={ident.get('git_revision')} "
                         f"exe={ident.get('executable')}")
        for problem in part.get("problems", []):
            lines.append(f"  PROBLEM   {problem}")
        for key, value in (part.get("description") or {}).items():
            if value is None or value == {}:
                continue
            lines.append(f"  {key:<24} {value}")
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="One health answer for the whole robot (phase M5).")
    parser.add_argument("--robot-url", default=DEFAULT_ROBOT_URL)
    parser.add_argument("--brain-url", default=DEFAULT_BRAIN_URL)
    parser.add_argument("--secret", default=None,
                        help="x-app-secret, if the services require one")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    report = check(args.robot_url, args.brain_url, args.secret, args.timeout)
    print(json.dumps(report, indent=2) if args.as_json else render(report))
    return 0 if report["status"] == OK else 1


if __name__ == "__main__":
    sys.exit(main())
