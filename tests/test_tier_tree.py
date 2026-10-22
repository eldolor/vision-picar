"""3.94: the behavior tree, and the tiered policy's decisions drawn with it.

Criterion 2a: on 10,000 seeded random states, the tree gives exactly the
decisions of the frozen code it replaced (`tests/tiered_reference.py`) --
the scene it returns (action, turn size, reasoning, readouts) and the state
it leaves behind. Criterion 4: it costs under 1 ms a frame (p90).
"""

import math
import random
import statistics
import time

import pytest

from brain.perceive import (ABSENT, CLIP_LOGIT_SCALE, DETECTED, UNAVAILABLE, Box,
                            Candidate, Detection, Perception)
from brain.tiered import TieredVision
from brain.tree import (FAILURE, RUNNING, SUCCESS, Action, Condition, Decorator,
                        Fallback, Sequence)
from tests.tiered_reference import ReferenceTieredVision

N_STATES = 10_000


# -- the tree module ----------------------------------------------------------

def test_sequence_stops_at_the_first_child_that_does_not_succeed():
    seen = []
    t = Sequence("s", [Action("a", lambda bb: seen.append("a") or True),
                       Action("b", lambda bb: seen.append("b") or RUNNING),
                       Action("c", lambda bb: seen.append("c") or True)])
    assert t.tick(None) == RUNNING and seen == ["a", "b"]


def test_fallback_stops_at_the_first_child_that_does_not_fail():
    seen = []
    t = Fallback("f", [Condition("a", lambda bb: seen.append("a") and False),
                       Action("b", lambda bb: seen.append("b") or True),
                       Action("c", lambda bb: seen.append("c") or True)])
    assert t.tick(None) == SUCCESS and seen == ["a", "b"]
    assert Fallback("g", [Condition("x", lambda bb: False)]).tick(None) == FAILURE


def test_a_decorator_wraps_its_child_and_a_bad_status_is_refused():
    t = Decorator("not", Condition("c", lambda bb: True),
                  lambda bb, child: FAILURE if child.tick(bb) == SUCCESS else SUCCESS)
    assert t.tick(None) == FAILURE
    with pytest.raises(ValueError):
        Action("bad", lambda bb: "maybe").tick(None)
    with pytest.raises(ValueError):
        Sequence("empty", [])


def test_describe_prints_the_tree_with_its_settings():
    tier = TieredVision(_Pipe(), lambda f: {})
    text = tier.describe_tree()
    assert text.splitlines()[0] == "sequence what to do this frame"
    assert "  decorator spin guard after_quarter_turns=8" in text
    assert "      action scan action=RIGHT" in text


# -- criterion 2a: the tree against the frozen code ------------------------

class _Pipe:
    hfov_deg = 66.0

    def perceive(self, frame):
        raise AssertionError("not called: _local_scene is driven directly")


def _candidate(rng, p=None, bearing="random"):
    p = rng.random() if p is None else p
    s = math.log(max(p, 1e-6) / max(1 - p, 1e-6)) / CLIP_LOGIT_SCALE
    b = rng.uniform(-40, 40) if bearing == "random" else bearing
    return Candidate(detection=Detection(Box(0, 0, 10, 10), "x", 0.9), scores=(s, 0.0),
                     bearing_deg=None if rng.random() < 0.15 else b)


def _perception(rng):
    status = rng.choice([DETECTED, DETECTED, ABSENT, ABSENT, ABSENT, UNAVAILABLE])
    cands = []
    if status != UNAVAILABLE:
        for _ in range(rng.choice([0, 0, 1, 2, 3])):
            # weight the band [0.5, 0.8) and the gate's neighbourhood
            cands.append(_candidate(rng, rng.choice([rng.random(), rng.uniform(0.45, 0.85)])))
    best = None
    if status == DETECTED:
        best = _candidate(rng, rng.uniform(0.8, 1.0),
                          rng.choice([rng.uniform(-40, 40), rng.uniform(-4, 4), None]))
        cands.append(best)
    return Perception(status=status, candidates=cands, best=best,
                      pan_deg=rng.choice([0.0, 0.0, 0.0, 20.0]))


def _config(rng):
    return dict(steer_on_sight=rng.random() < 0.85, steer_on_candidate=rng.random() < 0.6,
                hold_goal=rng.random() < 0.8, hold_bearing=rng.random() < 0.5,
                hold_bearing_max_m=rng.choice([0.5, 1.0, 3.0]),
                spin_guard_after=rng.choice([0, 1, 2, 8]),
                local_band_low=rng.choice([0.5, 0.6]))


def _odo(distance_m, heading_deg, usable=True):
    return {"usable": usable, "distance_m": distance_m if usable else None,
            "heading_deg": heading_deg if usable else None}


def _set_state(tier, rng):
    """The state `_local_scene` reads, at random. Called with the same seed
    on both policies, so they start identical."""
    d, h = 0.0, rng.uniform(-180, 180)
    usable = rng.random() < 0.8
    for _ in range(rng.choice([0, 1, 3])):
        d += rng.uniform(0, 0.8)
        h += rng.uniform(-60, 60)
        tier._read_odometry({"odometry": _odo(d, h, usable)})
    if rng.random() < 0.5 and tier._odom.samples:
        tier._goal.sight(tier._odom.pose, rng.uniform(-60, 60))
        for _ in range(rng.choice([0, 1, 2])):
            d += rng.uniform(0, 1.5)
            h += rng.uniform(-90, 90)
            tier._read_odometry({"odometry": _odo(d, h, usable)})
    r = rng.random()
    tier._local_turn = (None if r < 0.5 else
                        (rng.choice(["LEFT", "RIGHT"]), rng.randint(5, 90)) if r < 0.8 else
                        ("FORWARD", None))
    tier._local_held = rng.choice([None, None, "FORWARD", "LEFT"])
    tier._local_held_why = rng.choice([None, "located", "direction"]) if tier._local_held else None
    tier._last_cloud_scene = rng.choice(
        [None, {"safest_direction": rng.choice(["LEFT", "RIGHT", "FORWARD", "REVERSE"])}])
    tier._inflight = object() if rng.random() < 0.2 else None
    tier._inflight_trigger = "cold_search" if tier._inflight else None
    tier._consecutive_turn_deg = rng.choice([0.0, 45.0, 90.0, 170.0, 360.0, 719.0, 720.0, 800.0])
    tier._candidate_rejected = rng.random() < 0.2
    tier._landed_corroboration = rng.choice([None, {"verdict": "corroborated"}])
    tier._local_landed = rng.choice([None, {"trigger": "x", "answer": "yes"}])


STATE_KEYS = ("_consecutive_turn_deg", "_local_turn", "_dead_reckoned", "_steer_turn_deg",
              "_reckoned_turn_deg", "_landed_corroboration", "_local_landed",
              "_local_held", "_local_held_why", "_candidate_rejected")


def _state(tier):
    goal = tier._goal
    return ({k: getattr(tier, k, None) for k in STATE_KEYS},
            (goal.held, goal.sightings, goal._seen_at, getattr(goal, "_bearing_world", None)),
            tier.stats.as_dict())


def _pair(seed):
    rng = random.Random(seed)
    cfg = _config(rng)
    tiers = []
    for cls in (ReferenceTieredVision, TieredVision):
        t = cls(_Pipe(), lambda f: {}, **cfg)
        _set_state(t, random.Random(seed * 7919 + 1))
        tiers.append(t)
    return tiers, rng


def test_criterion_2a_the_tree_decides_exactly_as_the_code_it_replaced():
    differences, rungs = [], {}
    for seed in range(N_STATES):
        (ref, new), rng = _pair(seed)
        frames = rng.choice([1, 1, 3])          # some state carried across frames
        for i in range(frames):
            p_seed = seed * 31 + i
            note = rng.choice(["", "dispatched", "call cap reached"])
            a = ref._local_scene(_perception(random.Random(p_seed)), note=note)
            b = new._local_scene(_perception(random.Random(p_seed)), note=note)
            if a != b or _state(ref) != _state(new):
                differences.append((seed, i, a.get("safest_direction"), b.get("safest_direction"),
                                    a["_navigate"]["reasoning"], b["_navigate"]["reasoning"]))
            why = b["_navigate"]["reasoning"]
            key = ("spin" if "spin guard" in why else "steer" if "steering on local" in why
                   else "located" if "located" not in why and "saw the target" in why
                   else "reckon" if "dead-reckoning" in why else "candidate" if "uncertain" in why
                   else "held" if "holding" in why else "scan")
            rungs[key] = rungs.get(key, 0) + 1
    assert not differences, f"{len(differences)} differences, first: {differences[:3]}"
    # Every rung was exercised, so 0 differences means something.
    assert set(rungs) == {"spin", "steer", "located", "reckon", "candidate", "held", "scan"}, rungs


def test_criterion_4_the_tree_costs_under_a_millisecond_a_frame():
    times = {"reference": [], "tree": []}
    for seed in range(2_000):
        (ref, new), rng = _pair(seed)
        for name, tier in (("reference", ref), ("tree", new)):
            p = _perception(random.Random(seed))
            t0 = time.perf_counter()
            tier._local_scene(p)
            times[name].append((time.perf_counter() - t0) * 1000)
    p90 = {k: statistics.quantiles(v, n=10)[-1] for k, v in times.items()}
    assert p90["tree"] <= 1.0, p90
