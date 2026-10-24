"""3.94: the behavior tree, and the tiered policy's decisions drawn with it.

Criterion 2a: on 10,000 seeded random states, the tree gives exactly the
decisions of the frozen code it replaced (`tests/tiered_reference.py`) --
the scene it returns (action, turn size, reasoning, readouts) and the state
it leaves behind. Criterion 4: it costs under 1 ms a frame (p90).
"""

import math
import random
from concurrent.futures import Future
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
                  lambda bb, child_status: FAILURE if child_status == SUCCESS else SUCCESS)
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

    def __init__(self, match_probability=None):
        self.match_probability = match_probability

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
    return dict(match_probability=rng.choice([None, None, 0.7, 0.9]),
                steer_on_sight=rng.random() < 0.85, steer_on_candidate=rng.random() < 0.6,
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
                        (rng.choice(["LEFT", "RIGHT"]), rng.randint(5, 90)) if r < 0.75 else
                        ("FORWARD", rng.choice([None, 12])))
    tier._local_held = rng.choice([None, None, "FORWARD", "LEFT", "RIGHT", "REVERSE"])
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


# 3.91.C changes behaviour on purpose (a remembered no, the disagreement
# question): off here, and its counters left out, so the trees are compared
# with the code they replaced (tests/test_tier_remember_no.py covers it on).
C_OFF = {"remember_no": False}
C_STATS = ("rejections", "releases", "cannot_tell", "masked_frames")


def _off(cls):
    # Both: the reference is TieredVision with the old bodies, and inherits
    # the rest, 3.91.C's switches included.
    return C_OFF


def _stats(tier):
    # Both: the reference counts with this module's TierStats.
    out = tier.stats.as_dict()
    for k in C_STATS:
        out.pop(k, None)
    return out


def _scene(scene):
    """The new policy's scene without 3.91.C's readout (None / False with C
    off)."""
    if isinstance(scene, dict) and isinstance(scene.get("_tier"), dict):
        scene = {**scene, "_tier": {k: v for k, v in scene["_tier"].items()
                                    if k not in ("rejected", "probation", "candidate_refused")}}
    return scene


def _state(tier):
    goal = tier._goal
    return ({k: getattr(tier, k, None) for k in STATE_KEYS},
            (goal.held, goal.sightings, goal._seen_at, goal.direction_deg, goal.x, goal.y,
             goal.range_m),
            _stats(tier))


def _pair(seed):
    rng = random.Random(seed)
    cfg = _config(rng)
    tiers = []
    mp = cfg.pop("match_probability")
    for cls in (ReferenceTieredVision, TieredVision):
        t = cls(_Pipe(mp), lambda f: {}, **cfg, **_off(cls))
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
            kw = rng.choice([{}, {}, {"dispatched": "cold_search"}, {"local_asked": "uncertain_band"}])
            if i and rng.random() < 0.5:        # the robot moved between frames
                odo = _odo(rng.uniform(0, 3), rng.uniform(-180, 180), rng.random() < 0.9)
                ref._read_odometry({"odometry": odo})
                new._read_odometry({"odometry": odo})
            a = ref._local_scene(_perception(random.Random(p_seed)), note=note, **kw)
            b = _scene(new._local_scene(_perception(random.Random(p_seed)), note=note, **kw))
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


def test_describe_reads_the_settings_live():
    tier = TieredVision(_Pipe(), lambda f: {})
    tier.spin_guard_after = 3
    assert "decorator spin guard after_quarter_turns=3" in tier.describe_tree()


def test_a_ladder_that_chooses_nothing_is_an_error():
    tier = TieredVision(_Pipe(), lambda f: {})
    tier._rung_scan = lambda bb: False             # as if the default rung were missing
    tier._what_tree = tier._build_what_tree()
    with pytest.raises(RuntimeError, match="chose no direction"):
        tier._local_scene(Perception(status=ABSENT))


# -- 3.94.B: when to ask -------------------------------------------------------

TRIGGER_STATE = ("_started", "_retry_start", "_run", "_run_since", "_votes", "_vote_ok",
                 "_band_run", "_band_asked", "_candidate_rejected", "_absent_streak",
                 "_last_confirmed", "_since_call")


def _trigger_pair(seed):
    import types
    rng = random.Random(seed)
    cfg = dict(consecutive_frames=rng.choice([1, 2, 2, 3]),
               sighting_span_s=rng.choice([None, 0.25]),
               frame_vote=rng.choice([None, None, (2, 3), (3, 5)]),
               cold_search_after=rng.choice([3, 6, 8]),
               cold_search_after_cm=rng.choice([None, None, 50.0]),
               stale_after=rng.choice([0, 4, 8]),
               max_calls=rng.choice([None, None, 3]),
               oov_cold_search_after=rng.choice([None, 2]),
               local_cold_search_after=rng.choice([None, 1]),
               local_max_unable=rng.choice([1, 2]),
               local_sighting_fn=rng.choice([None, lambda frame, box=False: {}]),
               local_band_low=rng.choice([0.5, 0.6]))
    mp = rng.choice([None, 0.7, 0.8])
    st = random.Random(seed * 7919 + 3)
    state = dict(_started=st.random() < 0.8, _retry_start=st.random() < 0.15,
                 _local_track=st.random() < 0.3, _frame_unable=st.choice([0, 0, 1, 2, 3]),
                 cloud_calls=st.choice([0, 1, 3, 5]),
                 vocabulary=st.choice([None, types.SimpleNamespace(in_vocabulary=True),
                                       types.SimpleNamespace(in_vocabulary=False)]),
                 _absent_streak=st.choice([0, 1, 2, 5, 7, 9]),
                 _last_confirmed=st.choice([ABSENT, DETECTED]),
                 _since_call=st.choice([0, 3, 4, 8, 12]),
                 _distance_since_call=st.choice([None, 0.0, 30.0, 60.0]),
                 _band_run=st.choice([0, 1, 2, 3]), _band_asked=st.random() < 0.3,
                 _candidate_rejected=st.random() < 0.2)
    tiers = []
    for cls in (ReferenceTieredVision, TieredVision):
        t = cls(_Pipe(mp), lambda f: {}, **cfg, **_off(cls))
        for k, v in state.items():
            if k == "cloud_calls":
                t.stats.cloud_calls = v
            else:
                setattr(t, k, v)
        tiers.append(t)
    return tiers, rng


def _trigger_state(t):
    return ({k: (list(v) if isinstance(v, list) else v)
             for k, v in ((k, getattr(t, k)) for k in TRIGGER_STATE)},
            dict(t.stats.vote or {}))


def test_criterion_2a_when_to_ask_exactly_as_the_code_it_replaced():
    differences, fired = [], {}
    for seed in range(N_STATES):
        (ref, new), rng = _trigger_pair(seed)
        stamp = rng.choice([None, 100.0])
        for i in range(rng.choice([1, 3, 8])):
            if stamp is not None:
                stamp += rng.choice([0.0, 0.1, 0.25, 0.6])
            repeat = rng.random() < 0.1
            p_seed = seed * 131 + i
            if i and rng.random() < 0.3:
                # What the dispatch path does between frames: an answer landed
                # (the pacing counts reset), a question went unanswered (an edge
                # re-armed), a band no, the robot moved (review of B).
                change = dict(rng.choice([
                    {"_since_call": 0, "_absent_streak": 0, "_frame_unable": 0},
                    {"_frame_unable": rng.choice([1, 2, 3])},
                    {"_retry_start": True}, {"_last_confirmed": ABSENT},
                    {"_candidate_rejected": True}, {"_local_track": rng.random() < 0.5},
                    {"_distance_since_call": rng.choice([None, 10.0, 80.0])},
                    {"_since_call": rng.choice([3, 9])}]))
                for t in (ref, new):
                    for k, v in change.items():
                        setattr(t, k, v)
            got = []
            for t in (ref, new):
                t._frame_stamp, t._repeat_frame = stamp, repeat
                got.append(t._trigger_for(_perception(random.Random(p_seed))))
            if got[0] != got[1] or _trigger_state(ref) != _trigger_state(new):
                differences.append((seed, i, got))
            fired[got[1]] = fired.get(got[1], 0) + 1
    assert not differences, f"{len(differences)} differences, first: {differences[:3]}"
    from brain import tiered as t
    assert set(fired) == {None, t.TRIGGER_START, t.TRIGGER_TRACK, t.TRIGGER_CANDIDATE,
                          t.TRIGGER_BAND, t.TRIGGER_OOV, t.TRIGGER_COLD_SEARCH,
                          t.TRIGGER_STALE}, fired


def test_when_to_ask_prints_as_a_tree():
    tier = TieredVision(_Pipe(), lambda f: {})
    text = tier.describe_when_tree()
    assert text.splitlines()[0] == "sequence when to ask"
    assert "action staleness after=8" in text


def test_criterion_4_when_to_ask_costs_under_a_millisecond_a_frame():
    times = []
    for seed in range(2_000):
        (_, new), rng = _trigger_pair(seed)
        new._frame_stamp = None
        p = _perception(random.Random(seed))
        t0 = time.perf_counter()
        new._trigger_for(p)
        times.append((time.perf_counter() - t0) * 1000)
    assert statistics.quantiles(times, n=10)[-1] <= 1.0


# -- 3.94.C: who to ask --------------------------------------------------------

class _ScriptPipe(_Pipe):
    def __init__(self, seed, match_probability=None):
        super().__init__(match_probability)
        self.rng = random.Random(seed)
        self.tier = None

    def perceive(self, frame):
        if self.tier is not None and self.rng.random() < 0.05:
            self.tier._epoch += 1        # a Stop lands while the frame is in perception
        return _perception(self.rng)


def _route_pair(seed):
    from brain.local_vlm import LocalVLMAbsent
    rng = random.Random(seed)
    async_cloud = rng.random() < 0.5
    fatal = rng.random() < 0.5
    has_local = rng.random() < 0.6
    cfg = dict(async_cloud=async_cloud, cloud_errors_fatal=fatal,
               max_calls=rng.choice([None, None, 1, 3]),
               local_max_unable=rng.choice([1, 2]),
               consecutive_frames=rng.choice([1, 2]),
               cold_search_after=rng.choice([2, 4]), stale_after=rng.choice([0, 3]),
               sighting_span_s=None)
    tiers, logs = [], []
    for cls in (ReferenceTieredVision, TieredVision):
        log = {"cloud": 0, "local": 0, "dispatch": []}
        crng, lrng, drng = (random.Random(seed * 3 + k) for k in range(3))

        def cloud(frame, log=log, crng=crng):
            log["cloud"] += 1
            if crng.random() < 0.15:
                raise RuntimeError("cloud down")
            return {"safest_direction": crng.choice(["LEFT", "RIGHT", "FORWARD"]),
                    "obstacles_ahead": [], "important_objects": [],
                    "_navigate": {"target_visible": crng.random() < 0.3}}

        def local(frame, box=False, log=log, lrng=lrng):
            log["local"] += 1
            r = lrng.random()
            if r < 0.1:
                raise LocalVLMAbsent("down")
            if r < 0.25:
                return {"status": "timeout"}
            return {"status": "answered", "score": lrng.random(),
                    "box": [100, 100, 200, 200] if lrng.random() < 0.5 else None}

        t = cls(_ScriptPipe(seed * 11), cloud, local_sighting_fn=local if has_local else None,
                **cfg, **_off(cls))
        t.pipeline.tier = t

        def dispatch(frame, trigger, epoch=None, tier="cloud", job=None, log=log, drng=drng,
                     t=t):
            # As the real one: an accepted call is in flight until a later frame
            # lands it (review of C), as a cloud scene or a failure.
            ok = drng.random() < 0.9
            log["dispatch"].append((trigger, tier, ok))
            if ok:
                t._inflight, t._inflight_trigger, t._inflight_tier = Future(), trigger, tier
            return ok
        t._dispatch = dispatch
        tiers.append(t)
        logs.append(log)
    return tiers, logs, rng


ROUTE_STATE = ("_since_call", "_absent_streak", "_last_confirmed", "_band_asked",
               "_frame_unable", "_last_cloud_scene", "_local_held", "_local_held_why",
               "_local_turn", "_retry_start", "_local_landed", "_inflight_trigger",
               "_inflight_tier", "_distance_since_call", "_pending_error")


def test_criterion_2a_who_to_ask_exactly_as_the_code_it_replaced(monkeypatch):
    import types
    import brain.tiered as tiered_mod
    import tests.tiered_reference as ref_mod
    # No timing noise: a clock for these two modules only, not the process.
    clock = types.SimpleNamespace(perf_counter=lambda: 0.0, monotonic=time.monotonic,
                                  time=time.time, sleep=time.sleep)
    monkeypatch.setattr(tiered_mod, "time", clock)
    monkeypatch.setattr(ref_mod, "time", clock)
    differences, routes = [], {}
    for seed in range(N_STATES // 4):
        (ref, new), logs, rng = _route_pair(seed)
        for i in range(rng.choice([1, 4, 10])):
            held_now = rng.choice([None, None, (("LEFT", 20), "FORWARD", "located"),
                                   (None, "RIGHT", "direction")])
            land = rng.random()
            outcome = rng.choice(["scene", "error"])
            arrival_busy = rng.random() < 0.05
            closed = rng.random() < 0.05
            out = []
            for t in (ref, new):
                fut = t._inflight
                if fut is not None and not fut.done() and land < 0.5:
                    # A call out since an earlier frame lands now.
                    if outcome == "scene":
                        fut.set_result({"safest_direction": "LEFT", "obstacles_ahead": [],
                                        "important_objects": [],
                                        "_navigate": {"target_visible": False}})
                    else:
                        fut.set_exception(RuntimeError("late failure"))
                t._local_inflight = object() if arrival_busy else None
                t._closed = closed
                if held_now is not None:
                    # A located yes or a held local answer waiting (review of C).
                    t._local_turn, t._local_held, t._local_held_why = held_now
                try:
                    r = t({"seq": i, "image_width": 640})
                except Exception as e:  # noqa: BLE001 -- the same failure on both is equivalence
                    r = ("raised", type(e).__name__, str(e))
                out.append(r)
            state = [({k: getattr(t, k) for k in ROUTE_STATE}, _stats(t)) for t in (ref, new)]
            if out[0] != _scene(out[1]) or state[0] != state[1] or logs[0] != logs[1]:
                differences.append((seed, i))
            r = out[1]
            key = (r[0] if isinstance(r, tuple) else
                   r["_navigate"].get("reasoning", "cloud answered").split("] ", 1)[-1]
                   .split(" -- ")[0][:30])
            routes[key] = routes.get(key, 0) + 1
    assert not differences, f"{len(differences)} differences, first: {differences[:3]}"
    for want in ("call cap reached", "call still in flight", "mission ended",
                 "asked the local tier", "dispatched", "raised", "cloud answered",
                 "cloud call failed", "no trigger", "local VLM busy with the arrival check"):
        assert any(want[:30] in k for k in routes), (want, routes)


def test_who_to_ask_prints_as_a_tree_with_live_settings():
    tier = TieredVision(_Pipe(), lambda f: {})
    tier.max_calls = 7
    text = tier.describe_who_tree()
    assert text.splitlines()[0] == "fallback who to ask"
    assert "action the call cap max_calls=7" in text


def test_a_who_tree_that_ends_without_an_answer_is_an_error():
    tier = TieredVision(_ScriptPipe(1), lambda f: {}, max_calls=0)
    tier._who_tree = Fallback("empty", [Action("never", lambda bb: False)])
    with pytest.raises(RuntimeError, match="no route answered"):
        tier({"seq": 0})


# -- 3.94.D: the tree from config (criterion 3) -------------------------------

def test_criterion_3_the_config_defaults_are_todays_values():
    from control.brain_config import TIER_TREE_DEFAULTS, TIER_TREE_ABSENT_EVENTS, \
        TIER_TREE_RUNGS, load_brain_config
    from brain import tiered
    assert tiered.TREE_DEFAULTS == TIER_TREE_DEFAULTS
    assert list(TIER_TREE_RUNGS) == tiered.TREE_DEFAULTS["what_rungs"]
    assert list(TIER_TREE_ABSENT_EVENTS) == tiered.TREE_DEFAULTS["absent_events"]
    assert load_brain_config()["tier_tree"] == TIER_TREE_DEFAULTS     # robot.yaml as shipped


@pytest.mark.parametrize("bad, words", [
    ({"what_rungs": ["steer", "held"]}, "must end with 'scan'"),
    ({"what_rungs": ["steer", "steer", "scan"]}, "names one twice"),
    ({"what_rungs": ["steer", "fly", "scan"]}, "unknown ['fly']"),
    ({"absent_events": ["band", "stale"]}, "unknown ['stale']"),
    ({"who_order": ["cloud", "mission_ended"]}, "hard rules"),
    ({"steer_band_deg": 0}, "steer_band_deg must be in (0, 3.0]"),
    ({"steer_band_deg": 6}, "steer_band_deg must be in (0, 3.0]"),     # past the arrival's band
    ({"max_turn_deg": 30}, "scan_turn_deg must be in [5, 30]"),        # the scan outside the bounds
    ({"scan_turn_deg": 45.5}, "must be a whole number"),
    ({"min_turn_deg": 60}, "scan_turn_deg must be in [60, 90]"),
    ({"what_rungs": ["steer", "held", "scan"]}, "must keep ['located']"),
    ({"max_turn_deg": 4}, "max_turn_deg must be in [5, 90]"),
    ({"scan_turn_deg": 120}, "scan_turn_deg must be in [5, 90]"),
    ({"min_turn_deg": 30, "max_turn_deg": 10}, "must be <= max_turn_deg"),
    ({"local_yes_at": 0.05}, "local_yes_at must be in (0.1, 1]"),
    ({"steer_band_deg": True}, "must be a number"),
    ("fast", "must be a mapping"),
])
def test_criterion_3_a_bad_tree_is_refused_at_load(tmp_path, bad, words):
    import yaml
    from control.brain_config import load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": {"tier_tree": bad}}))
    with pytest.raises(ValueError) as e:
        load_brain_config(path)
    assert words in str(e.value)


def test_under_an_explore_search_the_band_and_bounds_stay(tmp_path):
    import yaml
    from control.brain_config import load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": {"tier_search": "explore",
                                              "tier_tree": {"min_turn_deg": 10}}}))
    with pytest.raises(ValueError, match="under tier_search: explore"):
        load_brain_config(path)
    path.write_text(yaml.safe_dump({"brain": {"tier_search": "explore",
                                              "tier_tree": {"what_rungs": ["steer", "located",
                                                                           "scan"]}}}))
    assert load_brain_config(path)["tier_tree"]["what_rungs"] == ["steer", "located", "scan"]


def test_whole_numbers_given_as_floats_load_as_ints_and_pass_explores_rule(tmp_path):
    import yaml
    from control.brain_config import load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": {"tier_search": "explore", "tier_tree": {
        "min_turn_deg": 5.0, "max_turn_deg": 90.0, "steer_band_deg": 3}}}))
    tree = load_brain_config(path)["tier_tree"]
    assert tree["min_turn_deg"] == 5 and isinstance(tree["min_turn_deg"], int)
    for moved in ({"max_turn_deg": 80}, {"steer_band_deg": 2.0}):
        path.write_text(yaml.safe_dump({"brain": {"tier_search": "explore", "tier_tree": moved}}))
        with pytest.raises(ValueError, match="under tier_search: explore"):
            load_brain_config(path)


def test_the_band_cap_is_the_arrivals_centring_band():
    from brain.arrival import ARRIVAL_CENTRE_DEG
    from control import brain_config
    assert brain_config.ARRIVAL_CENTRE_DEG == ARRIVAL_CENTRE_DEG


def test_a_policy_built_directly_refuses_a_bad_tree_by_name():
    with pytest.raises(ValueError, match="absent_events must name"):
        TieredVision(_Pipe(), lambda f: {}, tree={"absent_events": "band"})
    with pytest.raises(ValueError, match="what_rungs must name"):
        TieredVision(_Pipe(), lambda f: {}, tree={"what_rungs": ["steer", "steer", "scan"]})


def test_criterion_3_a_partial_tree_keeps_the_other_defaults(tmp_path):
    import yaml
    from control.brain_config import TIER_TREE_DEFAULTS, load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": {"tier_tree": {"scan_turn_deg": 30}}}))
    tree = load_brain_config(path)["tier_tree"]
    assert tree == {**TIER_TREE_DEFAULTS, "scan_turn_deg": 30}


def _absent_frame():
    return Perception(status=ABSENT)


def _detected_at(bearing):
    best = Candidate(detection=Detection(Box(0, 0, 10, 10), "x", 0.9), scores=(1.0, 0.0),
                     bearing_deg=bearing)
    return Perception(status=DETECTED, candidates=[best], best=best)


def test_criterion_3_the_rung_order_takes_effect():
    held = {"safest_direction": "LEFT"}
    t = TieredVision(_Pipe(), lambda f: {}, spin_guard_after=0)
    t._last_cloud_scene = held
    assert t._local_scene(_absent_frame())["safest_direction"] == "LEFT"   # held wins over scan
    t = TieredVision(_Pipe(), lambda f: {}, spin_guard_after=0,
                     tree={"what_rungs": ["steer", "located", "scan"]})
    t._last_cloud_scene = held
    assert t._local_scene(_absent_frame())["safest_direction"] == "RIGHT"  # held left out: scan
    assert "hold the last goal" not in t.describe_tree()


def test_criterion_3_the_numbers_take_effect():
    t = TieredVision(_Pipe(), lambda f: {}, tree={"steer_band_deg": 1.0, "scan_turn_deg": 30,
                                                   "min_turn_deg": 12, "max_turn_deg": 40})
    assert t._local_scene(_detected_at(2.0))["safest_direction"] == "RIGHT"     # outside 1
    s = t._local_scene(_detected_at(-70.0))
    assert (s["safest_direction"], s["turn_deg"]) == ("LEFT", 40)                # capped at 40
    assert t._local_scene(_detected_at(11.0))["turn_deg"] == 12                  # floor 12
    assert t._local_scene(_absent_frame())["turn_deg"] == 30                     # the scan step
    base = TieredVision(_Pipe(), lambda f: {})
    assert base._local_scene(_detected_at(2.0))["safest_direction"] == "FORWARD"  # 3-degree band


def test_criterion_3_the_absent_events_order_takes_effect():
    from brain import tiered
    def tier(order):
        t = TieredVision(_Pipe(), lambda f: {}, cold_search_after=1, stale_after=1,
                         tree={"absent_events": order})
        t._started, t._since_call, t._absent_streak = True, 5, 5
        return t
    assert tier(["cold_frames"])._trigger_for(_absent_frame()) == tiered.TRIGGER_COLD_SEARCH
    assert tier([])._trigger_for(_absent_frame()) == tiered.TRIGGER_STALE   # none: the floor


def test_criterion_3_local_yes_at_moves_the_frame_question_not_the_arrival():
    from brain.tiered import _local_kind
    got = {"status": "answered", "score": 0.6}
    t = TieredVision(_Pipe(), lambda f: {}, tree={"local_yes_at": 0.7})
    assert _local_kind(got, t.local_no_below, t.local_yes_at) == "unable_band"
    assert _local_kind(got, t.local_no_below) == "yes"          # the default, as the arrival's


# -- 3.95: on py_trees ---------------------------------------------------------

def _trees(tier):
    return {"what": tier._what_tree, "when": tier._when_tree, "who": tier._who_tree}


def test_criterion_1_py_trees_ticks_every_node_of_the_three_trees():
    from py_trees import behaviour, composites, decorators
    tier = TieredVision(_Pipe(), lambda f: {}, local_identity_fn=lambda f: {},
                        local_sighting_fn=lambda *a, **k: {}, steer_on_candidate=True)
    kinds = (composites.Sequence, composites.Selector, decorators.Decorator,
             behaviour.Behaviour)
    seen = 0
    for name, tree in _trees(tier).items():
        for node in tree.walk():
            assert isinstance(node, kinds), (name, node.name)
            if isinstance(node, (composites.Sequence, composites.Selector)):
                assert node.memory is False, (name, node.name)
            seen += 1
    assert seen > 40, seen


@pytest.mark.parametrize("tree", ["what", "when", "who"])
def test_criterion_3_an_error_in_any_tree_reaches_the_caller(tree):
    tier = TieredVision(_ScriptPipe(1), lambda f: {})

    class Planted(Exception):
        pass

    def boom(bb):
        raise Planted(tree)
    # The first action of each tree runs on every frame that reaches it.
    first = next(n for n in _trees(tier)[tree].walk() if hasattr(n, "act"))
    first.act = boom
    with pytest.raises(Planted, match=tree):
        for seq in range(50):
            tier({"seq": seq})


def test_a_decorator_sees_its_childs_status_after_the_child_ran():
    seen = []
    t = Decorator("d", Action("c", lambda bb: seen.append("child") or FAILURE),
                  lambda bb, child_status: seen.append(child_status) or SUCCESS)
    assert t.tick(None) == SUCCESS and seen == ["child", FAILURE]


def test_walk_is_parent_first_and_find_takes_the_first_in_that_order():
    t = Sequence("s", [Fallback("f", [Action("x", lambda bb: True)]), Action("x", lambda bb: True)])
    assert [n.name for n in t.walk()] == ["s", "f", "x", "x"]
    assert t.find("x") is t.children[0].children[0]
