"""3.109: compare `brain/goal_policy.py`'s numpy `features()` with the loop
it replaced (`tests/goal_features_reference.py`). Shared by
`test_goal_features_numpy.py` (fast tier) and `test_goal_features_sweep.py`."""
from brain import frontier as fr
from brain.goal_policy import features, goal_context
from tests import frontier_compare as fc
from tests import goal_features_reference as ref


def candidates(m: dict, x: float, y: float, seen: set) -> list:
    """Every frontier and view gap a choice at (x, y) would score, as
    (goal, kwargs) -- the calls explore's `_pool` makes."""
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    out = [(f.goal, {"kind": "frontier", "path_m": f.distance_m, "size_m": f.size_m})
           for f in fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)]
    out += [(g.goal, {"kind": "view", "path_m": g.distance_m, "area_m2": g.area_m2})
            for g in fr.find_view_gaps(m, x, y, seen, clearance_m=fc.LEAVE, reach_map=r)]
    return out


def differences(m: dict, pose: dict, goals: list, seen: set) -> list:
    """Every candidate whose new features differ from the reference's, with
    a shared context and without one."""
    bad, ctx = [], goal_context(m, seen)
    for k, (goal, kw) in enumerate(goals):
        want = ref.features(m, pose, goal, seen_cells=seen, failures=1, reached=1,
                            goals_sent=7, **kw)
        for got in (features(m, pose, goal, seen_cells=seen, failures=1, reached=1,
                             goals_sent=7, ctx=ctx, **kw),
                    features(m, pose, goal, seen_cells=seen, failures=1, reached=1,
                             goals_sent=7, **kw)):
            if got != want or [type(v) for v in got] != [type(v) for v in want]:
                bad.append((k, goal, want, got))
    return bad
