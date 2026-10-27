"""
evaluations/goal-learn-3110/ladder.py -- 3.110 stage 1: the copying ladder.
F-large's 17 facts plus 3.101's 25 new ones added one at a time (ORDER),
each rung cloned from the oracle with F-large's network shape (S32x32) and
3.96's clone, three seeds, held-out agreement recorded. No PPO, no test.

    python evaluations/goal-learn-3110/ladder.py all <root> [jobs]   # every rung x seed, skipping done ones
    python evaluations/goal-learn-3110/ladder.py one <k> <seed> <root>
    python evaluations/goal-learn-3110/ladder.py report <root>

<root> holds 3.101's oracle sets (`oracle-large/`, the held-out selection
shards), as `stage_oracle` / `_select_oracle` write them.
"""
import json
import multiprocessing
import os
import statistics
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-3103"))

# 3.110's order (shuffled clone-agreement drop in 3.101's importance.json,
# largest first), fixed before any rung ran.
ORDER = ["lidar_gain", "since_new_ground", "pool_size", "time_used", "gain_rank",
         "path_rank", "trail_near", "room_seen", "path_gap", "unknown_4m", "cluster_2m",
         "trail_min", "unknown_2m", "cam_gain", "known_share", "room_hops", "wall_clear",
         "since_growth", "rule_pick", "detour", "room_degree", "nearest_other",
         "room_dead_end", "dominated", "gain_per_m"]
SEEDS = (3961, 3962, 3963)
SHAPE = "S32x32"                      # F-large's
B_CHOICES = {"n_train": 37064, "n_held": 609}


def cols(k: int) -> list:
    from rich import RICH_FEATURES
    assert sorted(ORDER) == sorted(RICH_FEATURES), "ORDER must be 3.101's 25 new facts"
    return list(range(17)) + [17 + RICH_FEATURES.index(f) for f in ORDER[:k]]


def arm(k: int, seed: int) -> str:
    return f"F-k{k:02d}-s{seed}"


def one(k: int, seed: int, root: str) -> None:
    import lean
    import run as B                   # 3.103.B's: 3.96's trainer, path set up
    T = B.T
    c = cols(k)
    lean.N_FEAT = len(c)              # read by make_torch_net's Net at construction
    T.SEED = seed                     # stage_clone's manual_seed and epoch shuffles
    T.torch_net = lean.make_torch_net(SHAPE)
    T.load_choices = lambda shards, kind: [(x[:, c], cc, y, p)
                                           for x, cc, y, p in B._load_choices(shards, kind)]
    import torch
    torch.set_num_threads(1)
    out = os.path.join(root, arm(k, seed))
    os.makedirs(out, exist_ok=True)
    with ProcessPoolExecutor(1) as ex:
        shards = T.stage_oracle("large", root, ex)        # all exist: nothing is run
        st = T.stage_clone(arm(k, seed), out, shards, ex)
    got = {key: st[key] for key in B_CHOICES}
    inputs = int(st["net"]["lin.weight"].shape[1])   # the patch took effect
    if inputs != len(c):
        raise SystemExit(f"{arm(k, seed)}: the net reads {inputs} facts, the rung {len(c)}")
    row = {"k": k, "facts": len(c), "seed": seed, "added": ORDER[k - 1] if k else None,
           "held_out_agreement": st["held_out_agreement"], "net_inputs": inputs, **got,
           "counts_as_B": got == B_CHOICES}
    with open(os.path.join(out, "row.json"), "w") as f:
        json.dump(row, f, indent=1)
    print(json.dumps(row), flush=True)


def run_all(root: str, jobs: int) -> None:
    todo = [(k, s) for s in SEEDS for k in range(len(ORDER) + 1)
            if not os.path.exists(os.path.join(root, arm(k, s), "row.json"))]
    env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}

    def go(ks):
        k, s = ks
        log = os.path.join(root, arm(k, s) + ".log")
        with open(log, "w") as f:
            r = subprocess.run([sys.executable, __file__, "one", str(k), str(s), root],
                               stdout=f, stderr=subprocess.STDOUT, env=env, cwd=REPO)
        print(f"{arm(k, s)}: exit {r.returncode}", flush=True)

    with ThreadPoolExecutor(jobs) as ex:
        list(ex.map(go, todo))


def report(root: str) -> dict:
    rows = [json.load(open(os.path.join(root, arm(k, s), "row.json")))
            for k in range(len(ORDER) + 1) for s in SEEDS
            if os.path.exists(os.path.join(root, arm(k, s), "row.json"))]
    by = {}
    for r in rows:
        by.setdefault(r["k"], []).append(r["held_out_agreement"] * 100)
    curve = [{"k": k, "facts": 17 + k, "added": ORDER[k - 1] if k else None,
              "mean": round(statistics.mean(v), 2), "min": round(min(v), 2),
              "max": round(max(v), 2), "seeds": len(v)} for k, v in sorted(by.items())]
    steps = [c["k"] for p, c in zip(curve, curve[1:])
             if c["seeds"] == len(SEEDS) and p["seeds"] == len(SEEDS)
             and c["mean"] - p["mean"] >= 3.0 and c["min"] > p["max"]]
    best = max(c["mean"] for c in curve) if curve else None
    knee = next((c["k"] for c in curve if c["mean"] >= best - 1.0), None) if curve else None
    out = {"curve": curve, "steps": steps, "knee_k": knee, "best_mean": best,
           "all_counts_as_B": all(r["counts_as_B"] for r in rows), "clones": len(rows)}
    with open(os.path.join(root, "ladder.json"), "w") as f:
        json.dump(out, f, indent=1)
    for c in curve:
        print(f"k={c['k']:2d} {c['facts']} facts  {c['mean']:5.2f}%  [{c['min']:.2f}, {c['max']:.2f}]  +{c['added']}")
    print("steps (criterion 1):", steps, " knee (criterion 2): k =", knee)
    return out


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd = sys.argv[1]
    if cmd == "one":
        one(int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
    elif cmd == "all":
        run_all(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 4)
    else:
        report(sys.argv[2])
