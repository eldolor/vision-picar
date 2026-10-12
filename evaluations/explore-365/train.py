"""
evaluations/explore-365/train.py -- 3.65 phase 2: train explore's goal
weights by the cross-entropy method on the TRAINING houses (usable 300-569),
choosing between iterations on the SELECTION houses (usable 100-199). The
test houses (0-99, 200-299) are never touched here; `evaluate.py` runs them
once at the end.

Each iteration: sample POP weight vectors from N(mean, std), run each on the
same BATCH training houses (a fresh sample per iteration), keep the ELITE
best by mean reward, refit mean and std to them. Every SELECT_EVERY
iterations the mean is run on the selection houses; the best of those is
the result.

    python evaluations/explore-365/train.py OUT.json
"""
import json
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, ".")
from harness import run_house                      # noqa: E402
from brain.goal_policy import FEATURES             # noqa: E402
from sim.maps.procthor import usable               # noqa: E402

SEED = 3652
POP, ELITE, ITERS, BATCH = 24, 6, 20, 40
SELECT_EVERY = 4
INIT_STD, MIN_STD = 2.0, 0.2
# The rule as a linear score, with views pushed late rather than last.
INIT_MEAN = [-10.0, 0.0, -10.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
WORKERS = max(1, (os.cpu_count() or 4) - 2)


def _job(args):
    return run_house(*args)


def mean_reward(ex, weights, houses):
    rows = list(ex.map(_job, [(h, weights) for h in houses]))
    return sum(r["reward"] for r in rows) / len(rows), rows


def main(out: str) -> None:
    rng = random.Random(SEED)
    u = usable()
    train, select = u[300:570], u[100:200]
    mean, std = list(INIT_MEAN), [INIT_STD] * len(FEATURES)
    log = {"features": FEATURES, "seed": SEED, "pop": POP, "elite": ELITE, "batch": BATCH,
           "iters": [], "selection": []}
    best = None
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=WORKERS) as ex:
        for it in range(ITERS):
            batch = rng.sample(train, BATCH)
            samples = [[rng.gauss(m, s) for m, s in zip(mean, std)] for _ in range(POP)]
            jobs = [(h, w) for w in samples for h in batch]
            rows = list(ex.map(_job, jobs))
            scores = [sum(r["reward"] for r in rows[i * BATCH:(i + 1) * BATCH]) / BATCH
                      for i in range(POP)]
            order = sorted(range(POP), key=lambda i: -scores[i])[:ELITE]
            elite = [samples[i] for i in order]
            mean = [sum(e[k] for e in elite) / ELITE for k in range(len(FEATURES))]
            std = [max(MIN_STD, (sum((e[k] - mean[k]) ** 2 for e in elite) / ELITE) ** 0.5)
                   for k in range(len(FEATURES))]
            log["iters"].append({"it": it, "best": round(scores[order[0]], 4),
                                 "elite_mean": round(sum(scores[i] for i in order) / ELITE, 4),
                                 "pop_mean": round(sum(scores) / POP, 4),
                                 "mean": [round(x, 3) for x in mean]})
            print(f"it {it:2d}  best {scores[order[0]]:.3f}  elite {log['iters'][-1]['elite_mean']:.3f}"
                  f"  pop {log['iters'][-1]['pop_mean']:.3f}  ({time.time() - t0:.0f} s)", flush=True)
            if (it + 1) % SELECT_EVERY == 0 or it == ITERS - 1:
                r, srows = mean_reward(ex, mean, select)
                found = sum(x["outcome"] == "found" for x in srows)
                entry = {"it": it, "reward": round(r, 4), "found": found,
                         "weights": [round(x, 3) for x in mean]}
                log["selection"].append(entry)
                print(f"   selection: reward {r:.3f}  found {found}/100", flush=True)
                if best is None or r > best["reward"]:
                    best = entry
            json.dump({**log, "best": best}, open(out, "w"), indent=1)
    print("best", best)


if __name__ == "__main__":
    main(sys.argv[1])
