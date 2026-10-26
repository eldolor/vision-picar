"""
evaluations/goal-learn-3101/importance.py -- 3.101, reported: which features
F-rich's choices lean on. Each feature's column is shuffled across every
candidate of the held-out oracle choices (the selection houses, both modes;
seeded), and two things are measured against the unshuffled pick:

* `clone_agreement_drop`: the clone's agreement with the oracle, before
  minus after (the clone is what learned from the oracle's picks);
* `best_pick_changed`: the share of choices whose pick the final policy
  (best.json) changes.

    python evaluations/goal-learn-3101/importance.py OUT.json
"""
import json
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-396"))
sys.path.insert(0, HERE)
import model as M                                             # noqa: E402
from brain.goal_policy import FEATURES                        # noqa: E402
from features import ROOM_FEATURES                            # noqa: E402
from rich import RICH_FEATURES                                # noqa: E402

RUN = os.environ.get("RUN_3101", os.path.join(HERE, "run"))
NAMES = list(FEATURES) + list(ROOM_FEATURES) + list(RICH_FEATURES)
SEED = 3101


def picks(P, choices, Xs):
    return np.asarray([int(np.argmax(M.score_np(P, "F", X))) for X in Xs])


def main(out: str) -> None:
    import torch
    M.N_FEAT = len(NAMES)
    rows = pickle.load(open(os.path.join(RUN, "oracle-select", "oracle-select.pkl"), "rb"))
    ch = [c for r in rows for c in r["choices"] if (c["costs"] >= 0).any()]
    Xs = [c["X"] for c in ch]
    y = np.asarray([c["oracle"] for c in ch])
    net = M.torch_net("F")
    net.load_state_dict(torch.load(os.path.join(RUN, "F-rich", "clone.pt"), weights_only=False)["net"])
    clone = M.to_np(net.export())
    best = M.to_np(json.load(open(os.path.join(RUN, "F-rich", "best.json")))["params"])
    base_c = picks(clone, ch, Xs)
    base_b = picks(best, ch, Xs)
    agree0 = float((base_c == y).mean())
    lens = np.cumsum([0] + [len(x) for x in Xs])
    allX = np.concatenate(Xs)
    rng = np.random.default_rng(SEED)
    res = []
    for j, name in enumerate(NAMES):
        Z = allX.copy()
        Z[:, j] = rng.permutation(Z[:, j])
        Zs = [Z[lens[k]:lens[k + 1]] for k in range(len(Xs))]
        pc, pb = picks(clone, ch, Zs), picks(best, ch, Zs)
        res.append({"feature": name, "new": j >= len(FEATURES) + len(ROOM_FEATURES),
                    "clone_agreement_drop": round(agree0 - float((pc == y).mean()), 4),
                    "best_pick_changed": round(float((pb != base_b).mean()), 4)})
    res.sort(key=lambda r: -r["best_pick_changed"])
    report = {"choices": len(ch), "clone_agreement": round(agree0, 4), "features": res,
              "linear_weights": dict(zip(NAMES, np.round(best["w"], 3).tolist()))}
    json.dump(report, open(out, "w"), indent=1)
    for r in res:
        print(f"{r['feature']:17s} {'new' if r['new'] else '   '}  pick changed {r['best_pick_changed']:.3f}"
              f"  clone agreement -{r['clone_agreement_drop']:.3f}")


if __name__ == "__main__":
    main(sys.argv[1])
