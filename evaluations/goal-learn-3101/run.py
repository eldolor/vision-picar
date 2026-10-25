"""
evaluations/goal-learn-3101/run.py -- 3.101: train F-rich, exactly as 3.96
trained F-large (its `train.py`), with RICH_FEATURES appended to X.

    python evaluations/goal-learn-3101/run.py train OUT_ROOT   # oracle sets, clone, PPO -> OUT_ROOT/F-rich/
    python evaluations/goal-learn-3101/run.py baseline OUT     # F-large's best on the selection houses today

Resumable, as 3.96's stages are. WORKERS sets the pool (default: every core).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-396"))
os.environ.setdefault("WORKERS", str(os.cpu_count() or 4))
import model as M                                             # noqa: E402
import train as T                                             # noqa: E402
from rich import RICH_FEATURES, rich_features                 # noqa: E402

F_LARGE = os.path.join(HERE, "..", "goal-learn-396", "cloud", "ckpt", "F-large", "best.json")


def use_rich(on: bool) -> None:
    """Set in the parent before the pool forks, so every worker inherits it."""
    T.EXTRA_FEATURES = rich_features if on else None
    M.N_FEAT = 17 + (len(RICH_FEATURES) if on else 0)


def cmd_train(root: str) -> None:
    use_rich(True)
    T.main("F-rich", os.path.join(root, "F-rich"))


def cmd_baseline(out: str) -> None:
    from concurrent.futures import ProcessPoolExecutor
    use_rich(False)
    b = json.load(open(F_LARGE))
    with ProcessPoolExecutor(T.WORKERS) as ex:
        sel = T.run_selection(b["kind"], b["params"], ex)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    T.save_atomic(out, lambda p: json.dump({"recorded_2026_10_23": b["best_selection"],
                                            "today": sel}, open(p, "w"), indent=1))
    print(json.dumps({"recorded": b["best_selection"], "today": sel}))


if __name__ == "__main__":
    {"train": cmd_train, "baseline": cmd_baseline}[sys.argv[1]](sys.argv[2])
