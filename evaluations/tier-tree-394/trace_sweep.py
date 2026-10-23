"""
evaluations/tier-tree-394/trace_sweep.py

3.94 criterion 2b: 3.80.O's in-process sweep (`evaluations/tiered-380/O/
sweep_inprocess.py`), with every frame's decision recorded -- the action, the
turn's size, the cloud trigger, the local question, the call counts -- so two
checkouts can be compared frame for frame. `--root` picks the code (the
sweep reads it at import); the harness and its stand-ins are this tree's.

    .venv/bin/python evaluations/tier-tree-394/trace_sweep.py --out <file.json> \\
        [--root <checkout>] [sweep_inprocess's own flags: --sync --decoy --cloud --cold-every --houses]
"""

import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
args = sys.argv[1:]
out = Path(args[args.index("--out") + 1])
del args[args.index("--out"):args.index("--out") + 2]
sys.argv = ["sweep_inprocess.py", "--name", "trace", *args]

spec = importlib.util.spec_from_file_location(
    "sweep_inprocess", HERE.parent / "tiered-380/O/sweep_inprocess.py")
sweep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep)          # binds brain/ from --root

trace = []
_call = sweep.TieredVision.__call__

# The worker thread made async missions differ from run to run on either
# checkout (the same house, the same code: 50 frames apart), so a frame-for-
# frame comparison needs a deterministic worker. This one runs each job
# LAND_AFTER frames after it was submitted -- the call is still in flight on
# the frames between, as on the car -- or at once when someone waits on it.
LAND_AFTER = 2
_pending = []


from concurrent.futures import Future  # noqa: E402


class _Job(Future):
    def __init__(self, fn, args, kwargs):
        super().__init__()
        self._job, self.age = (fn, args, kwargs), 0

    def run_now(self):
        if self._job is None:
            return
        fn, args, kwargs = self._job
        self._job = None
        try:
            self.set_result(fn(*args, **kwargs))
        except BaseException as e:  # noqa: BLE001 -- carried in the future, as the pool does
            self.set_exception(e)

    def result(self, timeout=None):
        self.run_now()
        return super().result(timeout)

    def exception(self, timeout=None):
        self.run_now()
        return super().exception(timeout)


class DeferredExecutor:
    def __init__(self, *a, **k):
        pass

    def submit(self, fn, *args, **kwargs):
        job = _Job(fn, args, kwargs)
        _pending.append(job)
        return job

    def shutdown(self, wait=True, cancel_futures=False):
        if cancel_futures:
            for job in _pending:
                if job._job is not None:
                    job._job = None
                    job.cancel()


sys.modules["brain.tiered"].ThreadPoolExecutor = DeferredExecutor


def _age_jobs():
    for job in list(_pending):
        job.age += 1
        if job.age >= LAND_AFTER:
            job.run_now()
            _pending.remove(job)


def _land_all():
    for job in list(_pending):
        job.run_now()
        _pending.remove(job)


# The arrival waits on the in-flight question's `local_done` Event, which only
# its job sets; with no worker thread that wait never ends. Waiting on it
# lands the pending jobs first, as waiting on a future does.
def _landing_first(name):
    original = getattr(sweep.TieredVision, name)

    def wrapped(self, *a, **k):
        q = self._inflight_question
        if q is not None and not q.local_done.is_set():
            _land_all()
        return original(self, *a, **k)
    setattr(sweep.TieredVision, name, wrapped)


for _name in ("confirm_local", "wait_local_free"):
    _landing_first(_name)


def traced(self, frame):
    _age_jobs()
    scene = _call(self, frame)
    tier = scene.get("_tier") or {}
    trace.append([scene.get("safest_direction"), scene.get("turn_deg"), tier.get("trigger"),
                  tier.get("local_called"), self.stats.cloud_calls, self.stats.local_frame_calls,
                  (scene.get("_navigate") or {}).get("reasoning")])
    return scene


sweep.TieredVision.__call__ = traced


def main():
    import argparse
    ap = argparse.ArgumentParser()
    for flag in ("--name", "--houses", "--cloud", "--root"):
        ap.add_argument(flag, default=None)
    ap.add_argument("--sync", action="store_true")
    ap.add_argument("--sighted", action="store_true")
    ap.add_argument("--jump", action="store_true")
    ap.add_argument("--decoy", type=float, default=0.0)
    ap.add_argument("--cold-every", type=int, default=6)
    a = ap.parse_args(sys.argv[1:])
    a.cloud = a.cloud or "truth"
    houses = (sweep.DEV_HOUSES if (a.houses or "dev") == "dev" else sweep.SWEEP_HOUSES
              if a.houses == "criterion" else a.houses.split(","))
    sweep.logging.disable(sweep.logging.WARNING)
    result = {"args": vars(a), "root": str(sweep.ROOT), "missions": []}
    for h in houses:
        del trace[:]
        del _pending[:]
        row = sweep.run(h, a)
        result["missions"].append({"house": h, "row": {k: v for k, v in row.items() if k != "tail"},
                                   "frames": list(trace)})
        print(h, row["outcome"], len(trace), flush=True)
    out.write_text(json.dumps(result) + "\n")


if __name__ == "__main__":
    main()
