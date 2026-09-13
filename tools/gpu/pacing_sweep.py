"""
tools/gpu/pacing_sweep.py

Phase D: choose the cold-search interval by measuring it, instead of
inheriting a default nobody picked for this job.

## Why this exists

The live trigger record says 90% of deliberation calls are fired by a
clock, not by perception: across six tiered walks and 70 calls,
`cold_search` 79%, `mission_start` 9%, `staleness` 3%, and
`candidate_sighting` -- the only trigger the local tier owns -- **10%, on
two walks of six**. So `cold_search_after` is not a backstop, it is the
search mechanism, and it has never been measured.

## The metric, and why it is not call count

Cost is explicitly not the constraint here. What matters is **how long the
target sits in view before anything looks at it**. So for every contiguous
span of visible frames in a walk's adjudicated `labels.json`, this measures
the frames from the span's start to the first cloud call at or after it --
`look_latency`. A span that ends with no call at all is a **miss**, and
misses are reported separately rather than averaged in, because an infinite
latency has no mean.

Call count is still reported. It is a cost readout, not the objective.

## The cloud is stubbed, and takes time measured in FRAMES

No paid calls, and nothing here depends on what a VLM would have said --
the trigger policy does not read the answer. `--cloud-frames` is how many
frames a call is outstanding, which is what makes Phase A measurable at
all: under `--async` a second trigger arriving while one is in flight is
coalesced away, and that changes both the call count and the latency. A
wall clock would make the result unreproducible; a frame count does not.

## The honest limit, and it is a real one

**Replayed walks have no odometry.** A phone on a wheeled rig has no
encoders, so `TeleopRobot` reports `usable: False` and the distance rule
falls back to frames -- on exactly the corpus that validates perception.
To measure the distance rule at all, `--nominal-cm` integrates the walk's
own recorded action stream at a nominal travel per FORWARD.

**That is derived, not measured**, and every distance row is labelled
`odometry: derived` for that reason. It answers "how often would the rule
have fired, given the moves the policy actually made" and it does not
answer "how far did the robot go". The real number needs encoders, which
means hardware.

    python pacing_sweep.py --out out --device cuda
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

from brain.perceive import ABSENT
from brain.tiered import TieredVision
from control.perception_eval import frame_dict, load_corpus

# Nominal travel per FORWARD, centimetres. Stage 0's rig instruction is to
# move "about 30cm" between captures, so this is the walk's own protocol
# rather than a guess -- but it is still a protocol, not a measurement.
DEFAULT_NOMINAL_CM = 30.0


class StubCloud:
    """Counts calls, returns a fixed scene, and stays 'in flight' for a
    fixed number of FRAMES rather than a wall-clock duration."""

    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear",
                "doorway_visible": False, "important_objects": [],
                "safest_direction": "FORWARD",
                "_navigate": {"reasoning": "stub", "target_visible": False,
                              "target_reached": False}}


class FrameClockExecutor:
    """A stand-in for the policy's ThreadPoolExecutor whose futures
    complete after N *frames* rather than after real work.

    This is what makes Phase A reproducible: the question "was a call
    still outstanding when the next trigger arrived" has one answer per
    (walk, config) instead of one per machine load.
    """

    class _Future:
        def __init__(self, fn, args, due):
            self._fn, self._args, self.due = fn, args, due
            self._value = None
            self._exc = None
            self._ran = False

        def done(self):
            return self.due <= 0

        def result(self, timeout=None):
            if not self._ran:
                self._ran = True
                try:
                    self._value = self._fn(*self._args)
                except BaseException as e:  # noqa: BLE001
                    self._exc = e
            if self._exc:
                raise self._exc
            return self._value

    def __init__(self, frames):
        self.frames = frames
        self.pending = []

    def submit(self, fn, *args):
        f = self._Future(fn, args, self.frames)
        self.pending.append(f)
        return f

    def advance(self):
        for f in self.pending:
            f.due -= 1
        self.pending = [f for f in self.pending if f.due > 0]

    def shutdown(self, wait=False, cancel_futures=False):
        self.pending = []


def visible_spans(walk) -> list:
    """Contiguous runs of visible frames, as (start_index, end_index)."""
    spans, start = [], None
    for i, (_path, visible, _adj) in enumerate(walk.frames):
        if visible and start is None:
            start = i
        elif not visible and start is not None:
            spans.append((start, i - 1))
            start = None
    if start is not None:
        spans.append((start, len(walk.frames) - 1))
    return spans


def actions_for(walk) -> list:
    """The walk's own recorded action per frame, for the nominal odometry.

    Read from `walk.jsonl` -- which `control/perception_eval.py` refuses to
    use as a perception reference, correctly, because it records what a
    model SAID. Here we want exactly that: what the policy did. It is
    evidence about the actions, which is what it is.
    """
    path = Path(walk.path) / "walk.jsonl"
    out = {}
    if not path.exists():
        return out
    for line in path.open():
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        nav = rec.get("navigate") or {}
        if rec.get("file"):
            out[rec["file"]] = nav.get("action")
    return out


class CachedPipeline:
    """Plays back a walk's already-computed perceptions, one per frame.

    **This is what makes the sweep affordable, and it is exact rather than
    an approximation.** The trigger policy reads `perception.status` and
    nothing else -- it does not read the cloud's answer, and with a stub
    cloud there is no corroboration to compute. So perception for frame N
    is identical under every pacing config, and running YOLO + SegFormer +
    CLIP fifteen times over the same 610 frames would produce fifteen
    identical answers at fifteen times the cost.

    Perception runs once per walk; the sweep then costs milliseconds.
    """

    def __init__(self, perceptions):
        self._perceptions = list(perceptions)
        self._i = 0
        # The readouts name the models, and a cache that could not would
        # make a swept row describe itself differently from the real one.
        self.detector = None
        self.scorer = None
        self.proposer = None

    def perceive(self, frame):
        p = self._perceptions[min(self._i, len(self._perceptions) - 1)]
        self._i += 1
        return p

    def rewind(self):
        self._i = 0


def perceive_walk(pipeline, walk, on_frame=None) -> list:
    """Every frame's Perception, computed once."""
    out = []
    for path, _visible, _adj in walk.frames:
        out.append(pipeline.perceive(frame_dict(path)))
        if on_frame:
            on_frame()
    return out


def run_walk(pipeline, walk, *, cold_after, cold_cm, async_cloud,
             cloud_frames, nominal_cm):
    cloud = StubCloud()
    tier = TieredVision(pipeline, cloud,
                        cold_search_after=cold_after,
                        cold_search_after_cm=cold_cm,
                        async_cloud=async_cloud)
    executor = FrameClockExecutor(cloud_frames)
    if async_cloud:
        tier._executor = executor

    actions = actions_for(walk) if cold_cm else {}
    travelled_m = 0.0
    call_at = []

    if hasattr(pipeline, "rewind"):
        pipeline.rewind()
    for i, (path, _visible, _adj) in enumerate(walk.frames):
        # The cached pipeline ignores the frame's pixels, so the dict only
        # has to carry the odometry the policy reads.
        frame = ({"image_base64": "cached", "image_width": 640}
                 if hasattr(pipeline, "rewind") else frame_dict(path))
        if cold_cm:
            frame["odometry"] = {"usable": True,
                                 "distance_m": round(travelled_m, 4),
                                 "heading_deg": 0.0}
        before = cloud.calls
        outstanding_before = tier._inflight is not None
        try:
            tier(frame)
        except Exception:                       # a stubbed cloud cannot fail
            pass
        if async_cloud:
            executor.advance()
        # A dispatched call counts as "looked at frame i" even though its
        # answer lands later -- the question this measures is when the
        # cloud was ASKED about a view, not when it replied.
        if cloud.calls > before or (tier._inflight is not None
                                    and not outstanding_before):
            call_at.append(i)
        if cold_cm and (actions.get(path.name) or "").upper() == "FORWARD":
            travelled_m += nominal_cm / 100.0

    tier.close()
    return cloud.calls, call_at, tier.stats


def score(walk, call_at) -> dict:
    """Look-latency per visible span, and the spans nothing looked at."""
    lat, missed = [], 0
    for start, end in visible_spans(walk):
        after = [c for c in call_at if start <= c <= end]
        if after:
            lat.append(after[0] - start)
        else:
            missed += 1
    return {"spans": len(visible_spans(walk)), "looked": len(lat),
            "missed": missed, "latencies": lat}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--out", default="out")
    ap.add_argument("--device", default=None)
    ap.add_argument("--frames", default="2,3,4,6,9,12",
                    help="cold_search_after values to sweep")
    ap.add_argument("--cms", default="20,40,80",
                    help="cold_search_after_cm values (DERIVED odometry)")
    ap.add_argument("--cloud-frames", type=int, default=4,
                    help="how many frames a cloud call stays outstanding")
    ap.add_argument("--nominal-cm", type=float, default=DEFAULT_NOMINAL_CM)
    args = ap.parse_args(argv)

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    walks = load_corpus(Path(args.recordings))
    print(f"corpus: {len(walks)} walks, "
          f"{sum(len(w.frames) for w in walks)} frames, "
          f"{sum(w.visible for w in walks)} visible", flush=True)

    from brain.perceive_lab import ClipScorer, SegformerFloorProposer, YoloDetector
    from brain.perceive import PerceptionPipeline
    detector = YoloDetector("yolo11s.pt", device=args.device)
    scorer = ClipScorer("RN50", device=args.device)
    proposer = SegformerFloorProposer(device=args.device)

    configs = []
    for n in [int(x) for x in args.frames.split(",")]:
        for is_async in (False, True):
            configs.append({"name": f"frames-{n}" + ("-async" if is_async else ""),
                            "cold_after": n, "cold_cm": None,
                            "async_cloud": is_async, "odometry": "n/a"})
    for cm in [float(x) for x in args.cms.split(",")]:
        configs.append({"name": f"cm-{int(cm)}", "cold_after": 999,
                        "cold_cm": cm, "async_cloud": False,
                        "odometry": "DERIVED from the walk's own actions"})

    # Stage one: perception, once per walk.
    print("\nperceiving each walk once -- the sweep reuses these", flush=True)
    cache = {}
    for walk in walks:
        pipeline = PerceptionPipeline(detector, scorer, walk.target,
                                      proposer=proposer)
        seen = perceive_walk(pipeline, walk)
        cache[walk.name] = seen
        detected = sum(1 for p in seen if p.status != ABSENT)
        print(f"  {walk.name:<42} {len(seen):>4} frames, "
              f"{detected:>3} not-absent", flush=True)

    # Stage two: sweep the pacing, in milliseconds.
    rows = []
    for cfg in configs:
        print(f"\n=== {cfg['name']} " + "=" * (52 - len(cfg["name"])), flush=True)
        calls = spans = looked = missed = 0
        lat = []
        for walk in walks:
            pipeline = CachedPipeline(cache[walk.name])
            c, call_at, _stats = run_walk(
                pipeline, walk, cold_after=cfg["cold_after"],
                cold_cm=cfg["cold_cm"], async_cloud=cfg["async_cloud"],
                cloud_frames=args.cloud_frames, nominal_cm=args.nominal_cm)
            s = score(walk, call_at)
            calls += c; spans += s["spans"]; looked += s["looked"]
            missed += s["missed"]; lat += s["latencies"]
            print(f"  {walk.name:<42} calls {c:>3}  spans {s['spans']:>2}  "
                  f"looked {s['looked']:>2}  missed {s['missed']:>2}", flush=True)
        row = {**cfg,
               "cloud_calls": calls, "frames": sum(len(w.frames) for w in walks),
               "spans": spans, "looked": looked, "missed": missed,
               "median_latency": (statistics.median(lat) if lat else None),
               "mean_latency": (round(statistics.mean(lat), 2) if lat else None),
               "max_latency": (max(lat) if lat else None)}
        rows.append(row)
        print(f"  -> calls {calls}  looked {looked}/{spans}  "
              f"median look-latency {row['median_latency']}  "
              f"max {row['max_latency']}", flush=True)

    (out / "pacing.json").write_text(json.dumps(rows, indent=2))
    print("\n" + json.dumps(rows, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
