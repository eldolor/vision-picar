"""
tests/test_target_probe.py

Handoff 2026-10-02, item 4h: `control/target_probe.py` read its "peak" from
`r.best`, which `PerceptionPipeline.perceive()` sets only on a DETECTED frame
(P >= 0.8). So a prompt that grounded at P 0.6 everywhere read "0.000 --
inert", and `--gate` below 0.8 could never count a hit. The peak and the
firing count are now taken over every candidate. A fake pipeline, no models.
"""

from brain.perceive import ABSENT, Box, Candidate, Detection, Perception
from control import target_probe


class _Pipeline:
    """Every frame yields one candidate at probability `p`, below 0.8, so
    the real gate never makes it `best`."""

    def __init__(self, p):
        # A two-way softmax: solve exp(a)/(exp(a)+1) = p for the target score.
        import math

        from brain.perceive import CLIP_LOGIT_SCALE
        self.scores = (math.log(p / (1 - p)) / CLIP_LOGIT_SCALE, 0.0)

    def perceive(self, frame):
        c = Candidate(detection=Detection(box=Box(0, 0, 1, 1), label="thing",
                                          confidence=0.5), scores=self.scores)
        return Perception(status=ABSENT, candidates=[c], best=None)


def _corpus(tmp_path, n=5):
    walk = tmp_path / "walk-1"
    walk.mkdir()
    for i in range(n):
        (walk / f"frame-{i:03d}.jpg").write_bytes(b"x")
    return str(tmp_path)


def _run(monkeypatch, tmp_path, p, gate):
    import brain.perceive
    import control.perception_eval

    monkeypatch.setattr(brain.perceive, "pipeline_for", lambda t: _Pipeline(p))
    monkeypatch.setattr(brain.perceive, "coco_class_for", lambda t: None)
    monkeypatch.setattr(control.perception_eval, "frame_dict", lambda path: {})
    return target_probe.probe(["a red toolbox"], recordings=_corpus(tmp_path),
                              sample=5, gate=gate)[0]


def test_the_peak_counts_candidates_below_the_detection_gate(monkeypatch, tmp_path):
    row = _run(monkeypatch, tmp_path, p=0.6, gate=0.8)
    assert abs(row["max_probability"] - 0.6) < 1e-6, row
    assert "inert" not in row["verdict"], row
    assert row["fires"] == 0


def test_a_gate_below_the_detection_gate_counts_hits(monkeypatch, tmp_path):
    row = _run(monkeypatch, tmp_path, p=0.6, gate=0.5)
    assert row["fires"] == 5, row
