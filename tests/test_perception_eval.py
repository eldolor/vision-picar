"""
tests/test_perception_eval.py

Phase P3 (`PLAN-onboard-perception.md` 4.10) -- corpus-wide scoring of the
on-board perception tier.

**Every test here runs against fakes and temp directories**, the same rule
`tests/test_perceive.py` follows: the models are a multi-gigabyte install
and the automated suite may never need them. What is tested is the part
that has actually been got wrong before -- the arithmetic. Three of the
findings in 4.10 had to be withdrawn or restated because a number was
computed against the wrong reference, at a coverage nobody checked, or
without its false-positive count beside it, so the scorer's own
correctness is not a formality here.

Run with: pytest tests/test_perception_eval.py -v
"""

import json

import pytest

from control.perception_eval import (
    DEFAULT_GATES,
    METRIC_CONFIDENCE,
    METRIC_MARGIN,
    METRIC_PROBABILITY,
    CorpusError,
    FrameScore,
    best_score,
    by_walk,
    format_comparison,
    format_gate_table,
    format_per_walk,
    load_corpus,
    load_records,
    load_walk,
    decisions_by_frame,
    recall_at_fp_budget,
    run_walk,
    save_records,
    score_at,
    sweep,
    totals,
)
from brain.perceive import (ABSENT, DETECTED, UNAVAILABLE, Box, Candidate,
                            Detection, Perception)


def record(walk="w", frame="frame-0000.jpg", visible=False, score=None,
           status=None, **extra):
    if status is None:
        status = ABSENT if score is None else DETECTED
    return FrameScore(walk=walk, frame=frame, visible=visible, status=status,
                      score=score, **extra)


# ---------------------------------------------------------------------------
# score_at -- the arithmetic everything else rests on
# ---------------------------------------------------------------------------

def test_score_at_counts_the_four_cells():
    records = [
        record(visible=True, score=0.9),    # TP
        record(visible=True, score=0.1),    # FN
        record(visible=False, score=0.9),   # FP
        record(visible=False, score=0.1),   # TN
    ]
    s = score_at(records, 0.5)
    assert (s["tp"], s["fn"], s["fp"], s["tn"]) == (1, 1, 1, 1)
    assert s["recall"] == 0.5
    assert s["precision"] == 0.5


def test_a_frame_with_no_score_is_never_a_detection():
    """No crop survived the gate, so there is nothing to threshold. It is a
    miss, not a pass, and not an error."""
    s = score_at([record(visible=True, score=None)], 0.0)
    assert s["tp"] == 0 and s["fn"] == 1


def test_unavailable_is_counted_apart_and_never_scored():
    """1.12's whole point: 'I could not tell' is not 'it is not there'.
    Folding it into the misses would blame the matcher for a dead camera."""
    records = [record(visible=True, score=None, status=UNAVAILABLE),
               record(visible=True, score=0.9)]
    s = score_at(records, 0.5)
    assert s["unavailable"] == 1
    assert s["tp"] == 1 and s["fn"] == 1


def test_the_gate_is_inclusive_at_its_own_value():
    assert score_at([record(visible=True, score=0.8)], 0.8)["tp"] == 1
    assert score_at([record(visible=True, score=0.7999)], 0.8)["tp"] == 0


def test_precision_is_none_rather_than_zero_when_nothing_was_admitted():
    """0% precision says 'everything it found was wrong'. Nothing was
    found, which is a different claim."""
    s = score_at([record(visible=True, score=0.1)], 0.9)
    assert s["precision"] is None
    assert s["recall"] == 0.0


def test_recall_is_none_when_no_frame_is_visible():
    s = score_at([record(visible=False, score=0.9)], 0.5)
    assert s["recall"] is None
    assert s["precision"] == 0.0


def test_sweep_is_one_row_per_gate_in_order():
    rows = sweep([record(visible=True, score=0.75)], (0.9, 0.8, 0.7))
    assert [r["gate"] for r in rows] == [0.9, 0.8, 0.7]
    assert [r["tp"] for r in rows] == [0, 0, 1]


# ---------------------------------------------------------------------------
# recall_at_fp_budget -- the primitive that makes two configs comparable
# ---------------------------------------------------------------------------

def test_recall_at_fp_budget_admits_exactly_the_budget():
    """4.11's rule: a recall without its false-positive count is not a
    result. Matching on FP first is what makes two models comparable."""
    records = [record(visible=True, score=s) for s in (0.9, 0.7, 0.5, 0.3)]
    records += [record(visible=False, score=s) for s in (0.8, 0.6, 0.4)]
    s = recall_at_fp_budget(records, 1)
    assert s["fp"] == 1
    # Gate lands just above the second-highest negative (0.6), so 0.9 and
    # 0.7 are the true positives kept.
    assert s["tp"] == 2


def test_a_zero_budget_excludes_every_false_positive():
    records = [record(visible=True, score=0.95), record(visible=True, score=0.5),
               record(visible=False, score=0.9)]
    s = recall_at_fp_budget(records, 0)
    assert s["fp"] == 0 and s["tp"] == 1


def test_a_budget_larger_than_the_negatives_admits_everything():
    records = [record(visible=True, score=0.1), record(visible=False, score=0.2)]
    s = recall_at_fp_budget(records, 99)
    assert s["tp"] == 1 and s["fp"] == 1
    assert s["gate"] == float("-inf")


def test_the_budget_is_a_ceiling_not_a_target():
    """Ties on the boundary must not smuggle an extra false positive in:
    two negatives at the same score are admitted together or not at all."""
    records = [record(visible=False, score=0.5), record(visible=False, score=0.5),
               record(visible=True, score=0.5)]
    s = recall_at_fp_budget(records, 1)
    assert s["fp"] == 0
    assert s["tp"] == 0


# ---------------------------------------------------------------------------
# best_score -- gate-free, which is what makes one pass serve every gate
# ---------------------------------------------------------------------------

def _candidate(similarity, distractor=0.0, confidence=0.5, label="thing"):
    return Candidate(
        detection=Detection(box=Box(0, 0, 10, 10), label=label,
                            confidence=confidence),
        similarity=similarity, best_distractor=distractor,
        scores=(similarity, distractor))


def test_best_score_reads_every_candidate_not_just_the_pipelines_pick():
    """`Perception.best` is the argmax under whichever metric the pipeline
    was built with. A sweep must not inherit that choice, or every table
    would silently be a table about the shipped gate."""
    p = Perception(status=ABSENT, candidates=[_candidate(0.2), _candidate(0.4)],
                   best=None)
    assert best_score(p, METRIC_PROBABILITY) == pytest.approx(
        max(c.probability for c in p.candidates))


def test_best_score_is_none_when_the_frame_could_not_be_read():
    p = Perception(status=UNAVAILABLE, reason="decode failed")
    assert best_score(p) is None


def test_best_score_is_none_when_nothing_was_proposed():
    assert best_score(Perception(status=ABSENT)) is None


def test_best_score_can_read_the_margin_and_the_raw_confidence():
    """Every measurement before 2026-09-07 used the margin, and an
    open-vocabulary detector produces only a confidence -- reproducing an
    old number is how you tell a real change from a changed metric."""
    p = Perception(status=DETECTED,
                   candidates=[_candidate(0.3, 0.1, confidence=0.42)])
    assert best_score(p, METRIC_MARGIN) == pytest.approx(0.2)
    assert best_score(p, METRIC_CONFIDENCE) == pytest.approx(0.42)


# ---------------------------------------------------------------------------
# Loading a corpus. The reference rule is enforced here or nowhere.
# ---------------------------------------------------------------------------

def _walk_dir(tmp_path, name="a-walk", labels=None, description="blue bottle",
              adjudicated=(), write_labels=True, frames=2):
    d = tmp_path / name
    d.mkdir()
    names = [f"frame-{i:04d}.jpg" for i in range(frames)]
    for n in names:
        (d / n).write_bytes(b"jpeg-ish")
    if write_labels:
        doc = {"walk": name, "description": description,
               "target_visible_labels": (labels if labels is not None
                                         else {n: False for n in names}),
               "adjudicated": list(adjudicated), "method": "by eye"}
        (d / "labels.json").write_text(json.dumps(doc))
    return d


def test_a_walk_without_labels_cannot_be_scored(tmp_path):
    """The reference rule, enforced rather than documented: `walk.jsonl` is
    what a model said at the time, and on the search walk 34 of its 44
    claims were a storage bin. A corpus-wide score against that is worse
    than no score, because it looks like one."""
    d = _walk_dir(tmp_path, write_labels=False)
    with pytest.raises(CorpusError, match="no labels.json"):
        load_walk(d)


def test_labels_without_a_description_are_refused(tmp_path):
    """The target string moved recall 7x on the shoes walk, so it may not
    be guessed from a directory name."""
    d = _walk_dir(tmp_path, description="")
    with pytest.raises(CorpusError, match="description"):
        load_walk(d)


def test_a_walk_reads_its_target_and_its_visible_count(tmp_path):
    d = _walk_dir(tmp_path, labels={"frame-0000.jpg": True,
                                    "frame-0001.jpg": False},
                  description="burgundy backpack",
                  adjudicated=["frame-0000.jpg"])
    walk = load_walk(d)
    assert walk.target == "burgundy backpack"
    assert walk.visible == 1
    assert [p.name for p, _, _ in walk.frames] == ["frame-0000.jpg",
                                                   "frame-0001.jpg"]
    assert [adj for _, _, adj in walk.frames] == [True, False]


def test_an_image_with_no_label_is_left_out_rather_than_assumed(tmp_path):
    """An unlabelled frame is not a negative. Counting it as one is how a
    precision figure gets quietly inflated by frames nobody looked at."""
    d = _walk_dir(tmp_path, labels={"frame-0000.jpg": True}, frames=3)
    assert len(load_walk(d).frames) == 1


def test_load_corpus_skips_directories_with_no_labels(tmp_path):
    _walk_dir(tmp_path, "labelled")
    _walk_dir(tmp_path, "unlabelled", write_labels=False)
    assert [w.name for w in load_corpus(tmp_path)] == ["labelled"]


def test_load_corpus_names_a_walk_that_is_not_there(tmp_path):
    _walk_dir(tmp_path, "labelled")
    with pytest.raises(CorpusError, match="missing-walk"):
        load_corpus(tmp_path, ["missing-walk"])


def test_an_empty_corpus_says_where_the_walks_live(tmp_path):
    with pytest.raises(CorpusError, match="four rig walks"):
        load_corpus(tmp_path)


# ---------------------------------------------------------------------------
# run_walk, over a fake pipeline
# ---------------------------------------------------------------------------

class FakePipeline:
    """Answers by frame name, so a test can pin exactly which frame got
    which score without a model in the room."""

    def __init__(self, answers):
        self.answers = answers
        self.seen = []

    def perceive(self, frame):
        self.seen.append(frame)
        return self.answers[len(self.seen) - 1]


def test_run_walk_records_one_row_per_labelled_frame(tmp_path):
    d = _walk_dir(tmp_path, labels={"frame-0000.jpg": True,
                                    "frame-0001.jpg": False})
    walk = load_walk(d)
    pipeline = FakePipeline([
        Perception(status=DETECTED, candidates=[_candidate(0.9, 0.1)],
                   best=_candidate(0.9, 0.1)),
        Perception(status=UNAVAILABLE, reason="wedged"),
    ])
    records = run_walk(pipeline, walk)
    assert [r.frame for r in records] == ["frame-0000.jpg", "frame-0001.jpg"]
    assert [r.visible for r in records] == [True, False]
    assert records[0].score is not None and records[1].score is None
    assert records[1].status == UNAVAILABLE


def test_run_walk_passes_a_decodable_frame_with_its_width(tmp_path):
    """The bearing is computed against the width, and a bearing scaled by a
    guessed width is wrong by exactly the ratio nobody checked."""
    d = _walk_dir(tmp_path, frames=1)
    walk = load_walk(d)
    pipeline = FakePipeline([Perception(status=ABSENT)])
    run_walk(pipeline, walk)
    sent = pipeline.seen[0]
    assert sent["media_type"] == "image/jpeg"
    assert "image_base64" in sent


# ---------------------------------------------------------------------------
# Grouping, totals and round-tripping
# ---------------------------------------------------------------------------

def test_by_walk_keeps_every_walk_separate():
    records = [record(walk="a"), record(walk="b"), record(walk="a")]
    grouped = by_walk(records)
    assert sorted(grouped) == ["a", "b"] and len(grouped["a"]) == 2


def test_totals_counts_the_tri_state_and_the_adjudicated_frames():
    records = [record(visible=True, score=0.9, status=DETECTED, adjudicated=True),
               record(status=UNAVAILABLE),
               record(status=ABSENT)]
    t = totals(records)
    assert (t["detected"], t["absent"], t["unavailable"]) == (1, 1, 1)
    assert t["adjudicated"] == 1 and t["visible"] == 1


def test_records_round_trip_so_a_slow_model_is_paid_for_once(tmp_path):
    """SAM is seconds per frame. Re-running a corpus to move a threshold
    would be minutes of compute measuring nothing new."""
    records = [record(visible=True, score=0.9), record(visible=False)]
    path = tmp_path / "run.json"
    save_records(path, records, {"detector": "yolo11s.pt", "proposer": "sam"})
    back, config = load_records(path)
    assert back == records
    assert config["proposer"] == "sam"


# ---------------------------------------------------------------------------
# The tables. Formatting is where a wrong number becomes a wrong belief.
# ---------------------------------------------------------------------------

def test_the_gate_table_prints_false_positives_beside_every_recall():
    records = [record(visible=True, score=0.9), record(visible=False, score=0.9)]
    text = format_gate_table(records, (0.5,))
    assert "FP" in text and "recall" in text
    assert "100%" in text and "50%" in text


def test_the_per_walk_table_totals_the_corpus():
    records = [record(walk="a", visible=True, score=0.9),
               record(walk="b", visible=True, score=0.1)]
    text = format_per_walk(records, 0.5)
    assert "TOTAL" in text and "a" in text and "b" in text


def test_the_comparison_prints_a_row_per_config_per_budget():
    a = [record(visible=True, score=0.9), record(visible=False, score=0.4)]
    b = [record(visible=True, score=0.3), record(visible=False, score=0.8)]
    text = format_comparison([("shipped", a), ("candidate", b)], [0, 3])
    assert text.count("shipped") == 2 and text.count("candidate") == 2


def test_default_gates_straddle_the_shipped_gate_and_the_corroboration_bar():
    """0.80 is DEFAULT_MATCH_PROBABILITY and 0.50 is 1.11a's proposed
    corroboration bar. A sweep that skipped either would be unable to
    reproduce the two numbers every argument in 4.10 is built on."""
    assert 0.8 in DEFAULT_GATES and 0.5 in DEFAULT_GATES


# ---------------------------------------------------------------------------
# The command line. `compare` needs no model at all, and `score`'s plumbing
# is testable with the pipeline injected -- which is worth doing, because
# every mistake this module exists to prevent is a mistake in a printed
# table rather than in a return value.
# ---------------------------------------------------------------------------

from control import perception_eval  # noqa: E402


def test_no_subcommand_prints_help_rather_than_traceback(capsys):
    assert perception_eval.main([]) == 2
    assert "score" in capsys.readouterr().out


def test_score_refuses_a_corpus_with_no_labels_and_says_where_they_live(tmp_path, capsys):
    """A usable message, not a stack trace: the four walks live in S3 and in
    `recordings/`, and the commonest way to run this wrong is from the wrong
    directory."""
    rc = perception_eval.main(["score", "--recordings", str(tmp_path)])
    assert rc == 1
    assert "four rig walks" in capsys.readouterr().out


def test_score_runs_the_corpus_and_prints_both_tables(tmp_path, monkeypatch, capsys):
    _walk_dir(tmp_path, "a-walk", labels={"frame-0000.jpg": True,
                                          "frame-0001.jpg": False})

    answers = iter([
        Perception(status=DETECTED, candidates=[_candidate(0.9, 0.0)],
                   best=_candidate(0.9, 0.0)),
        Perception(status=ABSENT, candidates=[_candidate(0.0, 0.5)]),
    ])

    class Stub:
        def perceive(self, frame):
            return next(answers)

    monkeypatch.setattr(perception_eval, "_build_pipeline",
                        lambda args, target: Stub())
    save = tmp_path / "records.json"
    rc = perception_eval.main(["score", "--recordings", str(tmp_path),
                               "--quiet", "--save", str(save)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "per walk at P >= 0.80" in out and "TOTAL" in out
    assert "precision" in out
    # The reference rule is stated in the output itself, so a pasted table
    # carries it -- every wrong number in 4.10 was a table read without one.
    assert "labels.json" in out and "never walk.jsonl" in out
    assert save.is_file()


def test_score_reports_unreadable_frames_rather_than_burying_them(tmp_path, monkeypatch, capsys):
    """1.12: `unavailable` must never be silently counted as a miss. If it
    is non-zero the run is measuring the harness, not the model."""
    _walk_dir(tmp_path, "a-walk", labels={"frame-0000.jpg": True})

    class Wedged:
        def perceive(self, frame):
            return Perception(status=UNAVAILABLE, reason="wedged")

    monkeypatch.setattr(perception_eval, "_build_pipeline",
                        lambda args, target: Wedged())
    perception_eval.main(["score", "--recordings", str(tmp_path), "--quiet"])
    assert "could not be read" in capsys.readouterr().out


def test_score_says_so_when_the_models_are_not_installed(tmp_path, monkeypatch, capsys):
    """`ultralytics`/`torch` are an optional install, so this is the normal
    state of a fresh checkout -- the message naming the pip command is the
    useful part, exactly as it is at mission start."""
    from brain.perceive import PerceptionUnavailable

    _walk_dir(tmp_path, "a-walk")

    def missing(args, target):
        raise PerceptionUnavailable("ultralytics is not installed. pip install ...")

    monkeypatch.setattr(perception_eval, "_build_pipeline", missing)
    rc = perception_eval.main(["score", "--recordings", str(tmp_path), "--quiet"])
    assert rc == 1
    assert "pip install" in capsys.readouterr().out


def test_score_prints_a_line_per_frame_unless_quiet(tmp_path, monkeypatch, capsys):
    _walk_dir(tmp_path, "a-walk", labels={"frame-0000.jpg": True})

    class Stub:
        def perceive(self, frame):
            return Perception(status=DETECTED, candidates=[_candidate(0.9)],
                              best=_candidate(0.9))

    monkeypatch.setattr(perception_eval, "_build_pipeline",
                        lambda args, target: Stub())
    perception_eval.main(["score", "--recordings", str(tmp_path)])
    assert "frame-0000.jpg" in capsys.readouterr().out


def test_score_can_be_pointed_at_one_walk(tmp_path, monkeypatch, capsys):
    _walk_dir(tmp_path, "wanted")
    _walk_dir(tmp_path, "not-wanted")

    class Stub:
        def perceive(self, frame):
            return Perception(status=ABSENT)

    monkeypatch.setattr(perception_eval, "_build_pipeline",
                        lambda args, target: Stub())
    perception_eval.main(["score", "--recordings", str(tmp_path),
                          "--walk", "wanted", "--quiet"])
    out = capsys.readouterr().out
    assert "wanted" in out and "not-wanted" not in out


def test_compare_matches_two_configs_on_false_positives(tmp_path, capsys):
    """The headline behaviour. 4.11's caution: a detector run at low enough
    confidence beats anything on recall while inventing targets, and
    reporting that recall without its false-positive count is how YOLO-World
    would have looked like a win."""
    shipped = [record(visible=True, score=0.9), record(visible=True, score=0.85),
               record(visible=False, score=0.2)]
    greedy = [record(visible=True, score=0.9), record(visible=True, score=0.9),
              record(visible=False, score=0.9)]
    a, b = tmp_path / "a.json", tmp_path / "b.json"
    save_records(a, shipped, {"detector": "yolo11s.pt", "proposer": "floor"})
    save_records(b, greedy, {"detector": "gdino", "proposer": "none"})

    rc = perception_eval.main(["compare", f"shipped={a}", f"greedy={b}",
                               "--fp-budget", "0"])
    out = capsys.readouterr().out
    assert rc == 0
    # At zero false positives the greedy config's recall collapses, which is
    # the entire reason the budget is matched before the recall is read.
    assert "shipped" in out and "greedy" in out
    assert "false-positive count" in out


def test_compare_labels_a_file_from_its_own_config_when_unnamed(tmp_path, capsys):
    path = tmp_path / "run.json"
    save_records(path, [record(visible=True, score=0.9)],
                 {"detector": "yolo11s.pt", "proposer": "sam"})
    perception_eval.main(["compare", str(path), "--fp-budget", "3"])
    out = capsys.readouterr().out
    assert "yolo11s.pt" in out and "sam" in out


# ---------- aligning decisions to the frames they were made on ----------

def _walk_with_teleop(tmp_path, rows, name="aligned-walk"):
    d = tmp_path / name
    d.mkdir()
    for i in range(len(rows) + 2):
        (d / ("frame-%04d.jpg" % i)).write_bytes(b"jpeg-ish")
    (d / "walk.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n")
    return d


def _row(seq, teleop_seq, decided_on, action="FORWARD"):
    return {
        "seq": seq, "file": "frame-%04d.jpg" % seq, "teleop_seq": teleop_seq,
        "navigate": {"action": action,
                     "_missionStatus": {"last_frame_seq": decided_on,
                                        "last_action": action}},
    }


def test_a_decision_is_tied_to_the_frame_it_was_actually_made_on(tmp_path):
    """The lag is the whole point: frame 5 carries the status, but that
    decision was taken on the frame pushed as teleop_seq 2 -- which is
    frame 2, not frame 5."""
    d = _walk_with_teleop(tmp_path, [
        _row(2, 2, None), _row(5, 5, 2, action="LEFT"),
    ])
    out = decisions_by_frame(d)
    assert "frame-0002.jpg" in out
    assert out["frame-0002.jpg"]["last_action"] == "LEFT"
    # And emphatically NOT filed against the frame that merely carried it.
    assert "frame-0005.jpg" not in out


def test_a_walk_with_no_alignment_yields_nothing_rather_than_a_guess(tmp_path):
    """A plain Robot-view walk has no teleop sequence. Returning the naive
    pairing here would be exactly the coincidence this function exists to
    replace -- and it would look identical to a real alignment."""
    d = _walk_with_teleop(tmp_path, [
        {"seq": 0, "file": "frame-0000.jpg",
         "navigate": {"action": "FORWARD"}},
    ])
    assert decisions_by_frame(d) == {}


def test_a_walk_with_no_jsonl_is_empty_not_an_error(tmp_path):
    d = tmp_path / "bare"
    d.mkdir()
    assert decisions_by_frame(d) == {}


# ---------- a separation too thin to use is not a separation ----------

def test_a_saturated_score_is_reported_as_unseparable():
    """Qwen3-VL-4B, 2026-09-09: true sightings at 0.9999999999856 and
    confabulated frames at 0.9999999999766. The arithmetic separates them and
    reports 10/10 at zero false positives off a **9e-12** margin -- while at
    any usable gate the same model fires on 34 of 34 invented frames. The
    number was real and useless, and it flattered the model the hardware
    decision was leaning toward."""
    records = [record(visible=True, score=0.9999999999856) for _ in range(3)]
    records += [record(visible=False, score=0.9999999999766) for _ in range(5)]
    s = recall_at_fp_budget(records, 0)
    assert s["tp"] == 3              # the arithmetic still finds it
    assert s["separable"] is False   # and the caller is told not to believe it
    assert s["margin"] < 1e-6


def test_a_real_separation_is_reported_as_separable():
    records = [record(visible=True, score=0.75) for _ in range(3)]
    records += [record(visible=False, score=0.14) for _ in range(5)]
    s = recall_at_fp_budget(records, 0)
    assert s["tp"] == 3 and s["separable"] is True
    assert s["margin"] > 0.5


def test_frames_from_restricts_the_corpus_to_a_detections_file(tmp_path):
    """P16's replay path. A recorded detector covers the frames that run was
    given; scoring it against the whole corpus puts every uncovered frame in
    the denominator as a miss. That is not a small distortion -- it read 869
    unavailable of 1234 and made a quantized config whose detector is
    wrecked outscore fp32, which is the reverse of every detector-level
    measurement of the same two runs."""
    _walk_dir(tmp_path, "a-walk",
              labels={"frame-0000.jpg": True, "frame-0001.jpg": True})

    walks = perception_eval.load_corpus(tmp_path)
    assert [len(w.frames) for w in walks] == [2]

    covered = perception_eval.load_corpus(
        tmp_path, frames_only={"a-walk/frame-0000.jpg"})
    assert [len(w.frames) for w in covered] == [1]
    # `visible` is derived, so trimming the frame list is the whole job.
    assert covered[0].visible == 1


def test_frames_from_refuses_a_filter_that_matches_nothing(tmp_path):
    """Silently scoring zero frames would report 0% and look like a result."""
    _walk_dir(tmp_path, "a-walk", labels={"frame-0000.jpg": True})
    with pytest.raises(perception_eval.CorpusError, match="do not overlap"):
        perception_eval.load_corpus(tmp_path, frames_only={"other/frame.jpg"})
