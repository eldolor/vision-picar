"""The Hailo compile loop, exercised without a Hailo and without torch.

`tools/hailo/` answers one question -- can OWLv2 compile to a HEF -- on a
rented x86 box, and its two failure modes are both silent. The host head can
reassemble the accelerator's outputs *wrongly* and still produce plausible
boxes; the sweep can lose a variant's failure and report a tidier matrix
than it measured. Neither shows up on the day, and both would be discovered
after ordering a part.

So both are pinned here, against fakes -- the same rule the rest of this
suite follows for the perception tier (`tests/test_perceive.py`): the logic
runs in CI, the model runs by hand. Nothing here imports torch, transformers
or `hailo_sdk_client`.
"""

from __future__ import annotations

import json
import sys
import types

import numpy as np
import pytest

from tools.hailo import compile_owlv2, owlv2_host_head


# --------------------------------------------------------------- the head --

def _parts(patches=7, dim=5, seed=0):
    rng = np.random.default_rng(seed)
    return {
        "image_class_embeds": rng.normal(size=(1, patches, dim)).astype(np.float32),
        "logit_shift": rng.normal(size=(1, patches, 1)).astype(np.float32),
        "logit_scale": rng.normal(size=(1, patches, 1)).astype(np.float32),
        "pred_boxes": rng.normal(size=(1, patches, 4)).astype(np.float32),
        "objectness_logits": rng.normal(size=(1, patches)).astype(np.float32),
    }


def _reference(parts, query, box_bias):
    """The head as transformers writes it, transcribed independently."""
    def l2(x):
        return x / (np.linalg.norm(x, axis=-1, keepdims=True) + 1e-6)

    embeds = l2(parts["image_class_embeds"])
    scale = np.where(parts["logit_scale"] > 0, parts["logit_scale"],
                     np.expm1(np.minimum(parts["logit_scale"], 0.0))) + 1.0
    logits = (embeds @ np.swapaxes(l2(query[None]), -1, -2)
              + parts["logit_shift"]) * scale
    boxes = 1.0 / (1.0 + np.exp(-(parts["pred_boxes"] + box_bias)))
    return logits, boxes


def test_the_two_heads_are_the_same_head():
    """`minimal` moves three ops to the CPU and must change no number.

    That equivalence is the entire licence to try the minimal export: if the
    full graph will not compile, the fallback has to be the same model, not
    a similar one.
    """
    raw = _parts()
    box_bias = np.linspace(-1, 1, 7 * 4).reshape(7, 4).astype(np.float32)
    query = np.random.default_rng(1).normal(size=(1, 5)).astype(np.float32)

    ref_logits, ref_boxes = _reference(raw, query, box_bias)

    # `minimal`: the accelerator hands back pre-activation tensors.
    got_logits, got_boxes, _ = owlv2_host_head.join(
        raw, query, head="minimal", box_bias=box_bias)
    assert np.allclose(got_logits, ref_logits, atol=1e-5)
    assert np.allclose(got_boxes, ref_boxes, atol=1e-6)

    # `full`: the accelerator already did them, so the host is handed the
    # post-activation values and must NOT apply them a second time.
    pre_applied = dict(raw)
    pre_applied["image_class_embeds"] = owlv2_host_head.normalize(
        raw["image_class_embeds"])
    pre_applied["logit_scale"] = np.where(
        raw["logit_scale"] > 0, raw["logit_scale"],
        np.expm1(np.minimum(raw["logit_scale"], 0.0))) + 1.0
    pre_applied["pred_boxes"] = ref_boxes
    full_logits, full_boxes, _ = owlv2_host_head.join(pre_applied, query,
                                                      head="full")
    assert np.allclose(full_logits, ref_logits, atol=1e-5)
    assert np.allclose(full_boxes, ref_boxes, atol=1e-6)


def test_minimal_head_refuses_to_guess_the_box_bias():
    """A missing box_bias must raise, never silently produce boxes.

    `box_bias` is a per-patch constant, so omitting it yields boxes that are
    wrong by a smooth function of position -- plausible everywhere and
    correct nowhere, which is the worst shape a bug can have here.
    """
    with pytest.raises(ValueError, match="box_bias"):
        owlv2_host_head.join(_parts(), np.zeros((1, 5), np.float32),
                             head="minimal")


def test_unknown_head_is_an_error_not_a_default():
    with pytest.raises(ValueError, match="minimal"):
        owlv2_host_head.join(_parts(), np.zeros((1, 5), np.float32),
                             head="fast")


def test_detections_are_thresholded_and_converted_to_xyxy():
    parts = _parts(patches=4, dim=3)
    parts["pred_boxes"] = np.array(
        [[[0.5, 0.5, 0.2, 0.4]] * 4], dtype=np.float32)
    query = np.ones((1, 3), dtype=np.float32)
    out = owlv2_host_head.detections(parts, query, threshold=0.0,
                                     size=(100, 200), head="full")
    assert len(out) == 4
    _, (x1, y1, x2, y2) = out[0]
    assert (x1, y1, x2, y2) == pytest.approx((80.0, 30.0, 120.0, 70.0))


# -------------------------------------------------------------- the sweep --

class _FakeRunner:
    """A ClientRunner that fails where it is told to."""

    def __init__(self, fail_at=None, message="unsupported op Elu"):
        self.fail_at = fail_at
        self.message = message
        self.calls = []

    def translate_onnx_model(self, *a, **kw):
        self.calls.append("translate")
        if self.fail_at == "translate":
            raise ValueError(self.message)
        return object(), {}

    def save_har(self, path):
        open(path, "wb").close()

    def load_model_script(self, script):
        self.calls.append("model_script")

    def optimize(self, calib):
        self.calls.append("optimize")
        if self.fail_at == "optimize":
            raise MemoryError(self.message)

    def compile(self):
        self.calls.append("compile")
        if self.fail_at == "compile":
            raise RuntimeError(self.message)
        return b"HEF\x00fake"


@pytest.fixture
def build(tmp_path, monkeypatch):
    """A build directory shaped like a real export, minus the 348MB."""
    meta = {
        "model_id": "google/owlv2-base-patch16-ensemble",
        "input_name": "pixel_values", "image_size": 960, "num_patches": 3600,
        "image_mean": [0.485, 0.456, 0.406], "image_std": [0.229, 0.224, 0.225],
        "opsets": [17], "heads": ["full"],
    }
    (tmp_path / "owlv2_export.json").write_text(json.dumps(meta))
    (tmp_path / "owlv2_image_tower_960_op17_full.onnx").write_bytes(b"onnx")
    (tmp_path / "owlv2_image_tower_960_op14_full.onnx").write_bytes(b"onnx")
    np.save(tmp_path / "calib_normalized.npy", np.zeros((2, 4, 4, 3), np.float32))
    np.save(tmp_path / "calib_uint8.npy", np.zeros((2, 4, 4, 3), np.uint8))
    (tmp_path / "calib_manifest.json").write_text(
        json.dumps({"n": 2, "seed": 0, "walks": ["w"]}))
    return tmp_path


def _sweep(build, monkeypatch, runner, argv):
    monkeypatch.setattr(compile_owlv2, "_runner", lambda arch: runner)
    monkeypatch.setattr(compile_owlv2, "_dfc_version", lambda: "3.31.0-fake")
    compile_owlv2.main([*argv, "--build", str(build)])
    return json.loads((build / "compile_report.json").read_text())


def test_a_successful_sweep_writes_a_hef_and_says_so(build, monkeypatch, capsys):
    report = _sweep(build, monkeypatch, _FakeRunner(),
                    ["--layout", "normalized", "--opset", "17"])
    attempt, = report["attempts"]
    assert attempt["reached"] == "compile"
    assert (build / attempt["hef"]).read_bytes() == b"HEF\x00fake"
    assert "OWLv2 COMPILES" in capsys.readouterr().out


def test_a_failure_records_the_op_it_died_on(build, monkeypatch):
    """The headline is the deliverable.

    "It did not compile" is not an answer -- an unsupported op may be one
    line moved to the CPU, while a resource wall is a different board. The
    report has to carry which one it was.
    """
    runner = _FakeRunner(fail_at="translate",
                         message="Unsupported op ReduceL2 in node /Div")
    report = _sweep(build, monkeypatch, runner,
                    ["--layout", "normalized", "--opset", "17"])
    stages = report["attempts"][0]["stages"]
    assert stages["translate"]["ok"] is False
    assert "ReduceL2" in stages["translate"]["headline"]
    assert report["attempts"][0]["reached"] is None
    assert "optimize" not in stages          # never attempted after a fail
    assert (build / "logs" / stages["translate"]["log"].split("/")[-1]).exists()


def test_an_out_of_memory_optimize_is_a_result_not_a_crash(build, monkeypatch):
    """MemoryError inherits from Exception, but SystemExit does not.

    The DFC raises both, and an OOM at 3600 tokens is precisely the outcome
    P5 predicts -- so it has to land in the report as a measurement rather
    than take the sweep down with it.
    """
    runner = _FakeRunner(fail_at="optimize", message="out of host memory")
    report = _sweep(build, monkeypatch, runner,
                    ["--layout", "normalized", "--opset", "17"])
    stages = report["attempts"][0]["stages"]
    assert stages["translate"]["ok"] is True
    assert stages["optimize"]["error_type"] == "MemoryError"
    assert report["attempts"][0]["reached"] == "translate"


def test_one_variant_failing_does_not_end_the_sweep(build, monkeypatch):
    report = _sweep(build, monkeypatch, _FakeRunner(fail_at="compile"),
                    ["--opset", "17", "--opset", "14"])
    # two opsets x two layouts, all four attempted despite all four failing
    assert len(report["attempts"]) == 4
    assert {a["reached"] for a in report["attempts"]} == {"optimize"}


def test_the_report_survives_a_sweep_that_never_finishes(build, monkeypatch):
    """Written after every cell, not at the end.

    A sweep can run for hours on a rented box. If it dies in the third one,
    the first two are still most of the answer, and re-renting to re-learn
    them is the avoidable cost.
    """
    seen = []

    class _Interrupted(_FakeRunner):
        def translate_onnx_model(self, *a, **kw):
            if len(seen) >= 2:
                raise KeyboardInterrupt
            seen.append(1)
            return super().translate_onnx_model(*a, **kw)

    with pytest.raises(KeyboardInterrupt):
        _sweep(build, monkeypatch, _Interrupted(),
               ["--opset", "17", "--opset", "14"])
    report = json.loads((build / "compile_report.json").read_text())
    assert len(report["attempts"]) == 2
    assert all(a["hef"] for a in report["attempts"])


def test_the_uint8_layout_ships_a_normalization_model_script(build, monkeypatch):
    """Raw camera bytes to the part, mean/std on the part.

    This is the arrangement worth having on the robot -- it keeps a per-frame
    float conversion off the Pi's CPU -- so the scale matters: the DFC's
    normalization() works in 0-255 units, not 0-1.
    """
    runner = _FakeRunner()
    report = _sweep(build, monkeypatch, runner,
                    ["--layout", "uint8", "--opset", "17"])
    script = report["attempts"][0]["model_script"]
    assert "normalization1 = normalization(" in script
    assert "123.675" in script          # 0.485 * 255
    assert "model_script" in runner.calls

    report = _sweep(build, monkeypatch, _FakeRunner(),
                    ["--layout", "normalized", "--opset", "17"])
    assert "model_script" not in report["attempts"][0]


def test_calibration_limit_is_honoured(build, monkeypatch):
    calib = compile_owlv2._calibration(build, "normalized", 1)
    assert calib.shape[0] == 1
    assert compile_owlv2._calib_meta(build, 1)["n"] == 1


def test_a_missing_calibration_set_is_named_not_guessed(tmp_path):
    with pytest.raises(SystemExit, match="calibration_set"):
        compile_owlv2._calibration(tmp_path, "normalized", None)
