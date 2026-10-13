"""
tools/jetson/bench_identity_trt.py

3.78 arm T (`docs/plans/ros-alignment/3.78-trt-vs-llamacpp.md`): the arrival
question of 3.48 asked of Qwen2.5-VL-3B under TensorRT Edge-LLM v0.6.0 on the
Jetson, on the same pinned frames, so its records score with
`control/identity_eval.py` exactly as `bench_identity_llama`'s do.

    python -m tools.jetson.bench_identity_trt \\
        --runner ~/bench-378/TensorRT-Edge-LLM/build/examples/llm/llm_inference \\
        --engine-dir ~/bench-378/ws/X/engines/llm \\
        --visual-engine-dir ~/bench-378/ws/X/engines/visual \\
        --tokenizer ~/bench-378/ws/X/onnx/llm/tokenizer.json \\
        --recordings recordings --walk <w> ... --out T.json

**Amendment 1.** Edge-LLM's `handleRequest()` returns token ids and text
only -- no logprobs, no stream -- so:

* the decision is read from the greedy tokens with 3.48's own rule
  (`p_localised` fed a one-hot distribution: the generated token at
  probability 1);
* stopping at the decision is done by length: pass 1 (untimed) generates up
  to 64 tokens a frame and finds the deciding token's index; pass 2 (timed)
  runs each frame with `max_generate_length` = that index + 1.

The runner is Edge-LLM's `llm_inference` patched (`patch_llm_inference.py`
beside this file) to put each request's wall time (`ms`, stream
synchronised, JPEG decode and preprocessing included) and its `output_ids`
in its output, and to honour a per-request `max_generate_length`. The model
loads once per pass and the frames run back to back after 3 warm-up runs.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import time
from pathlib import Path
from typing import Optional, Sequence

from tools.jetson.bench_identity_llama import (
    MAX_TOKENS, PROMPT, MemSampler, mem_available_mb, p_localised, percentile)

WARMUP = 3


def one_hot_content(tokens: Sequence[str]) -> list:
    """Generated token texts -> the `logprobs.content` shape `p_localised`
    reads, each token at probability 1 (amendment 1)."""
    return [{"token": t, "logprob": 0.0, "top_logprobs": [{"token": t, "logprob": 0.0}]}
            for t in tokens]


def decide(tokens: Sequence[str]) -> tuple:
    """`(score, parsed, decided_at)` for one greedy reply: score 1.0 (YES)
    or 0.0 (NO), as `p_localised` reads a one-hot stream."""
    return p_localised(one_hot_content(tokens))


def stop_length(decided_at: Optional[int]) -> int:
    """Pass 2's `max_generate_length`: up to and including the deciding
    token, or the full 64 when the reply never decided (3.58's rule)."""
    return MAX_TOKENS if decided_at is None else decided_at + 1


def request(image: Path, target: str, max_len: int) -> dict:
    return {"max_generate_length": max_len,
            "messages": [{"role": "user", "content": [
                {"type": "image", "image": str(image)},
                {"type": "text", "text": PROMPT.format(target=target)}]}]}


def input_doc(reqs: list) -> dict:
    # Greedy: top_k 1 picks the argmax whatever the temperature.
    return {"batch_size": 1, "temperature": 1.0, "top_p": 1.0, "top_k": 1,
            "max_generate_length": MAX_TOKENS, "requests": reqs}


def token_texts(tok, ids: Sequence[int]) -> list:
    """Each id decoded alone, as llama-server's per-token `token` field is."""
    return [tok.decode([i]) for i in ids]


def run_pass(args, reqs: list, out: Path) -> dict:  # pragma: no cover -- needs the board
    inp = out.with_suffix(".input.json")
    inp.write_text(json.dumps(input_doc(reqs)))
    cmd = [str(Path(args.runner).expanduser()),
           f"--engineDir={Path(args.engine_dir).expanduser()}",
           f"--multimodalEngineDir={Path(args.visual_engine_dir).expanduser()}",
           f"--inputFile={inp}", f"--outputFile={out}", f"--warmup={WARMUP}"]
    before = mem_available_mb()
    sampler = MemSampler()
    sampler.start()
    t0 = time.monotonic()
    with open(f"{out}.log", "w") as log:
        rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
    wall = time.monotonic() - t0
    sampler.stop()
    doc = json.loads(out.read_text()) if out.exists() else {"responses": []}
    return {"rc": rc, "wall_s": wall, "responses": doc.get("responses", []),
            "mem_before": before, "mem_low": sampler.low}


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover
    from tokenizers import Tokenizer
    from control.perception_eval import load_corpus

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--runner", required=True)
    ap.add_argument("--engine-dir", required=True)
    ap.add_argument("--visual-engine-dir", required=True)
    ap.add_argument("--tokenizer", required=True)
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--walk", action="append", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    tok = Tokenizer.from_file(str(Path(args.tokenizer).expanduser()))
    walks = load_corpus(Path(args.recordings), only=args.walk)
    frames = [(w, p, vis, adj) for w in walks for p, vis, adj in w.frames]
    out = Path(args.out).expanduser()

    # Pass 1, untimed: where does each reply decide?
    p1 = run_pass(args, [request(p, w.target, MAX_TOKENS) for w, p, _, _ in frames],
                  out.with_suffix(".pass1.json"))
    first = {}
    for r in p1["responses"]:
        ids = r.get("output_ids")
        first[r["request_idx"]] = decide(token_texts(tok, ids)) if ids is not None else None

    # Pass 2, timed: each frame generates up to its deciding token.
    lengths = [stop_length(first[i][2]) if first.get(i) else MAX_TOKENS
               for i in range(len(frames))]
    p2 = run_pass(args, [request(p, w.target, n) for (w, p, _, _), n in zip(frames, lengths)],
                  out.with_suffix(".pass2.json"))
    by_idx = {r["request_idx"]: r for r in p2["responses"]}

    records, ms, replies = [], [], {}
    errors = noparse = object_replies = changed = 0
    for i, (w, p, vis, adj) in enumerate(frames):
        r = by_idx.get(i)
        if r is None or r.get("output_ids") is None:
            errors += 1
            records.append({"walk": w.name, "frame": p.name, "visible": vis,
                            "status": "unavailable", "score": None, "metric": "confidence",
                            "label": None, "candidates": 0, "adjudicated": adj, "ms": 0.0})
            print(f"{w.name}/{p.name} ERROR", flush=True)
            continue
        texts = token_texts(tok, r["output_ids"])
        score, parsed, decided = decide(texts)
        noparse += 0 if parsed else 1
        if parsed and texts[decided].strip().startswith("{"):
            object_replies += 1
        if first.get(i) and first[i][0] != score:
            changed += 1
        replies[f"{w.name}/{p.name}"] = "".join(texts)[:160]
        ms.append(r["ms"])
        records.append({"walk": w.name, "frame": p.name, "visible": vis,
                        "status": "detected" if score >= 0.5 else "absent",
                        "score": score, "metric": "confidence", "label": None,
                        "candidates": 1, "adjudicated": adj, "ms": r["ms"]})
        print(f"{w.name}/{p.name} vis={vis} p={score:.0f} {r['ms']:.0f}ms", flush=True)

    low = p2["mem_low"] if p2["mem_low"] is not None else p2["mem_before"]
    doc = {
        "config": {"detector": "trt-edgellm:Qwen2.5-VL-3B-Instruct int4_awq + visual fp16",
                   "runtime": "TensorRT Edge-LLM v0.6.0 llm_inference (patched, 3.78 amendment 1)",
                   "device": "jetson-orin-nano-super-15w", "prompt": PROMPT,
                   "max_tokens": MAX_TOKENS, "metric": "confidence (one-hot, greedy)",
                   "stop_at_decision": "by length (pass 1 -> pass 2)"},
        "records": records,
        "latency_ms": ({"n": len(ms), "p50": statistics.median(ms),
                        "p90": percentile(ms, 90), "max": max(ms)} if ms else None),
        "memory_mb": {"available_before": p2["mem_before"], "available_low": low,
                      "drop": p2["mem_before"] - low},
        # Process wall time less the timed requests: load, warm-up and exit.
        "overhead_s": p2["wall_s"] - sum(ms) / 1000.0,
        "pass_rc": [p1["rc"], p2["rc"]], "decision_changed_pass1_to_2": changed,
        "noparse": noparse, "object_replies": object_replies, "errors": errors,
        "replies": replies,
    }
    out.write_text(json.dumps(doc, indent=1))
    print(json.dumps({k: doc[k] for k in ("latency_ms", "memory_mb", "overhead_s", "pass_rc",
                                          "decision_changed_pass1_to_2", "noparse",
                                          "object_replies", "errors")}))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
