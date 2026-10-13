"""
tools/jetson/bench_identity_trt.py

3.78 arm T (`docs/plans/ros-alignment/3.78-trt-vs-llamacpp.md`): the arrival
question of 3.48 asked of a VLM under TensorRT Edge-LLM v0.9.1 on the Jetson,
on the same pinned frames, so its records score with
`control/identity_eval.py` exactly as `bench_identity_llama`'s do.

    python -m tools.jetson.bench_identity_trt \\
        --driver ~/bench-378/ell91/build/examples/llm/edgellm_identity \\
        --engine-dir ~/bench-378/ws91/X/engines/llm \\
        --visual-engine-dir ~/bench-378/ws91/X/engines/visual \\
        --tokenizer ~/bench-378/ws91/X/onnx/llm/tokenizer.json \\
        --recordings recordings --walk <w> ... --out T.json

**Amendment 3: measured as arm L is.** The driver (`edgellm_identity.cpp`
beside this file) keeps one runtime resident and streams each reply token by
token with its top-20 logprobs (log softmax of the raw logits, the quantity
llama-server's `top_logprobs` returns). This client applies L's own
`decision_reached`, sends `C` to cancel at the deciding token, and scores the
reply with L's own `p_localised`. `ms` is client-side, from sending the
request -- whose parsing decodes the JPEG -- to receiving the deciding token.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Optional, Sequence

from tools.jetson.bench_identity_llama import (
    MAX_TOKENS, PROMPT, TOP_LOGPROBS, WARMUP, MemSampler, decision_reached,
    mem_available_mb, p_localised, percentile)


def request_doc(image: Path, target: str) -> dict:
    """One frame as an Edge-LLM request file: greedy (top_k 1 picks the
    argmax whatever the temperature), 64 tokens at most, top-20 logprobs."""
    return {"batch_size": 1, "temperature": 1.0, "top_p": 1.0, "top_k": 1,
            "max_generate_length": MAX_TOKENS, "num_logprobs": TOP_LOGPROBS,
            "requests": [{"messages": [{"role": "user", "content": [
                {"type": "image", "image": str(image)},
                {"type": "text", "text": PROMPT.format(target=target)}]}]}]}


def parse_token_line(line: str) -> tuple:
    """`T <id> <k> <id>:<lp> ...` -> `(id, [(id, logprob), ...])`."""
    parts = line.split()
    if not parts or parts[0] != "T":
        raise ValueError(f"not a token line: {line!r}")
    chosen, k = int(parts[1]), int(parts[2])
    alts = []
    for p in parts[3:3 + k]:
        i, lp = p.split(":")
        alts.append((int(i), float(lp)))
    return chosen, alts


def parse_end_line(line: str) -> tuple:
    """`E <reason> <ms> <ok>` -> `(reason, ms, ok)`."""
    _, reason, ms, ok = line.split()
    return int(reason), float(ms), ok == "1"


def content_entry(decode, chosen: int, alts: Sequence[tuple]) -> dict:
    """One streamed token in the `logprobs.content` shape `p_localised`
    reads. `decode` turns one id into its text, as llama-server's per-token
    `token` field is."""
    lp = next((v for i, v in alts if i == chosen), 0.0)
    return {"token": decode(chosen), "logprob": lp,
            "top_logprobs": [{"token": decode(i), "logprob": v} for i, v in alts]}


class Driver:  # pragma: no cover -- needs the board
    def __init__(self, cmd: list, log):
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=log, text=True, bufsize=1)

    def wait_ready(self):
        for line in self.proc.stdout:
            if line.strip() == "READY":
                return
        raise RuntimeError(f"driver exited before READY ({self.proc.wait()})")

    def ask(self, req_path: Path, decode) -> dict:
        """One frame under the stopping rule: content up to the decision,
        the client-side ms to it, and the driver's end line."""
        t0 = time.perf_counter()
        self.proc.stdin.write(f"{req_path}\n")
        self.proc.stdin.flush()
        content, took, decided = [], None, False
        for raw in self.proc.stdout:
            line = raw.strip()
            if line.startswith("T "):
                if decided:
                    continue  # tokens generated before the cancel landed
                chosen, alts = parse_token_line(line)
                content.append(content_entry(decode, chosen, alts))
                if decision_reached(content):
                    took = (time.perf_counter() - t0) * 1000
                    decided = True
                    self.proc.stdin.write("C\n")
                    self.proc.stdin.flush()
            elif line.startswith("E "):
                reason, driver_ms, ok = parse_end_line(line)
                if took is None:
                    took = (time.perf_counter() - t0) * 1000
                return {"content": content, "ms": took, "decided": decided,
                        "ok": ok, "reason": reason, "driver_ms": driver_ms}
        raise RuntimeError(f"driver exited mid-request ({self.proc.wait()})")

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=60)
        except Exception:
            self.proc.kill()


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover
    from tokenizers import Tokenizer
    from control.perception_eval import load_corpus

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--driver", required=True)
    ap.add_argument("--engine-dir", required=True)
    ap.add_argument("--visual-engine-dir", required=True)
    ap.add_argument("--tokenizer", required=True)
    ap.add_argument("--model-name", default="Qwen2.5-VL-3B-Instruct")
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--walk", action="append", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    tok = Tokenizer.from_file(str(Path(args.tokenizer).expanduser()))
    cache: dict = {}

    def decode(i: int) -> str:
        if i not in cache:
            cache[i] = tok.decode([i], skip_special_tokens=False)
        return cache[i]

    walks = load_corpus(Path(args.recordings), only=args.walk)
    frames = [(w, p, vis, adj) for w in walks for p, vis, adj in w.frames]
    reqdir = Path(tempfile.mkdtemp(prefix="trt378-"))
    paths = []
    for n, (w, p, _, _) in enumerate(frames):
        rp = reqdir / f"{n:04d}.json"
        rp.write_text(json.dumps(request_doc(p.resolve(), w.target)))
        paths.append(rp)

    out = Path(args.out).expanduser()
    before = mem_available_mb()
    sampler = MemSampler()
    sampler.start()
    log = open(f"{out}.driver.log", "w")
    drv = Driver([str(Path(args.driver).expanduser()),
                  str(Path(args.engine_dir).expanduser()),
                  str(Path(args.visual_engine_dir).expanduser())], log)
    records, ms, replies = [], [], {}
    errors = noparse = object_replies = undecided = bare_tokens = 0
    try:
        t0 = time.monotonic()
        drv.wait_ready()
        load_s = time.monotonic() - t0
        for rp in paths[:WARMUP]:
            drv.ask(rp, decode)
        for (w, p, vis, adj), rp in zip(frames, paths):
            got = drv.ask(rp, decode)
            if not got["ok"] and not got["decided"]:
                errors += 1
                records.append({"walk": w.name, "frame": p.name, "visible": vis,
                                "status": "unavailable", "score": None,
                                "metric": "confidence", "label": None, "candidates": 0,
                                "adjudicated": adj, "ms": 0.0})
                print(f"{w.name}/{p.name} ERROR reason={got['reason']}", flush=True)
                continue
            content = got["content"]
            # A token streamed without alternatives would score as one-hot
            # (amendment 1's rule) where amendment 3 promises top-20: count it.
            bare_tokens += sum(1 for e in content if not e["top_logprobs"])
            score, parsed, decided = p_localised(content)
            noparse += 0 if parsed else 1
            undecided += 0 if got["decided"] else 1
            if parsed and content[decided]["token"].strip().startswith("{"):
                object_replies += 1
            replies[f"{w.name}/{p.name}"] = "".join(e["token"] for e in content)[:160]
            ms.append(got["ms"])
            records.append({"walk": w.name, "frame": p.name, "visible": vis,
                            "status": "detected" if score >= 0.5 else "absent",
                            "score": score, "metric": "confidence", "label": None,
                            "candidates": 1, "adjudicated": adj, "ms": got["ms"]})
            print(f"{w.name}/{p.name} vis={vis} p={score:.3f} {got['ms']:.0f}ms", flush=True)
    finally:
        sampler.stop()
        drv.close()
        log.close()

    low = sampler.low if sampler.low is not None else before
    doc = {
        "config": {"detector": f"trt-edgellm:{args.model_name} int4_awq + visual fp16",
                   "runtime": "TensorRT Edge-LLM v0.9.1, edgellm_identity (3.78 amendment 3)",
                   "device": "jetson-orin-nano-super-15w", "prompt": PROMPT,
                   "max_tokens": MAX_TOKENS, "top_logprobs": TOP_LOGPROBS,
                   "metric": "confidence", "stop_at_decision": True},
        "records": records,
        "latency_ms": ({"n": len(ms), "p50": statistics.median(ms),
                        "p90": percentile(ms, 90), "max": max(ms)} if ms else None),
        "memory_mb": {"available_before": before, "available_low": low, "drop": before - low},
        "load_s": load_s, "noparse": noparse, "undecided": undecided,
        "tokens_without_logprobs": bare_tokens,
        "object_replies": object_replies, "errors": errors, "replies": replies,
    }
    out.write_text(json.dumps(doc, indent=1))
    print(json.dumps({k: doc[k] for k in ("latency_ms", "memory_mb", "load_s", "noparse",
                                          "undecided", "tokens_without_logprobs",
                                          "object_replies", "errors")}))
    if bare_tokens:
        print(f"WARNING: {bare_tokens} tokens arrived without logprobs -- "
              "those frames were scored one-hot, not as amendment 3 states", flush=True)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
