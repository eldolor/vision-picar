"""
tools/jetson/bench_identity_llama.py

3.48 amendment 1 (`docs/plans/ros-alignment/3.48-local-identity.md`): one
small VLM, as a GGUF under llama.cpp's `llama-server`, asked the arrival
question on the pinned 299 frames -- on the Jetson, at 15 W, the way it
would run on the car. Writes a records file `control/identity_eval.py`
scores (`--records name=<file>:0.5`), plus the latency and memory the plan's
criteria 5 and 6 read.

    python -m tools.jetson.bench_identity_llama \\
        --server ~/bench-348/llama.cpp/build/bin/llama-server \\
        --model ~/bench-348/models/X.gguf --mmproj ~/bench-348/models/mmproj-X.gguf \\
        --recordings recordings --walk <w> ... --out identity-X.json

**The score is P(it localises), the A10G VLM rows' quantity**
(`brain/perceive_lab.py` `VlmDetector._locate`, whose prompt is reused
verbatim): at the step where the reply opens its JSON list, the
probability that a box follows (`{`) rather than the list closing empty
(`]`). It is read from the chat-completions API's `top_logprobs`, so it
has to cope with tokenizers that merge `[` with what follows (`[{`, `[]`):
see `p_localised`. Never the reply's text alone -- a yes/no read from text
can be answered from prior (P7: P(yes) 0.884 on a frame the model would not
box).

**Memory** is the drop in the board's `MemAvailable` from before the server
starts to its lowest point during the run, sampled every second: on the
Jetson's unified memory a process's RSS misses what CUDA holds, and
`MemAvailable` is what the 3.25 GB bar was derived from (3.33 / 3.37).

**`--stop-at-decision`** (3.58, `docs/plans/ros-alignment/3.58-local-
identity-decision.md`): the reply is streamed and the client closes the
connection -- llama-server then cancels the task -- at the first token after
which `p_localised` has everything it reads (`decision_reached`). The
record's `ms` is then the client-side time from before the image is read and
encoded to the arrival of that token: all a deployment would wait for.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import statistics
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional, Sequence

# `VlmDetector._locate()`'s prompt, verbatim, so these rows ask exactly what
# the A10G VLM rows asked.
PROMPT = ('Output the bounding box of the {target} as JSON: '
          '[{{"bbox_2d": [x1, y1, x2, y2]}}]. If it is not visible, '
          'output []. Output JSON only.')
MAX_TOKENS = 64
TOP_LOGPROBS = 20
WARMUP = 3


def _mass(alternatives, pred) -> float:
    return sum(math.exp(a["logprob"]) for a in alternatives
               if pred(a["token"].strip()))


def _is_ws(tok: str) -> bool:
    return tok != "" and tok.strip() == ""


def _resolve(content: Sequence[dict], i: int) -> float:
    """P(a box follows) from step `i` on, `i` being the step after the list
    opened. `{` decides found, `]` decides empty, and a whitespace-only
    token defers: its mass carries to the next generated step when
    whitespace is what the model generated (amendment 2)."""
    if i >= len(content):
        return 0.0
    alts = content[i].get("top_logprobs") or []
    b = _mass(alts, lambda t: t.startswith("{"))
    e = _mass(alts, lambda t: t.startswith("]"))
    w = sum(math.exp(a["logprob"]) for a in alts if _is_ws(a["token"]))
    f = (_resolve(content, i + 1) if _is_ws(content[i]["token"])
         else (b / (b + e) if (b + e) > 0 else 0.0))
    denom = b + e + w
    return (b + w * f) / denom if denom > 0 else 0.0


def p_localised(content: Sequence[dict]) -> tuple:
    """`content` is the API's `choices[0].logprobs.content`: one entry per
    generated token, `{"token", "logprob", "top_logprobs": [{"token",
    "logprob"}, ...]}`. Returns `(score, parsed, decided_at)`: `parsed` is
    False when the reply never opened a list (scored 0), and `decided_at`
    is the index of the generated token that settled it (for the
    time-to-decision readout), or None.

    At the first generated token that starts with `[` (whitespace
    stripped), the alternatives split three ways: a box already (`[{`...),
    empty already (`[]`), or a bare `[` (with or without trailing
    whitespace) whose fate is decided later. With `F`, `E`, `O` those masses
    and `f` the probability a box follows the bare `[`:

        P(found) = (F + O * f) / (F + E + O)

    When the bare `[` was generated, `f` is read from the following steps
    (`_resolve`, deferring through whitespace); when a merged token was
    generated, the bare branch was never sampled and `f` is `F / (F + E)`.
    A token outside the top 20 contributes nothing.
    """
    for i, step in enumerate(content):
        tok = step["token"].strip()
        if tok.startswith("{"):
            # A bare object before any list -- `{"bbox_2d": [...]}` -- is a
            # box, answered outside the asked-for list. Read at this step:
            # a box opening (`{` or `[{`) against an empty list (`[]`); a
            # bare `[` here has an unknown fate and is left out (review of
            # amendment 2). `bench_identity_llama` counts these replies.
            alts = step.get("top_logprobs") or [{"token": step["token"],
                                                 "logprob": step["logprob"]}]
            box = _mass(alts, lambda t: t.startswith("{") or t.startswith("[{"))
            none = _mass(alts, lambda t: t.startswith("[]"))
            return (box / (box + none) if (box + none) > 0 else 0.0), True, i
        if not tok.startswith("["):
            continue
        alts = step.get("top_logprobs") or [{"token": step["token"],
                                             "logprob": step["logprob"]}]
        found = _mass(alts, lambda t: t.startswith("[{"))
        empty = _mass(alts, lambda t: t.startswith("[]"))
        bare = _mass(alts, lambda t: t == "[")
        decided = i
        if tok == "[":
            f = _resolve(content, i + 1)
            j = i + 1
            while j < len(content) and _is_ws(content[j]["token"]):
                j += 1
            decided = min(j, len(content) - 1)
        else:
            f = found / (found + empty) if (found + empty) > 0 else 0.0
        denom = found + empty + bare
        return ((found + bare * f) / denom if denom > 0 else 0.0), True, decided
    return 0.0, False, None


def decision_reached(content: Sequence[dict]) -> bool:
    """3.58's stopping rule: has the reply so far given `p_localised`
    everything it reads? True at a bare object (`{` before any list), at a
    merged `[{` / `[]`, or -- after a bare `[` -- at the first token that is
    not whitespace-only. `p_localised` reads nothing past that token, so on
    the same greedy stream the score over the stopped reply equals the
    score over the full one (pinned by a test)."""
    for i, step in enumerate(content):
        tok = step["token"].strip()
        if tok.startswith("{"):
            return True
        if not tok.startswith("["):
            continue
        if tok != "[":
            return True
        return any(not _is_ws(s["token"]) for s in content[i + 1:])
    return False


def compare_decisions(new: Sequence[dict], old: Sequence[dict],
                      gate: float = 0.5) -> dict:
    """3.58 criterion S: the YES/NO decision per frame in two records
    files, matched by walk/frame. A frame either side could not answer
    (`score` None) counts as a mismatch -- it is not the same answer."""
    def key(r):
        return f"{r['walk']}/{r['frame']}"

    def yes(r):
        return r["score"] is not None and r["score"] >= gate

    before = {key(r): r for r in old}
    flips, missing, diffs = [], [], []
    for r in new:
        o = before.get(key(r))
        if o is None:
            missing.append(key(r))
            continue
        if r["score"] is None or o["score"] is None or yes(r) != yes(o):
            flips.append(key(r))
        else:
            diffs.append(abs(r["score"] - o["score"]))
    return {"compared": len(new) - len(missing), "flips": flips,
            "missing": missing, "max_score_diff": max(diffs) if diffs else None}


def decision_ms(timings: dict, decided_at) -> Optional[float]:
    """Prompt (image included) plus generation up to and including the
    deciding token, from llama-server's `timings`. Reported beside, never
    as, criterion 5 (amendment 2)."""
    if decided_at is None or not timings or not timings.get("predicted_n"):
        return None
    per = timings["predicted_ms"] / timings["predicted_n"]
    return timings["prompt_ms"] + per * (decided_at + 1)


def percentile(values: Sequence[float], q: float) -> float:
    """Nearest-rank percentile (q in 0..100), the convention the Jetson
    benches already report."""
    s = sorted(values)
    if not s:
        return float("nan")
    k = max(0, min(len(s) - 1, math.ceil(q / 100 * len(s)) - 1))
    return s[k]


def mem_available_mb() -> float:
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024
    raise RuntimeError("no MemAvailable in /proc/meminfo")


class MemSampler(threading.Thread):  # pragma: no cover -- needs /proc
    def __init__(self, period_s: float = 1.0):
        super().__init__(daemon=True)
        self.period_s, self.low, self._stop = period_s, None, threading.Event()

    def run(self):
        while not self._stop.is_set():
            v = mem_available_mb()
            self.low = v if self.low is None else min(self.low, v)
            self._stop.wait(self.period_s)

    def stop(self):
        self._stop.set()


def _post(url: str, body: dict, timeout: float = 120.0) -> dict:  # pragma: no cover
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={"content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def ask(base: str, jpeg: bytes, target: str) -> dict:  # pragma: no cover
    body = {
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {
                "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}},
            {"type": "text", "text": PROMPT.format(target=target)}]}],
        "max_tokens": MAX_TOKENS, "temperature": 0, "top_k": 1,
        "logprobs": True, "top_logprobs": TOP_LOGPROBS,
    }
    return _post(base + "/v1/chat/completions", body)


def stream_chunks(lines):
    """`data:` lines of a streamed chat completion -> the per-token logprob
    entries, in order, as they arrive. Ends at `[DONE]`."""
    for raw in lines:
        line = raw.decode() if isinstance(raw, bytes) else raw
        line = line.strip()
        if not line.startswith("data:"):
            continue
        data = line[5:].strip()
        if data == "[DONE]":
            return
        chunk = json.loads(data)
        choice = (chunk.get("choices") or [{}])[0]
        for entry in (choice.get("logprobs") or {}).get("content") or []:
            yield entry


def ask_until_decided(base: str, jpeg_path, target: str) -> dict:  # pragma: no cover
    """One frame under the stopping rule. Times from before the image is
    read and encoded to the arrival of the deciding token (or the end of
    the reply, if it never decides), then closes the connection."""
    t0 = time.perf_counter()
    jpeg = Path(jpeg_path).read_bytes()
    body = {
        "stream": True,
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {
                "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}},
            {"type": "text", "text": PROMPT.format(target=target)}]}],
        "max_tokens": MAX_TOKENS, "temperature": 0, "top_k": 1,
        "logprobs": True, "top_logprobs": TOP_LOGPROBS,
    }
    req = urllib.request.Request(base + "/v1/chat/completions",
                                 data=json.dumps(body).encode(),
                                 headers={"content-type": "application/json"})
    content, decided = [], False
    with urllib.request.urlopen(req, timeout=120.0) as r:
        for entry in stream_chunks(r):
            content.append(entry)
            if decision_reached(content):
                decided = True
                break
    return {"content": content, "decided": decided,
            "ms": (time.perf_counter() - t0) * 1000,
            "reply": "".join(e["token"] for e in content)}


def wait_ready(base: str, proc, timeout_s: float = 300.0):  # pragma: no cover
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if proc.poll() is not None:
            raise RuntimeError(f"llama-server exited with {proc.returncode}")
        try:
            with urllib.request.urlopen(base + "/health", timeout=5) as r:
                if r.status == 200:
                    return
        except (urllib.error.URLError, ConnectionError, OSError):
            pass
        time.sleep(1)
    raise RuntimeError("llama-server never became ready")


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover
    from control.perception_eval import load_corpus

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--server", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--mmproj", required=True)
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--walk", action="append", required=True)
    ap.add_argument("--port", type=int, default=9481)
    ap.add_argument("--ctx", type=int, default=4096)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stop-at-decision", action="store_true",
                    help="3.58: stream, and stop at the deciding token")
    args = ap.parse_args(argv)

    walks = load_corpus(Path(args.recordings), only=args.walk)
    frames = [(w, p, vis, adj) for w in walks for p, vis, adj in w.frames]
    base = f"http://127.0.0.1:{args.port}"
    before = mem_available_mb()
    sampler = MemSampler()
    sampler.start()
    # Appended, not with_suffix: `identity-qwen2.5-3b` would lose `.5-3b`.
    log = open(f"{args.out}.server.log", "w")
    proc = subprocess.Popen(
        [str(Path(args.server).expanduser()), "-m", str(Path(args.model).expanduser()),
         "--mmproj", str(Path(args.mmproj).expanduser()), "-ngl", "99",
         "-c", str(args.ctx), "--port", str(args.port), "--host", "127.0.0.1",
         "-np", "1", "--no-webui",
         # Amendment 3: no host-RAM prompt cache. Its 8 GiB default caches
         # every frame's image prompt -- never reused, since no two frames
         # match -- in the Jetson's unified memory until the OOM killer
         # takes the server (it did, on four of five first runs).
         "--cache-ram", "0"], stdout=log, stderr=subprocess.STDOUT)
    records, ms, noparse, replies, to_decision = [], [], 0, {}, []
    errors, object_replies = 0, 0
    try:
        t0 = time.monotonic()
        wait_ready(base, proc)
        load_s = time.monotonic() - t0
        for w, p, _, _ in frames[:WARMUP]:
            if args.stop_at_decision:
                ask_until_decided(base, p, w.target)
            else:
                ask(base, p.read_bytes(), w.target)
        for w, p, vis, adj in frames:
            t = time.perf_counter()
            try:
                if args.stop_at_decision:
                    got = ask_until_decided(base, p, w.target)
                    out = {"choices": [{"logprobs": {"content": got["content"]},
                                        "message": {"content": got["reply"]}}]}
                else:
                    out = ask(base, p.read_bytes(), w.target)
            except (urllib.error.URLError, OSError, ValueError) as e:
                # One failed call is one unavailable frame -- a miss if the
                # target was visible (score_at's rule) -- not a lost run.
                errors += 1
                records.append({"walk": w.name, "frame": p.name, "visible": vis,
                                "status": "unavailable", "score": None,
                                "metric": "confidence", "label": None, "candidates": 0,
                                "adjudicated": adj, "ms": 0.0})
                print(f"{w.name}/{p.name} ERROR {e}", flush=True)
                continue
            # Under the stopping rule `ms` is the client's time to the
            # deciding token (image encoding included), not the whole call.
            took = (got["ms"] if args.stop_at_decision
                    else (time.perf_counter() - t) * 1000)
            choice = out["choices"][0]
            content = (choice.get("logprobs") or {}).get("content") or []
            score, parsed, decided = p_localised(content)
            noparse += 0 if parsed else 1
            if parsed and content[decided]["token"].strip().startswith("{"):
                object_replies += 1
            dm = decision_ms(out.get("timings") or {}, decided)
            if dm is not None:
                to_decision.append(dm)
            replies[f"{w.name}/{p.name}"] = (choice["message"].get("content") or "")[:160]
            ms.append(took)
            records.append({"walk": w.name, "frame": p.name, "visible": vis,
                            "status": "detected" if score >= 0.5 else "absent",
                            "score": score, "metric": "confidence", "label": None,
                            "candidates": 1, "adjudicated": adj, "ms": took})
            print(f"{w.name}/{p.name} vis={vis} p={score:.3f} {took:.0f}ms", flush=True)
    finally:
        sampler.stop()
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
    commit = subprocess.run(["git", "-C", str(Path(args.server).expanduser().parents[2]),
                             "log", "-1", "--format=%h"], capture_output=True,
                            text=True).stdout.strip()
    doc = {
        "config": {"detector": f"llama.cpp:{Path(args.model).name}",
                   "mmproj": Path(args.mmproj).name, "llama_cpp": commit,
                   "device": "jetson-orin-nano-super-15w", "runtime": "llama-server CUDA",
                   "prompt": PROMPT, "max_tokens": MAX_TOKENS,
                   "top_logprobs": TOP_LOGPROBS, "metric": "confidence",
                   "cache_ram_mib": 0, "stop_at_decision": bool(args.stop_at_decision)},
        "records": records,
        "latency_ms": {"n": len(ms), "p50": statistics.median(ms),
                       "p90": percentile(ms, 90), "max": max(ms)},
        "to_decision_ms": ({"n": len(to_decision), "p50": statistics.median(to_decision),
                            "p90": percentile(to_decision, 90)} if to_decision else None),
        "memory_mb": {"available_before": before, "available_low": sampler.low,
                      "drop": before - (sampler.low if sampler.low is not None else before)},
        "load_s": load_s, "noparse": noparse, "object_replies": object_replies,
        "errors": errors, "replies": replies,
    }
    Path(args.out).write_text(json.dumps(doc, indent=1))
    print(json.dumps({k: doc[k] for k in ("latency_ms", "to_decision_ms", "memory_mb",
                                          "noparse", "object_replies", "errors")}))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
