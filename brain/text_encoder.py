"""Text features for perception's prompts, encoded outside its process (3.81).

Perception needs two text encodings per target: YOLOE's prompt (MobileCLIP's
text tower) and CLIP's target and distractor vectors (CLIP RN50's text
tower). Both towers used to live in perception's process:

* **T1, YOLOE.** Ultralytics builds the whole of `mobileclip_blt.ts` (both
  towers, 572 MB fp32) in the calling process, uses it once and drops it,
  and the process keeps about 0.45-0.6 GB of heap afterwards (3.81.A:
  TorchScript holds what it loaded).
* **T2, CLIP.** RN50's text tower sat on the GPU for the whole run (243
  MB) to encode a handful of strings once per target.

So the texts are encoded here instead:

* **a cache on disk**, one JSON file per (model, texts): a target seen
  before costs nothing;
* **on a miss, a short-lived child process**, CPU only, that loads the
  text tower, encodes, prints the features and exits, so its memory goes
  back to the board when it ends. It costs seconds on a new target
  (3.81 T1 bar 3, failed and kept by the user, 2026-10-14).

The features are the encoders' normalised outputs, exactly as before: on
the board MobileCLIP's are bit-identical (YOLOE's model is on the CPU when
its prompt is set), and CLIP's CPU features differ from its GPU ones by at
most 3.7e-7.

  python -m brain.text_encoder yoloe <mobileclip file> <text>
  python -m brain.text_encoder clip <model> <pretrained> <text>...
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Callable, Sequence

from brain.perceive import PerceptionUnavailable

log = logging.getLogger(__name__)

# The text model YOLOE's checkpoints name, and the one file this child
# knows how to cut; ultralytics' `build_text_model` maps it the same way.
YOLOE_TEXT_MODEL = "mobileclip:blt"
MOBILECLIP = "mobileclip_blt.ts"
# Under the brain's 30 s tick guard (B3.3) with room for the frame itself;
# a miss takes ~7 s on the Jetson, the one-time tower cut longer.
CHILD_TIMEOUT_S = 20.0
# A failed child is not run again for the same texts for this long, so a
# broken encoder costs one timeout, not one per frame.
RETRY_AFTER_S = 60.0
_failed: dict = {}
REPO = Path(__file__).resolve().parent.parent

Rows = list  # list[list[float]], one row per text

Runner = Callable[[Sequence[str]], Rows]


def cache_dir() -> Path:
    return Path(os.environ.get("PICAR_TEXT_CACHE")
                or Path.home() / ".cache" / "vision-picar" / "text-features")


def tower_path(source: str) -> Path:
    """The text-tower-only file kept beside the full MobileCLIP file, named
    with that file's size so a replaced checkpoint gets its own cut."""
    p = Path(source)
    size = p.stat().st_size if p.exists() else 0
    return p.with_name(f"{p.stem}_text_{size}{p.suffix}")


def cache_key(model_id: str, texts: Sequence[str]) -> str:
    return hashlib.sha256("\0".join([model_id, *texts]).encode()).hexdigest()


def run_child(args: Sequence[str]) -> Rows:
    key = tuple(args)
    if time.monotonic() - _failed.get(key, -RETRY_AFTER_S) < RETRY_AFTER_S:
        raise PerceptionUnavailable("text encoder failed recently; not retried yet")
    try:
        return _run(args)
    except PerceptionUnavailable:
        _failed[key] = time.monotonic()
        raise


def _run(args: Sequence[str]) -> Rows:
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="")  # no CUDA context in the child
    try:
        out = subprocess.run([sys.executable, "-m", "brain.text_encoder", *args],
                             capture_output=True, text=True, env=env, cwd=REPO,
                             timeout=CHILD_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PerceptionUnavailable(f"text encoder did not run: {exc}") from exc
    if out.returncode != 0:
        raise PerceptionUnavailable(
            f"text encoder failed ({out.returncode}): {out.stderr.strip()[-500:]}")
    try:
        return json.loads(out.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError) as exc:
        raise PerceptionUnavailable(f"text encoder printed no features: {exc}") from exc


def _valid(rows, n: int) -> Rows:
    ok = (isinstance(rows, list) and len(rows) == n
          and all(isinstance(r, list) and r and len(r) == len(rows[0])
                  and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in r)
                  for r in rows))
    if not ok:
        raise ValueError(f"not {n} rows of numbers")
    return [[float(v) for v in r] for r in rows]


def features(model_id: str, texts: Sequence[str], args: Sequence[str], *,
             runner: Runner = run_child) -> Rows:
    """One feature row per text, from the cache or a child given `args`."""
    texts = list(texts)
    path = cache_dir() / f"{cache_key(model_id, texts)}.json"
    try:
        rec = json.loads(path.read_text())
        if rec.get("model") == model_id and rec.get("texts") == texts:
            return _valid(rec["features"], len(texts))
    except FileNotFoundError:
        pass
    except (OSError, ValueError, KeyError, AttributeError) as exc:
        log.warning("text cache %s unreadable (%s); encoding again", path, exc)
    try:
        rows = _valid(runner(args), len(texts))
    except ValueError as exc:
        raise PerceptionUnavailable(f"text encoder: {exc}") from exc
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            json.dump({"model": model_id, "texts": texts, "features": rows}, f)
        os.replace(tmp, path)
    except OSError as exc:  # a read-only home still perceives, it just re-encodes
        log.warning("text cache not written (%s)", exc)
    return rows


def yoloe_features(source: str, text: str, *, runner: Runner = run_child) -> list[float]:
    """MobileCLIP's normalised features for YOLOE's prompt. `source` is the
    full MobileCLIP file, resolved (absolute); its name and size stand in
    for a hash of 572 MB in the cache key."""
    p = Path(source)
    model_id = f"{YOLOE_TEXT_MODEL}:{p.name}:{p.stat().st_size}"
    return features(model_id, [text], ["yoloe", str(p), text], runner=runner)[0]


def clip_features(model: str, pretrained: str, texts: Sequence[str], *,
                  runner: Runner = run_child) -> Rows:
    """open_clip's normalised text features, one row per text, encoded as
    one batch as `ClipScorer` always did."""
    return features(f"open_clip:{model}:{pretrained}", texts,
                    ["clip", model, pretrained, *texts], runner=runner)


def _yoloe(source: str, text: str):  # pragma: no cover - needs torch and the weights
    import torch
    import clip  # ultralytics' own tokenizer for MobileCLIPTS

    tower = tower_path(source)
    if not tower.exists():
        # Once per board: cut the text tower out of the full file. This one
        # load is the heavy one (both towers, ~0.9 GB on CPU).
        full = torch.jit.load(source, map_location="cpu")
        fd, tmp = tempfile.mkstemp(dir=tower.parent, suffix=".tmp")  # one per process
        os.close(fd)
        torch.jit.save(full.text_encoder, tmp)
        os.replace(tmp, tower)
        del full
    enc = torch.jit.load(str(tower), map_location="cpu")
    with torch.inference_mode():
        # truncate=True: what ultralytics' MobileCLIPTS.tokenize passes
        f = enc(clip.clip.tokenize([text], truncate=True))
        # the full model's forward, after its text tower, verbatim
        f = torch.div(f, torch.clamp_min(
            torch.linalg.vector_norm(f, 2.0, [-1], True), 1e-12).expand_as(f))
    return f


def _clip(model: str, pretrained: str, texts: list[str]):  # pragma: no cover
    import open_clip
    import torch

    m = open_clip.create_model(model, pretrained=pretrained, device="cpu")
    m.eval()
    with torch.no_grad():
        f = m.encode_text(open_clip.get_tokenizer(model)(texts))
        f /= f.norm(dim=-1, keepdim=True)  # ClipScorer's own normalisation
    return f


def main(argv: list[str]) -> int:  # pragma: no cover - needs torch and the weights
    kind, *rest = argv
    if kind == "yoloe":
        f = _yoloe(*rest)
    elif kind == "clip":
        f = _clip(rest[0], rest[1], rest[2:])
    else:
        raise SystemExit(f"unknown encoder {kind}")
    print(json.dumps(f.float().tolist()))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
