"""3.78 amendment 5: build the reduced vocabulary Edge-LLM exports with.

Edge-LLM's `tensorrt-edgellm-reduce-vocab` picks the most frequent tokens
of a text corpus. The arrival question also needs every token its replies
use. This merges the two: the corpus selection, plus a forced list (special
tokens, JSON structure, digits and whitespace, and every token in every
recorded reply), padded to a multiple of 128 from a larger corpus
selection. Output is the same `vocab_map.safetensors` + `reduced_vocab.json`
the tool writes. Run on the export host:

    python tools/jetson/edgellm_vocab_map.py --base rv16k --pad rv32k \\
        --forced forced_ids.json --vocab-size 151665 --out reduced_vocab
"""
import argparse
import json
from pathlib import Path


def merge(base: list, pad: list, forced: list, multiple: int = 128) -> list:
    """Sorted ids: base | forced, then ids from `pad` (in its order) until
    the size is a multiple of `multiple`."""
    keep = set(base) | set(forced)
    for i in pad:
        if len(keep) % multiple == 0:
            break
        keep.add(i)
    if len(keep) % multiple:
        raise ValueError(f"pad ran out at {len(keep)} ids")
    return sorted(keep)


def main():  # pragma: no cover -- runs on the export host
    import torch
    from safetensors.torch import load_file, save_file
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--pad", required=True)
    ap.add_argument("--forced", required=True)
    ap.add_argument("--vocab-size", type=int, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    base = load_file(f"{a.base}/vocab_map.safetensors")["vocab_map"].tolist()
    pad = load_file(f"{a.pad}/vocab_map.safetensors")["vocab_map"].tolist()
    forced = json.loads(Path(a.forced).read_text())
    ids = merge(base, pad, forced)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    save_file({"vocab_map": torch.tensor(ids, dtype=torch.int32)},
              str(out / "vocab_map.safetensors"))
    (out / "reduced_vocab.json").write_text(json.dumps({
        "vocab_size": a.vocab_size, "reduced_vocab_size": len(ids),
        "method": "input_aware (cnn_dailymail) + forced (3.78 amendment 5)",
        "forced": len(set(forced)), "base": len(base)}, indent=2))
    print(f"{len(ids)} ids ({len(set(forced) - set(base))} forced beyond base)")


if __name__ == "__main__":
    main()
