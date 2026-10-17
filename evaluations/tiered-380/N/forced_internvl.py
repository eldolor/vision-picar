"""3.80.N: forced token ids for InternVL3.5-4B's reduced vocabulary, on the
export host. 3.80.K's list (3.78's ids -- InternVL's base vocabulary is
byte-identical to Qwen2.5-VL's -- the direction words, every token in the
replies file), plus every added token (ids from 151,643 up: `<img>`,
`<IMG_CONTEXT>`, `<box>`, chat markers).

    python forced_internvl.py <model-dir> forced_ids.json ivl4b.jsonl forced.json
"""
import json
import subprocess
import sys

from transformers import AutoTokenizer

model, base, replies, out = sys.argv[1:5]
subprocess.run([sys.executable, "forced_extra.py", model, base, replies, out], check=True)
tok = AutoTokenizer.from_pretrained(model)
ids = set(json.load(open(out)))
ids.update(range(151643, len(tok)))
json.dump(sorted(ids), open(out, "w"))
print(len(ids), "forced ids with the added tokens; tokenizer", len(tok))
