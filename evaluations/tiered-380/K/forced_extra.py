"""3.80.K: the forced token ids for RoboBrain's reduced vocabulary, on the
export host: 3.78's list (forced_ids.json; RoboBrain's vocab.json is
byte-identical to Qwen2.5-VL-3B's) plus every tokenisation of the direction
words and every token in RoboBrain's 3.80.J replies.

    python forced_extra.py <model-dir> forced_ids.json robobrain-q4.jsonl forced.json
"""
import json
import sys

from transformers import AutoTokenizer

model, base, replies, out = sys.argv[1:5]
tok = AutoTokenizer.from_pretrained(model)
ids = set(json.load(open(base)))
words = ["left", "right", "forward", "Left", "Right", "Forward", "LEFT", "RIGHT", "FORWARD",
         "Move", "move", "Move:", "turn", "Turn"]
for w in words:
    for v in (w, " " + w, w + ".", " " + w + "."):
        ids.update(tok.encode(v, add_special_tokens=False))
for line in open(replies):
    r = json.loads(line)
    for a in (r, r["mirror"]):
        ids.update(tok.encode(a.get("id_text") or "", add_special_tokens=False))
json.dump(sorted(ids), open(out, "w"))
print(len(ids), "forced ids")
