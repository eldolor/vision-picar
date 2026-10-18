"""
evaluations/tiered-380/R/native_server.py

3.80.R: a model that is not run by llama-server (torch, or Moondream's
Photon), behind the slice of llama-server's API that the benches use, so
`tools.jetson.bench_identity_llama` and `M/run_steer.py` ask it exactly what
they ask a GGUF:
* `GET /health`;
* `POST /v1/chat/completions`: one image (a `data:` URL) and one text, greedy,
  `max_tokens`; `logprobs` / `top_logprobs` as llama-server returns them (each
  step's chosen token and its top 20 alternatives, read off `generate`'s
  scores); `stream: true` answers with the same entries as SSE chunks once
  the reply is complete (so a streamed call's time is the whole reply).

A backend with a native grounding call (Florence-2, Photon) is asked through
it when the text is that model's native prompt (`prompts.json`); its reply
is written back as `[{"bbox_2d": [...]}]` in the frame's pixels (`[]` when it
names nothing), and its logprobs are ONE synthetic step -- `[{` if it named a
box or a point, `[]` if not -- so the benches' readers score it 1 or 0: the
plan's binary score.

The bench starts a "server" with llama-server's argv; `nsrv.sh` passes it
here, and only `-m` (`<backend>:<model>`) and `--port` are read.

    python evaluations/tiered-380/R/native_server.py -m florence:microsoft/Florence-2-base --port 9481
"""

import argparse
import base64
import io
import json
import math
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
NATIVE = json.loads((HERE / "prompts.json").read_text())["native"]
TOP = 20


def _template_re(template: str) -> re.Pattern:
    return re.compile("^" + re.escape(template).replace(re.escape("{target}"), "(.+?)") + "$", re.S)


def native_target(backend: str, text: str):
    """`(skill, target)` if `text` is one of this backend's native prompts."""
    for skill, template in (NATIVE.get(backend) or {}).items():
        m = _template_re(template).match(text.strip())
        if m:
            return skill, m.group(1)
    return None


def boxes_reply(boxes):
    """A native answer as the benches read it: the text and one synthetic step."""
    text = json.dumps([{"bbox_2d": [round(v, 1) for v in b]} for b in boxes])
    tok = "[{" if boxes else "[]"
    step = {"token": tok, "logprob": 0.0, "top_logprobs": [{"token": tok, "logprob": 0.0}]}
    return text, [step]


def word_reply(word: str):
    """A free-text answer with no scores (Photon's query): one step per word."""
    steps = [{"token": w, "logprob": 0.0, "top_logprobs": [{"token": w, "logprob": 0.0}]}
             for w in re.findall(r"\S+|\s+", word)]
    return word, steps


class HFGenerate:
    """Shared by the transformers backends: greedy `generate` with scores, the
    llama-server logprobs shape built from them."""

    def steps(self, tok, out, n_in):
        import torch
        # The last len(scores) ids are the generated ones, whether or not the
        # model returns its inputs too (models that embed them do not).
        seq = out.sequences[0][-len(out.scores):] if out.scores else out.sequences[0][:0]
        content = []
        for i, logits in enumerate(out.scores):
            if i >= len(seq):
                break
            lp = torch.log_softmax(logits[0].float(), -1)
            top = torch.topk(lp, TOP)
            chosen = int(seq[i])
            content.append({
                "token": tok.decode([chosen]), "logprob": float(lp[chosen]),
                "top_logprobs": [{"token": tok.decode([int(j)]), "logprob": float(v)}
                                 for v, j in zip(top.values, top.indices)]})
        return tok.decode(seq, skip_special_tokens=True), content


class Florence(HFGenerate):
    def __init__(self, model_id, device):
        import torch
        from transformers import AutoModelForCausalLM, AutoProcessor
        self.device, self.dtype = device, torch.float16
        kw = {"torch_dtype": self.dtype}
        try:
            from transformers import Florence2ForConditionalGeneration as M
        except ImportError:
            M, kw = AutoModelForCausalLM, {**kw, "trust_remote_code": True}
        self.model = M.from_pretrained(model_id, **kw).to(device).eval()
        self.proc = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)

    def answer(self, image, text, max_tokens):
        import torch
        hit = native_target("florence", text)
        if hit is None:
            return "", []  # no free-text arm: an unparsed reply
        task = "<CAPTION_TO_PHRASE_GROUNDING>"
        inputs = self.proc(text=text.strip(), images=image, return_tensors="pt").to(self.device)
        inputs["pixel_values"] = inputs["pixel_values"].to(self.dtype)
        with torch.inference_mode():
            ids = self.model.generate(**inputs, max_new_tokens=max(max_tokens, 64),
                                      do_sample=False, num_beams=1)
        raw = self.proc.batch_decode(ids, skip_special_tokens=False)[0]
        got = self.proc.post_process_generation(raw, task=task, image_size=image.size)[task]
        return boxes_reply(got.get("bboxes") or [])


class ChatHF(HFGenerate):
    """A transformers chat VLM taking one image and one text through its
    processor's chat template (Eagle2, Molmo2, FastVLM, NVILA-family as each
    allows)."""

    def __init__(self, model_id, device, name):
        import torch
        from transformers import AutoModel, AutoModelForCausalLM, AutoModelForImageTextToText, AutoProcessor
        self.name, self.device = name, device
        dtype = torch.float16 if device != "cpu" else torch.float32
        self.proc = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        last = None
        for M in (AutoModelForImageTextToText, AutoModelForCausalLM, AutoModel):
            try:
                self.model = M.from_pretrained(model_id, torch_dtype=dtype, trust_remote_code=True)
                break
            except (ValueError, KeyError, TypeError) as e:
                last = e
        else:
            raise last
        self.model = self.model.to(device).eval()
        self.tok = getattr(self.proc, "tokenizer", self.proc)

    def answer(self, image, text, max_tokens):
        import torch
        msgs = [{"role": "user", "content": [{"type": "image", "image": image},
                                             {"type": "text", "text": text}]}]
        inputs = self.proc.apply_chat_template(msgs, add_generation_prompt=True, tokenize=True,
                                               return_dict=True, return_tensors="pt")
        inputs = {k: (v.to(self.device) if hasattr(v, "to") else v) for k, v in inputs.items()}
        for k, v in inputs.items():
            if hasattr(v, "is_floating_point") and v.is_floating_point():
                inputs[k] = v.to(self.model.dtype)
        n_in = inputs["input_ids"].shape[1]
        with torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=max_tokens, do_sample=False,
                                      num_beams=1, output_scores=True,
                                      return_dict_in_generate=True)
        return self.steps(self.tok, out, n_in)


class Eagle(ChatHF):
    """nvidia/Eagle2: its model card's recipe (the chat template as text, the
    images through `process_vision_info`, then the processor)."""

    def answer(self, image, text, max_tokens):
        import torch
        msgs = [{"role": "user", "content": [{"type": "image", "image": image},
                                             {"type": "text", "text": text}]}]
        prompt = self.proc.apply_chat_template(msgs, tokenize=False, add_generation_prompt=True)
        images, videos = self.proc.process_vision_info(msgs)
        inputs = self.proc(text=[prompt], images=images, videos=videos, return_tensors="pt",
                           padding=True).to(self.device)
        for k, v in inputs.items():
            if hasattr(v, "is_floating_point") and v.is_floating_point():
                inputs[k] = v.to(self.model.dtype)
        n_in = inputs["input_ids"].shape[1]
        with torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=max_tokens, do_sample=False,
                                      output_scores=True, return_dict_in_generate=True)
        return self.steps(self.tok, out, n_in)


class FastVLM(HFGenerate):
    """apple/FastVLM: its model card's recipe (an `<image>` placeholder spliced
    as IMAGE_TOKEN_INDEX -200, the model's own image processor)."""

    def __init__(self, model_id, device):
        import torch
        from transformers import AutoModelForCausalLM, AutoTokenizer
        self.device = device
        self.tok = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id, torch_dtype=torch.float16, trust_remote_code=True).to(device).eval()

    def answer(self, image, text, max_tokens):
        import torch
        msgs = [{"role": "user", "content": f"<image>\n{text}"}]
        rendered = self.tok.apply_chat_template(msgs, add_generation_prompt=True, tokenize=False)
        pre, post = rendered.split("<image>", 1)
        pre_ids = self.tok(pre, return_tensors="pt", add_special_tokens=False).input_ids
        post_ids = self.tok(post, return_tensors="pt", add_special_tokens=False).input_ids
        ids = torch.cat([pre_ids, torch.tensor([[-200]]), post_ids], 1).to(self.device)
        px = self.model.get_vision_tower().image_processor(images=image, return_tensors="pt")["pixel_values"]
        with torch.inference_mode():
            out = self.model.generate(inputs=ids, attention_mask=torch.ones_like(ids),
                                      images=px.to(self.device, self.model.dtype),
                                      max_new_tokens=max_tokens, do_sample=False,
                                      output_scores=True, return_dict_in_generate=True)
        return self.steps(self.tok, out, 0)


class PaliGemma(HFGenerate):
    """google/paligemma2 (mix): only its native `detect <name>` prompt. The
    reply keeps its `<locNNNN>` tokens (y1, x1, y2, x2 on 0-1023); the
    logprobs are generate's own, so the first token decides as for any
    native reply (a `<loc` token opens a box)."""

    def __init__(self, model_id, device):
        import torch
        from transformers import AutoProcessor, PaliGemmaForConditionalGeneration
        self.device = device
        self.proc = AutoProcessor.from_pretrained(model_id)
        self.model = PaliGemmaForConditionalGeneration.from_pretrained(
            model_id, torch_dtype=torch.float16).to(device).eval()
        self.tok = self.proc.tokenizer

    def answer(self, image, text, max_tokens):
        import torch
        if native_target("paligemma", text) is None:
            return "", []
        inputs = self.proc(text="<image>" + text.strip(), images=image, return_tensors="pt").to(self.device)
        inputs["pixel_values"] = inputs["pixel_values"].to(self.model.dtype)
        with torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=max(max_tokens, 32), do_sample=False,
                                      output_scores=True, return_dict_in_generate=True)
        _t, content = self.steps(self.tok, out, 0)
        seq = out.sequences[0][-len(out.scores):]
        return self.tok.decode(seq, skip_special_tokens=False), content


class Photon:
    """Moondream through its Photon local engine: `detect` and `point` for the
    native prompts, `query` for any other text (no scores: binary / words)."""

    def __init__(self, model_id, device):
        import moondream as md
        self.model = md.vl(model=model_id, local=True)

    def answer(self, image, text, max_tokens):
        hit = native_target("photon", text)
        if hit is None:
            return word_reply(self.model.query(image, text)["answer"])
        skill, target = hit
        w, h = image.size
        if skill == "detect":
            objs = self.model.detect(image, target)["objects"]
            return boxes_reply([[o["x_min"] * w, o["y_min"] * h, o["x_max"] * w, o["y_max"] * h]
                                for o in objs])
        pts = self.model.point(image, target)["points"]
        return boxes_reply([[p["x"] * w, p["y"] * h, p["x"] * w + 1, p["y"] * h + 1] for p in pts])


class RoboRefer(HFGenerate):
    """Zhoues/RoboRefer-2B-SFT through its own VILA code (`llava`, from
    `$ROBOREFER_SRC`, default ~/bench-380R/roborefer): `generate_content` as
    its API does, RGB only (no depth), greedy. Its inner `generate` is wrapped
    so the scores come back too and the reply has llama-server's logprobs."""

    def __init__(self, model_id, device):
        import os
        sys.path.insert(0, os.path.expanduser(os.environ.get("ROBOREFER_SRC", "~/bench-380R/roborefer")))
        import llava
        from llava import conversation as clib
        self.model = llava.load(model_id)
        clib.default_conversation = clib.conv_templates["auto"].copy()
        self.tok, self.Image, self._last = self.model.tokenizer, llava.Image, None
        inner = self.model.generate

        def generate(**kw):
            self._last = inner(**kw)
            return self._last.sequences
        self.model.generate = generate

    def answer(self, image, text, max_tokens):
        import tempfile
        gc = self.model.default_generation_config
        gc.update(max_new_tokens=max_tokens, do_sample=False, num_beams=1,
                  output_scores=True, return_dict_in_generate=True)
        with tempfile.NamedTemporaryFile(suffix=".jpg") as f:
            image.save(f.name, quality=95)
            self.model.generate_content([self.Image(f.name), text], generation_config=gc)
        return self.steps(self.tok, self._last, 0)


def build(spec: str, device: str):
    backend, model_id = spec.split(":", 1)
    if backend == "florence":
        return Florence(model_id, device)
    if backend == "fastvlm":
        return FastVLM(model_id, device)
    if backend == "photon":
        return Photon(model_id, device)
    if backend == "chat":
        return ChatHF(model_id, device, model_id)
    if backend == "paligemma":
        return PaliGemma(model_id, device)
    if backend == "eagle":
        return Eagle(model_id, device, model_id)
    if backend == "roborefer":
        return RoboRefer(model_id, device)
    raise SystemExit(f"unknown backend {backend}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", required=True)
    ap.add_argument("--port", type=int, default=9481)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--device", default=None)
    args, _ignored = ap.parse_known_args(argv)

    import torch
    import uvicorn
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse, StreamingResponse
    from PIL import Image

    device = args.device or ("cuda" if torch.cuda.is_available()
                             else "mps" if torch.backends.mps.is_available() else "cpu")
    model = build(args.m, device)
    print(f"3.80.R native_server: {args.m} on {device}", flush=True)
    app = FastAPI()

    @app.get("/health")
    def health():
        return {"status": "ok"}

    @app.post("/v1/chat/completions")
    async def chat(req: Request):
        body = await req.json()
        parts = body["messages"][-1]["content"]
        url = next(p["image_url"]["url"] for p in parts if p["type"] == "image_url")
        text = next(p["text"] for p in parts if p["type"] == "text")
        image = Image.open(io.BytesIO(base64.b64decode(url.split(",", 1)[1]))).convert("RGB")
        t0 = time.perf_counter()
        reply, content = model.answer(image, text, int(body.get("max_tokens", 64)))
        ms = (time.perf_counter() - t0) * 1000
        timings = {"predicted_n": len(content), "total_ms": ms}
        if body.get("stream"):
            def events():
                for e in content:
                    chunk = {"choices": [{"delta": {"content": e["token"]},
                                          "logprobs": {"content": [e]}}]}
                    yield "data: " + json.dumps(chunk) + "\n\n"
                yield "data: [DONE]\n\n"
            return StreamingResponse(events(), media_type="text/event-stream")
        return JSONResponse({"choices": [{"message": {"content": reply},
                                          "logprobs": {"content": content}}],
                             "timings": timings})

    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main(sys.argv[1:])
