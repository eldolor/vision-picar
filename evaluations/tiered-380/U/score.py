"""
evaluations/tiered-380/U/score.py

3.80.U: scores the top five's runs on another frame set with the scorers the
phone-frame rows used, so the rows compare within the set (never across sets,
P24's rule for frame sets):

* identity: `control.identity_eval` on the identity walks, gate 0.5;
* steering, the GGUF rows: `M/decide_scale.py`'s box scale, then 3.80.I's
  `accuracy` on the box's third; VE and Florence: `S/score.py`'s `score_res`
  (YOLOE + CLIP alone, the second model alone, the gate);
* memory: `MemAvailable` before the run against its lowest in each phase.

Reads `<dir>/` (default `U/board/out/<tag>`), the board's `~/out-380U` and
`~/out-380S/<tag>-*` copied back. Refuses while S1 has an unlabelled frame.

    PICAR_380_SETS=evaluations/tiered-380/U/rover PICAR_380_RECORDINGS=recordings \\
        .venv/bin/python evaluations/tiered-380/U/score.py --walk <w> [--walk <w> ...] [--tag rover]
"""

import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.resolve().parents[2]
sys.path.insert(0, str(HERE.parent / "I"))
sys.path.insert(0, str(HERE.parent / "M"))
sys.path.insert(0, str(ROOT))
import common                                               # noqa: E402
import decide_scale                                         # noqa: E402
from brain.tiered import percentiles                        # noqa: E402

MODELS = ("ve", "bonsai", "minicpm", "ivl4b", "florence")
GATED = ("ve", "florence")


def _load_s():
    spec = importlib.util.spec_from_file_location("score_380s", HERE.parent / "S" / "score.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def memory(d: Path, name: str) -> dict | None:
    """Before the run, and the lowest in each phase (`<name>.marks`)."""
    mem, marks = d / f"{name}.mem", d / f"{name}.marks"
    if not mem.exists():
        return None
    samples = [(int(a), int(b)) for a, b in (line.split() for line in mem.read_text().splitlines()
                                              if len(line.split()) == 2) if b.isdigit()]
    if not samples:
        return None
    before = samples[0][1]
    out = {"before": before, "min": min(v for _, v in samples)}
    out["drop"] = before - out["min"]
    if marks.exists():
        m = [(int(t), label) for t, label in (line.split() for line in marks.read_text().splitlines())]
        for (t0, label), nxt in zip(m, m[1:] + [(10 ** 12, "")]):
            vals = [v for t, v in samples if t0 <= t < nxt[0]]
            if label != "end" and vals:
                out[label] = {"min": min(vals), "drop": before - min(vals)}
    return out


def latency(path: Path) -> dict | None:
    if not path.exists():
        return None
    d = json.loads(path.read_text())
    return {"reply": d.get("latency_ms"), "to_decision": d.get("to_decision_ms")}


def steering_gguf(d: Path, name: str) -> dict | None:
    if not (d / f"{name}.jsonl").exists():
        return None
    decide_scale.main(name, str(d))
    rows = [json.loads(line) for line in (d / f"{name}.scaled.jsonl").read_text().splitlines() if line.strip()]
    s1 = [r for r in rows if r["set"] == "S1"]
    s2 = [r for r in rows if r["set"] == "S2"]
    import score as I                                       # noqa: E402  (after the sets are checked)
    return {"S1": I.accuracy(s1, "box_answer"), "S1_mirror": I.mirror(s1, "box_answer"),
            "S2_n": len(s2), "S2_boxes": sum(r.get("box_answer") is not None for r in s2),
            "box_ms": percentiles([r["id_rtt_ms"] for r in rows if r.get("id_rtt_ms") is not None])}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--walk", action="append", required=True)
    ap.add_argument("--tag", default="rover")
    ap.add_argument("--dir", default=None)
    ap.add_argument("--recordings", default=str(common.RECORDINGS))
    a = ap.parse_args(argv)
    d = Path(a.dir) if a.dir else HERE / "board" / "out" / a.tag
    unlabelled = [r for r in common.labels_s1() if r.get("truth") is None]
    if unlabelled:
        sys.exit(f"{len(unlabelled)} S1 frames have no truth yet: label them by viewing first")

    records = [f"{m}={d / f'{a.tag}-{m}-A.json'}:0.5" for m in MODELS
               if (d / f"{a.tag}-{m}-A.json").exists()]
    if records:
        cmd = [sys.executable, "-m", "control.identity_eval", "--recordings", a.recordings,
               *[x for w in a.walk for x in ("--walk", w)], *[x for r in records for x in ("--records", r)],
               "--json", str(d / "identity.json")]
        subprocess.run(cmd, cwd=ROOT, check=True)

    S = _load_s()
    S.OUT = d
    out = {}
    for m in MODELS:
        name = f"{a.tag}-{m}"
        row = {"memory": memory(d, name), "latency": latency(d / f"{name}-A.json")}
        if m in GATED:
            gate = f"{name}-gate"                           # S/board/run.sh's own rows and memory
            row["steering"] = S.score_res(gate) if (d / f"{gate}.jsonl").exists() else None
        else:
            row["steering"] = steering_gguf(d, name)
        out[m] = row
    (d / "scores.json").write_text(json.dumps(out, indent=1) + "\n")
    for m, row in out.items():
        st = row["steering"] or {}
        if m in GATED and st:
            s1 = {p: st[p]["S1"]["right"] for p in ("A", "F", "G")}
            s1n = st["A"]["S1"]["n"]
            s2 = {p: st[p]["S2_answered"] for p in ("A", "F", "G")}
            print(f"{m:9s} S1 right of {s1n}: perception {s1['A']}, {m} {s1['F']}, gate {s1['G']} | "
                  f"S2 answered: {s2} | with perception {st['memory']}")
        elif st:
            print(f"{m:9s} S1 right {st['S1']['right']}/{st['S1']['n']} | S2 boxes {st['S2_boxes']}/{st['S2_n']}")
        print(f"{'':9s} memory {row['memory']} | latency {row['latency']}")


if __name__ == "__main__":
    main()
