#!/bin/bash
# P17 -- OWLv2 on a Hailo-10H, thoroughly.
#
# OWLv2 is the best model this project has measured: 82% at 3 FP (P7),
# against YOLO-World + CLIP's 72% and the shipped YOLO11s + CLIP's 45%.
# It has never been runnable on the chosen part.
#
# The two verdicts against it are BOTH narrower than they look:
#
#   * P6 -- "dies at allocation on 73 layernorm and 38 softmax layers".
#     That is an 8-series dataflow result on DFC 3.34.0. The 10H is a
#     different architecture on a different compiler line, and the 5.4.0
#     Model Zoo ships a ViT-B/16 image encoder for it -- which is exactly
#     OWLv2's tower. So the verdict does not transfer; it has to be re-run.
#   * P14 -- "NetworkXUnfeasible: graph contains a cycle" on 5.4.0. VOID.
#     P15 traced every P14 failure to the environment we assembled around
#     the wheel, not the compiler.
#
# What does NOT change is P7d: INT8 destroys OWLv2. That was properly run
# and it stands, which is why the 16-bit rows below exist -- 16-bit is
# cheaper on the 10H's on-module memory than it was on an 8L, and that is
# an argument for trying it, not a result.
#
# Run inside the vendor's AI Software Suite container (P15), on a GPU box
# so the optimization passes actually execute rather than silently
# dropping to level 0 (P11).
#
# Stages are reported SEPARATELY and never merged: P6's whole finding was
# that OWLv2 translates and quantizes fine and dies at the third stage, so
# a row that only says FAIL throws away the answer.
set -uo pipefail

SHARE=/local/shared_with_docker
OUT="$SHARE/out"
CW="$SHARE/cw"
mkdir -p "$OUT" || { echo "cannot create $OUT" >&2; exit 1; }
REPORT="$OUT/owlv2_10h.tsv"
: > "$REPORT"
printf 'row\thead\tlayout\ttranslate\toptimize\tcompile\thef_mb\tdetail\n' | tee -a "$REPORT"

cd "$CW" || exit 1

run_cell() {
  local row="$1" head="$2" layout="$3" extra="${4:-}"
  local log="$OUT/$row.owlv2.$head.$layout.log"
  local rep="$OUT/$row.report.json"

  echo "=== $row: OWLv2 head=$head layout=$layout extra='${extra}' ==="
  # 3h ceiling per cell. P6's whole 8L sweep was 3.1 hours; a single cell
  # that outlives that is not going to finish, and a wedged allocator on a
  # $1.21/hr box is how $12 went overnight once.
  timeout 10800 python -m tools.hailo.compile_owlv2 \
      --build build/owlv2 --arch hailo10h \
      --opset 17 --head "$head" --layout "$layout" \
      ${extra:+--extra-script "$extra"} \
      --report "$rep" > "$log" 2>&1

  # Read the three stages out of the tool's own report rather than
  # guessing from the exit code -- which is the whole point of it never
  # letting one stage's failure end the run.
  python - "$rep" "$row" "$head" "$layout" "$REPORT" <<'PY'
import json, sys
rep, row, head, layout, report = sys.argv[1:6]
try:
    doc = json.load(open(rep))
except Exception as exc:
    line = f"{row}\t{head}\t{layout}\t?\t?\t?\t-\tno report: {exc}"
else:
    a = (doc.get("attempts") or [{}])[0]
    st = a.get("stages", {})
    def mark(name):
        s = st.get(name)
        if not s: return "-"
        return "ok" if s.get("ok") else "FAIL"
    hef = a.get("hef") or {}
    mb = round(hef.get("bytes", 0) / 1e6, 1) if hef.get("bytes") else "-"
    detail = ""
    for name in ("translate", "optimize", "compile"):
        s = st.get(name) or {}
        if s and not s.get("ok"):
            detail = str(s.get("error", ""))[:180].replace("\t", " ").replace("\n", " ")
            break
    line = (f"{row}\t{head}\t{layout}\t{mark('translate')}\t{mark('optimize')}"
            f"\t{mark('compile')}\t{mb}\t{detail}")
print(line)
open(report, "a").write(line + "\n")
PY
}

# The matrix. `minimal` keeps three ops on the Pi's CPU and is the split
# P6 got furthest with; `full` is the whole tower. uint8 is the layout
# worth having on the robot -- it keeps a per-frame float conversion off
# the Pi -- so both are tried rather than only the easy one.
run_cell O1 minimal uint8
run_cell O2 minimal normalized
run_cell O3 full uint8
# NOTE no clsfold row here: `owlv2_op17_minimal_clsfold.onnx` is outside
# export_owlv2_onnx.py's onnx_name() scheme, so compile_owlv2 cannot
# address it by --head. It is banked (the CLS-token Expand folded to a
# constant, verified an exact identity at max |diff| 0.0) and gets its own
# direct ClientRunner run if the rows above make it worth trying.

# O4 (optimization level 2) is deliberately NOT run here. It is an
# ACCURACY row, the passes it turns on need a GPU, and on a CPU box the
# DFC silently drops to level 0 and reports success -- which is exactly
# what P11 was. It runs on a GPU box, and only if a HEF exists to make it
# worth paying for.
#
# RAM, not GPU, is what this matrix needs: OWLv2 translate peaked at
# 31 GB and was OOM-killed on a 32 GB g5.2xlarge. P6 used a 128 GB
# r6i.4xlarge and that is the box this belongs on.

echo
echo "=== OWLv2 10H matrix complete ==="
column -t -s $'\t' "$REPORT" 2>/dev/null || cat "$REPORT"
