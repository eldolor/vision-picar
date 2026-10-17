#!/bin/bash
# 3.80.M: fetch each file with curl (resumable) and check its SHA256.
cd ~/models-380M
while read repo fn sha; do
  if [ -f "$fn" ] && [ "$(sha256sum "$fn" | cut -d' ' -f1)" = "$sha" ]; then echo "$fn OK (had)"; continue; fi
  for try in 1 2 3 4 5; do
    curl -sSL -C - --retry 5 --speed-time 60 --speed-limit 10000 -o "$fn" "https://huggingface.co/$repo/resolve/main/$fn" && break
    echo "$fn retry $try"
  done
  got=$(sha256sum "$fn" | cut -d' ' -f1)
  [ "$got" = "$sha" ] && echo "$fn OK $got" || echo "$fn MISMATCH $got"
done <<'L'
bartowski/OpenGVLab_InternVL3_5-2B-GGUF mmproj-OpenGVLab_InternVL3_5-2B-f16.gguf e83ba6e675b747f7801557dc24594f43c17a7850b6129d4972d55e3e9b010359
bartowski/OpenGVLab_InternVL3_5-4B-GGUF OpenGVLab_InternVL3_5-4B-Q4_K_M.gguf 7c1612b6896ad14caa501238e72afa17a600651d0984225e3ff78b39de86099c
bartowski/OpenGVLab_InternVL3_5-4B-GGUF mmproj-OpenGVLab_InternVL3_5-4B-f16.gguf 0f9704972fcb9cb0a4f2c0f4eb7fe4f58e53ccd4b06ec17cf7a80271aa963eb7
openbmb/MiniCPM-V-4-gguf ggml-model-Q4_K_M.gguf b0ff610e9c92b30389ff1e0dd40fffed3c1f02a9d34a735fd5fba6a5ad25672b
openbmb/MiniCPM-V-4-gguf mmproj-model-f16.gguf f0faa9ae63532300999c86a196f140c716cd0fbb08bbbd81850f1f9a631f7761
tencent/Youtu-VL-4B-Instruct-GGUF mmproj-Youtu-VL-4b-Instruct-BF16.gguf 1bcb2b7a99687be9a47e9bf27ee96237b73fe94b6fda938b17b18e7d4f92f9f2
tencent/Youtu-VL-4B-Instruct-GGUF Youtu-VL-4B-Instruct-F16.gguf 31478432a7b1797899b16261594b3447c9a7ba6e17768bbdcba4e7060a660af5
L
echo ALL DONE
