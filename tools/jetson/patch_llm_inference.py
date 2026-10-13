"""3.78 amendment 1: patch Edge-LLM v0.6.0's examples/llm/llm_inference.cpp
so each response carries its wall time, its token ids, and honours a
per-request max_generate_length. Idempotent."""
import sys
from pathlib import Path

p = Path(sys.argv[1])
s = p.read_text()
if "PICAR_378" in s:
    print("already patched")
    sys.exit(0)


def sub(old, new):
    global s
    assert s.count(old) == 1, f"anchor not unique: {old[:60]!r} x{s.count(old)}"
    s = s.replace(old, new)


sub('#include "profileFormatter.h"',
    '#include "profileFormatter.h"\n#include <chrono> // PICAR_378')

# per-request max_generate_length (one request per batch in our runs)
sub('''                bool disableSpecDecode = requestItem.value("disable_spec_decode", false);''',
    '''                if (requestItem.contains("max_generate_length")) // PICAR_378
                {
                    batchRequest.maxGenerateLength = requestItem["max_generate_length"].get<int64_t>();
                }
                bool disableSpecDecode = requestItem.value("disable_spec_decode", false);''')

# wall time around the real (non-warm-up) request
sub('''        bool requestStatus = false;
        if (args.eagleArgs.enabled)
        {
            requestStatus = eagleInferenceRuntime->handleRequest(request, response, stream);
        }
        else
        {
            requestStatus = llmInferenceRuntime->handleRequest(request, response, stream);
        }
''', '''        bool requestStatus = false;
        cudaStreamSynchronize(stream);
        auto picarT0 = std::chrono::steady_clock::now(); // PICAR_378
        if (args.eagleArgs.enabled)
        {
            requestStatus = eagleInferenceRuntime->handleRequest(request, response, stream);
        }
        else
        {
            requestStatus = llmInferenceRuntime->handleRequest(request, response, stream);
        }
        cudaStreamSynchronize(stream);
        double picarMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - picarT0).count();
''')

sub('''            responseJson["request_idx"] = requestIdx;''',
    '''            responseJson["request_idx"] = requestIdx;
            responseJson["ms"] = picarMs; // PICAR_378
            if (requestStatus && batchIdx < response.outputIds.size())
            {
                responseJson["output_ids"] = response.outputIds[batchIdx];
            }''')

p.write_text(s)
print("patched")
