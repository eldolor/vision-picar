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

# Images are decoded inside the timed request, one frame at a time (3.78
# review): stock llm_inference decodes every image while parsing the input
# file, which keeps the decode out of the timing and all 299 frames in
# memory at once. Arm L decodes inside its timed request and holds one.
sub("""                                auto image = rt::imageUtils::loadImageFromFile(msgContent.content);
                                if (image.buffer != nullptr)
                                {
                                    imageBuffers.push_back(std::move(image));
                                }""",
    """                                // PICAR_378: decoded lazily, inside the timed request""")

sub("""    // Perform warmup runs if requested
    if (args.warmup > 0)""",
    """    // PICAR_378: decode a request's images (and only its images) on demand.
    auto picarLoad = [](rt::LLMGenerationRequest& req) {
        for (auto& rq : req.requests)
        {
            rq.imageBuffers.clear();
            for (auto const& m : rq.messages)
            {
                for (auto const& c : m.contents)
                {
                    if (c.type == "image")
                    {
                        auto image = rt::imageUtils::loadImageFromFile(c.content);
                        if (image.buffer != nullptr)
                        {
                            rq.imageBuffers.push_back(std::move(image));
                        }
                    }
                }
            }
        }
    };
    auto picarFree = [](rt::LLMGenerationRequest& req) {
        for (auto& rq : req.requests)
        {
            rq.imageBuffers.clear();
            rq.imageBuffers.shrink_to_fit();
        }
    };

    // Perform warmup runs if requested
    if (args.warmup > 0)""")

sub("""            rt::LLMGenerationResponse warmupResponse;
            bool requestStatus = false;""",
    """            rt::LLMGenerationResponse warmupResponse;
            picarLoad(firstRequest); // PICAR_378
            bool requestStatus = false;""")

sub("""        LOG_INFO("Warmup of %d runs completed. Starting actual benchmark runs...", args.warmup);""",
    """        picarFree(firstRequest); // PICAR_378
        LOG_INFO("Warmup of %d runs completed. Starting actual benchmark runs...", args.warmup);""")

sub("""        auto picarT0 = std::chrono::steady_clock::now(); // PICAR_378
""",
    """        auto picarT0 = std::chrono::steady_clock::now(); // PICAR_378
        picarLoad(request);
""")

sub("""        double picarMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - picarT0).count();
""",
    """        double picarMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - picarT0).count();
        picarFree(request);
""")

p.write_text(s)
print("patched")
