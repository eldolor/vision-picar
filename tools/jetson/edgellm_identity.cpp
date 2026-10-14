/*
 * tools/jetson/edgellm_identity.cpp -- 3.78 amendment 3 (vision-picar).
 *
 * A resident TensorRT Edge-LLM (v0.9.1) runtime driven line by line, so
 * `bench_identity_trt.py` can measure arm T exactly as arm L: stream each
 * reply with its top-K logprobs and cancel at the deciding token.
 *
 * Built inside Edge-LLM's examples/llm (see `build_edgellm_identity.sh`),
 * modelled on that release's llm_stream.cpp. Apache-2.0 code it links to is
 * NVIDIA's; this file is ours.
 *
 * Protocol (one request in flight at a time):
 *   stdout  READY                         runtime loaded, decode graph captured
 *   stdin   <path to a request JSON>      one request, batch 1 (its parsing decodes the image)
 *   stdout  T <id> <k> <id>:<lp> ...      one line per generated token, top-k entries
 *   stdin   C                             cancel the current request (optional)
 *   stdout  E <reason> <ms> <ok>          request over: reason 1 end-id, 2 length, 3 cancelled
 *   stdin   EOF                           exit
 */

#include "common/trtUtils.h"
#include "requestFileParser.h"
#include "runtime/llmInferenceRuntime.h"
#include "runtime/llmRuntimeUtils.h"
#include "runtime/streaming.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <poll.h>
#include <unistd.h>

using namespace trt_edgellm;

namespace
{

//! A non-blocking look at stdin: true if a line saying "C" is waiting.
bool cancelRequested()
{
    struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if (poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN))
    {
        return false;
    }
    std::string line;
    if (!std::getline(std::cin, line))
    {
        return false;
    }
    return line == "C";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "usage: " << argv[0] << " <llm engine dir> <visual engine dir>\n";
        return 1;
    }
    std::string const engineDir = argv[1];
    std::string const visualDir = argv[2];

    auto pluginHandles = loadEdgellmPluginLib();
    cudaStream_t stream{};
    if (cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking) != cudaSuccess)
    {
        std::cerr << "cudaStreamCreate failed\n";
        return 3;
    }
    std::unique_ptr<rt::LLMInferenceRuntime> runtime;
    try
    {
        runtime = std::make_unique<rt::LLMInferenceRuntime>(
            engineDir, visualDir, std::unordered_map<std::string, std::string>{}, stream);
    }
    catch (std::exception const& e)
    {
        std::cerr << "Runtime init failed: " << e.what() << "\n";
        return 3;
    }
    runtime->captureDecodingCUDAGraph(stream);
    std::printf("READY\n");
    std::fflush(stdout);

    std::string path;
    while (std::getline(std::cin, path))
    {
        if (path.empty() || path == "C")
        {
            continue; // a cancel that arrived after its request ended
        }
        auto const t0 = std::chrono::steady_clock::now();
        rt::LLMGenerationRequest req;
        bool parsed = false;
        try
        {
            auto [lora, batches] = exampleUtils::parseRequestFile(path, /*batchSizeOverride=*/1);
            if (!batches.empty() && batches[0].requests.size() == 1)
            {
                req = std::move(batches[0]);
                parsed = true;
            }
        }
        catch (std::exception const& e)
        {
            std::cerr << "parse failed: " << e.what() << "\n";
        }
        if (!parsed)
        {
            std::printf("E 0 0 0\n");
            std::fflush(stdout);
            continue;
        }

        auto ch = rt::StreamChannel::create();
        ch->setStreamInterval(1);
        ch->setSkipSpecialTokens(false);
        req.streamChannels.clear();
        req.streamChannels.push_back(ch);

        rt::LLMGenerationResponse response;
        std::atomic<bool> ok{false};
        std::thread worker([&] { ok = runtime->handleRequest(req, response, stream); });

        rt::FinishReason reason = rt::FinishReason::kNotFinished;
        bool cancelled = false;
        ch->consume([&](rt::StreamChunk&& c) {
            for (size_t i = 0; i < c.tokenIds.size(); ++i)
            {
                std::string line = "T " + std::to_string(c.tokenIds[i]);
                if (i < c.logprobs.size())
                {
                    line += " " + std::to_string(c.logprobs[i].size());
                    for (auto const& e : c.logprobs[i])
                    {
                        char buf[48];
                        std::snprintf(buf, sizeof(buf), " %d:%.6f", e.tokenId, e.logprob);
                        line += buf;
                    }
                }
                else
                {
                    line += " 0";
                }
                std::printf("%s\n", line.c_str());
            }
            std::fflush(stdout);
            if (c.finished)
            {
                reason = c.reason;
            }
            if (!cancelled && cancelRequested())
            {
                cancelled = true;
                ch->cancel();
            }
        });
        worker.join();
        if (reason == rt::FinishReason::kNotFinished)
        {
            reason = ch->getReason();
        }
        double const ms
            = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("E %d %.3f %d\n", static_cast<int>(reason), ms, ok.load() ? 1 : 0);
        std::fflush(stdout);
    }

    runtime.reset();
    cudaStreamDestroy(stream);
    return 0;
}
