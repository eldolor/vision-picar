"""3.78 amendment 3: let Edge-LLM v0.9.1 build against JetPack 6.2's
TensorRT 10.3. v0.9.1 lists "JetPack 6.2+" as compatible but loads engines
through `nvinfer1::IStreamReaderV2`, which TensorRT added in 10.7. Below 10.7
the engine file is read into memory and deserialised from the buffer -- the
path v0.9.1's own audio runners already use. Idempotent.

    python tools/jetson/patch_edgellm_trt103.py <edge-llm tree>
"""
import sys
from pathlib import Path

GUARD = "#if NV_TENSORRT_MAJOR > 10 || (NV_TENSORRT_MAJOR == 10 && NV_TENSORRT_MINOR >= 7)"


def patch(text: str) -> str:
    """Each edit checks for itself, so a tree patched by an earlier version
    of this script still gets the later edits."""
    if "PICAR_378_TRT103" not in text:
        head = "class FileStreamReaderV2 : public nvinfer1::IStreamReaderV2"
        i = text.index(head)
        j = text.index("\n};\n", i) + len("\n};\n")
        text = text[:i] + GUARD + " // PICAR_378_TRT103\n" + text[i:j] + "#endif\n" + text[j:]
    old = """    FileStreamReaderV2 streamReader(enginePath);
    auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime.deserializeCudaEngine(streamReader));
"""
    new = GUARD + """
    FileStreamReaderV2 streamReader(enginePath);
    auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime.deserializeCudaEngine(streamReader));
#else
    std::ifstream file(enginePath, std::ios::binary | std::ios::ate);
    std::vector<char> blob(file ? static_cast<size_t>(file.tellg()) : 0);
    file.seekg(0);
    if (!blob.empty())
    {
        file.read(blob.data(), static_cast<std::streamsize>(blob.size()));
    }
    auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime.deserializeCudaEngine(blob.data(), blob.size()));
#endif
"""
    if "#else\n    std::ifstream file(enginePath" not in text:
        assert text.count(old) == 1, "loader anchor not found"
        text = text.replace(old, new)
    # Only the V2 reader calls it; unused below 10.7 and -Werror is on.
    text = text.replace("\nbool isDeviceAccessiblePointer(void* ptr) noexcept",
                        "\n[[maybe_unused]] bool isDeviceAccessiblePointer(void* ptr) noexcept", 1)
    if "#include <fstream>" not in text:
        text = "#include <fstream>\n#include <vector>\n" + text
    return text


FP4_LINE = ("        return desc.type == DataType::kFP4 || desc.type == DataType::kINT8 "
            "|| desc.type == DataType::kFP8;\n")


def patch_fp4(text: str) -> str:
    """`DataType::kFP4` arrived in TensorRT 10.8. The NVFP4 all-reduce plugin
    is Blackwell-only and never runs on Orin; below 10.8 it accepts the
    other two types. Idempotent."""
    if "PICAR_378_FP4" in text:
        return text
    assert text.count(FP4_LINE) >= 1, "kFP4 anchor not found"
    return text.replace(FP4_LINE, (
        "#if NV_TENSORRT_MAJOR > 10 || (NV_TENSORRT_MAJOR == 10 && NV_TENSORRT_MINOR >= 8) "
        "// PICAR_378_FP4\n" + FP4_LINE + "#else\n"
        "        return desc.type == DataType::kINT8 || desc.type == DataType::kFP8;\n#endif\n"))


CUBIN_OLD = "                CUDA_DRIVER_CHECK(cuModuleLoadData(&hModule, kernelMeta.mCubin));\n"
CUBIN_NEW = """                // PICAR_378_CUBIN: v0.9.1 ships two sm87 custom-mask cubins
                // (head dim 256) built by CUDA 12.8+ (ELF ABI 8), which
                // JetPack 6.2's 12.6 driver rejects. Skip what cannot load;
                // a model that needs a skipped kernel fails at its lookup.
                CUresult const picarLoad = cuModuleLoadData(&hModule, kernelMeta.mCubin);
                if (picarLoad == CUDA_ERROR_INVALID_IMAGE)
                {
                    LOG_WARNING("Skipping FMHA cubin for %s: built by a newer CUDA than this driver",
                        kernelMeta.mFuncName);
                    continue;
                }
                CUDA_DRIVER_CHECK(picarLoad);
"""


def patch_cubin(text: str) -> str:
    if "PICAR_378_CUBIN" in text:
        return text
    assert text.count(CUBIN_OLD) == 1, "cubin load anchor not found"
    return text.replace(CUBIN_OLD, CUBIN_NEW)


if __name__ == "__main__":
    tree = Path(sys.argv[1])
    p = tree / "cpp/common/trtUtils.cpp"
    p.write_text(patch(p.read_text()))
    q = tree / "cpp/plugins/fusedNvfp4GemmAllReducePlugin/fusedNvfp4GemmAllReducePlugin.cpp"
    q.write_text(patch_fp4(q.read_text()))
    r = tree / "cpp/kernels/contextAttentionKernels/contextFMHARunner.cpp"
    r.write_text(patch_cubin(r.read_text()))
    print("patched", p, q, r)
