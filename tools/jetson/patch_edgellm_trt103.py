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
    if "PICAR_378_TRT103" in text:
        return text
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
    assert text.count(old) == 1, "loader anchor not found"
    text = text.replace(old, new)
    # Only the V2 reader calls it; unused below 10.7 and -Werror is on.
    text = text.replace("\nbool isDeviceAccessiblePointer(void* ptr) noexcept",
                        "\n[[maybe_unused]] bool isDeviceAccessiblePointer(void* ptr) noexcept", 1)
    if "#include <fstream>" not in text:
        text = "#include <fstream>\n#include <vector>\n" + text
    return text


if __name__ == "__main__":
    p = Path(sys.argv[1]) / "cpp/common/trtUtils.cpp"
    p.write_text(patch(p.read_text()))
    print("patched", p)
