#!/usr/bin/env bash
# 3.78 amendment 3: build TensorRT Edge-LLM v0.9.1 on the Jetson (JetPack 6.2,
# CUDA 12.6, TensorRT 10.3) with edgellm_identity.cpp added as an example.
#
#   bash tools/jetson/build_edgellm_identity.sh ~/bench-378/ell91
#
# The tree is a clone of NVIDIA/TensorRT-Edge-LLM at v0.9.1 with submodules.
# ENABLE_CUTE_DSL stays off: its kernels are Blackwell/Thor-only and need
# CUDA 12.8 types (Edge-LLM issue #183).
set -euo pipefail
SRC="$(cd "$(dirname "$0")" && pwd)/edgellm_identity.cpp"
TREE="${1:?usage: build_edgellm_identity.sh <edge-llm tree>}"
cd "$TREE"
cp "$SRC" examples/llm/edgellm_identity.cpp
if ! grep -q edgellm_identity examples/llm/CMakeLists.txt; then
  cat >> examples/llm/CMakeLists.txt <<'CMAKE'

# vision-picar 3.78 amendment 3
add_executable(edgellm_identity edgellm_identity.cpp)
target_link_libraries(edgellm_identity PRIVATE edgellmCore exampleUtils commonLibraryExt)
target_include_directories(edgellm_identity PRIVATE ${COMMON_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/examples/utils)
add_cross_build_link_options(edgellm_identity)
CMAKE
fi
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DTRT_PACKAGE_DIR=/usr \
  -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64_linux_toolchain.cmake \
  -DEMBEDDED_TARGET=jetson-orin -DCUDA_CTK_VERSION=12.6
make -j"${JOBS:-5}" llm_build visual_build edgellm_identity
ls -la examples/llm/llm_build examples/multimodal/visual_build examples/llm/edgellm_identity
