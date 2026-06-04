#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

# /opt/homebrew/bin/cmake --no-warn-unused-cli 
#    -D CMAKE_BUILD_TYPE:STRING=RelWithDebInfo
#    -D CMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE 
#    -D CMAKE_C_COMPILER:FILEPATH=/opt/homebrew/opt/llvm@17/bin/clang -DCMAKE_CXX_COMPILER:FILEPATH=/opt/homebrew/opt/llvm@17/bin/clang++ 
#    -S /Users/huohuo/Desktop/UntanglingProject/Codes/ForMerge/LuisaComputeSimulator 
#    -B /Users/huohuo/Desktop/UntanglingProject/Codes/ForMerge/LuisaComputeSimulator/build 
#    -G Ninja

cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DLCS_BUILD_PYBINDINGS=ON \
      "-DLCS_PYTHON_EXECUTABLE=${ROOT}/.venv/bin/python"

cmake --build build -j --target stubs

echo ""
echo "Done. Use with:"
echo "  export PYTHONPATH=${ROOT}/build/bin"
echo "  .venv/bin/python PythonBindings/robotics/test_cartpole.py --headless --advance_frames 300"
echo "  .venv/bin/python -m pip install -e . --no-build-isolation"
