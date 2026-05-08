#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_DIR="$SCRIPT_DIR/build"

echo "========== 快速编译 robot_viewer =========="
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Debug
# cmake --build "$BUILD_DIR" --target robot_viewer -j"$(nproc)"

cd build
make -j"$(nproc)"
echo "✅ 快速编译完成"
