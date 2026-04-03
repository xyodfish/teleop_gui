#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
SRC_DIR="$SCRIPT_DIR/../teleop_gui_release"
TARGET_DIR="${TELEOP_GUI_RELEASE_TARGET_DIR:-$SCRIPT_DIR/../../teleop_gui_release}"

if [ ! -d "$SRC_DIR" ]; then
    echo "❌ 错误: 源目录不存在: $SRC_DIR"
    exit 1
fi

mkdir -p "$TARGET_DIR"
rm -rf "$TARGET_DIR"/*

cp -a "$SRC_DIR"/. "$TARGET_DIR"/
find "$TARGET_DIR" -type f -name "*.sh" -exec chmod +x {} \;

echo "✅ 已复制发布目录到: $TARGET_DIR"
