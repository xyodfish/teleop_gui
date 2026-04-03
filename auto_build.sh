#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

show_help() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -q         quick build 快速编译"
    echo "  --all      all rebuild 全量重编译（默认）"
    echo "  -old       调用 switch_env.sh 时打印旧版环境提示"
    echo "  --skip-env 跳过 switch_env.sh"
    echo "  --self-test 打包后执行发布包自检"
    echo "  -h, --help 显示帮助"
}

MODE="all"
USE_OLD_ENV="false"
SKIP_SWITCH_ENV="false"
RUN_SELF_TEST="false"

for arg in "$@"; do
    case "$arg" in
        -q)
            MODE="quick"
            ;;
        --all)
            MODE="all"
            ;;
        -old)
            USE_OLD_ENV="true"
            ;;
        --skip-env)
            SKIP_SWITCH_ENV="true"
            ;;
        --self-test)
            RUN_SELF_TEST="true"
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $arg"
            show_help
            exit 1
            ;;
    esac
done

# 兼容参考仓库的环境切换流程
SWITCH_ENV_SCRIPT="$HOME/switch_env.sh"
if [ "$SKIP_SWITCH_ENV" == "true" ] || [ "${TELEOP_GUI_SKIP_SWITCH_ENV:-0}" == "1" ]; then
    echo "根据参数/环境变量，跳过 switch_env.sh"
elif [ -f "$SWITCH_ENV_SCRIPT" ]; then
    echo "检测到 switch_env.sh，正在执行..."
    cd "$SCRIPT_DIR" && "$SWITCH_ENV_SCRIPT" control
    if [ "$USE_OLD_ENV" == "true" ]; then
        echo "使用旧版环境"
    else
        echo "使用新版环境"
    fi
    echo "✅ switch_env.sh 执行完成"
else
    echo "未检测到 switch_env.sh，跳过执行"
fi

if [ "$MODE" == "quick" ]; then
    BUILD_SCRIPT="$SCRIPT_DIR/build.sh"
else
    BUILD_SCRIPT="$SCRIPT_DIR/all_rebuild.sh"
fi

COLLECT_DEP_SCRIPT="$SCRIPT_DIR/scripts/collect_dep.sh"
COPY_FILES_FOR_RELEASE_SCRIPT="$SCRIPT_DIR/scripts/copy_files_for_release.sh"
TEMP_RELEASE_DIR="$SCRIPT_DIR/teleop_gui_release"
TARGET_RELEASE_DIR="${TELEOP_GUI_RELEASE_TARGET_DIR:-$SCRIPT_DIR/../../teleop_gui_release}"

echo "========== 开始执行构建脚本 =========="
if [ -x "$BUILD_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && "$BUILD_SCRIPT"
else
    echo "❌ 错误: 脚本不存在或不可执行: $BUILD_SCRIPT"
    exit 1
fi

echo "========== 开始收集依赖 =========="
if [ -x "$COLLECT_DEP_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && "$COLLECT_DEP_SCRIPT"
else
    echo "❌ 错误: 脚本不存在或不可执行: $COLLECT_DEP_SCRIPT"
    exit 1
fi

echo "========== 开始复制发布目录 =========="
if [ -x "$COPY_FILES_FOR_RELEASE_SCRIPT" ]; then
    cd "$SCRIPT_DIR" && "$COPY_FILES_FOR_RELEASE_SCRIPT"
else
    echo "❌ 错误: 脚本不存在或不可执行: $COPY_FILES_FOR_RELEASE_SCRIPT"
    exit 1
fi

if [ "$RUN_SELF_TEST" == "true" ] || [ "${TELEOP_GUI_RUN_SELF_TEST:-0}" == "1" ]; then
    echo "========== 执行发布包自检 =========="
    if [ -x "$TARGET_RELEASE_DIR/run.sh" ]; then
        "$TARGET_RELEASE_DIR/run.sh" --self-test
        echo "✅ 发布包自检通过"
    else
        echo "❌ 错误: 找不到可执行自检脚本: $TARGET_RELEASE_DIR/run.sh"
        exit 1
    fi
fi

if [ "${TELEOP_GUI_KEEP_STAGE_DIR:-0}" == "1" ]; then
    echo "========== 保留中间目录 =========="
    echo "已保留: $TEMP_RELEASE_DIR"
else
    echo "========== 清理中间目录 =========="
    rm -rf "$TEMP_RELEASE_DIR"
fi

echo "✅ ✅ ✅ 全流程完成！GUI 部署包已就绪。"
