#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/.."; pwd)

MARKER_TOPIC="/teleop_gui/ik_target_pose"
MARKER_FRAME="world"
RVIZ_FIXED_FRAME="map"
VIEWER_BIN="$REPO_DIR/bin/robot_kinematic_viewer"
VIEWER_CONFIG="$REPO_DIR/config/robot_viewer.yaml"

START_RVIZ=1
START_VIEWER=1

show_help() {
    cat <<'EOF'
Usage: start_rviz_ik_stack.sh [options]

Options:
  --topic <name>             Marker PoseStamped topic (default: /teleop_gui/ik_target_pose)
  --marker-frame <frame>     Marker frame_id (default: world)
  --fixed-frame <frame>      RViz fixed frame (default: map)
  --viewer-bin <path>        robot_kinematic_viewer executable path
  --viewer-config <path>     viewer config file path
  --no-rviz                  do not start RViz
  --no-viewer                do not start robot_kinematic_viewer
  -h, --help                 show this help

Notes:
  - Script starts: roscore, static TF(map->world), RViz marker node, RViz, viewer.
  - Press Ctrl-C in this terminal to stop all started processes.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --topic)
            MARKER_TOPIC="${2:-}"
            shift 2
            ;;
        --marker-frame)
            MARKER_FRAME="${2:-}"
            shift 2
            ;;
        --fixed-frame)
            RVIZ_FIXED_FRAME="${2:-}"
            shift 2
            ;;
        --viewer-bin)
            VIEWER_BIN="${2:-}"
            shift 2
            ;;
        --viewer-config)
            VIEWER_CONFIG="${2:-}"
            shift 2
            ;;
        --no-rviz)
            START_RVIZ=0
            shift
            ;;
        --no-viewer)
            START_VIEWER=0
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            show_help
            exit 1
            ;;
    esac
done

source /opt/ros/noetic/setup.bash

mkdir -p "$REPO_DIR/build/logs"
LOG_DIR="$REPO_DIR/build/logs"
PIDS=()

cleanup() {
    set +e
    echo
    echo "[cleanup] Stopping started processes..."
    for ((i=${#PIDS[@]}-1; i>=0; --i)); do
        kill "${PIDS[i]}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[cleanup] Done."
}
trap cleanup EXIT INT TERM

wait_for_ros_master() {
    local retries=60
    while [[ $retries -gt 0 ]]; do
        if rostopic list >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.2
        retries=$((retries - 1))
    done
    return 1
}

if rostopic list >/dev/null 2>&1; then
    echo "[info] ROS master already running."
else
    echo "[start] roscore"
    roscore >"$LOG_DIR/roscore.log" 2>&1 &
    PIDS+=("$!")
    if ! wait_for_ros_master; then
        echo "[error] roscore did not become ready. See $LOG_DIR/roscore.log" >&2
        exit 1
    fi
fi

echo "[start] static tf: map -> world"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map world >"$LOG_DIR/static_tf_map_world.log" 2>&1 &
PIDS+=("$!")

echo "[start] RViz interactive marker publisher"
python3 "$SCRIPT_DIR/rviz_ik_interactive_marker.py" \
    "_frame_id:=$MARKER_FRAME" \
    "_publish_topic:=$MARKER_TOPIC" \
    >"$LOG_DIR/rviz_ik_interactive_marker.log" 2>&1 &
PIDS+=("$!")

if [[ $START_RVIZ -eq 1 ]]; then
    echo "[start] rviz (fixed frame: $RVIZ_FIXED_FRAME)"
    rviz -f "$RVIZ_FIXED_FRAME" >"$LOG_DIR/rviz.log" 2>&1 &
    PIDS+=("$!")
fi

if [[ $START_VIEWER -eq 1 ]]; then
    if [[ ! -x "$VIEWER_BIN" ]]; then
        echo "[error] viewer binary not found or not executable: $VIEWER_BIN" >&2
        exit 1
    fi
    echo "[start] robot_kinematic_viewer"
    "$VIEWER_BIN" "$VIEWER_CONFIG" \
        "_enable_external_ik_target:=true" \
        "_ik_target_pose_topic:=$MARKER_TOPIC" \
        "_ik_target_pose_frame:=$MARKER_FRAME" \
        >"$LOG_DIR/robot_kinematic_viewer.log" 2>&1 &
    PIDS+=("$!")
fi

echo "[ready] All processes started."
echo "[ready] Marker topic: $MARKER_TOPIC"
echo "[ready] Marker frame: $MARKER_FRAME"
echo "[ready] Logs: $LOG_DIR"
echo "[ready] Press Ctrl-C to stop all."

wait
