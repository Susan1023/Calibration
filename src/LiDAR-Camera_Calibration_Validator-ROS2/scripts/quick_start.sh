#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later

set -e

IMAGE_TOPIC="/image_raw"
CLOUD_TOPIC="/cloudpoints"
CALIBRATION_FILE=""
SYNC_MODE="approximate"
LATEST_THRESHOLD="0.5"
FUSED_QOS="best_effort"
CHECK_DEPS=false
FORCE_COMPILE=false

show_help() {
  cat <<HELP
LiDAR-Camera Calibration Validator (ROS2) quick start

Usage:
  $0 [options]

Options:
  -i, --image-topic TOPIC          Image topic
  -c, --cloud-topic TOPIC          Point cloud topic
  -f, --calibration-file FILE      Calibration file path
  --sync-mode MODE                 approximate | latest | latest_pair
  --latest-threshold SEC           Threshold for latest pairing warning
  --fused-qos MODE                 best_effort | reliable
  --check-deps                     Check environment dependencies
  --force-compile                  Force colcon rebuild
  -h, --help                       Show this help
HELP
}

while [[ $# -gt 0 ]]; do
  case $1 in
    -i|--image-topic) IMAGE_TOPIC="$2"; shift 2 ;;
    -c|--cloud-topic) CLOUD_TOPIC="$2"; shift 2 ;;
    -f|--calibration-file) CALIBRATION_FILE="$2"; shift 2 ;;
    --sync-mode) SYNC_MODE="$2"; shift 2 ;;
    --latest-threshold) LATEST_THRESHOLD="$2"; shift 2 ;;
    --fused-qos) FUSED_QOS="$2"; shift 2 ;;
    --check-deps) CHECK_DEPS=true; shift ;;
    --force-compile) FORCE_COMPILE=true; shift ;;
    -h|--help) show_help; exit 0 ;;
    *) echo "Unknown option: $1"; show_help; exit 1 ;;
  esac
done

case "$SYNC_MODE" in
  approximate|latest|latest_pair) ;;
  *) echo "Invalid sync_mode=$SYNC_MODE, fallback to approximate"; SYNC_MODE="approximate" ;;
esac

case "$FUSED_QOS" in
  best_effort|reliable) ;;
  *) echo "Invalid fused-qos=$FUSED_QOS, fallback to best_effort"; FUSED_QOS="best_effort" ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$PACKAGE_DIR")"
if [ "$(basename "$PARENT_DIR")" = "src" ]; then
  WORKSPACE_DIR="$(dirname "$PARENT_DIR")"
else
  WORKSPACE_DIR="$PARENT_DIR"
fi

if [ "$CHECK_DEPS" = true ]; then
  command -v ros2 >/dev/null || { echo "ros2 not found"; exit 1; }
  command -v colcon >/dev/null || { echo "colcon not found"; exit 1; }
  echo "Dependency check passed"
fi

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ] || [ "$FORCE_COMPILE" = true ]; then
  echo "Building workspace with colcon..."
  cd "$WORKSPACE_DIR"
  colcon build --symlink-install --packages-select lidar_cam_validator
fi

source "$WORKSPACE_DIR/install/setup.bash"

LAUNCH_ARGS=(
  "image_topic:=$IMAGE_TOPIC"
  "cloud_topic:=$CLOUD_TOPIC"
  "sync_mode:=$SYNC_MODE"
  "latest_pair_time_threshold:=$LATEST_THRESHOLD"
  "fused_image_qos_reliability:=$FUSED_QOS"
)

if [ -n "$CALIBRATION_FILE" ]; then
  LAUNCH_ARGS+=("calibration_file:=$CALIBRATION_FILE")
fi

echo "Launching validator_node..."
exec ros2 launch lidar_cam_validator validator.launch.py "${LAUNCH_ARGS[@]}"
