#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 ]]; then
  echo '用法: docker/run-direct-visual-jazzy.sh 命令及参数（如 ros2 run ...）' >&2
  exit 2
fi
root=$(cd "$(dirname "$0")/.." && pwd)
docker run --rm --net host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -v "$root:$root" -w "$root" \
  koide3/direct_visual_lidar_calibration:jazzy "$@"
