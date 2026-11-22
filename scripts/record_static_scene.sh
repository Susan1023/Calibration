#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 4 || $# -gt 5 ]]; then
  echo '用法: record_static_scene.sh 图像话题 PointCloud2话题 输出bag目录 场景编号 [CameraInfo话题]' >&2
  exit 2
fi
image_topic=$1
cloud_topic=$2
bag_dir=$3
scene_id=$4
info_topic=${5:-}
[[ ! -e "$bag_dir" ]] || { echo "输出已存在: $bag_dir" >&2; exit 2; }
[[ $(ros2 topic type "$image_topic") == sensor_msgs/msg/Image ]] || { echo '图像话题必须是 sensor_msgs/msg/Image' >&2; exit 2; }
[[ $(ros2 topic type "$cloud_topic") == sensor_msgs/msg/PointCloud2 ]] || { echo '点云话题必须是 sensor_msgs/msg/PointCloud2' >&2; exit 2; }
topics=("$image_topic" "$cloud_topic")
if [[ -n "$info_topic" ]]; then
  [[ $(ros2 topic type "$info_topic") == sensor_msgs/msg/CameraInfo ]] || { echo 'CameraInfo 话题类型错误' >&2; exit 2; }
  topics+=("$info_topic")
fi
mkdir -p "$(dirname "$bag_dir")"
echo "场景 $scene_id: 保持传感器和场景静止，录制至少 10–15 秒，然后 Ctrl+C。"
ros2 bag record -o "$bag_dir" "${topics[@]}"
