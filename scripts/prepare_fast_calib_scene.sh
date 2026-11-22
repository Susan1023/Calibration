#!/usr/bin/env bash
# Convert one static ROS2 scene to the ROS1 PointCloud2 bag and raw images
# expected by FAST-Calib. Usage: script ROS2_BAG OUT_DIR [--lidar-only | IMAGE_TOPIC ...]
set -euo pipefail

if (( $# < 2 )); then
  echo '用法: prepare_fast_calib_scene.sh ROS2_BAG目录 输出目录 [图像话题 ...]' >&2
  exit 2
fi

bag_dir=$1
output_dir=$2
shift 2
image_topics=("$@")
lidar_only=false
if (( ${#image_topics[@]} == 1 )) && [[ ${image_topics[0]} == --lidar-only ]]; then
  lidar_only=true
  image_topics=()
elif (( ${#image_topics[@]} == 0 )); then
  image_topics=(/camera1/image /camera2/image)
fi
if [[ "$lidar_only" == false && -e "$output_dir" ]]; then
  echo "输出目录已存在，请换新目录: $output_dir" >&2
  exit 2
fi
if [[ -e "$output_dir/lidar.bag" ]]; then
  echo "拒绝覆盖已有 ROS1 bag: $output_dir/lidar.bag" >&2
  exit 2
fi
if [[ "$lidar_only" == true ]]; then
  for camera in camera1 camera2; do
    if [[ ! -f "$output_dir/${camera}_image.png" ]]; then
      echo "缺少原始照片: $output_dir/${camera}_image.png" >&2
      exit 2
    fi
  done
fi

converter=${ROSBAGS_CONVERT:-rosbags-convert}
if ! command -v "$converter" >/dev/null 2>&1; then
  echo '找不到 rosbags-convert；先激活安装了 rosbags 的 Python 3.12 虚拟环境' >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
image_args=()
for topic in "${image_topics[@]}"; do
  image_args+=(--camera-topic "$topic")
done

if [[ "$lidar_only" == false ]]; then
  /usr/bin/python3 "$script_dir/export_fast_calib_images.py" \
    "$bag_dir" "$output_dir" "${image_args[@]}"
fi

"$converter" --src "$bag_dir" --dst "$output_dir/lidar.bag" \
  --include-topic /livox/lidar --src-typestore ros2_jazzy

echo "已准备 $output_dir/lidar.bag 和原始相机图像；FAST-Calib 默认不旋转图像。"
