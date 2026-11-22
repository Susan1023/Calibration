#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
docker run --rm --entrypoint /bin/bash -e ROS_DOMAIN_ID=177 \
  -v "$root/src/LiDAR-Camera_Calibration_Validator-ROS2:/tmp/validator_ws/src/LiDAR-Camera_Calibration_Validator-ROS2:ro" \
  -v "$root/tests:/tmp/tests:ro" \
  -w /tmp/validator_ws tdt-radar:lyrical-cuda13.3 -lc \
  'source /opt/ros/lyrical/setup.bash && colcon build --packages-select lidar_cam_validator --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && source install/setup.bash && /usr/bin/python3 /tmp/tests/ros_validator_smoke.py'
