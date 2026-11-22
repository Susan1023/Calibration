#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/validator_ws/install/setup.bash
exec "$@"
