FROM koide3/direct_visual_lidar_calibration:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-pcl-conversions ros-jazzy-pcl-ros libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/validator_ws
COPY src/LiDAR-Camera_Calibration_Validator-ROS2 /root/validator_ws/src/LiDAR-Camera_Calibration_Validator-ROS2
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select lidar_cam_validator --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'

COPY docker/validator-entrypoint.sh /validator-entrypoint.sh
RUN chmod +x /validator-entrypoint.sh
ENTRYPOINT ["/validator-entrypoint.sh"]
CMD ["bash"]
