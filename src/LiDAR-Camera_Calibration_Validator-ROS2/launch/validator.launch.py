#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_cam_validator')
    default_settings = os.path.join(pkg_share, 'config', 'settings.yaml')
    default_calib = os.path.join(pkg_share, 'config', 'sample_calibration.yaml')

    calibration_file = LaunchConfiguration('calibration_file')
    image_topic = LaunchConfiguration('image_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    sync_mode = LaunchConfiguration('sync_mode')
    latest_pair_time_threshold = LaunchConfiguration('latest_pair_time_threshold')
    fused_image_qos_reliability = LaunchConfiguration('fused_image_qos_reliability')
    settings_file = LaunchConfiguration('settings_file')
    open_param_panel = LaunchConfiguration('open_param_panel')

    return LaunchDescription([
        DeclareLaunchArgument('settings_file', default_value=default_settings),
        DeclareLaunchArgument('calibration_file', default_value=default_calib),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('cloud_topic', default_value='/cloudpoints'),
        DeclareLaunchArgument('sync_mode', default_value='approximate'),
        DeclareLaunchArgument('latest_pair_time_threshold', default_value='0.5'),
        DeclareLaunchArgument('fused_image_qos_reliability', default_value='best_effort'),
        DeclareLaunchArgument('open_param_panel', default_value='true'),
        Node(
            package='lidar_cam_validator',
            executable='validator_node',
            name='validator_node',
            output='screen',
            parameters=[
                settings_file,
                {
                    'calibration_file': calibration_file,
                    'image_topic': image_topic,
                    'cloud_topic': cloud_topic,
                    'sync_mode': sync_mode,
                    'latest_pair_time_threshold': latest_pair_time_threshold,
                    'fused_image_qos_reliability': fused_image_qos_reliability,
                },
            ],
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'validator_param_panel.py'),
                 '--node', '/validator_node'],
            output='screen',
            condition=IfCondition(open_param_panel),
        ),
    ])
