"""合成消息，仅验证 Validator ROS2 投影通路；不代表真实标定精度。"""
import json
import os
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

with tempfile.TemporaryDirectory() as tmp:
    calib = tmp + '/simulated.yaml'
    fs = cv2.FileStorage(calib, cv2.FILE_STORAGE_WRITE)
    fs.write('K_0', np.array([[200., 0, 320], [0, 200, 240], [0, 0, 1.]]))
    fs.write('C_0', np.zeros((1, 5)))
    fs.write('E_0', np.eye(4))
    fs.release()
    command = ['ros2', 'run', 'lidar_cam_validator', 'validator_node', '--ros-args',
               '-p', f'calibration_file:={calib}', '-p', 'image_topic:=/test/image',
               '-p', 'cloud_topic:=/test/cloud', '-p', 'sync_mode:=latest',
               '-p', 'show_window:=false', '-p', 'show_statistics:=false',
               '-p', 'show_depth_colorbar:=false', '-p', 'enable_metrics:=false']
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        rclpy.init()
        node = rclpy.create_node('validator_smoke_test')
        image_pub = node.create_publisher(Image, '/test/image', qos_profile_sensor_data)
        cloud_pub = node.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
        fused = []
        info = []
        node.create_subscription(Image, '/validator/fused_image', lambda m: fused.append(m), qos_profile_sensor_data)
        node.create_subscription(String, '/validator/validation_info', lambda m: info.append(json.loads(m.data)), 10)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not (fused and info):
            stamp = node.get_clock().now().to_msg()
            header = Header(stamp=stamp, frame_id='livox_frame')
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, [(0., 0., 5.), (1., 0., 5.)]))
            msg = Image(header=Header(stamp=stamp, frame_id='camera_optical_frame'),
                        height=480, width=640, encoding='bgr8', is_bigendian=0, step=640*3,
                        data=np.zeros((480, 640, 3), dtype=np.uint8).tobytes())
            image_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
        if not fused or not info:
            raise RuntimeError(f'未收到融合图或统计消息；fused={len(fused)}, info={len(info)}')
        if fused[0].width != 640 or fused[0].height != 480:
            raise AssertionError('融合图尺寸错误')
        print(json.dumps({'result': 'PASS', 'data': 'simulated',
                          'fused_image': [fused[0].width, fused[0].height],
                          'valid_projections': info[-1]['valid_projections']}, ensure_ascii=False))
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            stdout, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, _ = proc.communicate()
        print(stdout[-2000:])
        node.destroy_node()
        rclpy.shutdown()
