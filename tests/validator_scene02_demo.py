"""Run the upstream Validator on one real image/cloud pair as a pipeline demo.

The trial extrinsic is borrowed from the upstream sample and has no calibration
meaning for this camera and LiDAR. Output is a functionality preview only.
"""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
import rosbag2_py
import yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


def read_pair(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    images = []
    clouds = []
    while reader.has_next() and (len(images) < 30 or len(clouds) < 100):
        topic, raw, _ = reader.read_next()
        if topic == "/camera1/image" and len(images) < 30:
            images.append(deserialize_message(raw, Image))
        elif topic == "/livox/lidar" and len(clouds) < 100:
            clouds.append(deserialize_message(raw, PointCloud2))
    if not images or not clouds:
        raise RuntimeError("Bag lacks camera images or point clouds")
    image = images[len(images) // 2]
    stamp = lambda m: m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
    cloud = min(clouds, key=lambda m: abs(stamp(m) - stamp(image)))
    return image, cloud, abs(stamp(image) - stamp(cloud)) / 1e6


def write_trial_calibration(intrinsics_path, sample_path, output_path):
    with open(intrinsics_path, encoding="utf-8") as file:
        intrinsics = yaml.safe_load(file)
    sample = cv2.FileStorage(sample_path, cv2.FILE_STORAGE_READ)
    extrinsic = sample.getNode("E_0").mat()
    sample.release()
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    fs.write("K_0", np.asarray(intrinsics["K"], dtype=np.float64))
    fs.write("C_0", np.asarray(intrinsics["D"], dtype=np.float64).reshape(1, -1))
    fs.write("E_0", extrinsic)
    fs.release()


def main():
    project = sys.argv[1]
    output_dir = sys.argv[2]
    scene = sys.argv[3] if len(sys.argv) > 3 else "scene02"
    bag_dir = os.path.join(project, "data/lidar", scene)
    image, cloud, pair_gap_ms = read_pair(bag_dir)
    os.makedirs(output_dir, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        calib_path = os.path.join(tmp, "trial_not_calibrated.yaml")
        write_trial_calibration(
            os.path.join(project, "config/camera_intrinsics.yaml"),
            os.path.join(project, "src/LiDAR-Camera_Calibration_Validator-ROS2/config/sample_calibration.yaml"),
            calib_path,
        )
        command = [
            "ros2", "run", "lidar_cam_validator", "validator_node", "--ros-args",
            "-p", f"calibration_file:={calib_path}",
            "-p", "image_topic:=/camera1/image",
            "-p", "cloud_topic:=/livox/lidar",
            "-p", "sync_mode:=latest",
            "-p", "show_window:=false",
            "-p", "show_statistics:=false",
            "-p", "show_depth_colorbar:=false",
            "-p", "enable_metrics:=false",
        ]
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        rclpy.init()
        node = rclpy.create_node("validator_scene02_demo")
        image_pub = node.create_publisher(Image, "/camera1/image", qos_profile_sensor_data)
        cloud_pub = node.create_publisher(PointCloud2, "/livox/lidar", qos_profile_sensor_data)
        fused = []
        info = []
        node.create_subscription(Image, "/validator/fused_image", lambda msg: fused.append(msg), qos_profile_sensor_data)
        node.create_subscription(String, "/validator/validation_info", lambda msg: info.append(json.loads(msg.data)), 10)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not (fused and info):
                new_stamp = node.get_clock().now().to_msg()
                image.header.stamp = new_stamp
                cloud.header.stamp = new_stamp
                cloud_pub.publish(cloud)
                image_pub.publish(image)
                rclpy.spin_once(node, timeout_sec=0.2)
            if not fused or not info:
                raise RuntimeError(f"No fusion output: fused={len(fused)}, info={len(info)}")
            msg = fused[-1]
            if msg.encoding != "bgr8":
                raise RuntimeError(f"Unexpected image encoding: {msg.encoding}")
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
            array = array.reshape(msg.height, msg.width, 3)
            preview = os.path.join(output_dir, f"{scene}_validator_trial.png")
            cv2.imwrite(preview, array)
            result = {
                "result": "PASS",
                "data": f"{scene} real image and cloud, trial extrinsic only",
                "pair_stamp_gap_ms": round(pair_gap_ms, 2),
                "fused_image": [msg.width, msg.height],
                "valid_projections": info[-1].get("valid_projections"),
                "preview": preview,
            }
            print(json.dumps(result, ensure_ascii=False), flush=True)
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                stdout, _ = proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout, _ = proc.communicate()
            print(stdout[-2500:], flush=True)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
