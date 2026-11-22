#!/usr/bin/env python3
"""Preview two ROS2 cameras and save one raw frame from each."""

import argparse
import csv
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraPair(Node):
    def __init__(self, topics):
        super().__init__('fast_calib_image_capture')
        self.bridge = CvBridge()
        self.latest = {}
        for name, topic in topics.items():
            self.create_subscription(Image, topic,
                                     lambda msg, camera=name: self.on_image(camera, msg),
                                     qos_profile_sensor_data)

    def on_image(self, camera, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'{camera} 图像转换失败: {exc}')
            return
        self.latest[camera] = (image.copy(), msg.header.stamp.sec,
                               msg.header.stamp.nanosec, msg.header.frame_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--cam1-topic', default='/camera1/image')
    parser.add_argument('--cam2-topic', default='/camera2/image')
    args = parser.parse_args()
    outputs = {name: args.output_dir / f'{name}_image.png' for name in ('camera1', 'camera2')}
    for output in outputs.values():
        if output.exists():
            parser.error(f'拒绝覆盖已有照片：{output}')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CameraPair({'camera1': args.cam1_topic, 'camera2': args.cam2_topic})
    for camera in outputs:
        cv2.namedWindow(camera, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(camera, 800, 600)
    print('让标定板停稳，点击任一小窗后按空格，同时保存两台相机的原始照片；q 退出。')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            for camera in outputs:
                if camera in node.latest:
                    raw = node.latest[camera][0]
                    cv2.imshow(camera, raw)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                return
            if key == ord(' '):
                missing = set(outputs) - node.latest.keys()
                if missing:
                    print(f'还没有收到图像：{sorted(missing)}')
                    continue
                with (args.output_dir / 'image_manifest.csv').open('w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['camera', 'topic', 'file', 'saved_at', 'ros_stamp_sec',
                                     'ros_stamp_nanosec', 'frame_id', 'width', 'height',
                                     'saved_rotation_deg'])
                    for camera, output in outputs.items():
                        raw, sec, nsec, frame_id = node.latest[camera]
                        if not cv2.imwrite(str(output), raw):
                            raise OSError(f'无法保存：{output}')
                        topic = args.cam1_topic if camera == 'camera1' else args.cam2_topic
                        writer.writerow([camera, topic, str(output),
                                         datetime.now().isoformat(timespec='seconds'),
                                         sec, nsec, frame_id, raw.shape[1], raw.shape[0], 0])
                        print(f'{camera} 原始照片：{output}')
                return
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
