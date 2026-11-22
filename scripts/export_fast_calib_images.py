#!/usr/bin/env python3
"""Export one unrotated camera frame per topic from a static ROS2 scene bag.

FAST-Calib rotates these raw images itself when rotate_180 is enabled.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def bgr_image(msg, msgtype):
    if msgtype == 'sensor_msgs/msg/CompressedImage':
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('压缩图像解码失败')
        return image
    if msgtype != 'sensor_msgs/msg/Image':
        raise ValueError(f'图像话题类型不支持：{msgtype}')
    if msg.encoding in ('bgr8', 'rgb8'):
        row_bytes = msg.width * 3
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        image = image[:, :row_bytes].reshape(msg.height, msg.width, 3)
        return image.copy() if msg.encoding == 'bgr8' else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        return cv2.cvtColor(image[:, :msg.width], cv2.COLOR_GRAY2BGR)
    raise ValueError(f'图像编码不支持：{msg.encoding}；请录制相机的原始 bgr8 话题')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir', type=Path, help='ROS2 bag 目录，内含 metadata.yaml')
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--camera-topic', action='append', required=True,
                        help='可重复指定，例如 /camera1/image 和 /camera2/image')
    parser.add_argument('--offset-seconds', type=float, default=0.0,
                        help='从 bag 开始后等待几秒再取图，默认取第一帧')
    args = parser.parse_args()
    if args.offset_seconds < 0:
        parser.error('--offset-seconds 不能为负')
    metadata_file = args.bag_dir / 'metadata.yaml'
    if not metadata_file.is_file():
        parser.error(f'未找到 {metadata_file}')
    metadata = yaml.safe_load(metadata_file.read_text())['rosbag2_bagfile_information']
    storage_id = metadata['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = set(args.camera_topic) - topic_types.keys()
    if missing:
        parser.error(f'bag 中没有图像话题：{sorted(missing)}')
    for topic in args.camera_topic:
        if topic_types[topic] not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            parser.error(f'{topic} 的类型是 {topic_types[topic]}，不是图像')
    first_stamp = None
    saved = set()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    while reader.has_next() and len(saved) < len(args.camera_topic):
        topic, raw, stamp = reader.read_next()
        if first_stamp is None:
            first_stamp = stamp
        if topic not in args.camera_topic or topic in saved:
            continue
        if stamp - first_stamp < args.offset_seconds * 1e9:
            continue
        msg = deserialize_message(raw, get_message(topic_types[topic]))
        image = bgr_image(msg, topic_types[topic])
        name = topic.strip('/').replace('/', '_') + '.png'
        output = args.output_dir / name
        if output.exists():
            raise FileExistsError(f'拒绝覆盖已有图像：{output}')
        if not cv2.imwrite(str(output), image):
            raise OSError(f'无法保存图像：{output}')
        print(f'{topic} -> {output} ({image.shape[1]}x{image.shape[0]})')
        saved.add(topic)
    if len(saved) < len(args.camera_topic):
        raise RuntimeError(f'未能导出这些话题的图像：{sorted(set(args.camera_topic) - saved)}')


if __name__ == '__main__':
    main()
