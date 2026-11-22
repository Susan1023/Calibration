#!/usr/bin/env python3
"""Convert LiDAR frames near both camera snapshots to a FAST-Calib ROS1 bag."""

import argparse
import csv
from pathlib import Path

from rosbags.convert.converter import create_connections_converters
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ros2_bag', type=Path)
    parser.add_argument('fast_dir', type=Path)
    parser.add_argument('--seconds-around', type=float, default=5.0,
                        help='每张照片前后保留的点云秒数，默认 5 秒')
    args = parser.parse_args()
    if args.seconds_around <= 0:
        parser.error('--seconds-around 必须大于 0')
    manifest = args.fast_dir / 'image_manifest.csv'
    if not manifest.is_file():
        parser.error(f'缺少相机拍照记录：{manifest}')
    with manifest.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if {row['camera'] for row in rows} != {'camera1', 'camera2'} or len(rows) != 2:
        parser.error('拍照记录必须恰好包含 camera1、camera2 各一张')
    stamps = [int(row['ros_stamp_sec']) * 1_000_000_000 +
              int(row['ros_stamp_nanosec']) for row in rows]
    margin_ns = round(args.seconds_around * 1_000_000_000)
    start, stop = min(stamps) - margin_ns, max(stamps) + margin_ns
    output = args.fast_dir / 'lidar.bag'
    if output.exists():
        parser.error(f'拒绝覆盖已有 ROS1 bag：{output}')
    if not args.ros2_bag.is_dir():
        parser.error(f'ROS2 bag 目录不存在：{args.ros2_bag}')

    args.fast_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    with AnyReader([args.ros2_bag],
                   default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader, \
            Writer(output) as writer:
        connections = [conn for conn in reader.connections
                       if conn.topic == '/livox/lidar' and
                       conn.msgtype == 'sensor_msgs/msg/PointCloud2']
        if not connections:
            raise ValueError('ROS2 bag 中没有 /livox/lidar PointCloud2')
        connmap, convmap = create_connections_converters(
            connections, None, reader, writer)
        for conn, timestamp, data in reader.messages(
                connections=connections, start=start, stop=stop):
            writer.write(connmap[(conn.id, conn.owner)], timestamp,
                         convmap[conn.msgtype](data))
            count += 1
    if count == 0:
        output.unlink()
        raise ValueError('照片时间附近没有点云；检查拍照记录与 bag 是否同一场景')
    print(f'已保存 {count} 帧点云：{output}')
    print(f'点云窗口：照片前后各 {args.seconds_around:g} 秒')


if __name__ == '__main__':
    main()
