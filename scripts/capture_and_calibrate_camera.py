#!/usr/bin/env python3
"""采集 ROS2 棋盘格原图，达到目标张数后自动计算相机内参。"""
import argparse
import csv
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import cv2

from calibrate_camera import calibrate


IMAGE_EXTENSIONS = {'.png', '.jpg', '.jpeg'}


def detect_corners(image, columns, rows):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    found, corners = cv2.findChessboardCornersSB(
        enhanced, (columns, rows), flags=cv2.CALIB_CB_EXHAUSTIVE)
    return corners if found else None


def image_files(directory):
    return sorted(p for p in directory.iterdir()
                  if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)


def solve_session(images_dir, output_dir, args):
    files = image_files(images_dir)
    if len(files) < args.min_views:
        raise ValueError(f'只保存了 {len(files)} 张照片；至少需要 {args.min_views} 张')
    result = calibrate(files, args.columns, args.rows,
                       args.square_mm / 1000.0 if args.square_mm is not None else None,
                       output_dir, args.outlier_px)
    print(f'\n标定完成：{len(files)} 张照片，RMS={result["rms_px"]:.4f} px')
    print(f'内参：{output_dir / "camera_intrinsics.yaml"}')
    print(f'逐图误差：{output_dir / "camera_reprojection_errors.csv"}')
    print(f'报告：{output_dir / "camera_calibration_report.md"}')
    print('此 RMS 是棋盘角点拟合误差，不是 LiDAR–Camera 外参精度。')
    return result


def capture_live(args):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class ImageReceiver(Node):
        def __init__(self):
            super().__init__('camera_intrinsics_capture')
            self.bridge = CvBridge()
            self.latest = None
            self.frame_number = 0
            self.create_subscription(Image, args.topic, self.on_image,
                                     qos_profile_sensor_data)

        def on_image(self, msg):
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:
                self.get_logger().error(f'图像转换失败: {exc}')
                return
            if args.rotate_180:
                image = cv2.rotate(image, cv2.ROTATE_180)
            self.latest = (image.copy(), msg.header.stamp.sec,
                           msg.header.stamp.nanosec, msg.header.frame_id,
                           msg.encoding, self.frame_number + 1)
            self.frame_number += 1

    images_dir = args.session_dir / 'images'
    images_dir.mkdir(parents=True, exist_ok=True)
    existing = image_files(images_dir)
    if existing and not args.resume:
        raise ValueError(f'{images_dir} 已有照片；使用新目录，或加 --resume 继续采集')
    saved = len(existing)
    expected_size = None
    if existing:
        first = cv2.imread(str(existing[0]))
        if first is None:
            raise ValueError(f'无法读取已有照片：{existing[0]}')
        expected_size = (first.shape[1], first.shape[0])
    next_index = 1
    while (images_dir / f'chessboard_{next_index:03d}.png').exists():
        next_index += 1
    if saved >= args.target_views:
        print(f'已有 {saved} 张照片，直接计算内参。')
        return solve_session(images_dir, args.session_dir, args)

    manifest = args.session_dir / 'capture_manifest.csv'
    manifest_exists = manifest.exists()
    rclpy.init()
    receiver = ImageReceiver()
    pending = None
    last_captured_frame = -1
    last_displayed_frame = -1
    executor = ThreadPoolExecutor(max_workers=1)
    cv2.namedWindow('Camera intrinsics capture', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera intrinsics capture', 800, 600)
    print(f'订阅 {args.topic}。移动棋盘并停稳后按空格保存；达到 {args.target_views} 张自动计算。')
    print(f'按回车可在至少 {args.min_views} 张时提前计算；按 q 退出，照片保留在 {images_dir}。')
    try:
        with manifest.open('a', newline='') as metadata_file:
            writer = csv.writer(metadata_file)
            if not manifest_exists:
                writer.writerow(['file', 'captured_at', 'ros_stamp_sec',
                                 'ros_stamp_nanosec', 'frame_id', 'encoding',
                                 'width', 'height'])
            while rclpy.ok():
                rclpy.spin_once(receiver, timeout_sec=0.02)

                if pending is not None and pending.done():
                    frame, corners = pending.result()
                    pending = None
                    if corners is None:
                        print('未找到完整棋盘角点；检查 11×8 规格、曝光、清晰度和边界。')
                    else:
                        image, sec, nsec, frame_id, encoding, number = frame
                        size = (image.shape[1], image.shape[0])
                        if expected_size is None:
                            expected_size = size
                        if size != expected_size:
                            raise RuntimeError(f'分辨率从 {expected_size} 变为 {size}，本次采集已停止')
                        filename = images_dir / f'chessboard_{next_index:03d}.png'
                        if filename.exists():
                            raise RuntimeError(f'文件已存在，拒绝覆盖：{filename}')
                        if not cv2.imwrite(str(filename), image):
                            raise RuntimeError(f'无法保存：{filename}')
                        writer.writerow([str(filename), datetime.now().isoformat(timespec='seconds'),
                                         sec, nsec, frame_id, encoding, size[0], size[1]])
                        metadata_file.flush()
                        saved += 1
                        next_index += 1
                        while (images_dir / f'chessboard_{next_index:03d}.png').exists():
                            next_index += 1
                        last_captured_frame = number
                        print(f'已保存 {saved}/{args.target_views}：{filename.name}；'
                              f'分辨率 {size[0]}×{size[1]}，ROS 时间戳 {sec}.{nsec:09d}')
                        if saved >= args.target_views:
                            return solve_session(images_dir, args.session_dir, args)

                if receiver.latest is not None:
                    image, _, _, _, _, number = receiver.latest
                    if number != last_displayed_frame:
                        last_displayed_frame = number
                        display = image.copy()
                        status = f'{saved}/{args.target_views}  SPACE capture  ENTER solve  Q quit'
                        cv2.putText(display, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8, (0, 255, 0), 2)
                        cv2.imshow('Camera intrinsics capture', display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print('采集结束，已保存照片；未计算内参。')
                    return None
                if key == ord(' '):
                    if receiver.latest is None:
                        print('尚未收到相机图像；检查话题名和相机驱动。')
                    elif pending is not None:
                        print('上一张照片仍在检测角点，请稍候。')
                    elif receiver.latest[-1] == last_captured_frame:
                        print('这帧已保存；请移动棋盘并等待新图像。')
                    else:
                        frame = receiver.latest
                        pending = executor.submit(detect_and_return, frame,
                                                  args.columns, args.rows)
                if key in (10, 13):
                    if pending is not None:
                        print('角点检测中，请稍候。')
                    elif saved < args.min_views:
                        print(f'当前 {saved} 张；至少需要 {args.min_views} 张。')
                    else:
                        return solve_session(images_dir, args.session_dir, args)
    finally:
        executor.shutdown(wait=True)
        cv2.destroyAllWindows()
        receiver.destroy_node()
        rclpy.shutdown()


def detect_and_return(frame, columns, rows):
    return frame, detect_corners(frame[0], columns, rows)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera1/image',
                        help='ROS2 sensor_msgs/msg/Image 话题')
    parser.add_argument('--columns', type=int, default=11, help='横向内部角点数')
    parser.add_argument('--rows', type=int, default=8, help='纵向内部角点数')
    parser.add_argument('--square-mm', type=float,
                        help='实测单格边长，毫米；只求 K/D 可省略')
    parser.add_argument('--rotate-180', action=argparse.BooleanOptionalAction,
                        default=False,
                        help='预览和保存的图像旋转 180 度；相机已摆正，默认不旋转')
    parser.add_argument('--target-views', type=int, default=30,
                        help='达到此保存数量后自动计算')
    parser.add_argument('--min-views', type=int, default=20,
                        help='按回车提前计算所需的最少照片数')
    parser.add_argument('--session-dir', type=Path,
                        default=Path('data/camera') / datetime.now().strftime('session_%Y%m%d_%H%M%S'),
                        help='本次照片、采集记录和结果的独立目录')
    parser.add_argument('--resume', action='store_true', help='继续采集同一目录已有照片')
    parser.add_argument('--offline-images', type=Path,
                        help='离线模式：只对已有照片求解，不连接 ROS2 相机')
    parser.add_argument('--outlier-px', type=float,
                        help='可选：仅复算时剔除首次误差高于此值的照片')
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3:
        parser.error('棋盘内部角点每方向至少为 3')
    if args.square_mm is not None and args.square_mm <= 0:
        parser.error('--square-mm 必须大于 0')
    if args.min_views < 5 or args.target_views < args.min_views:
        parser.error('需满足 5 <= --min-views <= --target-views')
    if args.outlier_px is not None and args.outlier_px <= 0:
        parser.error('--outlier-px 必须大于 0')
    if args.offline_images is not None and not args.offline_images.is_dir():
        parser.error('--offline-images 不是目录')
    return args


def main():
    args = parse_args()
    if args.offline_images is not None:
        args.session_dir.mkdir(parents=True, exist_ok=True)
        solve_session(args.offline_images, args.session_dir, args)
    else:
        capture_live(args)


if __name__ == '__main__':
    main()
