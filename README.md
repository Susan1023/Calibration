# Livox Avia 与双单目相机联合标定

本仓库提供两台相机的棋盘格内参采集、静态板场景采集、ROS 2 点云 bag 转换、FAST-Calib 外参求解，以及 ROS 2 点云投影验证器。

**从新机器开始，请直接阅读 [从零标定指南](docs/from-scratch-calibration.md)。** 指南按终端顺序给出命令，并列出仓库之外必须准备的硬件驱动和 SDK。

## 流程

1. 固定两台相机与雷达，分别采集棋盘格照片并求内参。
2. 在至少三个不同板位姿下，同步保存两台相机照片与 Livox 点云。
3. 将 ROS 2 点云 bag 转换为 FAST-Calib 可读取的 ROS 1 bag。
4. 对每台相机分别运行三个单场景求解，再运行多场景求解。
5. 用未参与求解的新场景检查点云投影。

## 仓库内容

| 路径 | 用途 |
| --- | --- |
| `scripts/capture_and_calibrate_camera.py` | 棋盘格拍照与单相机内参求解 |
| `scripts/capture_fast_calib_images.py` | 同时保存两台相机的原始照片 |
| `scripts/convert_fast_calib_scene.py` | 按拍照时间裁剪点云并转换成 ROS 1 bag |
| `src/FAST-Calib/` | 港大 FAST-Calib 源码，求雷达到相机的外参 |
| `src/LiDAR-Camera_Calibration_Validator-ROS2/` | ROS 2 点云投影验证器源码 |
| `docker/fast-calib-noetic.Dockerfile` | 构建 FAST-Calib 的 ROS 1 Noetic 环境 |

Livox 和海康相机的 ROS 2 驱动、海康 MVS SDK、采集数据及本机编译结果不在仓库内。`data/` 被 Git 忽略，每位使用者需采集自己的数据。标定板尺寸、相机分辨率和话题名称都应按现场设备核对。

FAST-Calib 保留其 GPL-2.0 许可证；验证器保留其 GPL-3.0-or-later 许可证。
