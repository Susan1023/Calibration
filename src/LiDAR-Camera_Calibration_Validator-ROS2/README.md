<h1 align="center">LiDAR-Camera Calibration Validator</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-22314E" alt="ROS2 Humble"/>
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420" alt="Ubuntu 22.04"/>
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C" alt="C++17"/>
  <img src="https://img.shields.io/badge/License-GPL--3.0--or--later-3DA639" alt="GPL-3.0-or-later"/>
  <img src="https://img.shields.io/badge/Version-5.0.0-2F80ED" alt="Version 5.0.0"/>
</p>

<p align="center">
  <a href="https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator">LiDAR-Camera Calibration Validator ROS 1</a> |
  <a href="CHANGELOG.md">CHANGELOG</a> |
  <a href="README_EN.md">English</a>
</p>

▸ 本工具用于 LiDAR-相机联合标定结果的快速可视化校验，基于 ROS 实时同步图像与点云数据，支持畸变校正投影、边缘重合度与归一化互信息评估，并输出融合图像及指标统计结果

▸ 广泛支持各种混合固态式与非重复扫描雷达，并且不依赖于相机和激光雷达时间同步

▸ **🎉 What's New:** **已全面支持 ROS 2 Humble - 2026-03-07**


<p align="center">
  <img src="docs/LCCV.png" alt="LCCV Overview" width="100%"/>
</p>

<p align="center">
  <em>LiDAR-Camera Calibration Validator Overview</em>
</p>

---

## 🔄 系统流程

<p align="center">
  <img src="docs/system-pipeline.png" alt="System Pipeline" width="100%"/>
</p>

---

## 🧭 推荐标定工具

> [!IMPORTANT] 
> 在使用本工具前，**请务必先完成相机标定与联合标定**，更新标定参数 **[config/sample_calibration.yaml](config/sample_calibration.yaml)**

### 🚩相机标定

<details>
<summary><strong>推荐相关单目和双目相机标定相关教程</strong></summary>

- [ROS 相机标定详细教程](https://blog.csdn.net/2401_88008105/article/details/152665216?spm=1001.2014.3001.5506) **华北理工大学 HORIZON战队 雷达组 ROS 相机标定教程。**

- [ROS 单目相机标定官方教程](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) 介绍 ROS 1 下单目相机的标定方法，适合普通 USB 相机或工业相机内参标定。

- [ROS 双目相机标定官方教程](https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) 用于 ROS 1 下双目相机的联合标定流程，可用于获得左右相机参数及双目标定结果。

- [ROS2 单目相机标定官方教程](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_mono.html) 在 ROS 2 下单目相机标定工具的使用方法，包括启动标定节点和采集棋盘格图像。

- [ROS2 双目相机标定官方教程](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_stereo.html) 在 ROS 2 下双目相机标定流程，可用于完成左右目联合标定与参数求解。


<p align="center">
  <img src="docs/ROS-Calibration.png" alt="ROS Camera Calibration" width="90%"/>
</p>
<p align="center">
  <em>图 2：ROS 相机标定示例</em><br>
  <em>拍摄于 RM超级对抗赛前夕 2025 年 05 月 08 日 08:25 AM，应该是调了一通宵还没睡，哈哈</em><br>
  <em>真是段令人怀恋的日子😌</em>
</p>

</details>

---


### 🚩激光雷达-相机外参标定

<details>
<summary><strong>推荐相关激光雷达与相机外参标定工具</strong></summary>

- [direct_visual_lidar_calibration ](https://github.com/koide3/direct_visual_lidar_calibration)  直接把相机图像和激光雷达数据对齐，从而完成相机和激光雷达之间的标定，无需标定板，支持旋转式 LiDAR、Livox 这类非重复扫描 LiDAR、针孔、鱼眼、全向相机，同时支持 ROS 与 ROS2 （推荐）。
  
- [官方教程 | Official Tutorial](https://koide3.github.io/direct_visual_lidar_calibration/)  包含安装、数据准备、运行示例等详细步骤。
  
- [HKU FAST-Calib](https://github.com/hku-mars/FAST-Calib) 香港大学 MARS Lab 开源的快速激光雷达-相机外参标定工具，主打基于标定板的一秒级高精度外参标定。

- [HKU joint-lidar-camera-calib](https://github.com/hku-mars/joint-lidar-camera-calib) 香港大学 MARS Lab 开源的激光雷达-相机联合标定工具，支持在无标定板场景下同时估计相机内参和 LiDAR-相机外参。
  


<p align="center">
  <img src="docs/LiDAR-Camera-Calibration.png" alt="LiDAR-Camera Calibration" width="90%"/>
</p>
<p align="center">
  <em>联合标定效果</em><br>
  <em>拍摄于 2025 年 02 月 21 日 00:27 ，正式接手雷达站第一个任务</em><br>
  <em>老学长以前都是直接手算</em>
</p>

<p align="center">
  <img src="docs/LiDAR-Camera-Calibration_2.png" alt="LiDAR-Camera Calibration Process" width="90%"/>
</p>
<p align="center">
  <em>激光雷达与相机联合标定过程</em><br>
  <em>拍摄于 RM超级对抗赛前夕 2025 年 05 月 08 日 05:00 凌晨，还是这个通宵，一个人坐外面场地里疯狂调</em><br>
  <em>不知不觉间天亮了</em>
</p>

</details>

---

## 🧱 系统要求

### 依赖

- ROS2 Humble（Ubuntu 22.04）
- OpenCV 4.x
- PCL 1.10+
- Eigen3
- TBB (oneTBB)
- python3-tk（参数面板）
- Industrial Camera Driver (ROS2)
- LiDAR Driver (ROS2)

---

## 📦 安装

### 克隆项目

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator-ROS2.git
cd LiDAR-Camera_Calibration_Validator-ROS2
```

### 检查依赖

```bash
./scripts/quick_start.sh --check-deps
```

---

## 🚀 快速开始

### 一键启动

```bash
# 进入项目目录
cd ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2

# 编辑标定参数
vim ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2/config/sample_calibration.yaml

# 启动 LiDAR 与 Camera 驱动，发布话题

# 一键启动（自动编译并启动验证器）
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points

# latest 同步模式示例（适用于非重复扫描雷达）（用于去除时间同步依赖）
./scripts/quick_start.sh --sync-mode latest --latest-threshold 0.2 \
    -i /galaxy_camera/image_raw -c /livox/lidar
```

> [!NOTE]
> 若启动后出现 `/validator/fused_image` 的 `RELIABILITY_QOS_POLICY` 不兼容告警，通常表示**节点已正常运行**，但订阅端与发布端 QoS 不匹配。
>
> 当前默认发布为 `best_effort`，可选处理方式：
> 1. 启动时改为 `reliable`（推荐）：
>    ```bash
>    ./scripts/quick_start.sh --sync-mode latest --latest-threshold 0.2 \
>      --fused-qos reliable \
>      -i /galaxy_camera/image_raw -c /livox/lidar
>    ```
> 2. 将订阅端（如 RViz2）QoS Reliability 改为 `Best Effort`。

### 高级选项

```bash
./scripts/quick_start.sh --check-deps
./scripts/quick_start.sh --force-compile
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points
./scripts/quick_start.sh -f /path/to/your/calibration.yaml
./scripts/quick_start.sh --sync-mode latest --latest-threshold 0.2
./scripts/quick_start.sh --fused-qos reliable
./scripts/quick_start.sh --help

# 如需关闭自动参数面板
ros2 launch lidar_cam_validator validator.launch.py open_param_panel:=false
```

---

## 🛠️ 手动启动

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws
colcon build --symlink-install --packages-select lidar_cam_validator
source ~/ros2_ws/install/setup.bash

ros2 launch lidar_cam_validator validator.launch.py

# 新终端查看参数列表（命令行调参）
ros2 param list /validator_node
```

---

## ⚙️ 标定参数配置

编辑标定文件：

```bash
vim ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2/config/sample_calibration.yaml
```

格式示例：

```yaml
K_0: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]

C_0: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [k1, k2, p1, p2, k3]

E_0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [R11, R12, R13, tx,
          R21, R22, R23, ty,
          R31, R32, R33, tz,
          0,   0,   0,   1]
```

---

## 📡 话题配置

### 命令行指定（推荐）

```bash
./scripts/quick_start.sh -i /your_camera/image_raw -c /your_lidar/cloudpoints
```

### 配置文件修改

```bash
vim ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2/config/settings.yaml
```

```yaml
validator_node:
  ros__parameters:
    image_topic: "/camera/image_raw"
    cloud_topic: "/cloudpoints"
    fused_topic: "/validator/fused_image"
    fused_image_qos_reliability: "best_effort"   # best_effort | reliable
    info_topic: "/validator/validation_info"
    calibration_file: "$(find lidar_cam_validator)/config/sample_calibration.yaml"
    sync_mode: "approximate"
    latest_pair_time_threshold: 0.5
```

---

## ⏱️ 同步策略与非重复雷达支持

### 时间同步模式

- `sync_mode=approximate`（默认）：使用 ApproximateTime，同步时间接近的图像与点云。
- `sync_mode=latest` 或 `latest_pair`：按到达时间匹配最近一对消息，不依赖硬件时间同步。
- `latest_pair_time_threshold`：在 `latest` 模式下，用于提示图像与点云到达间隔的阈值（秒）。

### 点云累积（Livox 非重复扫描雷达）

对于 Livox Mid-70、Horizon 等非重复扫描雷达，单帧点云通常较稀疏，建议启用多帧累积：

1. 使用 `livox_ros_driver` + `livox_repub`（或其他转换节点）将 `CustomMsg` 转换为 `PointCloud2`。
2. 在 ROS2 参数中开启 `enable_accumulation`，并按需调整以下参数：
   `accumulation_time_sec`、`accumulation_frames`、`accumulation_max_points`。

<p align="center">
  <img src="docs/Weixin Image_20251127171946_11_1.png" alt="Livox Mid70-1" width="45%" />
  <img src="docs/Weixin Image_20251127171953_12_1.png" alt="Livox Mid70-2" width="45%" />
</p>

<p align="center">
  <em>LIVOX 非重复扫描激光雷达示例</em>
</p>

---


> [!NOTE]
>
> **1）为什么“目标越远，点云缝隙看起来越吻合”？**
>
> 这是正常的物理现象，核心原因是视差。
>
> 在近距离时，相机与雷达的平移基线带来的视差更明显；即使标定误差很小，也会放大为较大的像素偏移。
>
> 随着距离增加，平移误差影响迅速减弱，旋转误差逐渐占主导，因此视觉上更容易“对齐”。
>
> 建议将评估距离放在 **8-10 米**，至少 **5 米**。

> [!NOTE]
> **2）为什么评分长期显示 `Poor`？**
>
> 当前评分方法不具备强自适应能力，主要用于辅助可视化观察。
>
> 若远处目标轮廓与点云边缘在视觉上贴合良好，通常可认为标定结果已经可用。
>
> Mid-70 的花瓣状扫描不如传统旋转雷达规则，可能会影响依赖密集扫描线的评估算法表现。
>
> 同时，Livox 近距离前景点稀疏也会使边缘匹配度下降，`edge_overlap_score` 偏低属常见现象。
>
> 请勿过度依赖自动评分，建议结合可视化结果综合判断。

---

## 🎛️ 动态参数说明

使用 ROS2 param 进行实时调节，修改立即生效，无需重启。

<details>
<summary><strong>Visualization</strong></summary>

| 参数名 | 类型 | 范围 | 默认值 | 说明 |
|:------:|:----:|:----:|:------:|:----:|
| point_size | int | 1-8 | 3 | 点大小（像素） |
| min_depth | double | 0.1-5.0 | 0.5 | 颜色映射最小深度（米） |
| max_depth | double | 5.0-200.0 | 50.0 | 颜色映射最大深度（米） |
| show_statistics | bool | - | true | 显示统计信息 |
| show_edge_overlay | bool | - | false | 显示边缘叠加 |
| show_depth_colorbar | bool | - | true | 显示深度色条 |

</details>

<details>
<summary><strong>Performance</strong></summary>

| 参数名 | 类型 | 范围 | 默认值 | 说明 |
|:------:|:----:|:----:|:------:|:----:|
| enable_downsampling | bool | - | false | 启用下采样 |
| max_points | int | 10000-1000000 | 1000000 | 最大处理点数 |
| enable_accumulation | bool | - | false | 启用多帧累积 |
| accumulation_time_sec | double | 0.01-1.0 | 0.1 | 累积时间窗口（秒） |
| accumulation_frames | int | 1-20 | 3 | 累积帧上限 |
| accumulation_max_points | int | 1000-2000000 | 200000 | 累积后点数上限 |

</details>

<details>
<summary><strong>Filtering</strong></summary>

| 参数名 | 类型 | 范围 | 默认值 | 说明 |
|:------:|:----:|:----:|:------:|:----:|
| filter_by_distance | bool | - | true | 启用距离过滤 |
| min_distance | double | 0.1-2.0 | 0.3 | 最近距离（米） |
| max_distance | double | 10.0-200.0 | 100.0 | 最远距离（米） |

</details>

<details>
<summary><strong>Metrics</strong></summary>

| 参数名 | 类型 | 范围 | 默认值 | 说明 |
|:------:|:----:|:----:|:------:|:----:|
| enable_metrics | bool | - | true | 启用指标计算 |
| edge_threshold | int | 10-200 | 50 | Canny 阈值 |
| edge_distance_threshold | double | 1.0-20.0 | 5.0 | 边缘距离阈值（像素） |
| min_points_for_nmi | int | 50-1000 | 100 | 计算 NMI 的最小投影点数 |

</details>

<details>
<summary><strong>Control</strong></summary>

| 参数名 | 类型 | 说明 |
|:------:|:----:|:----:|
| reset_to_defaults | bool | 一键恢复默认参数 |

</details>

---

## 📤 输出

### 可视化窗口

- 融合图像：点云投影叠加到相机图像
- 深度颜色编码：近红远蓝的深度映射
- 颜色条：右上角深度范围指示
- 统计信息：左上角处理与指标统计

<p align="center">
  <img src="docs/VisualizationGUI.png" alt="Visualization GUI" width="85%"/>
</p>
<p align="center">
  <em>可视化界面</em>
</p>

### ROS 话题

- /validator/fused_image：融合图像（sensor_msgs/Image）
- /validator/validation_info：校验信息 JSON（std_msgs/String）

> [!NOTE]
> 若订阅 `/validator/fused_image` 出现 QoS 不兼容告警，可将参数 `fused_image_qos_reliability` 从 `best_effort` 切换为 `reliable`。

---

## 🗂️ 项目结构

```
LiDAR-Camera_Calibration_Validator-ROS2/
├── include/
│   └── calibration_validator.h
├── src/
│   ├── calibration_validator.cpp
│   └── validator_node.cpp
├── scripts/
│   └── quick_start.sh
├── config/
│   ├── sample_calibration.yaml
│   └── settings.yaml
├── launch/
│   └── validator.launch.py
├── docs/
│   ├── system-pipeline.png
│   └── ...
├── README.md
└── CMakeLists.txt
```


---

## 🤝 贡献指南

1. Fork 项目到个人仓库
2. 创建功能分支：`git checkout -b feature/your-feature`
3. 提交更改：`git commit -am 'Add some feature'`
4. 推送分支：`git push origin feature/your-feature`
5. 创建 Pull Request

---

## 📝 TODO

- [x] 支持 Livox 系列激光雷达
- [x] 去除外部时间同步依赖
- [x] 支持 ROS2
- [ ] 支持 ROS-Docker
- [ ] 支持 ROS2-Docker

---

## ⚖️ 许可证

本项目采用 GPL-3.0-or-later 许可，详见 [LICENSE](LICENSE)。

### 您的义务

- Copyleft 保护：衍生作品必须使用相同许可证
- 源码公开：分发时需提供源码或获取方式
- 许可证保留：保留原始许可与版权声明
- 修改声明：明确标注对原始代码的修改

---

## 🌟 致谢

- [ROS 社区](https://www.ros.org/)
- [OpenCV](https://opencv.org/) 
- [PCL](https://pointclouds.org/)
- Dr. Kenji Koide  [`direct_visual_lidar_calibration`](https://github.com/koide3/direct_visual_lidar_calibration)
---

## 📮 联系方式

如有问题或建议，请通过以下方式联系：

**项目作者** : YAO YUZHUO

**电子邮箱** : yaoyuzhuo6@gmail.com

---

<p align="center">
  <b>LiDAR-Camera Calibration Validator v5.0.0</b> - 激光雷达-相机联合标定结果快速验证！ 🚀
</p>
