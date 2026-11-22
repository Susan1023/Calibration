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
  <a href="README.md">中文</a>
</p>

▸ This tool provides fast visual validation for LiDAR-camera calibration results. It synchronizes images and point clouds in real time with ROS, supports distortion-corrected projection, edge overlap and normalized mutual information metrics, and outputs fused images with statistics.

▸ It supports a wide range of hybrid solid-state and non-repetitive scanning LiDARs, and does not rely on hardware time synchronization between camera and LiDAR.

▸ **🎉 What's New:** Fully supports ROS 2 Humble - 2026-03-07


<p align="center">
  <img src="docs/LCCV.png" alt="LCCV Overview" width="100%"/>
</p>

<p align="center">
  <em>LiDAR-Camera Calibration Validator Overview</em>
</p>

---

## 🔄 System Pipeline

<p align="center">
  <img src="docs/system-pipeline.png" alt="System Pipeline" width="100%"/>
</p>

---

## 🧭 Recommended Calibration Tools

> [!IMPORTANT]
> Before using this tool, **please complete camera calibration and LiDAR-camera calibration first**, and update **[config/sample_calibration.yaml](config/sample_calibration.yaml)**.

### 🚩 Camera Calibration

<details>
<summary><strong>Recommended monocular and stereo camera calibration tutorials</strong></summary>

- [ROS Monocular Camera Calibration (Official)](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration): ROS 1 monocular calibration tutorial for USB and industrial cameras.

- [ROS Stereo Camera Calibration (Official)](https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration): ROS 1 stereo calibration workflow for left/right intrinsics and stereo parameters.

- [Detailed ROS Camera Calibration Tutorial](https://blog.csdn.net/2401_88008105/article/details/152665216?spm=1001.2014.3001.5506): **HORIZON Team (NCUST) radar group tutorial.**

- [ROS 2 Monocular Camera Calibration (Official)](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_mono.html): How to run monocular calibration in ROS 2.

- [ROS 2 Stereo Camera Calibration (Official)](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_stereo.html): ROS 2 stereo calibration workflow.


<p align="center">
  <img src="docs/ROS-Calibration.png" alt="ROS Camera Calibration" width="90%"/>
</p>
<p align="center">
  <em>Figure 2: ROS camera calibration example</em><br>
  <em>Captured before RM Super Confrontation, May 08, 2025, 08:25 AM - likely after an all-night tuning session.</em><br>
  <em>Still a memorable period 😌</em>
</p>

</details>

---


### 🚩 LiDAR-Camera Extrinsic Calibration

<details>
<summary><strong>Recommended LiDAR-camera extrinsic calibration tools</strong></summary>

- [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration): Directly aligns camera images with LiDAR data for calibration without a checkerboard. Supports spinning LiDAR, Livox non-repetitive LiDAR, pinhole/fisheye/omni cameras, and both ROS and ROS 2 (recommended).

- [Official Tutorial](https://koide3.github.io/direct_visual_lidar_calibration/): Includes installation, data preparation, and running examples.

- [HKU FAST-Calib](https://github.com/hku-mars/FAST-Calib): HKU MARS Lab fast LiDAR-camera extrinsic calibration tool, focused on high-precision board-based calibration.

- [HKU joint-lidar-camera-calib](https://github.com/hku-mars/joint-lidar-camera-calib): HKU MARS Lab calibration tool that can jointly estimate camera intrinsics and LiDAR-camera extrinsics, including boardless scenarios.


<p align="center">
  <img src="docs/LiDAR-Camera-Calibration.png" alt="LiDAR-Camera Calibration" width="90%"/>
</p>
<p align="center">
  <em>Extrinsic calibration result</em><br>
  <em>Captured on Feb 21, 2025, 00:27 - first radar-station task after taking over.</em><br>
  <em>Seniors used to compute this manually.</em>
</p>

<p align="center">
  <img src="docs/LiDAR-Camera-Calibration_2.png" alt="LiDAR-Camera Calibration Process" width="90%"/>
</p>
<p align="center">
  <em>LiDAR-camera calibration process</em><br>
  <em>Captured before RM Super Confrontation, May 08, 2025, 05:00 AM - another all-night tuning session outdoors.</em><br>
  <em>And then sunrise came.</em>
</p>

</details>

---

## 🧱 Requirements

### Dependencies

- ROS2 Humble (Ubuntu 22.04)
- OpenCV 4.x
- PCL 1.10+
- Eigen3
- TBB (oneTBB)
- python3-tk (parameter panel)
- Industrial Camera Driver (ROS2)
- LiDAR Driver (ROS2)

---

## 📦 Installation

### Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator-ROS2.git
cd LiDAR-Camera_Calibration_Validator-ROS2
```

### Check Dependencies

```bash
./scripts/quick_start.sh --check-deps
```

---

## 🚀 Quick Start

### One-Command Launch

```bash
# Go to the project directory
cd ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2

# Edit calibration parameters
vim ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2/config/sample_calibration.yaml

# Start LiDAR and camera drivers first

# One-command launch (auto-build + run validator)
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points

# latest sync mode example (for non-repetitive LiDAR)
./scripts/quick_start.sh --sync-mode latest --latest-threshold 0.2 \
    -i /galaxy_camera/image_raw -c /livox/lidar
```

### Advanced Options

```bash
./scripts/quick_start.sh --check-deps
./scripts/quick_start.sh --force-compile
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points
./scripts/quick_start.sh -f /path/to/your/calibration.yaml
./scripts/quick_start.sh --sync-mode latest --latest-threshold 0.2
./scripts/quick_start.sh --fused-qos reliable
./scripts/quick_start.sh --help

# Disable auto parameter panel if needed
ros2 launch lidar_cam_validator validator.launch.py open_param_panel:=false
```

---

## 🛠️ Manual Launch

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws
colcon build --symlink-install --packages-select lidar_cam_validator
source ~/ros2_ws/install/setup.bash

ros2 launch lidar_cam_validator validator.launch.py

# In another terminal, inspect parameters (CLI tuning)
ros2 param list /validator_node
```

---

## ⚙️ Calibration Parameters

Edit calibration file:

```bash
vim ~/ros2_ws/src/LiDAR-Camera_Calibration_Validator-ROS2/config/sample_calibration.yaml
```

Format example:

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

## 📡 Topic Configuration

### Command Line (Recommended)

```bash
./scripts/quick_start.sh -i /your_camera/image_raw -c /your_lidar/cloudpoints
```

### Configure via File

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

## ⏱️ Sync Strategy and Non-Repetitive LiDAR Support

### Time Synchronization Modes

- `sync_mode=approximate` (default): uses ApproximateTime to pair image and cloud messages with close timestamps.
- `sync_mode=latest` or `latest_pair`: pairs by latest arrival time without hardware time synchronization.
- `latest_pair_time_threshold`: warning threshold (seconds) for arrival-time gap in `latest` mode.

### Point Cloud Accumulation (Livox Non-Repetitive LiDAR)

For Livox Mid-70, Horizon, and similar non-repetitive LiDARs, single-frame clouds are often sparse. Multi-frame accumulation is recommended:

1. Use `livox_ros_driver` + `livox_repub` (or equivalent converter) to convert `CustomMsg` to `PointCloud2`.
2. Enable `enable_accumulation` in ROS 2 parameters and tune:
   `accumulation_time_sec`, `accumulation_frames`, and `accumulation_max_points`.

<p align="center">
  <img src="docs/Weixin Image_20251127171946_11_1.png" alt="Livox Mid70-1" width="45%" />
  <img src="docs/Weixin Image_20251127171953_12_1.png" alt="Livox Mid70-2" width="45%" />
</p>

<p align="center">
  <em>LIVOX non-repetitive LiDAR examples</em>
</p>

---


> [!NOTE]
>
> **1) Why does alignment seem better at longer distance?**
>
> This is normal and mainly due to parallax.
>
> At short range, the translation baseline between camera and LiDAR produces stronger parallax; even small calibration errors can cause large pixel offsets.
>
> As distance increases, translation-induced error decays quickly and rotational error becomes dominant, so visual alignment often looks better.
>
> Recommended evaluation distance: **8-10 m**, minimum **5 m**.

> [!NOTE]
> **2) Why does the score stay `Poor` for a long time?**
>
> The current scoring method is not strongly adaptive and is mainly intended for visual assistance.
>
> If distant contours and projected cloud edges align well visually, calibration is usually usable.
>
> Mid-70's petal-like scanning pattern is less regular than spinning LiDAR and may degrade algorithms relying on dense scan lines.
>
> In addition, sparse near-field foreground points in Livox scans can reduce edge matching and make `edge_overlap_score` appear low.
>
> Do not over-rely on automatic scores; combine score and visual inspection.

---

## 🎛️ Dynamic Parameter Reference

Use ROS 2 parameters for real-time tuning. Changes take effect immediately without node restart.

<details>
<summary><strong>Visualization</strong></summary>

| Parameter | Type | Range | Default | Description |
|:---------:|:----:|:-----:|:-------:|:-----------:|
| point_size | int | 1-8 | 3 | Point size (pixels) |
| min_depth | double | 0.1-5.0 | 0.5 | Min depth for color mapping (m) |
| max_depth | double | 5.0-200.0 | 50.0 | Max depth for color mapping (m) |
| show_statistics | bool | - | true | Show statistics overlay |
| show_edge_overlay | bool | - | false | Show edge overlay |
| show_depth_colorbar | bool | - | true | Show depth colorbar |

</details>

<details>
<summary><strong>Performance</strong></summary>

| Parameter | Type | Range | Default | Description |
|:---------:|:----:|:-----:|:-------:|:-----------:|
| enable_downsampling | bool | - | false | Enable downsampling |
| max_points | int | 10000-1000000 | 1000000 | Max processed points |
| enable_accumulation | bool | - | false | Enable multi-frame accumulation |
| accumulation_time_sec | double | 0.01-1.0 | 0.1 | Accumulation window (s) |
| accumulation_frames | int | 1-20 | 3 | Max accumulated frames |
| accumulation_max_points | int | 1000-2000000 | 200000 | Max points after accumulation |

</details>

<details>
<summary><strong>Filtering</strong></summary>

| Parameter | Type | Range | Default | Description |
|:---------:|:----:|:-----:|:-------:|:-----------:|
| filter_by_distance | bool | - | true | Enable distance filtering |
| min_distance | double | 0.1-2.0 | 0.3 | Min distance (m) |
| max_distance | double | 10.0-200.0 | 100.0 | Max distance (m) |

</details>

<details>
<summary><strong>Metrics</strong></summary>

| Parameter | Type | Range | Default | Description |
|:---------:|:----:|:-----:|:-------:|:-----------:|
| enable_metrics | bool | - | true | Enable metric calculation |
| edge_threshold | int | 10-200 | 50 | Canny threshold |
| edge_distance_threshold | double | 1.0-20.0 | 5.0 | Edge distance threshold (pixels) |
| min_points_for_nmi | int | 50-1000 | 100 | Min points for NMI calculation |

</details>

<details>
<summary><strong>Control</strong></summary>

| Parameter | Type | Description |
|:---------:|:----:|:-----------:|
| reset_to_defaults | bool | Reset all parameters to defaults |

</details>

---

## 📤 Output

### Visualization Window

- Fused image: projected point cloud over camera image
- Depth colormap: near-red / far-blue mapping
- Color bar: depth range indicator (top-right)
- Statistics overlay: processing and metric information (top-left)

<p align="center">
  <img src="docs/VisualizationGUI.png" alt="Visualization GUI" width="85%"/>
</p>
<p align="center">
  <em>Visualization interface</em>
</p>

### ROS Topics

- /validator/fused_image: fused image (`sensor_msgs/Image`)
- /validator/validation_info: validation info JSON (`std_msgs/String`)

> [!NOTE]
> If you get QoS incompatibility warnings while subscribing to `/validator/fused_image`, switch `fused_image_qos_reliability` from `best_effort` to `reliable`.

---

## 🗂️ Project Structure

```text
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

## 🤝 Contributing

1. Fork this repository.
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push branch: `git push origin feature/your-feature`
5. Open a Pull Request.

---

## 📝 TODO

- [x] Livox LiDAR support
- [x] Remove external time-sync dependency
- [x] ROS2 support
- [ ] ROS-Docker support
- [ ] ROS2-Docker support

---

## ⚖️ License

This project is licensed under GPL-3.0-or-later. See [LICENSE](LICENSE) for details.

### Your Obligations

- Copyleft: derivative works must use the same license.
- Source disclosure: provide source code or source access when distributing.
- License preservation: retain original license and copyright notices.
- Modification notice: clearly indicate modifications.

---

## 🌟 Acknowledgments

- [ROS Community](https://www.ros.org/)
- [OpenCV](https://opencv.org/)
- [PCL](https://pointclouds.org/)
- Dr. Kenji Koide's [`direct_visual_lidar_calibration`](https://github.com/koide3/direct_visual_lidar_calibration)

---

## 📮 Contact

If you have any questions or suggestions, please contact:

**Author**: YAO YUZHUO

**Email**: yaoyuzhuo6@gmail.com

---

<p align="center">
  <b>LiDAR-Camera Calibration Validator v5.0.0</b> - Fast validation for LiDAR-camera calibration results! 🚀
</p>
