# 从零标定：Livox Avia 与两台单目相机

本指南以新机器 `git clone` 本仓库为起点。所有命令在仓库根目录执行；每开一个新终端，先运行：

```bash
cd /path/to/livox-calibration       # 改成 clone 后的实际路径
export ROOT="$PWD"
export RUN=20261121_01              # 本轮唯一编号；所有终端保持一致
```

## 0. 仓库之外的必要条件

以下内容**不在本仓库内，也不会随 git clone 下载**：

1. ROS 2 Jazzy、`colcon`、OpenCV/`cv_bridge`、Python 3 的 `venv` 和 PyYAML。相机采集和验证器均按 Jazzy 编写。
2. **Livox Avia 的 ROS 2 驱动**及其 SDK、雷达与主机的有线网络配置。驱动必须发布 `/livox/lidar`，类型为 `sensor_msgs/msg/PointCloud2`。本仓库没有可直接运行的雷达驱动启动命令。
3. **两台海康相机的 ROS 2 驱动**及海康 MVS SDK。驱动必须发布 `/camera1/image` 和 `/camera2/image`，类型均为 `sensor_msgs/msg/Image`；图像须为原始正向、1440×1080。相机驱动源码不在本仓库，因此本指南不提供虚构的 `ros2 launch` 命令。按所安装驱动的说明启动，并分别绑定两台相机的序列号。
4. 棋盘格和四圆孔 ArUco 标定板。下文默认棋盘格为 **11×8 个内部角点**。ArUco 板的尺寸参数必须按实物核对。
5. Docker。`fast-calib:noetic` 镜像由本仓库的 Dockerfile 构建，无须别人取得本机镜像。

本仓库的 `data/`、`.rosbags-deps/`、`build/`、`install/` 均被 `.gitignore` 排除。新机器会重新采集数据、安装转换依赖并编译。

`src/LiDAR-Camera_Calibration_Validator-ROS2/` 已作为普通源码目录纳入本仓库，包含本地修改。发布后请用新目录实际 clone 一次，确认这些文件存在。

## 1. 安装仓库内代码所需环境

安装 ROS 2 Jazzy 后，在仓库根目录执行：

```bash
python3 -m venv .venv-rosbags
.venv-rosbags/bin/python -m pip install 'rosbags==0.11.9' 'PyYAML>=6,<7'
docker build -t fast-calib:noetic -f docker/fast-calib-noetic.Dockerfile .
```

如果后面要运行实时投影验证器，先编译 ROS 2 包。显式指定系统 Python，避免 Conda Python 缺少 ROS 包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lidar_cam_validator \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 2. 启动传感器并核对话题

在三个独立终端，按**外部驱动**的安装说明启动 Livox 和两台相机。三个驱动需持续运行。然后在第四个终端：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t | rg '/livox/lidar|/camera1/image|/camera2/image'
ros2 topic hz /livox/lidar
ros2 topic hz /camera1/image
ros2 topic hz /camera2/image
```

三个话题都要有消息。若外部驱动发布了不同名称，请在驱动启动时重映射到上述名称；`convert_fast_calib_scene.py` 当前固定读取 `/livox/lidar`。

标定期间固定相机和雷达；锁定分辨率、焦距、对焦。重新标定时使用新的 `RUN`，避免覆盖旧数据。

## 3. 分别采集棋盘格并求内参

移动棋盘格覆盖画面中心、四角、远近和不同倾角；每次停稳后按空格保存。达到 30 张后自动求解。两台相机分开运行：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/capture_and_calibrate_camera.py \
  --topic /camera1/image --columns 11 --rows 8 --target-views 30 \
  --session-dir "data/runs/$RUN/camera/camera1"

/usr/bin/python3 scripts/capture_and_calibrate_camera.py \
  --topic /camera2/image --columns 11 --rows 8 --target-views 30 \
  --session-dir "data/runs/$RUN/camera/camera2"
```

如果实际棋盘内部角点不是 11×8，修改两个命令的 `--columns` 和 `--rows`。检查两个目录各自的 `camera_intrinsics.yaml` 和 `camera_calibration_report.md`；不能混用两台相机的照片。

## 4. 采集三个独立板位姿

每个场景让两台相机都看见四圆孔 ArUco 板。建议板距离雷达约 2.3 m、3.2 m、4.1 m，并改变板的朝向。每场景按以下顺序操作，以 `scene01` 为例。

**录制终端：**

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag record -o "data/runs/$RUN/scene01_ros2" /livox/lidar
```

录制开始约 6 秒后，**拍照终端**运行：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/capture_fast_calib_images.py \
  --output-dir "data/runs/$RUN/scene01_fast"
```

点击预览窗，按空格同时保存两台相机照片；再等约 6 秒，在录制终端按 Ctrl+C。拍照和录制期间板保持静止。把两个命令中的 `scene01` 改成 `scene02`、`scene03`，每次先换板位姿再重复。目录名不能复用。

## 5. 转成 FAST-Calib 需要的 ROS1 bag

```bash
for scene in scene01 scene02 scene03; do
  .venv-rosbags/bin/python scripts/convert_fast_calib_scene.py \
    "data/runs/$RUN/${scene}_ros2" "data/runs/$RUN/${scene}_fast"
done
```

每个 `scene##_fast/` 应有 `camera1_image.png`、`camera2_image.png`、`image_manifest.csv` 和 `lidar.bag`。转换脚本拒绝覆盖已有 bag。

## 6. 从本轮新内参生成六份 FAST-Calib 配置

先核对标定板实物：模板按 ArUco 边长 0.20 m、ArUco 中心半间距 0.55 × 0.35 m、圆孔中心完整间距 0.50 × 0.40 m、孔半径 0.12 m 编写。若不同，修改模板或生成后的六份配置。

以下裁剪范围只适用于约 2.3/3.2/4.1 m 的板位；按实际雷达坐标修改 `bounds`：

```bash
export RUN
.venv-rosbags/bin/python - <<'PY'
import os
from pathlib import Path
import yaml

base = Path('data/runs') / os.environ['RUN']
template = Path('src/FAST-Calib/config/qr_params.example.yaml')
config_dir = base / 'config'
config_dir.mkdir(parents=True, exist_ok=True)

# x_min, x_max, y_min, y_max, z_min, z_max，单位：米
bounds = {
    'scene01': (2.15, 2.50, -0.9, 0.9, -1.5, -0.1),
    'scene02': (2.70, 3.60, -1.0, 1.0, -1.6, -0.1),
    'scene03': (3.60, 4.55, -1.0, 1.0, -1.6, -0.1),
}

for cam in ('camera1', 'camera2'):
    intrinsics = yaml.safe_load((base / 'camera' / cam / 'camera_intrinsics.yaml').read_text())
    K, D = intrinsics['K'], intrinsics['D']
    for scene, crop in bounds.items():
        cfg = yaml.safe_load(template.read_text())
        cfg.update(dict(
            fx=K[0][0], fy=K[1][1], cx=K[0][2], cy=K[1][2],
            k1=D[0], k2=D[1], p1=D[2], p2=D[3], k3=D[4],
            rotate_180=False,
            bag_path=f'/calib_ws/project/{base}/{scene}_fast/lidar.bag',
            image_path=f'/calib_ws/project/{base}/{scene}_fast/{cam}_image.png',
            output_path=f'/calib_ws/project/{base}/results/{cam}',
            min_detected_markers=4,
        ))
        cfg.update(zip(('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'), crop))
        path = config_dir / f'{cam}_{scene}.yaml'
        path.write_text(yaml.safe_dump(cfg, sort_keys=False))
        print(path)
PY
```

## 7. 求解两台相机外参

对每台相机，先跑三个单场景，再跑多场景。两台相机写入不同结果目录：

```bash
for cam in camera1 camera2; do
  for scene in scene01 scene02 scene03; do
    docker run --rm --net host -v "$ROOT:/calib_ws/project" fast-calib:noetic \
      bash -lc "source /opt/ros/noetic/setup.bash; source /calib_ws/devel/setup.bash; \
        roslaunch fast_calib calib.launch rviz:=false \
        params_file:=/calib_ws/project/data/runs/$RUN/config/${cam}_${scene}.yaml"
  done

  docker run --rm --net host -v "$ROOT:/calib_ws/project" fast-calib:noetic \
    bash -lc "source /opt/ros/noetic/setup.bash; source /calib_ws/devel/setup.bash; \
      roslaunch fast_calib multi_calib.launch rviz:=false \
      params_file:=/calib_ws/project/data/runs/$RUN/config/${cam}_scene03.yaml"
done
```

结果：

```text
data/runs/<RUN>/results/camera1/multi_calib_result.txt
data/runs/<RUN>/results/camera2/multi_calib_result.txt
```

文件中的 `Rcl` 和 `Pcl` 是雷达到相机的旋转与平移。检查每个场景生成的 `qr_detect.png`，确认 ArUco 和圆孔检测正确。检测失败时先修正该场景的点云裁剪范围，再运行求解。

## 8. 独立场景验证

另采一组**没有参与上述求解**的场景，用点云投影检查图像边缘、标定板位置是否吻合。仓库内验证器源码在 `src/LiDAR-Camera_Calibration_Validator-ROS2/`，它读取 OpenCV 格式的 `K_0`、`C_0`、`E_0`。自带的 `sample_calibration.yaml` 是其他设备的示例参数，不能用于本轮结果。

先把本轮内参和 FAST-Calib 结果转换为验证器配置：

```bash
export RUN
.venv-rosbags/bin/python - <<'PY'
import os
from pathlib import Path
import yaml

base = Path('data/runs') / os.environ['RUN']
for cam in ('camera1', 'camera2'):
    intr = yaml.safe_load((base / 'camera' / cam / 'camera_intrinsics.yaml').read_text())
    extr = yaml.safe_load((base / 'results' / cam / 'multi_calib_result.txt').read_text())
    K, D, R, P = intr['K'], intr['D'], extr['Rcl'], extr['Pcl']
    E = [R[i*3:(i+1)*3] + [P[i]] for i in range(3)] + [[0., 0., 0., 1.]]

    def matrix(name, rows, cols, values):
        flat = [v for row in values for v in row]
        return (f'{name}: !!opencv-matrix\n   rows: {rows}\n   cols: {cols}\n'
                f'   dt: d\n   data: [{", ".join(map(str, flat))}]\n')

    content = ('%YAML:1.0\n'
               + matrix('K_0', 3, 3, K)
               + matrix('C_0', 1, 5, [D])
               + matrix('E_0', 4, 4, E))
    path = base / 'results' / cam / 'validator.yaml'
    path.write_text(content)
    print(path)
PY
```

保持外部的三路传感器驱动运行。验证相机 1：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_cam_validator validator.launch.py \
  image_topic:=/camera1/image cloud_topic:=/livox/lidar \
  calibration_file:="$ROOT/data/runs/$RUN/results/camera1/validator.yaml" \
  sync_mode:=latest latest_pair_time_threshold:=0.2 \
  fused_image_qos_reliability:=reliable open_param_panel:=false
```

在融合图像窗口检查投影。按 Ctrl+C 停止后，将命令中的两处 `camera1` 改为 `camera2`，验证第二台相机。融合图像也发布在 `/validator/fused_image`。
