# ROS2 Jazzy 安装与运行

本文适用于 Ubuntu 24.04、ROS2 Jazzy、C++17、Python 3.12 和 amd64。先按 [Jazzy 官方安装说明](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst)配置 ROS apt 源；安装依赖需要 sudo 和网络。以下命令均在项目根目录（当前为 `/home/scl/work/slam/SchurVINS`）执行；构建、安装和日志统一保存在项目内的 `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log` 下，并按配置分子目录。数据路径须替换为实际路径。其他架构须使用对应的系统 OpenCV CMake 目录。使用系统 Eigen 3.4、OpenCV 4.6、Ceres 2.2、yaml-cpp 0.8、Jazzy GTSAM 4.2 和 `/usr/bin/python3`，避免混用用户安装的 OpenCV 或 Conda。

## 依赖与资源

```bash
# 更新软件包索引
sudo apt update
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 无 | 无额外参数 | 软件源索引 | 不适用 |

```bash
# 安装构建、ROS 和运行依赖
sudo apt install \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libopencv-dev \
  libceres-dev \
  libyaml-cpp-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libopengv-dev \
  libglew-dev \
  libboost-all-dev \
  libblas-dev \
  liblapack-dev \
  libsuitesparse-dev \
  ros-jazzy-gtsam \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rclcpp-components \
  ros-jazzy-pcl-conversions \
  ros-jazzy-rosbag2-py \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-rqt-gui-py \
  ros-jazzy-python-qt-binding \
  ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-cmake-pytest \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-yaml \
  python3-psutil \
  python3-colorama \
  python3-pytest \
  xvfb
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 软件包名 | 指定依赖 | 输入值 | 示例列出本项目所需软件包 |

在仅加载 Jazzy 的新终端，从仓库根目录执行：

```bash
# 加载 Jazzy 环境
source /opt/ros/jazzy/setup.bash
# 优先使用系统工具
export PATH=/usr/bin:/bin:$PATH
# 指定项目根目录下的固定源码依赖目录
export SCHURVINS_DEPENDENCY_DIR="$PWD/.cache/dependencies"
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `source` 的文件路径 | 指定环境脚本 | 输入文件 | 必填；示例为 Jazzy 环境 |
| `PATH`（环境变量） | 指定可执行文件搜索路径 | 输入值 | 继承当前环境；示例将系统目录置前 |
| `SCHURVINS_DEPENDENCY_DIR`（环境变量） | 指定固定依赖位置 | 输入目录 | 在项目根目录执行，设置为 `$PWD/.cache/dependencies`；构建前须与准备依赖时使用的目录一致 |

```bash
# 准备锁定的源码依赖
/usr/bin/python3 scripts/prepare_dependencies.py \
  "$SCHURVINS_DEPENDENCY_DIR"
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `directory` | 依赖存放位置 | 输出目录 | 必填，无默认值 |
| `-h`, `--help` | 显示帮助 | 行为控制项 | 默认不显示 |

```bash
# 准备官方词典
/usr/bin/python3 scripts/prepare_resources.py \
  "$HOME/.cache/schurvins/resources"
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `directory` | 词典存放位置 | 输出目录 | 必填，无默认值 |
| `-h`, `--help` | 显示帮助 | 行为控制项 | 默认不显示 |

源码版本以 [dependencies.lock.json](../dependencies.lock.json) 为准，依赖源码保存在项目根目录的 `.cache/dependencies/` 下，包含 `minkindr/`、`fast_neon/` 和 `DBoW2/`，各依赖源码树须无未提交修改。项目根目录的 `.cache/` 已加入 `.gitignore`，不提交下载的依赖源码。词典保存在仓库外；脚本校验官方压缩包 SHA256 和文件列表。所需词典缺失或为空会导致启动报错。

## 三套独立构建配置

以下命令在项目根目录执行；新终端须先按上节加载 Jazzy、设置 `PATH` 和 `SCHURVINS_DEPENDENCY_DIR`。不要在同一终端加载多个配置的安装环境。

```bash
# 构建默认双目惯性配置
colcon --log-base ros2_ws/log/default build \
  --base-paths . \
  --build-base ros2_ws/build/default \
  --install-base ros2_ws/install/default \
  --cmake-args \
  -DSCHURVINS_DEPENDENCY_DIR="$SCHURVINS_DEPENDENCY_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DUSE_SCHUR_VINS=ON \
  -DUSE_LOOP_CLOSING=ON \
  -DUSE_GLOBAL_MAP=OFF
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--log-base` | 日志位置 | 输出目录 | `ros2_ws/log/<配置>` |
| `--base-paths` | 包搜索位置 | 输入目录 | `.`，即当前项目根目录 |
| `--build-base` | 构建文件位置 | 输出目录 | `ros2_ws/build/<配置>` |
| `--install-base` | 安装位置 | 输出目录 | `ros2_ws/install/<配置>` |
| `--cmake-args` | 传递 CMake 配置 | 行为控制项 | 示例使用下列配置 |
| `SCHURVINS_DEPENDENCY_DIR`（CMake 参数） | 指定固定源码依赖目录并更新缓存 | 输入目录 | 使用上节环境变量的值；须与准备依赖时的目录一致 |
| `CMAKE_BUILD_TYPE` | 构建类型 | 输入值 | 示例为 `Release` |
| `Python3_EXECUTABLE` | 指定 Python | 输入文件 | 示例为 `/usr/bin/python3` |
| `OpenCV_DIR` | 指定系统 OpenCV CMake 目录 | 输入目录 | 示例为 amd64 系统路径 |
| `USE_SCHUR_VINS` | 启用 SchurVINS | 行为控制项 | `ON` 或 `OFF` |
| `USE_LOOP_CLOSING` | 编译回环支持 | 行为控制项 | `ON` 或 `OFF`；与运行时 `runlc` 独立 |
| `USE_GLOBAL_MAP` | 编译全局地图支持 | 行为控制项 | `ON` 或 `OFF` |

```bash
# 加载默认配置的安装环境
source ros2_ws/install/default/setup.bash
# 执行默认配置的测试
colcon --log-base ros2_ws/log/default test \
  --base-paths . \
  --build-base ros2_ws/build/default \
  --install-base ros2_ws/install/default
# 查看测试结果
colcon test-result \
  --test-result-base ros2_ws/build/default \
  --verbose
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--log-base` | 测试日志位置 | 输出目录 | 使用对应配置目录 |
| `--build-base` | 构建目录 | 输入兼输出目录 | 使用对应配置目录 |
| `--install-base` | 安装目录 | 输入目录 | 使用对应配置目录 |
| `--test-result-base` | 测试结果位置 | 输入目录 | 使用对应构建目录 |
| `--verbose` | 显示详细结果 | 行为控制项 | 默认关闭 |

其他配置使用各自的目录重复测试；外部数据或显示环境相关测试须单独报告。

```bash
# 检查运行时库链接
ldd \
  ros2_ws/install/default/svo_ros/lib/svo_ros/svo_node
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 文件路径 | 指定待检查程序 | 输入文件 | 必填；合并安装时调整路径 |

所有 OpenCV 库须指向系统 4.6。若不匹配，使用新的构建目录并指定系统 `OpenCV_DIR`。

## 启动、参数与组件

启动和回放使用分别加载 Jazzy 与所选安装环境的终端。EuRoC 数据需包含 `/cam0/image_raw`、`/cam1/image_raw`、`/imu0`。

```bash
# 启动 EuRoC 双目惯性节点
ros2 launch svo_ros euroc_vio_stereo.launch.py \
  output_dir:="$HOME/schurvins-results/euroc"
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `output_dir` | 运行结果根目录 | 输出目录 | 默认 `/tmp/schurvins-results`；每次运行创建独立子目录 |
| `vocabulary_dir` | 外部锁定词典位置 | 输入目录 | 默认空；单目且 `runlc: true` 时必填 |
| `imu_topic` | IMU 话题 | 输入值 | 默认 `/imu0` |
| `use_sim_time` | 使用仿真时间 | 行为控制项 | 默认 `true` |

通用入口和默认值见 [svo.launch.py](../svo_ros/launch/svo.launch.py)。自定义参数使用 `ros__parameters` 和点号层级；算法和标定参数启动后只读。回环需设置 `runlc: true`、外部 `voc_path` 和 `voc_name`，不要自动训练替代词典。

```bash
# 查看通用启动参数
ros2 launch svo_ros svo.launch.py \
  --show-args
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `pipeline` | 选择流程 | 输入值 | 默认 `stereo`；支持 `stereo`、`mono`、`array` |
| `params` | 参数文件 | 输入文件 | 默认安装目录下 `param/vio_stereo.yaml` |
| `calib_file` | 标定文件 | 输入文件 | 默认安装目录下 `param/calib/euroc_stereo.yaml` |
| `vocabulary_dir` | 锁定词典目录 | 输入目录 | 默认空 |
| `imu_topic` | IMU 话题 | 输入值 | 默认 `/imu0` |
| `use_imu` | 启用 IMU | 行为控制项 | 默认 `true` |
| `use_sim_time` | 使用仿真时间 | 行为控制项 | 默认 `true` |
| `output_dir` | 结果根目录 | 输出目录 | 默认 `/tmp/schurvins-results` |
| `--show-args` | 显示启动参数 | 行为控制项 | 默认不显示 |

传统配置可使用 `euroc_vio_mono.launch.py`、`euroc_visual_mono.launch.py`、`euroc_visual_stereo.launch.py` 或 `array.launch.py`；后者需要四相机标定。四相机离线数据入口见[基准测试](../svo_benchmarking/README.md)。传统 + 全局地图配置可使用 `euroc_global_map_mono.launch.py`，并指定 `output_dir` 和 `vocabulary_dir`。

```bash
# 回放原生 ROS2 数据包
ros2 bag play \
  /absolute/path/to/ros2_euroc \
  --clock \
  --rate 0.25
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 数据包路径 | 指定数据 | 输入文件或目录 | 必填；原生 MCAP 或 SQLite3 |
| `--clock` | 发布仿真时钟 | 行为控制项 | 默认不启用 |
| `--rate` | 回放倍率 | 输入值 | 默认 `1.0`；示例为 `0.25` |
| `--start-paused` | 启动后暂停 | 行为控制项 | 默认不暂停 |
| `--qos-profile-overrides-path` | 指定 QoS 覆盖配置 | 输入文件 | 默认未设置 |
| `--topics` | 限定回放话题 | 输入值 | 默认回放全部话题 |

```bash
# 启动实时组件示例
ros2 launch svo_ros live_components.launch.py \
  calib_file:=/absolute/path/to/calibration.yaml \
  output_dir:="$HOME/schurvins-results/live"
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `calib_file` | 相机/IMU 标定 | 输入文件 | 必填，无默认值 |
| `output_dir` | 运行结果根目录 | 输出目录 | 默认 `/tmp/schurvins-results` |

实时组件需要外部驱动发布标准图像和 IMU 消息；组件冒烟测试不代表物理相机已验证。

```bash
# 查看注册的组件类型
ros2 component types
# 打开轨迹显示
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share svo_ros)/rviz_config_vio.rviz"
# 打开 rqt 界面
ros2 run rqt_svo rqt_svo
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `component types`：无 | 无额外参数 | 输出组件类型 | 不适用 |
| RViz `-d` | 指定显示配置 | 输入文件 | 示例使用已安装的 VIO 配置 |
| `pkg prefix --share` | 查询包资源目录 | 输入包名，输出目录 | `svo_ros` 为示例包名 |
| `rqt_svo`：无 | 无额外参数 | 图形界面 | 不适用；无显示时可使用 `xvfb-run` |

```bash
# 暂停估计
ros2 topic pub --once \
  /svo/remote_key \
  std_msgs/msg/String \
  '{data: "s"}'
# 重置估计
ros2 topic pub --once \
  /svo/remote_key \
  std_msgs/msg/String \
  '{data: "r"}'
# 停止并刷新输出
ros2 topic pub --once \
  /svo/remote_key \
  std_msgs/msg/String \
  '{data: "q"}'
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--once` | 发布一次 | 行为控制项 | 默认行为待确认 |
| 话题、消息类型、消息内容 | 指定控制消息 | 输入值 | 示例使用 `/svo/remote_key` 和 `std_msgs/msg/String`；支持 `q/r/s/c/C` |

重置会重建状态并创建新运行目录；暂停保留状态。时间戳倒退后须重启。组件模式下 `q` 停止 SVO 实例，不关闭容器。

## EGO 标定与完整回放

### 一键处理 MCAP

```bash
# 直接处理并保存原始消息、VIO odom/Path 和可选 TUM 轨迹
ros2 launch svo_ros ego.launch.py \
  inbag:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  cameras:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  save_trajectory:=true
```

| 参数 | 作用 | 默认值 |
| --- | --- | --- |
| `inbag` | 输入单个 MCAP 文件；为空时接收原有实时话题 | 空 |
| `cameras` | 源 EGO 标定，替代原 `source_calibration` | `/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml` |
| `save_trajectory` | 保存 TUM IMU/body 轨迹文本 | `false` |
| `realtime` | 按包记录时间以 1 倍速处理；落后时延迟但不丢帧 | `false`，无回放等待 |
| `output_dir` | 显式设置时结果直接保存到该目录 | 当前目录的 `output/result_YYYYMMDDHHMMSS_<包名>/` |
| `rviz` | 启动 RViz；离线完成后随 launch 关闭 | `false` |

目录时间固定为上海时区，包名去掉 `.mcap` 后缀。默认目录重名时追加数字；显式目录中的同名结果拒绝覆盖。输出单文件 `schurvins_<包名>.mcap` 保留原包全部话题的序列化消息、记录/发布时间和消息定义，包括未安装消息包的自定义话题，并新增 `/svo/odometry`（`nav_msgs/Odometry`）及 `/svo/path_imu`（`nav_msgs/Path`）。新增话题占用名称已存在于输入包时明确报错。odom 提供世界坐标系下的 IMU/body 位姿，`child_frame_id=imu`；twist 未估计，其协方差对角线置为 `1e12`。

开启保存时，`schurvins_<包名>_trajectory.txt` 每行为 `timestamp tx ty tz qx qy qz qw`，单位为秒、米，时间戳为图像原始时间；仅保存有效跟踪位姿，不与参考轨迹对齐。关闭时不生成轨迹文本。无 `inbag` 时包名使用 `live`，不生成 MCAP。诊断数据保留在 `frames.csv`、`summary.json` 和 `effective_config.yaml`；实时订阅模式仍在独立 `run_*` 子目录保存诊断。

两种速度使用同一有界输入调度，输入完整处理后关闭输出并自动退出。异常或中断时保留可恢复的 MCAP，摘要标记 `complete=false`；同一输出目录不用于重跑。最终平均 FPS 为实际进入估计器的双目帧组数除以累计 IMU/VIO 计算秒数，排除 MCAP 读写、JPEG 解码、格式转换、发布、可视化和等待；初始化与前导跳过分别记录。

```bash
# 快速模式完整验收；1 倍速使用新输出目录并增加 --realtime
/usr/bin/python3 scripts/accept_ego_offline.py \
  --bag /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --calibration /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  --output /tmp/schurvins-ego-offline \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt
```

验收入口验证 4909 组图像、65856 条 IMU、2 个前导跳过及 4907 次估计调用，并逐话题核对原始消息、odom/Path/TUM 一致性和队列排空。`--no-save-trajectory` 可验证关闭文本保存；默认保存。参考轨迹含其他 VIO 先验，不作为独立真值。

### 外部回放验收

优先使用自动验收。加载所选安装环境，源 MCAP、标定和参考轨迹保持只读。示例路径须替换；输出必须是尚不存在的新目录。参考轨迹含其他 VIO 先验，**不是独立真值**。

```bash
# 从零偏移以 0.25 倍速执行 EGO 验收
source ros2_ws/install/default/setup.bash
/usr/bin/python3 scripts/accept_ego.py \
  --bag /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --calibration /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  --output "$PWD/output/ego-acceptance-quarter-speed" \
  --rate 0.25 \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt
```

参数依据：[accept_ego.py](../scripts/accept_ego.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--bag` | 指定输入数据包 | 输入文件或目录 | 必填，无默认值 |
| `--calibration` | 指定源标定 | 输入文件 | 必填，无默认值 |
| `--output` | 保存结果和日志 | 输出目录 | 必填；目录必须不存在 |
| `--rate` | 回放倍率 | 输入值 | 默认 `0.25`，浮点数 |
| `--timeout` | 验收超时时间（秒） | 输入值 | 默认 `1800`，浮点数 |
| `--images` | 预期原始图像数 | 输入值 | 默认 `4909`，整数 |
| `--imu` | 预期原始 IMU 数 | 输入值 | 默认 `65856`，整数 |
| `--leading` | 预期跳过的开头图像数 | 输入值 | 默认 `2`，整数 |
| `--reference` | 指定参考轨迹 | 输入文件 | 默认未设置 |
| `-h`, `--help` | 显示帮助 | 行为控制项 | 默认不显示 |

检查 `acceptance.json`、`run_*/summary.json`、`run_*/frames.csv`、`schurvins_live_trajectory.txt` 和 `reference/` 报告。上述示例的完整性要求收到 4909 张原始图像和 65856 条原始 IMU，恰好跳过开头两张图像，其余 4907 对各处理一次；估计器须初始化并跟踪，位姿有限且时间单调，初始化/重定位帧须记录状态，结束时无未解释的丢失、崩溃或队列残留。保留失败日志，不合并运行；队列溢出时降低速率并从新估计器重跑。另用新输出目录和 `--rate 1` 运行性能验收，报告初始化时间、轨迹覆盖率、跟踪丢失/重置次数、处理时间百分位数、总耗时和峰值内存。传统/全功能及完整数据集结果须来自实际执行。

### 手动回放

手动流程用于检查；完整计数验收使用上面的自动入口。启动文件生成 `output_dir/ego_calibration_*/calibration.yaml`。标定转换使用 800 Hz、实际时间戳、原始噪声、`delay_imu_cam=0`、源 `T_cam_imu` 的逆作为 `T_B_C`，不进行图像校正、裁剪或缩放。3840×1080 JPEG 拆成两幅 1920×1080 灰度图并保留源时间戳；IMU 时间戳减去 0.007 秒，固定零偏扣除一次，内部从零估计剩余零偏。

```bash
# 启动 EGO 节点和 RViz
ros2 launch svo_ros ego.launch.py \
  cameras:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  output_dir:="$PWD/output/ego-quarter-speed" \
  save_trajectory:=true rviz:=true
```

参数依据：[ego.launch.py](../svo_ros/launch/ego.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `cameras` | 指定源标定 | 输入文件 | 默认 `/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml` |
| `output_dir` | 保存结果及转换后标定 | 输出目录 | 默认当前目录下的 `output/result_YYYYMMDDHHMMSS_live/` |
| `rviz` | 启动 RViz | 行为控制项 | 默认 `false`；支持 `true`、`false` |

在另一个已加载环境的终端创建 QoS 配置：

```bash
# 写入回放 QoS 覆盖文件
cat > /tmp/ego-playback-qos.yaml <<'YAML'
/stereo/image_raw:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 64
/imu:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 10000
YAML
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 重定向路径 | 保存 QoS 配置 | 输出文件 | 示例为 `/tmp/ego-playback-qos.yaml`，会覆盖已有文件 |
| `reliability`、`durability`、`history` | 配置话题 QoS | 输入值 | 示例分别为 `reliable`、`volatile`、`keep_last` |
| `depth` | 设置队列深度 | 输入值 | 示例图像为 `64`，IMU 为 `10000` |

```bash
# 查看图像话题端点
ros2 topic info \
  /stereo/image_raw \
  --verbose
# 查看 IMU 话题端点
ros2 topic info \
  /imu \
  --verbose
# 暂停启动 EGO 数据回放
ros2 bag play \
  /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --clock \
  --rate 1.0 \
  --start-paused \
  --qos-profile-overrides-path $PWD/config/ego-playback-qos.yaml \
  --topics /stereo/image_raw /imu
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 话题名 | 指定检查对象 | 输入值 | 必填 |
| `--verbose` | 显示端点详情 | 行为控制项 | 默认不显示详情 |

回放参数见前述 `ros2 bag play` 表。暂停期间检查状态，直到五个 `matched_*` 值均大于零；仅看到 ROS 图中的订阅关系不足以确认 DDS 匹配。

```bash
# 查看 DDS 匹配和解码状态
ros2 topic echo \
  /ego/runtime_status \
  --once
# 匹配完成后恢复回放
ros2 service call \
  /rosbag2_player/resume \
  rosbag2_interfaces/srv/Resume \
  '{}'
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `topic echo` 的话题 | 指定状态话题 | 输入值 | 必填 |
| `--once` | 接收一条消息后退出 | 行为控制项 | 默认持续接收 |
| `service call` 的服务、类型、请求 | 指定恢复请求 | 输入值 | 必填；示例请求为 `{}` |

回放结束后，检查 `/ego/runtime_status` 的解码计数、`pending_decode_images`、`decoder_inflight`、`image_queue_overflow`，以及 `/svo/runtime_status` 的处理计数、`pending_images`、`pending_imu`、`inflight`、`leading_skipped`、`overflow`、`time_jumps` 和 `run_dir`。等待解码与预期图像处理完成、两个图像队列为空且处理中标志为 false，再按前述命令发送 `q`；等待停止且核心队列清空后检查摘要。“bag 回放完成”不等于估计完成。

```bash
# 单独转换 EGO 标定
/usr/bin/python3 "$(ros2 pkg prefix --share svo_ros)/scripts/ego_calibration.py" \
  /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  /tmp/ego-calibration.yaml
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `source` | 指定源标定 | 输入文件 | 必填，无默认值 |
| `destination` | 保存转换后标定 | 输出文件 | 必填，无默认值 |
| `-h`, `--help` | 显示帮助 | 行为控制项 | 默认不显示 |

```bash
# 生成参考轨迹对比报告
ros2 run svo_benchmarking reference_report.py \
  /absolute/run_dir/imu_traj.txt \
  /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt \
  /absolute/report_dir \
  --tolerance 0.01
```

参数依据：[reference_report.py](../svo_benchmarking/scripts/reference_report.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `estimate` | 估计轨迹；批处理时为结果根目录 | 输入文件或目录 | 必填 |
| `reference` | 参考轨迹；批处理时为输出目录 | 输入文件或输出目录 | 必填 |
| `output` | 单轨迹报告目录 | 输出目录 | 单轨迹模式必填；批处理时不需要 |
| `--batch` | 启用批处理 | 行为控制项 | 默认关闭 |
| `--tolerance` | 时间关联容差（秒） | 输入值 | 默认 `0.01`，浮点数 |
| `-h`, `--help` | 显示帮助 | 行为控制项 | 默认不显示 |

报告包含固定尺度 SE(3) 对齐后的 ATE 位置 RMSE、1 秒平移/旋转 RPE、关联数量与覆盖率、轨迹图和 JSON；这些指标不能确立独立绝对精度，也不能证明没有 ROS1 回退。
