# ROS2 参数与运行接口

所有算法默认值保留于 `svo_factory.cpp`、`ceres_backend_factory.cpp`；配置覆盖项使用 `/**: {ros__parameters: ...}`。算法和标定参数在首次声明后只读，修改后重新启动节点。标准 `use_sim_time` 遵循 ROS2 时钟行为。

| ROS1 | ROS2 | 说明 |
| --- | --- | --- |
| `T_world_imuinit/qx` 等斜杠键 | `T_world_imuinit.qx` 等点号键 | 所有斜杠层级统一替换为点号 |
| `pipeline_is_stereo` | `pipeline: stereo/mono/array` | mono、array、纯视觉使用传统 SVO 构建 |
| `trace_dir` / 固定 logs 目录 | `output_dir` | 自动建立唯一 `run_*`，实际 trace_dir 固定为本次运行目录 |
| `cam0_topic` / `cam1_topic` | 同名 | array 另有 cam2_topic/cam3_topic，时间戳严格一致 |
| `imu_topic` | 同名 | 默认 `/imu`；EuRoC launch 指定 `/imu0` |
| `remote_key_topic` | 同名 | 默认 `/svo/remote_key`，`std_msgs/msg/String` |
| nodelet | `svo_ros::SvoNodeBase` | 独立 `svo_node` 和组件共用实现 |
| ROS1 硬件 nodelet launch | `live_components.launch.py` | 接标准 ROS2 驱动话题，未作硬件实机验证 |

`q` 停止本实例输入并排空已接收队列；不关闭组件容器。`r` 刷新旧输出并完整销毁、重建估计器和 IMU，建立新运行目录，然后等待 `s`。等待期间输入分别计入 idle_images/idle_imu，不累积历史队列；重启使用新输入。`s` 启动，`c/C` 开启/关闭光度补偿。时间倒退或队列溢出使当前运行失败，必须重启；暂停播放不重置状态。退出时无法配对或无 IMU 覆盖的尾帧明确记为失败。

输出包括 `imu_traj.txt`（TUM、world 下 IMU 位姿）、`frames.csv`、`effective_config.yaml` 和 `summary.json`。状态话题 `/svo/runtime_status` 提供真实估计器调用数、位姿数、队列、inflight、前导跳过、溢出及回跳统计；自动回放需在原始输入计数完整后等待图像队列为空且 inflight=false，再发布 q 并等待 stopped=true 和两个队列为空。尾部 IMU 在 q 前暂存，避免晚到的下一帧失去历史覆盖，awaiting_images=true 显式标记此状态。

EGO 输入组件 `svo_ros::EgoInput`（独立程序 `ego_input_node`）只订阅原始 `/stereo/image_raw` 和 `/imu`，发布 `/ego/cam0/image_raw`、`/ego/cam1/image_raw` 和 `/ego/imu`。陀螺仪消息采用标准 rad/s，标定偏置从 deg/s 转为 rad/s 后只减一次；加速度偏置同样只减一次。校正 IMU 时间固定减 7ms。核心配置偏置初始化为零、delay 为零。`ego_calibration.py` 读取原文件，以完整逆矩阵转换毫米 T_cam_imu，保留噪声并采用 800Hz，输出 pinhole + equidistant。

```bash
ros2 launch svo_ros ego.launch.py output_dir:=/tmp/ego-results
ros2 bag play /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap --clock --rate 0.25 --topics /stereo/image_raw /imu
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: q}'
```

RViz2 配置保留图像、位姿、轨迹、关键帧、地图点、回环、PGO 和全局地图显示项。上述资源迁移尚需集成构建、GUI 和数据回放验收；接口示例不表示实机已验证。

ROS2 topic tokens cannot start with digits: per-camera outputs are `/svo/image/cam0`, `/svo/dense_input/cam0`, `/svo/pose_cam/cam0` (and `cam1` etc.).
