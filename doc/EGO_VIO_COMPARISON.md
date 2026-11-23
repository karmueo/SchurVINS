# EGO VIO 配置对比

`scripts/compare_ego_vio.py` 串行运行单因素筛选、组合验证和最终三次复跑。固定使用 `job_002.mcap`、同目录标定和 IMU/body 参考轨迹；从零偏移开始，不改标定、时间校正或输入数据。参考轨迹包含其他 VIO 先验，结果不代表独立绝对精度。

先构建默认配置及独立 `vio-traditional` 配置，后者设置 `USE_SCHUR_VINS=OFF`、`USE_LOOP_CLOSING=ON`、`USE_GLOBAL_MAP=OFF`。两套配置均须包含最新的 `svo_ros` 安装资源；构建方法见 [运行指南](ROS2_JAZZY.md)。评估工具使用相邻的 `rpg_trajectory_evaluation` 仓库及其已准备的 uv 环境。

```bash
/usr/bin/python3 scripts/compare_ego_vio.py --output "$PWD/output/vio_compare_example"
```

| 参数 | 作用 |
| --- | --- |
| `--output` | 新实验根目录；默认按上海时间生成，不覆盖已有运行 |
| `--phase screen` | 默认 Schur、窗口 6/8、观测噪声 1/3、特征 300/500 与 450/750、组合，共八组快速筛选 |
| `--phase confirm` | 读取已有筛选结果，基线、最佳 Schur、传统 Ceres、传统 Ceres+回环各三次 1 倍速复跑 |
| `--phase all` | 默认值，依次执行筛选与复跑 |

直接运行自定义参数：

```bash
ros2 launch svo_ros ego.launch.py \
  inbag:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  cameras:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  params:=/absolute/path/to/complete_params.yaml \
  output_dir:=/absolute/path/to/new_result \
  save_trajectory:=true realtime:=true
```

`params` 是完整的 ROS 参数文件，省略时仍使用原 `vio_stereo.yaml`。启动入口覆盖输入、输出、标定及话题等 EGO 必需参数；实验文件控制算法参数。不要同时启用 SchurVINS 和独立 Ceres 后端。

每次保存完整 MCAP、轨迹、逐帧统计、有效参数、构建缓存及启动命令。SE(3)/Sim(3) 使用独立输入工作目录、`eval_cfg.yaml` 和缓存，避免评估工具复用关联结果。PGO 轨迹为 IMU/body 关键帧轨迹，仅与同时间戳的在线关键帧比较，不插值加入逐帧主排名。`loop_summary.json` 记录检测、应用校正及完成 PGO 次数；输入 EOF 后等待后台任务，超时会使该次运行失败。

汇总输出为 `results.json`、`report.md`、`comparison.png`，每组完整评估见 `evaluation/*/saved_results/result.md`，1 秒 RPE 见 `reference/reference_summary.json`。低覆盖或有跟踪丢失的结果保留但不参与优胜排名；跨后端性能比较使用端到端耗时，现有计算 FPS 不覆盖全部异步计算。
