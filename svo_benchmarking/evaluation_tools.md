# Bag and trajectory evaluation

Source Jazzy and the installed SchurVINS profile. Native bag tools support MCAP and SQLite3.

```bash
ros2 run svo_benchmarking analyze_stationary.py /absolute/recording.mcap /svo/backend_pose_imu 0.3 0.9
ros2 run svo_benchmarking cal_traj_len.py /absolute/recordings /reference_pose
ros2 run svo_benchmarking reference_report.py /absolute/estimate.txt /absolute/reference.txt /absolute/report
```

Use `--help` for each tool's positional arguments. The reference report performs no-scale SE(3) alignment with 10 ms association and one-second translation/rotation RPE; reports include JSON and a plot. EGO reference trajectories contain VIO priors and must not be called independent ground truth. See the [benchmarking guide](README.md) for batch analysis and [dataset tools](dataset_tools/README.md) for supported message formats.
