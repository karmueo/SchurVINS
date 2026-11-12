# ROS2 Jazzy installation and operation

This is the migration operating contract for Ubuntu 24.04, C++17 and Python 3.12. Follow the [official Jazzy Debian installation instructions](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst) to configure the ROS apt repository first. Commands assume an amd64 host; choose the system architecture's OpenCV cmake directory on other hosts. Validation results belong in the actual acceptance report, not this procedure.

## Dependencies and resources

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions \
  build-essential cmake git libeigen3-dev libopencv-dev libceres-dev libyaml-cpp-dev \
  libgoogle-glog-dev libgflags-dev libopengv-dev libglew-dev libboost-all-dev \
  libblas-dev liblapack-dev libsuitesparse-dev ros-jazzy-gtsam \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rclcpp-components ros-jazzy-pcl-conversions \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-rqt-gui-py ros-jazzy-python-qt-binding \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
  python3-numpy python3-scipy python3-matplotlib python3-yaml \
  python3-psutil python3-colorama python3-pytest xvfb
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
```

Use distribution Eigen 3.4, OpenCV 4.6, Ceres 2.2, yaml-cpp 0.8 and Jazzy GTSAM 4.2. Do not mix a user OpenCV 4.13 installation with system cv_bridge. The build pins `OpenCV_DIR` and rejects newer OpenCV versions; use `/usr/bin/python3` rather than a Conda interpreter.

From the repository root prepare source dependencies and official vocabulary resources **before** colcon:

```bash
export SCHURVINS_DEPENDENCY_DIR="$HOME/.cache/schurvins/dependencies"
/usr/bin/python3 scripts/prepare_dependencies.py "$SCHURVINS_DEPENDENCY_DIR"
/usr/bin/python3 scripts/prepare_resources.py "$HOME/.cache/schurvins/resources"
```

The vendor wrappers require clean source trees at these locked commits:

| Source | Commit |
|---|---|
| minkindr | `564f12639a8447d4d3e5e7707851424302941056` |
| fast_neon | `4914ed963aba8d2bb476551e186e4b20a206f03d` |
| DBoW2 | `3924753db6145f12618e7de09b7e6b258db93c6e` |

The authoritative lock is `dependencies.lock.json`. The official `https://rpg.ifi.uzh.ch/svo2/vocabularies.tar.gz` archive is checked against SHA256 `9265bead3a209239c8f016e1b986ce6ad1c4f16856a6174722670ac43dbe4a35` and its locked member list. Extracted vocabulary files stay outside the repository. A requested missing or empty vocabulary is an explicit startup error.

## Three independent build profiles

Place the checkout at `~/schurvins_ws/src/SchurVINS`. From `~/schurvins_ws`, in a fresh shell sourced only with Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
export SCHURVINS_DEPENDENCY_DIR="$HOME/.cache/schurvins/dependencies"
colcon --log-base log-default build --base-paths src/SchurVINS \
  --build-base build-default --install-base install-default \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DUSE_SCHUR_VINS=ON -DUSE_LOOP_CLOSING=ON -DUSE_GLOBAL_MAP=OFF
```

For full optimization use a new shell and the same command with `log-full`, `build-full`, `install-full`, and `-DUSE_GLOBAL_MAP=ON`. For traditional SVO use a third shell with `log-traditional`, `build-traditional`, `install-traditional`, `-DUSE_SCHUR_VINS=OFF`, `-DUSE_LOOP_CLOSING=ON`, `-DUSE_GLOBAL_MAP=OFF`. Keep all other arguments. Do not source multiple profile overlays into one shell.

| Profile | Estimator modes | Build flags |
|---|---|---|
| Default | SchurVINS stereo + IMU | Schur ON, loop closing ON, global map OFF |
| Full | Adds GTSAM global map entry points | Schur ON, loop closing ON, global map ON |
| Traditional | Mono, visual-only stereo/mono, array | Schur OFF; use Ceres/global configurations as required |

Build-time loop closing support and runtime `runlc` are separate switches. To run the traditional monocular global map example, build its own traditional + `USE_GLOBAL_MAP=ON` profile. SchurVINS is not extended to monocular, arbitrary arrays or IMU-free estimation.

```bash
source install-default/setup.bash
colcon --log-base log-default test --build-base build-default --install-base install-default
colcon test-result --test-result-base build-default --verbose
```

Repeat with the corresponding directory names in the other profiles. Dataset-dependent tests requiring external data or a display must be reported separately. Check final runtime links with `ldd install-default/svo_ros/lib/svo_ros/svo_node` (adapt the path if using a merged install): all OpenCV libraries must resolve to the system 4.6 build.

## Launch, parameters and components

```bash
ros2 launch svo_ros euroc_vio_stereo.launch.py output_dir:="$HOME/schurvins-results/euroc"
ros2 bag play /absolute/path/to/ros2_euroc --clock --rate 0.25
```

The EuRoC example consumes `/cam0/image_raw`, `/cam1/image_raw` and `/imu0`. Generic `svo.launch.py` accepts `pipeline` (`stereo`, `mono`, `array`), `params`, `calib_file`, `imu_topic`, `use_imu`, `use_sim_time`, `vocabulary_dir`, and `output_dir`. Monocular configurations with `runlc: true` require `vocabulary_dir:=/absolute/locked/vocabularies`. Parameters and calibration paths may be absolute external files. Inspect launch arguments with `ros2 launch svo_ros svo.launch.py --show-args`.

Traditional launch entries are `euroc_vio_mono.launch.py`, `euroc_visual_mono.launch.py`, `euroc_visual_stereo.launch.py`, and `array.launch.py` (four-camera calibration required). Four-camera datasets use the offline `svo_benchmark` entry and an image index with four paths; see [benchmarking](../svo_benchmarking/README.md). The global example is `euroc_global_map_mono.launch.py output_dir:=... vocabulary_dir:=...` under a traditional + global map build.

```bash
ros2 launch svo_ros live_components.launch.py \
  calib_file:=/absolute/path/to/calibration.yaml output_dir:="$HOME/schurvins-results/live"
ros2 component types
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share svo_ros)/rviz_config_vio.rviz"
ros2 run rqt_svo rqt_svo
```

`live_components` uses standard image/IMU topics, without a repository-external camera driver. Registered classes are `svo_ros::SvoNodeBase` and `svo_ros::EgoInput`; standalone executables are `svo_node` and `ego_input_node`. A hardware-specific image transport must publish the standard messages; no physical camera validation is implied by a component smoke test.

| Former setting/interface | ROS2 equivalent |
|---|---|
| Slash-separated parameter keys | Dot-separated keys under `ros__parameters` |
| Node private handle | Explicitly declared parameters on the node |
| Relative package resource paths | ament package index + installed share resources |
| Shared output folder | `output_dir` and unique run subdirectory |
| Nodelet | `rclcpp_components` component or standalone node |
| ROS1 bags | Native ROS2 MCAP or SQLite3 |
| Camera/IMU pose chain | TF `world → imu → cam0/cam1` |

For custom parameter files use `/**: {ros__parameters: ...}` or an explicit node selector. `T_world_imuinit/qx` becomes `T_world_imuinit.qx`. Algorithm/calibration parameters are read-only after launch. For loop closure set external `voc_path` and the vocabulary filename in `voc_name`, along with `runlc: true`; never train a replacement vocabulary automatically.

Control commands remain `q/r/s/c/C` on `/svo/remote_key`:

```bash
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: "s"}'
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: "r"}'
ros2 topic pub --once /svo/remote_key std_msgs/msg/String '{data: "q"}'
```

A reset reconstructs estimation state and creates a new output run. Paused playback preserves state. A backward timestamp stops the current run; restart before replaying from an earlier point. In component mode `q` stops the SVO instance without shutting down its container.

## EGO calibration and complete playback

The repository's automatic acceptance runner is the preferred reproducible entry. From any directory, source the selected install and invoke its absolute source path. `--output` must name a new directory:

```bash
/usr/bin/python3 ~/schurvins_ws/src/SchurVINS/scripts/accept_ego.py \
  --bag /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --calibration /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  --output "$HOME/schurvins-results/ego-acceptance-quarter-speed" --rate 0.25 \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt
```

The runner waits for DDS readiness, performs playback, checks full input/processing counts, sends `q` after images drain and waits for output/queue shutdown. Inspect `acceptance.json`, `run_*/summary.json`, `run_*/frames.csv`, `run_*/imu_traj.txt` and `reference/` JSON/plot reports. Run again with a different new output path and `--rate 1` for the performance report. Keep each failed attempt's logs; never merge runs. Traditional/full-profile and complete-dataset acceptance must be reported from actual executions, not inferred from this guide.

The following manual workflow is useful for inspection. Source MCAP, source calibration and reference remain read-only:

```bash
ros2 launch svo_ros ego.launch.py \
  source_calibration:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  output_dir:="$HOME/schurvins-results/ego-quarter-speed" rviz:=true
```

For complete-count acceptance use the automatic command above. For manual replay, start paused and wait for actual DDS endpoint matching before resuming in another sourced terminal:

For manual playback, create a reliable QoS override to match the input subscribers:

```bash
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

```bash
ros2 topic info /stereo/image_raw --verbose
ros2 topic info /imu --verbose
ros2 bag play /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --clock --rate 0.25 --start-paused --qos-profile-overrides-path /tmp/ego-playback-qos.yaml \
  --topics /stereo/image_raw /imu
```

While the player is paused, inspect `ros2 topic echo /ego/runtime_status --once` until all five `matched_*` values are positive. This checks DDS matches; merely seeing subscriptions in the ROS graph is insufficient. Then resume:

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
```

The JPEG input is 3840×1080 and splits into full-size 1920×1080 grayscale views. Both views retain the source timestamp. The adapter subtracts 0.007 seconds from IMU timestamps and subtracts fixed biases once (gyro deg/s → rad/s); the internal initial biases are zero and estimate the remaining bias. The generated configuration uses 800 Hz nominal IMU frequency, actual timestamp integration, unchanged source noise values, `delay_imu_cam=0`, millimetres → metres, inverse source `T_cam_imu` as `T_B_C`, and pinhole + equidistant distortion. No rectification, cropping or scaling is applied.

The launch creates `output_dir/ego_calibration_*/calibration.yaml`. For a separate conversion:

```bash
/usr/bin/python3 "$(ros2 pkg prefix --share svo_ros)/scripts/ego_calibration.py" \
  /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml /tmp/ego-calibration.yaml
```

The EGO reference is `/data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt`. It contains other VIO priors and is **not independent ground truth**. The main run starts at zero offset at 0.25× speed. Reliable QoS and bounded queues are used (64 stereo groups, 10000 IMU samples by default). Overflow is a failed completeness run: lower replay speed and rerun from a fresh estimator.

Acceptance requires 4909 raw images and 65856 raw IMU messages received, exactly two leading images explicitly skipped for insufficient corrected IMU coverage, and each remaining 4907 image pairs decoded/paired/processed once. The estimator must initialize and track, output finite monotonic poses, and finish with no unexplained drops, crashes or leftover queues. Initialization/relocalization frames need recorded states even without a pose.

Monitor `/ego/runtime_status` JSON (`raw_images`, `decoded_images`, `invalid_images`, `raw_imu`, `published_imu`, `pending_decode_images`, `decoder_inflight`, `image_queue_overflow`) and `/svo/runtime_status` (`processed`, `poses`, `pending_images`, `pending_imu`, `inflight`, `leading_skipped`, `overflow`, `time_jumps`, `awaiting_images`, `initialization_seconds`, `run_dir`). The adapter receives compressed images into a bounded queue and decodes them on a separate thread; `input_queue_capacity` defaults to 64. An overflow fails acceptance and requires slower replay. After playback ends, wait until decoding and all expected image processing are complete, both image queues are empty and both in-flight flags are false; send `q` to flush tail IMU and output. Wait for stopped state and both core queues empty, then evaluate the run summary. Do not mistake “bag playback completed” for estimation completed.

Repeat with a distinct `output_dir` and `--rate 1.0` for performance. Report initialization time, trajectory coverage, tracking loss/reset counts, processing-time percentiles, total wall time and peak memory. No absolute accuracy or real-time threshold is implied.

```bash
ros2 run svo_benchmarking reference_report.py \
  /absolute/run_dir/imu_traj.txt \
  /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt \
  /absolute/report_dir --tolerance 0.01
```

The report associates overlapping times within 10 ms, fits SE(3) with scale fixed to one, and writes ATE position RMSE, one-second translation/rotation RPE, association counts/coverage, a trajectory plot and JSON. These metrics cannot establish independent absolute accuracy or prove absence of ROS1 regression.

## Troubleshooting

- Vendor revision/dirty-source failure: inspect the locked external checkout; do not edit its algorithm source or let colcon fetch a moving branch.
- Vocabulary missing: run `prepare_resources.py` and set `voc_path`/`voc_name` to the verified external files.
- Python/cv_bridge import failure: use the system interpreter and source the selected install; avoid mixed Conda/ROS environments.
- OpenCV mismatch: clear only the affected profile's CMake cache through a fresh build directory and supply the system `OpenCV_DIR`; inspect `ldd`.
- No poses: inspect decoded/processed counts, corrected IMU coverage, calibration, initialization states and tracking losses before interpreting reference metrics.
- rqt display unavailable: use a display server or `xvfb-run`; the GUI fixture supports `QT_QPA_PLATFORM=offscreen`.
- Timestamp reversal/overflow: preserve the failed run report, restart and rerun at a suitable rate. Never merge data from separate time epochs.
