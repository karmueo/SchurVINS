# SchurVINS
## SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System
We propose a filter-based VINS framework named SchurVINS, which guarantees both high accuracy by building a complete residual model and low computational complexity with Schur complement. Technically, we first formulate the full residual model where Gradient, Hessian and observation covariance are explicitly modeled. Then Schur complement is employed to decompose the full model into ego-motion residual model and landmark residual model. Finally, Extended Kalman Filter (EKF) update is implemented in these two models with high efficiency. Experiments on EuRoC and TUM-VI datasets show that SchurVINS notably outperforms state-of-the-art (SOTA) methods in both accuracy and computational complexity. The main contributions include:
- An equivalent residual model is proposed to deal with hyper high-dimension observations, which consists of gradient, Hessian and the corresponding observation covariance. This method is of great generality in EKF systems.
- A lightweight EKF-based landmark solver is proposed to
estimate position of landmarks with high efficiency.
- A novel EKF-based VINS framework is developed to
achieve ego-motion and landmark estimation simultaneously with high accuracy and efficiency.

## 1. License

The code is licensed under GPLv3.

The SchurVINS is developed on SVO2.0(**[rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)**), and thus its license is retained at the beginning of the related files.

**Related Publication:**  
Yunfei Fan, Tianyu Zhao, Guidong Wang. SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System. (Accepted by CVPR 2024).**[PDF](https://arxiv.org/pdf/2312.01616.pdf)**.  

## 2. ROS2 Jazzy setup

The migration targets **Ubuntu 24.04, ROS2 Jazzy, C++17 and Python 3.12** with system Eigen 3.4, OpenCV 4.6, Ceres 2.2, yaml-cpp 0.8, GTSAM 4.2 and OpenGV. Use the [Jazzy installation and operating guide](doc/ROS2_JAZZY.md) for dependency installation, fixed vendor revisions, three isolated build profiles and troubleshooting. Validation status must be taken from the generated acceptance reports; the commands below are not a claim that every feature has passed validation.

After installing Jazzy and the dependencies documented in that guide:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
mkdir -p ~/schurvins_ws/src
cd ~/schurvins_ws/src
git clone https://github.com/bytedance/SchurVINS.git
export SCHURVINS_DEPENDENCY_DIR="$HOME/.cache/schurvins/dependencies"
/usr/bin/python3 SchurVINS/scripts/prepare_dependencies.py "$SCHURVINS_DEPENDENCY_DIR"
/usr/bin/python3 SchurVINS/scripts/prepare_resources.py "$HOME/.cache/schurvins/resources"
cd ~/schurvins_ws
colcon --log-base log-default build --base-paths src/SchurVINS \
  --build-base build-default --install-base install-default \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DUSE_SCHUR_VINS=ON -DUSE_LOOP_CLOSING=ON -DUSE_GLOBAL_MAP=OFF
source install-default/setup.bash
```

The default estimator is stereo SchurVINS with IMU. Build the separate traditional profile for monocular, visual-only and four-camera offline processing. Global mapping uses the full profile; it remains disabled by default. See [contributing](CONTRIBUTING.md) for the package layout and checks. Actual build, feature and dataset results are recorded in the [migration validation matrix](doc/MIGRATION_VALIDATION.md).

## 3. EGO stereo + IMU

Process a complete MCAP directly, with no separate `ros2 bag play` command:

```bash
ros2 launch svo_ros ego.launch.py \
  inbag:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  cameras:=/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  save_trajectory:=true
```

`realtime:=false` (default) processes as fast as possible; `realtime:=true` follows the original recording time at 1× and allows delay without dropping frames. The process prints average VIO FPS, excluding bag IO, JPEG decoding, format conversion, publishing and waits, then exits automatically. `rviz:=true` enables live inspection and closes RViz on completion.

Results default to `./output/result_YYYYMMDDHHMMSS_job_002/`, using Shanghai time. Set `output_dir:=/absolute/path` to write directly into that directory. `schurvins_job_002.mcap` preserves all original topics and adds `/svo/odometry` and `/svo/path_imu`. With `save_trajectory:=true`, `schurvins_job_002_trajectory.txt` contains TUM IMU/body poses (`timestamp tx ty tz qx qy qz qw`, seconds and meters). The default does not save trajectory text. Existing result files are never overwritten.

Run the complete offline acceptance check with a new output directory; add `--realtime` for the separate 1× run:

```bash
/usr/bin/python3 scripts/accept_ego_offline.py \
  --bag /data/scl_data/slam/stereo_live_bags/EGO/800hz/job_002.mcap \
  --calibration /data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml \
  --output /tmp/schurvins-ego-offline \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/800hz/output/GT/trajectory_gt_imu.txt
```

Inspect `acceptance.json`, `summary.json`, `frames.csv`, and the `reference/` report. The check compares every original topic's serialized messages and recording timestamps and verifies odom/Path/TUM consistency. The source EGO reference contains other VIO priors and is not independent ground truth.

Without `inbag`, the launch retains the live topic adapter. It splits each 3840×1080 JPEG into two 1920×1080 grayscale images and applies the existing IMU timestamp and fixed-bias correction once. Calibration conversion preserves the pinhole + equidistant model and inverse camera/IMU extrinsics. Live mode does not create an MCAP; optional text output is named `schurvins_live_trajectory.txt`. The DDS readiness and drain procedure, plus `scripts/accept_ego.py` for external playback acceptance, remain in the [Jazzy guide](doc/ROS2_JAZZY.md).

## 4. Other inputs and tools

```bash
ros2 launch svo_ros euroc_vio_stereo.launch.py output_dir:="$HOME/schurvins-results/euroc"
ros2 bag play /absolute/path/to/ros2_euroc --clock --rate 0.25
ros2 run rqt_svo rqt_svo
```

Input bags must be native ROS2 MCAP or SQLite3. Dataset export and batch offline experiments are documented in [svo_benchmarking](svo_benchmarking/README.md); trajectory comparison reports no-scale SE(3) ATE and one-second translation/rotation RPE. The source EGO reference contains other VIO priors and is not independent ground truth.

# 5. Reproducibility
There might be minor differences between the released version and the results in the paper. Please note that multi-thread performance has some randomness due to CPU utilization.
On EuRoC datasets, please note to skip the first few seconds of datasets with the following configurations for reproducibility.


|||||||||||
|----|------|----|----|------|----|----|------|----|----|
|**MH1**|**MH2**|**MH3**|**MH4**|**MH5**|**V11**|**V12**|**V13**|**V21**|**V22**|
|30s|30s|10s|13s|15s|0s|0s|2s|0s|0s|
|||||||||||

# 6. Security

If you discover a potential security issue in this project, or think you may
have discovered a security issue, we ask that you notify Bytedance Security via our [security center](https://security.bytedance.com/src) or [vulnerability reporting email](sec@bytedance.com).

# 7. Acknowledgement
This work incorporates the well-known SVO2.0 open-source code. We extend our gratitude to the authors of the software.
- [rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)


# 8. Citation
If you found this code/work to be useful in your own research, please considering citing the following information. Additionally, please considering citing SVO2.0([rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)) since SchurVINS is developed on it.
```bibtex
@inproceedings{fan2024schurvins,
  title={SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System},
  author={Fan, Yunfei and Zhao, Tianyu and Wang, Guidong},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={17964--17973},
  year={2024}
}
```

# 9. We are Hiring!
Our team is hiring FTEs with background in Deep Learning, SLAM, and 3D Vision. We are based in Beijing and Shanghai. If you are interested, please send your resume to frank.01[AT]bytedance[DOT]com.
