# Visual frontend on ROS2 Jazzy

Use a separate traditional build (`USE_SCHUR_VINS=OFF`) for monocular, IMU-free or four-camera frontend operation. The estimator's algorithm settings remain in installed YAML files; copy a configuration to an external path before customization. Use dot-separated keys inside `ros__parameters` and pass explicit calibration/output paths.

```bash
ros2 launch svo_ros euroc_visual_mono.launch.py output_dir:="$HOME/schurvins-results/visual-mono"
ros2 launch svo_ros euroc_visual_stereo.launch.py output_dir:="$HOME/schurvins-results/visual-stereo"
```

Run one launch at a time. Native ROS2 MCAP/SQLite3 playback must publish the expected left/right topics and `/clock`:

```bash
ros2 bag play /absolute/recording --clock --rate 0.25
```

For custom cameras use generic `svo.launch.py` with `pipeline:=mono` or `stereo`, `use_imu:=false`, `calib_file:=/absolute/calibration.yaml`, `params:=/absolute/parameters.yaml` and `output_dir:=/absolute/results`. Pinhole, radial-tangential, equidistant and omnidirectional model configuration must match the source calibration; see [calibration](../calibration.md). The repository's `fisheye` model is not the equidistant model.

Hardware examples use standard ROS2 component/topic interfaces, without external camera drivers. `live_components.launch.py` loads the SVO component for standard `/cam0/image_raw`, `/cam1/image_raw` and `/imu`; use the appropriate configuration and remappings. Four-camera data is processed by the traditional offline benchmark with four image paths per row and a four-camera calibration, as documented in [dataset preparation](../../svo_benchmarking/dataset_tools/README.md).

Important tuning settings include `max_fts`, `max_n_kfs`, feature-grid `cell_size`, keyframe disparity/distance thresholds, image-alignment pyramid levels, affine illumination compensation and pose-optimization thresholds. Preserve algorithm defaults when checking migration behavior and report any dataset-specific changes. Follow [ROS2_JAZZY.md](../ROS2_JAZZY.md) for build profiles, GUI, lifecycle and validation.
