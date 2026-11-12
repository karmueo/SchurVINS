# Visual-inertial Odometry with SVO frontend

The visual-inertial odometry consists of the SVO 2.0 frontend and OKVIS style sliding window backend. Both monocular and stereo setups are supprted.

Loop closure and correction using DBoW2 is also supported, but we are relative conservative on regarding loop corrections (i.e., only very good match results after 2D-2D/2D-3D check + large pose correction will be accepted). In brief, we additionally extract ORB features for the purpose of loop closure, and the depths of these ORB features are estimated using depth filters.

## ROS2 Jazzy operation

Build the traditional profile (`USE_SCHUR_VINS=OFF`) for monocular VIO and the Ceres frontend/backend combination. The default profile runs stereo SchurVINS + IMU. In a sourced selected install:

```bash
ros2 launch svo_ros euroc_vio_mono.launch.py output_dir:="$HOME/schurvins-results/mono"
ros2 bag play /absolute/ros2_euroc --clock --rate 0.25
```

For default stereo use `euroc_vio_stereo.launch.py`. The input must be native ROS2 MCAP or SQLite3. Preserve the documented sequence start offsets when comparing runs. Runtime parameters are under `ros__parameters` with dot-separated names; calibration uses camera poses `T_B_C`. See [ROS2_JAZZY.md](ROS2_JAZZY.md) for configuration, component lifecycle and validation requirements.

The historical image below illustrates the Ceres backend landmarks and loop constraints; it is not a Jazzy validation result.

![](./images/v202_mono_vio.gif)
