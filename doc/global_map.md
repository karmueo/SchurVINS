# Using SVO with iSAM2 based global map

As described in [SVO + OKVIS backend](./vio.md), fixed-lag smoothing is good for constant time update, but inevitably drifts over time. Using loop closure and pose graph to maintain global consistency is effective to a certain extent. However, lacking the ability to consider raw sensor measurements, loop closure + pose graph is limited in terms of accuracy. 

Therefore, we integrated a global bundle adjusted map using iSAM2. In short, once a keyframes goes out of the sliding window of the Ceres backend, we add it to a global visual-inertial bundle adjustment managed by iSAM2, which is executed in a separate thread. Moreover, in the visual front-end, we try to track points that are already optimized by iSAM2 (assuming iSAM2 converges fast enough and offers better accuracy).

The motivation of the design is to achieve high real-time accuracy by localizing against a globally consistent map at every frame, instead of relying on non-causal information to correct drift. The matching of the current frame against the global map is the same as in SVO - via direct tracking using patches. This is not very robust, but works quite well in mild motion (probably thanks to the gyroscopes).

One important implementation detail is that, to deal with occasionally indeterministic error with iSAM2, a weak prior of the landmark depth is added when the landmark is added to the global map (assuming the Ceres-based back-end provides reasonable initial estimate).

Note that the global map function is only tested with monocular configuration and is relatively unstable compared with the visual-inertial odometry part.



## ROS2 Jazzy operation

The global map remains disabled by default. Build `USE_GLOBAL_MAP=ON`; the existing monocular example additionally requires `USE_SCHUR_VINS=OFF` in its own build/install profile. The full profile also compiles the GTSAM interfaces for integration checks. In a sourced matching install:

```bash
ros2 launch svo_ros euroc_global_map_mono.launch.py \
  output_dir:="$HOME/schurvins-results/global-map" \
  vocabulary_dir:="$HOME/.cache/schurvins/resources/vocabularies"
ros2 bag play /absolute/ros2_euroc --clock --rate 0.25
```

Use verified official vocabulary resources and native ROS2 bags. Report real-data initialization, loop corrections, global-map updates, trajectory metrics and runtime independently; compilation alone does not establish functional acceptance. The following historical visualization explains landmarks managed by iSAM2 and fixed-anchor reprojections, without claiming Jazzy validation.

![](./images/mh03_gm.gif)

See [ROS2_JAZZY.md](ROS2_JAZZY.md) for isolated profiles, parameter migration and complete playback requirements.
