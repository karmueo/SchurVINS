# Contributing

SchurVINS is a ROS2 Jazzy ament workspace targeting Ubuntu 24.04, C++17 and Python 3.12. Preserve existing licenses, source datasets, calibration files and unrelated working-tree changes.

## Packages and style

Core estimation lives in `svo/`; frontends in `svo_direct/`, `svo_tracker/` and `svo_img_align/`; optimization in `svo_ceres_backend/`, `svo_pgo/` and `svo_global_map/`. Shared types and tools live in `svo_common/`, `svo_vio_common/`, `vikit/` and `rpg_common/`. ROS2 nodes, components, parameters, launch and RViz2 resources are in `svo_ros/`. Offline tools and Python regression fixtures are in `svo_benchmarking/`; rqt is in `rqt_svo/`.

Follow adjacent formatting: two spaces in existing C++, snake_case files/variables, PascalCase types, camelCase methods and trailing underscores for members. Keep algorithm changes separate from compatibility changes. Use target-level ament dependencies and export installed headers, libraries and resources; all tools must work outside the source tree. Algorithm parameters are declared once and remain read-only after startup.

## Build and test

Follow [ROS2_JAZZY.md](doc/ROS2_JAZZY.md) to prepare locked dependencies. Source only `/opt/ros/jazzy/setup.bash` before building a profile; use independent build/install/log directories for default, full and traditional SVO.

```bash
cd ~/schurvins_ws
source /opt/ros/jazzy/setup.bash
export SCHURVINS_DEPENDENCY_DIR="$HOME/.cache/schurvins/dependencies"
colcon --log-base log-default build --base-paths src/SchurVINS \
  --build-base build-default --install-base install-default \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DUSE_SCHUR_VINS=ON -DUSE_LOOP_CLOSING=ON -DUSE_GLOBAL_MAP=OFF
source install-default/setup.bash
colcon --log-base log-default test --build-base build-default --install-base install-default
colcon test-result --test-result-base build-default --verbose
```

Register C++ tests with `ament_add_gtest` and Python fixtures with `ament_add_pytest_test` in the owning package. Name fixtures `test/test_<feature>.cpp` or `.py`. Execute relevant numerical, lifecycle and dataset checks; record skipped external-data tests separately. Never count compilation or a fixture without assertions as feature acceptance.

For estimator changes report sequence, calibration, start offset, replay rate, initialized/tracked states, accuracy and timing. Preserve custom Ceres manifold direction/layout/scaling and verify Jacobians when changing those interfaces. Run the same checks in traditional and full profiles when affected.

## Review and commits

Use focused descriptive commits. Explain the concrete issue, affected packages, commands and observed results. Disclose unexecuted validation and remaining limitations. Attach trajectories or RViz2 screenshots when relevant. Do not commit downloaded bags/vocabularies, generated logs/results, build/install trees, or source calibration modifications. Pull requests should reference relevant issues and distinguish independent truth from reference trajectories containing other VIO priors.
