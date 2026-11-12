# Offline experiments and evaluation on ROS2 Jazzy

All tools use Python 3.12 and installed ament packages. Keep datasets, calibration and outputs in external writable directories. Read [dataset preparation](dataset_tools/README.md) for native ROS2 MCAP/SQLite3 exports; no ROS1 bag reader is provided.

## Batch offline experiments

Source the selected workspace profile. Mono, visual-only and array require `USE_SCHUR_VINS=OFF`. A dataset contains `calib.yaml`, `data/images.txt` and optional `data/imu.txt`, reference and timestamp matches. Stereo image rows have two image paths; array rows have four. Paths are relative to `data/` unless absolute.

```bash
ros2 run svo_benchmarking benchmark.py /absolute/path/experiment.yaml \
  --dataset-root /absolute/datasets --output-dir /absolute/experiments --n_trials 3
```

An experiment name without a path is read from the installed `share/svo_benchmarking/experiments`. Existing example YAML settings remain available. `datasets[].name` resolves under `--dataset-root`; `datasets[].directory` can override it with a relative or absolute directory. `SVO_DATASET_DIR` or top-level `dataset_root` in the experiment YAML is an alternative. Calibration defaults to each dataset's `calib.yaml` and may be changed with `calib_name`.

Each trial gets its own directory, parameter YAML, CPU/memory samples and the runtime's unique `benchmark_*` subdirectory. The actual trace directory receives effective parameters and available reference files. No output is written to installed package share. Command failures stop the batch instead of being counted as successful runs. `--n_aligned` and `--align_type` preserve legacy configuration metadata; the reference report below always uses no-scale SE(3).

An individual offline run can also invoke the installed `svo_benchmark` executable with a ROS2 parameter file selecting `dataset_directory`, `dataset_is_stereo`/`dataset_is_array`, calibration, estimator settings and `output_dir`. Use the traditional build for mono/array. Four-camera image indexes must have `id timestamp image0 image1 image2 image3` on each row.

## Reference metrics

Inputs are finite TUM rows `timestamp tx ty tz qx qy qz qw`, strictly increasing in time. The tool fits SE(3), never scale, and reports position ATE RMSE and one-second translation/rotation RPE. Default timestamp tolerance is 10 ms. Missing one-second pairs are reported as zero pairs/null metrics rather than fabricated values.

```bash
ros2 run svo_benchmarking reference_report.py /absolute/imu_traj.txt \
  /absolute/reference_tum.txt /absolute/report --tolerance 0.01
ros2 run svo_benchmarking reference_report.py /absolute/results /absolute/batch-reports --batch
ros2 run svo_benchmarking analyze_all_euroc.sh /absolute/results /absolute/batch-reports
```

Batch mode recursively finds `stamped_groundtruth.txt` and each available stamped estimate variant/trial (or `imu_traj.txt`) in the same folder. It writes individual JSON/plots plus `batch_summary.json`. Explicit per-run commands remain appropriate when reference and estimate live in different folders. The EGO reference contains other VIO priors and is not independent ground truth; accuracy claims must preserve that qualification.

Legacy id-keyed results can be converted locally:

```bash
ros2 run svo_benchmarking convert_old_fmt.py /absolute/legacy-results
```

The converter uses `groundtruth_matches.txt` and `groundtruth.txt` for an `id xyz xyzw` estimate, or strips the id from `id timestamp xyz xyzw`; outputs are `stamped_groundtruth.txt` and `stamped_traj_estimate0.txt`. It does not depend on a repository-external trajectory evaluator.

`organize_results.py` accepts an explicit source path and `--output-dir`; `organize_results_mul.py` accepts `--save_dir`. `evaluate.py`/`evaluate_all.py` execute the explicitly configured ROS2 evaluation commands with checked exit status. Plot utilities work without a system LaTeX installation.

## Regression fixtures

The owning ament package registers Python fixtures for native MCAP/SQLite3 reader/writer, typed exports, no-scale trajectory metrics and GUI lifecycle. In a sourced install, individual fixtures can be run directly from the source checkout:

```bash
python3 -m unittest discover -s /absolute/SchurVINS/svo_benchmarking/test -v
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s /absolute/SchurVINS/rqt_svo/test -v
```

These are execution instructions, not recorded passing results. Native storage tests require both rosbag2 storage plugins, cv_bridge, sensor/geometry/nav messages and system NumPy. Metric fixtures require SciPy/Matplotlib; GUI fixtures need python_qt_binding and an offscreen/display backend. Record datasets and feature fixtures separately from compilation results.
