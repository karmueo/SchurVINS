# Prepare offline datasets

The installed tools read native ROS2 MCAP/SQLite3 files or bag directories. All paths are caller-selected; datasets do not belong in installed package share. Source Jazzy and the chosen SchurVINS install before running from any directory.

```text
dataset/
  calib.yaml
  dataset.yaml                 # optional dataset parameters
  data/
    images.txt                 # id timestamp image0 [image1 ...]
    img/                       # grayscale PNGs
    imu.txt                    # id timestamp wx wy wz ax ay az
    groundtruth.txt            # id timestamp xyz qx qy qz qw
    stamped_groundtruth.txt    # timestamp xyz qx qy qz qw
    groundtruth_matches.txt    # image_id reference_id
```

Timestamps are seconds; gyroscope units are rad/s and acceleration m/s². Images paths are relative to `data/`. Export does not apply EGO bias/time/extrinsic correction; use the EGO adapter/calibration workflow for that source, exactly once.

## Images and IMU

```bash
ros2 run svo_benchmarking prepare_dataset_monocular.sh /absolute/recording \
  /cam0/image_raw /imu /absolute/datasets/mono
ros2 run svo_benchmarking prepare_dataset_stereo.sh /absolute/recording.mcap \
  /cam0/image_raw /cam1/image_raw /imu /absolute/datasets/stereo
```

Both raw Image and CompressedImage are decoded to grayscale PNG. Stereo pairing requires exactly equal timestamps and rejects unmatched/duplicate entries. The original image lists remain available for inspection.

For individual exports or a four-camera dataset:

```bash
ros2 run svo_benchmarking bag_to_image.py /absolute/recording /cam0/image_raw 0 \
  --output /absolute/dataset/data/images_0.txt
ros2 run svo_benchmarking bag_to_imu.py /absolute/recording /imu \
  --output /absolute/dataset/data/imu.txt
```

Repeat image export per camera and construct a synchronized four-path index for the array offline entry. Supply an appropriate four-camera calibration and use the traditional build. Copy a verified calibration into the external dataset as `calib.yaml`; no calibration is inferred from bag messages.

## Reference and hand-eye transform

```bash
ros2 run svo_benchmarking generate_ground_truth.sh /absolute/recording /reference_pose \
  /absolute/datasets/stereo none
ros2 run svo_benchmarking generate_ground_truth.sh /absolute/recording /reference_pose \
  /absolute/datasets/stereo /absolute/hand_eye.txt
```

Supported messages are PoseStamped, PoseWithCovarianceStamped, Odometry, PointStamped (identity rotation), and TransformStamped. The hand-eye file is a whitespace- or comma-delimited 4×4 matrix; poses are right-multiplied by it. The wrapper associates image/reference timestamps and creates a TUM reference without changing source files.

EuRoC CSV reference conversion:

```bash
ros2 run svo_benchmarking asl_groundtruth_to_pose.py /absolute/state_groundtruth/data.csv \
  --output /absolute/dataset/data/groundtruth.txt
ros2 run svo_benchmarking strip_gt_id.py /absolute/dataset/data/groundtruth.txt
```

## Native bag rewriting

```bash
ros2 run svo_benchmarking restamp_bag.py -i /absolute/source -o /absolute/new_bag --storage mcap
```

Output must not already exist. Available standard message headers become bag record timestamps. Unknown custom messages retain serialized payload and original record timestamps, so their message packages need not be installed. Storage choices are `mcap` and `sqlite3`; rewriting neither changes message header stamps nor calibrates sensors.

Other installed helpers include `prepend_id.py`, `swap_stamped_imu_meas.py`, `change_image_path.py`, `combine_images.py`, `associate_timestamps.py` and `transform_trajectory.py`; inspect `--help` for arguments. Review the output format before feeding a different dataset into the estimator.
