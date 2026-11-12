#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 4 ]]; then
  echo "Usage: $0 ROS2_BAG POSE_TOPIC OUTPUT_DIRECTORY HAND_EYE_MATRIX_OR_none" >&2
  exit 2
fi
bag=$(realpath "$1")
out=$(realpath -m "$3")
mkdir -p "$out/data"
ros2 run svo_benchmarking bag_to_pose.py "$bag" "$2" --output "$out/data/groundtruth.txt"
if [[ "$4" != none ]]; then
  ros2 run svo_benchmarking transform_trajectory.py "$out/data/groundtruth.txt" "$(realpath "$4")" --output "$out/data/groundtruth.txt"
fi
ros2 run svo_benchmarking associate_timestamps.py "$out/data/images.txt" "$out/data/groundtruth.txt" --output "$out/data/groundtruth_matches.txt"
ros2 run svo_benchmarking strip_gt_id.py "$out/data/groundtruth.txt"
