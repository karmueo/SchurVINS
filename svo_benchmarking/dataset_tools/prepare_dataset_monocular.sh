#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 4 ]]; then
  echo "Usage: $0 ROS2_BAG IMAGE_TOPIC IMU_TOPIC OUTPUT_DIRECTORY" >&2
  exit 2
fi
bag=$(realpath "$1")
out=$(realpath -m "$4")
mkdir -p "$out/data"
ros2 run svo_benchmarking bag_to_image.py "$bag" "$2" 0 --output "$out/data/images.txt"
ros2 run svo_benchmarking bag_to_imu.py "$bag" "$3" --output "$out/data/imu.txt"
