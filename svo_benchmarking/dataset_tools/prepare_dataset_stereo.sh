#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 5 ]]; then
  echo "Usage: $0 ROS2_BAG LEFT_TOPIC RIGHT_TOPIC IMU_TOPIC OUTPUT_DIRECTORY" >&2
  exit 2
fi
bag=$(realpath "$1")
out=$(realpath -m "$5")
mkdir -p "$out/data"
ros2 run svo_benchmarking bag_to_image.py "$bag" "$2" 0 --output "$out/data/images_0.txt"
ros2 run svo_benchmarking bag_to_image.py "$bag" "$3" 1 --output "$out/data/images_1.txt"
ros2 run svo_benchmarking combine_images.py --left "$out/data/images_0.txt" --right "$out/data/images_1.txt" --output "$out/data/images.txt"
ros2 run svo_benchmarking bag_to_imu.py "$bag" "$4" --output "$out/data/imu.txt"
