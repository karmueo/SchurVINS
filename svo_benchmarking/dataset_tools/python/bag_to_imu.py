#!/usr/bin/env python3
"""Export ROS2 Imu to id timestamp gyro[rad/s] acceleration[m/s^2]."""
import argparse
from svo_ceres_benchmarking.data_export import export_imu

extract = export_imu

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('topic')
    parser.add_argument('--output', default='imu.txt')
    args = parser.parse_args()
    print(f'Wrote {extract(args.bag, args.topic, args.output)} IMU rows to {args.output}')
