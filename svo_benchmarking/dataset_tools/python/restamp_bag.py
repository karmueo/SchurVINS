#!/usr/bin/env python3
# Original utility: www.asl.ethz.ch, IJRR MAV datasets. ROS2 port.
"""Copy a ROS2 bag using available standard message header timestamps."""
import argparse
from svo_ceres_benchmarking.rosbag2_io import restamp

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', '--ifile', required=True)
    parser.add_argument('-o', '--ofile', required=True, help='New bag output directory')
    parser.add_argument('--storage', choices=['mcap', 'sqlite3'], default='mcap')
    args = parser.parse_args()
    print(restamp(args.ifile, args.ofile, args.storage))
