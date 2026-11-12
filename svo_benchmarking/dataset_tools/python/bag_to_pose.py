#!/usr/bin/env python3
"""Export supported ROS2 stamped poses to id + TUM rows."""
import argparse
from svo_ceres_benchmarking.data_export import export_pose

extract = export_pose

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='ROS2 MCAP/SQLite3 file or bag directory')
    parser.add_argument('topic')
    parser.add_argument('--output', default='groundtruth.txt')
    args = parser.parse_args()
    print(f'Wrote {extract(args.bag, args.topic, args.output)} poses to {args.output}')
