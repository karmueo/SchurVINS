#!/usr/bin/env python3
"""Export ROS2 Image or CompressedImage to grayscale PNGs and an image index."""
import argparse
from svo_ceres_benchmarking.data_export import export_images

extract = export_images

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('topic')
    parser.add_argument('cam_id')
    parser.add_argument('--output', default='images.txt')
    args = parser.parse_args()
    print(f'Wrote {extract(args.bag, args.topic, args.output, args.cam_id)} images to {args.output}')
