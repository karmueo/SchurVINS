#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Original utility by Jonathan Huber, Apr 18 2018.
"""Right-multiply id + TUM poses by a constant 4x4 hand-eye matrix."""
import argparse
from pathlib import Path
import numpy as np
from svo_ceres_benchmarking.tfs_utils import quat2dcm, dcm2quat


def transform(source, matrix, output):
    values = np.atleast_2d(np.loadtxt(source))
    hand_eye = np.loadtxt(matrix, delimiter=',') if ',' in Path(matrix).read_text() else np.loadtxt(matrix)
    if values.shape[1] != 9 or hand_eye.shape != (4, 4):
        raise ValueError('Expected id timestamp xyz xyzw trajectory and 4x4 matrix')
    result = values.copy()
    for i, row in enumerate(values):
        pose = np.eye(4)
        pose[:3, :3] = quat2dcm(row[5:9])
        pose[:3, 3] = row[2:5]
        transformed = pose @ hand_eye
        result[i, 2:5] = transformed[:3, 3]
        result[i, 5:9] = dcm2quat(transformed[:3, :3])
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output, result, fmt=['%d'] + ['%.12f'] * 8,
               header='id timestamp tx ty tz qx qy qz qw')
    return len(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory')
    parser.add_argument('transformation', help='Whitespace-delimited 4x4 matrix')
    parser.add_argument('--output', default='transformed_poses.txt')
    args = parser.parse_args()
    print(f'Transformed {transform(args.trajectory, args.transformation, args.output)} poses')
