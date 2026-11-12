#!/usr/bin/env python3
"""Convert legacy id-keyed benchmark estimates to TUM using groundtruth matches."""
import argparse
from pathlib import Path
import numpy as np


def convert(directory):
    directory = Path(directory)
    groundtruth = np.atleast_2d(np.loadtxt(directory / 'groundtruth.txt'))
    estimate = np.atleast_2d(np.loadtxt(directory / 'traj_estimate.txt'))
    if groundtruth.shape[1] != 9:
        raise ValueError('groundtruth.txt must be id timestamp xyz xyzw')
    np.savetxt(directory / 'stamped_groundtruth.txt', groundtruth[:, 1:], fmt='%.12f')
    if estimate.shape[1] == 9:
        stamped = estimate[:, 1:]
    elif estimate.shape[1] == 8:
        matches = np.atleast_2d(np.loadtxt(directory / 'groundtruth_matches.txt', dtype=int))
        reference_stamps = {int(row[0]): row[1] for row in groundtruth}
        matched_ids = {int(row[0]): int(row[1]) for row in matches}
        rows = []
        for row in estimate:
            gt_id = matched_ids.get(int(row[0]))
            if gt_id in reference_stamps:
                rows.append([reference_stamps[gt_id], *row[1:]])
        stamped = np.asarray(rows)
        if not len(stamped):
            raise ValueError(f'No matched legacy estimates in {directory}')
    else:
        raise ValueError('Estimate must be id xyz xyzw or id timestamp xyz xyzw')
    np.savetxt(directory / 'stamped_traj_estimate0.txt', stamped, fmt='%.12f')
    return len(stamped)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir')
    args = parser.parse_args()
    root = Path(args.result_dir).resolve()
    folders = [root] if (root / 'traj_estimate.txt').exists() else sorted(p for p in root.iterdir() if p.is_dir())
    for folder in folders:
        print(f'{folder}: {convert(folder)} poses')
