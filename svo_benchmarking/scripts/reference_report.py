#!/usr/bin/env python3
"""No-scale SE(3) ATE and one-second RPE; supports individual or recursive batch runs."""
import argparse
import json
from pathlib import Path
from svo_ceres_benchmarking.reference_report import report


def batch(root, output, tolerance):
    summaries = []
    for reference in sorted(Path(root).rglob('stamped_groundtruth.txt')):
        folder = reference.parent
        estimates = sorted(set(folder.glob('stamped_*estimate*.txt')) | set(folder.glob('trajectory_imu.txt')) | set(folder.glob('imu_traj.txt')))
        for estimate in estimates:
            target = Path(output) / folder.relative_to(Path(root)) / estimate.stem
            result = report(estimate, reference, target, tolerance)
            summaries.append({'estimate': str(estimate), 'reference': str(reference), **result})
    if not summaries:
        raise ValueError(f'No estimate/reference pairs found under {root}')
    Path(output).mkdir(parents=True, exist_ok=True)
    (Path(output) / 'batch_summary.json').write_text(json.dumps(summaries, indent=2) + '\n')
    return summaries


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimate', help='Estimate file, or result root with --batch')
    parser.add_argument('reference', help='Reference file, or output directory with --batch')
    parser.add_argument('output', nargs='?')
    parser.add_argument('--batch', action='store_true')
    parser.add_argument('--tolerance', type=float, default=0.01)
    args = parser.parse_args()
    if args.batch:
        result = batch(Path(args.estimate).resolve(), args.reference, args.tolerance)
    else:
        if not args.output:
            parser.error('output directory is required for a single trajectory')
        result = report(args.estimate, args.reference, args.output, args.tolerance)
    print(json.dumps(result, indent=2))
