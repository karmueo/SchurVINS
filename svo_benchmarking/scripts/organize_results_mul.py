#!/usr/bin/env python3
"""Collect estimate variants/trials from legacy or nested ROS2 result folders."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from shutil import copy2
import yaml

kNsToEstFnMapping = {'traj_est': 'stamped_traj_estimate',
                     'pose_graph': 'stamped_pose_graph_estimate',
                     'ba_estimate': 'stamped_ba_estimate'}
kNsToAlgoMapping = {'traj_est': 'svo_ceres_rp', 'pose_graph': 'svo_ceres_pg',
                    'ba_estimate': 'svo_ceres_ba'}


def organize(root, output, platform, estimate_types, max_trials=None):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output == root or root in output.parents:
        raise ValueError('Organized output must be outside the input result tree')
    groups = defaultdict(list)
    for reference in sorted(root.rglob('stamped_groundtruth.txt')):
        folder = reference.parent
        params = yaml.safe_load((folder / 'params.yaml').read_text()) if (folder / 'params.yaml').exists() else {}
        dataset = str((params or {}).get('dataset_name', folder.relative_to(root))).replace('/', '_') or root.name
        for kind in estimate_types:
            for estimate in sorted(folder.glob(kNsToEstFnMapping[kind] + '*.txt')):
                groups[(dataset, kind)].append((estimate, reference))
    if not groups:
        raise ValueError(f'No requested estimate/reference pairs under {root}')
    manifest = []
    for (dataset, kind), pairs in sorted(groups.items()):
        if max_trials is not None:
            pairs = pairs[:max_trials]
        target = output / platform / kNsToAlgoMapping[kind] / dataset
        target.mkdir(parents=True, exist_ok=True)
        for trial, (estimate, reference) in enumerate(pairs):
            destination = target / f'{kNsToEstFnMapping[kind]}{trial}.txt'
            if destination.exists():
                raise FileExistsError(f'Refusing to overwrite {destination}')
            copy2(estimate, destination)
            copy2(reference, target / 'stamped_groundtruth.txt')
            manifest.append({'source': str(estimate), 'output': str(destination), 'trial': trial})
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir')
    parser.add_argument('--save_dir')
    parser.add_argument('--platform', default='local')
    parser.add_argument('--est_types', nargs='+', choices=sorted(kNsToEstFnMapping), default=['traj_est', 'pose_graph'])
    parser.add_argument('--mul_trials', type=int, help='Maximum trials per dataset and estimate type')
    args = parser.parse_args()
    if args.mul_trials is not None and args.mul_trials < 1:
        parser.error('--mul_trials must be positive')
    output = args.save_dir or str(Path(args.result_dir).resolve()) + '_organized'
    print(json.dumps(organize(args.result_dir, output, args.platform, args.est_types, args.mul_trials), indent=2))
