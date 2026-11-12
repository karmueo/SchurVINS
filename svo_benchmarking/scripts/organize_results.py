#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import shutil


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Organize results for analysis.')
    parser.add_argument('results_dir',
                        help="Each sub-folder under this should be the "
                        "results for one dataset, and the folder name will "
                        "be used as the dataset name.")
    parser.add_argument('platform',
                        help='top level: <platform>/<config>')
    parser.add_argument('config',
                        help='second level: <platform>/<config>')
    parser.add_argument('-success_flag',
                        help='create a SUCESS flag in the result folder',
                        action='store_true')
    parser.add_argument('--output-dir', default=str(Path.home() / 'schurvins-results' / 'organized'))
    args = parser.parse_args()

    results_top = os.path.abspath(os.path.expanduser(args.output_dir))
    print(("Top directory for results is {0}".format(results_top)))

    results_from = os.path.abspath(os.path.expanduser(args.results_dir))
    assert os.path.isdir(results_from), \
        "Specified results_dir does not exist {0}".format(results_from)
    print(("Will organize results from {0}".format(results_from)))

    results_to = os.path.join(results_top, args.platform+'/'+args.config)
    if os.path.isdir(results_to):
        print("Target directory already exists, possible to overwrite.")
    else:
        os.makedirs(results_to)
    print(("The target folder is {0}".format(results_to)))

    prefix = args.platform + '_' + args.config + '_'

    dataset_names = [d for d in os.listdir(results_from)
                     if os.path.isdir(os.path.join(results_from, d))]
    dataset_names.sort()
    print(("Found datasets: {0}".format(dataset_names)))

    for d in dataset_names:
        from_d = os.path.join(results_from, d)
        assert os.path.isdir(from_d)

        to_d = os.path.join(results_to, prefix+d)
        if not os.path.isdir(to_d):
            os.makedirs(to_d)
        suc_flag = os.path.join(to_d, 'SUCCESS')
        if args.success_flag and not os.path.isfile(suc_flag):
            Path(suc_flag).touch()

        print(("{0} -> {1}".format(from_d, to_d)))
        for f in os.listdir(from_d):
            source, destination = os.path.join(from_d, f), os.path.join(to_d, f)
            if os.path.isdir(source):
                shutil.copytree(source, destination, dirs_exist_ok=True)
            elif os.path.isfile(source):
                shutil.copy(source, destination)
