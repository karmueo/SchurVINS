#!/usr/bin/env python3
"""
@author: Christian Forster
"""

import os
import argparse
import yaml
import subprocess

from colorama import init, Fore

init(autoreset=True)


def run_evaluation_script(trace_dir, script):
    command = ['ros2', 'run', script['ros_node'], script['ros_executable']]
    for value in script.get('params', []):
        command.append(trace_dir if value == 'trace_dir' else str(value))
    for key, value in script.get('flags', {}).items():
        command.append(f'--{key}={value}')
    print('Starting:', command)
    subprocess.run(command, check=True)


def evaluate_dataset(trace_dir):
    if os.path.exists(os.path.join(trace_dir, 'evaluation_scripts.yaml')):
        eval_scripts = yaml.safe_load(
            open(os.path.join(trace_dir, 'evaluation_scripts.yaml'), 'r'))
        for script in eval_scripts:
            run_evaluation_script(trace_dir, script)
    else:
        print(('Folder "'+trace_dir +
              '" does not contain a file "evaluation_scripts.yaml".'))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''
    Evaluates tracefiles of SVO and generates the plots.
    The experiment folder should contain an "evaluation_scripts.yaml" that
    specifies scripts to run.
    ''')
    parser.add_argument(
        'experiment_dir', help='directory name of the tracefiles')
    args = parser.parse_args()

    evaluate_dataset(args.experiment_dir)
