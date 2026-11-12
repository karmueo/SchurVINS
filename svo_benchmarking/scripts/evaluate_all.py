#!/usr/bin/env python3
"""Run configured evaluations recursively, including isolated benchmark trial runs."""
import argparse
from pathlib import Path
from evaluate import evaluate_dataset

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('top_dir')
    args = parser.parse_args()
    configurations = sorted(Path(args.top_dir).resolve().rglob('evaluation_scripts.yaml'))
    if not configurations:
        raise SystemExit('No evaluation_scripts.yaml found')
    for configuration in configurations:
        evaluate_dataset(str(configuration.parent))
