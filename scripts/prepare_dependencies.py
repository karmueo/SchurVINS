#!/usr/bin/env python3
"""Fetch exact revisions before colcon; never modifies an existing dirty checkout."""
import argparse
import json
from pathlib import Path
import subprocess


def prepare(destination):
    lock = json.loads((Path(__file__).resolve().parents[1] / 'dependencies.lock.json').read_text())
    destination.mkdir(parents=True, exist_ok=True)
    for name, spec in lock['sources'].items():
        target = destination / name
        if not target.exists():
            subprocess.run(['git', 'clone', spec['url'], str(target)], check=True)
        if subprocess.check_output(['git', '-C', str(target), 'status', '--porcelain']).strip():
            raise RuntimeError(f'Refusing to modify dirty dependency: {target}')
        subprocess.run(['git', '-C', str(target), 'fetch', 'origin', spec['commit']], check=True)
        subprocess.run(['git', '-C', str(target), 'checkout', '--detach', spec['commit']], check=True)
    # Source checkouts are not ROS packages; only the repository vendor wrappers are built.
    (destination / 'COLCON_IGNORE').touch()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    prepare(parser.parse_args().directory.resolve())
