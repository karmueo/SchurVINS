"""Filesystem and argument rules shared by EGO launch and its tests."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'Expected true or false, got {value!r}')
    return value.lower() == 'true'


def output_directory(explicit, bag_name, *, cwd=None, now=None):
    if explicit:
        output = Path(explicit).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        return output
    shanghai = ZoneInfo('Asia/Shanghai')
    timestamp = (now or datetime.now(shanghai)).astimezone(shanghai).strftime('%Y%m%d%H%M%S')
    root = Path(cwd or Path.cwd()) / 'output'
    root.mkdir(parents=True, exist_ok=True)
    name = f'result_{timestamp}_{bag_name}'
    suffix = 0
    while True:
        output = root / (name if suffix == 0 else f'{name}_{suffix}')
        try:
            output.mkdir()
            return output.resolve()
        except FileExistsError:
            suffix += 1


def check_outputs(output, bag_name, offline, save_trajectory):
    paths = []
    if offline:
        paths += [output / f'schurvins_{bag_name}.mcap',
                  output / f'.schurvins_{bag_name}.bag',
                  output / 'summary.json', output / 'frames.csv', output / 'effective_config.yaml']
    if save_trajectory:
        paths.append(output / f'schurvins_{bag_name}_trajectory.txt')
    for path in paths:
        if path.exists():
            raise FileExistsError(f'Refusing to overwrite {path}')
