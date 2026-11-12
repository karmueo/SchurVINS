from datetime import datetime, timezone
from pathlib import Path
import runpy
import pytest

helpers = runpy.run_path(str(Path(__file__).parents[1] / 'scripts' / 'ego_launch_support.py'))


def test_shanghai_directory_and_collision(tmp_path):
    now = datetime(2026, 11, 12, 1, 2, 3, tzinfo=timezone.utc)
    first = helpers['output_directory']('', 'job_002', cwd=tmp_path, now=now)
    second = helpers['output_directory']('', 'job_002', cwd=tmp_path, now=now)
    assert first == tmp_path / 'output' / 'result_20261112090203_job_002'
    assert second == tmp_path / 'output' / 'result_20261112090203_job_002_1'
    explicit = helpers['output_directory'](str(tmp_path / 'chosen'), 'job_002')
    assert explicit == tmp_path / 'chosen'


def test_output_overwrite_and_live_mode(tmp_path):
    existing = tmp_path / 'schurvins_job_002.mcap'
    existing.write_bytes(b'original')
    with pytest.raises(FileExistsError):
        helpers['check_outputs'](tmp_path, 'job_002', True, False)
    assert existing.read_bytes() == b'original'
    helpers['check_outputs'](tmp_path, 'live', False, False)
    trajectory = tmp_path / 'schurvins_live_trajectory.txt'
    trajectory.touch()
    with pytest.raises(FileExistsError):
        helpers['check_outputs'](tmp_path, 'live', False, True)


def test_boolean_parameters():
    assert helpers['boolean']('true') is True
    assert helpers['boolean']('FALSE') is False
    with pytest.raises(ValueError):
        helpers['boolean']('perhaps')
