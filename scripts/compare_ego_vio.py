#!/usr/bin/env python3
"""Serial EGO parameter screening, repeated confirmation and isolated RPG evaluation."""
import argparse
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
EVALUATOR = ROOT.parent / 'rpg_trajectory_evaluation'
DATA = Path('/data/scl_data/slam/stereo_live_bags/EGO/800hz')
SCREEN = {
    'baseline': {}, 'window6': {'window_size': 6}, 'window8': {'window_size': 8},
    'noise1': {'obs_dev': 1.0}, 'noise3': {'obs_dev': 3.0},
    'features300': {'max_fts': 300, 'max_map_fts': 500},
    'features450': {'max_fts': 450, 'max_map_fts': 750},
}
LOOP_COUNTERS = ('detected', 'corrections_applied', 'pgo_completed')


def shell(command, log, timeout=1800, cwd=ROOT):
    with log.open('w') as stream:
        process = subprocess.Popen(['bash', '--noprofile', '--norc', '-c', command],
                                   cwd=cwd, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True,
                                   env={**os.environ, 'MPLBACKEND': 'Agg'})
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise
    if code:
        raise RuntimeError(f'Exit {code}: {log}')


def environment(build):
    install = ROOT / 'ros2_ws/install' / build
    setup = shlex.quote(str(install / 'local_setup.bash'))
    prefixes = ' | '.join(shlex.quote(str(path)) for path in (install, install / 'svo_ros'))
    return (
        'unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX '
        'CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION\n'
        'export PATH=/usr/bin:/bin\n'
        'source /opt/ros/jazzy/setup.bash || exit $?\n'
        f'source {setup} || exit $?\n'
        'svo_ros_prefix=$(ros2 pkg prefix svo_ros) || exit $?\n'
        'case "$svo_ros_prefix" in\n'
        f'    {prefixes}) ;;\n'
        '    *) printf "Unexpected svo_ros prefix: %s\\n" "$svo_ros_prefix" >&2; exit 1 ;;\n'
        'esac\n'
    )


def complete_loop_statistics(stats):
    return isinstance(stats, dict) and all(
        type(stats.get(key)) is int and stats[key] >= 0 for key in LOOP_COUNTERS)


def evaluate(trajectory, destination, reference, plots=True):
    summaries = {}
    for alignment in ('se3', 'sim3'):
        work = destination.resolve() / alignment
        work.mkdir(parents=True)
        # Config and match caches are read from the estimate directory, not --output_dir.
        (work / 'estimate.txt').symlink_to(trajectory.resolve())
        (work / 'eval_cfg.yaml').write_text(yaml.safe_dump({'align_type': alignment, 'align_num_frames': -1}))
        command = ['uv', 'run', '--offline', 'analyze_trajectory_single', str(reference),
                   str(work / 'estimate.txt'), '--output_dir', str(work), '--recalculate_errors']
        if not plots:
            command.append('--no_plot')
        shell(shlex.join(command), work / 'evaluation.log', cwd=EVALUATOR)
        stats = list((work / 'saved_results').rglob(f'absolute_err_statistics_{alignment}_-1.yaml'))
        if len(stats) != 1:
            raise RuntimeError(f'Expected one absolute statistics file in {work}: {stats}')
        summaries[alignment] = yaml.safe_load(stats[0].read_text())
    return summaries


def write_report(output, rows):
    lines = ['# EGO VIO 配置比较', '', '参考轨迹含其他 VIO 先验；以下不是独立绝对精度。', '',
             '| 配置 | 模式 | 试次 | 有效 | SE3 ATE m | Sim3 ATE m | 耗时 s | 丢失 |',
             '|---|---|---:|---|---:|---:|---:|---:|']
    for row in rows:
        ev, runtime = row.get('evaluation', {}), row.get('runtime', {})
        se3 = ev.get('se3', {}).get('trans', {}).get('rmse', float('nan'))
        sim3 = ev.get('sim3', {}).get('trans', {}).get('rmse', float('nan'))
        lines.append(f"| {row['name']} | {'1x' if row['realtime'] else 'fast'} | {row['trial']} | {row['valid']} | {se3:.6f} | {sim3:.6f} | {runtime.get('elapsed_seconds', 0):.2f} | {runtime.get('tracking_lost', '-')} |")
    lines += ['', '## 三次复跑', '']
    for name in dict.fromkeys(r['name'] for r in rows if r['realtime']):
        group = [r for r in rows if r['name'] == name and r['realtime'] and r['valid']]
        scores = [r['evaluation']['se3']['trans']['rmse'] for r in group]
        if scores:
            lines.append(f"- {name}: {len(scores)} 次有效，SE3 ATE 中位数 {np.median(scores):.6f} m，范围 {min(scores):.6f}–{max(scores):.6f} m。")
    groups = {name: [r['evaluation']['se3']['trans']['rmse'] for r in rows
                     if r['name'] == name and r['realtime'] and r['valid']]
              for name in ('baseline', 'best_schur', 'ceres', 'ceres_loop')}
    baseline, best = groups['baseline'], groups['best_schur']
    if len(baseline) == len(best) == 3:
        if max(best) < min(baseline):
            lines += ['', '最佳 Schur 配置三次均优于默认配置，改善超过本轮观察到的复跑范围。']
        elif min(best) > max(baseline):
            lines += ['', '最佳 Schur 配置的最低 ATE 仍高于默认配置的最高 ATE。']
        else:
            lines += ['', '最佳 Schur 与默认配置的复跑范围重叠，不能认定改善超过运行波动。']
    loop_runs = [r for r in rows if r['name'] == 'ceres_loop']
    if loop_runs:
        lines += ['', '## 回环实际生效记录', '']
        valid_loop_stats = []
        for row in loop_runs:
            item = row.get('loop')
            if not row['valid'] or not complete_loop_statistics(item):
                lines.append(f"- 第 {row['trial']} 次：试次无效或回环统计不完整。")
                continue
            valid_loop_stats.append(item)
            lines.append(f"- 第 {row['trial']} 次：检测 {item['detected']}，已应用校正 {item['corrections_applied']}，完成 PGO {item['pgo_completed']}。")
        if len(valid_loop_stats) != len(loop_runs):
            lines += ['', '有回环试次无效或缺少统计，无法给出全部试次的回环结论。']
        elif not any(item['detected'] for item in valid_loop_stats):
            lines += ['', '本序列未验证到回环收益：有效回环检测数为零。']
    (output / 'report.md').write_text('\n'.join(lines) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    selected = [r for r in rows if r['valid'] and (r['realtime'] and r['trial'] == 1)]
    if not selected:
        selected = [r for r in rows if r['valid']]
    for row in selected:
        directory = Path(row['directory']) / 'reference'
        if not (directory / 'associated_aligned.txt').exists():
            continue
        estimated = np.loadtxt(directory / 'associated_aligned.txt', ndmin=2)
        reference = np.loadtxt(DATA / 'output/GT/trajectory_gt_imu.txt', ndmin=2)
        indices = np.searchsorted(reference[:, 0], estimated[:, 0]).clip(1, len(reference)-1)
        indices -= (abs(reference[indices-1, 0]-estimated[:, 0]) < abs(reference[indices, 0]-estimated[:, 0]))
        axes[0].plot(estimated[:, 1], estimated[:, 2], label=row['name'])
        axes[1].plot(estimated[:, 0]-estimated[0, 0], np.linalg.norm(estimated[:, 1:4]-reference[indices, 1:4], axis=1), label=row['name'])
    reference = np.loadtxt(DATA / 'output/GT/trajectory_gt_imu.txt', ndmin=2)
    axes[0].plot(reference[:, 1], reference[:, 2], 'k--', label='Reference (VIO prior)')
    axes[0].set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
    axes[1].set(xlabel='time [s]', ylabel='position error [m]')
    for ax in axes:
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / 'comparison.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--phase', choices=['screen', 'confirm', 'all'], default='all')
    args = parser.parse_args()
    output = (args.output or ROOT / 'output' / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('vio_compare_%Y%m%d%H%M%S')).resolve()
    output.mkdir(parents=True, exist_ok=args.phase == 'confirm')
    reference = DATA / 'output/GT/trajectory_gt_imu.txt'
    base = yaml.safe_load((ROOT / 'svo_ros/param/vio_stereo.yaml').read_text())['/**']['ros__parameters']
    rows = []
    results = output / 'results.json'
    if results.exists():
        rows = json.loads(results.read_text())
    (output / 'source.diff').write_text(subprocess.check_output(['git', 'diff'], cwd=ROOT, text=True))
    (output / 'source_status.txt').write_text(subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True))
    (output / 'source_head.txt').write_text(subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True))

    def run(name, changes, realtime=False, build='default', trial=1):
        run_dir = output / f'{name}_{"realtime" if realtime else "fast"}_{trial}'
        run_dir.mkdir()
        params = dict(base, use_global_map=False, **changes)
        params_file = run_dir / 'params.yaml'
        params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        row = {'name': name, 'trial': trial, 'realtime': realtime, 'build': build,
               'changes': changes, 'directory': str(run_dir), 'valid': False}
        command = environment(build) + shlex.join(['ros2', 'launch', 'svo_ros', 'ego.launch.py',
            f'inbag:={DATA / "job_002.mcap"}', f'cameras:={DATA / "calib.yaml"}',
            f'params:={params_file}', f'output_dir:={run_dir}', 'save_trajectory:=true',
            f'realtime:={str(realtime).lower()}', 'rviz:=false'])
        (run_dir / 'command.sh').write_text(command + '\n')
        cache = ROOT / 'ros2_ws/build' / build / 'svo/CMakeCache.txt'
        if cache.exists():
            (run_dir / 'CMakeCache.txt').write_bytes(cache.read_bytes())
        print(f'Running {run_dir.name}', flush=True)
        started = time.monotonic()
        try:
            shell(command, run_dir / 'launch.log')
            summary = json.loads((run_dir / 'summary.json').read_text())
            row['runtime'] = summary
            checks = { 'complete': summary['complete'], 'copied': summary['copied_messages'] == 141530, 'images': summary['decoded_images'] == 4909,
                'imu': summary['imu_received'] == summary['imu_consumed'] == 65856,
                'calls': summary['processed'] == 4907, 'leading': summary['leading_skipped'] == 2,
                'tracking': summary['poses'] == 4907 and summary['tracking_lost'] == summary['resets'] == 0,
                'queues': summary['pending_images'] == summary['pending_imu'] == 0,
                'input': summary['overflow'] == summary['duplicates'] == summary['time_jumps'] == 0}
            row['checks'] = checks
            if params.get('runlc'):
                loop_summary = run_dir / 'loop_summary.json'
                if not loop_summary.is_file():
                    raise RuntimeError(f'Missing loop statistics: {loop_summary}')
                loop_stats = json.loads(loop_summary.read_text())
                if not complete_loop_statistics(loop_stats):
                    raise RuntimeError(f'Incomplete or invalid loop statistics: {loop_summary}')
                row['loop'] = loop_stats
            trajectory = run_dir / 'schurvins_job_002_trajectory.txt'
            poses = np.loadtxt(trajectory, ndmin=2)
            checks['finite_monotonic_trajectory'] = bool(poses.shape == (summary['poses'], 8) and np.isfinite(poses).all() and np.all(np.diff(poses[:, 0]) > 0))
            row['evaluation'] = evaluate(trajectory, run_dir / 'evaluation', reference, plots=trial == 1)
            if params.get('runlc') and params.get('global_map_type', 'BuiltInPoseGraph') == 'BuiltInPoseGraph':
                pgo_path = run_dir / 'pgo_trajectory.txt'
                if pgo_path.stat().st_size:
                    pgo = np.loadtxt(pgo_path, ndmin=2)
                    online = np.loadtxt(trajectory, ndmin=2)
                    pairs = [(i, int(np.argmin(abs(online[:, 0] - t)))) for i, t in enumerate(pgo[:, 0])]
                    pairs = [(i, j) for i, j in pairs if abs(pgo[i, 0] - online[j, 0]) < 1e-5]
                    if len(pairs) >= 3:
                        before = run_dir / 'pgo_before_common.txt'
                        after = run_dir / 'pgo_after_common.txt'
                        np.savetxt(before, online[[j for i, j in pairs]], fmt='%.17g')
                        np.savetxt(after, pgo[[i for i, j in pairs]], fmt='%.17g')
                        row['pgo_before'] = evaluate(before, run_dir / 'pgo_before_evaluation', reference)
                        row['pgo_after'] = evaluate(after, run_dir / 'pgo_after_evaluation', reference)
            # Independent fixed-time RPE, using the repository's existing evaluator.
            shell(environment(build) + shlex.join(['/usr/bin/python3', '-c',
                f'import sys; sys.path.insert(0, {str(ROOT / "svo_benchmarking/src")!r}); '
                'from svo_ceres_benchmarking.reference_report import report; '
                f'report({str(trajectory)!r}, {str(reference)!r}, {str(run_dir / "reference")!r}, tolerance=0.02)']),
                run_dir / 'reference.log')
            row['reference'] = json.loads((run_dir / 'reference/reference_summary.json').read_text())
            frames = list(csv.DictReader((run_dir / 'frames.csv').open()))
            checks['unique_frame_records'] = len(frames) == 4909 and len({f['timestamp_ns'] for f in frames}) == 4909
            row['valid'] = all(checks.values())
            durations = [float(f['processing_ms']) for f in frames if f['status'] == 'processed']
            row['processing_ms'] = dict(zip(['p50', 'p95', 'p99'], map(float, np.percentile(durations, [50, 95, 99]))))
        except Exception as error:
            row['valid'] = False
            row['error'] = str(error)
        row['wall_seconds_including_evaluation'] = time.monotonic() - started
        rows.append(row)
        results.write_text(json.dumps(rows, indent=2) + '\n')
        (run_dir / 'acceptance.json').write_text(json.dumps(row, indent=2) + '\n')
        print(json.dumps({k: row[k] for k in ('name', 'valid')}), flush=True)
        return row

    def score(row):
        return row.get('evaluation', {}).get('se3', {}).get('trans', {}).get('rmse', float('inf')) if row['valid'] else float('inf')

    if args.phase != 'confirm':
        for name, changes in SCREEN.items():
            run(name, changes)
        baseline = score(rows[0])
        combined = {}
        for group in [('window6', 'window8'), ('noise1', 'noise3'), ('features300', 'features450')]:
            candidate = min((r for r in rows if r['name'] in group), key=score)
            if score(candidate) < baseline:
                combined.update(candidate['changes'])
        run('combined', combined)
    if args.phase != 'screen':
        eligible = [r for r in rows if r['valid'] and not r['realtime']]
        if not eligible:
            raise RuntimeError('No valid screening result')
        best = min(eligible, key=score)
        (output / 'selected.json').write_text(json.dumps(best, indent=2))
        for name, changes, build in [
                ('ceres_loop', {'use_ceres_backend': True, 'runlc': True,
                 'voc_path': '/home/scl/.cache/schurvins/resources/vocabularies',
                 'global_map_type': 'BuiltInPoseGraph'}, 'vio-traditional'),
                ('ceres', {'use_ceres_backend': True}, 'vio-traditional'),
                ('baseline', {}, 'default'), ('best_schur', best['changes'], 'default')]:
            for trial in range(1, 4):
                run(name, changes, True, build, trial)
    write_report(output, rows)
    print(output, flush=True)


if __name__ == '__main__':
    main()
