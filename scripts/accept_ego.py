#!/usr/bin/env python3
"""Launch EGO, wait for DDS readiness, replay only standard inputs and assert completeness."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rosbag2_interfaces.srv import Resume


def stop(process):
    if process is None:
        return
    def signal_group(value):
        try:
            os.killpg(process.pid, value)
            return True
        except ProcessLookupError:
            return False
    # ros2 wrappers may exit while a stuck node remains in their process group.
    if not signal_group(signal.SIGINT):
        process.wait()
        return
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            signal_group(signal.SIGKILL)
            process.wait()
    # Never leave an orphaned estimator consuming CPU after a failed attempt.
    signal_group(signal.SIGKILL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--rate', type=float, default=0.25)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--images', type=int, default=4909)
    parser.add_argument('--imu', type=int, default=65856)
    parser.add_argument('--leading', type=int, default=2)
    parser.add_argument('--reference')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    qos_file = args.output / 'replay_qos.yaml'
    qos_file.write_text('/stereo/image_raw:\n  reliability: reliable\n  history: keep_last\n  depth: 64\n/imu:\n  reliability: reliable\n  history: keep_last\n  depth: 10000\n')
    rclpy.init()
    node = rclpy.create_node('schurvins_acceptance')
    status = {'ego': {}, 'svo': {}}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(String, f'/{key}/runtime_status',
                     lambda msg, key=key: status.__setitem__(key, json.loads(msg.data)), qos)
                     for key in status]
    control = node.create_publisher(String, '/svo/remote_key', 10)
    started = time.monotonic()
    launched = playback = None
    outcome = {'passed': False, 'rate': args.rate, 'start_offset_seconds': 0,
               'input': str(Path(args.bag).resolve()), 'calibration': str(Path(args.calibration).resolve())}
    launch_log = (args.output / 'launch.log').open('w')
    playback_log = (args.output / 'playback.log').open('w')
    def spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        if launched.poll() is not None:
            raise RuntimeError(f'Launch process exited: {launched.returncode}')
        if status['svo'].get('failed'):
            raise RuntimeError(f"Estimator reported failure: {status['svo']}")
        if status['ego'].get('image_queue_overflow', 0):
            raise RuntimeError(f"EGO decode queue overflow: {status['ego']}")
        if status['ego'].get('invalid_images', 0):
            raise RuntimeError(f"EGO image decoding failed: {status['ego']}")
        if time.monotonic() - started > args.timeout:
            raise TimeoutError(f'Acceptance timeout: {status}')
    try:
        launched = subprocess.Popen(['ros2', 'launch', 'svo_ros', 'ego.launch.py',
                     f'cameras:={Path(args.calibration).resolve()}',
                     f'output_dir:={args.output.resolve()}', 'save_trajectory:=true', 'rviz:=false'],
                     stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        topics = ['/stereo/image_raw', '/imu', '/ego/cam0/image_raw', '/ego/cam1/image_raw', '/ego/imu']
        while not (status['svo'] and status['ego'] and all(node.count_subscribers(t) >= 1 for t in topics)):
            spin()
        playback = subprocess.Popen(['ros2', 'bag', 'play', args.bag, '--rate', str(args.rate),
                     '--clock', '--start-offset', '0', '--start-paused', '--disable-keyboard-controls',
                     '--wait-for-all-acked', '30', '--qos-profile-overrides-path', str(qos_file),
                     '--topics', '/stereo/image_raw', '/imu'],
                     stdout=playback_log, stderr=subprocess.STDOUT, start_new_session=True)
        # Graph discovery alone does not mean DDS writers/readers have matched.
        # Keep the bag paused until both adapter inputs and all three outputs match.
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        matches = ['matched_image_publishers', 'matched_imu_publishers',
                   'matched_left_subscribers', 'matched_right_subscribers', 'matched_imu_subscribers']
        while not (resume.service_is_ready() and all(status['ego'].get(key, 0) > 0 for key in matches)):
            if playback.poll() is not None:
                raise RuntimeError(f'Playback exited before DDS readiness: {playback.returncode}')
            spin()
        resumed = resume.call_async(Resume.Request())
        while not resumed.done():
            spin()
        resumed.result()
        while playback.poll() is None:
            spin()
        if playback.returncode:
            raise RuntimeError(f'Playback failed: {playback.returncode}')
        drain_deadline = time.monotonic() + 120
        while not (status['ego'].get('raw_images') == args.images and
                   status['ego'].get('raw_imu') == args.imu and
                   status['ego'].get('decoded_images') == args.images and
                   status['ego'].get('pending_decode_images', 0) == 0 and
                   not status['ego'].get('decoder_inflight', False) and
                   status['svo'].get('image_groups') == args.images and
                   status['svo'].get('imu_received') == args.imu and
                   status['svo'].get('pending_images') == 0 and
                   not status['svo'].get('inflight', True)):
            if time.monotonic() > drain_deadline:
                raise RuntimeError(f'Inputs did not drain to expected counts: {status}')
            spin()
        while not status['svo'].get('stopped'):
            control.publish(String(data='q'))
            spin()
        run_dir = Path(status['svo']['run_dir'])
        summary = json.loads((run_dir / 'summary.json').read_text())
        expected = args.images - args.leading
        checks = {
            'raw_images': status['ego']['raw_images'] == args.images,
            'raw_imu': status['ego']['raw_imu'] == args.imu,
            'decoded_images': status['ego']['decoded_images'] == args.images,
            'invalid_images': status['ego']['invalid_images'] == 0,
            'decode_queue_drained': status['ego'].get('pending_decode_images', 0) == 0 and
                                    not status['ego'].get('decoder_inflight', False),
            'decode_queue_overflow': status['ego'].get('image_queue_overflow', 0) == 0,
            'published_imu': status['ego']['published_imu'] == args.imu,
            'image_messages': summary['image_messages'] == 2 * args.images,
            'paired_images': summary['image_groups'] == args.images,
            'leading_skipped': summary['leading_skipped'] == args.leading,
            'processed_once': summary['processed'] == expected,
            'imu_consumed': summary['imu_consumed'] == args.imu,
            'no_overflow': summary['overflow'] == 0,
            'no_duplicates': summary['duplicates'] == 0,
            'no_time_jump': summary['time_jumps'] == 0,
            'drained': summary['pending_images'] == summary['pending_imu'] == 0,
            'tracking_entered': summary['poses'] > 0,
        }
        with (run_dir / 'frames.csv').open() as stream:
            frames = list(csv.DictReader(stream))
        checks['per_frame_unique'] = len(frames) == args.images and len({f['timestamp_ns'] for f in frames}) == args.images
        checks['actual_estimator_calls'] = sum(f['status'] == 'processed' for f in frames) == expected
        # Flush the component's trajectory stream by orderly container shutdown.
        stop(launched)
        trajectory = np.atleast_2d(np.loadtxt(args.output / 'schurvins_live_trajectory.txt'))
        checks['finite_monotonic_trajectory'] = bool(trajectory.shape[1] == 8 and np.isfinite(trajectory).all() and np.all(np.diff(trajectory[:, 0]) > 0))
        summary['valid_trajectory_coverage'] = len(trajectory) / expected
        outcome.update(checks=checks, runtime=summary, adapter=status['ego'], run_dir=str(run_dir))
        outcome['passed'] = all(checks.values())
        if args.reference and len(trajectory) >= 3:
            from svo_ceres_benchmarking.reference_report import report
            outcome['reference'] = report(args.output / 'schurvins_live_trajectory.txt', args.reference, args.output / 'reference')
        if not outcome['passed']:
            raise RuntimeError(f'Failed integrity checks: {[key for key,value in checks.items() if not value]}')
    except BaseException as error:
        outcome['passed'] = False
        outcome['error'] = str(error)
        raise
    finally:
        outcome['total_wall_seconds'] = time.monotonic() - started
        outcome['last_status'] = status
        stop(playback)
        stop(launched)
        (args.output / 'acceptance.json').write_text(json.dumps(outcome, indent=2) + '\n')
        launch_log.close()
        playback_log.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(outcome, indent=2))


if __name__ == '__main__':
    main()
