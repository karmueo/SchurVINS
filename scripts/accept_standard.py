#!/usr/bin/env python3
"""Assert a complete standard-topic EuRoC replay through the standalone node."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosbag2_interfaces.srv import Resume
from std_msgs.msg import String


def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--pipeline', choices=['mono', 'stereo'], default='stereo')
    parser.add_argument('--no-imu', action='store_true')
    parser.add_argument('--ceres', action='store_true')
    parser.add_argument('--loop', action='store_true')
    parser.add_argument('--global-map', action='store_true')
    parser.add_argument('--vocabulary-dir')
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--timeout', type=float, default=900)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('svo_ros'))
    params = yaml.safe_load((share / 'param' / f'vio_{args.pipeline}.yaml').read_text())['/**']['ros__parameters']
    if args.global_map:
        params.update(yaml.safe_load((share / 'param/global_map.yaml').read_text())['/**']['ros__parameters'])
    params.update(pipeline=args.pipeline, use_imu=not args.no_imu, use_ceres_backend=args.ceres,
                  runlc=args.loop, use_global_map=args.global_map, use_sim_time=True,
                  calib_file=str(share / 'param/calib' / f'euroc_{args.pipeline}.yaml'),
                  imu_topic='/imu0', output_dir=str(args.output.resolve()))
    if args.vocabulary_dir:
        params['voc_path'] = args.vocabulary_dir
    params_file = args.output / 'parameters.yaml'
    params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    topics = ['/cam0/image_raw'] + (['/cam1/image_raw'] if args.pipeline == 'stereo' else [])
    cameras = len(topics)
    if not args.no_imu:
        topics += ['/imu0']
    metadata = rosbag2_py.Info().read_metadata(args.bag)
    counts = {entry.topic_metadata.name: entry.message_count for entry in metadata.topics_with_message_count}
    images = counts['/cam0/image_raw']
    if cameras == 2 and counts['/cam1/image_raw'] != images:
        raise ValueError('Stereo input counts differ')
    imu_count = 0 if args.no_imu else counts['/imu0']
    qos_file = args.output / 'replay_qos.yaml'
    qos_file.write_text(yaml.safe_dump({topic: {'reliability': 'reliable', 'history': 'keep_last',
        'depth': 10000 if topic == '/imu0' else 64} for topic in topics}))
    rclpy.init()
    node = rclpy.create_node('standard_acceptance')
    status = {}
    def receive(message):
        status.update(json.loads(message.data))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, '/svo/runtime_status', receive, qos)
    control = node.create_publisher(String, '/svo/remote_key', 10)
    result = {'passed': False, 'pipeline': args.pipeline, 'use_imu': not args.no_imu,
              'ceres': args.ceres, 'loop': args.loop, 'global_map': args.global_map,
              'bag': args.bag, 'rate': args.rate, 'start_offset_seconds': 0}
    started = time.monotonic()
    launch_log = (args.output / 'node.log').open('w')
    playback_log = (args.output / 'playback.log').open('w')
    process = playback = None
    def spin(expect_alive=True):
        rclpy.spin_once(node, timeout_sec=.1)
        if status.get('failed'):
            raise RuntimeError(f'Estimator failed: {status}')
        if expect_alive and process.poll() is not None:
            raise RuntimeError(f'Node exited before completion: {process.returncode}')
        if time.monotonic() - started > args.timeout:
            raise TimeoutError(str(status))
    try:
        process = subprocess.Popen(['ros2', 'run', 'svo_ros', 'svo_node', '--ros-args', '--params-file', str(params_file)],
            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        while not status:
            spin()
        playback = subprocess.Popen(['ros2', 'bag', 'play', args.bag, '--clock', '--start-paused',
            '--rate', str(args.rate), '--disable-keyboard-controls', '--wait-for-all-acked', '30',
            '--qos-profile-overrides-path', str(qos_file), '--topics', *topics],
            stdout=playback_log, stderr=subprocess.STDOUT, start_new_session=True)
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        while not (resume.service_is_ready() and status.get('matched_inputs') == len(topics)):
            spin()
        resumed = resume.call_async(Resume.Request())
        while not resumed.done():
            spin()
        resumed.result()
        while playback.poll() is None:
            spin()
        if playback.returncode:
            raise RuntimeError(f'Playback exited {playback.returncode}')
        while not (status.get('image_groups') == images and status.get('imu_received') == imu_count
                   and status.get('pending_images') == 0 and not status.get('inflight', True)):
            spin()
        control.publish(String(data='q'))
        while process.poll() is None:
            spin(False)
        if process.returncode:
            raise RuntimeError(f'Node shutdown returned {process.returncode}')
        run_dir = Path(status['run_dir'])
        summary = json.loads((run_dir / 'summary.json').read_text())
        trajectory = np.atleast_2d(np.loadtxt(run_dir / 'imu_traj.txt'))
        with (run_dir / 'frames.csv').open() as stream:
            frames = list(csv.DictReader(stream))
        checks = {
            'image_messages': summary['image_messages'] == cameras*images,
            'image_groups': summary['image_groups'] == images,
            'processed_and_leading': summary['processed'] + summary['leading_skipped'] == images,
            'imu': summary['imu_received'] == summary['imu_consumed'] == imu_count,
            'no_overflow': summary['overflow'] == 0,
            'no_duplicates': summary['duplicates'] == 0,
            'no_time_jump': summary['time_jumps'] == 0,
            'drained': summary['pending_images'] == summary['pending_imu'] == 0,
            'tracking': summary['poses'] > 0,
            'finite_monotonic': bool(trajectory.shape[1] == 8 and np.isfinite(trajectory).all() and np.all(np.diff(trajectory[:,0]) > 0)),
            'per_frame_unique': len(frames) == images and len({frame['timestamp_ns'] for frame in frames}) == images,
        }
        result.update(checks=checks, runtime=summary, run_dir=str(run_dir),
                      trajectory_coverage=len(trajectory)/images, passed=all(checks.values()))
        if not result['passed']:
            raise RuntimeError(f'Failed checks: {checks}')
    except BaseException as error:
        result.update(passed=False, error=str(error))
        raise
    finally:
        stop(playback); stop(process)
        result['wall_seconds'] = time.monotonic() - started
        result['last_status'] = status
        (args.output / 'acceptance.json').write_text(json.dumps(result, indent=2)+'\n')
        launch_log.close(); playback_log.close()
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
