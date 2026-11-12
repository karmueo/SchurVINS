#!/usr/bin/env python3
"""Run one-command EGO MCAP processing and verify inputs, results and reference metrics."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import struct
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path as PathMessage
from sensor_msgs.msg import CompressedImage


def reader(path):
    stream = rosbag2_py.SequentialReader()
    stream.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return stream


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def pose_row(stamp, pose):
    p, q = pose.position, pose.orientation
    return [stamp * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def verify(source, output, images=4909, imu=65856, leading=2, save_trajectory=True):
    source, output = Path(source), Path(output)
    result = output / f'schurvins_{source.stem}.mcap'
    summary = json.loads((output / 'summary.json').read_text())
    digests = [defaultdict(hashlib.sha256), defaultdict(hashlib.sha256)]
    counts = [Counter(), Counter()]
    types = []
    image_stamps = {}
    odom_rows, path_rows = [], []
    output_topics = {'/svo/odometry', '/svo/path_imu'}
    paths_consistent = True
    path_count = 0
    path_pose_dtype = np.dtype({'names': ['stamp', 'frame', 'pose'],
                               'formats': ['V8', 'V10', 'V56'],
                               'offsets': [0, 8, 24], 'itemsize': 80})
    previous_path_poses = np.empty(0, dtype=path_pose_dtype)
    last_path_data = None
    frames = list(csv.DictReader((output / 'frames.csv').open()))
    for index, bag in enumerate((source, result)):
        stream = reader(bag)
        types.append({t.name: (t.type, t.serialization_format, t.type_description_hash)
                      for t in stream.get_all_topics_and_types()})
        while stream.has_next():
            topic, data, record_stamp = stream.read_next()
            if index == 1 and topic in output_topics:
                if topic == '/svo/odometry':
                    message = deserialize_message(data, Odometry)
                    stamp = stamp_ns(message.header.stamp)
                    paths_consistent &= image_stamps.get(stamp) == record_stamp
                    paths_consistent &= message.header.frame_id == 'world' and message.child_frame_id == 'imu'
                    odom_rows.append(pose_row(stamp, message.pose.pose))
                else:
                    # Check growing Path prefixes in CDR bytes. Deserializing all
                    # cumulative paths into Python objects would be quadratic.
                    endian = '<' if data[1] & 1 else '>'
                    sec, nsec, frame_length = struct.unpack_from(endian+'iII', data, 4)
                    frame_end = 16 + frame_length
                    sequence_offset = 4 + ((frame_end-4+3)//4)*4
                    count, = struct.unpack_from(endian+'I', data, sequence_offset)
                    payload_offset = sequence_offset+4
                    path_count += 1
                    paths_consistent &= count == path_count and data[16:frame_end] == b'world\0'
                    paths_consistent &= image_stamps.get(sec*1000000000+nsec) == record_stamp
                    # Our PoseStamped headers use the fixed frame "world".
                    # Compare semantic fields only: CDR alignment padding is unspecified.
                    current_path_poses = np.frombuffer(data, dtype=path_pose_dtype, count=count, offset=payload_offset)
                    paths_consistent &= np.array_equal(current_path_poses[:len(previous_path_poses)], previous_path_poses)
                    previous_path_poses = current_path_poses
                    last_path_data = data
                continue
            counts[index][topic] += 1
            digests[index][topic].update(record_stamp.to_bytes(8, 'little', signed=True))
            digests[index][topic].update(len(data).to_bytes(8, 'little'))
            digests[index][topic].update(data)
            if index == 0 and topic == '/stereo/image_raw':
                image = deserialize_message(data, CompressedImage)
                image_stamps[stamp_ns(image.header.stamp)] = record_stamp
    if last_path_data is not None:
        last_path = deserialize_message(last_path_data, PathMessage)
        path_rows = [pose_row(stamp_ns(pose.header.stamp), pose.pose) for pose in last_path.poses]
        paths_consistent &= all(pose.header.frame_id == 'world' for pose in last_path.poses)
    odometry = np.asarray(odom_rows).reshape(-1, 8)
    paths = np.asarray(path_rows).reshape(-1, 8)
    expected = images-leading
    checks = {
        'complete': summary['complete'],
        'original_counts': counts[0] == counts[1],
        'original_payloads_and_record_stamps': {k: v.digest() for k, v in digests[0].items()} ==
                                              {k: v.digest() for k, v in digests[1].items()},
        'original_topic_definitions': all(types[1].get(k) == v for k, v in types[0].items()),
        'output_topic_types': types[1].get('/svo/odometry', ('',))[0] == 'nav_msgs/msg/Odometry' and
                              types[1].get('/svo/path_imu', ('',))[0] == 'nav_msgs/msg/Path',
        'raw_images': counts[0]['/stereo/image_raw'] == summary['decoded_images'] == images,
        'raw_imu': counts[0]['/imu'] == summary['imu_received'] == summary['imu_consumed'] == imu,
        'image_groups': summary['image_groups'] == images and summary['image_messages'] == 2*images,
        'processed_once': summary['processed'] == expected,
        'leading_skipped': summary['leading_skipped'] == leading,
        'no_overflow_duplicate_or_rollback': summary['overflow'] == summary['duplicates'] == summary['time_jumps'] == 0,
        'queues_drained': summary['pending_images'] == summary['pending_imu'] == 0,
        'per_frame_unique': len(frames) == images and len({f['timestamp_ns'] for f in frames}) == images,
        'actual_estimator_calls': sum(f['status'] == 'processed' for f in frames) == expected,
        'finite_monotonic_trajectory': bool(len(odometry) and np.isfinite(odometry).all() and
                                           np.all(np.diff(odometry[:, 0]) > 0)),
        'all_tracking_poses_recorded': len(odometry) == len(paths) == summary['poses'] and summary['poses'] > 0,
        'path_odom_consistent': paths_consistent and odometry.shape == paths.shape and np.array_equal(odometry, paths),
        'fps_formula': summary['compute_seconds'] > 0 and
                       np.isclose(summary['average_fps'], summary['processed']/summary['compute_seconds']),
    }
    trajectory = output / f'schurvins_{source.stem}_trajectory.txt'
    checks['trajectory_switch'] = trajectory.is_file() == save_trajectory
    if save_trajectory and trajectory.is_file():
        tum = np.loadtxt(trajectory, ndmin=2)
        checks['tum_matches_odometry'] = tum.shape == odometry.shape and np.allclose(tum, odometry, rtol=0, atol=1e-6)
    checks = {key: bool(value) for key, value in checks.items()}
    return {'passed': all(checks.values()), 'checks': checks, 'runtime': summary}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--realtime', action='store_true')
    parser.add_argument('--no-save-trajectory', action='store_true')
    parser.add_argument('--images', type=int, default=4909)
    parser.add_argument('--imu', type=int, default=65856)
    parser.add_argument('--leading', type=int, default=2)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    outcome = {'passed': False}
    try:
        with (args.output / 'launch.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'svo_ros', 'ego.launch.py', f'inbag:={args.bag.resolve()}',
                f'cameras:={args.calibration.resolve()}', f'output_dir:={args.output.resolve()}',
                f'realtime:={str(args.realtime).lower()}',
                f'save_trajectory:={str(not args.no_save_trajectory).lower()}', 'rviz:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=args.timeout)
            except BaseException:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise
        if code:
            raise RuntimeError(f'Launch failed with exit code {code}; inspect launch.log and summary.json')
        outcome = verify(args.bag, args.output, args.images, args.imu, args.leading, not args.no_save_trajectory)
        if args.reference:
            if args.no_save_trajectory:
                raise ValueError('--reference requires trajectory saving')
            from svo_ceres_benchmarking.reference_report import report
            trajectory = args.output / f'schurvins_{args.bag.stem}_trajectory.txt'
            outcome['reference'] = report(trajectory, args.reference, args.output / 'reference')
    except BaseException as error:
        outcome.update(passed=False, error=str(error))
        raise
    finally:
        outcome['total_wall_seconds'] = time.monotonic()-started
        (args.output / 'acceptance.json').write_text(json.dumps(outcome, indent=2)+'\n')
    print(json.dumps(outcome, indent=2))
    if not outcome['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
