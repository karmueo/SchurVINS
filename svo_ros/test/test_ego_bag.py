"""Small installed-launch integration tests; no external dataset is required."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage, Imu
import yaml


def calibration(path):
    transform = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]
    raw = {'imu': {'accelerometer_bias': [1., 2., 3.], 'gyroscope_bias': [0., 0., 0.]},
           'gyroscope_noise_density': .004, 'accelerometer_noise_density': .08,
           'gyroscope_random_walk': .000002, 'accelerometer_random_walk': .00004}
    for name in ('cam0', 'cam1'):
        raw[name] = {'T_cam_imu': [row[:] for row in transform], 'resolution': [1920, 1080],
                     'intrinsics': [700., 700., 960., 540.], 'distortion_coeffs': [0., 0., 0., 0.]}
    raw['cam1']['T_cam_imu'][0][3] = -100.
    path.write_text(yaml.safe_dump(raw))
    return path


def make_bag(tmp_path, records, unknown=False):
    bag_dir = tmp_path / 'input'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = [('/stereo/image_raw', 'sensor_msgs/msg/CompressedImage'), ('/imu', 'sensor_msgs/msg/Imu')]
    if unknown:
        topics.append(('/custom', 'uninstalled_fixture_msgs/msg/Unknown'))
    for index, (topic, msg_type) in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(index, topic, msg_type, 'cdr'))
    for topic, data, stamp in records:
        writer.write(topic, data, stamp)
    del writer
    return next(bag_dir.glob('*.mcap'))


def messages(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    result = []
    while reader.has_next():
        result.append(reader.read_next())
    return result


def command(bag, cameras, output, **options):
    return ['ros2', 'launch', 'svo_ros', 'ego.launch.py', f'inbag:={bag}',
            f'cameras:={cameras}', f'output_dir:={output}', 'rviz:=false'] + [
                f'{key}:={str(value).lower()}' for key, value in options.items()]


def test_unknown_messages_and_no_text_survive_failed_eof(tmp_path):
    imu = Imu()
    imu.header.stamp.sec = 10
    bag = make_bag(tmp_path, [('/imu', serialize_message(imu), 10_000_000_000),
                              ('/custom', b'\x00\x01\x00\x00\x42\x00\x00\x00', 10_001_000_000)], True)
    cameras = calibration(tmp_path/'cameras.yaml')
    output = tmp_path/'output'
    completed = subprocess.run(command(bag, cameras, output), capture_output=True, timeout=30)
    assert completed.returncode != 0
    summary = json.loads((output/'summary.json').read_text())
    assert summary['complete'] is False and 'never entered tracking' in summary['error']
    assert messages(output/f'schurvins_{bag.stem}.mcap') == messages(bag)
    assert not list(output.glob('*.txt'))
    assert not list(output.glob('*.bag'))
    # A retry cannot truncate or overwrite the partial result.
    result_bag = output/f'schurvins_{bag.stem}.mcap'
    before = result_bag.read_bytes()
    retry = subprocess.run(command(bag, cameras, output), capture_output=True, timeout=30)
    assert retry.returncode != 0
    assert result_bag.read_bytes() == before


def test_missing_imu_coverage_at_eof_is_reported(tmp_path):
    image, imu = CompressedImage(), Imu()
    image.header.stamp.sec = 10
    image.format = 'jpeg'
    image.data = b'unused: final IMU coverage never arrives'
    imu.header.stamp.sec = 9
    bag = make_bag(tmp_path, [('/imu', serialize_message(imu), 9_000_000_000),
                              ('/stereo/image_raw', serialize_message(image), 10_000_000_000)])
    output = tmp_path/'output'
    completed = subprocess.run(command(bag, calibration(tmp_path/'cameras.yaml'), output),
                               capture_output=True, timeout=30)
    assert completed.returncode != 0
    summary = json.loads((output/'summary.json').read_text())
    assert summary['complete'] is False and 'coverage at EOF' in summary['error']
    assert summary['pending_images'] == 1 and summary['processed'] == 0
    assert messages(output/f'schurvins_{bag.stem}.mcap') == messages(bag)


def test_interrupt_closes_partial_mcap(tmp_path):
    imu = Imu()
    imu.header.stamp.sec = 10
    data = serialize_message(imu)
    bag = make_bag(tmp_path, [('/imu', data, 10_000_000_000), ('/imu', data, 70_000_000_000)])
    output = tmp_path/'output'
    process = subprocess.Popen(command(bag, calibration(tmp_path/'cameras.yaml'), output, realtime=True),
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.monotonic()+15
        while not list(output.glob('.*.bag/*.mcap')):
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(.02)
        time.sleep(.1)
        os.killpg(process.pid, signal.SIGINT)
        assert process.wait(timeout=20) != 0
        summary = json.loads((output/'summary.json').read_text())
        assert summary['complete'] is False and 'interrupted' in summary['error']
        assert len(messages(output/f'schurvins_{bag.stem}.mcap')) == 1
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def test_initialization_prior_waits_until_eof(tmp_path):
    before, after, image = Imu(), Imu(), CompressedImage()
    before.header.stamp.sec, before.header.stamp.nanosec = 9, 907_000_000
    after.header.stamp.sec, after.header.stamp.nanosec = 10, 107_000_000
    image.header.stamp.sec, image.format = 10, 'jpeg'
    image.data = b'unused: initialization prior remains unavailable'
    bag = make_bag(tmp_path, [('/imu', serialize_message(before), 9_900_000_000),
                              ('/stereo/image_raw', serialize_message(image), 10_000_000_000),
                              ('/imu', serialize_message(after), 10_100_000_000)])
    output = tmp_path/'output'
    completed = subprocess.run(command(bag, calibration(tmp_path/'cameras.yaml'), output),
                               capture_output=True, timeout=30)
    assert completed.returncode != 0
    summary = json.loads((output/'summary.json').read_text())
    assert summary['complete'] is False and 'prior unavailable at EOF' in summary['error']
    assert summary['imu_consumed'] == 2 and summary['processed'] == 0
    assert summary['pending_images'] == 1
