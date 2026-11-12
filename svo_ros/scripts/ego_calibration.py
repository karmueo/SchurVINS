#!/usr/bin/env python3
"""Convert read-only EGO calibration to SVO's metric camera-to-body convention."""
import argparse
from pathlib import Path
import numpy as np
import yaml


def convert(source, destination):
    with open(source, encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)
    cameras = []
    for name in ('cam0', 'cam1'):
        camera = raw[name]
        transform = np.asarray(camera['T_cam_imu'], dtype=float).reshape(4, 4)
        transform[:3, 3] *= 0.001
        body_camera = np.linalg.inv(transform)
        if not np.isfinite(body_camera).all():
            raise ValueError('Non-finite extrinsic')
        cameras.append({
            'camera': {
                'label': name, 'id': name, 'line-delay-nanoseconds': 0,
                'image_height': int(camera['resolution'][1]),
                'image_width': int(camera['resolution'][0]), 'type': 'pinhole',
                'intrinsics': {'cols': 1, 'rows': 4, 'data': camera['intrinsics']},
                'distortion': {'type': 'equidistant', 'parameters': {
                    'cols': 1, 'rows': 4, 'data': camera['distortion_coeffs']}},
            },
            'T_B_C': {'cols': 4, 'rows': 4, 'data': body_camera.ravel().tolist()},
        })
    result = {
        'label': 'EGO 800 Hz (fixed biases removed by ego_input)',
        'cameras': cameras,
        'imu_params': {
            'delay_imu_cam': 0.0, 'max_imu_delta_t': 0.01,
            'acc_max': 176.0, 'omega_max': 17.0,
            'sigma_omega_c': raw['gyroscope_noise_density'],
            'sigma_acc_c': raw['accelerometer_noise_density'],
            'sigma_omega_bias_c': raw['gyroscope_random_walk'],
            'sigma_acc_bias_c': raw['accelerometer_random_walk'],
            'sigma_integration': 0.0, 'g': 9.81007, 'imu_rate': 800.0,
        },
        'imu_initialization': {
            'velocity': [0.0]*3, 'omega_bias': [0.0]*3, 'acc_bias': [0.0]*3,
            'velocity_sigma': 2.0, 'omega_bias_sigma': 0.01, 'acc_bias_sigma': 0.1,
        },
    }
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(result, stream, sort_keys=False)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    arguments = parser.parse_args()
    convert(arguments.source, arguments.destination)
