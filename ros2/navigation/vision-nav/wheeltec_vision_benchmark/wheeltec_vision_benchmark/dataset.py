"""Strict EuRoC loader. Never convert a timestamp to floating point."""
import csv
from dataclasses import dataclass
import hashlib
from pathlib import Path
import re

import numpy as np
import yaml
from .geometry import rigid, rotation


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def timestamp(text):
    if not re.fullmatch(r'[0-9]+', str(text).strip()):
        raise ValueError(f'timestamp must be an integer nanosecond value: {text!r}')
    t = int(text)
    if t // 1_000_000_000 > 2**31 - 1:
        raise ValueError('timestamp exceeds ROS signed seconds range')
    return t


def read_csv(path, columns):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.reader(stream, skipinitialspace=True))
    header = rows.pop(0) if rows and rows[0][0].startswith('#') else []
    times, values = [], []
    for row in rows:
        if not row or row[0].startswith('#'):
            continue
        if len(row) < columns:
            raise ValueError(f'{path}: expected at least {columns} columns')
        t = timestamp(row[0])
        if times and t <= times[-1]:
            raise ValueError(f'{path}: timestamps must be strictly increasing')
        times.append(t)
        values.append(row[1:])
    if not times:
        raise ValueError(f'{path}: empty data')
    return header, times, values


def sensor(path):
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(value, dict):
        raise ValueError(f'{path}: sensor YAML must be a mapping')
    matrix = value.get('T_BS', {})
    if matrix.get('rows') != 4 or matrix.get('cols') != 4:
        raise ValueError(f'{path}: T_BS requires a 4x4 extrinsic')
    value['transform'] = rigid(matrix.get('data', []))
    return value


@dataclass
class Camera:
    times: list
    paths: list
    K: np.ndarray
    D: np.ndarray
    size: tuple
    body_T_sensor: np.ndarray
    calibration: dict


@dataclass
class Imu:
    times: list
    values: np.ndarray
    calibration: dict


@dataclass
class Dataset:
    root: Path
    cameras: list
    imu: Imu
    gt_times: list
    gt_values: np.ndarray
    gt_sensor: dict
    files: list

    def identity(self):
        files = {p.relative_to(self.root).as_posix(): dict(size_bytes=p.stat().st_size, sha256=digest(p))
                 for p in sorted(self.files)}
        encoded = ''.join(f'{k}:{v["sha256"]}\n' for k, v in files.items()).encode()
        return dict(mav0=str(self.root), sha256=hashlib.sha256(encoded).hexdigest(), files=files,
                    camera_frames=len(self.cameras[0].times), imu_samples=len(self.imu.times),
                    gt_samples=len(self.gt_times), imu_units='rad/s,m/s^2', body_origin='imu0',
                    ground_truth_reference=dict(
                        sensor_yaml_text=(self.root / 'state_groundtruth_estimate0/sensor.yaml').read_text(encoding='utf-8-sig'),
                        interpretation='EuRoC supplied reference; MH reference is a Leica position + IMU batch solution with time-offset estimation; orientation is observed only by IMU, not independent motion-capture orientation'))


def load_dataset(path):
    root = Path(path).expanduser().resolve()
    if (root / 'mav0').is_dir():
        root /= 'mav0'
    files = [root / name / suffix for name in ('cam0', 'cam1', 'imu0', 'state_groundtruth_estimate0')
             for suffix in ('data.csv', 'sensor.yaml')]
    if any(not p.is_file() for p in files):
        raise ValueError('EuRoC requires cam0/cam1/imu0/state_groundtruth_estimate0 CSV and sensor.yaml')
    imu_cfg = sensor(root / 'imu0/sensor.yaml')
    if imu_cfg.get('sensor_type') != 'imu' or float(imu_cfg.get('rate_hz', 0)) != 200.:
        raise ValueError('EuRoC IMU requires sensor_type imu, rate_hz 200')
    for key in ('gyroscope_noise_density', 'gyroscope_random_walk',
                'accelerometer_noise_density', 'accelerometer_random_walk'):
        if not np.isfinite(float(imu_cfg.get(key, np.nan))) or float(imu_cfg[key]) <= 0:
            raise ValueError(f'IMU requires positive SI noise parameter {key}')
    header, imu_times, imu_rows = read_csv(root / 'imu0/data.csv', 7)
    fields = [''.join(field.split()) for field in header[1:7]]
    if fields != ['w_RS_S_x[rads^-1]', 'w_RS_S_y[rads^-1]', 'w_RS_S_z[rads^-1]',
                  'a_RS_S_x[ms^-2]', 'a_RS_S_y[ms^-2]', 'a_RS_S_z[ms^-2]']:
        raise ValueError('IMU CSV units must be EuRoC rad s^-1 and m s^-2 in gyro/accel order')
    imu_values = np.array([row[:6] for row in imu_rows], dtype=float)
    if not np.isfinite(imu_values).all():
        raise ValueError('IMU samples must be finite SI values')
    cameras = []
    for side in ('cam0', 'cam1'):
        cfg = sensor(root / side / 'sensor.yaml')
        if (cfg.get('sensor_type') != 'camera' or cfg.get('camera_model') != 'pinhole'
                or cfg.get('distortion_model') != 'radial-tangential' or float(cfg.get('rate_hz', 0)) != 20.):
            raise ValueError('EuRoC camera requires 20 Hz pinhole radial-tangential calibration')
        intrinsics = np.asarray(cfg.get('intrinsics'), dtype=float)
        d = np.asarray(cfg.get('distortion_coefficients'), dtype=float)
        size = cfg.get('resolution', [])
        if (intrinsics.shape != (4,) or d.shape != (4,) or not np.isfinite(intrinsics).all()
                or not np.isfinite(d).all() or np.any(intrinsics[:2] <= 0) or len(size) != 2
                or any(type(v) is not int or v <= 0 for v in size)):
            raise ValueError('invalid camera intrinsics/distortion/resolution')
        fx, fy, cx, cy = intrinsics
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        _, times, rows = read_csv(root / side / 'data.csv', 2)
        paths = []
        directory = (root / side / 'data').resolve()
        for row in rows:
            image = (directory / row[0]).resolve()
            if not image.is_relative_to(directory) or not image.is_file():
                raise ValueError(f'image must exist within {directory}: {row[0]}')
            paths.append(image)
        files.extend(paths)
        cameras.append(Camera(times, paths, k, d, tuple(size),
                              np.linalg.inv(imu_cfg['transform']) @ cfg['transform'], cfg))
    if cameras[0].times != cameras[1].times:
        raise ValueError('exact stereo timestamp pairing required')
    if cameras[0].size != cameras[1].size:
        raise ValueError('stereo resolutions must match')
    if np.linalg.norm(cameras[0].body_T_sensor[:3, 3] - cameras[1].body_T_sensor[:3, 3]) < 1e-5:
        raise ValueError('stereo baseline must be nonzero')
    _, gt_times, gt_rows = read_csv(root / 'state_groundtruth_estimate0/data.csv', 8)
    gt_values = np.array([row[:7] for row in gt_rows], dtype=float)
    if not np.isfinite(gt_values).all():
        raise ValueError('ground truth must be finite')
    for row in gt_values:
        rotation(row[[4, 5, 6, 3]])
    if gt_times[-1] < cameras[0].times[0] or gt_times[0] > cameras[0].times[-1]:
        raise ValueError('ground truth and cameras must overlap')
    gt_cfg = sensor(root / 'state_groundtruth_estimate0/sensor.yaml')
    return Dataset(root, cameras, Imu(imu_times, imu_values, imu_cfg), gt_times, gt_values, gt_cfg, files)
