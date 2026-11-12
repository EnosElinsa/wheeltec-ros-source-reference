"""Generate measured EuRoC calibration for RTAB-Map's VINS backend."""
from pathlib import Path
import cv2
import numpy as np


def imu_noise_policy(data):
    """Map continuous densities to the pinned VINS midpoint noise convention.

    Its two independent endpoint measurement columns each have weight dt/2;
    bias-driving columns have weight dt. Match leading per-step variance q*dt.
    This does not model correlations from reusing endpoints across steps.
    """
    imu = data.imu.calibration
    rate = float(imu['rate_hz'])
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError('IMU noise conversion requires a positive calibrated rate')
    intervals = [b - a for a, b in zip(data.imu.times, data.imu.times[1:])]
    expected_ns = 1e9 / rate
    # EuRoC's stored epoch timestamps have up to 256 ns quantization error.
    if not intervals or any(abs(dt - expected_ns) > 256 for dt in intervals):
        raise ValueError('IMU stamped intervals disagree with calibrated rate; fixed-rate noise conversion is unsupported')
    raw = {key: float(imu[key]) for key in ('accelerometer_noise_density',
        'gyroscope_noise_density', 'accelerometer_random_walk', 'gyroscope_random_walk')}
    if any(not np.isfinite(value) or value <= 0 for value in raw.values()):
        raise ValueError('IMU noise conversion requires positive finite continuous densities')
    derived = dict(acc_n=raw['accelerometer_noise_density'] * np.sqrt(2 * rate),
        gyr_n=raw['gyroscope_noise_density'] * np.sqrt(2 * rate),
        acc_w=raw['accelerometer_random_walk'] * np.sqrt(rate),
        gyr_w=raw['gyroscope_random_walk'] * np.sqrt(rate))
    return dict(policy='pinned_vins_midpoint_step_variance_v1',
        backend_revision='72023bc5d2abb14faa4c02a5c040d84f996b587a',
        backend_model='vins/src/factor/integration_base.h: independent endpoint noise, V measurement=dt/2, V bias=dt',
        calibrated_rate_hz=rate, stamped_interval_ns=dict(minimum=min(intervals), maximum=max(intervals), tolerance=256),
        continuous_densities=raw,
        continuous_units=dict(accelerometer_noise_density='m/s^2/sqrt(Hz)',
            gyroscope_noise_density='rad/s/sqrt(Hz)', accelerometer_random_walk='m/s^3/sqrt(Hz)',
            gyroscope_random_walk='rad/s^2/sqrt(Hz)'),
        formulas=dict(measurement='density * sqrt(2 * rate_hz)', bias='random_walk_density * sqrt(rate_hz)'),
        derived_vins_parameters=derived,
        derived_units=dict(acc_n='m/s^2', gyr_n='rad/s', acc_w='m/s^3', gyr_w='rad/s^2'),
        interpretation='Leading per-step covariance match to continuous density squared times dt; independent midpoint endpoints; not an exact correlated multi-step noise model or tuned accuracy guarantee')


def write_opencv(path, values):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise OSError(f'cannot create {path}')
    try:
        for key, value in values.items():
            if isinstance(value, dict):
                fs.startWriteStruct(key, cv2.FileNode_MAP)
                for k, v in value.items():
                    fs.write(k, v)
                fs.endWriteStruct()
            else:
                fs.write(key, value)
    finally:
        fs.release()


def generate_config(data, rect, output, mode):
    if mode not in ('stereo', 'vio'):
        raise ValueError('mode must be stereo or vio')
    noise = imu_noise_policy(data)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'solver_output').mkdir()
    for i, name in enumerate(('left', 'right')):
        p = rect.P[i]
        write_opencv(output / f'{name}.yaml', dict(model_type='PINHOLE', camera_name=name,
            image_width=rect.size[0], image_height=rect.size[1],
            distortion_parameters=dict(k1=0., k2=0., p1=0., p2=0.),
            projection_parameters=dict(fx=p[0, 0], fy=p[1, 1], cx=p[0, 2], cy=p[1, 2])))
    values = dict(imu=int(mode == 'vio'), num_of_cam=2, cam0_calib='left.yaml', cam1_calib='right.yaml',
                  image_width=rect.size[0], image_height=rect.size[1],
                  body_T_cam0=np.asarray(rect.body_T_rect[0]), body_T_cam1=np.asarray(rect.body_T_rect[1]),
                  estimate_extrinsic=0, multiple_thread=0, max_cnt=200, min_dist=25, show_track=0,
                  flow_back=1, max_solver_time=.04, max_num_iterations=8, keyframe_parallax=10.,
                  estimate_td=0, td=0., **noise['derived_vins_parameters'], g_norm=9.81,
                  output_path=(output / 'solver_output').as_posix())
    write_opencv(output / 'vins.yaml', values)
    return output / 'vins.yaml'
