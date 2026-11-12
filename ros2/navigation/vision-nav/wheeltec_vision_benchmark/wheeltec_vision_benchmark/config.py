"""Generate measured EuRoC calibration for RTAB-Map's VINS backend."""
from pathlib import Path
import cv2
import numpy as np


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
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'solver_output').mkdir()
    for i, name in enumerate(('left', 'right')):
        p = rect.P[i]
        write_opencv(output / f'{name}.yaml', dict(model_type='PINHOLE', camera_name=name,
            image_width=rect.size[0], image_height=rect.size[1],
            distortion_parameters=dict(k1=0., k2=0., p1=0., p2=0.),
            projection_parameters=dict(fx=p[0, 0], fy=p[1, 1], cx=p[0, 2], cy=p[1, 2])))
    imu = data.imu.calibration
    values = dict(imu=int(mode == 'vio'), num_of_cam=2, cam0_calib='left.yaml', cam1_calib='right.yaml',
                  image_width=rect.size[0], image_height=rect.size[1],
                  body_T_cam0=np.asarray(rect.body_T_rect[0]), body_T_cam1=np.asarray(rect.body_T_rect[1]),
                  estimate_extrinsic=0, multiple_thread=0, max_cnt=200, min_dist=25, show_track=0,
                  flow_back=1, max_solver_time=.04, max_num_iterations=8, keyframe_parallax=10.,
                  estimate_td=0, td=0., acc_n=float(imu['accelerometer_noise_density']),
                  gyr_n=float(imu['gyroscope_noise_density']), acc_w=float(imu['accelerometer_random_walk']),
                  gyr_w=float(imu['gyroscope_random_walk']), g_norm=9.81,
                  output_path=(output / 'solver_output').as_posix())
    write_opencv(output / 'vins.yaml', values)
    return output / 'vins.yaml'
