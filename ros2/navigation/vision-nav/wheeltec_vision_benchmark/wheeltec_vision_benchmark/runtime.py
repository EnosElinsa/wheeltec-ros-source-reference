"""ROS-free launch, manifest and replay contracts."""
import heapq
import json
from pathlib import Path
import platform

import cv2
import numpy as np
from .config import generate_config
from .dataset import digest, load_dataset
from .rectification import StereoRectifier

FRAME = 'base_footprint'
CAMERA_FRAMES = ('benchmark_left_rect', 'benchmark_right_rect')


def stamp_parts(t):
    return divmod(t, 1_000_000_000)


def events(data):
    imu = ((t, 0, i) for i, t in enumerate(data.imu.times))
    stereo = ((t, 1, i) for i, t in enumerate(data.cameras[0].times))
    for t, kind, i in heapq.merge(imu, stereo):
        yield t, 'imu' if kind == 0 else 'stereo', i


def estimator_spec(config, mode):
    params = dict(use_sim_time=True, frame_id=FRAME, odom_frame_id='benchmark_odom', publish_tf=False,
                  publish_null_when_lost=False,
                  approx_sync=False, qos=1, qos_imu=1, queue_size=400, sync_queue_size=60,
                  output_queue_size=60, imu_queue_size=400,
                  wait_for_transform=.5, wait_imu_to_init=mode == 'vio',
                  **{'Odom/Strategy': '9', 'Reg/Force3DoF': 'false', 'OdomVINSFusion/ConfigPath': str(config),
                     'Rtabmap/ImagesAlreadyRectified': 'true', 'Odom/ResetCountdown': '0'})
    return dict(package='rtabmap_odom', executable='stereo_odometry', name='benchmark_vins', output='screen',
                parameters=[params], remappings=[
                    ('left/image_rect', '/benchmark/left/image_rect'),
                    ('right/image_rect', '/benchmark/right/image_rect'),
                    ('left/camera_info', '/benchmark/left/camera_info'),
                    ('right/camera_info', '/benchmark/right/camera_info'),
                    ('imu', '/benchmark/imu' if mode == 'vio' else '/benchmark/unused_imu'),
                    ('odom', '/benchmark/odom'), ('odom_info', '/benchmark/odom_info'),
                    ('odom_info_lite', '/benchmark/odom_info_lite')])


def prepare_run(dataset, output, mode, run_estimator):
    data = load_dataset(dataset)
    rect = StereoRectifier(data)
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = generate_config(data, rect, output / 'config', mode)
    identity = data.identity()
    manifest = dict(schema_version=1, data=identity, mode=mode, run_estimator=run_estimator,
                    timestamp_policy='original integer nanoseconds; 1x CSV time; no resampling',
                    frame_id=FRAME, body_origin='imu0, full 6DoF; not planar chassis',
                    ground_truth_published=False, imu_orientation='unavailable; covariance[0]=-1',
                    estimator_covariance='RTAB-Map VINS heuristic (0.0001/9999), recorded verbatim; not calibrated uncertainty',
                    config_files={p.name: digest(p) for p in sorted((output / 'config').glob('*.yaml'))},
                    versions=dict(python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__),
                    estimator=estimator_spec(config, mode) if run_estimator else None,
                    source_files={p.name: digest(p) for p in sorted(Path(__file__).parent.glob('*.py'))})
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return data, rect, config
