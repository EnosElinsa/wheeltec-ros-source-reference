import csv
from pathlib import Path

import numpy as np
import pytest
import yaml


@pytest.fixture
def euroc(tmp_path):
    import cv2
    root = tmp_path / 'sequence' / 'mav0'
    start = 1403636579763555584  # exceeds exact float nanosecond precision
    for name in ('cam0', 'cam1', 'imu0', 'state_groundtruth_estimate0'):
        (root / name).mkdir(parents=True)
    transform = np.eye(4)
    for index in range(2):
        transform[0, 3] = index * .12
        sensor = dict(sensor_type='camera', rate_hz=20, camera_model='pinhole',
                      intrinsics=[100., 100., 32., 24.], resolution=[64, 48],
                      distortion_model='radial-tangential', distortion_coefficients=[.01, -.001, 0., 0.],
                      T_BS=dict(rows=4, cols=4, data=transform.ravel().tolist()))
        (root / f'cam{index}' / 'sensor.yaml').write_text(yaml.safe_dump(sensor))
        (root / f'cam{index}' / 'data').mkdir()
        with (root / f'cam{index}' / 'data.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['#timestamp [ns]', 'filename'])
            for k in range(5):
                t = start + k * 50_000_000
                w.writerow([t, f'{t}.png'])
                cv2.imwrite(str(root / f'cam{index}' / 'data' / f'{t}.png'), np.zeros((48, 64), np.uint8))
    imu = dict(sensor_type='imu', rate_hz=200, T_BS=dict(rows=4, cols=4, data=np.eye(4).ravel().tolist()),
               gyroscope_noise_density=.00017, gyroscope_random_walk=.000019,
               accelerometer_noise_density=.002, accelerometer_random_walk=.003)
    (root / 'imu0' / 'sensor.yaml').write_text(yaml.safe_dump(imu))
    with (root / 'imu0' / 'data.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]',
                    'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
        for k in range(41):
            w.writerow([start + k * 5_000_000, 0., 0., .1, 0., 0., 9.81])
    (root / 'state_groundtruth_estimate0' / 'sensor.yaml').write_text(yaml.safe_dump(
        dict(T_BS=dict(rows=4, cols=4, data=np.eye(4).ravel().tolist()))))
    with (root / 'state_groundtruth_estimate0' / 'data.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['#timestamp', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz'])
        for k in range(41):
            w.writerow([start + k * 5_000_000, k * .005, 0., 0., 1., 0., 0., 0.])
    return root


def test_dataset_preserves_integer_times_and_exact_pairs(euroc):
    from wheeltec_vision_benchmark.dataset import load_dataset
    data = load_dataset(euroc.parent)
    assert data.cameras[0].times[0] == 1403636579763555584
    assert data.cameras[0].times == data.cameras[1].times
    assert len(data.imu.times) == 41
    assert data.identity()['files']['imu0/data.csv']['sha256']
    csv_path = euroc / 'cam1/data.csv'
    csv_path.write_text(csv_path.read_text().replace('1403636579763555584', '1403636579763555585', 1))
    with pytest.raises(ValueError, match='exact stereo'):
        load_dataset(euroc)


@pytest.mark.parametrize('bad', ['1403636579763555584.0', '1e18', '-1'])
def test_rejects_noninteger_timestamps(euroc, bad):
    from wheeltec_vision_benchmark.dataset import load_dataset
    path = euroc / 'imu0/data.csv'
    path.write_text(path.read_text().replace('1403636579763555584', bad, 1))
    with pytest.raises(ValueError, match='timestamp'):
        load_dataset(euroc)


def test_rejects_duplicates_missing_images_and_wrong_units(euroc):
    from wheeltec_vision_benchmark.dataset import load_dataset
    path = euroc / 'imu0/data.csv'
    original = path.read_text()
    path.write_text(original.replace('1403636579768555584', '1403636579763555584'))
    with pytest.raises(ValueError, match='strictly increasing'):
        load_dataset(euroc)
    path.write_text(original.replace('[rad s^-1]', '[deg s^-1]'))
    with pytest.raises(ValueError, match='units'):
        load_dataset(euroc)
    path.write_text(original)
    next((euroc / 'cam0/data').glob('*.png')).unlink()
    with pytest.raises(ValueError, match='image'):
        load_dataset(euroc)


def test_rectification_geometry_and_body_extrinsics(euroc, tmp_path):
    import cv2
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.rectification import StereoRectifier
    from wheeltec_vision_benchmark.config import generate_config
    data = load_dataset(euroc)
    rect = StereoRectifier(data)
    world = np.array([[.1, .2, 3.], [.4, -.2, 4.], [-.2, .1, 2.]])
    pixels = []
    for i in range(2):
        p, _ = cv2.projectPoints(world - np.array([i * .12, 0., 0.]), np.zeros(3), np.zeros(3),
                                 data.cameras[i].K, data.cameras[i].D)
        pixels.append(cv2.undistortPoints(p, data.cameras[i].K, data.cameras[i].D,
                                         R=rect.R[i], P=rect.P[i]).reshape(-1, 2))
    assert np.allclose(pixels[0][:, 1], pixels[1][:, 1], atol=1e-7)
    assert np.all(pixels[0][:, 0] > pixels[1][:, 0])
    assert rect.P[1][0, 3] == pytest.approx(-rect.P[1][0, 0] * .12)
    assert np.allclose(rect.body_T_rect[1][:3, 3], [.12, 0., 0.])
    config = generate_config(data, rect, tmp_path / 'config', 'vio')
    fs = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    assert fs.getNode('imu').real() == 1
    assert fs.getNode('estimate_extrinsic').real() == 0
    assert np.allclose(fs.getNode('body_T_cam1').mat(), rect.body_T_rect[1])
    fs.release()
    with pytest.raises(FileExistsError):
        generate_config(data, rect, tmp_path / 'config', 'vio')


def test_se3_alignment_does_not_fit_scale_and_unique_association():
    from wheeltec_vision_benchmark.trajectory import Pose, evaluate, associate
    ts = [1403636579763555584 + i * 100_000_000 for i in range(30)]
    xyz = np.array([[i * .1, np.sin(i * .2), np.cos(i * .2)] for i in range(30)])
    def poses(points):
        return [Pose(t, np.array(p), np.array([0., 0., 0., 1.])) for t, p in zip(ts, points)]
    gt = poses(xyz)
    est = poses(xyz + [3., -2., 4.])
    score = evaluate(gt, est, ts, interval_s=.5)
    assert score['ate_translation_rmse_m'] < 1e-10
    assert score['rpe_translation_rmse_m'] < 1e-10
    assert score['coverage'] == 1.
    assert evaluate(gt, poses(xyz * 2.), ts)['ate_translation_rmse_m'] > .5
    close = [Pose(ts[0] + 1, np.zeros(3), np.array([0., 0., 0., 1.])),
             Pose(ts[0] + 2, np.zeros(3), np.array([0., 0., 0., 1.]))]
    assert len(associate(gt, close, 10_000_000)) == 1
    assert evaluate(gt, [], ts)['status'] == 'no_output'
    with pytest.raises(ValueError, match='strictly increasing'):
        evaluate(gt, [est[0], est[0]], ts)


def test_body_origin_rotation_and_gt_offset(euroc):
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.trajectory import ground_truth
    path = euroc / 'imu0/sensor.yaml'
    sensor = yaml.safe_load(path.read_text())
    transform = np.eye(4)
    transform[:3, 3] = [.3, .2, .1]
    sensor['T_BS']['data'] = transform.ravel().tolist()
    path.write_text(yaml.safe_dump(sensor))
    poses = ground_truth(load_dataset(euroc))
    assert np.allclose(poses[0].p, [.3, .2, .1])


def test_nontrivial_rectification_and_gt_sensor_coordinates(euroc):
    import cv2
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.geometry import pose_matrix, quaternion
    from wheeltec_vision_benchmark.rectification import StereoRectifier
    from wheeltec_vision_benchmark.trajectory import ground_truth
    body_T_imu = pose_matrix([.2, -.1, .3], np.array([0., 0., np.sin(.2), np.cos(.2)]))
    path = euroc / 'imu0/sensor.yaml'
    cfg = yaml.safe_load(path.read_text())
    cfg['T_BS']['data'] = body_T_imu.ravel().tolist()
    path.write_text(yaml.safe_dump(cfg))
    for i in range(2):
        angle = .04 if i else -.03
        imu_T_cam = pose_matrix([i * .12, 0., 0.], np.array([0., np.sin(angle/2), 0., np.cos(angle/2)]))
        path = euroc / f'cam{i}/sensor.yaml'
        cfg = yaml.safe_load(path.read_text())
        cfg['T_BS']['data'] = (body_T_imu @ imu_T_cam).ravel().tolist()
        path.write_text(yaml.safe_dump(cfg))
    path = euroc / 'state_groundtruth_estimate0/sensor.yaml'
    body_T_gt = pose_matrix([.5, .4, -.2], np.array([np.sin(.1), 0., 0., np.cos(.1)]))
    path.write_text(yaml.safe_dump(dict(T_BS=dict(rows=4, cols=4, data=body_T_gt.ravel().tolist()))))
    data = load_dataset(euroc)
    rect = StereoRectifier(data)
    points = np.array([[.1, .2, 3.], [-.4, .2, 2.], [.5, -.2, 4.]])
    rect_pixels = []
    for i, cam in enumerate(data.cameras):
        cam_T_body = np.linalg.inv(cam.body_T_sensor)
        camera_points = points @ cam_T_body[:3, :3].T + cam_T_body[:3, 3]
        pixels, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), cam.K, cam.D)
        rect_pixels.append(cv2.undistortPoints(pixels, cam.K, cam.D, R=rect.R[i], P=rect.P[i]).reshape(-1, 2))
        assert np.allclose(rect.body_T_rect[i][:3, :3], cam.body_T_sensor[:3, :3] @ rect.R[i].T)
    assert np.allclose(rect_pixels[0][:, 1], rect_pixels[1][:, 1], atol=1e-6)
    expected = np.linalg.inv(body_T_gt) @ body_T_imu
    assert np.allclose(ground_truth(data)[0].matrix(), expected)


def test_rejects_bad_calibration_and_unreadable_images(euroc):
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.rectification import StereoRectifier
    path = euroc / 'cam0/sensor.yaml'
    cfg = yaml.safe_load(path.read_text())
    cfg['T_BS']['data'][0] = 2.
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match='SE\\(3\\)'):
        load_dataset(euroc)
    cfg['T_BS']['data'][0] = 1.
    path.write_text(yaml.safe_dump(cfg))
    data = load_dataset(euroc)
    rect = StereoRectifier(data)
    data.cameras[0].paths[0].write_bytes(b'invalid image')
    with pytest.raises(ValueError, match='decoded'):
        rect.image(data.cameras[0].paths[0], 0)
