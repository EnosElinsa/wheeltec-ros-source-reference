import json

import numpy as np
import pytest

from test_core import euroc


def test_schedule_uses_csv_times_imu_before_camera(euroc):
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.runtime import events, stamp_parts
    data = load_dataset(euroc)
    schedule = list(events(data))
    assert len(schedule) == 46
    assert [item[1] for item in schedule[:2]] == ['imu', 'stereo']
    assert [x[0] for x in schedule] == sorted(x[0] for x in schedule)
    t = data.cameras[0].times[0]
    seconds, nanos = stamp_parts(t)
    assert seconds * 1_000_000_000 + nanos == t


def test_prepare_run_records_identity_and_estimator_contract(euroc, tmp_path):
    from wheeltec_vision_benchmark.runtime import prepare_run, estimator_spec
    output = tmp_path / 'run'
    prepare_run(euroc, output, 'vio', True)
    manifest = json.loads((output / 'manifest.json').read_text())
    assert manifest['data']['camera_frames'] == 5
    assert manifest['data']['files']['cam0/data.csv']['sha256']
    assert manifest['ground_truth_published'] is False
    assert manifest['data']['ground_truth_reference']['sensor_yaml_text']
    assert 'IMU' in manifest['data']['ground_truth_reference']['interpretation']
    assert manifest['config_files']['vins.yaml']
    spec = estimator_spec(output / 'config/vins.yaml', 'vio')
    params = spec['parameters'][0]
    assert params['frame_id'] == 'base_footprint'
    assert params['publish_tf'] is False and params['Odom/Strategy'] == '9'
    assert params['publish_null_when_lost'] is False
    assert params['Reg/Force3DoF'] == 'false'
    assert params['qos'] == 1 and params['qos_imu'] == 1
    assert ('odom', '/benchmark/odom') in spec['remappings']
    assert ('odom_info', '/benchmark/odom_info') in spec['remappings']
    with pytest.raises(FileExistsError):
        prepare_run(euroc, output, 'vio', True)


def test_results_are_saved_even_when_no_output(euroc, tmp_path):
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.trajectory import save_evaluation
    data = load_dataset(euroc)
    out = tmp_path / 'evaluation'
    out.mkdir()
    score = save_evaluation(data, [], out)
    assert score['status'] == 'no_output'
    assert json.loads((out / 'metrics.json').read_text())['coverage'] == 0.
    assert (out / 'trajectory.png').stat().st_size > 1000


def test_rotation_quaternion_roundtrip_and_rpe_orientation():
    from wheeltec_vision_benchmark.geometry import rotation, quaternion
    from wheeltec_vision_benchmark.trajectory import Pose, evaluate
    q = np.array([.5, -.5, .5, .5])
    assert np.allclose(rotation(quaternion(rotation(q))), rotation(q))
    gt, est = [], []
    for i in range(30):
        gt.append(Pose(i * 100_000_000, np.array([i*.1, np.sin(i), .1*np.cos(i)]), np.array([0., 0., 0., 1.])))
        angle = i * .01
        est.append(Pose(i * 100_000_000, gt[-1].p, np.array([0., 0., np.sin(angle/2), np.cos(angle/2)])))
    score = evaluate(gt, est, [p.t for p in gt], interval_s=1.)
    assert score['rpe_rotation_rmse_rad'] == pytest.approx(.1)


def test_association_is_monotonic_when_nearest_candidates_conflict():
    from wheeltec_vision_benchmark.trajectory import Pose, associate
    def poses(times):
        return [Pose(t, np.zeros(3), np.array([0., 0., 0., 1.])) for t in times]
    pairs = associate(poses([0, 5]), poses([4, 6]), 10)
    assert all(b[1] > a[1] for a, b in zip(pairs, pairs[1:]))


def test_validation_and_scoring_entrypoints(euroc, tmp_path, capsys):
    from wheeltec_vision_benchmark.validate import main as validate
    from wheeltec_vision_benchmark.trajectory import main as score
    out = tmp_path / 'validation'
    assert validate(['--dataset', str(euroc), '--output', str(out)]) == 0
    report = json.loads((out / 'validation.json').read_text())
    summary = json.loads(capsys.readouterr().out)
    assert report['valid'] is True
    assert 'files' not in summary['data']
    assert summary['data']['sha256'] == report['data']['sha256']
    assert summary['file_count'] == len(report['data']['files'])
    assert summary['validation_json'] == str(out / 'validation.json')
    with pytest.raises(SystemExit) as exc:
        validate(['--dataset', str(euroc), '--output', str(out)])
    assert exc.value.code == 2
    trajectory = tmp_path / 'trajectory.csv'
    trajectory.write_text('timestamp_ns,px,py,pz,qx,qy,qz,qw\n')
    assert score(['--dataset', str(euroc), '--trajectory', str(trajectory),
                  '--output-dir', str(tmp_path / 'score')]) == 1
    assert 'no_output' in capsys.readouterr().out


def test_validator_without_output_reports_digest_without_creating_files(euroc, tmp_path, capsys, monkeypatch):
    from wheeltec_vision_benchmark.validate import main
    monkeypatch.chdir(tmp_path)
    assert main(['--dataset', str(euroc)]) == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary['validation_json'] is None
    assert summary['file_count'] == 18
    assert summary['data']['sha256']
    assert 'files' not in summary['data']
    assert list(tmp_path.iterdir()) == [euroc.parent]


def test_rank_one_alignment_fits_line_direction_before_orientation():
    from wheeltec_vision_benchmark.trajectory import Pose, evaluate
    q = np.array([0., 0., 0., 1.])
    gt = [Pose(i*1_000_000_000, np.array([float(i), 0., 0.]), q) for i in range(3)]
    est = [Pose(i*1_000_000_000, np.array([0., float(i), 0.]), q) for i in range(3)]
    score = evaluate(gt, est, [p.t for p in gt])
    assert score['alignment_position_rank'] == 1
    assert score['ate_translation_rmse_m'] < 1e-12
    r = np.array(score['alignment_rotation'])
    assert np.allclose(r @ [0., 1., 0.], [1., 0., 0.])
    assert np.linalg.det(r) == pytest.approx(1.)


def test_rank_one_alignment_uses_orientation_only_for_free_spin():
    from wheeltec_vision_benchmark.geometry import rotation
    from wheeltec_vision_benchmark.trajectory import Pose, alignment
    q = np.array([np.sin(.3), 0., 0., np.cos(.3)])
    gt = [Pose(i, np.array([float(i), 0., 0.]), q) for i in range(3)]
    est = [Pose(i, np.array([float(i), 0., 0.]), np.array([0., 0., 0., 1.])) for i in range(3)]
    r, t, rank = alignment(gt, est)
    assert rank == 1
    assert np.allclose(r, rotation(q))
    assert np.allclose(t, 0.)


@pytest.mark.parametrize('leading', [True, False])
def test_vio_skips_only_imu_unsupported_images_without_changing_raw_data(euroc, tmp_path, capsys, leading):
    from wheeltec_vision_benchmark.dataset import load_dataset
    from wheeltec_vision_benchmark.runtime import events, prepare_run, estimator_spec
    from wheeltec_vision_benchmark.trajectory import ground_truth, evaluate
    from wheeltec_vision_benchmark.validate import main as validate
    path = euroc / 'imu0/data.csv'
    rows = path.read_text().splitlines()
    # MH05-style 20 ms IMU start delay, or a symmetric unsupported final image.
    path.write_text('\n'.join([rows[0], *(rows[5:] if leading else rows[1:-4])]) + '\n')
    data = load_dataset(euroc)
    original_times = data.cameras[0].times.copy()
    original_identity = data.identity()
    vo = [t for t, kind, _ in events(data, mode='stereo') if kind == 'stereo']
    vio = [t for t, kind, _ in events(data, mode='vio') if kind == 'stereo']
    assert vo == original_times
    assert vio == (original_times[1:] if leading else original_times[:-1])
    assert [t for t, kind, _ in events(data, mode='vio') if kind == 'imu'] == data.imu.times
    assert data.cameras[0].times == original_times
    assert data.identity() == original_identity
    out = tmp_path / 'vio'
    prepare_run(euroc, out, 'vio', True)
    manifest = json.loads((out / 'manifest.json').read_text())
    policy = manifest['playback_policy']
    assert policy['expected_stereo_frames'] == 5
    assert policy['playable_stereo_frames'] == 4
    assert policy['excluded_stereo_frames'] == 1
    assert policy['excluded'][0]['timestamp_ns'] == original_times[0 if leading else -1]
    assert policy['excluded'][0]['reason'] == ('before_first_imu' if leading else 'after_last_imu')
    assert manifest['data']['files'] == original_identity['files']
    assert estimator_spec(out / 'config/vins.yaml', 'vio')['parameters'][0]['wait_imu_to_init'] is True
    assert estimator_spec(out / 'config/vins.yaml', 'stereo')['parameters'][0]['wait_imu_to_init'] is False
    estimates = [pose for pose in ground_truth(data) if pose.t in vio]
    score = evaluate(ground_truth(data), estimates, original_times)
    assert score['expected_frames'] == 5
    assert score['coverage'] == .8
    assert validate(['--dataset', str(euroc), '--mode', 'vio', '--output', str(tmp_path / 'validation')]) == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary['playback_policy']['excluded_stereo_frames'] == 1
    assert 'excluded' not in summary['playback_policy']
    assert summary['data']['camera_frames'] == 5
    assert validate(['--dataset', str(euroc), '--mode', 'stereo']) == 0
    vo_summary = json.loads(capsys.readouterr().out)
    assert vo_summary['playback_policy']['playable_stereo_frames'] == 5
    assert vo_summary['playback_policy']['excluded_stereo_frames'] == 0
