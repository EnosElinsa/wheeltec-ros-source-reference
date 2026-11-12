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
    assert json.loads((out / 'validation.json').read_text())['valid'] is True
    with pytest.raises(SystemExit) as exc:
        validate(['--dataset', str(euroc), '--output', str(out)])
    assert exc.value.code == 2
    trajectory = tmp_path / 'trajectory.csv'
    trajectory.write_text('timestamp_ns,px,py,pz,qx,qy,qz,qw\n')
    assert score(['--dataset', str(euroc), '--trajectory', str(trajectory),
                  '--output-dir', str(tmp_path / 'score')]) == 1
    assert 'no_output' in capsys.readouterr().out
