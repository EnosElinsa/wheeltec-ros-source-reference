"""Behavioral gates for sensor evidence; synthetic fixtures are not hardware proof."""

import copy
import hashlib
import importlib
import importlib.util
import json
import sys
from types import SimpleNamespace

import pytest


LEFT = '/camera/infra1/image_rect_raw'
RIGHT = '/camera/infra2/image_rect_raw'
LEFT_INFO = '/camera/infra1/camera_info'
RIGHT_INFO = '/camera/infra2/camera_info'
IMU = '/imu/data_raw'
EPOCH = 1_800_000_000_000_000_000


def logic():
    assert importlib.util.find_spec('wheeltec_vision_nav.vins_audit_logic'), 'pure audit is missing'
    return importlib.import_module('wheeltec_vision_nav.vins_audit_logic')


def cli():
    assert importlib.util.find_spec('wheeltec_vision_nav.vins_audit'), 'bag audit CLI is missing'
    return importlib.import_module('wheeltec_vision_nav.vins_audit')


def calibration():
    identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    second = copy.deepcopy(identity)
    second[0][3] = 0.05
    return dict(imu=1, estimate_extrinsic=0, body_T_cam0=identity,
                body_T_cam1=second, acc_n=0.1, gyr_n=0.01, acc_w=0.001,
                gyr_w=0.0001, g_norm=9.81)


def samples(imu_period_ns=5_000_000):
    result = {name: [] for name in (LEFT, RIGHT, LEFT_INFO, RIGHT_INFO, IMU)}
    for i in range(31):
        stamp = EPOCH + i * 33_333_333
        for image, info, frame, tx in ((LEFT, LEFT_INFO, 'left_optical', 0),
                                       (RIGHT, RIGHT_INFO, 'right_optical', -20)):
            common = dict(stamp_ns=stamp, arrival_ns=stamp + 2_000_000,
                          frame_id=frame, width=640, height=480)
            result[image].append(dict(common))
            result[info].append(dict(common, p=[400, 0, 320, tx, 0, 400, 240, 0, 0, 0, 1, 0]))
    for i in range(1_000_000_000 // imu_period_ns + 1):
        stamp = EPOCH + i * imu_period_ns
        result[IMU].append(dict(stamp_ns=stamp, arrival_ns=stamp + 1_000_000,
                                frame_id='imu_link', accel=[0, 0, 9.81], gyro=[0, 0, 0]))
    return result


def audit(data=None, **kwargs):
    return logic().audit_samples(samples() if data is None else data, **kwargs)


def check(result, name):
    return next(item for item in result['checks'] if item['name'] == name)


def test_exact_stereo_headers_and_complete_sensor_fixture_pass():
    result = audit(imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['schema_version'] == 1
    assert result['kind'] == 'vins_sensor_audit'
    assert result['stereo_ready'] and result['vio_ready']
    assert result['stereo']['matched_timestamps'] == 31
    assert result['matched_stereo_frames'] == 31
    assert result['camera_models']['right']['p'][3] == -20
    assert result['topics'][LEFT]['rate_hz'] == pytest.approx(30)
    assert result['topics'][IMU]['rate_hz'] == pytest.approx(200)
    assert result['topics'][IMU]['accel']['mean'] == pytest.approx([0, 0, 9.81])
    assert result['topics'][IMU]['gyro']['stddev'] == [0, 0, 0]
    assert result['topics'][IMU]['accel_norm']['mean'] == pytest.approx(9.81)
    assert result['stationary'] is True


@pytest.mark.parametrize('offset', [1, 33_333_333])
def test_adjacent_frames_and_one_ns_mismatch_cannot_count_as_stereo(offset):
    data = samples()
    for topic in (RIGHT, RIGHT_INFO):
        for row in data[topic]:
            row['stamp_ns'] += offset
    result = audit(data)
    assert not result['stereo_ready']
    assert result['stereo']['matched_timestamps'] < 31


@pytest.mark.parametrize('missing', [RIGHT, LEFT_INFO, RIGHT_INFO])
def test_missing_stereo_stream_is_failed(missing):
    data = samples()
    del data[missing]
    assert not audit(data)['stereo_ready']


@pytest.mark.parametrize('topic', [LEFT_INFO, RIGHT_INFO])
@pytest.mark.parametrize('mutation', ['offset', 'missing_frame'])
def test_every_image_needs_same_side_camera_info_with_exact_stamp(topic, mutation):
    data = samples()
    if mutation == 'offset':
        for row in data[topic]:
            row['stamp_ns'] += 1
    else:
        del data[topic][10]
    result = audit(data)
    assert not result['stereo_ready']
    assert not check(result, 'stereo_camera_info_pairs')['ok']


def test_extra_camera_info_does_not_prevent_complete_image_joins():
    data = samples()
    for topic in (LEFT_INFO, RIGHT_INFO):
        data[topic].append(dict(data[topic][-1], stamp_ns=EPOCH + 1_033_333_333))
    assert audit(data)['stereo_ready']


def test_rate_uses_each_topics_headers_and_integer_epoch_differences():
    data = samples()
    data[IMU] = [dict(data[IMU][0], stamp_ns=EPOCH + i * 1_000_001) for i in range(11)]
    result = audit(data)
    assert result['topics'][IMU]['rate_hz'] == pytest.approx(999.999000001)
    assert result['topics'][IMU]['median_interval_ns'] == 1_000_001
    assert result['topics'][LEFT]['rate_hz'] == pytest.approx(30)
    assert result['topics'][LEFT]['arrival_minus_header_ns']['median'] == 2_000_000


@pytest.mark.parametrize('kind', ['repeat', 'backward', 'zero', 'noninteger'])
def test_bad_header_timestamps_block_the_affected_stage(kind):
    data = samples()
    data[IMU][1]['stamp_ns'] = {'repeat': EPOCH, 'backward': EPOCH - 1,
                               'zero': 0, 'noninteger': float(EPOCH)}[kind]
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['stereo_ready']
    assert not result['vio_ready']
    assert not check(result, 'imu_headers')['ok']
    if kind in ('repeat', 'backward'):
        key = 'repeated_timestamps' if kind == 'repeat' else 'backward_timestamps'
        assert result['topics'][IMU][key] == 1


def test_repeated_stereo_headers_block_stereo_even_if_both_sides_repeat():
    data = samples()
    for topic in (LEFT, RIGHT, LEFT_INFO, RIGHT_INFO):
        data[topic][1]['stamp_ns'] = data[topic][0]['stamp_ns']
    assert not audit(data)['stereo_ready']


@pytest.mark.parametrize('topic,key,value', [
    (RIGHT_INFO, 'p', [400, 0, 320, 0, 0, 400, 240, 0, 0, 0, 1, 0]),
    (RIGHT_INFO, 'p', [400, 0, 320, -20, 0, 400, 240, 0, 0.1, 0, 1, 0]),
    (RIGHT, 'width', 320), (RIGHT_INFO, 'frame_id', 'wrong_optical'),
    (LEFT, 'frame_id', ''),
])
def test_invalid_baseline_rectification_dimensions_or_frames_block_stereo(topic, key, value):
    data = samples()
    for row in data[topic]:
        row[key] = value
    assert not audit(data)['stereo_ready']


@pytest.mark.parametrize('provenance', ['unknown', 'host_receive'])
def test_low_jitter_does_not_upgrade_unverified_imu_timing(provenance):
    result = audit(imu_timing_provenance=provenance, stationary=True, calibration=calibration())
    assert result['stereo_ready']
    assert not result['vio_ready']
    assert not check(result, 'imu_timing_provenance')['ok']


def test_stationary_evidence_requires_explicit_assertion_and_static_statistics():
    result = audit(imu_timing_provenance='sensor', calibration=calibration())
    assert not result['vio_ready']
    data = samples()
    for row in data[IMU]:
        row['accel'] = [0, 0, 1.0]
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert not result['vio_ready']
    assert not check(result, 'imu_stationary_evidence')['ok']


def test_low_imu_rate_keeps_stereo_but_blocks_vio():
    result = audit(samples(20_000_000), imu_timing_provenance='sensor', stationary=True,
                   calibration=calibration())
    assert result['stereo_ready'] and not result['vio_ready']
    assert not check(result, 'imu_rate')['ok']


@pytest.mark.parametrize('gap_ns,ready', [(50_000_000, True), (50_000_001, False), (60_000_000, False)])
def test_high_average_imu_rate_cannot_hide_a_long_gap(gap_ns, ready):
    data = samples()
    for row in data[IMU][100:]:
        row['stamp_ns'] += gap_ns - 5_000_000
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['stereo_ready']
    assert result['topics'][IMU]['rate_hz'] > 100
    assert result['topics'][IMU]['largest_gap_ns'] == gap_ns
    assert result['vio_ready'] is ready
    assert check(result, 'imu_intervals')['ok'] is ready


@pytest.mark.parametrize('slow_interval_ns,ready', [(20_000_000, True), (20_000_001, False)])
def test_imu_p95_gate_is_independent_of_average_and_maximum_gap(slow_interval_ns, ready):
    data = samples()
    stamp = EPOCH
    for i, row in enumerate(data[IMU]):
        if i:
            stamp += 5_000_000 if i <= 188 else slow_interval_ns
        row['stamp_ns'] = stamp
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['topics'][IMU]['rate_hz'] > 100
    assert result['topics'][IMU]['largest_gap_ns'] < 50_000_000
    assert result['topics'][IMU]['p95_interval_ns'] == slow_interval_ns
    assert result['vio_ready'] is ready
    assert check(result, 'imu_intervals')['ok'] is ready


@pytest.mark.parametrize('imu_offset_ns,ready', [
    (50_000_000, True), (50_000_001, False), (-50_000_000, True),
    (-50_000_011, False), (10_000_000_000, False),
])
def test_imu_interval_covers_camera_clock_with_at_most_50_ms_endpoint_tolerance(imu_offset_ns, ready):
    data = samples()
    for row in data[IMU]:
        row['stamp_ns'] += imu_offset_ns
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['stereo_ready']
    assert result['topics'][IMU]['rate_hz'] == pytest.approx(200)
    assert result['vio_ready'] is ready
    assert check(result, 'imu_camera_time_coverage')['ok'] is ready


def test_calibrated_td_is_added_to_camera_headers_before_imu_interval_comparison():
    data = samples()
    for row in data[IMU]:
        row['stamp_ns'] += 100_000_000
    config = calibration()
    assert not audit(data, imu_timing_provenance='sensor', stationary=True, calibration=config)['vio_ready']
    config['td'] = 0.1
    assert audit(data, imu_timing_provenance='sensor', stationary=True, calibration=config)['vio_ready']
    config['td'] = -0.1
    assert not audit(data, imu_timing_provenance='sensor', stationary=True, calibration=config)['vio_ready']


@pytest.mark.parametrize('td', [float('nan'), float('inf'), 'unknown', None, True, 1e308])
def test_invalid_or_unusable_td_cannot_authorize_imu_coverage(td):
    config = calibration()
    config['td'] = td
    result = audit(imu_timing_provenance='sensor', stationary=True, calibration=config)
    assert result['stereo_ready'] and not result['vio_ready']
    assert not check(result, 'imu_camera_time_coverage')['ok']
    json.dumps(result, allow_nan=False)


def test_stereo_experimental_minimum_accepts_interleaved_15_hz_and_rejects_5_hz():
    data = samples()
    for topic in (LEFT, RIGHT, LEFT_INFO, RIGHT_INFO):
        for i, row in enumerate(data[topic]):
            row['stamp_ns'] = EPOCH + i * 66_666_667
    assert audit(data)['stereo_ready']
    for topic in (LEFT, RIGHT, LEFT_INFO, RIGHT_INFO):
        for i, row in enumerate(data[topic]):
            row['stamp_ns'] = EPOCH + i * 200_000_000
    assert not audit(data)['stereo_ready']


def stationary_reference():
    return dict(schema_version=1, kind='vins_sensor_audit', stationary=True,
                vio_ready=True, imu_timing_provenance='sensor', calibration_sha256='a' * 64)


def test_bound_stationary_reference_can_supply_evidence_for_moving_bag():
    data = samples()
    for row in data[IMU]:
        row['accel'] = [2.0, 0.5, 9.0]
        row['gyro'] = [0.5, 0, 0]
    result = audit(data, imu_timing_provenance='sensor', calibration=calibration(),
                   calibration_sha256='a' * 64, stationary_reference=stationary_reference())
    assert result['vio_ready']
    assert result['stationary'] is False
    assert result['stationary_evidence']['source'] == 'reference'


@pytest.mark.parametrize('field,value', [('schema_version', 2), ('kind', 'other'),
                                        ('stationary', False), ('vio_ready', False),
                                        ('imu_timing_provenance', 'host_receive'),
                                        ('calibration_sha256', 'b' * 64)])
def test_unbound_or_nonstationary_reference_cannot_authorize_vio(field, value):
    reference = stationary_reference()
    reference[field] = value
    result = audit(imu_timing_provenance='sensor', calibration=calibration(),
                   calibration_sha256='a' * 64, stationary_reference=reference)
    assert not result['vio_ready']


def test_stationary_reference_cannot_authorize_itself_from_another_reference():
    reference = stationary_reference()
    reference['stationary_evidence'] = {'source': 'reference'}
    assert not audit(imu_timing_provenance='sensor', calibration=calibration(),
                     calibration_sha256='a' * 64, stationary_reference=reference)['vio_ready']


def test_reference_never_bypasses_current_bags_sensor_data_gates():
    data = samples(20_000_000)
    assert not audit(data, imu_timing_provenance='sensor', calibration=calibration(),
                     calibration_sha256='a' * 64, stationary_reference=stationary_reference())['vio_ready']


@pytest.mark.parametrize('field,value', [('accel', [0, float('nan'), 9.81]),
                                        ('gyro', [0, 0, float('inf')]),
                                        ('accel', [0, 9.81]), ('frame_id', '')])
def test_invalid_imu_samples_fail_and_json_is_finite(field, value):
    data = samples()
    data[IMU][4][field] = value
    result = audit(data, imu_timing_provenance='sensor', stationary=True, calibration=calibration())
    assert result['stereo_ready'] and not result['vio_ready']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('field,value', [('imu', 0), ('estimate_extrinsic', 1),
                                        ('acc_n', 0), ('gyr_n', float('inf')),
                                        ('acc_w', -1), ('gyr_w', 'bad'), ('g_norm', 0),
                                        ('body_T_cam0', [[1, 0, 0, 0]] * 4)])
def test_invalid_calibration_cannot_pass_vio(field, value):
    config = calibration()
    config[field] = value
    result = audit(imu_timing_provenance='sensor', stationary=True, calibration=config)
    assert not result['vio_ready']
    assert not check(result, 'imu_calibration')['ok']


def test_reflection_is_not_a_rigid_camera_rotation():
    config = calibration()
    config['body_T_cam1'][0][0] = -1
    assert not audit(imu_timing_provenance='sensor', stationary=True,
                     calibration=config)['vio_ready']


def test_malformed_pure_record_fails_as_audit_instead_of_raising():
    data = samples()
    data[RIGHT_INFO].append({'stamp_ns': EPOCH + 2_000_000_000})
    result = audit(data)
    assert not result['stereo_ready']
    json.dumps(result, allow_nan=False)


def fake_ros(monkeypatch, data, *, corrupt=None, storage='mcap'):
    """Only replace unavailable ROS transport; real extraction and gates execute."""
    rows = []
    for topic, records in data.items():
        for record in records:
            sec, nanosec = divmod(record['stamp_ns'], 1_000_000_000)
            header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
                                     frame_id=record['frame_id'])
            msg = SimpleNamespace(header=header)
            if topic == IMU:
                msg.linear_acceleration = SimpleNamespace(**dict(zip('xyz', record['accel'])))
                msg.angular_velocity = SimpleNamespace(**dict(zip('xyz', record['gyro'])))
            else:
                msg.width, msg.height = record['width'], record['height']
                if topic.endswith('camera_info'):
                    msg.p = record['p']
            rows.append((topic, msg, record['arrival_ns']))
    if corrupt:
        rows.append((corrupt, 'broken', EPOCH + 3_000_000_000))

    class Reader:
        def __init__(self):
            self.rows = iter(rows)
            self.remaining = len(rows)

        def open(self, options, converter):
            assert options.storage_id == storage
            assert converter.input_serialization_format == 'cdr'
            assert converter.output_serialization_format == 'cdr'

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=t, type='sensor_msgs/msg/' +
                    ('Imu' if t == IMU else 'CameraInfo' if t.endswith('camera_info') else 'Image'))
                    for t in data]

        def has_next(self):
            return self.remaining > 0

        def read_next(self):
            self.remaining -= 1
            return next(self.rows)

    def deserialize(data, message_type):
        if data == 'broken':
            raise ValueError('invalid CDR fixture')
        return data

    monkeypatch.setitem(sys.modules, 'rosbag2_py', SimpleNamespace(
        SequentialReader=Reader, StorageOptions=lambda **kw: SimpleNamespace(**kw),
        ConverterOptions=lambda **kw: SimpleNamespace(**kw)))
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', SimpleNamespace(deserialize_message=deserialize))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', SimpleNamespace(get_message=lambda name: name))


def bag_dir(tmp_path, storage='mcap'):
    bag = tmp_path / 'bag'
    bag.mkdir()
    metadata = f'rosbag2_bagfile_information:\n  storage_identifier: {storage}\n  relative_file_paths: [trial.bin]\n'
    (bag / 'metadata.yaml').write_text(metadata, encoding='utf-8')
    (bag / 'trial.bin').write_bytes(b'fake ROS bag transport fixture')
    return bag


def test_cli_reads_metadata_selected_bag_and_digests_exact_input_bytes(tmp_path, monkeypatch):
    bag = bag_dir(tmp_path)
    fake_ros(monkeypatch, samples())
    config = tmp_path / 'calibration.yaml'
    config.write_text(json.dumps(calibration()), encoding='utf-8')
    output = tmp_path / 'audit.json'
    assert cli().main(['--bag', str(bag), '--output', str(output), '--stationary',
                       '--imu-timing-provenance', 'sensor', '--calibration', str(config)]) == 0
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['vio_ready']
    assert result['bag_uri'] == str(bag.resolve())
    assert result['calibration_sha256'] == hashlib.sha256(config.read_bytes()).hexdigest()
    assert result['bag_metadata_sha256'] == hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()
    output.write_text('keep existing', encoding='utf-8')
    assert cli().main(['--bag', str(bag), '--output', str(output)]) == 2
    assert output.read_text(encoding='utf-8') == 'keep existing'


@pytest.mark.parametrize('storage', ['sqlite3', 'mcap'])
def test_cli_stereo_failure_returns_one(tmp_path, monkeypatch, storage):
    bag = bag_dir(tmp_path, storage)
    data = samples()
    data[RIGHT] = []
    fake_ros(monkeypatch, data, storage=storage)
    output = tmp_path / 'audit.json'
    assert cli().main(['--bag', str(bag), '--output', str(output)]) == 1
    assert not json.loads(output.read_text(encoding='utf-8'))['stereo_ready']


def test_cli_required_message_deserialization_error_is_hard_failed_audit(tmp_path, monkeypatch):
    bag = bag_dir(tmp_path)
    fake_ros(monkeypatch, samples(), corrupt=IMU)
    output = tmp_path / 'audit.json'
    assert cli().main(['--bag', str(bag), '--output', str(output)]) == 2
    result = json.loads(output.read_text(encoding='utf-8'))
    assert not result['stereo_ready'] and not result['vio_ready']
    assert not check(result, 'bag_deserialization')['ok']


def test_cli_no_ros_fails_clearly_without_metadata_success(tmp_path, monkeypatch, capsys):
    bag = bag_dir(tmp_path)
    monkeypatch.setitem(sys.modules, 'rosbag2_py', None)
    output = tmp_path / 'audit.json'
    assert cli().main(['--bag', str(bag), '--output', str(output)]) == 2
    assert 'ROS' in capsys.readouterr().err
    assert not output.exists()


def test_cli_reads_opencv_matrix_calibration(tmp_path):
    config = tmp_path / 'opencv.yaml'
    config.write_text('%YAML:1.0\n---\nbody_T_cam0: !!opencv-matrix\n'
                      '  rows: 4\n  cols: 4\n  dt: d\n'
                      '  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n',
                      encoding='utf-8')
    parsed = cli().load_calibration(config)
    assert parsed['body_T_cam0'] == calibration()['body_T_cam0']


def test_cli_stationary_reference_digest_is_bound_to_moving_bag_audit(tmp_path, monkeypatch):
    bag = bag_dir(tmp_path)
    data = samples()
    for row in data[IMU]:
        row['gyro'] = [0.5, 0, 0]
    fake_ros(monkeypatch, data)
    config = tmp_path / 'calibration.yaml'
    config.write_text(json.dumps(calibration()), encoding='utf-8')
    reference = stationary_reference()
    reference['calibration_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest()
    source = tmp_path / 'static-audit.json'
    source.write_text(json.dumps(reference), encoding='utf-8')
    output = tmp_path / 'moving-audit.json'
    assert cli().main(['--bag', str(bag), '--output', str(output), '--stationary-audit', str(source),
                       '--imu-timing-provenance', 'sensor', '--calibration', str(config)]) == 0
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['vio_ready'] and not result['stationary']
    assert result['stationary_evidence']['path'] == str(source.resolve())
    assert result['stationary_evidence']['sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
