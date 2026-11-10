"""Gate replay on recorded static TF, including the backend YAML override."""
import copy
import importlib
import math
import sys
from types import SimpleNamespace

import pytest


def gate():
    return importlib.import_module('wheeltec_vision_nav.vins_tf')


def edge(parent, child, xyz=(0, 0, 0), quat=(0, 0, 0, 1)):
    return dict(parent=parent, child=child, translation=list(xyz), rotation=list(quat))


def rig():
    q = (0, 0, math.sqrt(.5), math.sqrt(.5))
    return [edge('base_footprint', 'imu', (1, 2, 3), q),
            edge('base_footprint', 'mount', (1, 2, 3), q),
            edge('mount', 'left', (.1, .2, .3), q),
            edge('left', 'right', (.05, 0, 0))]


def calibration():
    return {'body_T_cam0': [[0, -1, 0, .1], [1, 0, 0, .2],
                            [0, 0, 1, .3], [0, 0, 0, 1]],
            'body_T_cam1': [[0, -1, 0, .1], [1, 0, 0, .25],
                            [0, 0, 1, .3], [0, 0, 0, 1]]}


def validate(transforms=None, config=None, **kwargs):
    return gate().validate_extrinsics(rig() if transforms is None else transforms,
                                     calibration() if config is None else config,
                                     imu_frame='imu', left_frame='left', right_frame='right',
                                     baseline=.05, **kwargs)


def test_rotated_translated_rig_passes_without_mutation():
    transforms, config = rig(), calibration()
    original = copy.deepcopy((transforms, config))
    assert validate(transforms, config) is None
    assert (transforms, config) == original


def test_plausible_yaml_does_not_let_identity_cad_tf_pass():
    transforms = [edge('base_footprint', 'imu'), edge('base_footprint', 'left'),
                  edge('left', 'right', (.05, 0, 0))]
    with pytest.raises(ValueError, match='(?i)overrid.*YAML|YAML.*overrid'):
        validate(transforms)


def test_missing_base_imu_connectivity_rejects():
    with pytest.raises(ValueError, match='(?i)connect|path|missing'):
        validate(rig()[1:])


def test_recorded_right_baseline_rejects():
    transforms = rig()
    transforms[-1]['translation'][0] = -.05
    with pytest.raises(ValueError, match='(?i)baseline|rectif'):
        validate(transforms)


def test_supplied_right_calibration_must_have_rectified_baseline():
    config = calibration()
    config['body_T_cam1'][1][3] = .3
    with pytest.raises(ValueError, match='(?i)baseline|rectif|cam1'):
        validate(config=config)


@pytest.mark.parametrize('extra', [edge('other', 'left'), edge('right', 'base_footprint')])
def test_multi_parent_or_cycle_rejects(extra):
    with pytest.raises(ValueError, match='(?i)parent|cycle'):
        validate(rig() + [extra])


def test_changed_duplicate_static_edge_rejects():
    with pytest.raises(ValueError, match='(?i)chang|conflict|duplicate'):
        validate(rig() + [edge('left', 'right', (.051, 0, 0))])


def test_identical_and_quaternion_sign_equivalent_duplicates_pass():
    transforms = rig()
    duplicate = copy.deepcopy(transforms[0])
    duplicate['rotation'] = [-v for v in duplicate['rotation']]
    assert validate(transforms + copy.deepcopy(transforms) + [duplicate]) is None


@pytest.mark.parametrize('quat', [(0, 0, 0, 0), (0, 0, 0, 2),
                                   (0, 0, math.nan, 1), (0, 0, 1)])
def test_bad_quaternion_rejects(quat):
    transforms = rig()
    transforms[0]['rotation'] = quat
    with pytest.raises(ValueError, match='(?i)quaternion|rotation'):
        validate(transforms)


def test_nonfinite_translation_rejects():
    transforms = rig()
    transforms[0]['translation'][0] = math.inf
    with pytest.raises(ValueError, match='(?i)translation|finite'):
        validate(transforms)


def test_disconnected_base_rejects_even_if_sensor_relative_transforms_match():
    transforms = rig()
    for item in transforms:
        if item['parent'] == 'base_footprint':
            item['parent'] = 'floating_rig'
    with pytest.raises(ValueError, match='(?i)connect|path|missing'):
        validate(transforms)


@pytest.mark.parametrize('distance, accepted', [(0.00005, True), (0.0002, False)])
def test_calibration_comparison_uses_point_one_mm_absolute_tolerance(distance, accepted):
    transforms = rig()
    transforms[2]['translation'][0] += distance
    if accepted:
        assert validate(transforms) is None
    else:
        with pytest.raises(ValueError, match='(?i)cam0'):
            validate(transforms)


def test_reflected_calibration_rotation_rejects():
    config = calibration()
    config['body_T_cam0'][2][2] = -1
    with pytest.raises(ValueError, match='(?i)rigid|rotation'):
        validate(config=config)


def test_opencv_calibration_matrix_mapping_passes():
    config = {name: {'rows': 4, 'cols': 4, 'data': sum(rows, [])}
              for name, rows in calibration().items()}
    assert validate(config=config) is None


@pytest.mark.parametrize('value', [None, [[1]], {'rows': 3, 'cols': 4, 'data': [0] * 12}])
def test_missing_or_invalid_calibration_rejects(value):
    config = calibration()
    config['body_T_cam0'] = value
    with pytest.raises(ValueError, match='(?i)cam0|matrix|calibration'):
        validate(config=config)


def report():
    return {'camera_models': {'left': {'frame_id': 'left'},
                              'right': {'frame_id': 'right',
                                        'p': [400., 0., 320., -20., 0., 400., 240., 0., 0., 0., 1., 0.]}},
            'topics': {'/imu/data_raw': {'frame_ids': ['imu']}}}


def prepare_bag(tmp_path, storage='mcap'):
    (tmp_path / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  storage_identifier: ' + storage + '\n')
    import yaml
    config = tmp_path / 'calibration.yaml'
    config.write_text(yaml.safe_dump(calibration()))
    return config


def fake_ros(monkeypatch, transforms, *, storage='mcap', topic_type='tf2_msgs/msg/TFMessage'):
    messages = []
    for item in transforms:
        t, q = item['translation'], item['rotation']
        messages.append(SimpleNamespace(header=SimpleNamespace(frame_id=item['parent']),
                                        child_frame_id=item['child'], transform=SimpleNamespace(
                                            translation=SimpleNamespace(x=t[0], y=t[1], z=t[2]),
                                            rotation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))))

    class Reader:
        def __init__(self):
            self.messages = [('/tf_static', SimpleNamespace(transforms=messages), 123)]

        def open(self, options, converters):
            if options.storage_id != storage:
                raise ValueError('metadata storage was not honored')

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name='/tf_static', type=topic_type)]

        def set_filter(self, value):
            if value.topics != ['/tf_static']:
                raise ValueError('must filter only static TF')

        def has_next(self):
            return bool(self.messages)

        def read_next(self):
            return self.messages.pop(0)

    monkeypatch.setitem(sys.modules, 'rosbag2_py', SimpleNamespace(
        SequentialReader=Reader, ConverterOptions=SimpleNamespace,
        StorageOptions=SimpleNamespace, StorageFilter=SimpleNamespace))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', SimpleNamespace(
        deserialize_message=lambda data, cls: data))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', SimpleNamespace(
        get_message=lambda name: object))


@pytest.mark.parametrize('storage', ['mcap', 'sqlite3'])
def test_bag_gate_reads_actual_static_tf_with_metadata_storage(tmp_path, monkeypatch, storage):
    config = prepare_bag(tmp_path, storage)
    fake_ros(monkeypatch, rig(), storage=storage)
    assert gate().validate_bag_extrinsics(tmp_path, config, report()) is None


def test_bag_gate_rejects_recorded_cad_extrinsics(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path)
    transforms = [edge('base_footprint', 'imu'), edge('base_footprint', 'left'),
                  edge('left', 'right', (.05, 0, 0))]
    fake_ros(monkeypatch, transforms)
    with pytest.raises(ValueError, match='(?i)overrid.*YAML|YAML.*overrid'):
        gate().validate_bag_extrinsics(tmp_path, config, report())


def test_missing_ros_dependencies_fail_clearly(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path)
    monkeypatch.setitem(sys.modules, 'rosbag2_py', None)
    with pytest.raises(ValueError, match='(?i)ROS2|rosbag2_py'):
        gate().validate_bag_extrinsics(tmp_path, config, report())


@pytest.mark.parametrize('frames', [[], ['imu', 'other']])
def test_bag_gate_requires_one_gyro_frame(tmp_path, monkeypatch, frames):
    config = prepare_bag(tmp_path)
    fake_ros(monkeypatch, rig())
    audit = report()
    audit['topics']['/imu/data_raw']['frame_ids'] = frames
    with pytest.raises(ValueError, match='(?i)IMU|gyro|frame'):
        gate().validate_bag_extrinsics(tmp_path, config, audit)


def test_missing_tf_static_rejects(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path)
    fake_ros(monkeypatch, [])
    with pytest.raises(ValueError, match='(?i)connect|missing|tf_static'):
        gate().validate_bag_extrinsics(tmp_path, config, report())


def test_wrong_tf_topic_type_rejects(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path)
    fake_ros(monkeypatch, rig(), topic_type='sensor_msgs/msg/Imu')
    with pytest.raises(ValueError, match='(?i)TFMessage|type'):
        gate().validate_bag_extrinsics(tmp_path, config, report())


def test_unsupported_bag_storage_rejects(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path, 'unrecognized')
    fake_ros(monkeypatch, rig())
    with pytest.raises(ValueError, match='storage_identifier'):
        gate().validate_bag_extrinsics(tmp_path, config, report())


def test_nonpositive_report_baseline_rejects(tmp_path, monkeypatch):
    config = prepare_bag(tmp_path)
    fake_ros(monkeypatch, rig())
    audit = report()
    audit['camera_models']['right']['p'][3] = 20.
    with pytest.raises(ValueError, match='(?i)baseline'):
        gate().validate_bag_extrinsics(tmp_path, config, audit)
