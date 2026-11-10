"""Reject VINS replay when recorded static TF disagrees with calibration.

OdometryVINSFusion replaces YAML TIC/RIC using IMU and camera local TF.
Validation never rewrites the bag, static TF, or the supplied calibration.
ROS dependencies are loaded only when opening a bag.
"""
from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from pathlib import Path

import numpy as np


_TOLERANCE = 1e-4
_BACKEND_CONTEXT = ('RTAB-Map VINS backend overrides YAML TIC/RIC with '
                    'imuLocalTransform.inverse() * model.localTransform(); '
                    'recorded /tf_static must match the fixed camera-IMU calibration.')


def _fail(reason):
    raise ValueError(f'{reason}. {_BACKEND_CONTEXT}')


def _array(value, shape, name):
    try:
        result = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError):
        _fail(f'Invalid {name}')
    if result.shape != shape or not np.isfinite(result).all():
        _fail(f'{name} must have shape {shape} and finite values')
    return result


def _rigid_matrix(value, name):
    if isinstance(value, Mapping):
        if value.get('rows') != 4 or value.get('cols') != 4:
            _fail(f'{name} calibration matrix requires rows=4 and cols=4')
        data = _array(value.get('data'), (16,), name)
        value = data.reshape(4, 4)
    matrix = _array(value, (4, 4), name)
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=_TOLERANCE)
            or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=_TOLERANCE)
            or not np.isclose(np.linalg.det(rotation), 1, rtol=0, atol=_TOLERANCE)):
        _fail(f'{name} calibration must be a rigid homogeneous matrix')
    return matrix


def _frame(value, name):
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        _fail(f'{name} frame must be a nonempty unambiguous string')
    return value


def _transform(record):
    if not isinstance(record, Mapping):
        _fail('Static TF record must be a mapping')
    parent = _frame(record.get('parent'), 'parent')
    child = _frame(record.get('child'), 'child')
    if parent == child:
        _fail(f'Static TF cycle at {parent}')
    translation = _array(record.get('translation'), (3,), 'translation')
    quaternion = _array(record.get('rotation'), (4,), 'rotation quaternion')
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or not np.isclose(norm, 1, rtol=0, atol=_TOLERANCE):
        _fail('Static TF quaternion must be nonzero and unit length')
    x, y, z, w = quaternion / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]]
    matrix[:3, 3] = translation
    return parent, child, matrix


def _graph(transforms):
    parents, edges, graph = {}, {}, {}
    try:
        records = iter(transforms)
    except TypeError:
        _fail('Missing static TF transforms')
    for record in records:
        parent, child, matrix = _transform(record)
        key = (parent, child)
        if key in edges:
            if not np.allclose(edges[key], matrix, rtol=0, atol=1e-12):
                _fail(f'Changed duplicate static TF edge {parent} -> {child}')
            continue
        if child in parents:
            _fail(f'Static TF child {child} has multiple parents')
        ancestor = parent
        while ancestor in parents:
            ancestor = parents[ancestor]
            if ancestor == child:
                _fail(f'Static TF cycle involving {child}')
        parents[child] = parent
        edges[key] = matrix
        graph.setdefault(parent, []).append((child, matrix))
        graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
    if not edges:
        _fail('Missing recorded /tf_static transforms')
    return graph


def _lookup(graph, source, target):
    if source not in graph or target not in graph:
        _fail(f'Missing static TF connectivity {source} -> {target}')
    queue = deque([(source, np.eye(4))])
    seen = {source}
    while queue:
        frame, matrix = queue.popleft()
        if frame == target:
            return matrix
        for child, edge in graph[frame]:
            if child not in seen:
                seen.add(child)
                queue.append((child, matrix @ edge))
    _fail(f'Missing static TF path {source} -> {target}')


def _matches(actual, expected, description):
    if not np.allclose(actual, expected, rtol=0, atol=_TOLERANCE):
        _fail(description)


def validate_extrinsics(transforms, calibration, *, imu_frame, left_frame,
                        right_frame, baseline, base_frame='base_footprint'):
    """Validate rigid static TF against fixed rectified stereo calibration.

    Matrices use T_parent_child coordinates; translation tolerance is 0.1 mm
    and rotation elements use an absolute 1e-4 tolerance. Equivalent repeated
    static edges are accepted, including the two quaternion signs.
    """
    for name, frame in [('IMU', imu_frame), ('left camera', left_frame),
                        ('right camera', right_frame), ('base', base_frame)]:
        _frame(frame, name)
    if isinstance(baseline, bool):
        _fail('Rectified baseline must be finite and positive')
    try:
        baseline = float(baseline)
    except (TypeError, ValueError, OverflowError):
        _fail('Rectified baseline must be finite and positive')
    if not np.isfinite(baseline) or baseline <= 0:
        _fail('Rectified baseline must be finite and positive')
    if not isinstance(calibration, Mapping):
        _fail('Missing fixed camera-IMU calibration mapping')
    cam0 = _rigid_matrix(calibration.get('body_T_cam0'), 'body_T_cam0')
    cam1 = _rigid_matrix(calibration.get('body_T_cam1'), 'body_T_cam1')
    stereo = np.eye(4)
    stereo[0, 3] = baseline
    _matches(np.linalg.inv(cam0) @ cam1, stereo,
             'Supplied body_T_cam1 does not have the positive rectified stereo baseline')
    graph = _graph(transforms)
    base_imu = _lookup(graph, base_frame, imu_frame)
    base_left = _lookup(graph, base_frame, left_frame)
    _lookup(graph, base_frame, right_frame)
    imu_left = np.linalg.inv(base_imu) @ base_left
    left_right = _lookup(graph, left_frame, right_frame)
    _matches(left_right, stereo, 'Recorded TF does not have the positive rectified stereo baseline')
    _matches(imu_left, cam0, 'Recorded IMU-to-left camera TF disagrees with body_T_cam0')
    _matches(imu_left @ left_right, cam1,
             'Recorded IMU-to-right camera TF disagrees with body_T_cam1')


def validate_bag_extrinsics(bag, config_path, report):
    """Read only recorded /tf_static and fail closed before starting replay.

    ``report`` is the sensor audit mapping; frame IDs and the right projection
    matrix bind this check to the actual camera and gyro topics in that audit.
    Missing ROS or bag storage dependencies are reported as ValueError.
    """
    try:
        from wheeltec_vision_nav.vins_audit import _load_yaml, load_calibration

        bag = Path(bag)
        calibration = load_calibration(config_path)
        metadata = _load_yaml((bag / 'metadata.yaml').read_text(encoding='utf-8-sig'))
        if not isinstance(metadata, Mapping):
            _fail('Bag metadata must be a mapping')
        info = metadata.get('rosbag2_bagfile_information', metadata)
        if not isinstance(info, Mapping) or info.get('storage_identifier') not in ('sqlite3', 'mcap'):
            _fail('Bag metadata storage_identifier must be sqlite3 or mcap')
        models = report['camera_models']
        left = models['left']['frame_id']
        right = models['right']['frame_id']
        frames = report['topics']['/imu/data_raw']['frame_ids']
        if not isinstance(frames, (list, tuple)) or len(frames) != 1:
            _fail('Audit must identify exactly one IMU gyro frame')
        projection = _array(models['right']['p'], (12,), 'right camera projection')
        if projection[0] <= 0:
            _fail('Right camera projection focal length must be positive')
        baseline = -projection[3] / projection[0]
        try:
            from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            _fail(f'ROS2 rosbag2_py, rclpy and rosidl_runtime_py are required; source ROS2 ({exc})')
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(bag), storage_id=info['storage_identifier']),
                    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if types.get('/tf_static') != 'tf2_msgs/msg/TFMessage':
            _fail('Missing /tf_static or invalid TFMessage topic type')
        reader.set_filter(StorageFilter(topics=['/tf_static']))
        message_type = get_message('tf2_msgs/msg/TFMessage')
        transforms = []
        while reader.has_next():
            name, serialized, _stamp = reader.read_next()
            if name != '/tf_static':
                continue
            message = deserialize_message(serialized, message_type)
            for transform in message.transforms:
                translation, rotation = transform.transform.translation, transform.transform.rotation
                transforms.append({'parent': transform.header.frame_id,
                                   'child': transform.child_frame_id,
                                   'translation': [translation.x, translation.y, translation.z],
                                   'rotation': [rotation.x, rotation.y, rotation.z, rotation.w]})
        validate_extrinsics(transforms, calibration, imu_frame=frames[0],
                            left_frame=left, right_frame=right, baseline=baseline)
    except Exception as exc:
        if isinstance(exc, ValueError) and _BACKEND_CONTEXT in str(exc):
            raise
        _fail(f'Cannot validate recorded bag extrinsics: {exc}')
