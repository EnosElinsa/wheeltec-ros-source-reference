"""Read a ROS2 bag and emit a sensor audit without metadata-only fallback.

Run ``python -m wheeltec_vision_nav.vins_audit --help`` in a sourced ROS2
environment. Exit 0 means stereo-ready (VO phase), not VIO-ready. Exit 1 is
a failed stereo audit; exit 2 reports input, dependency or bag read failures.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

from wheeltec_vision_nav.vins_audit_logic import IMU_TOPIC, REQUIRED_TOPICS, audit_samples
from wheeltec_vision_nav.vins_experiment import bag_fingerprint


def _load_yaml(text, *, opencv=False):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError('PyYAML is required to read bag metadata and calibration.') from exc
    if not opencv:
        return yaml.safe_load(text)

    class CalibrationLoader(yaml.SafeLoader):
        pass

    def matrix(loader, node):
        value = loader.construct_mapping(node, deep=True)
        rows, cols, data = value.get('rows'), value.get('cols'), value.get('data')
        if (not isinstance(rows, int) or not isinstance(cols, int) or rows <= 0 or cols <= 0
                or not isinstance(data, list) or len(data) != rows * cols):
            raise ValueError('Malformed OpenCV matrix rows/cols/data')
        return [data[i * cols:(i + 1) * cols] for i in range(rows)]

    CalibrationLoader.add_constructor('tag:yaml.org,2002:opencv-matrix', matrix)
    # OpenCV FileStorage uses a nonstandard YAML directive, never unsafe tags.
    lines = text.lstrip('\ufeff').splitlines()
    if lines and lines[0].strip() == '%YAML:1.0':
        lines = lines[1:]
    return yaml.load('\n'.join(lines), Loader=CalibrationLoader)


def load_calibration(path):
    """Read ordinary or OpenCV FileStorage YAML using only a SafeLoader."""
    payload = _load_yaml(Path(path).read_text(encoding='utf-8-sig'), opencv=True)
    if not isinstance(payload, dict):
        raise ValueError('Calibration must be a YAML mapping')
    return payload


def _message_record(name, message, arrival_ns):
    header = message.header
    sec, nanosec = header.stamp.sec, header.stamp.nanosec
    if not isinstance(sec, int) or not isinstance(nanosec, int) or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('Malformed ROS header sec/nanosec')
    record = {'stamp_ns': sec * 1_000_000_000 + nanosec,
              'arrival_ns': int(arrival_ns), 'frame_id': header.frame_id}
    if name == IMU_TOPIC:
        record['accel'] = [getattr(message.linear_acceleration, axis) for axis in 'xyz']
        record['gyro'] = [getattr(message.angular_velocity, axis) for axis in 'xyz']
    else:
        record.update(width=message.width, height=message.height)
        if name.endswith('camera_info'):
            record['p'] = list(message.p)
    return record


def read_bag(bag_path, metadata):
    """Deserialize every required message, retaining failures as hard evidence.

    ROS imports stay here so that logic and CLI --help work without ROS. The
    bag timestamp is recorded separately from the sensor's header timestamp.
    """
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError('ROS2 rosbag2_py, rclpy and rosidl_runtime_py are required; '
                           'source the ROS2 environment. No metadata-only audit is available.') from exc
    info = metadata.get('rosbag2_bagfile_information', metadata)
    if not isinstance(info, dict):
        raise ValueError('Malformed bag metadata mapping')
    storage = info.get('storage_identifier')
    if storage not in ('sqlite3', 'mcap'):
        raise ValueError('Bag metadata storage_identifier must be sqlite3 or mcap')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id=storage),
                ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    samples = {name: [] for name in REQUIRED_TOPICS}
    errors = []
    message_types = {}
    for name in REQUIRED_TOPICS:
        if name in types:
            try:
                expected = 'sensor_msgs/msg/' + ('Imu' if name == IMU_TOPIC else
                                                 'CameraInfo' if name.endswith('camera_info') else 'Image')
                if types[name] != expected:
                    raise ValueError(f'expected {expected}, got {types[name]}')
                message_types[name] = get_message(types[name])
            except Exception as exc:
                errors.append(f'{name}: message type resolution failed: {exc}')
    while reader.has_next():
        name, serialized, arrival_ns = reader.read_next()
        if name not in samples:
            continue
        try:
            if name not in message_types:
                raise ValueError('required message type unavailable')
            message = deserialize_message(serialized, message_types[name])
            samples[name].append(_message_record(name, message, arrival_ns))
        except Exception as exc:
            errors.append(f'{name}: deserialization/extraction failed at bag ns {arrival_ns}: {exc}')
    return samples, errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--imu-timing-provenance', choices=('host_receive', 'unknown', 'sensor'), default='unknown')
    parser.add_argument('--calibration', type=Path)
    static = parser.add_mutually_exclusive_group()
    static.add_argument('--stationary', action='store_true', help='Assert the audited IMU segment is stationary.')
    static.add_argument('--stationary-audit', type=Path, help='Prior passing static audit bound to the same calibration bytes.')
    args = parser.parse_args(argv)
    try:
        bag = args.bag.expanduser().resolve()
        output = args.output.expanduser().resolve()
        if output.exists():
            raise FileExistsError(f'Refusing to overwrite existing output: {output}')
        if not bag.is_dir():
            raise ValueError('--bag must be a ROS2 bag directory containing metadata.yaml')
        metadata_path = bag / 'metadata.yaml'
        metadata_bytes = metadata_path.read_bytes()
        metadata = _load_yaml(metadata_bytes.decode('utf-8-sig'))
        if not isinstance(metadata, dict):
            raise ValueError('Bag metadata must be a mapping')
        config, config_digest = None, None
        if args.calibration is not None:
            config_bytes = args.calibration.read_bytes()
            config_digest = hashlib.sha256(config_bytes).hexdigest()
            config = load_calibration(args.calibration)
        reference, reference_bytes = None, None
        if args.stationary_audit is not None:
            reference_bytes = args.stationary_audit.read_bytes()
            reference = json.loads(reference_bytes.decode('utf-8-sig'))
        samples, errors = read_bag(bag, metadata)
        result = audit_samples(samples, imu_timing_provenance=args.imu_timing_provenance,
                               stationary=args.stationary, calibration=config, bag_uri=str(bag),
                               calibration_sha256=config_digest, stationary_reference=reference)
        result['bag_metadata_sha256'] = hashlib.sha256(metadata_bytes).hexdigest()
        result['bag_files'] = bag_fingerprint(bag)
        if args.stationary_audit is not None:
            result['stationary_evidence'].update(path=str(args.stationary_audit.expanduser().resolve()),
                                                 sha256=hashlib.sha256(reference_bytes).hexdigest())
        result['checks'].append({'name': 'bag_deserialization', 'ok': not errors,
                                 'detail': errors if errors else 'All required-topic messages decoded; no metadata fallback.'})
        if errors:
            result['stereo_ready'] = result['vio_ready'] = False
        # Exclusive creation also protects against an output appearing during bag processing.
        payload = json.dumps(result, indent=2, allow_nan=False) + '\n'
        with output.open('x', encoding='utf-8') as stream:
            stream.write(payload)
        return 2 if errors else 0 if result['stereo_ready'] else 1
    except Exception as exc:
        print(f'vins_audit: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
