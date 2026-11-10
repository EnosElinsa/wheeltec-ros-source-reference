"""ROS-independent experimental sensor gates for the R680 VINS evaluation.

These conservative experiment gates are not general VINS-Fusion requirements.
Equal header timestamps and arrival latency cannot establish hardware sync.
The calibration checks establish shape/plausibility, not physical calibration.
"""

from __future__ import annotations

import math
import statistics
from collections import Counter
from pathlib import Path

LEFT_IMAGE = '/camera/infra1/image_rect_raw'
RIGHT_IMAGE = '/camera/infra2/image_rect_raw'
LEFT_INFO = '/camera/infra1/camera_info'
RIGHT_INFO = '/camera/infra2/camera_info'
IMU_TOPIC = '/imu/data_raw'
STEREO_TOPICS = (LEFT_IMAGE, RIGHT_IMAGE, LEFT_INFO, RIGHT_INFO)
REQUIRED_TOPICS = STEREO_TOPICS + (IMU_TOPIC,)

MIN_STEREO_HZ = 10.0
MIN_IMU_HZ_EXCLUSIVE = 100.0
MAX_IMU_GAP_NS = 50_000_000
MAX_IMU_P95_INTERVAL_NS = 20_000_000
IMU_CAMERA_ENDPOINT_TOLERANCE_NS = 50_000_000
MIN_STATIONARY_DURATION_NS = 1_000_000_000
MAX_STATIC_ACCEL_STD_NORM = 0.5
MAX_STATIC_GYRO_MEAN_NORM = 0.1
MAX_STATIC_GYRO_STD_NORM = 0.05
GRAVITY_RELATIVE_TOLERANCE = 0.2


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _positive_stamp(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _safe_number(value):
    return value if value is not None and _finite_number(value) else None


def _mean(values):
    if not values:
        return None
    try:
        return _safe_number(statistics.fmean(values))
    except (OverflowError, ValueError):
        return None


def _stddev(values):
    if not values:
        return None
    try:
        return _safe_number(statistics.pstdev(values))
    except (OverflowError, ValueError, AttributeError):
        return None


def _distribution(values):
    if not values:
        return {'mean': None, 'stddev': None, 'median': None, 'p95': None,
                'min': None, 'max': None}
    ordered = sorted(values)
    return {'mean': _mean(values), 'stddev': _stddev(values),
            'median': _safe_number(statistics.median(ordered)),
            'p95': _safe_number(ordered[math.ceil(len(ordered) * 0.95) - 1]),
            'min': _safe_number(ordered[0]), 'max': _safe_number(ordered[-1])}


def _vector(value):
    return (isinstance(value, (list, tuple)) and len(value) == 3
            and all(_finite_number(axis) for axis in value))


def _vector_statistics(rows, key):
    vectors = [row[key] for row in rows if _vector(row.get(key))]
    axes = list(zip(*vectors)) if vectors else [[], [], []]
    norms = [math.hypot(*vector) for vector in vectors]
    finite_norms = [value for value in norms if math.isfinite(value)]
    return ({'mean': [_mean(axis) for axis in axes],
             'stddev': [_stddev(axis) for axis in axes],
             'valid_samples': len(vectors),
             'invalid_samples': len(rows) - len(vectors)}, _distribution(finite_norms))


def _topic_statistics(records, is_imu=False):
    rows = [row for row in records if isinstance(row, dict)]
    stamps = [row['stamp_ns'] for row in rows if _positive_stamp(row.get('stamp_ns'))]
    # Subtract integer epochs first. Converting epochs to float loses nanoseconds.
    intervals = [right - left for left, right in zip(stamps, stamps[1:])]
    positive_intervals = [gap for gap in intervals if gap > 0]
    duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
    frames = sorted({row['frame_id'] for row in rows
                     if isinstance(row.get('frame_id'), str) and row['frame_id'].strip()})
    latency = [row['arrival_ns'] - row['stamp_ns'] for row in rows
               if _positive_stamp(row.get('arrival_ns')) and _positive_stamp(row.get('stamp_ns'))]
    result = {
        'count': len(records), 'malformed_records': len(records) - len(rows),
        'valid_header_timestamps': len(stamps),
        'invalid_header_timestamps': len(records) - len(stamps),
        'repeated_timestamps': sum(gap == 0 for gap in intervals),
        'backward_timestamps': sum(gap < 0 for gap in intervals),
        'first_header_ns': stamps[0] if stamps else None,
        'last_header_ns': stamps[-1] if stamps else None,
        'duration_ns': max(duration, 0),
        'rate_hz': _safe_number((len(stamps) - 1) * 1_000_000_000 / duration) if duration > 0 else None,
        'median_interval_ns': statistics.median(positive_intervals) if positive_intervals else None,
        'p95_interval_ns': sorted(positive_intervals)[math.ceil(len(positive_intervals) * 0.95) - 1]
        if positive_intervals else None,
        'largest_gap_ns': max(positive_intervals) if positive_intervals else None,
        'frame_ids': frames,
        'invalid_frame_ids': sum(not isinstance(row.get('frame_id'), str)
                                 or not row['frame_id'].strip() for row in rows) + len(records) - len(rows),
        'arrival_minus_header_ns': _distribution(latency),
        'invalid_arrival_timestamps': len(records) - len(latency),
    }
    if is_imu:
        for key in ('accel', 'gyro'):
            result[key], result[key + '_norm'] = _vector_statistics(rows, key)
    return result


def _headers_ok(stats):
    return (stats['count'] > 0 and stats['invalid_header_timestamps'] == 0
            and stats['repeated_timestamps'] == 0 and stats['backward_timestamps'] == 0)


def _frames_ok(stats):
    return stats['invalid_frame_ids'] == 0 and len(stats['frame_ids']) == 1


def _dimensions(rows):
    dimensions = set()
    for row in rows:
        if not isinstance(row, dict):
            return None
        width, height = row.get('width'), row.get('height')
        if not (_positive_stamp(width) and _positive_stamp(height)):
            return None
        dimensions.add((width, height))
    return next(iter(dimensions)) if len(dimensions) == 1 else None


def _projection(rows, *, right):
    projections = []
    for row in rows:
        if not isinstance(row, dict):
            return None
        p = row.get('p')
        if not isinstance(p, (list, tuple)) or len(p) != 12 or not all(_finite_number(v) for v in p):
            return None
        if p[0] <= 0 or p[5] <= 0 or not math.isclose(p[10], 1.0, abs_tol=1e-9):
            return None
        if any(not math.isclose(p[i], 0.0, abs_tol=1e-9) for i in (1, 4, 7, 8, 9, 11)):
            return None
        baseline = -p[3] / p[0]
        if not math.isfinite(baseline) or (right and baseline == 0) or (not right and abs(p[3]) > 1e-9):
            return None
        projections.append(list(p))
    if not projections or any(not all(math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
                                      for a, b in zip(projections[0], p)) for p in projections[1:]):
        return None
    return projections[0]


def _camera_models(samples, stats):
    models = {}
    for label, image_topic, info_topic in (('left', LEFT_IMAGE, LEFT_INFO),
                                          ('right', RIGHT_IMAGE, RIGHT_INFO)):
        image_size, info_size = _dimensions(samples[image_topic]), _dimensions(samples[info_topic])
        if image_size is None or image_size != info_size:
            return {}, False
        if not (_frames_ok(stats[image_topic]) and _frames_ok(stats[info_topic])):
            return {}, False
        if stats[image_topic]['frame_ids'] != stats[info_topic]['frame_ids']:
            return {}, False
        p = _projection(samples[info_topic], right=label == 'right')
        if p is None:
            return {}, False
        models[label] = {'width': info_size[0], 'height': info_size[1],
                         'frame_id': stats[info_topic]['frame_ids'][0], 'p': p}
    left, right = models['left'], models['right']
    # Rectified images need equal sizes and compatible horizontal epipolar geometry.
    compatible = (left['width'] == right['width'] and left['height'] == right['height']
                  and all(math.isclose(left['p'][i], right['p'][i], rel_tol=1e-6, abs_tol=1e-6)
                          for i in (0, 5, 6)))
    return (models if compatible else {}), compatible


def _matrix(value):
    if isinstance(value, dict):
        if value.get('rows') != 4 or value.get('cols') != 4:
            return None
        data = value.get('data')
        if not isinstance(data, (tuple, list)) or len(data) != 16:
            return None
        value = [data[i:i + 4] for i in range(0, 16, 4)]
    if (not isinstance(value, (tuple, list)) or len(value) != 4
            or any(not isinstance(row, (tuple, list)) or len(row) != 4 for row in value)):
        return None
    return value if all(_finite_number(v) for row in value for v in row) else None


def _rigid_transform(value):
    matrix = _matrix(value)
    if matrix is None or any(not math.isclose(matrix[3][i], target, abs_tol=1e-6)
                             for i, target in enumerate((0, 0, 0, 1))):
        return False
    rotation = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if not math.isfinite(dot) or not math.isclose(dot, float(i == j), abs_tol=1e-5):
                return False
    a, b, c = rotation
    det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    return math.isfinite(det) and math.isclose(det, 1.0, abs_tol=1e-5)


def _calibration_ok(calibration):
    return (isinstance(calibration, dict) and calibration.get('imu') == 1
            and calibration.get('estimate_extrinsic') == 0
            and _finite_number(calibration.get('td', 0))
            and all(_finite_number(calibration.get(key)) and calibration[key] > 0
                    for key in ('acc_n', 'gyr_n', 'acc_w', 'gyr_w', 'g_norm'))
            and all(_rigid_transform(calibration.get(key)) for key in ('body_T_cam0', 'body_T_cam1')))


def _camera_imu_time_coverage(stats, calibration):
    if not isinstance(calibration, dict):
        return False
    td = calibration.get('td', 0)
    if not _finite_number(td) or not _finite_number(td * 1_000_000_000):
        return False
    # Fusion uses camera time + td. Convert only the small offset, never epochs.
    td_ns = round(td * 1_000_000_000)
    imu = stats[IMU_TOPIC]
    if not _headers_ok(imu):
        return False
    lower = imu['first_header_ns'] - IMU_CAMERA_ENDPOINT_TOLERANCE_NS
    upper = imu['last_header_ns'] + IMU_CAMERA_ENDPOINT_TOLERANCE_NS
    return all(_headers_ok(stats[topic])
               and lower <= stats[topic]['first_header_ns'] + td_ns
               and stats[topic]['last_header_ns'] + td_ns <= upper
               for topic in (LEFT_IMAGE, RIGHT_IMAGE))


def _norm(vector):
    return math.hypot(*vector) if _vector(vector) else None


def _stationary_ok(stats, calibration):
    if not _calibration_ok(calibration):
        return False
    accel_norm = stats['accel_norm']['mean']
    accel_std = _norm(stats['accel']['stddev'])
    gyro_mean = _norm(stats['gyro']['mean'])
    gyro_std = _norm(stats['gyro']['stddev'])
    return (stats['duration_ns'] >= MIN_STATIONARY_DURATION_NS
            and all(value is not None for value in (accel_norm, accel_std, gyro_mean, gyro_std))
            and abs(accel_norm - 9.80665) <= 9.80665 * GRAVITY_RELATIVE_TOLERANCE
            and abs(accel_norm - calibration['g_norm']) <= calibration['g_norm'] * GRAVITY_RELATIVE_TOLERANCE
            and accel_std <= MAX_STATIC_ACCEL_STD_NORM
            and gyro_mean <= MAX_STATIC_GYRO_MEAN_NORM
            and gyro_std <= MAX_STATIC_GYRO_STD_NORM)


def _sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _stationary_reference_ok(reference, calibration_sha256):
    if not isinstance(reference, dict):
        return False
    evidence = reference.get('stationary_evidence')
    return (reference.get('schema_version') == 1 and reference.get('kind') == 'vins_sensor_audit'
            and reference.get('stationary') is True and reference.get('vio_ready') is True
            and reference.get('imu_timing_provenance') == 'sensor'
            and _sha256(calibration_sha256) and reference.get('calibration_sha256') == calibration_sha256
            and (evidence is None or isinstance(evidence, dict) and evidence.get('source') == 'current_bag'))


def audit_samples(samples: dict[str, list[dict]], *, imu_timing_provenance='unknown',
                  stationary=False, calibration=None, bag_uri='', calibration_sha256=None,
                  stationary_reference=None):
    """Audit integer-nanosecond records without importing ROS, NumPy or YAML.

    ``stationary`` asserts the audited IMU segment is static. A separate static
    audit can instead supply evidence for motion trials only when bound to the
    identical calibration digest. Caller provenance remains an external assertion.
    """
    if imu_timing_provenance not in ('unknown', 'host_receive', 'sensor'):
        raise ValueError('IMU timing provenance must be unknown, host_receive or sensor')
    normalized = {topic: samples.get(topic, []) for topic in REQUIRED_TOPICS}
    for topic, rows in normalized.items():
        if not isinstance(rows, (list, tuple)):
            normalized[topic] = [None]
    stats = {topic: _topic_statistics(rows, topic == IMU_TOPIC) for topic, rows in normalized.items()}
    checks = []
    stereo_checks = []

    def add(name, ok, detail, *, stereo=False):
        checks.append({'name': name, 'ok': bool(ok), 'detail': detail})
        if stereo:
            stereo_checks.append(bool(ok))

    add('stereo_topics', all(stats[topic]['count'] > 0 for topic in STEREO_TOPICS),
        'Both raw rectified IR images and both CameraInfo topics must contain messages.', stereo=True)
    add('stereo_headers', all(_headers_ok(stats[topic]) for topic in STEREO_TOPICS),
        'Positive integer header ns, strictly increasing per topic; repeats and regressions fail.', stereo=True)
    add('stereo_rate', all(stats[topic]['rate_hz'] is not None
                           and stats[topic]['rate_hz'] >= MIN_STEREO_HZ for topic in (LEFT_IMAGE, RIGHT_IMAGE)),
        'Experimental B gate: each raw image stream >=10 Hz from its own headers.', stereo=True)
    stamps = {topic: Counter(row['stamp_ns'] for row in normalized[topic]
                             if isinstance(row, dict) and _positive_stamp(row.get('stamp_ns')))
              for topic in STEREO_TOPICS}
    matched = sum((stamps[LEFT_IMAGE] & stamps[RIGHT_IMAGE]).values())
    add('stereo_exact_pairs', matched >= 2 and matched == stats[LEFT_IMAGE]['count'] == stats[RIGHT_IMAGE]['count'],
        f'Exact integer-ns pairs={matched}; all images must pair. Adjacent frames are rejected; '
        'equal stamps cannot detect restamping or prove hardware synchronization.', stereo=True)
    add('stereo_camera_info_pairs', all(stats[image]['count'] > 0 and not (stamps[image] - stamps[info])
                                        for image, info in ((LEFT_IMAGE, LEFT_INFO), (RIGHT_IMAGE, RIGHT_INFO))),
        'Every image must join its own CameraInfo by exact integer header ns; '
        'extra CameraInfo messages are allowed, missing image counterparts fail.', stereo=True)
    models, camera_ok = _camera_models(normalized, stats)
    add('stereo_camera_models', camera_ok,
        'Stable matching dimensions/optical frames; finite rectified P with positive focal lengths, '
        'compatible epipolar geometry and nonzero right -Tx/fx baseline.', stereo=True)
    stereo_ready = all(stereo_checks)

    imu = stats[IMU_TOPIC]
    add('imu_headers', _headers_ok(imu), 'IMU header stamps must be positive integer ns and strictly increasing.')
    add('imu_rate', imu['rate_hz'] is not None and imu['rate_hz'] > MIN_IMU_HZ_EXCLUSIVE,
        'Experimental C gate: IMU rate strictly >100 Hz from its own headers, not a universal Fusion minimum.')
    add('imu_intervals', imu['largest_gap_ns'] is not None and imu['largest_gap_ns'] <= MAX_IMU_GAP_NS
        and imu['p95_interval_ns'] is not None and imu['p95_interval_ns'] <= MAX_IMU_P95_INTERVAL_NS,
        'Experimental C gate: largest IMU header gap <=50 ms and p95 interval <=20 ms; '
        'mean rate alone cannot authorize an intermittent stream.')
    add('imu_camera_time_coverage', _camera_imu_time_coverage(stats, calibration),
        'Experimental C gate: each camera header interval plus calibrated finite td seconds '
        '(default 0) must fit the IMU header interval within <=50 ms at either endpoint. '
        'Clock overlap does not prove physical synchronization.')
    add('imu_vectors', imu['count'] > 0 and imu['malformed_records'] == 0
        and all(imu[key]['invalid_samples'] == 0 for key in ('accel', 'gyro')),
        'Every IMU sample needs finite three-axis acceleration and angular velocity.')
    add('imu_frame', _frames_ok(imu), 'One stable, nonempty IMU frame_id is required.')
    add('imu_timing_provenance', imu_timing_provenance == 'sensor',
        'Experimental C gate requires asserted sensor timestamps; low jitter or arrival minus header '
        'cannot upgrade host_receive/unknown provenance or establish hardware synchronization.')
    add('imu_calibration', _calibration_ok(calibration),
        'imu:1, estimate_extrinsic:0, finite rigid body_T_cam0/1, positive finite '
        'acc_n/gyr_n/acc_w/gyr_w/g_norm, finite td (default 0). Shape checks do not verify physical calibration.')
    if stationary:
        static_ok = _stationary_ok(imu, calibration)
        evidence = {'source': 'current_bag', 'ok': static_ok}
    else:
        static_ok = _stationary_reference_ok(stationary_reference, calibration_sha256)
        evidence = {'source': 'reference' if stationary_reference is not None else 'none', 'ok': static_ok}
    add('imu_stationary_evidence', static_ok,
        'Experimental static segment gate: asserted >=1 s, acceleration within 20% of g and configured g_norm, '
        'accel axis-std norm <=0.5 m/s^2, gyro mean norm <=0.1 rad/s, gyro axis-std norm <=0.05 rad/s; '
        'or a nonrecursive passing sensor-timed stationary audit with the same calibration SHA256.')
    return {
        'schema_version': 1, 'kind': 'vins_sensor_audit',
        'bag_uri': str(Path(bag_uri).expanduser().resolve()) if bag_uri else '',
        'imu_timing_provenance': imu_timing_provenance, 'stationary': bool(stationary),
        'calibration_sha256': calibration_sha256, 'stationary_evidence': evidence,
        'checks': checks, 'stereo_ready': stereo_ready,
        'vio_ready': stereo_ready and all(item['ok'] for item in checks),
        'topics': stats, 'camera_models': models, 'matched_stereo_frames': matched,
        'stereo': {'matched_timestamps': matched,
                   'unmatched_left': stats[LEFT_IMAGE]['count'] - matched,
                   'unmatched_right': stats[RIGHT_IMAGE]['count'] - matched},
        'limitations': ['Experiment gates are not general VINS-Fusion requirements.',
                        'Header equality and arrival timing do not prove physical synchronization or expose all restamping.',
                        'Rigid-matrix and intrinsics checks do not validate physical camera-IMU calibration.',
                        'Sensor timestamp provenance and the stationary segment are operator assertions.'],
    }
