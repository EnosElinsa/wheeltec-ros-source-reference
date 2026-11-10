from types import SimpleNamespace as NS

import pytest

from wheeltec_vision_nav.depth_validity import ExactDepthGate, depth_has_valid_returns
from wheeltec_vision_nav.runtime_safety import SafetyConfig, SafetyController


def image(stamp=1_000_000_000, value=1000, bigendian=False):
    return NS(header=NS(stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000),
                        frame_id="camera_depth_optical_frame"),
              width=8, height=8, step=16, encoding="16UC1", is_bigendian=bigendian,
              data=value.to_bytes(2, "big" if bigendian else "little") * 64)


def info(msg):
    return NS(header=msg.header, width=msg.width, height=msg.height,
              k=[400., 0., 4., 0., 400., 4., 0., 0., 1.])


@pytest.mark.parametrize("bigendian", [False, True])
def test_depth_validity_decodes_endianness_and_padded_rows(bigendian):
    msg = image(bigendian=bigendian)
    msg.step += 2
    msg.data = b"".join(msg.data[i:i + 16] + b"\0\0" for i in range(0, len(msg.data), 16))
    assert depth_has_valid_returns(msg)
    assert not depth_has_valid_returns(image(value=0, bigendian=bigendian))


@pytest.mark.parametrize("info_first", [True, False])
def test_gate_preserves_exact_raw_acquisition_and_calibration_objects(info_first):
    gate = ExactDepthGate()
    msg = image()
    calibration = info(msg)
    first, second = ((gate.camera_info, calibration), (gate.image, msg)) if info_first else (
        (gate.image, msg), (gate.camera_info, calibration))
    assert first[0](first[1], 1_020_000_000) is None
    pair = second[0](second[1], 1_030_000_000)
    assert pair[0] is msg and pair[1] is calibration
    assert gate.image(msg, 1_040_000_000) is None


def test_gate_alternating_weak_depth_does_not_publish_or_invalidate_until_deadline():
    gate = ExactDepthGate()
    safety = SafetyController(SafetyConfig())
    published = []
    for index in range(20):
        now = 1_000_000_000 + index * 33_333_333
        msg = image(now, value=1000 if index % 2 == 0 else 0)
        gate.camera_info(info(msg), now)
        pair = gate.image(msg, now)
        if pair is not None:
            published.append(pair)
            safety.note("depth", now, now / 1e9, ros_now_ns=now)
        assert safety.sources["depth"].fresh(now / 1e9, .3)
    assert len(published) == 10
    last = safety.sources["depth"].received
    for index in range(1, 12):
        now = int((last + index * .033) * 1e9)
        msg = image(now, value=0)
        gate.camera_info(info(msg), now)
        assert gate.image(msg, now) is None
    assert not safety.sources["depth"].fresh(last + .301, .3)


def test_gate_never_substitutes_adjacent_frame_camera_info_or_retimes_delayed_depth():
    gate = ExactDepthGate()
    msg = image()
    adjacent = image(1_033_333_333)
    assert gate.camera_info(info(adjacent), 1_050_000_000) is None
    assert gate.image(msg, 1_060_000_000) is None
    assert gate.camera_info(info(msg), 1_300_000_000) is None
    assert gate.last_stamp == -1


@pytest.mark.parametrize("fault", ["frame", "dimensions", "intrinsics", "malformed", "future"])
def test_gate_invalid_images_or_calibration_cannot_generate_clearing_cloud(fault):
    gate = ExactDepthGate()
    msg = image()
    calibration = info(msg)
    if fault == "frame":
        calibration.header = NS(stamp=msg.header.stamp, frame_id="other")
    elif fault == "dimensions":
        calibration.width += 1
    elif fault == "intrinsics":
        calibration.k[0] = float("inf")
    elif fault == "malformed":
        msg.data = msg.data[:-1]
    now = 800_000_000 if fault == "future" else 1_010_000_000
    assert gate.camera_info(calibration, now) is None
    assert gate.image(msg, now) is None
    assert gate.last_stamp == -1


def test_gate_clock_rollback_clears_pending_old_epoch_and_accepts_new_acquisitions():
    gate = ExactDepthGate()
    old = image(10_000_000_000)
    gate.camera_info(info(old), 10_000_000_000)
    assert gate.image(old, 10_000_000_000) is not None
    pending = image(10_030_000_000)
    gate.camera_info(info(pending), 10_030_000_000)
    fresh = image(2_000_000_000)
    assert gate.image(fresh, 2_000_000_000) is None
    assert not gate.infos
    assert gate.camera_info(info(fresh), 2_010_000_000) is not None
    assert gate.image(pending, 2_020_000_000) is None


def test_gate_pending_memory_is_bounded_and_deadlines_are_configurable():
    gate = ExactDepthGate(max_pending=2, timeout_s=.6, future_tolerance_s=.02)
    for index in range(8):
        msg = image(1_000_000_000 + index)
        gate.camera_info(info(msg), 1_200_000_000)
    assert len(gate.infos) == 2
    assert gate.image(msg, 1_500_000_000) is not None
    future = image(1_600_000_000)
    assert gate.image(future, 1_570_000_000) is None
