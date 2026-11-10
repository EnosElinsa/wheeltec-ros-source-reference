import importlib.util
import json
import sys
from itertools import permutations
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest

from wheeltec_vision_nav.ekf_inputs_logic import (
    VX, VYAW, GyroBias, MessageGate, VoQuality, floor_twist_covariance,
    imu_error, odometry_error, quality_error, quality_twist_covariance,
)


def test_bias_is_learned_only_after_the_wheels_settle():
    bias = GyroBias(settle_s=0.5)
    bias.note_wheel(0.0, 0.0, 0.0)
    assert bias.correct(0.2, 0.0008) == 0.0008  # not settled yet
    for i in range(100):
        t = 0.6 + 0.05 * i
        bias.note_wheel(t, 0.0, 0.0)
        bias.correct(t, 0.0008)
    assert abs(bias.bias - 0.0008) < 1e-9
    assert abs(bias.correct(6.0, 0.0008)) < 1e-9


def test_bias_is_frozen_while_driving():
    bias = GyroBias(settle_s=0.0)
    bias.note_wheel(0.0, 0.0, 0.0)
    bias.correct(0.0, 0.0008)
    bias.note_wheel(1.0, 0.25, 0.3)
    corrected = bias.correct(1.0, 0.3008)
    assert abs(corrected - 0.3) < 1e-9
    assert abs(bias.bias - 0.0008) < 1e-12


def test_vo_twist_covariance_gets_a_floor():
    cov = [0.0] * 36
    cov[VX] = 7.75e-7
    cov[VYAW] = 3e-8
    out = floor_twist_covariance(cov, {VX: 2.5e-3, VYAW: 1e-3})
    assert out[VX] == 2.5e-3 and out[VYAW] == 1e-3
    lost = [9999.0] * 36
    assert floor_twist_covariance(lost, {VX: 2.5e-3})[VX] == 9999.0
    nan = [float("nan")] * 36
    with pytest.raises(ValueError):
        floor_twist_covariance(nan, {VX: 2.5e-3})


def test_one_still_wheel_sample_cannot_teach_bias_after_wheel_stream_stalls():
    bias = GyroBias()
    bias.note_wheel(1., 0., 0., receipt_time=10.)
    for i in range(1, 20):
        bias.correct(1. + i * .1, .0008, receipt_time=10. + i * .1)
    assert bias.samples == 0
    assert bias.bias == 0.
    assert not bias.is_still(3., receipt_time=12.)


def test_fresh_zero_wheels_do_not_allow_a_rotating_gyro_to_be_learned():
    bias = GyroBias()
    for i in range(30):
        stamp = 1. + i * .05
        bias.note_wheel(stamp, 0., 0.)
        assert bias.correct(stamp, .2) == .2
    assert bias.samples == 0


def test_fluctuating_gyro_does_not_count_as_stationary():
    bias = GyroBias()
    for i in range(30):
        stamp = 1. + i * .05
        bias.note_wheel(stamp, 0., 0.)
        bias.correct(stamp, -.015 if i % 2 else .015)
    assert bias.samples == 0


@pytest.mark.parametrize("bad_stamp", [1., .5])
def test_duplicate_or_backwards_wheel_clock_restarts_settling(bad_stamp):
    bias = GyroBias()
    bias.note_wheel(1., 0., 0., receipt_time=10.)
    assert not bias.note_wheel(bad_stamp, 0., 0., receipt_time=10.1)
    assert not bias.is_still(2., receipt_time=10.2)
    bias.note_wheel(bad_stamp + .1, 0., 0., receipt_time=10.2)
    assert not bias.is_still(2., receipt_time=10.3)


def test_gyro_clock_jump_and_invalid_values_reset_stationarity_without_erasing_bias():
    bias = GyroBias(settle_s=0.)
    bias.note_wheel(1., 0., 0., receipt_time=10.)
    bias.correct(1., .001, receipt_time=10.)
    assert bias.samples == 1
    bias.correct(.5, .002, receipt_time=10.1)
    assert bias.samples == 1 and bias.bias == .001
    assert bias._still_since is None
    assert not bias.note_wheel(.6, float("nan"), 0., receipt_time=10.2)
    assert bias.note_wheel(.7, 0., 0., receipt_time=10.3)


def test_monotonic_receipts_detect_a_stall_when_ros_clock_is_frozen():
    bias = GyroBias(settle_s=0.)
    bias.note_wheel(1., 0., 0., receipt_time=10.)
    bias.correct(1.01, .001, receipt_time=10.01)
    assert bias.samples == 1
    bias.correct(1.02, .015, receipt_time=11.)
    assert bias.samples == 1 and bias.bias == .001


def _odom(stamp_ns=1_000_000_000):
    vector = lambda: NS(x=0., y=0., z=0.)
    return NS(header=NS(stamp=NS(sec=stamp_ns // 1_000_000_000,
                                nanosec=stamp_ns % 1_000_000_000), frame_id="odom"),
              pose=NS(pose=NS(position=vector(), orientation=NS(x=0., y=0., z=0., w=1.)),
                      covariance=[1e-5] * 36),
              twist=NS(twist=NS(linear=vector(), angular=vector()), covariance=[1e-5] * 36))


def _imu():
    return NS(header=_odom().header, angular_velocity=NS(x=0., y=0., z=.001),
              angular_velocity_covariance=[0.] * 9,
              orientation=NS(x=0., y=0., z=0., w=1.), orientation_covariance=[0.] * 9)


@pytest.mark.parametrize("target", ["position", "quaternion", "twist", "pose_cov", "twist_cov"])
def test_nonfinite_odometry_fields_are_rejected(target):
    msg = _odom()
    if target == "position":
        msg.pose.pose.position.z = float("nan")
    elif target == "quaternion":
        msg.pose.pose.orientation.x = float("inf")
    elif target == "twist":
        msg.twist.twist.linear.y = float("nan")
    elif target == "pose_cov":
        msg.pose.covariance[10] = float("inf")
    else:
        msg.twist.covariance[3] = float("nan")
    assert odometry_error(msg) is not None
    assert odometry_error(_odom()) is None


def test_lost_negative_covariance_and_scaled_quaternion_cannot_be_repaired():
    for field, value in [("pose", 9999.), ("twist", -1.)]:
        msg = _odom()
        getattr(msg, field).covariance[0] = value
        assert odometry_error(msg) == "invalid_covariance"
    msg = _odom()
    msg.pose.pose.orientation.w = 2.
    assert odometry_error(msg) == "invalid_quaternion"


@pytest.mark.parametrize("kind", ["asymmetric", "indefinite"])
@pytest.mark.parametrize("field", ["pose", "twist"])
def test_invalid_planar_covariance_is_rejected_before_any_floor(field, kind):
    msg = _odom()
    covariance = getattr(msg, field).covariance
    covariance[1] = 2e-5
    covariance[6] = 0. if kind == "asymmetric" else 2e-5
    assert odometry_error(msg) == "invalid_covariance"
    with pytest.raises(ValueError):
        floor_twist_covariance(covariance, {VX: 2.5e-3})


def test_valid_singular_correlated_planar_covariance_is_accepted():
    msg = _odom()
    assert odometry_error(msg) is None
    assert floor_twist_covariance(msg.twist.covariance, {VX: .0025})[VX] == .0025


def test_planar_wheel_driver_can_mark_unused_axes_with_large_finite_variance():
    msg = _odom()
    for covariance in (msg.pose.covariance, msg.twist.covariance):
        for index in (14, 21, 28):
            covariance[index] = 1e6
        covariance[35] = 1e3
    assert odometry_error(msg) is None


def test_unused_invalid_imu_orientation_is_allowed_only_with_ros_absent_marker():
    msg = _imu()
    msg.orientation.w = float("nan")
    assert imu_error(msg) == "invalid_imu_orientation"
    msg.orientation_covariance[0] = -1.
    assert imu_error(msg) is None
    msg.angular_velocity_covariance[8] = float("nan")
    assert imu_error(msg) == "invalid_gyro_covariance"


def test_gate_rejects_old_duplicate_future_stamps_and_recovers_after_clock_back():
    gate = MessageGate()
    assert gate.check(1_000_000_000, 1_000_000_000) is None
    assert gate.check(1_000_000_000, 1_100_000_000) == "duplicate_or_out_of_order"
    assert gate.check(1_100_000_000, 2_000_000_000) == "old_stamp"
    assert gate.check(3_000_000_000, 2_000_000_000) == "future_stamp"
    assert gate.check(500_000_000, 500_000_000) == "clock_backwards"
    assert gate.check(600_000_000, 600_000_000) is None


def test_vo_quality_requires_recent_near_stamp_enough_inliers_and_not_lost():
    quality = VoQuality(1_000_000_000, 10., False, 20)
    assert quality_error(quality, 1_030_000_000, 10.1) is None
    assert quality_error(quality, 1_060_000_000, 10.1) == "quality_stamp_mismatch"
    assert quality_error(quality, 1_000_000_000, 10.6) == "stale_quality"
    assert quality_error(VoQuality(1, 10., True, 100), 1, 10.1) == "vo_lost"
    assert quality_error(VoQuality(1, 10., False, 9), 1, 10.1) == "low_inliers"
    assert quality_error(None, 1, 10.1) == "missing_quality"


def test_quality_floor_and_wheel_prior_inflation_are_conservative():
    covariance = [0.] * 36
    good = quality_twist_covariance(covariance, {VX: .0025}, VoQuality(1, 1., False, 100),
                                    correlation_inflation=1.)
    weak = quality_twist_covariance(covariance, {VX: .0025}, VoQuality(1, 1., False, 10))
    assert good[VX] == .0025
    assert weak[VX] == pytest.approx(.025)


@pytest.fixture
def runtime_module(monkeypatch):
    """Run real callbacks with small ROS transport substitutes, without a ROS installation."""
    class FakeMessage:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class FakeNode:
        def __init__(self, name):
            self.now_ns = 1_000_000_000

        def declare_parameter(self, name, value):
            return NS(value=value)

        def create_subscription(self, *args):
            return None

        def create_publisher(self, *args):
            messages = []
            return NS(messages=messages, publish=messages.append)

        def create_timer(self, *args):
            return None

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=self.now_ns))

    names = ("rclpy", "rclpy.node", "rclpy.qos", "nav_msgs", "nav_msgs.msg", "sensor_msgs",
             "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "rtabmap_msgs", "rtabmap_msgs.msg",
             "realsense2_camera_msgs", "realsense2_camera_msgs.msg")
    modules = {name: ModuleType(name) for name in names}
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    for name in ("DurabilityPolicy", "HistoryPolicy", "ReliabilityPolicy"):
        setattr(modules["rclpy.qos"], name,
                NS(BEST_EFFORT=1, KEEP_LAST=1, VOLATILE=1))
    for package, messages in (("nav_msgs.msg", ("Odometry",)),
                              ("sensor_msgs.msg", ("Imu", "Image", "CameraInfo")),
                              ("std_msgs.msg", ("String",)), ("rtabmap_msgs.msg", ("OdomInfo",)),
                              ("realsense2_camera_msgs.msg", ("Metadata",))):
        for name in messages:
            setattr(modules[package], name, FakeMessage)

    def load(name):
        path = Path(__file__).parents[1] / "wheeltec_vision_nav" / f"{name}.py"
        spec = importlib.util.spec_from_file_location(f"runtime_{name}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return load


def test_delayed_odom_info_releases_pending_vo_and_publishes_evidence(runtime_module, monkeypatch):
    module = runtime_module("ekf_inputs")
    now = [10.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = module.EkfInputs()
    msg = _odom()
    node._on_vo(msg)
    assert len(node._pending) == 1 and not node.vo_pub.messages
    now[0] += .02
    node._on_quality(NS(header=msg.header, lost=False, inliers=30))
    assert len(node.vo_pub.messages) == 1 and not node._pending
    assert node.vo_pub.messages[0].twist.covariance[VX] > .0025
    node._publish_status()
    status = json.loads(node.status_pub.messages[-1].data)
    assert status["accepted"]["vo"] == 1
    assert status["quality"]["inliers"] == 30
    assert status["last_reason"]["vo"] == "valid"


def test_vo_node_drops_lost_invalid_old_duplicate_and_missing_quality(runtime_module, monkeypatch):
    module = runtime_module("ekf_inputs")
    now = [10.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = module.EkfInputs()
    msg = _odom()
    node._on_quality(NS(header=msg.header, lost=True, inliers=100))
    node._on_vo(msg)
    assert node._last_reason["vo"] == "vo_lost"
    node._on_vo(msg)
    assert node._last_reason["vo"] == "duplicate_or_out_of_order"
    invalid = _odom(1_010_000_000)
    invalid.twist.covariance[0] = float("nan")
    node._on_vo(invalid)
    assert node._last_reason["vo"] == "invalid_covariance"
    node.now_ns = 2_000_000_000
    node._on_vo(_odom(1_010_000_000))
    assert node._last_reason["vo"] == "old_stamp"
    node._on_vo(_odom(2_000_000_000))
    now[0] += .2
    node._drain_pending()
    assert node._last_reason["vo"] == "pending_timeout"
    assert not node.vo_pub.messages


def test_late_matching_quality_cannot_bypass_the_pending_deadline(runtime_module, monkeypatch):
    module = runtime_module("ekf_inputs")
    now = [10.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = module.EkfInputs()
    msg = _odom()
    node._on_vo(msg)
    now[0] += .14
    node.now_ns += 140_000_000
    node._drain_pending()
    assert len(node._pending) == 1
    now[0] += .02
    node.now_ns += 20_000_000
    node._on_quality(NS(header=msg.header, lost=False, inliers=100))
    assert not node.vo_pub.messages and not node._pending
    assert node._last_reason["vo"] == "pending_timeout"
    assert node._rejections["vo:pending_timeout"] == 1


@pytest.mark.parametrize("order", list(permutations(("ir_a", "depth_a", "ir_b", "depth_b"))))
@pytest.mark.parametrize("depth_b_pixel,expected_pairs", [(40, 0), (99, 1)])
def test_image_classification_respects_complete_evidence_in_any_callback_order(
    runtime_module, monkeypatch, order, depth_b_pixel, expected_pairs,
):
    from test_emitter_pair import _image, _info
    module = runtime_module("emitter_pair")
    monkeypatch.setattr(module, "ir_dot_density", lambda image: float(image[0, 0]))
    monkeypatch.setattr(module, "depth_fill_ratio", lambda image: float(image[0, 0]))
    node = module.EmitterPair()
    images = {}
    for key, stamp_ns, pixel in (("ir_a", 1_000_000_000, 10),
                                  ("depth_a", 1_000_000_000, 90),
                                  ("ir_b", 1_033_333_333, 50),
                                  ("depth_b", 1_033_333_333, depth_b_pixel)):
        depth = key.startswith("depth")
        image = _image(depth=depth)
        image.header.stamp = NS(sec=1, nanosec=stamp_ns - 1_000_000_000)
        image.data = pixel.to_bytes(2 if depth else 1, "little") * 8
        images[key] = image
        (node._on_depth_info if depth else node._on_ir_info)(_info(image))
    for index, key in enumerate(order):
        (node._on_depth if key.startswith("depth") else node._on_ir)(images[key])
        assert len(node.ir_pub.messages) == (expected_pairs if index == 3 else 0)
    if not expected_pairs:
        assert node._classifier.label(1.) is None
        assert node._classifier.label(1.033333333) is None
    else:
        provenance = json.loads(node.provenance_pub.messages[-1].data)
        assert provenance["ir"]["source"] == "image"
        assert provenance["depth"]["source"] == "image"


def test_metadata_arriving_after_images_publishes_exact_provenance(runtime_module):
    from test_emitter_pair import _image, _info
    module = runtime_module("emitter_pair")
    node = module.EmitterPair()
    stamp = 1_800_000_000_000_000_001
    ir = _image()
    depth = _image(depth=True)
    for image, ns in ((ir, stamp), (depth, stamp + 33_333_333)):
        image.header.stamp = NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
    node._on_ir_info(_info(ir))
    node._on_depth_info(_info(depth))
    node._on_ir(ir)
    node._on_depth(depth)
    assert not node.ir_pub.messages
    node._on_ir_metadata(NS(header=ir.header, json_data='{"frame_emitter_mode":0}'))
    node._on_depth_metadata(NS(header=depth.header, json_data='{"frame_emitter_mode":1}'))
    assert len(node.ir_pub.messages) == 1
    provenance = json.loads(node.provenance_pub.messages[-1].data)
    assert provenance["ir_stamp_ns"] == stamp
    assert provenance["delta_ns"] == 33_333_333
    assert provenance["ir"]["source"] == "metadata"
    assert depth.header.stamp.nanosec == 33_333_334
    assert node.depth_pub.messages[0].header.stamp.nanosec == 1
    node._on_ir(ir)
    assert node._drops["duplicate_or_old_frame"] == 1
    node._publish_status()
    assert json.loads(node.status_pub.messages[-1].data)["drop_counts"]


def test_malformed_images_and_mismatched_calibration_have_drop_counters(runtime_module):
    from test_emitter_pair import _image, _info
    module = runtime_module("emitter_pair")
    node = module.EmitterPair()
    image = _image()
    image.header.stamp = NS(sec=1, nanosec=0)
    info = _info(image)
    info.width += 1
    node._on_ir_info(info)
    node._on_ir(image)
    assert node._drops["camera_info_dimensions"] == 1
    image.data = b"short"
    node._on_ir(image)
    assert node._drops["invalid_ir_image"] == 1
    assert not node.ir_pub.messages


@pytest.mark.parametrize("images_before_info", [True, False])
def test_out_of_order_matching_camera_info_replaces_neither_older_nor_future_calibration(
    runtime_module, images_before_info,
):
    from test_emitter_pair import _image, _info
    module = runtime_module("emitter_pair")
    node = module.EmitterPair()
    ir, depth = _image(), _image(depth=True)
    depth.header.stamp.nanosec = 33_333_333
    node._on_ir_metadata(NS(header=ir.header, json_data='{"frame_emitter_mode":0}'))
    node._on_depth_metadata(NS(header=depth.header, json_data='{"frame_emitter_mode":1}'))

    def mismatched_info(nanosec):
        for image, callback in ((ir, node._on_ir_info), (depth, node._on_depth_info)):
            info = _info(image)
            stamp_ns = 1_000_000_000 + nanosec
            info.header.stamp.sec = stamp_ns // 1_000_000_000
            info.header.stamp.nanosec = stamp_ns % 1_000_000_000
            info.k[0] = info.p[0] = 500.
            callback(info)

    if not images_before_info:
        mismatched_info(-33_333_333)
    node._on_ir(ir)
    node._on_depth(depth)
    # A later exposure can have the same dimensions/frame but new intrinsics.
    mismatched_info(66_666_666)
    assert not node.ir_pub.messages
    matching_ir, matching_depth = _info(ir), _info(depth)
    matching_ir.k[0] = matching_ir.p[0] = 125.
    matching_depth.k[0] = matching_depth.p[0] = 200.
    node._on_ir_info(matching_ir)  # out of order relative to the future info
    assert not node.ir_pub.messages
    node._on_depth_info(matching_depth)
    assert len(node.ir_pub.messages) == 1
    assert node.ir_info_pub.messages[0].k[0] == 125.
    assert node.depth_info_pub.messages[0].k[0] == 200.
    assert node.depth_info_pub.messages[0].header.stamp.nanosec == 0
    assert matching_depth.header.stamp.nanosec == 33_333_333


def test_camera_info_caches_are_bounded_and_do_not_reuse_static_zero_stamp(runtime_module):
    from test_emitter_pair import _image, _info
    module = runtime_module("emitter_pair")
    node = module.EmitterPair()
    info = _info(_image())
    info.header.stamp.sec = 0
    node._on_ir_info(info)
    assert not node._ir_info
    assert node._drops["invalid_camera_info_stamp"] == 1
    for index in range(node._info_capacity + 1):
        info = _info(_image())
        info.header.stamp.nanosec = index
        node._on_ir_info(info)
    assert len(node._ir_info) == node._info_capacity
    assert 1_000_000_000 not in node._ir_info
