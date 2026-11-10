#!/usr/bin/env python3
"""Validate gyro/VO evidence before publishing the measurements used by the EKF."""
from __future__ import annotations

import json
import math
import time
from collections import Counter, deque
from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from wheeltec_vision_nav.ekf_inputs_logic import (
    VX, VY, VYAW, GyroBias, MessageGate, VoQuality, imu_error, odometry_error,
    quality_error, quality_twist_covariance,
)

try:
    from rtabmap_msgs.msg import OdomInfo
except ImportError:  # pragma: no cover - fail closed when diagnostics are unavailable
    OdomInfo = None


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class EkfInputs(Node):
    def __init__(self) -> None:
        super().__init__("vision_ekf_inputs")
        self.gyro_variance = float(self.declare_parameter("gyro_yaw_rate_variance", 1e-5).value)
        vo_vx = float(self.declare_parameter("vo_vx_variance_min", 2.5e-3).value)
        vo_vyaw = float(self.declare_parameter("vo_vyaw_variance_min", 1e-3).value)
        self.vo_floors = {VX: vo_vx, VY: vo_vx, VYAW: vo_vyaw}
        self.min_inliers = int(self.declare_parameter("min_inliers", 10).value)
        self.quality_timeout = float(self.declare_parameter("quality_timeout_s", 0.5).value)
        self.quality_slop = float(self.declare_parameter("quality_stamp_slop_s", 0.05).value)
        self.pending_timeout = float(self.declare_parameter("pending_timeout_s", 0.15).value)
        self.pending_size = max(1, int(self.declare_parameter("pending_size", 8).value))
        self.quality_reference = int(self.declare_parameter("vo_quality_reference_inliers", 50).value)
        self.wheel_prior_used = bool(self.declare_parameter("wheel_prior_used", True).value)
        self.correlation_inflation = float(self.declare_parameter("vo_correlation_inflation", 2.0).value)
        wheel_timeout = float(self.declare_parameter("wheel_timeout_s", 0.3).value)
        settle = float(self.declare_parameter("bias_settle_s", 0.5).value)
        max_gyro = float(self.declare_parameter("bias_max_stationary_gyro", 0.03).value)
        gyro_range = float(self.declare_parameter("bias_max_gyro_range", 0.01).value)
        max_age = float(self.declare_parameter("message_timeout_s", 0.5).value)
        status_topic = str(self.declare_parameter("status_topic", "/vision/ekf_inputs/status").value)
        positive = (self.gyro_variance, vo_vx, vo_vyaw, self.quality_timeout,
                    self.pending_timeout, wheel_timeout, max_gyro, gyro_range, max_age)
        if (any(not math.isfinite(v) or v <= 0 for v in positive)
                or not math.isfinite(settle) or settle < 0
                or not math.isfinite(self.quality_slop) or self.quality_slop < 0
                or not math.isfinite(self.correlation_inflation) or self.correlation_inflation < 1
                or self.min_inliers < 1 or self.quality_reference < 1):
            raise ValueError("EKF input limits must be finite and conservative")
        self.bias = GyroBias(settle_s=settle, wheel_timeout_s=wheel_timeout,
                             max_stationary_gyro=max_gyro, max_gyro_range=gyro_range)
        self._gates = {name: MessageGate(max_age_s=max_age)
                       for name in ("wheel", "imu", "vo", "quality")}
        self._pending = deque()
        self._quality = deque(maxlen=32)
        self._rejections = Counter()
        self._accepted = Counter()
        self._last_reason = {}
        self._last_receipt = {}
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, sensor_qos)
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, sensor_qos)
        self.create_subscription(Odometry, "/odom_vo", self._on_vo, sensor_qos)
        if OdomInfo is not None:
            self.create_subscription(OdomInfo, "/odom_info_lite", self._on_quality, sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "/vision/imu", 20)
        self.vo_pub = self.create_publisher(Odometry, "/vision/odom_vo", 20)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(0.05, self._drain_pending)
        self.create_timer(0.5, self._publish_status)

    def _reject(self, stream: str, reason: str) -> None:
        self._rejections[f"{stream}:{reason}"] += 1
        self._last_reason[stream] = reason
        if stream in ("wheel", "imu"):
            self.bias.reset_stationarity()
        if reason == "clock_backwards":
            self._pending.clear()
            self._quality.clear()

    def _accept(self, stream: str, receipt: float) -> None:
        self._accepted[stream] += 1
        self._last_reason[stream] = "valid"
        self._last_receipt[stream] = receipt

    def _stamp_error(self, stream, msg):
        return self._gates[stream].check(_stamp_ns(msg.header.stamp),
                                         self.get_clock().now().nanoseconds)

    def _on_wheel(self, msg: Odometry) -> None:
        reason = odometry_error(msg) or self._stamp_error("wheel", msg)
        if reason:
            self._reject("wheel", reason)
            return
        receipt = time.monotonic()
        twist = msg.twist.twist
        valid = self.bias.note_wheel(_stamp_ns(msg.header.stamp) * 1e-9,
                                     max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.linear.z)),
                                     max(abs(twist.angular.x), abs(twist.angular.y), abs(twist.angular.z)),
                                     receipt_time=receipt)
        if valid:
            self._accept("wheel", receipt)
        else:
            self._reject("wheel", "bias_clock_or_value")

    def _on_imu(self, msg: Imu) -> None:
        reason = imu_error(msg) or self._stamp_error("imu", msg)
        if reason:
            self._reject("imu", reason)
            return
        receipt = time.monotonic()
        out = deepcopy(msg)
        out.angular_velocity.z = self.bias.correct(_stamp_ns(msg.header.stamp) * 1e-9,
                                                    msg.angular_velocity.z, receipt_time=receipt)
        covariance = list(out.angular_velocity_covariance)
        covariance[8] = max(covariance[8], self.gyro_variance)
        out.angular_velocity_covariance = covariance
        self.imu_pub.publish(out)
        self._accept("imu", receipt)

    def _on_quality(self, msg) -> None:
        reason = self._stamp_error("quality", msg)
        if reason:
            self._reject("quality", reason)
            return
        receipt = time.monotonic()
        try:
            quality = VoQuality(_stamp_ns(msg.header.stamp), receipt, bool(msg.lost), int(msg.inliers))
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._reject("quality", "invalid_quality")
            return
        self._quality.append(quality)
        self._accept("quality", receipt)
        self._drain_pending()

    def _quality_for(self, stamp_ns, receipt):
        nearby = [q for q in self._quality
                  if abs(q.stamp_ns - stamp_ns) <= self.quality_slop * 1e9]
        quality = min(nearby, key=lambda q: abs(q.stamp_ns - stamp_ns)) if nearby else None
        reason = quality_error(quality, stamp_ns, receipt, min_inliers=self.min_inliers,
                               timeout_s=self.quality_timeout, stamp_slop_s=self.quality_slop)
        return quality, reason

    def _on_vo(self, msg: Odometry) -> None:
        reason = odometry_error(msg) or self._stamp_error("vo", msg)
        if reason:
            self._reject("vo", reason)
            return
        if len(self._pending) >= self.pending_size:
            self._pending.popleft()
            self._reject("vo", "pending_overflow")
        self._pending.append((msg, time.monotonic()))
        self._drain_pending()

    def _drain_pending(self) -> None:
        receipt = time.monotonic()
        waiting = deque()
        now_ns = self.get_clock().now().nanoseconds
        while self._pending:
            msg, received = self._pending.popleft()
            if not 0 <= receipt - received <= self.pending_timeout:
                self._reject("vo", "pending_timeout")
                continue
            stamp_ns = _stamp_ns(msg.header.stamp)
            age = (now_ns - stamp_ns) * 1e-9
            if not -0.05 <= age <= self._gates["vo"].max_age_s:
                self._reject("vo", "expired_pending_stamp")
                continue
            quality, reason = self._quality_for(stamp_ns, receipt)
            if reason in ("missing_quality", "stale_quality", "quality_stamp_mismatch"):
                waiting.append((msg, received))
                self._last_reason["vo"] = f"waiting_{reason}"
                continue
            if reason:
                self._reject("vo", reason)
                continue
            # Keep publication ordered if the older exposure awaits delayed quality.
            if waiting:
                waiting.append((msg, received))
                continue
            out = deepcopy(msg)
            out.twist.covariance = quality_twist_covariance(
                msg.twist.covariance, self.vo_floors, quality,
                reference_inliers=self.quality_reference,
                correlation_inflation=self.correlation_inflation if self.wheel_prior_used else 1.0)
            self.vo_pub.publish(out)
            self._accept("vo", receipt)
        self._pending = waiting

    def _publish_status(self) -> None:
        now = time.monotonic()
        quality = self._quality[-1] if self._quality else None
        status = {
            "accepted": dict(self._accepted), "rejected": dict(self._rejections),
            "last_reason": self._last_reason,
            "receipt_age_s": {name: now - receipt for name, receipt in self._last_receipt.items()},
            "pending_vo": len(self._pending), "quality_available": OdomInfo is not None,
            "quality": None if quality is None else {
                "stamp_ns": quality.stamp_ns, "age_s": now - quality.receipt_time,
                "lost": quality.lost, "inliers": quality.inliers},
            "bias": self.bias.bias, "bias_samples": self.bias.samples,
            "wheels_still": self.bias.is_still(0.0, receipt_time=now),
            "wheel_prior_used": self.wheel_prior_used,
        }
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main() -> None:
    rclpy.init()
    node = EkfInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
