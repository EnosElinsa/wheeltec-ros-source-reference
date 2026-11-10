"""Prepare gyro and visual-odometry samples for the EKF. No ROS imports.

robot_localization takes each measurement's covariance from the message. Two
inputs need correcting before they reach it:

- The chassis gyro has a constant yaw-rate bias (about 8e-4 rad/s on this
  robot) and reports a variance of 1e-6, so the EKF integrates the bias into a
  steady heading drift. The bias is estimated while the wheels report
  standstill and subtracted.
- rgbd_odometry reports twist variances around 1e-7, far smaller than its real
  error, which would let a single mis-registered frame override wheel speed
  and gyro. A lower bound is applied.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from wheeltec_vision_nav.runtime_safety import covariance_valid

STILL_LINEAR = 1e-3
STILL_ANGULAR = 1e-3


@dataclass
class GyroBias:
    """Yaw-rate bias estimated from gyro samples taken while the wheels stand still."""

    settle_s: float = 0.5
    alpha: float = 0.02
    bias: float = 0.0
    samples: int = 0
    _still_since: float | None = None
    wheel_timeout_s: float = 0.3
    max_stationary_gyro: float = 0.03
    max_gyro_range: float = 0.01
    _last_wheel_stamp: float | None = None
    _last_wheel_receipt: float | None = None
    _last_gyro_stamp: float | None = None
    _last_gyro_receipt: float | None = None
    _gyro_window: deque = field(default_factory=lambda: deque(maxlen=512))

    def reset_stationarity(self) -> None:
        self._still_since = None
        self._gyro_window.clear()

    def note_wheel(self, stamp: float, vx: float, vyaw: float,
                   receipt_time: float | None = None) -> bool:
        receipt = stamp if receipt_time is None else receipt_time
        if not all(math.isfinite(v) for v in (stamp, vx, vyaw, receipt)):
            self.reset_stationarity()
            self._last_wheel_receipt = None
            self._last_wheel_stamp = None
            return False
        if self._last_wheel_stamp is not None and (
            stamp <= self._last_wheel_stamp or receipt <= self._last_wheel_receipt
        ):
            self.reset_stationarity()
            self._last_wheel_stamp, self._last_wheel_receipt = stamp, receipt
            return False
        if (self._last_wheel_receipt is not None
                and receipt - self._last_wheel_receipt > self.wheel_timeout_s):
            self.reset_stationarity()
        self._last_wheel_stamp, self._last_wheel_receipt = stamp, receipt
        if abs(vx) < STILL_LINEAR and abs(vyaw) < STILL_ANGULAR:
            if self._still_since is None:
                self._still_since = receipt
        else:
            self.reset_stationarity()
        return True

    def is_still(self, stamp: float, receipt_time: float | None = None) -> bool:
        receipt = stamp if receipt_time is None else receipt_time
        if (not math.isfinite(receipt) or self._last_wheel_receipt is None
                or not 0 <= receipt - self._last_wheel_receipt <= self.wheel_timeout_s):
            self.reset_stationarity()
            return False
        return self._still_since is not None and receipt - self._still_since >= self.settle_s

    def correct(self, stamp: float, gyro_z: float, receipt_time: float | None = None) -> float:
        """Return the bias-corrected yaw rate; learn the bias while standing still."""
        receipt = stamp if receipt_time is None else receipt_time
        if not all(math.isfinite(v) for v in (stamp, gyro_z, receipt)):
            self.reset_stationarity()
            return gyro_z - self.bias
        increasing = self._last_gyro_stamp is None or (
            stamp > self._last_gyro_stamp and receipt > self._last_gyro_receipt)
        self._last_gyro_stamp, self._last_gyro_receipt = stamp, receipt
        if not increasing:
            self.reset_stationarity()
            return gyro_z - self.bias
        if not self.is_still(stamp, receipt) and self._still_since is None:
            return gyro_z - self.bias
        if abs(gyro_z) > self.max_stationary_gyro:
            self.reset_stationarity()
            return gyro_z - self.bias
        self._gyro_window.append((receipt, gyro_z))
        while len(self._gyro_window) > 1 and receipt - self._gyro_window[0][0] > self.settle_s:
            self._gyro_window.popleft()
        rates = [value for _, value in self._gyro_window]
        if max(rates) - min(rates) > self.max_gyro_range:
            self.reset_stationarity()
            return gyro_z - self.bias
        if self.is_still(stamp, receipt):
            if self.samples == 0:
                self.bias = gyro_z
            else:
                self.bias += self.alpha * (gyro_z - self.bias)
            self.samples += 1
        return gyro_z - self.bias


# Row-major 6x6 twist covariance diagonal indices.
VX, VY, VYAW = 0, 7, 35


def floor_twist_covariance(covariance, floors: dict[int, float]) -> list[float]:
    """Raise selected diagonal entries to at least the given variance."""
    out = list(covariance)
    if not covariance_is_valid(out):
        raise ValueError("invalid twist covariance cannot be repaired")
    for index, floor in floors.items():
        if not math.isfinite(floor) or floor <= 0:
            raise ValueError("covariance floor must be finite and positive")
        out[index] = max(out[index], floor)
    return out


def covariance_is_valid(covariance, *, reject_lost: bool = False) -> bool:
    try:
        return (len(covariance) == 36 and all(math.isfinite(v) for v in covariance)
                and all(covariance[i] >= 0 and (not reject_lost or covariance[i] < 9000)
                        for i in (0, 7, 14, 21, 28, 35))
                and covariance_valid(covariance))
    except (TypeError, ValueError):
        return False


def quaternion_is_valid(quaternion, tolerance: float = 0.05) -> bool:
    try:
        return (len(quaternion) == 4 and all(math.isfinite(v) for v in quaternion)
                and abs(math.sqrt(sum(v * v for v in quaternion)) - 1) <= tolerance)
    except (TypeError, ValueError, OverflowError):
        return False


def odometry_error(msg) -> str | None:
    """Validate the complete pose/twist and covariance before any flooring."""
    try:
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, p.z, t.linear.x, t.linear.y, t.linear.z,
                  t.angular.x, t.angular.y, t.angular.z)
        if not all(math.isfinite(v) for v in values):
            return "nonfinite_pose_or_twist"
        if not quaternion_is_valid((q.x, q.y, q.z, q.w)):
            return "invalid_quaternion"
        if not (covariance_is_valid(msg.pose.covariance)
                and covariance_is_valid(msg.twist.covariance)):
            return "invalid_covariance"
        # Planar drivers legitimately set unused z/roll/pitch variances to 1e6.
        # Loss sentinels on the planar components used by this EKF are rejected.
        if any(covariance[i] >= 9000 for covariance in (msg.pose.covariance, msg.twist.covariance)
               for i in (VX, VY, VYAW)):
            return "invalid_covariance"
    except (AttributeError, TypeError, ValueError):
        return "malformed_odometry"
    return None


def imu_error(msg) -> str | None:
    """Only angular velocity z and its covariance are used by this EKF."""
    try:
        covariance = msg.angular_velocity_covariance
        if not all(math.isfinite(v) for v in (
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            return "nonfinite_gyro"
        if (len(covariance) != 9 or not all(math.isfinite(v) for v in covariance)
                or any(covariance[i] < 0 for i in (0, 4, 8))):
            return "invalid_gyro_covariance"
        # ROS permits an absent orientation with covariance[0] == -1.
        if msg.orientation_covariance[0] != -1:
            q = msg.orientation
            if not quaternion_is_valid((q.x, q.y, q.z, q.w)):
                return "invalid_imu_orientation"
    except (AttributeError, IndexError, TypeError, ValueError):
        return "malformed_imu"
    return None


@dataclass
class MessageGate:
    """ROS stamp age and stream order, with a reset on backwards ROS time."""

    max_age_s: float = 0.5
    future_slop_s: float = 0.05
    last_stamp_ns: int | None = None
    last_now_ns: int | None = None

    def check(self, stamp_ns: int, now_ns: int) -> str | None:
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.last_stamp_ns = None
            self.last_now_ns = now_ns
            return "clock_backwards"
        self.last_now_ns = now_ns
        if stamp_ns <= 0:
            return "invalid_stamp"
        age_ns = now_ns - stamp_ns
        if age_ns > self.max_age_s * 1e9:
            return "old_stamp"
        if age_ns < -self.future_slop_s * 1e9:
            return "future_stamp"
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return "duplicate_or_out_of_order"
        self.last_stamp_ns = stamp_ns
        return None


@dataclass(frozen=True)
class VoQuality:
    stamp_ns: int
    receipt_time: float
    lost: bool
    inliers: int


def quality_error(quality: VoQuality | None, stamp_ns: int, now: float,
                  *, min_inliers: int = 10, timeout_s: float = 0.5,
                  stamp_slop_s: float = 0.05) -> str | None:
    if quality is None:
        return "missing_quality"
    if not 0 <= now - quality.receipt_time <= timeout_s:
        return "stale_quality"
    if abs(stamp_ns - quality.stamp_ns) > stamp_slop_s * 1e9:
        return "quality_stamp_mismatch"
    if quality.lost:
        return "vo_lost"
    if quality.inliers < min_inliers:
        return "low_inliers"
    return None


def quality_twist_covariance(covariance, floors, quality: VoQuality, *,
                             reference_inliers: int = 50, correlation_inflation: float = 2.0):
    scale = max(1.0, reference_inliers / max(1, quality.inliers))
    if not math.isfinite(correlation_inflation) or correlation_inflation < 1:
        raise ValueError("correlation inflation must be at least one")
    out = floor_twist_covariance(covariance, {i: value * scale for i, value in floors.items()})
    # Scaling the complete matrix preserves covariance correlations.
    return [v * correlation_inflation for v in out]
