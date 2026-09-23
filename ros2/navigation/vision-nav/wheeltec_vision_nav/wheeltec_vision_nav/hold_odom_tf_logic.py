"""Gate visual odometry so RTAB-Map always gets a pose at the image stamp."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from wheeltec_vision_nav.wheel_odom_tf_logic import planar_wheel_pose, pose_relative_to

WHEEL_FRESH_S = 0.3

# Variance bounds written to /odom. RTAB-Map inverts the 6x6 covariance to an
# information matrix (Rtabmap.cpp localization cache: odomCovariance.inv()
# without a singularity check), so every diagonal entry must be finite and > 0.
MIN_VARIANCE = 1e-6
MAX_VARIANCE = 1.0
DEFAULT_ANG_VARIANCE = 1e-4


@dataclass(frozen=True)
class OdomSample:
    stamp_sec: int
    stamp_nsec: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    cov0: float
    cov5: float = DEFAULT_ANG_VARIANCE


def odom_sample_is_valid(sample: OdomSample) -> bool:
    norm = math.sqrt(
        sample.qx * sample.qx
        + sample.qy * sample.qy
        + sample.qz * sample.qz
        + sample.qw * sample.qw
    )
    if norm < 0.5:
        return False
    if not math.isfinite(sample.cov0) or sample.cov0 <= 0.0 or sample.cov0 >= 9000.0:
        return False
    return True


def clamp_variance(value: float, default: float) -> float:
    if not math.isfinite(value) or value <= 0.0 or value >= 9000.0:
        return default
    return min(max(value, MIN_VARIANCE), MAX_VARIANCE)


def diagonal_covariance(sample: OdomSample) -> list[float]:
    """Full 6x6 row-major covariance with a strictly positive diagonal."""
    lin = clamp_variance(sample.cov0, MIN_VARIANCE)
    ang = clamp_variance(sample.cov5, DEFAULT_ANG_VARIANCE)
    cov = [0.0] * 36
    cov[0] = lin
    cov[7] = lin
    cov[14] = lin
    cov[21] = ang
    cov[28] = ang
    cov[35] = ang
    return cov


def identity_odom_sample(stamp_sec: int, stamp_nsec: int) -> OdomSample:
    return OdomSample(
        stamp_sec=stamp_sec,
        stamp_nsec=stamp_nsec,
        x=0.0,
        y=0.0,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        cov0=1e-5,
    )


def sample_to_pose(sample: OdomSample) -> tuple[float, float, float, float, float, float, float]:
    return sample.x, sample.y, sample.z, sample.qx, sample.qy, sample.qz, sample.qw


def pose_to_sample(
    stamp_sec: int,
    stamp_nsec: int,
    pose: tuple[float, float, float, float, float, float, float],
    cov0: float = 1e-5,
    cov5: float = DEFAULT_ANG_VARIANCE,
) -> OdomSample:
    x, y, z, qx, qy, qz, qw = pose
    return OdomSample(stamp_sec, stamp_nsec, x, y, z, qx, qy, qz, qw, cov0, cov5)


def planar_sample(sample: OdomSample) -> OdomSample:
    pose = planar_wheel_pose(*sample_to_pose(sample))
    return pose_to_sample(sample.stamp_sec, sample.stamp_nsec, pose, sample.cov0, sample.cov5)


def wheel_relative_sample(
    origin: OdomSample | None, incoming: OdomSample
) -> tuple[OdomSample, OdomSample]:
    """Return (origin, pose in odom). First wheel sample is the odom origin."""
    planar = planar_sample(incoming)
    if origin is None:
        return planar, identity_odom_sample(incoming.stamp_sec, incoming.stamp_nsec)
    rel = pose_relative_to(sample_to_pose(origin), sample_to_pose(planar))
    return origin, pose_to_sample(incoming.stamp_sec, incoming.stamp_nsec, rel, planar.cov0, planar.cov5)


def wheel_source_is_fresh(
    now: float, last_wheel_time: float, timeout_s: float = WHEEL_FRESH_S
) -> bool:
    return last_wheel_time > 0.0 and (now - last_wheel_time) < timeout_s


def gated_odom_sample(last_valid: OdomSample | None, incoming: OdomSample) -> OdomSample:
    """Pass a valid VO pose through. Otherwise reuse the last valid pose at this stamp."""
    if odom_sample_is_valid(incoming):
        return incoming
    if last_valid is not None and odom_sample_is_valid(last_valid):
        return replace(last_valid, stamp_sec=incoming.stamp_sec, stamp_nsec=incoming.stamp_nsec)
    return identity_odom_sample(incoming.stamp_sec, incoming.stamp_nsec)


def yaw_from_sample(sample: OdomSample) -> float:
    return math.atan2(
        2.0 * (sample.qw * sample.qz + sample.qx * sample.qy),
        1.0 - 2.0 * (sample.qy * sample.qy + sample.qz * sample.qz),
    )


def _wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def se2_compose(ax: float, ay: float, ayaw: float, bx: float, by: float, byaw: float) -> tuple[float, float, float]:
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, _wrap_yaw(ayaw + byaw)


def se2_inverse(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return -c * x - s * y, s * x - c * y, _wrap_yaw(-yaw)


def sample_from_se2(sample: OdomSample, x: float, y: float, yaw: float) -> OdomSample:
    return replace(
        sample,
        x=x,
        y=y,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=math.sin(yaw * 0.5),
        qw=math.cos(yaw * 0.5),
    )


@dataclass
class OdomStitch:
    """Keep /odom continuous when visual odometry resets to the origin."""

    ox: float = 0.0
    oy: float = 0.0
    oyaw: float = 0.0
    last_in_x: float = 0.0
    last_in_y: float = 0.0
    last_in_yaw: float = 0.0
    started: bool = False


def stitch_odom_sample(
    state: OdomStitch,
    incoming: OdomSample,
    max_jump_m: float = 1.0,
    max_jump_rad: float = 1.0,
) -> OdomSample:
    yaw = yaw_from_sample(incoming)
    if not state.started:
        state.ox, state.oy, state.oyaw = 0.0, 0.0, 0.0
        state.last_in_x, state.last_in_y, state.last_in_yaw = incoming.x, incoming.y, yaw
        state.started = True
        x, y, out_yaw = se2_compose(state.ox, state.oy, state.oyaw, incoming.x, incoming.y, yaw)
        return sample_from_se2(incoming, x, y, out_yaw)

    jump = math.hypot(incoming.x - state.last_in_x, incoming.y - state.last_in_y)
    dyaw = abs(_wrap_yaw(yaw - state.last_in_yaw))
    if jump > max_jump_m or dyaw > max_jump_rad:
        pub_x, pub_y, pub_yaw = se2_compose(
            state.ox, state.oy, state.oyaw, state.last_in_x, state.last_in_y, state.last_in_yaw
        )
        ix, iy, iyaw = se2_inverse(incoming.x, incoming.y, yaw)
        state.ox, state.oy, state.oyaw = se2_compose(pub_x, pub_y, pub_yaw, ix, iy, iyaw)

    state.last_in_x, state.last_in_y, state.last_in_yaw = incoming.x, incoming.y, yaw
    x, y, out_yaw = se2_compose(state.ox, state.oy, state.oyaw, incoming.x, incoming.y, yaw)
    return sample_from_se2(incoming, x, y, out_yaw)
