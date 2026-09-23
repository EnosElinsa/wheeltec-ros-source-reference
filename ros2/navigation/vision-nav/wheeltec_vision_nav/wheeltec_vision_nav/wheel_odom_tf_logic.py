"""Invert a pose so wheel odometry can hang under base_footprint in TF."""

from __future__ import annotations

import math

Pose = tuple[float, float, float, float, float, float, float]


def _quat_conj(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    return (-qx, -qy, -qz, qw)


def _quat_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(
    q: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    vx, vy, vz = vec
    rotated = _quat_mul(_quat_mul(q, (vx, vy, vz, 0.0)), _quat_conj(*q))
    return rotated[0], rotated[1], rotated[2]


def invert_pose(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Pose:
    """Return the inverse of pose (translation + quaternion).

    Wheel odometry is the pose of ``base_footprint`` in ``wheel_odom``. Publishing
    the inverse as ``base_footprint`` → ``wheel_odom`` keeps a single TF parent
    for the robot frame while ``lookup(wheel_odom, base_footprint)`` still yields
    that wheel pose for RTAB-Map's ``guess_frame_id``.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    iqx, iqy, iqz, iqw = _quat_conj(qx, qy, qz, qw)
    rx, ry, rz = _rotate((iqx, iqy, iqz, iqw), (x, y, z))
    return -rx, -ry, -rz, iqx, iqy, iqz, iqw


def planar_wheel_pose(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Pose:
    """Keep x, y, yaw. WHEELTEC stores yaw (rad) in ``position.z``; that is not height."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        yaw = z
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    half = yaw * 0.5
    return x, y, 0.0, 0.0, 0.0, math.sin(half), math.cos(half)


def pose_multiply(a: Pose, b: Pose) -> Pose:
    ax, ay, az, aqx, aqy, aqz, aqw = a
    bx, by, bz, bqx, bqy, bqz, bqw = b
    rx, ry, rz = _rotate((aqx, aqy, aqz, aqw), (bx, by, bz))
    qx, qy, qz, qw = _quat_mul((aqx, aqy, aqz, aqw), (bqx, bqy, bqz, bqw))
    return ax + rx, ay + ry, az + rz, qx, qy, qz, qw


def pose_relative_to(origin: Pose, pose: Pose) -> Pose:
    return pose_multiply(invert_pose(*origin), pose)
