import math

from wheeltec_vision_nav.wheel_odom_tf_logic import invert_pose, planar_wheel_pose, pose_relative_to


def test_invert_identity_translation():
    x, y, z, qx, qy, qz, qw = invert_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert (x, y, z) == (-1.0, -2.0, -3.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)


def test_invert_ninety_yaw_moves_x_to_minus_y():
    half = math.sqrt(0.5)
    x, y, z, qx, qy, qz, qw = invert_pose(1.0, 0.0, 0.0, 0.0, 0.0, half, half)
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9
    assert abs(z) < 1e-9
    assert abs(qx) < 1e-9 and abs(qy) < 1e-9
    assert abs(qz + half) < 1e-9
    assert abs(qw - half) < 1e-9


def test_invert_twice_recovers_pose():
    half = math.sqrt(0.5)
    pose = (0.4, -0.2, 0.05, 0.0, 0.0, half, half)
    inv = invert_pose(*pose)
    back = invert_pose(*inv)
    for a, b in zip(pose, back):
        assert abs(a - b) < 1e-9


def test_planar_wheel_pose_zeros_yaw_stored_as_height():
    # WHEELTEC writes Robot_Pos.Z (yaw, rad) into pose.position.z.
    yaw = -0.673328
    half = yaw * 0.5
    x, y, z, qx, qy, qz, qw = planar_wheel_pose(
        0.066001, -0.104610, yaw, 0.0, 0.0, math.sin(half), math.cos(half)
    )
    assert (x, y, z) == (0.066001, -0.104610, 0.0)
    assert abs(qx) < 1e-12 and abs(qy) < 1e-12
    assert abs(math.atan2(2 * qw * qz, 1 - 2 * qz * qz) - yaw) < 1e-9


def test_pose_relative_to_origin_is_identity():
    yaw = 0.4
    half = yaw * 0.5
    origin = planar_wheel_pose(0.3, -0.1, yaw, 0.0, 0.0, math.sin(half), math.cos(half))
    rel = pose_relative_to(origin, origin)
    assert abs(rel[0]) < 1e-9 and abs(rel[1]) < 1e-9 and abs(rel[2]) < 1e-9
    assert abs(rel[3]) < 1e-9 and abs(rel[4]) < 1e-9 and abs(rel[5]) < 1e-9
    assert abs(rel[6] - 1.0) < 1e-9
